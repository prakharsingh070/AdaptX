"""End-to-end adaptive resolution: raw frames through the whole chain.

The point of these tests is that the **real** Phase 2-7 outputs reach the
Phase 8 controller: processed frame, detections, tracks, trajectories, map and
risk assessments, with no parallel path. A controller fed hand-made
assessments would pass its unit tests and still be wrong about what the
pipeline actually produces.

Also covers the HTTP contract for ``POST /api/v1/lidar/adaptive-map`` and
``POST /api/v1/map/adaptive/reset``, the adaptive telemetry summary, and the
temporal behaviour that unit tests cannot reach: a scene evolving frame by
frame through the real tracker and predictor.
"""

from __future__ import annotations

from collections.abc import Iterator
from datetime import UTC, datetime, timedelta
from typing import Any

import numpy as np
import pytest
from fastapi.testclient import TestClient

from adaptx.api.app import create_app
from adaptx.config.settings import (
    AdaptiveResolutionSettings,
    LiDARSettings,
    MapSettings,
    Settings,
)
from adaptx.core.lifecycle import ApplicationContext, build_context
from adaptx.models.common import DataSource
from adaptx.models.map import ResolutionLevel
from adaptx.models.point_cloud import RawPointCloudFrame
from adaptx.models.resolution import ResolutionSource
from tests.fixtures import scenes

EPOCH = datetime(2026, 1, 1, 12, 0, 0, tzinfo=UTC)

PIPELINE_SETTINGS = LiDARSettings(
    min_points=0,
    max_points=200_000,
    min_range_m=0.0,
    max_range_m=200.0,
    roi_x_min_m=-100.0,
    roi_x_max_m=100.0,
    roi_y_min_m=-100.0,
    roi_y_max_m=100.0,
    roi_z_min_m=-50.0,
    roi_z_max_m=50.0,
    voxel_enabled=True,
    voxel_size_m=0.1,
    ground_enabled=True,
    ground_cell_size_m=1.0,
    noise_enabled=False,
)

MAP_SETTINGS = MapSettings(
    min_x_m=-40.0, max_x_m=40.0, min_y_m=-40.0, max_y_m=40.0, resolution_m=0.5
)

ADAPTIVE_SETTINGS = AdaptiveResolutionSettings(tile_size_m=10.0)


def road_scene(vehicle_x: float = 12.0) -> np.ndarray:
    """A road plane with a vehicle and a pedestrian on it."""
    return np.vstack(
        [
            scenes.ground_plane(extent_m=30.0),
            scenes.vehicle((vehicle_x, -3.0, scenes.GROUND_Z_M + 0.8)),
            scenes.pedestrian((8.0, 4.0, scenes.GROUND_Z_M + 0.9)),
        ]
    )


def open_scene() -> np.ndarray:
    """Open ground and nothing else: no object for the detector to find."""
    return scenes.ground_plane(extent_m=30.0)


def raw(points: np.ndarray, frame_id: int = 0, seconds: float = 0.0) -> RawPointCloudFrame:
    return RawPointCloudFrame.from_sequence(
        points.tolist(),
        frame_id=frame_id,
        sensor_id="roof_lidar",
        source=DataSource.SYNTHETIC_TEST,
        timestamp=EPOCH + timedelta(seconds=seconds),
    )


def settings() -> Settings:
    return Settings(
        app={"environment": "development", "debug": True},
        api={"cors_origins": []},
        logging={"level": "WARNING"},
        carla={"enabled": False, "use_mock": False},
        lidar=PIPELINE_SETTINGS.model_dump(),
        map=MAP_SETTINGS.model_dump(),
        adaptive=ADAPTIVE_SETTINGS.model_dump(),
        websocket={"telemetry_interval_s": 0.05, "max_connections": 4},
    )


@pytest.fixture
def context() -> ApplicationContext:
    """A context whose pipeline and map bounds suit a real scene."""
    return build_context(settings())


@pytest.fixture
def adaptive_client() -> Iterator[TestClient]:
    resolved = settings()
    built = build_context(resolved)
    with TestClient(create_app(settings=resolved, context=built)) as client:
        yield client


def frame_body(points: np.ndarray, **extra: Any) -> dict[str, Any]:
    body: dict[str, Any] = {
        "frame_id": 0,
        "sensor_id": "roof_lidar",
        "source": "synthetic_test",
        "points": points.tolist(),
        "timestamp": EPOCH.isoformat(),
    }
    body.update(extra)
    return body


def run_frame(
    context: ApplicationContext, points: np.ndarray, *, frame_id: int = 0, seconds: float = 0.0
) -> Any:
    """Drive one frame through the real chain and return the adaptive map."""
    processed = context.preprocessor.run(raw(points, frame_id=frame_id, seconds=seconds))
    detection = context.detector.detect(processed.frame)
    tracking = context.tracking.update(
        detection.objects,
        processed.frame.timestamp,
        frame_id=frame_id,
        sensor_id="roof_lidar",
    )
    prediction = context.prediction.predict_from_tracking(tracking)
    spatial_map = context.mapping.build(processed.frame)
    risk = context.risk.assess_from_pipeline(
        tracking, prediction=prediction, spatial_map=spatial_map
    )
    return context.adaptive_mapping.run_from_pipeline(
        processed.frame, risk, tracking, trajectories=prediction.trajectories
    )


class TestTheRealChainReachesTheController:
    """Phase 2-7 output must feed Phase 8 directly, with no second path."""

    def test_a_real_scene_produces_a_plan_covering_every_region(
        self, context: ApplicationContext
    ) -> None:
        result = run_frame(context, road_scene())

        assert result.tile_count == 64  # 80 m square in 10 m regions
        assert result.plan.tile_count == result.tile_count
        assert result.is_adaptive is True

    def test_the_controller_consumes_real_risk_assessments(
        self, context: ApplicationContext
    ) -> None:
        result = run_frame(context, road_scene())
        influencing = [d for d in result.plan.decisions if d.influencing_track_ids]

        assert result.plan.considered_assessment_count > 0
        assert influencing

    def test_the_mapper_receives_the_processed_frame_not_the_raw_one(
        self, context: ApplicationContext
    ) -> None:
        """Ground points are separated upstream, so they never reach the map."""
        points = road_scene()
        result = run_frame(context, points)

        assert result.accounting.input_point_count < len(points)
        assert result.accounting.input_point_count > 0

    def test_every_point_is_accounted_for_through_the_real_chain(
        self, context: ApplicationContext
    ) -> None:
        result = run_frame(context, road_scene())
        accounting = result.accounting

        assert (
            accounting.input_point_count
            == accounting.mapped_point_count + accounting.out_of_bounds_point_count
        )
        assert accounting.mapped_point_count == sum(t.mapped_point_count for t in result.tiles)


class TestRequiredScenarios:
    """The three scenes Phase 8 exists to distinguish."""

    def test_a_low_complexity_scene_stays_mostly_coarse(self, context: ApplicationContext) -> None:
        """Open ground with no objects: spend nothing where nothing is."""
        result = run_frame(context, open_scene())
        tiles = result.tiles_by_level()

        assert tiles["low"] == result.tile_count
        assert result.finest_resolution_m == result.coarsest_resolution_m

    def test_a_scene_with_an_object_refines_only_part_of_the_map(
        self, context: ApplicationContext
    ) -> None:
        result = run_frame(context, road_scene())
        tiles = result.tiles_by_level()

        assert tiles["low"] > 0, "some regions must stay coarse"
        assert sum(count for level, count in tiles.items() if level != "low") > 0

    def test_a_mixed_scene_differentiates_spatially(self, context: ApplicationContext) -> None:
        """One map, several resolutions, and the fine ones near the objects."""
        result = run_frame(context, road_scene())
        sizes = {tile.resolution_m for tile in result.tiles}

        assert len(sizes) > 1

        finest = min(result.tiles, key=lambda tile: tile.resolution_m)
        # The vehicle sits at x=12, y=-3; the finest region must be near it,
        # not in an empty corner of the map.
        assert finest.bounds.contains(12.0, -3.0) or finest.bounds.min_x >= 0.0

    def test_an_object_scene_costs_more_cells_than_an_empty_one(
        self, context: ApplicationContext
    ) -> None:
        """Adaptive means the bill tracks the scene, which is the whole claim."""
        empty = run_frame(context, open_scene(), frame_id=0, seconds=0.0)
        context.adaptive_mapping.reset()
        context.tracking.reset()
        busy = run_frame(context, road_scene(), frame_id=1, seconds=0.5)

        assert busy.accounting.total_cell_count > empty.accounting.total_cell_count


class TestFixedVersusAdaptive:
    def test_both_variants_see_identical_input(self, context: ApplicationContext) -> None:
        processed = context.preprocessor.run(raw(road_scene()))
        detection = context.detector.detect(processed.frame)
        tracking = context.tracking.update(
            detection.objects, processed.frame.timestamp, frame_id=0, sensor_id="roof_lidar"
        )
        prediction = context.prediction.predict_from_tracking(tracking)
        spatial_map = context.mapping.build(processed.frame)
        risk = context.risk.assess_from_pipeline(
            tracking, prediction=prediction, spatial_map=spatial_map
        )
        adaptive = context.adaptive_mapping.run_from_pipeline(
            processed.frame, risk, tracking, trajectories=prediction.trajectories
        )
        comparison = context.adaptive_mapping.compare_with_fixed(adaptive, spatial_map)

        assert comparison.fixed.mapped_point_count == comparison.adaptive.mapped_point_count
        assert comparison.fixed.is_adaptive is False
        assert comparison.adaptive.is_adaptive is True

    def test_the_baseline_mapper_is_untouched_by_the_adaptive_one(
        self, context: ApplicationContext
    ) -> None:
        """Phase 6 must stay measurable as it was (ADR-003)."""
        processed = context.preprocessor.run(raw(road_scene()))
        spatial_map = context.mapping.build(processed.frame)

        assert spatial_map.is_adaptive is False
        assert spatial_map.resolution.source is ResolutionSource.FIXED
        assert spatial_map.mapper == "fixed_resolution_mapper_v1"
        assert spatial_map.width == 160 and spatial_map.height == 160


class TestTemporalBehaviour:
    """What unit tests cannot reach: a scene evolving through the real tracker."""

    def test_resolution_does_not_oscillate_across_a_stable_sequence(
        self, context: ApplicationContext
    ) -> None:
        """A stationary scene must not make the map flicker."""
        distributions = []
        for index in range(6):
            result = run_frame(context, road_scene(), frame_id=index, seconds=0.5 * index)
            distributions.append(tuple(sorted(result.tiles_by_level().items())))

        # Allow the opening frames to settle as tracks confirm, then require
        # the distribution to hold.
        assert len(set(distributions[3:])) == 1

    def test_an_approaching_object_does_not_lower_detail(self, context: ApplicationContext) -> None:
        finest = []
        for index in range(5):
            result = run_frame(
                context,
                road_scene(vehicle_x=20.0 - 3.0 * index),
                frame_id=index,
                seconds=0.5 * index,
            )
            finest.append(result.finest_resolution_m)

        assert finest[-1] <= finest[0]

    def test_a_scene_emptying_returns_the_map_towards_coarse(
        self, context: ApplicationContext
    ) -> None:
        """Stale detail must not persist once the objects are gone."""
        for index in range(3):
            busy = run_frame(context, road_scene(), frame_id=index, seconds=0.5 * index)

        for index in range(3, 12):
            quiet = run_frame(context, open_scene(), frame_id=index, seconds=0.5 * index)

        assert quiet.accounting.total_cell_count <= busy.accounting.total_cell_count
        assert quiet.tiles_by_level()["low"] >= busy.tiles_by_level()["low"]

    def test_a_reset_clears_the_stabilisation_history(self, context: ApplicationContext) -> None:
        run_frame(context, road_scene())
        context.adaptive_mapping.reset()
        controller = context.adaptive_mapping.controller

        assert controller.frame_index == 0
        assert all(
            controller.current_level(index) is None
            for index in range(controller.tile_grid().tile_count)
        )

    def test_the_first_frame_after_a_reset_reports_no_change(
        self, context: ApplicationContext
    ) -> None:
        run_frame(context, road_scene())
        context.adaptive_mapping.reset()
        context.tracking.reset()
        result = run_frame(context, road_scene(), frame_id=1, seconds=0.5)

        assert result.plan.changed_tile_count == 0


class TestDeterminism:
    def test_the_same_sequence_produces_the_same_decisions(self) -> None:
        def run() -> list[tuple[int, str]]:
            built = build_context(settings())
            for index in range(3):
                result = run_frame(built, road_scene(), frame_id=index, seconds=0.5 * index)
            return [(d.tile_index, d.level.value) for d in result.plan.decisions]

        assert run() == run()

    def test_the_same_sequence_produces_the_same_cell_counts(self) -> None:
        def run() -> int:
            built = build_context(settings())
            for index in range(3):
                result = run_frame(built, road_scene(), frame_id=index, seconds=0.5 * index)
            return result.accounting.total_cell_count

        assert run() == run()


class TestAdaptiveMapEndpoint:
    def test_a_valid_frame_returns_an_adaptive_map_summary(
        self, adaptive_client: TestClient
    ) -> None:
        response = adaptive_client.post("/api/v1/lidar/adaptive-map", json=frame_body(road_scene()))

        assert response.status_code == 200
        body = response.json()
        assert body["accepted"] is True
        assert body["map"]["is_adaptive"] is True
        assert body["map"]["tile_count"] == 64
        assert body["map"]["accounting"]["occupied_cell_count"] > 0

    def test_the_dense_grid_is_never_returned(self, adaptive_client: TestClient) -> None:
        body = adaptive_client.post(
            "/api/v1/lidar/adaptive-map", json=frame_body(road_scene())
        ).json()

        assert "tiles" not in body["map"]
        assert "point_count" not in body["map"]
        assert body["cells"] is None

    def test_region_decisions_are_returned_and_carry_their_reasons(
        self, adaptive_client: TestClient
    ) -> None:
        body = adaptive_client.post(
            "/api/v1/lidar/adaptive-map", json=frame_body(road_scene())
        ).json()
        decisions = body["decisions"]

        assert decisions is not None
        assert all(d["resolution"]["source"] == "adaptive" for d in decisions)
        assert all(d["reason"] for d in decisions)

    def test_decisions_can_be_omitted(self, adaptive_client: TestClient) -> None:
        body = adaptive_client.post(
            "/api/v1/lidar/adaptive-map",
            json=frame_body(road_scene(), include_decisions=False),
        ).json()

        assert body["decisions"] is None

    def test_decision_truncation_is_reported_never_silent(
        self, adaptive_client: TestClient
    ) -> None:
        body = adaptive_client.post(
            "/api/v1/lidar/adaptive-map",
            json=frame_body(road_scene(), max_decisions=4),
        ).json()

        assert len(body["decisions"]) == 4
        assert body["decisions_truncated"] is True

    def test_cells_are_available_on_request_with_their_own_resolutions(
        self, adaptive_client: TestClient
    ) -> None:
        body = adaptive_client.post(
            "/api/v1/lidar/adaptive-map",
            json=frame_body(road_scene(), include_cells=True, max_cells=5000),
        ).json()
        cells = body["cells"]["cells"]

        assert cells
        assert all(cell["occupancy_state"] == "occupied" for cell in cells)
        assert all(cell["resolution_m"] > 0 for cell in cells)

    def test_the_fixed_comparison_is_available_on_request(
        self, adaptive_client: TestClient
    ) -> None:
        body = adaptive_client.post(
            "/api/v1/lidar/adaptive-map",
            json=frame_body(road_scene(), include_fixed_comparison=True),
        ).json()
        comparison = body["comparison"]

        assert comparison is not None
        assert comparison["fixed"]["is_adaptive"] is False
        assert comparison["adaptive"]["is_adaptive"] is True
        assert comparison["fixed"]["uniform_resolution_m"] == 0.5
        assert comparison["adaptive"]["uniform_resolution_m"] is None

    def test_the_comparison_is_absent_unless_asked_for(self, adaptive_client: TestClient) -> None:
        body = adaptive_client.post(
            "/api/v1/lidar/adaptive-map", json=frame_body(road_scene())
        ).json()

        assert body["comparison"] is None

    def test_the_plan_summary_accounts_for_every_assessment(
        self, adaptive_client: TestClient
    ) -> None:
        summary = adaptive_client.post(
            "/api/v1/lidar/adaptive-map", json=frame_body(road_scene())
        ).json()["plan_summary"]

        assert (
            summary["considered_assessments"]
            == summary["influencing_assessments"] + summary["excluded_assessments"]
        )
        assert summary["budget"]["within_budget"] is True

    def test_the_response_never_claims_a_probability(self, adaptive_client: TestClient) -> None:
        body = adaptive_client.post(
            "/api/v1/lidar/adaptive-map", json=frame_body(road_scene())
        ).json()

        assert "NOT a probability of collision" in body["detail"]
        for decision in body["decisions"]:
            assert "probability" not in decision["reason"].lower()

    def test_a_malformed_frame_is_rejected(self, adaptive_client: TestClient) -> None:
        """The same input contract as every other LiDAR endpoint.

        A wrongly shaped row rather than a NaN: NaN is not JSON-encodable, so
        it cannot reach the endpoint through a real request at all.
        """
        response = adaptive_client.post(
            "/api/v1/lidar/adaptive-map",
            json={
                "frame_id": 0,
                "sensor_id": "roof_lidar",
                "source": "synthetic_test",
                "points": [[1.0, 2.0]],
                "timestamp": EPOCH.isoformat(),
            },
        )

        assert response.status_code == 422

    def test_it_is_documented_in_the_openapi_schema(self, adaptive_client: TestClient) -> None:
        paths = adaptive_client.get("/openapi.json").json()["paths"]

        assert "/api/v1/lidar/adaptive-map" in paths
        assert "post" in paths["/api/v1/lidar/adaptive-map"]

    def test_the_existing_map_endpoint_is_unchanged(self, adaptive_client: TestClient) -> None:
        """Phase 8 is additive: the Phase 6 contract must still hold."""
        body = adaptive_client.post("/api/v1/lidar/map", json=frame_body(road_scene())).json()

        assert body["map"]["is_adaptive"] is False
        assert body["map"]["resolution"]["source"] == "fixed"
        assert body["map"]["width"] == 160


class TestAdaptiveResetEndpoint:
    def test_it_clears_the_remembered_levels(self, adaptive_client: TestClient) -> None:
        adaptive_client.post("/api/v1/lidar/adaptive-map", json=frame_body(road_scene()))
        response = adaptive_client.post("/api/v1/map/adaptive/reset")

        assert response.status_code == 200
        body = response.json()
        assert body["reset"] is True
        assert body["cleared_region_count"] == 64

    def test_it_reports_nothing_cleared_before_the_first_frame(
        self, adaptive_client: TestClient
    ) -> None:
        body = adaptive_client.post("/api/v1/map/adaptive/reset").json()

        assert body["cleared_region_count"] == 0

    def test_it_is_documented_in_the_openapi_schema(self, adaptive_client: TestClient) -> None:
        paths = adaptive_client.get("/openapi.json").json()["paths"]

        assert "/api/v1/map/adaptive/reset" in paths


class TestMapStatusReportsBoth:
    def test_it_names_the_controller_and_the_adaptive_mapper(
        self, adaptive_client: TestClient
    ) -> None:
        body = adaptive_client.get("/api/v1/map/status").json()

        assert body["adaptive_resolution_implemented"] is True
        assert body["adaptive_controller"] == "heuristic_resolution_controller_v1"
        assert body["adaptive_mapper"] == "tiled_adaptive_mapper_v1"
        assert body["adaptive_enabled"] is True
        assert body["tile_size_m"] == 10.0
        assert body["tile_count"] == 64

    def test_the_adaptive_summary_reflects_a_mapped_frame(
        self, adaptive_client: TestClient
    ) -> None:
        adaptive_client.post("/api/v1/lidar/adaptive-map", json=frame_body(road_scene()))
        summary = adaptive_client.get("/api/v1/map/status").json()["adaptive_summary"]

        assert summary["frames_mapped"] == 1
        assert summary["total_cells"] > 0
        assert sum(summary["tiles_by_level"].values()) == 64

    def test_the_component_is_ready_but_never_implemented(
        self, adaptive_client: TestClient
    ) -> None:
        components = {
            c["name"]: c for c in adaptive_client.get("/api/v1/system/status").json()["components"]
        }
        adaptive = components["adaptive_resolution"]

        assert adaptive["readiness"] == "READY"
        assert adaptive["implementation"] != "IMPLEMENTED"
        assert adaptive["phase"] == 8


class TestAdaptiveTelemetry:
    def test_the_summary_carries_counts_not_the_tiles(self, adaptive_client: TestClient) -> None:
        with adaptive_client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            adaptive = websocket.receive_json()["data"]["adaptive_mapping"]

        assert adaptive["mapper"] == "tiled_adaptive_mapper_v1"
        assert adaptive["is_adaptive"] is True
        for absent in ("tiles", "cells", "point_count", "decisions"):
            assert absent not in adaptive

    def test_it_states_what_the_priority_is_not(self, adaptive_client: TestClient) -> None:
        with adaptive_client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            adaptive = websocket.receive_json()["data"]["adaptive_mapping"]

        assert adaptive["priority_is_heuristic"] is True
        assert adaptive["is_collision_probability"] is False
        assert adaptive["is_baseline"] is True

    def test_the_payload_stays_bounded_after_a_frame(self, adaptive_client: TestClient) -> None:
        """A telemetry tick must not grow with the size of the map."""
        adaptive_client.post("/api/v1/lidar/adaptive-map", json=frame_body(road_scene()))
        with adaptive_client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            payload = websocket.receive_json()

        import json

        assert len(json.dumps(payload)) < 32_000

    def test_the_adaptive_stream_is_advertised(self, adaptive_client: TestClient) -> None:
        with adaptive_client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            data = websocket.receive_json()["data"]

        assert "adaptive_mapping" in data["provides"]
        assert "adaptive_map" not in data["not_yet_available"]
        assert "risk_field" in data["not_yet_available"]


class TestConfiguration:
    def test_disabling_adaptation_holds_the_base_level_everywhere(self) -> None:
        resolved = settings()
        resolved.adaptive.enabled = False
        built = build_context(resolved)
        result = run_frame(built, road_scene())

        assert {tile.level for tile in result.tiles} == {ResolutionLevel.LOW}

    def test_the_tile_size_is_configurable(self) -> None:
        resolved = settings()
        resolved.adaptive.tile_size_m = 20.0
        built = build_context(resolved)
        result = run_frame(built, road_scene())

        assert result.tile_size_m == 20.0
        assert result.tile_count == 16
