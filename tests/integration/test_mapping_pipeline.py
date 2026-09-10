"""End-to-end mapping: processed LiDAR frames through the Phase 6 mapper.

The point of these tests is that the **existing** Phase 2 processed frame
reaches the mapper without a parallel preprocessing path. A mapper that quietly
re-filtered its input would pass unit tests and still be wrong.

Also covers the HTTP contract for ``POST /api/v1/lidar/map`` and
``GET /api/v1/map/status``, and the mapping telemetry summary.
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
    DetectionSettings,
    LiDARSettings,
    MapSettings,
    PredictionSettings,
    Settings,
    TrackingSettings,
)
from adaptx.core.lifecycle import build_context
from adaptx.mapping.grid_mapper import FixedResolutionMapper
from adaptx.models.common import DataSource
from adaptx.models.point_cloud import RawPointCloudFrame
from adaptx.models.resolution import ResolutionDecision, ResolutionSource
from adaptx.perception.detector import GeometricObjectDetector
from adaptx.perception.pipeline import LiDARProcessingPipeline
from adaptx.prediction.constant_velocity import ConstantVelocityPredictor
from adaptx.services.mapping_service import MappingService
from adaptx.tracking.tracker import GeometricObjectTracker
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


def road_scene() -> np.ndarray:
    """A road plane with a vehicle and a pedestrian on it."""
    return np.vstack(
        [
            scenes.ground_plane(extent_m=30.0),
            scenes.vehicle((12.0, -3.0, scenes.GROUND_Z_M + 0.8)),
            scenes.pedestrian((8.0, 4.0, scenes.GROUND_Z_M + 0.9)),
        ]
    )


def raw(points: np.ndarray, frame_id: int = 0, seconds: float = 0.0) -> RawPointCloudFrame:
    return RawPointCloudFrame.from_sequence(
        points.tolist(),
        frame_id=frame_id,
        sensor_id="roof_lidar",
        source=DataSource.SYNTHETIC_TEST,
        timestamp=EPOCH + timedelta(seconds=seconds),
    )


class TestProcessedFrameReachesTheMapper:
    """Phase 2 output must feed Phase 6 directly, with no second filtering path."""

    def test_a_processed_frame_maps_without_further_preprocessing(self) -> None:
        pipeline = LiDARProcessingPipeline(PIPELINE_SETTINGS)
        mapper = FixedResolutionMapper(MAP_SETTINGS)

        processed = pipeline.run(raw(road_scene()))
        result = mapper.build(processed.frame)

        assert result.accounting.input_point_count == processed.frame.point_count
        assert result.occupied_cell_count > 0

    def test_the_mapper_consumes_exactly_what_the_pipeline_produced(self) -> None:
        """Every processed point is mapped or counted out of bounds - none vanish."""
        pipeline = LiDARProcessingPipeline(PIPELINE_SETTINGS)
        mapper = FixedResolutionMapper(MAP_SETTINGS)

        processed = pipeline.run(raw(road_scene()))
        accounting = mapper.build(processed.frame).accounting

        assert accounting.input_point_count == processed.frame.point_count
        assert accounting.input_point_count == (
            accounting.mapped_point_count + accounting.out_of_bounds_point_count
        )

    def test_the_non_ground_frame_maps_to_fewer_cells_than_the_full_one(self) -> None:
        """Ground separation is upstream, and the map reflects whichever it is given."""
        pipeline = LiDARProcessingPipeline(PIPELINE_SETTINGS)
        mapper = FixedResolutionMapper(MAP_SETTINGS)
        processed = pipeline.run(raw(road_scene()))
        assert processed.ground_frame is not None

        objects_only = mapper.build(processed.frame)
        ground_only = mapper.build(processed.ground_frame)

        assert objects_only.occupied_cell_count < ground_only.occupied_cell_count

    def test_measured_heights_reflect_the_scene(self) -> None:
        """A vehicle roof sits above the road, and the map must show that."""
        pipeline = LiDARProcessingPipeline(PIPELINE_SETTINGS)
        mapper = FixedResolutionMapper(MAP_SETTINGS)
        processed = pipeline.run(raw(road_scene()))

        result = mapper.build(processed.frame)
        observed = result.max_height_m[~np.isnan(result.max_height_m)]

        assert observed.size > 0
        assert observed.max() > scenes.GROUND_Z_M

    def test_consecutive_frames_are_independent(self) -> None:
        pipeline = LiDARProcessingPipeline(PIPELINE_SETTINGS)
        mapper = FixedResolutionMapper(MAP_SETTINGS)

        busy = mapper.build(pipeline.run(raw(road_scene(), 0, 0.0)).frame)
        quiet = mapper.build(pipeline.run(raw(scenes.ground_plane(extent_m=5.0), 1, 0.1)).frame)

        assert quiet.occupied_cell_count < busy.occupied_cell_count

    def test_the_whole_chain_reaches_mapping(self) -> None:
        """process -> detect -> track -> predict -> map, on one frame.

        Mapping does not consume detection, tracking or prediction; this only
        proves the stages coexist and that mapping stays independently usable.
        """
        pipeline = LiDARProcessingPipeline(PIPELINE_SETTINGS)
        detector = GeometricObjectDetector(DetectionSettings())
        tracker = GeometricObjectTracker(TrackingSettings())
        predictor = ConstantVelocityPredictor(PredictionSettings())
        mapper = FixedResolutionMapper(MAP_SETTINGS)

        processed = pipeline.run(raw(road_scene()))
        detection = detector.detect(processed.frame)
        tracking = tracker.update(detection.objects, processed.frame.timestamp)
        prediction = predictor.predict(tracking.tracks, tracking.timestamp)
        result = mapper.build(processed.frame)

        assert detection.object_count >= 1
        assert tracking.active_track_count >= 1
        assert prediction.considered_track_count >= 1
        assert result.occupied_cell_count > 0

    def test_provenance_survives_the_chain(self) -> None:
        pipeline = LiDARProcessingPipeline(PIPELINE_SETTINGS)
        mapper = FixedResolutionMapper(MAP_SETTINGS)

        result = mapper.build(pipeline.run(raw(road_scene())).frame)

        assert result.source is DataSource.SYNTHETIC_TEST
        assert result.sensor_id == "roof_lidar"
        assert result.is_adaptive is False


class TestMappingService:
    def test_the_service_orchestrates_without_holding_map_state(self) -> None:
        service = MappingService(MAP_SETTINGS)
        pipeline = LiDARProcessingPipeline(PIPELINE_SETTINGS)
        processed = pipeline.run(raw(road_scene()))

        first = service.build(processed.frame)
        second = service.build(processed.frame)

        assert np.array_equal(first.point_count, second.point_count)
        assert service.frames_mapped == 2

    def test_the_service_keeps_only_a_summary_not_the_grid(self) -> None:
        service = MappingService(MAP_SETTINGS)
        service.build(LiDARProcessingPipeline(PIPELINE_SETTINGS).run(raw(road_scene())).frame)
        last = service.last_summary

        assert last is not None
        assert not hasattr(last, "point_count")
        assert last.accounting.occupied_cell_count > 0

    def test_reset_clears_counters_but_changes_no_output(self) -> None:
        service = MappingService(MAP_SETTINGS)
        processed = LiDARProcessingPipeline(PIPELINE_SETTINGS).run(raw(road_scene()))

        before = service.build(processed.frame)
        service.reset()
        assert service.frames_mapped == 0
        assert service.last_summary is None

        after = service.build(processed.frame)
        assert np.array_equal(before.point_count, after.point_count)

    def test_the_summary_reports_no_frame_before_the_first(self) -> None:
        summary = MappingService(MAP_SETTINGS).summary()

        assert summary["frames_mapped"] == 0
        assert "no frame has been mapped" in str(summary["note"])

    def test_the_service_applies_a_supplied_resolution(self) -> None:
        service = MappingService(MAP_SETTINGS)
        processed = LiDARProcessingPipeline(PIPELINE_SETTINGS).run(raw(road_scene()))

        result = service.build(
            processed.frame,
            ResolutionDecision.override(1.0, reason="test", requested_by="test"),
        )

        assert result.resolution_m == 1.0
        assert result.resolution.source is ResolutionSource.OVERRIDE


@pytest.fixture
def map_client() -> Iterator[TestClient]:
    """A client whose pipeline and map bounds suit a real scene."""
    settings = Settings(
        app={"environment": "development", "debug": True},
        api={"cors_origins": []},
        logging={"level": "WARNING"},
        carla={"enabled": False, "use_mock": False},
        lidar=PIPELINE_SETTINGS.model_dump(),
        map=MAP_SETTINGS.model_dump(),
        websocket={"telemetry_interval_s": 0.05, "max_connections": 4},
    )
    context = build_context(settings)
    with TestClient(create_app(settings=settings, context=context)) as client:
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


class TestMapEndpoint:
    def test_a_valid_frame_returns_a_map_summary(self, map_client: TestClient) -> None:
        response = map_client.post("/api/v1/lidar/map", json=frame_body(road_scene()))

        assert response.status_code == 200
        body = response.json()
        assert body["accepted"] is True
        assert body["map"]["width"] == 160
        assert body["map"]["height"] == 160
        assert body["map"]["accounting"]["occupied_cell_count"] > 0

    def test_the_response_carries_no_dense_grid(self, map_client: TestClient) -> None:
        body = map_client.post("/api/v1/lidar/map", json=frame_body(road_scene())).json()

        assert "point_count" not in body["map"]
        assert "min_height_m" not in body["map"]
        assert body["cells"] is None

    def test_cells_are_returned_only_when_asked_for(self, map_client: TestClient) -> None:
        body = map_client.post(
            "/api/v1/lidar/map", json=frame_body(road_scene(), include_cells=True)
        ).json()

        assert body["cells"] is not None
        assert len(body["cells"]["cells"]) > 0
        assert all(cell["point_count"] > 0 for cell in body["cells"]["cells"])

    def test_truncation_is_reported_not_silent(self, map_client: TestClient) -> None:
        body = map_client.post(
            "/api/v1/lidar/map",
            json=frame_body(road_scene(), include_cells=True, max_cells=5),
        ).json()

        assert body["cells_truncated"] is True
        assert len(body["cells"]["cells"]) == 5

    def test_accounting_holds_over_the_api(self, map_client: TestClient) -> None:
        accounting = map_client.post("/api/v1/lidar/map", json=frame_body(road_scene())).json()[
            "map"
        ]["accounting"]

        assert accounting["input_point_count"] == (
            accounting["mapped_point_count"] + accounting["out_of_bounds_point_count"]
        )

    def test_a_resolution_override_is_applied_and_labelled(self, map_client: TestClient) -> None:
        body = map_client.post(
            "/api/v1/lidar/map", json=frame_body(road_scene(), resolution_m=1.0)
        ).json()

        assert body["map"]["resolution"]["resolution_m"] == 1.0
        assert body["map"]["resolution"]["source"] == "override"
        assert body["map"]["width"] == 80

    def test_an_out_of_range_resolution_is_rejected(self, map_client: TestClient) -> None:
        response = map_client.post(
            "/api/v1/lidar/map", json=frame_body(road_scene(), resolution_m=99.0)
        )

        assert response.status_code == 422

    def test_an_empty_frame_returns_a_valid_empty_map(self, map_client: TestClient) -> None:
        response = map_client.post(
            "/api/v1/lidar/map",
            json={
                "frame_id": 0,
                "sensor_id": "roof_lidar",
                "source": "synthetic_test",
                "points": [],
            },
        )

        assert response.status_code == 200
        body = response.json()["map"]
        assert body["accounting"]["occupied_cell_count"] == 0
        assert body["accounting"]["input_point_count"] == 0
        assert body["width"] == 160

    @pytest.mark.parametrize(
        "points", [[[0.0, 0.0]], [[0.0, 0.0, 0.0], [1.0, 1.0]], [["a", "b", "c"]]]
    )
    def test_malformed_points_are_rejected_with_422(
        self, map_client: TestClient, points: list[Any]
    ) -> None:
        response = map_client.post(
            "/api/v1/lidar/map",
            json={
                "frame_id": 0,
                "sensor_id": "roof_lidar",
                "source": "synthetic_test",
                "points": points,
            },
        )

        assert response.status_code == 422

    def test_the_response_is_labelled_a_fixed_resolution_baseline(
        self, map_client: TestClient
    ) -> None:
        body = map_client.post("/api/v1/lidar/map", json=frame_body(road_scene())).json()

        assert body["map"]["is_adaptive"] is False
        assert "adaptive resolution is not implemented" in body["detail"]
        assert "not a persistent world map" in body["detail"]

    def test_consecutive_requests_are_independent(self, map_client: TestClient) -> None:
        """Frame-local over HTTP: a second request must not inherit the first."""
        busy = map_client.post("/api/v1/lidar/map", json=frame_body(road_scene())).json()
        quiet = map_client.post(
            "/api/v1/lidar/map", json=frame_body(scenes.ground_plane(extent_m=5.0))
        ).json()

        assert (
            quiet["map"]["accounting"]["occupied_cell_count"]
            < busy["map"]["accounting"]["occupied_cell_count"]
        )

    def test_it_is_documented_in_the_openapi_schema(self, map_client: TestClient) -> None:
        paths = map_client.get("/openapi.json").json()["paths"]

        assert "/api/v1/lidar/map" in paths
        assert "post" in paths["/api/v1/lidar/map"]


class TestMapStatusEndpoint:
    def test_it_reports_the_configured_mapper(self, map_client: TestClient) -> None:
        body = map_client.get("/api/v1/map/status").json()

        assert body["mapper"] == "fixed_resolution_mapper_v1"
        assert body["is_adaptive"] is False
        assert body["adaptive_resolution_implemented"] is False
        assert body["lifecycle"] == "frame_local"

    def test_it_reports_the_mapped_extent_and_dimensions(self, map_client: TestClient) -> None:
        body = map_client.get("/api/v1/map/status").json()

        assert body["resolution_m"] == 0.5
        assert body["resolution_source"] == "fixed"
        assert body["width"] == 160
        assert body["height"] == 160
        assert body["total_cells"] == 160 * 160
        assert body["bounds"]["min_x"] == -40.0

    def test_active_cells_are_zero_before_any_frame(self, map_client: TestClient) -> None:
        body = map_client.get("/api/v1/map/status").json()

        assert body["active_cells"] == 0
        assert body["last_map_timestamp"] is None

    def test_it_reflects_a_frame_once_one_is_mapped(self, map_client: TestClient) -> None:
        map_client.post("/api/v1/lidar/map", json=frame_body(road_scene()))
        body = map_client.get("/api/v1/map/status").json()

        assert body["active_cells"] > 0
        assert body["last_map_timestamp"] is not None
        assert body["summary"]["frames_mapped"] == 1

    def test_it_names_what_it_does_not_model(self, map_client: TestClient) -> None:
        configuration = map_client.get("/api/v1/map/status").json()["configuration"]

        assert configuration["adaptive_resolution_implemented"] is False
        assert "probabilistic_occupancy" in configuration["not_modelled"]
        assert "adaptive_resolution" in configuration["not_modelled"]
        assert configuration["unobserved_height"] == "null, never zero"

    def test_the_component_is_ready_but_never_implemented(self, map_client: TestClient) -> None:
        component = map_client.get("/api/v1/map/status").json()["component"]

        assert component["readiness"] == "READY"
        assert component["implementation"] != "IMPLEMENTED"
        assert component["phase"] == 6


class TestMappingTelemetry:
    def test_the_summary_carries_counts_not_the_grid(self, map_client: TestClient) -> None:
        with map_client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            mapping = websocket.receive_json()["data"]["mapping"]

        assert mapping["mapper"] == "fixed_resolution_mapper_v1"
        assert mapping["is_adaptive"] is False
        assert "point_count" not in mapping
        assert "cells" not in mapping
        assert "occupancy" not in mapping

    def test_the_summary_reports_dimensions_and_lifecycle(self, map_client: TestClient) -> None:
        with map_client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            mapping = websocket.receive_json()["data"]["mapping"]

        assert mapping["resolution_m"] == 0.5
        assert mapping["width"] == 160
        assert mapping["lifecycle"] == "frame_local"
        assert mapping["adaptive_resolution_implemented"] is False

    def test_mapping_is_listed_among_what_the_channel_provides(
        self, map_client: TestClient
    ) -> None:
        with map_client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            data = websocket.receive_json()["data"]

        assert "mapping" in data["provides"]

    def test_the_adaptive_map_stream_remains_unavailable(self, map_client: TestClient) -> None:
        """A fixed-resolution map must not make the adaptive one look present."""
        with map_client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            data = websocket.receive_json()["data"]

        assert "adaptive_map" in data["not_yet_available"]

    def test_the_summary_reflects_a_mapped_frame(self, map_client: TestClient) -> None:
        map_client.post("/api/v1/lidar/map", json=frame_body(road_scene()))

        with map_client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            mapping = websocket.receive_json()["data"]["mapping"]

        assert mapping["frames_mapped"] == 1
        assert mapping["occupied_cells"] > 0
        assert mapping["input_points"] == (
            mapping["mapped_points"] + mapping["out_of_bounds_points"]
        )

    def test_the_payload_stays_small(self, map_client: TestClient) -> None:
        """A 25,600-cell grid must not reach a status channel."""
        import json

        map_client.post("/api/v1/lidar/map", json=frame_body(road_scene()))
        with map_client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            payload = websocket.receive_json()

        assert len(json.dumps(payload)) < 20_000
