"""End-to-end risk: the whole chain through to a risk assessment.

The point of these tests is that the **real** Phase 2-6 contracts feed the risk
engine without adapters or stand-ins. An engine that only worked on
hand-built tracks would pass its unit tests and still be wrong.

Also covers the HTTP contract for ``POST /api/v1/lidar/risk`` and
``GET /api/v1/risk/status``, and the risk telemetry summary.
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
    RiskSettings,
    Settings,
    TrackingSettings,
)
from adaptx.core.lifecycle import build_context
from adaptx.mapping.grid_mapper import FixedResolutionMapper
from adaptx.models.common import DataSource
from adaptx.models.point_cloud import RawPointCloudFrame
from adaptx.models.risk import RiskLevel
from adaptx.models.risk_assessment import AssessmentStatus, MapObservation
from adaptx.perception.detector import GeometricObjectDetector
from adaptx.perception.pipeline import LiDARProcessingPipeline
from adaptx.prediction.constant_velocity import ConstantVelocityPredictor
from adaptx.risk.heuristic import HeuristicRiskEngine
from adaptx.services.risk_service import RiskService
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


def scene_with_vehicle_at(x: float) -> np.ndarray:
    """A road plane with one vehicle at the given forward distance."""
    return np.vstack(
        [
            scenes.ground_plane(extent_m=40.0),
            scenes.vehicle((x, 0.0, scenes.GROUND_Z_M + 0.8)),
        ]
    )


def raw(points: np.ndarray, frame_id: int, seconds: float) -> RawPointCloudFrame:
    return RawPointCloudFrame.from_sequence(
        points.tolist(),
        frame_id=frame_id,
        sensor_id="roof_lidar",
        source=DataSource.SYNTHETIC_TEST,
        timestamp=EPOCH + timedelta(seconds=seconds),
    )


class Chain:
    """Every phase wired together, exactly as the API route does it."""

    def __init__(self, **risk_overrides: Any) -> None:
        self.pipeline = LiDARProcessingPipeline(PIPELINE_SETTINGS)
        self.detector = GeometricObjectDetector(DetectionSettings())
        self.tracker = GeometricObjectTracker(TrackingSettings(velocity_smoothing=1.0))
        self.predictor = ConstantVelocityPredictor(PredictionSettings())
        self.mapper = FixedResolutionMapper(MAP_SETTINGS)
        self.engine = HeuristicRiskEngine(RiskSettings(**risk_overrides))

    def frame(self, points: np.ndarray, frame_id: int, seconds: float) -> Any:
        processed = self.pipeline.run(raw(points, frame_id, seconds))
        detection = self.detector.detect(processed.frame)
        tracking = self.tracker.update(
            detection.objects,
            processed.frame.timestamp,
            frame_id=frame_id,
            sensor_id=processed.frame.sensor_id,
        )
        prediction = self.predictor.predict(tracking.tracks, tracking.timestamp, frame_id=frame_id)
        spatial_map = self.mapper.build(processed.frame)
        return self.engine.assess_many(
            tracking.tracks,
            trajectories=prediction.trajectories,
            spatial_map=spatial_map,
            timestamp=tracking.timestamp,
            frame_id=frame_id,
            sensor_id=processed.frame.sensor_id,
        )


class TestWholeChainReachesRisk:
    def test_a_real_scene_produces_assessments(self) -> None:
        result = Chain().frame(scene_with_vehicle_at(15.0), 0, 0.0)

        assert result.considered_track_count >= 1
        assert len(result.assessments) == result.considered_track_count

    def test_the_first_frame_has_no_velocity_and_says_so(self) -> None:
        """One observation cannot show motion, and risk must not pretend otherwise."""
        result = Chain().frame(scene_with_vehicle_at(15.0), 0, 0.0)
        assessment = result.assessments[0]

        assert assessment.closing_speed_mps is None
        assert assessment.uncertainty.velocity_known is False
        assert "never measured" in assessment.reason

    def test_measured_motion_reaches_the_assessment(self) -> None:
        """1 m per 0.5 s toward the ego is 2 m/s of closing speed."""
        chain = Chain()
        for index in range(4):
            result = chain.frame(scene_with_vehicle_at(20.0 - index), index, index * 0.5)

        assessment = result.assessments[0]
        assert assessment.closing_speed_mps == pytest.approx(2.0, abs=0.4)
        assert assessment.uncertainty.velocity_known is True

    def test_an_approaching_vehicle_grows_more_concerning(self) -> None:
        chain = Chain()
        scores = []
        for index in range(6):
            result = chain.frame(scene_with_vehicle_at(25.0 - index * 3.0), index, index * 0.5)
            scored = result.scored_assessments
            if scored:
                scores.append(max(a.risk_score for a in scored if a.risk_score is not None))

        assert len(scores) >= 3
        assert scores[-1] > scores[0]

    def test_prediction_reaches_the_assessment(self) -> None:
        chain = Chain()
        for index in range(3):
            result = chain.frame(scene_with_vehicle_at(20.0 - index), index, index * 0.5)

        assessment = result.assessments[0]
        assert assessment.trajectory is not None
        assert assessment.uncertainty.prediction_available is True
        assert assessment.trajectory.horizon_s == 3.0

    def test_map_context_reaches_the_assessment(self) -> None:
        result = Chain().frame(scene_with_vehicle_at(15.0), 0, 0.0)
        assessment = result.assessments[0]

        assert assessment.map_context.observation is MapObservation.OBSERVED_OCCUPIED
        assert assessment.map_context.point_count is not None
        assert assessment.map_context.point_count > 0

    def test_provenance_survives_the_whole_chain(self) -> None:
        result = Chain().frame(scene_with_vehicle_at(15.0), 0, 0.0)

        assert result.assessments[0].source is DataSource.SYNTHETIC_TEST
        assert result.sensor_id == "roof_lidar"
        assert result.is_baseline is True

    def test_the_chain_is_deterministic(self) -> None:
        def run() -> list[float | None]:
            chain = Chain()
            for index in range(3):
                result = chain.frame(scene_with_vehicle_at(20.0 - index), index, index * 0.5)
            return [a.risk_score for a in result.assessments]

        assert run() == run()

    def test_an_empty_scene_is_assessed_without_failing(self) -> None:
        result = Chain().frame(scenes.ground_plane(extent_m=5.0), 0, 0.0)

        assert result.considered_track_count == 0
        assert result.highest_risk_level is RiskLevel.UNKNOWN


class TestRiskService:
    def test_the_service_orchestrates_the_pipeline_outputs(self) -> None:
        service = RiskService(RiskSettings())
        pipeline = LiDARProcessingPipeline(PIPELINE_SETTINGS)
        detector = GeometricObjectDetector(DetectionSettings())
        tracker = GeometricObjectTracker(TrackingSettings())
        predictor = ConstantVelocityPredictor(PredictionSettings())
        mapper = FixedResolutionMapper(MAP_SETTINGS)

        processed = pipeline.run(raw(scene_with_vehicle_at(15.0), 0, 0.0))
        detection = detector.detect(processed.frame)
        tracking = tracker.update(detection.objects, processed.frame.timestamp)
        prediction = predictor.predict(tracking.tracks, tracking.timestamp)
        spatial_map = mapper.build(processed.frame)

        result = service.assess_from_pipeline(
            tracking, prediction=prediction, spatial_map=spatial_map
        )

        assert result.considered_track_count >= 1
        assert service.frames_assessed == 1

    def test_the_service_works_without_prediction_or_map(self) -> None:
        """Missing context raises uncertainty rather than blocking assessment."""
        service = RiskService(RiskSettings())
        pipeline = LiDARProcessingPipeline(PIPELINE_SETTINGS)
        detector = GeometricObjectDetector(DetectionSettings())
        tracker = GeometricObjectTracker(TrackingSettings())

        processed = pipeline.run(raw(scene_with_vehicle_at(15.0), 0, 0.0))
        detection = detector.detect(processed.frame)
        tracking = tracker.update(detection.objects, processed.frame.timestamp)

        result = service.assess_from_pipeline(tracking)
        assessment = result.assessments[0]

        assert assessment.status is AssessmentStatus.ASSESSED
        assert assessment.trajectory is None
        assert assessment.map_context.observation is MapObservation.NO_MAP
        assert assessment.uncertainty.score > 0.0

    def test_reset_clears_counters_but_changes_no_output(self) -> None:
        service = RiskService(RiskSettings())
        pipeline = LiDARProcessingPipeline(PIPELINE_SETTINGS)
        detector = GeometricObjectDetector(DetectionSettings())
        tracker = GeometricObjectTracker(TrackingSettings())
        processed = pipeline.run(raw(scene_with_vehicle_at(15.0), 0, 0.0))
        tracking = tracker.update(
            detector.detect(processed.frame).objects, processed.frame.timestamp
        )

        before = service.assess_from_pipeline(tracking)
        service.reset()
        assert service.frames_assessed == 0
        assert service.last_result is None

        after = service.assess_from_pipeline(tracking)
        assert [a.risk_score for a in before.assessments] == [
            a.risk_score for a in after.assessments
        ]

    def test_the_summary_reports_no_frame_before_the_first(self) -> None:
        summary = RiskService(RiskSettings()).summary()

        assert summary["frames_assessed"] == 0
        assert "no frame has been assessed" in str(summary["note"])

    def test_the_summary_ranks_the_most_concerning_objects_first(self) -> None:
        service = RiskService(RiskSettings())
        chain = Chain()
        processed = chain.pipeline.run(raw(scene_with_vehicle_at(8.0), 0, 0.0))
        tracking = chain.tracker.update(
            chain.detector.detect(processed.frame).objects, processed.frame.timestamp
        )
        service.assess_from_pipeline(tracking)
        objects = service.summary()["objects"]

        assert isinstance(objects, list)
        scores = [o["risk_score"] for o in objects if o["risk_score"] is not None]
        assert scores == sorted(scores, reverse=True)


@pytest.fixture
def risk_client() -> Iterator[TestClient]:
    """A client configured so the whole chain can actually run."""
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


def frame_body(points: np.ndarray, frame_id: int = 0, seconds: float = 0.0, **extra: Any) -> dict:
    body: dict[str, Any] = {
        "frame_id": frame_id,
        "sensor_id": "roof_lidar",
        "source": "synthetic_test",
        "points": points.tolist(),
        "timestamp": (EPOCH + timedelta(seconds=seconds)).isoformat(),
    }
    body.update(extra)
    return body


class TestRiskEndpoint:
    def test_a_valid_frame_returns_assessments(self, risk_client: TestClient) -> None:
        response = risk_client.post(
            "/api/v1/lidar/risk", json=frame_body(scene_with_vehicle_at(15.0))
        )

        assert response.status_code == 200
        body = response.json()
        assert body["accepted"] is True
        assert body["risk"]["engine"] == "heuristic_risk_v1"
        assert body["risk"]["considered_track_count"] >= 1

    def test_all_stages_are_reported(self, risk_client: TestClient) -> None:
        body = risk_client.post(
            "/api/v1/lidar/risk", json=frame_body(scene_with_vehicle_at(15.0))
        ).json()

        for stage in ("risk", "tracking", "detection", "processing"):
            assert stage in body
        assert body["prediction_summary"]["predictor"] == "constant_velocity_v1"
        assert body["map_summary"] is not None

    def test_multiple_objects_are_each_assessed(self, risk_client: TestClient) -> None:
        scene = np.vstack(
            [
                scenes.ground_plane(extent_m=40.0),
                scenes.vehicle((15.0, 0.0, scenes.GROUND_Z_M + 0.8)),
                scenes.vehicle((25.0, 10.0, scenes.GROUND_Z_M + 0.8)),
                scenes.pedestrian((8.0, -5.0, scenes.GROUND_Z_M + 0.9)),
            ]
        )
        body = risk_client.post("/api/v1/lidar/risk", json=frame_body(scene)).json()

        assert body["risk"]["considered_track_count"] >= 3
        ids = [a["track_id"] for a in body["risk"]["assessments"]]
        assert len(ids) == len(set(ids))

    def test_the_response_carries_no_raw_arrays(self, risk_client: TestClient) -> None:
        body = risk_client.post(
            "/api/v1/lidar/risk", json=frame_body(scene_with_vehicle_at(15.0))
        ).json()

        assert "points" not in body
        assert "cells" not in body
        assert "point_count" not in body["map_summary"]
        for assessment in body["risk"]["assessments"]:
            assert "points" not in assessment

    def test_an_empty_frame_assesses_nothing_without_failing(self, risk_client: TestClient) -> None:
        response = risk_client.post(
            "/api/v1/lidar/risk",
            json={
                "frame_id": 0,
                "sensor_id": "roof_lidar",
                "source": "synthetic_test",
                "points": [],
            },
        )

        assert response.status_code == 200
        risk = response.json()["risk"]
        assert risk["considered_track_count"] == 0
        assert risk["assessments"] == []

    def test_map_context_can_be_turned_off(self, risk_client: TestClient) -> None:
        body = risk_client.post(
            "/api/v1/lidar/risk",
            json=frame_body(scene_with_vehicle_at(15.0), include_map_context=False),
        ).json()

        assert body["map_summary"] is None
        for assessment in body["risk"]["assessments"]:
            assert assessment["map_context"]["observation"] == "no_map"

    def test_velocity_is_null_on_the_first_frame_over_http(self, risk_client: TestClient) -> None:
        body = risk_client.post(
            "/api/v1/lidar/risk", json=frame_body(scene_with_vehicle_at(15.0))
        ).json()
        assessment = body["risk"]["assessments"][0]

        assert assessment["closing_speed_mps"] is None
        assert assessment["speed_mps"] is None
        assert assessment["uncertainty"]["velocity_known"] is False

    def test_the_response_is_labelled_heuristic(self, risk_client: TestClient) -> None:
        body = risk_client.post(
            "/api/v1/lidar/risk", json=frame_body(scene_with_vehicle_at(15.0))
        ).json()

        assert body["risk"]["is_baseline"] is True
        assert body["risk"]["scoring_model"] == "heuristic_weighted_factors"
        assert "not a probability of collision" in body["detail"]
        assert "never folded into the score" in body["detail"]

    def test_the_response_carries_no_resolution(self, risk_client: TestClient) -> None:
        """The Phase 7/8 boundary, asserted at the wire format."""
        body = risk_client.post(
            "/api/v1/lidar/risk", json=frame_body(scene_with_vehicle_at(15.0))
        ).json()

        for assessment in body["risk"]["assessments"]:
            assert "resolution_m" not in assessment
            assert "resolution_level" not in assessment
        assert "resolution_m" not in body["risk"]["configuration"]

    @pytest.mark.parametrize(
        "points", [[[0.0, 0.0]], [[0.0, 0.0, 0.0], [1.0, 1.0]], [["a", "b", "c"]]]
    )
    def test_malformed_points_are_rejected_with_422(
        self, risk_client: TestClient, points: list[Any]
    ) -> None:
        response = risk_client.post(
            "/api/v1/lidar/risk",
            json={
                "frame_id": 0,
                "sensor_id": "roof_lidar",
                "source": "synthetic_test",
                "points": points,
            },
        )

        assert response.status_code == 422

    def test_it_is_documented_in_the_openapi_schema(self, risk_client: TestClient) -> None:
        paths = risk_client.get("/openapi.json").json()["paths"]

        assert "/api/v1/lidar/risk" in paths
        assert "post" in paths["/api/v1/lidar/risk"]


class TestRiskStatusEndpoint:
    def test_it_reports_the_configured_engine(self, risk_client: TestClient) -> None:
        body = risk_client.get("/api/v1/risk/status").json()

        assert body["engine"] == "heuristic_risk_v1"
        assert body["is_baseline"] is True
        assert body["baseline_engine"] == "baseline_proximity"
        assert body["scoring_model"] == "heuristic_weighted_factors"

    def test_it_never_claims_a_collision_probability(self, risk_client: TestClient) -> None:
        body = risk_client.get("/api/v1/risk/status").json()

        assert body["score_is_heuristic"] is True
        assert body["is_collision_probability"] is False
        assert body["uncertainty_is_heuristic"] is True

    def test_it_states_it_does_not_decide_resolution(self, risk_client: TestClient) -> None:
        assert risk_client.get("/api/v1/risk/status").json()["decides_resolution"] is False

    def test_it_names_what_it_does_not_model(self, risk_client: TestClient) -> None:
        unmodelled = risk_client.get("/api/v1/risk/status").json()["configuration"][
            "unmodelled_factors"
        ]

        assert "time_to_collision" in unmodelled
        assert "calibrated_collision_probability" in unmodelled

    def test_thresholds_cover_the_scored_levels_only(self, risk_client: TestClient) -> None:
        body = risk_client.get("/api/v1/risk/status").json()

        assert set(body["thresholds"]) == {"low", "medium", "high", "critical"}
        assert "unknown" in body["risk_levels"]
        assert "unknown" not in body["thresholds"]

    def test_it_reports_no_assessment_before_the_first_frame(self, risk_client: TestClient) -> None:
        body = risk_client.get("/api/v1/risk/status").json()

        assert body["frames_assessed"] == 0
        assert body["last_assessment_timestamp"] is None

    def test_it_reflects_a_frame_once_one_is_assessed(self, risk_client: TestClient) -> None:
        risk_client.post("/api/v1/lidar/risk", json=frame_body(scene_with_vehicle_at(15.0)))
        body = risk_client.get("/api/v1/risk/status").json()

        assert body["frames_assessed"] == 1
        assert body["last_assessment_timestamp"] is not None
        assert body["summary"]["total_objects"] >= 1

    def test_the_component_is_ready_but_never_implemented(self, risk_client: TestClient) -> None:
        component = risk_client.get("/api/v1/risk/status").json()["component"]

        assert component["readiness"] == "READY"
        assert component["implementation"] != "IMPLEMENTED"
        assert component["phase"] == 7


class TestRiskTelemetry:
    def test_the_summary_carries_counts_not_full_assessments(self, risk_client: TestClient) -> None:
        with risk_client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            risk = websocket.receive_json()["data"]["risk"]

        assert risk["risk_engine"] == "heuristic_risk_v1"
        assert risk["is_baseline"] is True
        assert "assessments" not in risk
        assert "trajectories" not in risk

    def test_the_summary_labels_the_score_heuristic(self, risk_client: TestClient) -> None:
        with risk_client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            risk = websocket.receive_json()["data"]["risk"]

        assert risk["score_is_heuristic"] is True
        assert risk["is_collision_probability"] is False

    def test_risk_is_listed_among_what_the_channel_provides(self, risk_client: TestClient) -> None:
        with risk_client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            data = websocket.receive_json()["data"]

        assert "risk" in data["provides"]

    def test_the_spatial_risk_field_remains_unavailable(self, risk_client: TestClient) -> None:
        """Object-level risk exists; a per-cell risk field does not."""
        with risk_client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            data = websocket.receive_json()["data"]

        assert "risk_field" in data["not_yet_available"]

    def test_the_summary_reflects_an_assessed_frame(self, risk_client: TestClient) -> None:
        risk_client.post("/api/v1/lidar/risk", json=frame_body(scene_with_vehicle_at(10.0)))

        with risk_client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            risk = websocket.receive_json()["data"]["risk"]

        assert risk["frames_assessed"] == 1
        assert risk["total_objects"] >= 1
        assert risk["highest_risk_level"] in {"low", "medium", "high", "critical", "unknown"}
        assert len(risk["objects"]) <= 5

    def test_the_payload_stays_small(self, risk_client: TestClient) -> None:
        import json

        risk_client.post("/api/v1/lidar/risk", json=frame_body(scene_with_vehicle_at(10.0)))
        with risk_client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            payload = websocket.receive_json()

        assert len(json.dumps(payload)) < 25_000
