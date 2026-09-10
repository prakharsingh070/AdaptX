"""End-to-end prediction: raw frames through processing, detection, tracking and prediction.

Feeds real point clouds through the whole chain frame by frame, so a trajectory
only exists if clustering, classification, association, velocity measurement
and extrapolation all worked on genuine geometry.

Also covers the HTTP contract for ``POST /api/v1/lidar/predict`` and
``GET /api/v1/prediction/status``.
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
    PredictionSettings,
    Settings,
    TrackingSettings,
)
from adaptx.core.lifecycle import build_context
from adaptx.models.common import DataSource, ObjectClass
from adaptx.models.point_cloud import RawPointCloudFrame
from adaptx.perception.detector import GeometricObjectDetector
from adaptx.perception.pipeline import LiDARProcessingPipeline
from adaptx.prediction.constant_velocity import ConstantVelocityPredictor
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


def scene_with_vehicle_at(x: float) -> np.ndarray:
    """A road plane with one vehicle at the given forward distance."""
    return np.vstack(
        [
            scenes.ground_plane(extent_m=40.0),
            scenes.vehicle((x, -3.0, scenes.GROUND_Z_M + 0.8)),
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
    """Processing, detection, tracking and prediction wired together."""

    def __init__(self, **prediction_overrides: Any) -> None:
        self.pipeline = LiDARProcessingPipeline(PIPELINE_SETTINGS)
        self.detector = GeometricObjectDetector(DetectionSettings())
        self.tracker = GeometricObjectTracker(TrackingSettings(velocity_smoothing=1.0))
        self.predictor = ConstantVelocityPredictor(PredictionSettings(**prediction_overrides))

    def frame(self, points: np.ndarray, frame_id: int, seconds: float) -> Any:
        processed = self.pipeline.run(raw(points, frame_id, seconds))
        detection = self.detector.detect(processed.frame)
        tracking = self.tracker.update(
            detection.objects,
            processed.frame.timestamp,
            frame_id=frame_id,
            sensor_id=processed.frame.sensor_id,
        )
        return self.predictor.predict(
            tracking.tracks,
            tracking.timestamp,
            frame_id=tracking.frame_id,
            sensor_id=tracking.sensor_id,
        )


class TestFullPerceptionChain:
    def test_the_first_frame_predicts_nothing_because_nothing_has_moved_yet(self) -> None:
        """One observation cannot show motion, so no trajectory can exist."""
        result = Chain().frame(scene_with_vehicle_at(10.0), 0, 0.0)

        assert result.trajectories == []
        assert result.considered_track_count >= 1
        assert all(s.status.value == "insufficient_velocity" for s in result.skipped)

    def test_a_trajectory_appears_once_velocity_has_been_measured(self) -> None:
        chain = Chain()
        chain.frame(scene_with_vehicle_at(10.0), 0, 0.0)
        result = chain.frame(scene_with_vehicle_at(11.0), 1, 0.5)

        assert result.predicted_track_count >= 1

    def test_the_predicted_path_follows_the_measured_motion(self) -> None:
        """1 m per 0.5 s is 2 m/s, so t+1 must be about 2 m further forward."""
        chain = Chain()
        for index in range(4):
            result = chain.frame(scene_with_vehicle_at(10.0 + index), index, index * 0.5)

        trajectory = result.trajectories[0]
        start = trajectory.points[0].position.x
        at_one_second = next(p for p in trajectory.points if p.time_offset_s == 1.0)

        assert at_one_second.position.x == pytest.approx(start + 2.0, abs=0.3)

    def test_predictions_never_overwrite_the_measured_track_position(self) -> None:
        chain = Chain()
        chain.frame(scene_with_vehicle_at(10.0), 0, 0.0)
        chain.frame(scene_with_vehicle_at(11.0), 1, 0.5)
        tracking = chain.tracker.update([], EPOCH + timedelta(seconds=1.0), frame_id=2)

        for track in tracking.tracks:
            assert track.position.x == pytest.approx(11.0, abs=1.5)

    def test_the_chain_is_deterministic_across_identical_runs(self) -> None:
        def run() -> list[float]:
            chain = Chain()
            for index in range(3):
                result = chain.frame(scene_with_vehicle_at(10.0 + index), index, index * 0.5)
            return [p.position.x for p in result.trajectories[0].points]

        assert run() == run()

    def test_provenance_survives_the_whole_chain(self) -> None:
        """Synthetic points must not become live-sensor trajectories."""
        chain = Chain()
        chain.frame(scene_with_vehicle_at(10.0), 0, 0.0)
        result = chain.frame(scene_with_vehicle_at(11.0), 1, 0.5)

        assert result.trajectories[0].source is DataSource.SYNTHETIC_TEST
        assert result.sensor_id == "roof_lidar"

    def test_a_vehicle_that_stops_yields_a_stationary_trajectory(self) -> None:
        """A measured standstill is real information, unlike an unknown velocity."""
        chain = Chain()
        chain.frame(scene_with_vehicle_at(10.0), 0, 0.0)
        chain.frame(scene_with_vehicle_at(11.0), 1, 0.5)
        for index in range(2, 6):
            result = chain.frame(scene_with_vehicle_at(11.0), index, index * 0.5)

        trajectory = result.trajectories[0]
        travelled = abs(trajectory.points[-1].position.x - trajectory.points[0].position.x)

        assert travelled < 0.5
        assert trajectory.points[-1].position_uncertainty_m > (
            trajectory.points[0].position_uncertainty_m
        )

    def test_a_vehicle_class_is_carried_through_to_prediction(self) -> None:
        chain = Chain()
        chain.frame(scene_with_vehicle_at(10.0), 0, 0.0)
        result = chain.frame(scene_with_vehicle_at(11.0), 1, 0.5)
        tracking = chain.tracker

        assert any(
            track.object_class is ObjectClass.VEHICLE
            for track in tracking.update([], EPOCH + timedelta(seconds=1.0), frame_id=2).tracks
        )
        assert result.predicted_track_count >= 1


@pytest.fixture
def predict_client() -> Iterator[TestClient]:
    """A client whose pipeline is configured so detection can actually run."""
    settings = Settings(
        app={"environment": "development", "debug": True},
        api={"cors_origins": []},
        logging={"level": "WARNING"},
        carla={"enabled": False, "use_mock": False},
        lidar=PIPELINE_SETTINGS.model_dump(),
        websocket={"telemetry_interval_s": 0.05, "max_connections": 4},
    )
    context = build_context(settings)
    with TestClient(create_app(settings=settings, context=context)) as client:
        yield client


def frame_body(points: np.ndarray, frame_id: int, seconds: float) -> dict[str, Any]:
    return {
        "frame_id": frame_id,
        "sensor_id": "roof_lidar",
        "source": "synthetic_test",
        "points": points.tolist(),
        "timestamp": (EPOCH + timedelta(seconds=seconds)).isoformat(),
    }


class TestPredictEndpoint:
    def test_a_valid_frame_returns_all_four_stages(self, predict_client: TestClient) -> None:
        response = predict_client.post(
            "/api/v1/lidar/predict", json=frame_body(scene_with_vehicle_at(10.0), 0, 0.0)
        )

        assert response.status_code == 200
        body = response.json()
        assert body["accepted"] is True
        for stage in ("prediction", "tracking", "detection", "processing"):
            assert stage in body

    def test_the_first_frame_reports_skipped_tracks_not_invented_paths(
        self, predict_client: TestClient
    ) -> None:
        body = predict_client.post(
            "/api/v1/lidar/predict", json=frame_body(scene_with_vehicle_at(10.0), 0, 0.0)
        ).json()
        prediction = body["prediction"]

        assert prediction["trajectories"] == []
        assert len(prediction["skipped"]) >= 1
        assert prediction["skipped"][0]["status"] == "insufficient_velocity"

    def test_a_second_frame_produces_a_trajectory(self, predict_client: TestClient) -> None:
        predict_client.post(
            "/api/v1/lidar/predict", json=frame_body(scene_with_vehicle_at(10.0), 0, 0.0)
        )
        body = predict_client.post(
            "/api/v1/lidar/predict", json=frame_body(scene_with_vehicle_at(11.0), 1, 0.5)
        ).json()
        trajectories = body["prediction"]["trajectories"]

        assert len(trajectories) >= 1
        trajectory = trajectories[0]
        assert len(trajectory["points"]) == 13
        assert trajectory["points"][0]["time_offset_s"] == 0.0
        assert trajectory["points"][-1]["time_offset_s"] == 3.0
        assert trajectory["predictor_name"] == "constant_velocity_v1"

    def test_uncertainty_increases_along_the_returned_trajectory(
        self, predict_client: TestClient
    ) -> None:
        predict_client.post(
            "/api/v1/lidar/predict", json=frame_body(scene_with_vehicle_at(10.0), 0, 0.0)
        )
        body = predict_client.post(
            "/api/v1/lidar/predict", json=frame_body(scene_with_vehicle_at(11.0), 1, 0.5)
        ).json()
        points = body["prediction"]["trajectories"][0]["points"]
        values = [p["position_uncertainty_m"] for p in points]

        assert values == sorted(values)
        assert values[-1] > values[0]

    def test_the_response_carries_no_raw_point_arrays(self, predict_client: TestClient) -> None:
        body = predict_client.post(
            "/api/v1/lidar/predict", json=frame_body(scene_with_vehicle_at(10.0), 0, 0.0)
        ).json()

        assert "points" not in body
        assert "points" not in body["summary"]
        assert "points" not in body["prediction"]

    def test_the_response_is_labelled_a_baseline(self, predict_client: TestClient) -> None:
        body = predict_client.post(
            "/api/v1/lidar/predict", json=frame_body(scene_with_vehicle_at(10.0), 0, 0.0)
        ).json()

        assert body["prediction"]["is_baseline"] is True
        assert body["prediction"]["model_name"] == "constant_velocity"
        assert "unmeasured" in body["detail"]

    def test_resetting_tracking_clears_what_prediction_sees(
        self, predict_client: TestClient
    ) -> None:
        predict_client.post(
            "/api/v1/lidar/predict", json=frame_body(scene_with_vehicle_at(10.0), 0, 0.0)
        )
        predict_client.post(
            "/api/v1/lidar/predict", json=frame_body(scene_with_vehicle_at(11.0), 1, 0.5)
        )
        predict_client.post("/api/v1/tracking/reset")
        body = predict_client.post(
            "/api/v1/lidar/predict", json=frame_body(scene_with_vehicle_at(12.0), 2, 1.0)
        ).json()

        assert body["prediction"]["trajectories"] == []

    @pytest.mark.parametrize(
        "points",
        [
            [[0.0, 0.0]],
            [[0.0, 0.0, 0.0], [1.0, 1.0]],
            [["a", "b", "c"]],
        ],
    )
    def test_malformed_points_are_rejected_with_422(
        self, predict_client: TestClient, points: list[Any]
    ) -> None:
        response = predict_client.post(
            "/api/v1/lidar/predict",
            json={
                "frame_id": 0,
                "sensor_id": "roof_lidar",
                "source": "synthetic_test",
                "points": points,
            },
        )

        assert response.status_code == 422

    def test_an_empty_frame_predicts_nothing_without_failing(
        self, predict_client: TestClient
    ) -> None:
        response = predict_client.post(
            "/api/v1/lidar/predict",
            json={
                "frame_id": 0,
                "sensor_id": "roof_lidar",
                "source": "synthetic_test",
                "points": [],
            },
        )

        assert response.status_code == 200
        prediction = response.json()["prediction"]
        assert prediction["trajectories"] == []
        assert prediction["considered_track_count"] == 0


class TestPredictionStatusEndpoint:
    def test_it_reports_the_configured_predictor(self, client: TestClient) -> None:
        body = client.get("/api/v1/prediction/status").json()

        assert body["predictor"] == "constant_velocity_v1"
        assert body["model_name"] == "constant_velocity"
        assert body["is_baseline"] is True

    def test_it_reports_the_supported_horizon_and_interval(self, client: TestClient) -> None:
        body = client.get("/api/v1/prediction/status").json()

        assert body["horizon_s"] == 3.0
        assert body["interval_s"] == 0.25
        assert body["points_per_trajectory"] == 13

    def test_it_declares_its_uncertainty_heuristic(self, client: TestClient) -> None:
        body = client.get("/api/v1/prediction/status").json()

        assert body["uncertainty_is_heuristic"] is True
        assert body["uncertainty_model"] == "heuristic_linear_growth"

    def test_it_names_what_it_does_not_model(self, client: TestClient) -> None:
        body = client.get("/api/v1/prediction/status").json()
        unmodelled = body["configuration"]["unmodelled_factors"]

        assert "acceleration" in unmodelled
        assert "object_interaction" in unmodelled

    def test_it_reports_no_accuracy_figure(self, client: TestClient) -> None:
        """No labelled trajectories exist, so no accuracy may be claimed."""
        body = client.get("/api/v1/prediction/status").json()

        assert not any("accuracy" in key for key in body)
        assert body["component"]["implementation"] == "PARTIAL"

    def test_the_component_is_ready_but_never_implemented(self, client: TestClient) -> None:
        body = client.get("/api/v1/prediction/status").json()

        assert body["component"]["readiness"] == "READY"
        assert body["component"]["implementation"] != "IMPLEMENTED"
        assert body["component"]["phase"] == 5

    def test_it_is_documented_in_the_openapi_schema(self, client: TestClient) -> None:
        paths = client.get("/openapi.json").json()["paths"]

        assert "/api/v1/prediction/status" in paths
        assert "/api/v1/lidar/predict" in paths
        assert "post" in paths["/api/v1/lidar/predict"]


class TestPredictionTelemetry:
    def test_the_summary_carries_counts_not_trajectory_points(self, client: TestClient) -> None:
        with client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            prediction = websocket.receive_json()["data"]["prediction"]

        assert prediction["predictor"] == "constant_velocity_v1"
        assert prediction["is_baseline"] is True
        assert prediction["uncertainty_is_heuristic"] is True
        assert "trajectories" not in prediction
        assert "points" not in prediction

    def test_the_summary_reports_the_horizon_and_model(self, client: TestClient) -> None:
        with client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            prediction = websocket.receive_json()["data"]["prediction"]

        assert prediction["horizon_s"] == 3.0
        assert prediction["model"] == "constant_velocity"

    def test_before_any_frame_the_summary_says_so(self, client: TestClient) -> None:
        with client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            prediction = websocket.receive_json()["data"]["prediction"]

        assert prediction["frames_predicted"] == 0
        assert "no frame has been predicted" in prediction["note"]

    def test_prediction_is_listed_among_what_the_channel_provides(self, client: TestClient) -> None:
        with client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            data = websocket.receive_json()["data"]

        assert "prediction" in data["provides"]

    def test_the_summary_reflects_a_frame_once_one_is_predicted(
        self, predict_client: TestClient
    ) -> None:
        predict_client.post(
            "/api/v1/lidar/predict", json=frame_body(scene_with_vehicle_at(10.0), 0, 0.0)
        )
        summary = predict_client.get("/api/v1/prediction/status").json()["summary"]

        assert summary["frames_predicted"] == 1
        assert summary["considered_tracks"] >= 1
        assert "counts_by_status" in summary
