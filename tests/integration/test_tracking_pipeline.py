"""End-to-end tracking: raw frames through processing, detection and tracking.

Feeds real point clouds through the whole chain frame by frame, so a track only
exists if clustering, classification and association all worked on genuine
geometry.
"""

from __future__ import annotations

import math
from collections.abc import Iterator
from datetime import UTC, datetime, timedelta
from typing import Any

import numpy as np
import pytest
from fastapi.testclient import TestClient

from adaptx.api.app import create_app
from adaptx.config.settings import DetectionSettings, LiDARSettings, Settings, TrackingSettings
from adaptx.core.lifecycle import build_context
from adaptx.models.common import DataSource, ObjectClass
from adaptx.models.point_cloud import RawPointCloudFrame
from adaptx.perception.detector import GeometricObjectDetector
from adaptx.perception.pipeline import LiDARProcessingPipeline
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
    """Processing, detection and tracking wired together for a test sequence."""

    def __init__(self, **tracking_overrides: Any) -> None:
        self.pipeline = LiDARProcessingPipeline(PIPELINE_SETTINGS)
        self.detector = GeometricObjectDetector(DetectionSettings())
        self.tracker = GeometricObjectTracker(TrackingSettings(**tracking_overrides))

    def frame(self, points: np.ndarray, frame_id: int, seconds: float) -> Any:
        processed = self.pipeline.run(raw(points, frame_id, seconds))
        detection = self.detector.detect(processed.frame)
        return self.tracker.update(
            detection.objects,
            processed.frame.timestamp,
            frame_id=frame_id,
            sensor_id=processed.frame.sensor_id,
        )


class TestFullTemporalChain:
    def test_a_vehicle_keeps_one_id_across_frames(self) -> None:
        chain = Chain()
        results = [
            chain.frame(scene_with_vehicle_at(10.0 + index), index, index * 0.5)
            for index in range(4)
        ]

        vehicle_ids = [
            [t.track_id for t in result.tracks if t.object_class is ObjectClass.VEHICLE]
            for result in results
        ]
        assert all(len(ids) == 1 for ids in vehicle_ids)
        assert len({ids[0] for ids in vehicle_ids}) == 1

    def test_velocity_emerges_only_after_two_observations(self) -> None:
        chain = Chain()
        first = chain.frame(scene_with_vehicle_at(10.0), 0, 0.0)
        second = chain.frame(scene_with_vehicle_at(11.0), 1, 0.5)

        vehicle_first = next(t for t in first.tracks if t.object_class is ObjectClass.VEHICLE)
        vehicle_second = next(t for t in second.tracks if t.object_class is ObjectClass.VEHICLE)

        assert vehicle_first.velocity is None
        assert vehicle_second.velocity is not None
        assert vehicle_second.velocity.x == pytest.approx(2.0, abs=0.2)

    def test_measured_velocity_matches_the_supplied_timestamps(self) -> None:
        """1 m per 0.5 s must read as 2 m/s, from the timestamps alone."""
        chain = Chain(velocity_smoothing=1.0)
        for index in range(4):
            result = chain.frame(scene_with_vehicle_at(10.0 + index), index, index * 0.5)

        vehicle = next(t for t in result.tracks if t.object_class is ObjectClass.VEHICLE)
        assert vehicle.velocity.x == pytest.approx(2.0, abs=0.2)
        assert vehicle.speed_mps == pytest.approx(2.0, abs=0.2)

    def test_class_is_preserved_across_frames(self) -> None:
        chain = Chain()
        for index in range(4):
            result = chain.frame(scene_with_vehicle_at(10.0 + index), index, index * 0.5)

        assert any(t.object_class is ObjectClass.VEHICLE for t in result.tracks)

    def test_metadata_propagates_through_the_chain(self) -> None:
        chain = Chain()
        result = chain.frame(scene_with_vehicle_at(10.0), 42, 0.0)

        assert result.frame_id == 42
        assert result.sensor_id == "roof_lidar"
        assert result.timestamp == EPOCH
        assert all(t.source is DataSource.SYNTHETIC_TEST for t in result.tracks)

    def test_no_invalid_numbers_reach_the_tracks(self) -> None:
        chain = Chain()
        for index in range(4):
            result = chain.frame(scene_with_vehicle_at(10.0 + index), index, index * 0.5)

        for track in result.tracks:
            assert math.isfinite(track.position.x)
            assert math.isfinite(track.confidence)
            if track.velocity is not None:
                assert math.isfinite(track.velocity.magnitude)
            if track.heading_rad is not None:
                assert math.isfinite(track.heading_rad)

    def test_a_disappearing_object_coasts_then_is_dropped(self) -> None:
        chain = Chain(min_hits_to_confirm=1, max_missed_frames=1)
        for index in range(3):
            chain.frame(scene_with_vehicle_at(10.0 + index), index, index * 0.5)

        ground_only = scenes.ground_plane(extent_m=40.0)
        coasting = chain.frame(ground_only, 3, 1.5)
        assert any(t.missed_frames == 1 for t in coasting.tracks)

        dropped = chain.frame(ground_only, 4, 2.0)
        assert dropped.deleted_track_ids

    def test_a_new_object_entering_gets_a_new_id(self) -> None:
        chain = Chain()
        chain.frame(scene_with_vehicle_at(10.0), 0, 0.0)

        with_pedestrian = np.vstack(
            [
                scene_with_vehicle_at(11.0),
                scenes.pedestrian((20.0, 8.0, scenes.GROUND_Z_M + 0.9)),
            ]
        )
        result = chain.frame(with_pedestrian, 1, 0.5)
        assert result.new_track_ids

    def test_the_chain_is_deterministic(self) -> None:
        def run_chain() -> list[tuple[int, float]]:
            chain = Chain()
            for index in range(4):
                result = chain.frame(scene_with_vehicle_at(10.0 + index), index, index * 0.5)
            return sorted((t.track_id, round(t.position.x, 6)) for t in result.tracks)

        assert run_chain() == run_chain()


class TestTrackingAPI:
    @pytest.fixture
    def track_client(self) -> Iterator[TestClient]:
        settings = Settings(
            logging={"level": "CRITICAL"},
            carla={"enabled": False},
            api={"cors_origins": []},
            lidar=PIPELINE_SETTINGS.model_dump(),
        )
        app = create_app(settings=settings, context=build_context(settings))
        with TestClient(app) as client:
            yield client

    @staticmethod
    def _body(points: np.ndarray, frame_id: int, seconds: float) -> dict[str, Any]:
        return {
            "frame_id": frame_id,
            "sensor_id": "roof_lidar",
            "source": DataSource.SYNTHETIC_TEST.value,
            "timestamp": (EPOCH + timedelta(seconds=seconds)).isoformat(),
            "points": points.tolist(),
        }

    def test_endpoint_tracks_across_sequential_frames(self, track_client: TestClient) -> None:
        ids: list[int] = []
        for index in range(3):
            response = track_client.post(
                "/api/v1/lidar/track",
                json=self._body(scene_with_vehicle_at(10.0 + index), index, index * 0.5),
            )
            assert response.status_code == 200
            vehicles = [
                t for t in response.json()["tracking"]["tracks"] if t["object_class"] == "vehicle"
            ]
            assert len(vehicles) == 1
            ids.append(vehicles[0]["track_id"])

        assert len(set(ids)) == 1

    def test_velocity_is_null_on_the_first_frame_then_measured(
        self, track_client: TestClient
    ) -> None:
        first = track_client.post(
            "/api/v1/lidar/track", json=self._body(scene_with_vehicle_at(10.0), 0, 0.0)
        ).json()
        second = track_client.post(
            "/api/v1/lidar/track", json=self._body(scene_with_vehicle_at(11.0), 1, 0.5)
        ).json()

        vehicle_first = next(
            t for t in first["tracking"]["tracks"] if t["object_class"] == "vehicle"
        )
        vehicle_second = next(
            t for t in second["tracking"]["tracks"] if t["object_class"] == "vehicle"
        )

        assert vehicle_first["velocity"] is None
        assert vehicle_second["velocity"]["x"] == pytest.approx(2.0, abs=0.2)

    def test_reset_clears_state(self, track_client: TestClient) -> None:
        track_client.post(
            "/api/v1/lidar/track", json=self._body(scene_with_vehicle_at(10.0), 0, 0.0)
        )
        reset = track_client.post("/api/v1/tracking/reset")

        assert reset.status_code == 200
        assert reset.json()["cleared_track_count"] >= 1

        after = track_client.get("/api/v1/tracking/status").json()
        assert after["summary"]["active_tracks"] == 0
        assert after["summary"]["frames_tracked"] == 0

    def test_ids_restart_after_reset(self, track_client: TestClient) -> None:
        track_client.post(
            "/api/v1/lidar/track", json=self._body(scene_with_vehicle_at(10.0), 0, 0.0)
        )
        track_client.post("/api/v1/tracking/reset")
        result = track_client.post(
            "/api/v1/lidar/track", json=self._body(scene_with_vehicle_at(10.0), 0, 0.0)
        ).json()

        assert min(t["track_id"] for t in result["tracking"]["tracks"]) == 0

    def test_status_reports_state(self, track_client: TestClient) -> None:
        track_client.post("/api/v1/tracking/reset")
        track_client.post(
            "/api/v1/lidar/track", json=self._body(scene_with_vehicle_at(10.0), 0, 0.0)
        )
        summary = track_client.get("/api/v1/tracking/status").json()["summary"]

        assert summary["frames_tracked"] == 1
        assert summary["tracker"] == "geometric_tracker_v1"
        assert summary["is_baseline"] is True

    def test_response_carries_all_three_stages(self, track_client: TestClient) -> None:
        payload = track_client.post(
            "/api/v1/lidar/track", json=self._body(scene_with_vehicle_at(10.0), 0, 0.0)
        ).json()

        assert payload["processing"]["duration_ms"] > 0.0
        assert payload["detection"]["duration_ms"] > 0.0
        assert payload["tracking"]["duration_ms"] > 0.0

    def test_response_carries_no_raw_points(self, track_client: TestClient) -> None:
        payload = track_client.post(
            "/api/v1/lidar/track", json=self._body(scene_with_vehicle_at(10.0), 0, 0.0)
        ).json()

        assert "points" not in payload
        assert "points" not in payload["tracking"]

    def test_no_future_trajectories_are_exposed(self, track_client: TestClient) -> None:
        """Prediction is not implemented; nothing may look like a forecast."""
        payload = track_client.post(
            "/api/v1/lidar/track", json=self._body(scene_with_vehicle_at(10.0), 0, 0.0)
        ).json()
        track = payload["tracking"]["tracks"][0]

        assert "trajectory" not in track
        assert "predicted_trajectory" not in track
        assert "predicted_position" in track  # association state, explicitly named

    def test_malformed_points_are_rejected(self, track_client: TestClient) -> None:
        response = track_client.post(
            "/api/v1/lidar/track",
            json={
                "frame_id": 1,
                "sensor_id": "x",
                "source": "synthetic_test",
                "points": [[0.0, 0.0, 0.0], [1.0, 1.0]],
            },
        )
        assert response.status_code == 422
        assert response.json()["error"]["code"] == "invalid_point_cloud"

    def test_existing_endpoints_still_work(self, track_client: TestClient) -> None:
        assert track_client.get("/health").status_code == 200
        assert track_client.get("/api/v1/system/status").status_code == 200
        assert (
            track_client.post(
                "/api/v1/lidar/detect", json=self._body(scene_with_vehicle_at(10.0), 0, 0.0)
            ).status_code
            == 200
        )

    def test_endpoints_are_documented(self, track_client: TestClient) -> None:
        paths = track_client.get("/openapi.json").json()["paths"]
        assert "/api/v1/lidar/track" in paths
        assert "/api/v1/tracking/reset" in paths
        assert "/api/v1/tracking/status" in paths
