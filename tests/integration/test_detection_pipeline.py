"""End-to-end detection: raw frame through Phase 2 processing into Phase 3.

Covers the whole chain the API exercises — raw cloud, ground segmentation,
clustering, filtering, classification — and the detection endpoint itself.
"""

from __future__ import annotations

from collections.abc import Iterator
from typing import Any

import numpy as np
import pytest
from fastapi.testclient import TestClient

from adaptx.api.app import create_app
from adaptx.config.settings import DetectionSettings, LiDARSettings, Settings
from adaptx.core.lifecycle import build_context
from adaptx.models.common import DataSource, ObjectClass
from adaptx.models.point_cloud import RawPointCloudFrame
from adaptx.perception.detector import GeometricObjectDetector
from adaptx.perception.pipeline import LiDARProcessingPipeline
from tests.fixtures import scenes

#: Sensing volume wide enough for the test scenes, with the 2B stages enabled
#: so the detector receives non-ground points as it is designed to.
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
    ground_height_tolerance_m=0.2,
    noise_enabled=False,
)


def road_scene() -> np.ndarray:
    """Ground plane with a vehicle and a pedestrian standing on it."""
    return np.vstack(
        [
            scenes.ground_plane(extent_m=40.0),
            scenes.vehicle((12.0, -3.0, scenes.GROUND_Z_M + 0.8)),
            scenes.pedestrian((8.0, 4.0, scenes.GROUND_Z_M + 0.9)),
        ]
    )


def raw(points: np.ndarray, **kwargs: Any) -> RawPointCloudFrame:
    defaults: dict[str, Any] = {
        "frame_id": 7,
        "sensor_id": "roof_lidar",
        "source": DataSource.SYNTHETIC_TEST,
    }
    defaults.update(kwargs)
    return RawPointCloudFrame.from_sequence(points.tolist(), **defaults)


def run_chain(points: np.ndarray, **detection_overrides: Any) -> tuple[Any, Any]:
    """Process a raw cloud, then detect in the non-ground output."""
    processed = LiDARProcessingPipeline(PIPELINE_SETTINGS).run(raw(points))
    detector = GeometricObjectDetector(DetectionSettings(**detection_overrides))
    return processed, detector.detect(processed.frame)


class TestFullChain:
    def test_a_road_scene_yields_the_expected_objects(self) -> None:
        processed, detection = run_chain(road_scene())

        assert processed.ground_frame is not None
        assert processed.ground_frame.point_count > 0

        classes = sorted(o.object_class.value for o in detection.objects)
        assert "vehicle" in classes
        assert "pedestrian" in classes

    def test_ground_points_do_not_become_objects(self) -> None:
        """The whole reason detection consumes the non-ground output."""
        _, detection = run_chain(scenes.ground_plane(extent_m=40.0))
        assert detection.object_count == 0

    def test_detection_consumes_only_the_non_ground_points(self) -> None:
        processed, detection = run_chain(road_scene())
        assert detection.input_point_count == processed.frame.point_count
        assert detection.non_ground_point_count == processed.frame.point_count

    def test_noise_is_rejected_rather_than_detected(self) -> None:
        """Stray returns above the road reach the detector and are filtered there.

        Placed over the ground plane on purpose: a speck alone in its own xy
        cell would be the lowest point of that cell and Phase 2B would classify
        it as ground, so it would never reach detection at all. That is the
        stage-order consequence recorded in the Phase 2B tests; here the point
        is to exercise the detector's own point-count filter.
        """
        strays = np.array(
            [
                [10.0, 5.0, scenes.GROUND_Z_M + 1.0],
                [10.3, 5.0, scenes.GROUND_Z_M + 1.0],
                [10.0, 5.3, scenes.GROUND_Z_M + 1.0],
                [10.15, 5.15, scenes.GROUND_Z_M + 1.1],
            ]
        )
        _, detection = run_chain(np.vstack([road_scene(), strays]))

        assert detection.rejected_count >= 1
        assert all(o.point_count >= 10 for o in detection.objects)

    def test_metadata_is_preserved_through_the_whole_chain(self) -> None:
        processed = LiDARProcessingPipeline(PIPELINE_SETTINGS).run(
            raw(road_scene(), frame_id=99, sensor_id="front_lidar")
        )
        detection = GeometricObjectDetector(DetectionSettings()).detect(processed.frame)

        assert detection.frame_id == 99
        assert detection.sensor_id == "front_lidar"
        assert all(o.frame_id == 99 for o in detection.objects)
        assert all(o.source is DataSource.SYNTHETIC_TEST for o in detection.objects)

    def test_no_invalid_numbers_reach_the_output(self) -> None:
        points = np.vstack([road_scene(), [[float("nan"), 0.0, 0.0]]])
        _, detection = run_chain(points)

        for detected in detection.objects:
            assert np.isfinite(detected.position.x)
            assert np.isfinite(detected.distance_m)
            assert np.isfinite(detected.confidence)
            assert all(np.isfinite(extent) for extent in detected.extents)

    def test_processing_metrics_are_populated(self) -> None:
        processed, detection = run_chain(road_scene())

        assert processed.metrics.duration_ms > 0.0
        assert detection.duration_ms > 0.0
        assert detection.clustering_duration_ms > 0.0

    def test_objects_sit_above_the_ground_plane(self) -> None:
        _, detection = run_chain(road_scene())
        for detected in detection.objects:
            assert detected.position.z > scenes.GROUND_Z_M

    def test_the_chain_is_deterministic(self) -> None:
        first = run_chain(road_scene())[1]
        second = run_chain(road_scene())[1]

        assert first.object_count == second.object_count
        assert [o.object_class for o in first.objects] == [o.object_class for o in second.objects]


class TestDetectionAPI:
    @pytest.fixture
    def detect_client(self) -> Iterator[TestClient]:
        """A client whose backend is configured for the detection endpoint."""
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
    def _body(points: np.ndarray) -> dict[str, Any]:
        return {
            "frame_id": 3,
            "sensor_id": "roof_lidar",
            "source": DataSource.SYNTHETIC_TEST.value,
            "points": points.tolist(),
        }

    def test_endpoint_returns_detections(self, detect_client: TestClient) -> None:
        response = detect_client.post("/api/v1/lidar/detect", json=self._body(road_scene()))
        assert response.status_code == 200

        payload = response.json()
        assert payload["accepted"] is True
        assert len(payload["detection"]["objects"]) > 0
        assert payload["detection"]["detector"] == "geometric_detector_v1"

    def test_response_carries_geometry_and_classification(self, detect_client: TestClient) -> None:
        payload = detect_client.post("/api/v1/lidar/detect", json=self._body(road_scene())).json()
        detected = payload["detection"]["objects"][0]

        for field in (
            "object_id",
            "object_class",
            "confidence",
            "position",
            "bounding_box",
            "point_count",
            "distance_m",
            "classifier",
        ):
            assert field in detected
        assert detected["velocity"] is None

    def test_response_carries_processing_metrics(self, detect_client: TestClient) -> None:
        payload = detect_client.post("/api/v1/lidar/detect", json=self._body(road_scene())).json()

        assert payload["processing"]["duration_ms"] > 0.0
        assert payload["detection"]["duration_ms"] > 0.0
        assert payload["detection"]["cluster_count"] >= 1

    def test_response_declares_the_baseline(self, detect_client: TestClient) -> None:
        payload = detect_client.post("/api/v1/lidar/detect", json=self._body(road_scene())).json()

        assert payload["detection"]["is_baseline"] is True
        assert "baseline" in payload["detail"]
        assert all(o["is_baseline_classification"] for o in payload["detection"]["objects"])

    def test_response_carries_no_raw_point_arrays(self, detect_client: TestClient) -> None:
        """Summaries and geometry only: the cloud itself must not be echoed."""
        payload = detect_client.post("/api/v1/lidar/detect", json=self._body(road_scene())).json()

        assert "points" not in payload
        assert "points" not in payload["summary"]
        assert "points" not in payload["detection"]

    def test_ground_summary_is_returned(self, detect_client: TestClient) -> None:
        payload = detect_client.post("/api/v1/lidar/detect", json=self._body(road_scene())).json()
        assert payload["ground_summary"]["point_count"] > 0

    def test_malformed_points_are_rejected(self, detect_client: TestClient) -> None:
        response = detect_client.post(
            "/api/v1/lidar/detect",
            json={
                "frame_id": 1,
                "sensor_id": "x",
                "source": "synthetic_test",
                "points": [[0.0, 0.0, 0.0], [1.0, 1.0]],
            },
        )
        assert response.status_code == 422
        assert response.json()["error"]["code"] == "invalid_point_cloud"

    def test_an_empty_scene_detects_nothing_without_erroring(
        self, detect_client: TestClient
    ) -> None:
        response = detect_client.post(
            "/api/v1/lidar/detect",
            json=self._body(scenes.ground_plane(extent_m=20.0)),
        )
        assert response.status_code == 200
        assert response.json()["detection"]["objects"] == []

    def test_existing_endpoints_still_work(self, detect_client: TestClient) -> None:
        assert detect_client.get("/health").status_code == 200
        assert detect_client.get("/api/v1/system/status").status_code == 200
        assert (
            detect_client.post("/api/v1/lidar/frame", json=self._body(scenes.vehicle())).status_code
            == 202
        )

    def test_endpoint_is_documented(self, detect_client: TestClient) -> None:
        paths = detect_client.get("/openapi.json").json()["paths"]
        assert "/api/v1/lidar/detect" in paths
        assert "post" in paths["/api/v1/lidar/detect"]

    def test_repeated_requests_agree(self, detect_client: TestClient) -> None:
        body = self._body(road_scene())
        first = detect_client.post("/api/v1/lidar/detect", json=body).json()
        second = detect_client.post("/api/v1/lidar/detect", json=body).json()

        assert [o["object_class"] for o in first["detection"]["objects"]] == [
            o["object_class"] for o in second["detection"]["objects"]
        ]


class TestClassificationInContext:
    def test_a_lone_vehicle_classifies_as_a_vehicle(self) -> None:
        points = np.vstack(
            [
                scenes.ground_plane(extent_m=30.0),
                scenes.vehicle((12.0, 0.0, scenes.GROUND_Z_M + 0.8)),
            ]
        )
        _, detection = run_chain(points)
        assert any(o.object_class is ObjectClass.VEHICLE for o in detection.objects)

    def test_a_lone_pedestrian_classifies_as_a_pedestrian(self) -> None:
        points = np.vstack(
            [
                scenes.ground_plane(extent_m=30.0),
                scenes.pedestrian((10.0, 0.0, scenes.GROUND_Z_M + 0.9)),
            ]
        )
        _, detection = run_chain(points)
        assert any(o.object_class is ObjectClass.PEDESTRIAN for o in detection.objects)

    def test_ambiguous_geometry_is_reported_unknown(self) -> None:
        points = np.vstack(
            [
                scenes.ground_plane(extent_m=40.0),
                scenes.ambiguous((20.0, 0.0, scenes.GROUND_Z_M + 1.2)),
            ]
        )
        _, detection = run_chain(points)
        unknown = [o for o in detection.objects if o.object_class is ObjectClass.UNKNOWN]

        assert unknown, "expected the ambiguous shape to be reported UNKNOWN"
        assert all(o.confidence == 0.0 for o in unknown)
