"""Integration tests for the Phase 2B stages through the LiDAR endpoint.

The default client fixture leaves the 2B stages off, so these build their own
app with them enabled and assert both that the metrics reach the response and
that the default contract is untouched.
"""

from __future__ import annotations

import json
from collections.abc import Iterator
from typing import Any

import pytest
from fastapi.testclient import TestClient

from adaptx.api.app import create_app
from adaptx.config.settings import Settings
from adaptx.core.lifecycle import build_context
from adaptx.models.common import DataSource


def scene_points() -> list[list[float]]:
    """Road, an obstacle on it, and one isolated stray return."""
    road = [[float(x) * 0.5, float(y) * 0.5, -1.8] for x in range(6) for y in range(4)]
    obstacle = [[1.0 + i * 0.05, 1.0, -0.5] for i in range(8)]
    return road + obstacle + [[40.0, 40.0, 3.0]]


def body(points: list[list[float]], **extra: Any) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "frame_id": 3,
        "sensor_id": "test_lidar",
        "source": DataSource.SYNTHETIC_TEST.value,
        "points": points,
        "preprocess": True,
    }
    payload.update(extra)
    return payload


@pytest.fixture
def full_client() -> Iterator[TestClient]:
    """A client whose backend has every Phase 2B stage enabled."""
    settings = Settings(
        logging={"level": "CRITICAL"},
        carla={"enabled": False},
        api={"cors_origins": []},
        lidar={
            "min_points": 0,
            "max_points": 10_000,
            "min_range_m": 0.0,
            "max_range_m": 1000.0,
            "roi_x_min_m": -500.0,
            "roi_x_max_m": 500.0,
            "roi_y_min_m": -500.0,
            "roi_y_max_m": 500.0,
            "roi_z_min_m": -500.0,
            "roi_z_max_m": 500.0,
            "voxel_enabled": True,
            "voxel_size_m": 0.2,
            "ground_enabled": True,
            "ground_cell_size_m": 1.0,
            "noise_enabled": True,
            "noise_cell_size_m": 1.0,
            "noise_min_neighbors": 1,
        },
    )
    app = create_app(settings=settings, context=build_context(settings))
    with TestClient(app) as client:
        yield client


class TestDefaultsUnchanged:
    def test_default_backend_reports_four_stages(self, client: TestClient) -> None:
        """The stock configuration must still behave exactly as Phase 2A did."""
        response = client.post("/api/v1/lidar/frame", json=body([[1.0, 0.0, 0.0]]))
        payload = response.json()

        assert response.status_code == 202
        assert len(payload["processing"]["stages"]) == 4
        assert payload["ground_summary"] is None
        assert payload["processing"]["voxel_reduced_count"] == 0
        assert payload["processing"]["ground_point_count"] == 0
        assert payload["processing"]["noise_removed_count"] == 0

    def test_new_response_field_is_optional_in_the_schema(self, client: TestClient) -> None:
        schema = client.get("/openapi.json").json()
        response_schema = schema["components"]["schemas"]["LiDARFrameResponse"]

        assert "ground_summary" in response_schema["properties"]
        assert "ground_summary" not in response_schema.get("required", [])

    def test_new_metric_fields_are_documented(self, client: TestClient) -> None:
        schema = client.get("/openapi.json").json()
        properties = schema["components"]["schemas"]["ProcessingMetrics"]["properties"]

        for field in ("voxel_reduced_count", "ground_point_count", "noise_removed_count"):
            assert field in properties


class TestStagesEnabled:
    def test_all_seven_stages_are_reported(self, full_client: TestClient) -> None:
        response = full_client.post("/api/v1/lidar/frame", json=body(scene_points()))
        assert response.status_code == 202

        stages = [s["stage"] for s in response.json()["processing"]["stages"]]
        assert stages == [
            "validation",
            "invalid_removal",
            "roi_filter",
            "range_filter",
            "voxel_downsample",
            "ground_segmentation",
            "noise_filter",
        ]

    def test_metrics_report_real_counts(self, full_client: TestClient) -> None:
        payload = full_client.post("/api/v1/lidar/frame", json=body(scene_points())).json()
        processing = payload["processing"]

        assert processing["input_point_count"] == len(scene_points())
        assert processing["voxel_reduced_count"] > 0
        assert processing["ground_point_count"] > 0
        assert processing["duration_ms"] > 0.0

    def test_counts_partition_the_input(self, full_client: TestClient) -> None:
        processing = full_client.post("/api/v1/lidar/frame", json=body(scene_points())).json()[
            "processing"
        ]

        accounted = (
            processing["invalid_point_count"]
            + processing["roi_rejected_count"]
            + processing["range_rejected_count"]
            + processing["voxel_reduced_count"]
            + processing["ground_point_count"]
            + processing["noise_removed_count"]
            + processing["output_point_count"]
        )
        assert accounted == processing["input_point_count"]

    def test_ground_summary_is_returned_separately(self, full_client: TestClient) -> None:
        payload = full_client.post("/api/v1/lidar/frame", json=body(scene_points())).json()

        assert payload["ground_summary"] is not None
        assert (
            payload["ground_summary"]["point_count"] == payload["processing"]["ground_point_count"]
        )
        # `summary` describes the non-ground output, not the whole cloud.
        assert payload["summary"]["point_count"] == payload["processing"]["output_point_count"]
        assert payload["ground_summary"]["frame_id"] == payload["summary"]["frame_id"]

    def test_ingest_records_the_non_ground_frame(self, full_client: TestClient) -> None:
        payload = full_client.post("/api/v1/lidar/frame", json=body(scene_points())).json()
        metrics = full_client.get("/api/v1/system/metrics").json()

        assert metrics["point_count"] == payload["processing"]["output_point_count"]

    def test_malformed_input_is_still_rejected(self, full_client: TestClient) -> None:
        response = full_client.post("/api/v1/lidar/frame", json=body([[0.0, 0.0, 0.0], [1.0, 1.0]]))
        assert response.status_code == 422
        assert response.json()["error"]["code"] == "invalid_point_cloud"

    def test_non_finite_points_are_still_removed_and_counted(self, full_client: TestClient) -> None:
        """Posted as a raw body: httpx refuses to serialise an infinity, but
        `1e999` is valid JSON number syntax that parses to one."""
        payload_json = json.dumps(body(scene_points())).replace(
            '"points": [', '"points": [[1e999, 0.0, 0.0], ', 1
        )
        response = full_client.post(
            "/api/v1/lidar/frame",
            content=payload_json,
            headers={"Content-Type": "application/json"},
        )
        assert response.status_code == 202
        assert response.json()["processing"]["invalid_point_count"] == 1

    def test_repeated_requests_give_identical_metrics(self, full_client: TestClient) -> None:
        first = full_client.post("/api/v1/lidar/frame", json=body(scene_points())).json()
        second = full_client.post("/api/v1/lidar/frame", json=body(scene_points())).json()

        for field in (
            "input_point_count",
            "voxel_reduced_count",
            "ground_point_count",
            "noise_removed_count",
            "output_point_count",
        ):
            assert first["processing"][field] == second["processing"][field]
