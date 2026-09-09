"""Integration tests for preprocessing through POST /api/v1/lidar/frame.

The endpoint's Phase 1 behaviour must be unchanged when ``preprocess`` is
absent or false; the Phase 2A pipeline runs only when it is explicitly
requested.
"""

from __future__ import annotations

from typing import Any

from fastapi.testclient import TestClient

from adaptx.models.common import DataSource


def body(points: list[list[float]], **extra: Any) -> dict[str, Any]:
    """A frame request body with the given points."""
    payload: dict[str, Any] = {
        "frame_id": 1,
        "sensor_id": "test_lidar",
        "source": DataSource.SYNTHETIC_TEST.value,
        "points": points,
    }
    payload.update(extra)
    return payload


class TestBackwardCompatibility:
    def test_request_without_the_flag_behaves_as_before(self, client: TestClient) -> None:
        response = client.post("/api/v1/lidar/frame", json=body([[1.0, 2.0, 0.0]]))
        assert response.status_code == 202

        payload = response.json()
        assert payload["accepted"] is True
        assert payload["summary"]["point_count"] == 1
        assert payload["processing"] is None
        assert payload["input_summary"] is None

    def test_non_finite_values_are_still_rejected_without_the_flag(
        self, client: TestClient
    ) -> None:
        """Phase 1 contract: an unprocessed frame must be finite."""
        response = client.post(
            "/api/v1/lidar/frame", json=body([[1.0, 0.0, 0.0], [None, 0.0, 0.0]])
        )
        assert response.status_code == 422

    def test_explicit_false_matches_the_default(self, client: TestClient) -> None:
        response = client.post(
            "/api/v1/lidar/frame", json=body([[1.0, 2.0, 0.0]], preprocess=False)
        )
        assert response.status_code == 202
        assert response.json()["processing"] is None


class TestPreprocessingRequested:
    def test_metrics_are_returned(self, client: TestClient) -> None:
        response = client.post(
            "/api/v1/lidar/frame",
            json=body([[1.0, 2.0, 0.0], [3.0, 0.0, 0.0]], preprocess=True),
        )
        assert response.status_code == 202

        processing = response.json()["processing"]
        assert processing["processor"] == "preprocessing_v1"
        assert processing["input_point_count"] == 2
        assert processing["output_point_count"] == 2
        assert processing["duration_ms"] >= 0.0
        assert len(processing["stages"]) == 4

    def test_stage_names_and_order(self, client: TestClient) -> None:
        response = client.post("/api/v1/lidar/frame", json=body([[1.0, 0.0, 0.0]], preprocess=True))
        stages = [stage["stage"] for stage in response.json()["processing"]["stages"]]
        assert stages == ["validation", "invalid_removal", "roi_filter", "range_filter"]

    def test_out_of_range_points_are_filtered(self, client: TestClient) -> None:
        """The default max range is 100 m, so a 500 m point is dropped."""
        response = client.post(
            "/api/v1/lidar/frame",
            json=body([[5.0, 0.0, 0.0], [500.0, 0.0, 0.0]], preprocess=True),
        )
        assert response.status_code == 202

        payload = response.json()
        assert payload["summary"]["point_count"] == 1
        assert payload["input_summary"]["point_count"] == 2
        # 500 m on +x is outside the default ROI (x max 80 m), so the ROI stage
        # rejects it first - stage order is ROI then range.
        assert payload["processing"]["roi_rejected_count"] == 1

    def test_points_below_minimum_range_are_filtered(self, client: TestClient) -> None:
        """The default minimum range is 0.5 m (sensor blind zone)."""
        response = client.post(
            "/api/v1/lidar/frame",
            json=body([[0.1, 0.0, 0.0], [5.0, 0.0, 0.0]], preprocess=True),
        )
        payload = response.json()

        assert payload["processing"]["range_rejected_count"] == 1
        assert payload["summary"]["point_count"] == 1

    def test_summaries_describe_input_and_output(self, client: TestClient) -> None:
        response = client.post(
            "/api/v1/lidar/frame",
            json=body([[1.0, 0.0, 0.0], [500.0, 0.0, 0.0]], preprocess=True),
        )
        payload = response.json()

        assert payload["input_summary"]["point_count"] == 2
        assert payload["summary"]["point_count"] == 1
        assert payload["summary"]["frame_id"] == payload["input_summary"]["frame_id"]
        assert payload["summary"]["source"] == "synthetic_test"

    def test_ingest_records_the_processed_frame(self, client: TestClient) -> None:
        client.post(
            "/api/v1/lidar/frame",
            json=body([[1.0, 0.0, 0.0], [500.0, 0.0, 0.0]], preprocess=True),
        )
        metrics = client.get("/api/v1/system/metrics").json()
        assert metrics["point_count"] == 1

    def test_malformed_points_are_still_rejected(self, client: TestClient) -> None:
        response = client.post(
            "/api/v1/lidar/frame",
            json=body([[0.0, 0.0, 0.0], [1.0, 1.0]], preprocess=True),
        )
        assert response.status_code == 422
        assert response.json()["error"]["code"] == "invalid_point_cloud"

    def test_point_count_limits_still_apply(self, client: TestClient) -> None:
        """The test settings cap ingestion at 1000 points."""
        points = [[float(i % 50), 0.0, 0.0] for i in range(1001)]
        response = client.post("/api/v1/lidar/frame", json=body(points, preprocess=True))
        assert response.status_code == 422
        assert response.json()["error"]["details"]["max_points"] == 1000

    def test_a_frame_reduced_to_zero_points_is_still_accepted(self, client: TestClient) -> None:
        """Filtering everything out is a valid observation, not an error."""
        response = client.post(
            "/api/v1/lidar/frame",
            json=body([[5000.0, 0.0, 0.0], [6000.0, 0.0, 0.0]], preprocess=True),
        )
        assert response.status_code == 202

        payload = response.json()
        assert payload["summary"]["point_count"] == 0
        assert payload["processing"]["output_point_count"] == 0
        assert payload["summary"]["bounds"] is None


class TestOpenAPIContract:
    def test_preprocess_flag_is_documented(self, client: TestClient) -> None:
        schema = client.get("/openapi.json").json()
        request_schema = schema["components"]["schemas"]["LiDARFrameRequest"]
        assert "preprocess" in request_schema["properties"]
        assert "preprocess" not in request_schema.get("required", [])

    def test_response_additions_are_optional(self, client: TestClient) -> None:
        schema = client.get("/openapi.json").json()
        response_schema = schema["components"]["schemas"]["LiDARFrameResponse"]
        required = response_schema.get("required", [])

        assert "processing" in response_schema["properties"]
        assert "processing" not in required
        assert "input_summary" not in required


class TestSystemStatusReporting:
    def test_preprocessing_is_reported_as_its_own_component(self, client: TestClient) -> None:
        components = {
            c["name"]: c for c in client.get("/api/v1/system/status").json()["components"]
        }
        preprocessing = components["lidar_preprocessing"]

        assert preprocessing["readiness"] == "READY"
        assert preprocessing["implementation"] == "PARTIAL"
        assert preprocessing["phase"] == 2

    def test_detection_is_still_reported_as_planned(self, client: TestClient) -> None:
        """Preprocessing existing must not make object detection look implemented."""
        components = {
            c["name"]: c for c in client.get("/api/v1/system/status").json()["components"]
        }
        assert components["perception"]["implementation"] == "PLANNED"
        assert components["perception"]["readiness"] == "NOT_READY"

    def test_unimplemented_stages_are_named_in_the_detail(self, client: TestClient) -> None:
        components = {
            c["name"]: c for c in client.get("/api/v1/system/status").json()["components"]
        }
        detail = components["lidar_preprocessing"]["detail"]
        assert "no downsampling" in detail
        assert "ground segmentation" in detail
