"""Integration tests: FastAPI startup, routing and endpoint behaviour."""

from __future__ import annotations

from fastapi import FastAPI
from fastapi.testclient import TestClient

from adaptx import __version__
from adaptx.models.common import DataSource
from tests.fixtures.point_clouds import frame_request_body, make_points


class TestApplicationStartup:
    def test_lifespan_attaches_the_context(self, app: FastAPI) -> None:
        with TestClient(app) as client:
            assert client.app.state.context is not None
            assert client.app.state.connections is not None
            assert client.get("/health").status_code == 200

    def test_openapi_schema_is_served(self, client: TestClient) -> None:
        schema = client.get("/openapi.json")
        assert schema.status_code == 200

        paths = schema.json()["paths"]
        for path in (
            "/health",
            "/api/v1/system/status",
            "/api/v1/system/metrics",
            "/api/v1/carla/status",
            "/api/v1/lidar/frame",
            "/api/v1/map/status",
            "/api/v1/risk/status",
        ):
            assert path in paths

    def test_unknown_route_returns_404(self, client: TestClient) -> None:
        assert client.get("/api/v1/does-not-exist").status_code == 404


class TestHealth:
    def test_health_reports_ok(self, client: TestClient) -> None:
        response = client.get("/health")
        assert response.status_code == 200

        body = response.json()
        assert body["status"] == "ok"
        assert body["version"] == __version__
        assert body["name"] == "ADAPT-X"


class TestSystemStatus:
    def test_status_shape(self, client: TestClient) -> None:
        response = client.get("/api/v1/system/status")
        assert response.status_code == 200

        body = response.json()
        assert body["state"] == "RUNNING"
        assert body["lidar"]["status"] == "DISCONNECTED"
        assert body["carla"]["status"] == "DISCONNECTED"
        assert body["uptime_s"] >= 0

    def test_every_subsystem_is_reported(self, client: TestClient) -> None:
        components = {
            c["name"]: c for c in client.get("/api/v1/system/status").json()["components"]
        }
        expected = {
            "api",
            "configuration",
            "telemetry",
            "lidar_ingest",
            "perception",
            "tracking",
            "mapping",
            "risk",
            "prediction",
        }
        assert expected <= set(components)

    def test_planned_modules_are_not_presented_as_working(self, client: TestClient) -> None:
        """Phase 5 moved prediction out of this list; mapping is still planned."""
        components = {
            c["name"]: c for c in client.get("/api/v1/system/status").json()["components"]
        }

        for name in ("mapping",):
            assert components[name]["implementation"] == "PLANNED"
            assert components[name]["readiness"] == "NOT_READY"

    def test_tracking_is_reported_as_partial_not_finished(self, client: TestClient) -> None:
        """Phase 4 shipped a geometric baseline, which is not a finished tracker."""
        components = {
            c["name"]: c for c in client.get("/api/v1/system/status").json()["components"]
        }
        tracking = components["tracking"]

        assert tracking["implementation"] == "PARTIAL"
        assert "baseline" in tracking["detail"]
        assert "no re-identification" in tracking["detail"]

    def test_prediction_is_reported_as_partial_not_finished(self, client: TestClient) -> None:
        """Phase 5 shipped a constant-velocity baseline, not a finished predictor.

        Retargeted in Phase 5: this test previously asserted PLANNED, because
        nothing implemented the predictor. A predictor now exists, so the
        property worth guarding is that shipping a *baseline* must never make
        prediction look finished or its numbers look validated.
        """
        components = {
            c["name"]: c for c in client.get("/api/v1/system/status").json()["components"]
        }
        prediction = components["prediction"]

        assert prediction["implementation"] == "PARTIAL"
        assert prediction["implementation"] != "IMPLEMENTED"
        assert "baseline" in prediction["detail"]
        assert "unmeasured" in prediction["detail"]

    def test_detection_is_reported_as_partial_not_finished(self, client: TestClient) -> None:
        """Phase 3 shipped a geometric baseline, which is not a finished detector."""
        components = {
            c["name"]: c for c in client.get("/api/v1/system/status").json()["components"]
        }
        perception = components["perception"]

        assert perception["implementation"] == "PARTIAL"
        assert "No trained model" in perception["detail"]


class TestSystemMetrics:
    def test_unmeasured_values_are_null_and_explained(self, client: TestClient) -> None:
        body = client.get("/api/v1/system/metrics").json()

        assert body["fps"] is None
        assert body["gpu_percent"] is None
        assert body["sample_count"] == 0
        assert body["unavailable"]

    def test_metrics_reflect_ingested_frames(self, client: TestClient) -> None:
        client.post("/api/v1/lidar/frame", json=frame_request_body(20, frame_id=0))
        client.post("/api/v1/lidar/frame", json=frame_request_body(30, frame_id=1))

        body = client.get("/api/v1/system/metrics").json()
        assert body["point_count"] == 30
        assert body["sample_count"] == 2
        assert body["fps"] is not None
        assert body["latency_ms"] >= 0


class TestCarlaStatus:
    def test_disabled_carla_is_reported_not_errored(self, client: TestClient) -> None:
        response = client.get("/api/v1/carla/status")
        assert response.status_code == 200

        body = response.json()
        assert body["status"] == "DISCONNECTED"
        assert body["enabled"] is False
        assert body["is_mock"] is False
        assert body["detail"]


class TestLiDARFrame:
    def test_valid_frame_is_accepted(self, client: TestClient) -> None:
        response = client.post("/api/v1/lidar/frame", json=frame_request_body(15, frame_id=2))
        assert response.status_code == 202

        body = response.json()
        assert body["accepted"] is True
        assert body["summary"]["frame_id"] == 2
        assert body["summary"]["point_count"] == 15
        assert body["summary"]["source"] == "synthetic_test"
        assert body["summary"]["bounds"] is not None

    def test_frame_with_intensity_is_accepted(self, client: TestClient) -> None:
        payload = frame_request_body(5)
        payload["points"] = make_points(5, columns=4).tolist()

        response = client.post("/api/v1/lidar/frame", json=payload)
        assert response.status_code == 202
        assert response.json()["summary"]["fields"] == ["x", "y", "z", "intensity"]

    def test_ragged_points_are_rejected(self, client: TestClient) -> None:
        payload = frame_request_body(3)
        payload["points"] = [[0.0, 0.0, 0.0], [1.0, 1.0]]

        response = client.post("/api/v1/lidar/frame", json=payload)
        assert response.status_code == 422
        assert response.json()["error"]["code"] == "invalid_point_cloud"

    def test_wrong_column_count_is_rejected(self, client: TestClient) -> None:
        payload = frame_request_body(3)
        payload["points"] = [[0.0, 1.0], [2.0, 3.0]]
        assert client.post("/api/v1/lidar/frame", json=payload).status_code == 422

    def test_too_many_points_are_rejected(self, client: TestClient) -> None:
        """The test settings cap ingestion at 1000 points."""
        response = client.post("/api/v1/lidar/frame", json=frame_request_body(1001))
        assert response.status_code == 422
        assert response.json()["error"]["details"]["max_points"] == 1000

    def test_missing_source_is_rejected(self, client: TestClient) -> None:
        """Provenance is mandatory so synthetic data cannot pass as sensor data."""
        payload = frame_request_body(3)
        del payload["source"]
        assert client.post("/api/v1/lidar/frame", json=payload).status_code == 422

    def test_invalid_source_is_rejected(self, client: TestClient) -> None:
        payload = frame_request_body(3)
        payload["source"] = "definitely_real_sensor"
        assert client.post("/api/v1/lidar/frame", json=payload).status_code == 422

    def test_ingest_updates_the_system_status(self, client: TestClient) -> None:
        client.post(
            "/api/v1/lidar/frame",
            json=frame_request_body(10, source=DataSource.SYNTHETIC_TEST),
        )
        lidar = client.get("/api/v1/system/status").json()["lidar"]

        assert lidar["status"] == "SIMULATED"
        assert lidar["frames_received"] == 1
        assert lidar["source"] == "synthetic_test"

    def test_live_labelled_ingest_reports_connected(self, client: TestClient) -> None:
        client.post(
            "/api/v1/lidar/frame",
            json=frame_request_body(10, source=DataSource.LIVE_SENSOR),
        )
        assert client.get("/api/v1/system/status").json()["lidar"]["status"] == "CONNECTED"


class TestMapStatus:
    def test_reports_configuration_and_no_implementation(self, client: TestClient) -> None:
        response = client.get("/api/v1/map/status")
        assert response.status_code == 200

        body = response.json()
        assert body["component"]["implementation"] == "PLANNED"
        assert body["active_cells"] == 0
        assert body["configuration"]["is_adaptive_algorithm_implemented"] is False
        assert set(body["resolution_levels"]) == {"low", "medium", "high", "critical"}
        assert body["resolution_levels"]["low"] > body["resolution_levels"]["critical"]


class TestRiskStatus:
    def test_reports_the_baseline_engine_explicitly(self, client: TestClient) -> None:
        response = client.get("/api/v1/risk/status")
        assert response.status_code == 200

        body = response.json()
        assert body["engine"] == "baseline_proximity"
        assert body["is_baseline"] is True
        assert body["component"]["implementation"] == "PARTIAL"
        assert body["configuration"]["modelled_factors"] == ["proximity"]
        assert "time_to_collision" in body["configuration"]["unmodelled_factors"]

    def test_thresholds_are_ordered_on_the_normalised_scale(self, client: TestClient) -> None:
        thresholds = client.get("/api/v1/risk/status").json()["thresholds"]
        assert (
            thresholds["low"] < thresholds["medium"] < thresholds["high"] < thresholds["critical"]
        )
        assert thresholds["critical"] <= 1.0
