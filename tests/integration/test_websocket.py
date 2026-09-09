"""Integration tests for the /ws/telemetry channel."""

from __future__ import annotations

from fastapi.testclient import TestClient

from tests.fixtures.point_clouds import frame_request_body


class TestTelemetryChannel:
    def test_hello_message_describes_the_channel(self, client: TestClient) -> None:
        with client.websocket_connect("/ws/telemetry") as websocket:
            message = websocket.receive_json()

        assert message["type"] == "hello"
        assert message["sequence"] == 0
        assert message["timestamp"]
        assert message["data"]["name"] == "ADAPT-X"
        assert message["data"]["provides"] == ["system", "metrics"]

    def test_telemetry_carries_real_status_and_metrics(self, client: TestClient) -> None:
        with client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()  # hello
            message = websocket.receive_json()

        assert message["type"] == "telemetry"
        assert message["sequence"] == 1

        data = message["data"]
        assert data["system"]["state"] == "RUNNING"
        assert data["system"]["carla"]["status"] == "DISCONNECTED"
        assert "unavailable" in data["metrics"]

    def test_no_perception_data_is_fabricated(self, client: TestClient) -> None:
        """Phase 1 telemetry must not invent objects, tracks, risk or map cells."""
        with client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            data = websocket.receive_json()["data"]

        for stream in (
            "detected_objects",
            "tracked_objects",
            "predicted_trajectories",
            "risk_field",
            "adaptive_map",
        ):
            assert stream not in data
            assert stream in data["not_yet_available"]

    def test_messages_repeat_with_increasing_sequence(self, client: TestClient) -> None:
        with client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()  # hello
            sequences = [websocket.receive_json()["sequence"] for _ in range(3)]

        assert sequences == [1, 2, 3]

    def test_telemetry_reflects_ingested_frames(self, client: TestClient) -> None:
        client.post("/api/v1/lidar/frame", json=frame_request_body(12, frame_id=0))

        with client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()  # hello
            data = websocket.receive_json()["data"]

        assert data["metrics"]["point_count"] == 12
        assert data["system"]["lidar"]["frames_received"] == 1

    def test_connections_are_registered_and_released(self, client: TestClient) -> None:
        manager = client.app.state.connections
        assert manager.connection_count == 0

        with client.websocket_connect("/ws/telemetry") as websocket:
            websocket.receive_json()
            assert manager.connection_count == 1

        assert manager.connection_count == 0
