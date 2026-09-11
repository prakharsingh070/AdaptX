"""The live simulation loop against the fake simulator (post-Phase-12 extension).

What these prove: the loop drives the real Phase 2-8 chain on every frame,
the controller reacts to what the chain produced, the snapshot carries what
was produced and nothing else, session controls behave, and every actor is
gone when the session ends. What they do not prove: anything about CARLA -
that is ``pytest -m carla``.
"""

from __future__ import annotations

import time
from collections.abc import Iterator

import pytest
from fastapi.testclient import TestClient

from adaptx.api.app import create_app
from adaptx.carla.session import CarlaSimulationSession
from adaptx.config.settings import CarlaSettings, Settings
from adaptx.control.models import ControllerState
from adaptx.core.exceptions import SimulatorUnavailableError
from adaptx.core.lifecycle import build_context
from adaptx.live.models import LiveState
from adaptx.live.service import LiveSessionError
from tests.fixtures.fake_carla import FakeActor, FakeCarlaModule, FakeWorld
from tests.integration.test_carla_pipeline import sim_settings


def live_settings(**live: object) -> Settings:
    base = sim_settings()
    payload = base.model_dump()
    payload["carla"]["enabled"] = True
    payload["carla"]["ego_spawn_index"] = 0
    payload["live"] = {"camera_width": 64, "camera_height": 36, **live}
    payload["control"] = {"max_speed_mps": 6.0, "resume_dwell_frames": 3}
    return Settings.model_validate(payload)


class Harness:
    """A context whose live service opens sessions on one fake world."""

    def __init__(self, settings: Settings) -> None:
        self.world = FakeWorld()
        self.module = FakeCarlaModule(self.world)
        self.sessions: list[CarlaSimulationSession] = []
        self.context = build_context(settings)

        def factory(carla_settings: CarlaSettings) -> CarlaSimulationSession:
            session = CarlaSimulationSession(
                carla_settings, carla_module=self.module, drive_ego=True
            )
            self.sessions.append(session)
            return session

        self.context.live._factory = factory  # type: ignore[attr-defined]

    def wait_frames(self, count: int, timeout_s: float = 20.0) -> None:
        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline:
            if self.context.live.status().snapshots_published >= count:
                return
            if self.context.live.state in (LiveState.ERROR, LiveState.STOPPED):
                break
            time.sleep(0.02)
        status = self.context.live.status()
        raise AssertionError(
            f"only {status.frames_processed} frames after {timeout_s}s: "
            f"{status.state} {status.detail} {status.last_error}"
        )

    def ego(self) -> FakeActor:
        return next(a for a in self.world.actors.values() if a.type_id.startswith("vehicle."))


@pytest.fixture
def harness() -> Iterator[Harness]:
    h = Harness(live_settings())
    yield h
    h.context.live.shutdown()


class TestLoop:
    def test_start_runs_the_chain_and_publishes_live_snapshots(self, harness: Harness) -> None:
        status = harness.context.live.start("static_obstacle", 7)
        assert status.state is LiveState.RUNNING
        assert status.scenario_id == "static_obstacle" and status.seed == 7
        assert status.carla_connected
        harness.wait_frames(8)

        sequence, snapshot = harness.context.scene.latest()
        assert sequence >= 8 and snapshot is not None
        assert snapshot.origin == "live:static_obstacle"
        assert snapshot.live is not None and snapshot.live.seed == 7
        assert snapshot.points is not None and snapshot.points.stage.value == "raw"
        assert snapshot.ego is not None and snapshot.ego.source.value == "simulation"
        assert snapshot.control is not None
        assert snapshot.live.timing.loop_ms > 0.0
        payload = snapshot.model_dump()
        assert "ground_truth" not in payload
        assert "expected_poses" not in payload
        # The frames also count as ingested LiDAR frames for status/metrics.
        assert harness.context.lidar.frames_received >= 8
        assert harness.context.carla.status().status.value == "CONNECTED"

    def test_the_ego_actually_moves_under_the_controller(self, harness: Harness) -> None:
        harness.context.live.start("random_urban_traffic", 1)
        harness.wait_frames(15)
        ego = harness.ego()
        assert ego.controls_applied, "the controller applied commands to the ego"
        assert ego.velocity.x > 0.0, "with no object ahead the ego cruises"
        assert ego.get_transform().location.x > 0.0
        status = harness.context.live.status()
        assert status.control is not None and status.control.state is ControllerState.CRUISING
        assert status.ego_speed_mps is not None and status.ego_speed_mps > 0.0

    def test_an_obstacle_ahead_is_detected_tracked_and_stops_the_ego(
        self, harness: Harness
    ) -> None:
        """The obstacle-stop demo, against the fake: the loop, end to end."""
        harness.context.live.start("static_obstacle", 3)
        # The parked car appears at 1 s (20 frames), 45 m ahead of the ego.
        harness.wait_frames(45)
        seen: dict[str, object] = {}
        deadline = time.perf_counter() + 30.0
        while time.perf_counter() < deadline:
            status = harness.context.live.status()
            if status.state is not LiveState.RUNNING:
                break
            command = status.control
            if command is not None and command.nearest_in_path_m is not None:
                seen["in_path"] = command.nearest_in_path_m
            if command is not None and command.state in (
                ControllerState.SLOWING,
                ControllerState.HOLDING,
                ControllerState.EMERGENCY_BRAKING,
                ControllerState.STOPPED,
            ):
                seen["reacted"] = command.state
            if (
                status.ego_speed_mps is not None
                and status.ego_speed_mps < 0.1
                and "reacted" in seen
            ):
                seen["stopped_at_frame"] = status.frames_processed
                break
            time.sleep(0.02)
        assert "in_path" in seen, "the parked car was never an in-path object"
        assert "reacted" in seen, "the controller never reacted"
        assert "stopped_at_frame" in seen, "the ego never stopped"
        events = harness.context.live.events()
        kinds = {e.kind for e in events}
        assert {"scenario", "object_detected", "controller"} <= kinds
        # It stopped before the obstacle, not in it: no collision was recorded.
        assert harness.context.live.status().collision_count == 0
        ego_x = harness.ego().get_transform().location.x
        obstacle = next(a for a in harness.world.actors.values() if a.type_id == "vehicle.audi.tt")
        assert ego_x < obstacle.get_transform().location.x

    def test_pause_freezes_the_simulator_and_resume_continues(self, harness: Harness) -> None:
        harness.context.live.start("random_urban_traffic", 1)
        harness.wait_frames(5)
        paused = harness.context.live.pause()
        assert paused.state is LiveState.PAUSED
        time.sleep(0.2)  # the frame in flight when pause was requested completes
        frames = harness.context.live.status().frames_processed
        world_frame = harness.world.get_snapshot().frame
        time.sleep(0.3)
        assert harness.context.live.status().frames_processed == frames
        assert harness.world.get_snapshot().frame == world_frame, "nothing ticked the world"
        resumed = harness.context.live.resume()
        assert resumed.state is LiveState.RUNNING
        harness.wait_frames(frames + 3)
        with pytest.raises(LiveSessionError):
            harness.context.live.resume()

    def test_stop_destroys_every_actor_and_restores_the_world(self, harness: Harness) -> None:
        harness.context.live.start("mixed_obstacles", 5)
        harness.wait_frames(25)
        assert len(harness.world.actors) > 1, "traffic, ego, sensors and the obstacle exist"
        status = harness.context.live.stop()
        assert status.state is LiveState.STOPPED
        assert harness.world.actors == {}, "no actor leaked"
        assert harness.world.settings.synchronous_mode is False, "world settings restored"
        client = harness.module.clients[-1]
        assert client.traffic_manager is not None and client.traffic_manager.synchronous is False
        assert harness.context.carla.status().status.value == "DISCONNECTED"
        assert any(e.kind == "session_ended" for e in harness.context.live.events())

    def test_reset_restarts_the_same_scenario_and_seed(self, harness: Harness) -> None:
        harness.context.live.start("cyclist_crossing", 11)
        harness.wait_frames(3)
        first = harness.sessions[-1]
        status = harness.context.live.reset()
        assert status.state is LiveState.RUNNING
        assert status.scenario_id == "cyclist_crossing" and status.seed == 11
        assert harness.sessions[-1] is not first
        assert status.frames_processed == 0

    def test_a_collision_stops_the_session_and_is_recorded(self, harness: Harness) -> None:
        harness.context.live.start("random_urban_traffic", 1)
        harness.wait_frames(3)
        ego = harness.ego()
        other = next(
            a
            for a in harness.world.actors.values()
            if a is not ego and not a.type_id.startswith("sensor.")
        )
        harness.world.collide(ego, other)
        deadline = time.perf_counter() + 10.0
        while harness.context.live.state is LiveState.RUNNING and time.perf_counter() < deadline:
            time.sleep(0.02)
        status = harness.context.live.status()
        assert status.state is LiveState.COLLIDED
        assert status.collision_count == 1
        assert any(e.kind == "collision" for e in harness.context.live.events())
        assert harness.world.actors == {}

    def test_start_twice_is_refused_and_a_dead_server_is_reported(self, harness: Harness) -> None:
        harness.context.live.start("static_obstacle")
        with pytest.raises(LiveSessionError, match="already"):
            harness.context.live.start("static_obstacle")
        harness.context.live.stop()
        harness.module.connect_error = ConnectionError("refused")
        with pytest.raises(SimulatorUnavailableError):
            harness.context.live.start("static_obstacle")
        assert harness.context.live.state is LiveState.ERROR
        assert harness.world.actors == {}

    def test_controls_can_be_disabled(self) -> None:
        h = Harness(live_settings(controls_enabled=False))
        with pytest.raises(LiveSessionError, match="disabled"):
            h.context.live.start("static_obstacle")
        assert h.context.live.status().controls_enabled is False

    def test_the_camera_is_served_as_png_and_is_display_only(self, harness: Harness) -> None:
        harness.context.live.start("static_obstacle")
        harness.wait_frames(2)
        png = harness.context.live.camera_png()
        assert png is not None and png.startswith(b"\x89PNG")
        assert harness.context.live.status().camera_available


class TestApi:
    @pytest.fixture
    def client(self, harness: Harness) -> Iterator[TestClient]:
        app = create_app(settings=harness.context.settings, context=harness.context)
        with TestClient(app) as client:
            yield client

    def test_the_session_is_driven_over_http(self, client: TestClient, harness: Harness) -> None:
        assert client.get("/api/v1/live/status").json()["state"] == "IDLE"
        scenarios = client.get("/api/v1/live/scenarios").json()
        assert {s["scenario_id"] for s in scenarios} >= {"static_obstacle", "mixed_obstacles"}
        started = client.post(
            "/api/v1/live/start", json={"scenario_id": "static_obstacle", "seed": 2}
        )
        assert started.status_code == 202 and started.json()["state"] == "RUNNING"
        harness.wait_frames(4)
        latest = client.get("/api/v1/scene/latest").json()
        assert latest["scene"]["origin"] == "live:static_obstacle"
        assert latest["scene"]["live"]["seed"] == 2
        assert "ground_truth" not in latest["scene"]
        events = client.get("/api/v1/live/events").json()
        assert events["events"] and events["latest_sequence"] >= 1
        camera = client.get("/api/v1/live/camera")
        assert camera.status_code == 200 and camera.headers["content-type"] == "image/png"
        assert client.get("/api/v1/carla/status").json()["status"] == "CONNECTED"
        assert client.post("/api/v1/live/pause").json()["state"] == "PAUSED"
        assert client.post("/api/v1/live/resume").json()["state"] == "RUNNING"
        assert client.post("/api/v1/live/start", json={}).status_code == 409
        stopped = client.post("/api/v1/live/stop")
        assert stopped.json()["state"] == "STOPPED"
        assert harness.world.actors == {}

    def test_the_scene_channel_reports_skipped_frames(
        self, client: TestClient, harness: Harness
    ) -> None:
        client.post("/api/v1/live/start", json={"scenario_id": "random_urban_traffic"})
        harness.wait_frames(3)
        with client.websocket_connect("/ws/scene") as ws:
            hello = ws.receive_json()
            assert hello["type"] == "hello" and hello["data"]["has_scene"]
            first = ws.receive_json()
            assert first["type"] == "scene" and first["skipped"] == 0
            harness.wait_frames(first["sequence"] + 6)
            later = ws.receive_json()
            assert later["type"] == "scene"
            assert later["sequence"] > first["sequence"]
            assert later["skipped"] >= 0
        client.post("/api/v1/live/stop")
