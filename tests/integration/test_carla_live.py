"""Live CARLA smoke test (Phase 9).

The only test in the suite that touches a real simulator. It is deselected by
default (``-m "not carla"`` in ``pyproject.toml``) **and** skips itself when
the package or a server is missing, so it can never turn an absent optional
dependency into a red suite.

Run it deliberately, against a running server::

    ./CarlaUE4.sh -RenderOffScreen        # in the CARLA install
    pytest -m carla

What it proves, and what it does not
------------------------------------
It proves the adapter works against the real API: connecting, applying
deterministic settings, spawning, receiving genuine LiDAR, converting it, and
cleaning up. That is the one thing a stand-in cannot prove.

It proves **nothing about accuracy**. Detections here are unvalidated, and no
figure it prints may be reported as a perception-accuracy result. Comparing
perception against the ground truth it records is Phase 11.
"""

from __future__ import annotations

from itertools import pairwise
from typing import Any

import pytest

from adaptx.carla.client import carla_package_available
from adaptx.carla.session import CarlaSimulationSession
from adaptx.config.settings import Settings
from adaptx.core.exceptions import SimulatorUnavailableError
from adaptx.core.lifecycle import build_context
from adaptx.models.common import DataSource
from adaptx.models.system import SimulationState

pytestmark = pytest.mark.carla


def live_settings() -> Settings:
    """Settings pointed at a real server, with a short run.

    The ``carla`` section is read from the environment (``ADAPTX_CARLA__*``)
    and then switched on: host, port and - on Town10HD_Opt - the spawn point
    are properties of the server being tested, not of the test. The catalogue's
    placements are ego-relative and on that map are on the road only from
    spawn point 1, so run with ``ADAPTX_CARLA__EGO_SPAWN_INDEX=1`` there
    (Experiment 011).
    """
    settings = Settings(
        app={"environment": "development", "debug": True},
        logging={"level": "WARNING"},
    )
    settings.carla.enabled = True
    settings.carla.sensor_timeout_s = 20.0
    return settings


def require_live_server() -> Settings:
    """Skip unless a CARLA server actually answers.

    Skipping is the honest outcome here: an absent optional dependency is not
    a failure, and reporting it as one would train everyone to ignore the
    suite.
    """
    if not carla_package_available():
        pytest.skip("the optional CARLA package is not installed")

    settings = live_settings()
    session = CarlaSimulationSession(settings.carla)
    try:
        session.open()
    except SimulatorUnavailableError as exc:
        pytest.skip(f"no CARLA server reachable: {exc.message}")
    finally:
        session.close()
    return settings


class TestLiveCarla:
    def test_a_live_session_opens_and_cleans_up(self) -> None:
        settings = require_live_server()
        session = CarlaSimulationSession(settings.carla)
        session.open()
        assert session.state is SimulationState.READY
        assert session.ego_actor_id is not None
        session.close()
        assert session.state is SimulationState.STOPPED

    def test_a_live_frame_is_labelled_simulation(self) -> None:
        settings = require_live_server()
        with CarlaSimulationSession(settings.carla) as session:
            frame = session.step()
        assert frame.source is DataSource.SIMULATION
        assert frame.point_count > 0

    def test_live_frames_advance_by_exactly_the_timestep(self) -> None:
        settings = require_live_server()
        with CarlaSimulationSession(settings.carla) as session:
            frames = [session.step() for _ in range(3)]

        deltas = [
            (later.timestamp - earlier.timestamp).total_seconds()
            for earlier, later in pairwise(frames)
        ]
        expected = settings.carla.fixed_delta_seconds
        assert deltas == pytest.approx([expected, expected], abs=1e-6)

    def test_live_ground_truth_is_recorded_beside_the_frame(self) -> None:
        settings = require_live_server()
        with CarlaSimulationSession(settings.carla) as session:
            frame = session.step()
            truth = session.ground_truth()

        assert truth.frame_id == frame.frame_id
        assert truth.source is DataSource.SIMULATION
        assert truth.ego_actor_id is not None

    def test_a_live_catalogue_scenario_completes(self) -> None:
        """The whole Phase 9 claim, against a real simulator - now as a scenario.

        Retargeted in Phase 10 when the smoke run became ``vehicle_approach``.
        """
        from adaptx.scenarios import load, run_scenario

        settings = require_live_server()
        result = run_scenario(
            load("vehicle_approach"), settings=settings, context=build_context(settings)
        )

        assert result.completed, result.error
        assert result.frame_count == result.planned_frame_count
        assert result.timestamps_are_monotonic()
        assert all(record.point_count > 0 for record in result.frames)
        assert all(
            record.pipeline is not None and record.pipeline.adaptive_cells > 0
            for record in result.frames
        )

    def test_every_catalogue_scenario_runs_live(self) -> None:
        """Phase 10 against a real simulator: each definition, start to finish.

        The blueprints a scenario asks for must exist on the server; a missing
        one is a real finding about the catalogue, not a test bug, and is
        reported as such through the FAILED result.
        """
        from adaptx.scenarios import load, run_scenario, scenario_ids

        settings = require_live_server()
        for scenario_id in scenario_ids():
            result = run_scenario(load(scenario_id), settings=settings, process=False)
            assert result.completed, f"{scenario_id}: {result.error}"
            assert result.frame_count == result.planned_frame_count
            assert len(result.ground_truth) == result.frame_count
            assert all(truth.others() for truth in result.ground_truth), scenario_id

    def test_the_live_run_leaves_no_actors_behind(self) -> None:
        """A leaked actor persists in the server for every later run."""
        settings = require_live_server()
        session = CarlaSimulationSession(settings.carla)
        session.open()
        world = session._require_world()
        before = len(world.get_actors())
        session.spawn_ahead_of_ego("vehicle.audi.tt", forward_m=25.0)
        session.step()
        session.close()

        remaining = CarlaSimulationSession(settings.carla)
        remaining.open()
        after = len(remaining._require_world().get_actors())
        remaining.close()
        assert after <= before


class TestLiveLoop:
    """The post-Phase-12 live loop against the real server.

    What it proves: a driven ego moves under the controller, the scripted
    obstacle is a real actor the real LiDAR sees, the pipeline's own outputs
    make the controller slow and hold, and the session leaves nothing behind.
    What it does not prove: anything about the quality of the perception -
    Experiments 011/012 measured that, and it is not flattering.
    """

    def _run(self, scenario: str, seconds: float, *, seed: int = 42) -> tuple[Any, Any]:
        import time

        settings = require_live_server()
        settings.lidar.ground_enabled = True
        settings.live.camera_enabled = True
        context = build_context(settings)
        live = context.live
        trace: list[Any] = []
        started = time.perf_counter()
        live.start(scenario, seed)
        try:
            while time.perf_counter() - started < seconds:
                time.sleep(0.25)
                status = live.status()
                trace.append(status)
                if status.state.value != "RUNNING":
                    break
        finally:
            final = live.stop()
        return final, trace

    def test_the_ego_drives_and_the_loop_measures_itself(self) -> None:
        final, trace = self._run("random_urban_traffic", 20.0, seed=1)
        assert final.state.value == "STOPPED", final.detail
        speeds = [s.ego_speed_mps for s in trace if s.ego_speed_mps is not None]
        assert max(speeds) > 1.0, "the ego never moved"
        last = next(s for s in reversed(trace) if s.timing is not None)
        assert last.timing.loop_ms > 0 and last.timing.pipeline_ms > 0
        assert last.frames_processed == last.snapshots_published > 10
        assert last.traffic_count >= 1, "the Traffic Manager spawned traffic"
        assert last.camera_available
        assert final.collision_count == 0

    def test_the_obstacle_stop_demo_holds_before_the_parked_car(self) -> None:
        """Ego drives, sees the parked car, slows, holds short of it. Live."""
        final, trace = self._run("static_obstacle", 75.0)
        assert final.state.value == "STOPPED", final.detail
        states = [s.control.state.value for s in trace if s.control is not None]
        assert "SLOWING" in states or "HOLDING" in states, states
        held = [
            s
            for s in trace
            if s.control is not None
            and s.control.state.value == "STOPPED"
            and s.control.nearest_in_path_m is not None
        ]
        assert held, "the ego never came to a hold on an in-path object"
        assert held[0].control.nearest_in_path_m < 9.0, "held short of the in-path object"
        assert final.collision_count == 0, "stopped short of the car, no contact"

    def test_the_live_session_leaves_no_actors_and_restores_the_world(self) -> None:
        settings = require_live_server()
        probe = CarlaSimulationSession(settings.carla)
        probe.open()
        world = probe._require_world()
        before = len(world.get_actors())
        probe.close()
        original = world.get_settings().synchronous_mode  # after the probe restored it

        final, _ = self._run("mixed_obstacles", 6.0)
        assert final.state.value == "STOPPED"

        after = CarlaSimulationSession(settings.carla)
        after.open()
        world = after._require_world()
        assert len(world.get_actors()) <= before + 2  # this probe's own ego + lidar
        after.close()
        assert world.get_settings().synchronous_mode == original


class TestLivePerception:
    """Live perception upgrade acceptance: classes, distances and identity, on the server.

    Reads the published snapshots only - the same records the dashboard draws.
    Ground truth is not consulted; what is asserted is that the pipeline
    labelled and measured the scripted actors the way Experiment 015 recorded.
    """

    def _records(self, scenario: str, seconds: float) -> list[Any]:
        import time

        settings = require_live_server()
        settings.lidar.ground_enabled = True
        settings.live.camera_enabled = False
        context = build_context(settings)
        live = context.live
        seen: set[int] = set()
        frames: list[Any] = []
        started = time.perf_counter()
        live.start(scenario, 42)
        try:
            while time.perf_counter() - started < seconds:
                time.sleep(0.05)
                sequence, snapshot = context.scene.latest()
                if snapshot is None or sequence in seen:
                    continue
                seen.add(sequence)
                frames.append(snapshot)
                if live.status().state.value != "RUNNING":
                    break
        finally:
            live.stop()
        return frames

    def test_the_parked_car_is_a_vehicle_with_a_stable_id_and_a_real_distance(self) -> None:
        frames = self._records("static_obstacle", 60.0)
        # The in-path object the controller held on: one track id for the approach.
        in_path = [
            r
            for s in frames
            for r in s.objects
            if r.in_ego_path and r.distance_m is not None and r.distance_m < 20.0
        ]
        assert in_path, "nothing in the path was ever closer than 20 m"
        classes = {r.object_class.value for r in in_path}
        assert "vehicle" in classes, classes
        vehicle_ids = [r.track_id for r in in_path if r.object_class.value == "vehicle"]
        assert len(set(vehicle_ids)) <= 2, f"identity churn on the parked car: {set(vehicle_ids)}"
        distances = [r.distance_m for r in in_path if r.track_id == vehicle_ids[-1]]
        assert min(distances) < 9.0, "the ego held short of it"
        assert all(r.longitudinal_distance_m > 0 for r in in_path), "ahead of the ego"
        now = [r for r in in_path if r.path_relation.value == "IN_PATH"]
        assert now, "the car was IN_PATH at some point"
        assert all(abs(r.lateral_distance_m) <= 1.8 for r in now), "IN_PATH is in the corridor"

    def test_the_crossing_walker_is_a_pedestrian_whose_distance_and_path_change(self) -> None:
        frames = self._records("pedestrian_crossing", 55.0)
        walkers = [
            r
            for s in frames
            for r in s.objects
            if r.object_class.value == "pedestrian" and r.tracking_state.value == "confirmed"
        ]
        assert walkers, "no confirmed pedestrian track"
        by_id: dict[int, list[Any]] = {}
        for r in walkers:
            by_id.setdefault(r.track_id, []).append(r)
        longest = max(by_id.values(), key=len)
        assert len(longest) >= 20, "the walker was tracked for at least a second"
        relations = {r.path_relation.value for r in longest}
        assert relations & {"IN_PATH", "CROSSING"}, relations
        lateral = [r.lateral_distance_m for r in longest]
        assert max(lateral) - min(lateral) > 1.0, "its lateral position changed as it crossed"
        speeds = [r.speed_mps for r in longest if r.speed_mps is not None]
        assert speeds, "a tracked speed exists once velocity is measured"
        assert all(r.confidence is not None for r in longest), "a classified track has a fit score"
