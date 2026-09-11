"""CARLA simulation session lifecycle tests (Phase 9).

CARLA is not installed here, so the session is driven against
:mod:`tests.fixtures.fake_carla` - a stand-in implementing the small slice of
the simulator API the session actually touches.

What that can and cannot prove
------------------------------
It **can** prove the lifecycle logic: spawn order, cleanup on failure, actors
destroyed, world settings restored, ticks matched to sensor frames, timestamps
advancing by exactly one timestep. That is the code most likely to leak an
actor or leave a server wedged in synchronous mode, and none of it needs a real
simulator to be wrong.

It **cannot** prove anything about CARLA itself - not its API compatibility,
not its performance, not its sensor model. Nothing measured here is a CARLA
measurement. The live smoke test in ``tests/integration/test_carla_live.py``
is the only thing that touches a real server, and it skips when there is none.
"""

from __future__ import annotations

from itertools import pairwise

import pytest

from adaptx.carla.session import LIDAR_BLUEPRINT, CarlaSimulationSession
from adaptx.config.settings import CarlaSettings
from adaptx.core.exceptions import SimulatorUnavailableError
from adaptx.models.common import DataSource
from adaptx.models.system import SimulationState
from tests.fixtures.fake_carla import FakeCarlaModule, FakeSensor, FakeWorld


def settings(**overrides: object) -> CarlaSettings:
    """CARLA settings with the simulator enabled, for session tests."""
    base: dict[str, object] = {
        "enabled": True,
        "fixed_delta_seconds": 0.05,
        "lidar_rotation_frequency_hz": 20.0,
        "sensor_timeout_s": 1.0,
    }
    base.update(overrides)
    return CarlaSettings(**base)  # type: ignore[arg-type]


def session(world: FakeWorld | None = None, **overrides: object) -> CarlaSimulationSession:
    """A session bound to a fake simulator."""
    module = FakeCarlaModule(world if world is not None else FakeWorld())
    return CarlaSimulationSession(settings(**overrides), carla_module=module)


class TestLifecycleTransitions:
    def test_a_new_session_is_idle(self) -> None:
        assert session().state is SimulationState.IDLE

    def test_opening_reaches_ready_without_stepping(self) -> None:
        instance = session()
        instance.open()
        assert instance.state is SimulationState.READY
        assert instance.is_running is False
        instance.close()

    def test_stepping_moves_to_running(self) -> None:
        instance = session()
        instance.open()
        instance.step()
        assert instance.state is SimulationState.RUNNING
        assert instance.is_running is True
        instance.close()

    def test_closing_reaches_stopped(self) -> None:
        instance = session()
        instance.open()
        instance.close()
        assert instance.state is SimulationState.STOPPED

    def test_the_session_works_as_a_context_manager(self) -> None:
        world = FakeWorld()
        with session(world) as instance:
            instance.step()
        assert world.actors == {}

    def test_opening_twice_is_refused_rather_than_leaking_a_second_world(self) -> None:
        instance = session()
        instance.open()
        with pytest.raises(SimulatorUnavailableError, match="already open"):
            instance.open()
        instance.close()

    def test_closing_twice_is_harmless(self) -> None:
        instance = session()
        instance.open()
        instance.close()
        instance.close()
        assert instance.state is SimulationState.STOPPED

    def test_closing_without_opening_is_harmless(self) -> None:
        session().close()

    def test_stepping_before_opening_is_refused(self) -> None:
        with pytest.raises(SimulatorUnavailableError, match="not open"):
            session().step()

    def test_a_disabled_simulator_refuses_to_open(self) -> None:
        module = FakeCarlaModule()
        instance = CarlaSimulationSession(CarlaSettings(enabled=False), carla_module=module)
        with pytest.raises(SimulatorUnavailableError, match="disabled"):
            instance.open()


class TestDeterministicConfiguration:
    def test_synchronous_mode_and_the_timestep_are_applied(self) -> None:
        world = FakeWorld()
        instance = session(world)
        instance.open()
        applied = world.applied_settings[0]
        assert applied.synchronous_mode is True
        assert applied.fixed_delta_seconds == pytest.approx(0.05)
        instance.close()

    def test_world_settings_are_restored_on_close(self) -> None:
        """A server left in synchronous mode blocks on a client that has gone."""
        world = FakeWorld()
        original = world.get_settings()
        instance = session(world)
        instance.open()
        instance.close()
        assert world.settings.synchronous_mode == original.synchronous_mode
        assert world.settings.fixed_delta_seconds == original.fixed_delta_seconds

    def test_settings_are_restored_even_when_setup_fails_later(self) -> None:
        world = FakeWorld()
        world.refuse_spawn.add(LIDAR_BLUEPRINT)
        instance = session(world)
        with pytest.raises(SimulatorUnavailableError):
            instance.open()
        assert world.settings.synchronous_mode is False

    def test_a_configured_town_is_loaded(self) -> None:
        module = FakeCarlaModule()
        instance = CarlaSimulationSession(settings(town="Town05"), carla_module=module)
        instance.open()
        assert module.clients[0].loaded == ["Town05"]
        instance.close()


class TestActorCleanup:
    def test_every_spawned_actor_is_destroyed_on_close(self) -> None:
        world = FakeWorld()
        instance = session(world)
        instance.open()
        instance.spawn_ahead_of_ego("vehicle.audi.tt", forward_m=25.0)
        assert len(world.actors) == 3  # ego, lidar, target
        instance.close()
        assert world.actors == {}

    def test_the_sensor_is_stopped_before_it_is_destroyed(self) -> None:
        world = FakeWorld()
        instance = session(world)
        instance.open()
        sensor = next(a for a in world.actors.values() if isinstance(a, FakeSensor))
        instance.close()
        assert sensor.stopped is True
        assert sensor.destroyed is True

    def test_a_failed_ego_spawn_leaves_nothing_behind(self) -> None:
        world = FakeWorld()
        world.refuse_spawn.add("vehicle.tesla.model3")
        instance = session(world)
        with pytest.raises(SimulatorUnavailableError, match="ego vehicle"):
            instance.open()
        assert world.actors == {}
        assert instance.state is SimulationState.ERROR

    def test_a_failed_sensor_spawn_destroys_the_ego(self) -> None:
        """Partial setup must not strand the actors it did manage to create."""
        world = FakeWorld()
        world.refuse_spawn.add(LIDAR_BLUEPRINT)
        instance = session(world)
        with pytest.raises(SimulatorUnavailableError, match="LiDAR"):
            instance.open()
        assert world.actors == {}

    def test_a_missing_blueprint_is_reported_by_name(self) -> None:
        instance = session(FakeWorld(known_blueprints={"sensor.lidar.ray_cast"}))
        with pytest.raises(SimulatorUnavailableError, match=r"vehicle\.tesla\.model3"):
            instance.open()

    def test_a_map_without_spawn_points_is_reported(self) -> None:
        instance = session(FakeWorld(spawn_points=False))
        with pytest.raises(SimulatorUnavailableError, match="spawn points"):
            instance.open()

    def test_cleanup_continues_when_one_actor_refuses_to_die(self) -> None:
        """One stubborn actor must not strand the rest."""
        world = FakeWorld()
        instance = session(world)
        instance.open()
        target = instance.spawn_ahead_of_ego("vehicle.audi.tt", forward_m=20.0)

        def explode() -> None:
            raise RuntimeError("actor already destroyed on the server")

        target.destroy = explode  # type: ignore[method-assign]
        instance.close()

        # The stubborn one is still registered; everything else is gone.
        assert list(world.actors) == [target.id]


class TestFrameProgression:
    def test_a_step_returns_a_frame_labelled_simulation(self) -> None:
        instance = session()
        instance.open()
        frame = instance.step()
        assert frame.source is DataSource.SIMULATION
        assert frame.point_count > 0
        instance.close()

    def test_the_frame_id_is_the_simulator_frame_number(self) -> None:
        world = FakeWorld()
        instance = session(world)
        instance.open()
        frame = instance.step()
        assert frame.frame_id == world.get_snapshot().frame
        instance.close()

    def test_successive_frames_advance_by_exactly_one_timestep(self) -> None:
        """Phase 4 measures velocity from this interval, so it must be exact."""
        instance = session()
        instance.open()
        frames = [instance.step() for _ in range(3)]
        instance.close()

        deltas = [
            (later.timestamp - earlier.timestamp).total_seconds()
            for earlier, later in pairwise(frames)
        ]
        assert deltas == pytest.approx([0.05, 0.05], abs=1e-9)

    def test_frame_ids_increase(self) -> None:
        instance = session()
        instance.open()
        ids = [instance.step().frame_id for _ in range(3)]
        instance.close()
        assert ids == sorted(ids)
        assert len(set(ids)) == 3

    def test_two_runs_of_the_same_scenario_produce_the_same_frames(self) -> None:
        """Determinism: no wall clock, no randomness in the path from tick to frame."""

        def run() -> list[tuple[int, float, int]]:
            instance = session()
            instance.open()
            out = []
            for _ in range(3):
                frame = instance.step()
                out.append((frame.frame_id, frame.timestamp.timestamp(), frame.point_count))
            instance.close()
            return out

        assert run() == run()

    def test_a_sensor_that_never_delivers_times_out_loudly(self) -> None:
        world = FakeWorld()
        world.deliver_frames = False
        instance = session(world, sensor_timeout_s=0.05)
        instance.open()
        with pytest.raises(SimulatorUnavailableError, match="no LiDAR frame arrived"):
            instance.step()
        assert instance.state is SimulationState.ERROR
        instance.close()

    def test_a_failing_tick_is_reported_and_not_swallowed(self) -> None:
        world = FakeWorld()
        instance = session(world)
        instance.open()
        world.tick_error = RuntimeError("server dropped the connection")
        with pytest.raises(SimulatorUnavailableError, match="tick failed"):
            instance.step()
        assert instance.state is SimulationState.ERROR
        instance.close()

    def test_a_stale_queued_frame_is_discarded_rather_than_returned(self) -> None:
        """A stale scan would pair this tick's ground truth with old points."""
        world = FakeWorld()
        instance = session(world)
        instance.open()
        sensor = next(a for a in world.actors.values() if isinstance(a, FakeSensor))
        sensor.deliver(frame=1, timestamp=0.0)  # far older than any tick

        frame = instance.step()
        assert frame.frame_id == world.get_snapshot().frame
        instance.close()


class TestSessionStatus:
    def test_an_idle_session_reports_no_simulator_detail(self) -> None:
        status = session().status()
        assert status.state is SimulationState.IDLE
        assert status.frames_stepped == 0
        assert status.simulation_frame is None
        assert status.last_point_count is None

    def test_status_reflects_a_stepped_frame(self) -> None:
        instance = session()
        instance.open()
        frame = instance.step()
        status = instance.status()
        instance.close()

        assert status.state is SimulationState.RUNNING
        assert status.frames_stepped == 1
        assert status.simulation_frame == frame.frame_id
        assert status.last_point_count == frame.point_count
        assert status.map_name == "FakeTown"

    def test_status_carries_counts_and_never_point_data(self) -> None:
        instance = session()
        instance.open()
        instance.step()
        payload = instance.status().model_dump(mode="json")
        instance.close()

        for forbidden in ("points", "raw_data", "cloud", "actors_detail"):
            assert forbidden not in payload
        assert isinstance(payload["last_point_count"], int)


class TestScenarioPlacement:
    def test_an_ego_relative_spawn_lands_where_asked(self) -> None:
        """The fake's ego sits at the world origin facing +x, so an ADAPT-X
        offset of 30 m ahead and 4 m left is CARLA world (30, -4)."""
        world = FakeWorld()
        instance = session(world)
        instance.open()
        target = instance.spawn_ahead_of_ego(
            "vehicle.audi.tt", forward_m=30.0, left_m=4.0, up_m=0.5
        )
        location = target.get_transform().location
        instance.close()

        assert location.x == pytest.approx(30.0)
        assert location.y == pytest.approx(-4.0)

    def test_moving_an_actor_updates_its_pose(self) -> None:
        world = FakeWorld()
        instance = session(world)
        instance.open()
        target = instance.spawn_ahead_of_ego("vehicle.audi.tt", forward_m=40.0)
        instance.place_ahead_of_ego(target, forward_m=20.0)
        location = target.get_transform().location
        instance.close()
        assert location.x == pytest.approx(20.0)

    def test_spawning_before_an_ego_exists_is_refused(self) -> None:
        with pytest.raises(SimulatorUnavailableError, match="no ego"):
            session()._world_position_for(10.0, 0.0, 0.0)
