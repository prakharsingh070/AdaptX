"""Live scenario catalogue, resolution and the anchored scenario manager."""

from __future__ import annotations

import pytest

from adaptx.carla.session import CarlaSimulationSession
from adaptx.config.settings import CarlaSettings
from adaptx.live.models import ActorKind, LiveMotion, LiveScenarioDefinition, SpawnEvent
from adaptx.live.scenarios import (
    LIVE_CATALOGUE,
    LiveScenarioError,
    LiveScenarioManager,
    load_live,
    resolve_live,
    summaries,
)
from tests.fixtures.fake_carla import FakeCarlaModule, FakeWorld


class TestCatalogue:
    def test_the_required_scenarios_exist_and_validate(self) -> None:
        assert set(LIVE_CATALOGUE) == {
            "random_urban_traffic",
            "pedestrian_crossing",
            "vehicle_cut_in",
            "cyclist_crossing",
            "static_obstacle",
            "mixed_obstacles",
            "pedestrian_roadside",
            "multiple_vehicles",
        }
        for definition in LIVE_CATALOGUE.values():
            LiveScenarioDefinition.model_validate(definition.model_dump())
        assert {s.scenario_id for s in summaries()} == set(LIVE_CATALOGUE)

    def test_an_unknown_id_is_an_error(self) -> None:
        with pytest.raises(LiveScenarioError):
            load_live("no_such_scene")

    def test_the_obstacle_demo_makes_the_obstacle_leave(self) -> None:
        obstacle = load_live("static_obstacle").events[0]
        assert obstacle.kind is ActorKind.OBSTACLE
        assert obstacle.lifetime_s is not None, "the demo needs the obstacle to leave"
        assert obstacle.forward_m > 20.0, "far enough ahead to be seen before it matters"

    def test_definitions_refuse_duplicate_actor_ids(self) -> None:
        event = SpawnEvent(
            actor_id="x", kind=ActorKind.OBSTACLE, blueprint="b", at_s=0.0, forward_m=1
        )
        with pytest.raises(ValueError, match="unique"):
            LiveScenarioDefinition(
                scenario_id="d", name="d", description="d", default_seed=0, events=[event, event]
            )


class TestResolution:
    def test_the_same_seed_draws_the_same_scene_and_another_seed_differs(self) -> None:
        definition = load_live("mixed_obstacles")
        one = resolve_live(definition, 42, spawn_point_count=40)
        two = resolve_live(definition, 42, spawn_point_count=40)
        other = resolve_live(definition, 43, spawn_point_count=40)
        assert [s.base for s in one.spawns] == [s.base for s in two.spawns]
        assert one.traffic == two.traffic
        assert one.traffic != other.traffic or [s.base for s in one.spawns] != [
            s.base for s in other.spawns
        ]
        assert len(one.traffic) == definition.traffic_vehicles
        assert len({index for _, index in one.traffic}) == len(one.traffic), "distinct points"

    def test_jitter_stays_within_bounds(self) -> None:
        definition = load_live("pedestrian_crossing")
        for seed in range(20):
            spawn = resolve_live(definition, seed, spawn_point_count=5).spawns[0]
            assert abs(spawn.base[0] - spawn.event.forward_m) <= spawn.event.jitter_m
            assert abs(spawn.base[1] - spawn.event.left_m) <= spawn.event.jitter_m

    def test_motion_is_closed_form(self) -> None:
        segment = LiveMotion(start_s=1.0, stop_s=3.0, forward_mps=2.0, left_mps=-1.0)
        assert segment.displacement_at(0.5) == (0.0, 0.0)
        assert segment.displacement_at(2.0) == (2.0, -1.0)
        assert segment.displacement_at(10.0) == (4.0, -2.0)


def open_session(world: FakeWorld) -> CarlaSimulationSession:
    session = CarlaSimulationSession(
        CarlaSettings(enabled=True, ego_spawn_index=0, sensor_timeout_s=1.0),
        carla_module=FakeCarlaModule(world),
        drive_ego=True,
    )
    session.open()
    session.step()  # a real server reports poses only after the first tick
    return session


class TestManager:
    def test_actors_appear_on_time_anchored_to_the_ego_pose_then_and_leave(self) -> None:
        world = FakeWorld()
        session = open_session(world)
        try:
            definition = load_live("static_obstacle")
            resolved = resolve_live(definition, 1, spawn_point_count=2)
            manager = LiveScenarioManager(resolved, session)
            assert manager.advance(0.5) == []
            notes = manager.advance(1.0)
            assert notes and "appeared" in notes[0]
            assert len(manager.active) == 1
            obstacle = world.actors[manager.active[0].simulator_actor_id]
            anchored_x = obstacle.get_transform().location.x
            # Drive the ego forward: the obstacle must NOT move with it.
            ego = next(a for a in world.actors.values() if a.type_id == "vehicle.tesla.model3")
            ego.set_transform(
                type(ego.get_transform())(
                    type(ego.get_transform().location)(20.0, 0.0, 0.0),
                    ego.get_transform().rotation,
                )
            )
            manager.advance(5.0)
            assert obstacle.get_transform().location.x == pytest.approx(anchored_x)
            assert obstacle.simulate_physics is False, "placed, not simulated"
            manager.advance(1.0 + resolved.spawns[0].event.lifetime_s)
            assert manager.active == []
            assert obstacle.id not in world.actors, "destroyed when its lifetime ends"
        finally:
            session.close()
        assert world.actors == {}

    def test_scripted_motion_moves_the_actor_in_the_anchor_frame(self) -> None:
        world = FakeWorld()
        session = open_session(world)
        try:
            definition = load_live("cyclist_crossing")
            manager = LiveScenarioManager(resolve_live(definition, 0, spawn_point_count=2), session)
            manager.advance(definition.events[0].at_s)
            actor = world.actors[manager.active[0].simulator_actor_id]
            y0 = actor._transform.location.y  # set pose; the fake reports it after a tick
            manager.advance(definition.events[0].at_s + 2.0)
            y1 = actor._transform.location.y
            # ADAPT-X +left is CARLA -y: a leftward crossing decreases CARLA y.
            assert y1 < y0
        finally:
            session.close()

    def test_traffic_is_best_effort_and_autopiloted(self) -> None:
        world = FakeWorld()
        world.get_map().spawn_points.extend(
            type(world.get_map().spawn_points[0])() for _ in range(6)
        )
        session = open_session(world)
        try:
            definition = load_live("random_urban_traffic")
            resolved = resolve_live(definition, 3, spawn_point_count=8)
            manager = LiveScenarioManager(resolved, session)
            spawned = manager.spawn_traffic()
            assert 0 < spawned <= definition.traffic_vehicles
            assert manager.traffic_count == spawned
            traffic = [a for a in world.actors.values() if a.autopilot is not None]
            assert len(traffic) == spawned
            assert all(a.autopilot[0] for a in traffic)
            client = session._carla.clients[-1]  # type: ignore[union-attr]
            assert client.traffic_manager is not None
            assert client.traffic_manager.synchronous is True
            assert client.traffic_manager.seed == session._settings.seed  # type: ignore[attr-defined]
        finally:
            session.close()
        assert world.actors == {}
