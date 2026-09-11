"""Scenario runner tests (Phase 10).

Driven against :mod:`tests.fixtures.fake_carla`, so nothing here is a CARLA
measurement. What these tests can prove is the runner's own logic: the
lifecycle, that the seed is consumed and reproducible, that actors land where
the script says, that cleanup runs on every failure path, and that two runs
share nothing.

The fake's ego sits at the world origin facing +x, so an ego-relative offset
of (forward, left) in ADAPT-X terms lands at CARLA world ``(forward, -left)``.
That is the one piece of fake-specific arithmetic the placement tests rely on.
"""

from __future__ import annotations

from itertools import pairwise

import pytest

from adaptx.carla.session import CarlaSimulationSession
from adaptx.config.settings import CarlaSettings, Settings
from adaptx.models.common import DataSource, ObjectClass
from adaptx.scenarios.models import (
    MotionSegment,
    Placement,
    ScenarioActor,
    ScenarioDefinition,
    ScenarioState,
)
from adaptx.scenarios.result import StageCounts
from adaptx.scenarios.runner import ScenarioError, ScenarioRunner, resolve
from tests.fixtures.fake_carla import FakeCarlaModule, FakeWorld

TIMESTEP = 0.05


def settings() -> Settings:
    return Settings(
        app={"environment": "development", "debug": True},
        logging={"level": "WARNING"},
        carla=CarlaSettings(
            enabled=True, fixed_delta_seconds=TIMESTEP, sensor_timeout_s=1.0
        ).model_dump(),
    )


def simulator(world: FakeWorld | None = None) -> CarlaSimulationSession:
    return CarlaSimulationSession(
        settings().carla, carla_module=FakeCarlaModule(world if world is not None else FakeWorld())
    )


def actor(
    actor_id: str = "target",
    *,
    forward_m: float = 20.0,
    left_m: float = 0.0,
    motion: list[MotionSegment] | None = None,
    jitter: float = 0.0,
    blueprint: str = "vehicle.audi.tt",
) -> ScenarioActor:
    return ScenarioActor(
        actor_id=actor_id,
        blueprint=blueprint,
        object_class=ObjectClass.VEHICLE,
        placement=Placement(forward_m=forward_m, left_m=left_m),
        placement_jitter_m=jitter,
        motion=motion or [],
    )


def definition(
    scenario_id: str = "runner_scenario",
    *,
    seed: int = 7,
    duration_s: float = 0.5,
    actors: list[ScenarioActor] | None = None,
) -> ScenarioDefinition:
    return ScenarioDefinition(
        scenario_id=scenario_id,
        name="Runner scenario",
        description="For runner tests.",
        seed=seed,
        duration_s=duration_s,
        fixed_delta_seconds=TIMESTEP,
        actors=[actor()] if actors is None else actors,
    )


def counting_processor(seen: list[object]) -> object:
    """A processor that records what it was handed and returns fixed counts."""

    def process(frame: object) -> StageCounts:
        seen.append(frame)
        return StageCounts(
            processed_points=1,
            detections=0,
            tracks=0,
            trajectories=0,
            risk_level="unknown",
            adaptive_cells=1,
            fixed_cells=1,
        )

    return process


class TestLifecycle:
    def test_a_new_runner_is_created(self) -> None:
        assert ScenarioRunner(definition(), simulator=simulator()).state is ScenarioState.CREATED

    def test_a_successful_run_ends_completed(self) -> None:
        runner = ScenarioRunner(definition(), simulator=simulator(), settings=settings())
        result = runner.run()
        assert runner.state is ScenarioState.COMPLETED
        assert result.state is ScenarioState.COMPLETED
        assert result.completed
        assert result.error is None

    def test_the_result_carries_the_resolved_scenario(self) -> None:
        result = ScenarioRunner(definition(), simulator=simulator(), settings=settings()).run()
        assert result.resolved.definition.scenario_id == "runner_scenario"
        assert result.resolved is not None

    def test_a_runner_runs_exactly_once(self) -> None:
        """A second run would inherit the first's state; refuse it."""
        runner = ScenarioRunner(definition(), simulator=simulator(), settings=settings())
        runner.run()
        with pytest.raises(ScenarioError, match="exactly once"):
            runner.run()

    def test_a_malformed_definition_is_refused_before_any_simulator_contact(self) -> None:
        """Validation failure is a misuse, raised explicitly - not a FAILED result."""
        bad = definition().model_copy(update={"duration_s": 0.01})  # bypasses validation
        world = FakeWorld()
        runner = ScenarioRunner(bad, simulator=simulator(world), settings=settings())

        with pytest.raises(ScenarioError, match="not valid"):
            runner.run()
        assert runner.state is ScenarioState.FAILED
        assert world.applied_settings == [], "the simulator must never have been opened"

    def test_a_failed_spawn_yields_a_failed_result_not_an_exception(self) -> None:
        world = FakeWorld()
        world.refuse_spawn.add("vehicle.audi.tt")
        runner = ScenarioRunner(definition(), simulator=simulator(world), settings=settings())
        result = runner.run()

        assert runner.state is ScenarioState.FAILED
        assert result.state is ScenarioState.FAILED
        assert result.error is not None
        assert "vehicle.audi.tt" in result.error
        assert result.frame_count == 0

    def test_a_failure_mid_run_keeps_the_frames_stepped_so_far(self) -> None:
        world = FakeWorld()
        runner = ScenarioRunner(
            definition(duration_s=1.0), simulator=simulator(world), settings=settings()
        )
        # Break the simulator after a few frames by making the tick raise.
        original_tick = world.tick

        def failing_tick() -> int:
            if world.get_snapshot().frame >= 103:
                raise RuntimeError("server went away")
            return original_tick()

        world.tick = failing_tick  # type: ignore[method-assign]
        result = runner.run()

        assert result.state is ScenarioState.FAILED
        assert 0 < result.frame_count < result.planned_frame_count
        assert "server went away" in (result.error or "")

    def test_a_processor_error_is_a_failed_run_after_cleanup(self) -> None:
        world = FakeWorld()

        def explode(frame: object) -> StageCounts:
            raise ValueError("processing blew up")

        runner = ScenarioRunner(
            definition(), simulator=simulator(world), processor=explode, settings=settings()
        )
        result = runner.run()

        assert result.state is ScenarioState.FAILED
        assert "processing blew up" in (result.error or "")
        assert world.actors == {}


class TestCleanup:
    def test_a_completed_run_leaves_no_actors(self) -> None:
        world = FakeWorld()
        ScenarioRunner(definition(), simulator=simulator(world), settings=settings()).run()
        assert world.actors == {}

    def test_a_failed_spawn_leaves_no_actors(self) -> None:
        world = FakeWorld()
        world.refuse_spawn.add("vehicle.audi.tt")
        ScenarioRunner(definition(), simulator=simulator(world), settings=settings()).run()
        assert world.actors == {}

    def test_a_second_actor_failing_destroys_the_first(self) -> None:
        """Partial spawning must not strand what was already created."""
        world = FakeWorld()
        world.refuse_spawn.add("walker.pedestrian.0001")
        two = definition(
            actors=[
                actor("car"),
                actor("walker", blueprint="walker.pedestrian.0001", forward_m=10.0),
            ]
        )
        result = ScenarioRunner(two, simulator=simulator(world), settings=settings()).run()

        assert result.state is ScenarioState.FAILED
        assert world.actors == {}
        assert [a.actor_id for a in result.actors] == ["car"]

    def test_world_settings_are_restored_after_a_run(self) -> None:
        world = FakeWorld()
        ScenarioRunner(definition(), simulator=simulator(world), settings=settings()).run()
        assert world.settings.synchronous_mode is False


class TestSeedAndDeterminism:
    def test_the_seed_is_reported_on_the_result(self) -> None:
        result = ScenarioRunner(
            definition(seed=99), simulator=simulator(), settings=settings()
        ).run()
        assert result.seed == 99

    def test_the_seed_is_passed_to_the_simulator_settings(self) -> None:
        runner = ScenarioRunner(definition(seed=123), simulator=simulator(), settings=settings())
        runner.run()
        assert runner._settings.carla.seed == 123

    def test_resolution_is_the_identity_without_jitter(self) -> None:
        resolved = resolve(definition())
        assert resolved.actors[0].base == definition().actors[0].placement
        assert resolved.actors[0].jitter_applied_m == (0.0, 0.0)

    def test_the_same_seed_resolves_identically(self) -> None:
        jittered = definition(actors=[actor(jitter=1.0)], seed=5)
        assert resolve(jittered) == resolve(jittered)

    def test_a_different_seed_changes_a_randomised_placement(self) -> None:
        first = resolve(definition(actors=[actor(jitter=1.0)], seed=5))
        second = resolve(definition(actors=[actor(jitter=1.0)], seed=6))
        assert first.actors[0].base != second.actors[0].base

    def test_a_different_seed_changes_nothing_without_jitter(self) -> None:
        first = resolve(definition(seed=5))
        second = resolve(definition(seed=6))
        assert first.actors == second.actors

    def test_jitter_stays_within_its_half_width(self) -> None:
        for seed in range(20):
            resolved = resolve(definition(actors=[actor(jitter=0.5)], seed=seed))
            d_forward, d_left = resolved.actors[0].jitter_applied_m
            assert -0.5 <= d_forward <= 0.5
            assert -0.5 <= d_left <= 0.5

    def test_resolution_does_not_touch_the_global_random_state(self) -> None:
        """An explicit generator: nothing else in the process can perturb a draw."""
        import random

        random.seed(1)
        before = random.random()
        random.seed(1)
        resolve(definition(actors=[actor(jitter=1.0)], seed=5))
        after = random.random()
        assert before == after

    def test_two_runs_produce_identical_frames(self) -> None:
        def run() -> list[tuple[int, float, int]]:
            result = ScenarioRunner(
                definition(actors=[actor(jitter=0.3)]), simulator=simulator(), settings=settings()
            ).run()
            return [(r.simulator_frame_id, r.scenario_time_s, r.point_count) for r in result.frames]

        assert run() == run()


class TestActorPlacement:
    def test_actors_are_spawned_at_their_resolved_placement(self) -> None:
        """Fake ego at the origin facing +x: (forward, left) -> world (forward, -left)."""
        world = FakeWorld()
        placed = definition(actors=[actor(forward_m=30.0, left_m=4.0)])
        runner = ScenarioRunner(placed, simulator=simulator(world), settings=settings())
        spawned: list[tuple[float, float]] = []

        original = world.try_spawn_actor

        def capture(blueprint, transform, attach_to=None):  # type: ignore[no-untyped-def]
            spawned.append((transform.location.x, transform.location.y))
            return original(blueprint, transform, attach_to)

        world.try_spawn_actor = capture  # type: ignore[method-assign]
        runner.run()

        # ego, sensor, target - the target is the last spawn
        assert spawned[-1] == pytest.approx((30.0, -4.0))

    def test_the_result_links_scenario_ids_to_simulator_ids(self) -> None:
        two = definition(actors=[actor("car"), actor("van", forward_m=30.0)])
        result = ScenarioRunner(two, simulator=simulator(), settings=settings()).run()

        assert [a.actor_id for a in result.actors] == ["car", "van"]
        ids = [a.simulator_actor_id for a in result.actors]
        assert len(set(ids)) == 2

    def test_simulator_ids_are_deterministic_across_runs(self) -> None:
        def ids() -> list[int]:
            two = definition(actors=[actor("car"), actor("van", forward_m=30.0)])
            result = ScenarioRunner(two, simulator=simulator(), settings=settings()).run()
            return [a.simulator_actor_id for a in result.actors]

        assert ids() == ids()


class TestScriptedMotionOnFrames:
    def test_the_expected_pose_is_recorded_on_every_frame(self) -> None:
        moving = definition(
            duration_s=1.0,
            actors=[
                actor(
                    forward_m=45.0,
                    motion=[MotionSegment(start_s=0.0, stop_s=1.0, forward_mps=-8.0)],
                )
            ],
        )
        result = ScenarioRunner(moving, simulator=simulator(), settings=settings()).run()

        for record in result.frames:
            pose = record.expected_poses[0].position
            assert pose.x == pytest.approx(45.0 - 8.0 * record.scenario_time_s)

    def test_a_timed_start_holds_position_until_it_begins(self) -> None:
        waits = definition(
            duration_s=1.0,
            actors=[
                actor(
                    forward_m=15.0,
                    left_m=-4.0,
                    motion=[MotionSegment(start_s=0.5, stop_s=1.0, left_mps=2.0)],
                )
            ],
        )
        result = ScenarioRunner(waits, simulator=simulator(), settings=settings()).run()

        before = [r for r in result.frames if r.scenario_time_s < 0.5 - 1e-9]
        after = [r for r in result.frames if r.scenario_time_s > 0.5 + 1e-9]
        assert all(r.expected_poses[0].position.y == pytest.approx(-4.0) for r in before)
        assert all(r.expected_poses[0].position.y > -4.0 for r in after)

    def test_the_actor_is_moved_in_the_simulator_to_match(self) -> None:
        """Ground truth reflects the scripted pose, because the actor was put there."""
        moving = definition(
            duration_s=1.0,
            actors=[
                actor(
                    forward_m=40.0,
                    motion=[MotionSegment(start_s=0.0, stop_s=1.0, forward_mps=-8.0)],
                )
            ],
        )
        result = ScenarioRunner(moving, simulator=simulator(), settings=settings()).run()

        for record, truth in zip(result.frames, result.ground_truth, strict=True):
            reported = truth.nearest()
            assert reported is not None
            expected = record.expected_poses[0].position
            assert reported.position.x == pytest.approx(expected.x, abs=0.05)

    def test_frame_intervals_are_exactly_the_timestep(self) -> None:
        result = ScenarioRunner(definition(), simulator=simulator(), settings=settings()).run()
        assert result.frame_intervals_s() == pytest.approx([TIMESTEP] * (result.frame_count - 1))

    def test_scenario_time_and_simulation_time_advance_together(self) -> None:
        result = ScenarioRunner(definition(), simulator=simulator(), settings=settings()).run()
        for earlier, later in pairwise(result.frames):
            assert later.scenario_time_s - earlier.scenario_time_s == pytest.approx(TIMESTEP)
            assert (later.timestamp - earlier.timestamp).total_seconds() == pytest.approx(TIMESTEP)


class TestGroundTruth:
    def test_one_ground_truth_frame_per_stepped_frame(self) -> None:
        result = ScenarioRunner(definition(), simulator=simulator(), settings=settings()).run()
        assert len(result.ground_truth) == result.frame_count

    def test_ground_truth_and_sensor_frames_share_identity(self) -> None:
        result = ScenarioRunner(definition(), simulator=simulator(), settings=settings()).run()
        for record, truth in zip(result.frames, result.ground_truth, strict=True):
            assert record.simulator_frame_id == truth.frame_id
            assert record.timestamp == truth.timestamp

    def test_ground_truth_is_labelled_simulation(self) -> None:
        result = ScenarioRunner(definition(), simulator=simulator(), settings=settings()).run()
        assert all(truth.source is DataSource.SIMULATION for truth in result.ground_truth)
        assert result.source is DataSource.SIMULATION

    def test_the_processor_receives_only_the_sensor_frame(self) -> None:
        """Separation by signature: ground truth cannot reach the pipeline."""
        from adaptx.models.point_cloud import RawPointCloudFrame

        seen: list[object] = []
        ScenarioRunner(
            definition(),
            simulator=simulator(),
            processor=counting_processor(seen),  # type: ignore[arg-type]
            settings=settings(),
        ).run()

        assert seen
        assert all(isinstance(frame, RawPointCloudFrame) for frame in seen)

    def test_frames_can_be_recorded_without_processing(self) -> None:
        result = ScenarioRunner(definition(), simulator=simulator(), settings=settings()).run()
        assert all(record.pipeline is None for record in result.frames)
        assert result.frame_count == 10


class TestIsolation:
    """Scenario B must inherit nothing from scenario A."""

    def test_two_scenarios_in_sequence_do_not_share_actors(self) -> None:
        world = FakeWorld()
        first = ScenarioRunner(
            definition("first", actors=[actor("a1"), actor("a2", forward_m=30.0)]),
            simulator=simulator(world),
            settings=settings(),
        ).run()
        second = ScenarioRunner(
            definition("second", actors=[actor("b1")]),
            simulator=simulator(world),
            settings=settings(),
        ).run()

        assert [a.actor_id for a in first.actors] == ["a1", "a2"]
        assert [a.actor_id for a in second.actors] == ["b1"]
        assert all(t.actor_count <= 2 for t in second.ground_truth)  # ego + b1, never a1/a2
        assert world.actors == {}

    def test_each_run_reports_its_own_seed(self) -> None:
        first = ScenarioRunner(definition(seed=1), simulator=simulator(), settings=settings()).run()
        second = ScenarioRunner(
            definition(seed=2), simulator=simulator(), settings=settings()
        ).run()
        assert (first.seed, second.seed) == (1, 2)

    def test_a_failed_run_does_not_poison_the_next(self) -> None:
        world = FakeWorld()
        world.refuse_spawn.add("walker.pedestrian.0001")
        bad = definition("bad", actors=[actor("w", blueprint="walker.pedestrian.0001")])
        assert ScenarioRunner(bad, simulator=simulator(world), settings=settings()).run().state is (
            ScenarioState.FAILED
        )

        good = ScenarioRunner(
            definition("good"), simulator=simulator(world), settings=settings()
        ).run()
        assert good.completed
        assert good.frame_count == 10


class TestRunResultContract:
    def test_frames_and_ground_truth_must_align(self) -> None:
        from pydantic import ValidationError

        result = ScenarioRunner(definition(), simulator=simulator(), settings=settings()).run()
        with pytest.raises(ValidationError, match="every frame must have exactly one"):
            result.model_copy(update={"ground_truth": result.ground_truth[:-1]}).model_validate(
                result.model_copy(update={"ground_truth": result.ground_truth[:-1]}).model_dump()
            )

    def test_the_result_round_trips_through_json(self) -> None:
        """Raw evidence must survive serialisation: Phase 11 will read it from disk."""
        from adaptx.scenarios.result import ScenarioRunResult

        result = ScenarioRunner(definition(), simulator=simulator(), settings=settings()).run()
        rebuilt = ScenarioRunResult.model_validate_json(result.model_dump_json())
        assert rebuilt.frame_count == result.frame_count
        assert rebuilt.resolved == result.resolved
        assert rebuilt.ground_truth[0].frame_id == result.ground_truth[0].frame_id

    def test_the_result_records_the_version_the_server_reported(self) -> None:
        """The ``carla`` package ships no ``__version__`` (0.9.16), so the
        server's own answer is the only version worth writing down."""
        result = ScenarioRunner(definition(), simulator=simulator(), settings=settings()).run()
        assert result.simulator_version == "fake-0.0"

    def test_the_result_carries_no_evaluation_metric(self) -> None:
        """Raw evidence only. A metric here would be a Phase 11 claim."""
        from adaptx.scenarios.result import ScenarioFrameRecord, ScenarioRunResult, StageCounts

        for model in (ScenarioRunResult, ScenarioFrameRecord, StageCounts):
            names = " ".join(model.model_fields)
            for forbidden in ("accuracy", "precision", "recall", "error_m", "iou", "score"):
                assert forbidden not in names, f"{model.__name__} carries '{forbidden}'"
