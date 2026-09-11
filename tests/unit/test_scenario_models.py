"""Scenario definition contract tests (Phase 10).

A malformed scenario must be rejected at construction, before it can reach a
simulator. Every rule here is asserted with an explicit error, and every
expected value in the motion tests is arithmetic on the definition rather
than something the code happened to return.

No simulator is involved anywhere in this file: the definition models import
none, which is what makes them reusable beyond CARLA (ADR-046).
"""

from __future__ import annotations

import pytest
from pydantic import ValidationError

from adaptx.models.common import ObjectClass
from adaptx.scenarios.models import (
    EgoDefinition,
    MotionSegment,
    Placement,
    ScenarioActor,
    ScenarioDefinition,
    ScenarioState,
)


def actor(
    actor_id: str = "target",
    *,
    forward_m: float = 20.0,
    left_m: float = 0.0,
    motion: list[MotionSegment] | None = None,
    jitter: float = 0.0,
) -> ScenarioActor:
    return ScenarioActor(
        actor_id=actor_id,
        blueprint="vehicle.audi.tt",
        object_class=ObjectClass.VEHICLE,
        placement=Placement(forward_m=forward_m, left_m=left_m),
        placement_jitter_m=jitter,
        motion=motion or [],
    )


def definition(**overrides: object) -> ScenarioDefinition:
    base: dict[str, object] = {
        "scenario_id": "unit_scenario",
        "name": "Unit scenario",
        "description": "For contract tests.",
        "seed": 7,
        "duration_s": 2.0,
        "fixed_delta_seconds": 0.05,
        "actors": [actor()],
    }
    base.update(overrides)
    return ScenarioDefinition(**base)  # type: ignore[arg-type]


class TestValidDefinition:
    def test_a_minimal_definition_is_accepted(self) -> None:
        scenario = definition()
        assert scenario.scenario_id == "unit_scenario"
        assert scenario.frame_count == 40
        assert scenario.seed == 7

    def test_the_frame_count_is_duration_over_timestep_rounded_down(self) -> None:
        assert definition(duration_s=1.0, fixed_delta_seconds=0.05).frame_count == 20
        assert definition(duration_s=1.02, fixed_delta_seconds=0.05).frame_count == 20

    def test_an_exact_division_does_not_lose_a_frame_to_float_error(self) -> None:
        """3.0 / 0.05 is 59.999... in floating point; the frame count must be 60."""
        assert definition(duration_s=3.0, fixed_delta_seconds=0.05).frame_count == 60

    def test_scenario_time_advances_by_the_timestep(self) -> None:
        scenario = definition()
        assert scenario.scenario_time(0) == 0.0
        assert scenario.scenario_time(10) == pytest.approx(0.5)

    def test_an_empty_actor_list_is_a_valid_scene(self) -> None:
        """Open ground is a legitimate scenario, not a malformed one."""
        assert definition(actors=[]).actors == []

    def test_an_actor_can_be_looked_up_by_id(self) -> None:
        assert definition().actor("target").blueprint == "vehicle.audi.tt"
        with pytest.raises(KeyError, match="no actor"):
            definition().actor("nobody")


class TestInvalidDefinition:
    def test_a_non_positive_duration_is_rejected(self) -> None:
        with pytest.raises(ValidationError):
            definition(duration_s=0.0)

    def test_a_non_positive_timestep_is_rejected(self) -> None:
        with pytest.raises(ValidationError):
            definition(fixed_delta_seconds=0.0)

    def test_a_timestep_above_the_physics_ceiling_is_rejected(self) -> None:
        with pytest.raises(ValidationError):
            definition(fixed_delta_seconds=0.5)

    def test_a_scenario_of_one_frame_is_rejected(self) -> None:
        """One frame cannot show motion; the framework refuses to pretend."""
        with pytest.raises(ValidationError, match="at least 2"):
            definition(duration_s=0.05, fixed_delta_seconds=0.05)

    def test_a_negative_seed_is_rejected(self) -> None:
        with pytest.raises(ValidationError):
            definition(seed=-1)

    def test_an_empty_scenario_id_is_rejected(self) -> None:
        with pytest.raises(ValidationError):
            definition(scenario_id="")

    def test_a_scenario_id_must_be_a_machine_name(self) -> None:
        with pytest.raises(ValidationError):
            definition(scenario_id="Pedestrian Crossing")

    def test_duplicate_actor_ids_are_rejected(self) -> None:
        with pytest.raises(ValidationError, match="duplicate actor ids"):
            definition(actors=[actor("a"), actor("a")])

    def test_ego_is_a_reserved_actor_id(self) -> None:
        with pytest.raises(ValidationError, match="reserved"):
            definition(actors=[actor("ego")])

    def test_an_empty_actor_id_is_rejected(self) -> None:
        with pytest.raises(ValidationError):
            actor("")

    def test_a_non_finite_placement_is_rejected(self) -> None:
        with pytest.raises(ValidationError, match="finite"):
            Placement(forward_m=float("inf"))

    def test_a_moving_ego_is_rejected_explicitly(self) -> None:
        """Ego motion is a Phase 10 limitation and is stated, not silently ignored."""
        with pytest.raises(ValidationError, match="stationary"):
            EgoDefinition(stationary=False)

    def test_unknown_fields_are_rejected(self) -> None:
        with pytest.raises(ValidationError):
            definition(traffic_density=5)


class TestMotionSegments:
    def test_a_segment_must_end_after_it_starts(self) -> None:
        with pytest.raises(ValidationError, match="end after it starts"):
            MotionSegment(start_s=2.0, stop_s=1.0, forward_mps=1.0)

    def test_a_segment_must_fit_inside_the_scenario(self) -> None:
        with pytest.raises(ValidationError, match="after the scenario ends"):
            definition(
                duration_s=2.0,
                actors=[actor(motion=[MotionSegment(start_s=0.0, stop_s=3.0, forward_mps=1.0)])],
            )

    def test_overlapping_segments_are_rejected(self) -> None:
        with pytest.raises(ValidationError, match="overlap"):
            actor(
                motion=[
                    MotionSegment(start_s=0.0, stop_s=1.0, forward_mps=1.0),
                    MotionSegment(start_s=0.5, stop_s=1.5, forward_mps=1.0),
                ]
            )

    def test_out_of_order_segments_are_rejected(self) -> None:
        with pytest.raises(ValidationError, match="out of order"):
            actor(
                motion=[
                    MotionSegment(start_s=1.0, stop_s=2.0, forward_mps=1.0),
                    MotionSegment(start_s=0.0, stop_s=0.5, forward_mps=1.0),
                ]
            )

    def test_a_non_finite_velocity_is_rejected(self) -> None:
        with pytest.raises(ValidationError, match="finite"):
            MotionSegment(start_s=0.0, stop_s=1.0, forward_mps=float("nan"))

    def test_adjacent_segments_that_touch_are_allowed(self) -> None:
        moving = actor(
            motion=[
                MotionSegment(start_s=0.0, stop_s=1.0, forward_mps=1.0),
                MotionSegment(start_s=1.0, stop_s=2.0, left_mps=1.0),
            ]
        )
        assert len(moving.motion) == 2

    def test_segment_speed_is_the_vector_magnitude(self) -> None:
        segment = MotionSegment(start_s=0.0, stop_s=1.0, forward_mps=3.0, left_mps=4.0)
        assert segment.speed_mps == pytest.approx(5.0)
        assert segment.duration_s == pytest.approx(1.0)


class TestScriptedMotion:
    """Expected positions are closed-form arithmetic on the definition."""

    def test_a_stationary_actor_never_moves(self) -> None:
        parked = actor(forward_m=20.0, left_m=3.0)
        assert parked.is_stationary
        for t in (0.0, 1.0, 5.0):
            assert parked.expected_offset(t, parked.placement) == (20.0, 3.0)

    def test_constant_velocity_from_time_zero(self) -> None:
        moving = actor(
            forward_m=45.0, motion=[MotionSegment(start_s=0.0, stop_s=3.0, forward_mps=-8.0)]
        )
        assert moving.expected_offset(0.0, moving.placement) == pytest.approx((45.0, 0.0))
        assert moving.expected_offset(1.0, moving.placement) == pytest.approx((37.0, 0.0))
        assert moving.expected_offset(2.5, moving.placement) == pytest.approx((25.0, 0.0))

    def test_motion_does_not_start_before_its_start_time(self) -> None:
        """A timed start: the actor is exactly where it was placed until then."""
        waiting = actor(
            forward_m=15.0,
            left_m=-4.0,
            motion=[MotionSegment(start_s=1.0, stop_s=5.0, left_mps=1.4)],
        )
        assert waiting.expected_offset(0.0, waiting.placement) == pytest.approx((15.0, -4.0))
        assert waiting.expected_offset(0.99, waiting.placement) == pytest.approx((15.0, -4.0))
        assert waiting.expected_offset(1.0, waiting.placement) == pytest.approx((15.0, -4.0))

    def test_motion_freezes_at_its_stop_time(self) -> None:
        """A timed stop: the actor holds its final position afterwards."""
        crossing = actor(
            forward_m=15.0,
            left_m=-4.0,
            motion=[MotionSegment(start_s=1.0, stop_s=5.0, left_mps=1.4)],
        )
        at_stop = crossing.expected_offset(5.0, crossing.placement)
        after = crossing.expected_offset(9.0, crossing.placement)
        assert at_stop == pytest.approx((15.0, -4.0 + 1.4 * 4.0))
        assert after == at_stop

    def test_a_diagonal_segment_moves_on_both_axes(self) -> None:
        diagonal = actor(
            forward_m=12.0,
            left_m=8.0,
            motion=[MotionSegment(start_s=0.5, stop_s=3.5, forward_mps=3.0, left_mps=-4.0)],
        )
        assert diagonal.expected_offset(1.5, diagonal.placement) == pytest.approx((15.0, 4.0))

    def test_a_multi_segment_path_sums_its_displacements(self) -> None:
        """Forward for one second, then left for one: an L-shaped path."""
        path = actor(
            forward_m=10.0,
            motion=[
                MotionSegment(start_s=0.0, stop_s=1.0, forward_mps=2.0),
                MotionSegment(start_s=1.0, stop_s=2.0, left_mps=3.0),
            ],
        )
        assert path.expected_offset(0.5, path.placement) == pytest.approx((11.0, 0.0))
        assert path.expected_offset(1.0, path.placement) == pytest.approx((12.0, 0.0))
        assert path.expected_offset(1.5, path.placement) == pytest.approx((12.0, 1.5))
        assert path.expected_offset(2.0, path.placement) == pytest.approx((12.0, 3.0))

    def test_a_gap_between_segments_holds_position(self) -> None:
        gapped = actor(
            forward_m=10.0,
            motion=[
                MotionSegment(start_s=0.0, stop_s=1.0, forward_mps=1.0),
                MotionSegment(start_s=2.0, stop_s=3.0, forward_mps=1.0),
            ],
        )
        assert gapped.expected_offset(1.5, gapped.placement) == pytest.approx((11.0, 0.0))
        assert gapped.expected_offset(3.0, gapped.placement) == pytest.approx((12.0, 0.0))

    def test_the_same_time_always_yields_the_same_position(self) -> None:
        """Closed form: no accumulation, so evaluation order is irrelevant."""
        moving = actor(motion=[MotionSegment(start_s=0.0, stop_s=2.0, forward_mps=-8.0)])
        forward = [moving.expected_offset(t / 10, moving.placement) for t in range(20)]
        backward = [moving.expected_offset(t / 10, moving.placement) for t in reversed(range(20))]
        assert forward == list(reversed(backward))

    def test_motion_ends_reports_the_last_stop(self) -> None:
        moving = actor(
            motion=[
                MotionSegment(start_s=0.0, stop_s=1.0, forward_mps=1.0),
                MotionSegment(start_s=1.5, stop_s=1.8, forward_mps=1.0),
            ]
        )
        assert moving.motion_ends_s == pytest.approx(1.8)
        assert actor().motion_ends_s == 0.0


class TestSeedAndRandomness:
    def test_the_seed_is_retained_even_with_nothing_to_randomise(self) -> None:
        scenario = definition(seed=42)
        assert scenario.seed == 42
        assert scenario.has_randomised_elements is False

    def test_jitter_marks_a_scenario_as_randomised(self) -> None:
        assert definition(actors=[actor(jitter=0.5)]).has_randomised_elements is True

    def test_negative_jitter_is_rejected(self) -> None:
        with pytest.raises(ValidationError):
            actor(jitter=-0.1)


class TestLifecycleVocabulary:
    def test_the_states_are_the_documented_ones(self) -> None:
        assert [state.value for state in ScenarioState] == [
            "CREATED",
            "VALIDATING",
            "READY",
            "RUNNING",
            "COMPLETED",
            "FAILED",
        ]


class TestSerialisation:
    def test_a_definition_round_trips_through_json(self) -> None:
        """Configuration, not code: the description alone must rebuild it."""
        original = definition(
            actors=[
                actor(
                    "walker",
                    forward_m=15.0,
                    left_m=-4.0,
                    motion=[MotionSegment(start_s=1.0, stop_s=1.9, left_mps=1.4)],
                    jitter=0.25,
                )
            ],
            tags=["a", "b"],
        )
        rebuilt = ScenarioDefinition.model_validate_json(original.model_dump_json())
        assert rebuilt == original
