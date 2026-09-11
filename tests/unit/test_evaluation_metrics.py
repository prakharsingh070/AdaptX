"""Evaluation primitives (Phase 11): statistics, matching, alignment, ordering.

Every function here is asserted on a hand-built case with an exactly known
answer. No pipeline runs; nothing is sampled.
"""

from __future__ import annotations

import math

import pytest
from pydantic import ValidationError

from adaptx.evaluation.adaptive import tile_for
from adaptx.evaluation.dataset import ReferenceActor, ego_to_sensor_frame
from adaptx.evaluation.matching import Candidate, match_frame, planar_distance
from adaptx.evaluation.models import (
    MIN_SAMPLES_FOR_P95,
    Distribution,
    EvaluationConfiguration,
)
from adaptx.evaluation.prediction import frames_ahead
from adaptx.evaluation.risk import is_alerting, ordering_concordance
from adaptx.evaluation.tracking import _fragments
from adaptx.models.adaptive_resolution import ResolutionPlan
from adaptx.models.common import Dimensions, ObjectClass, Vector3
from adaptx.models.map import ResolutionLevel
from adaptx.models.risk import RiskLevel
from tests.fixtures.assessments import assessed, unknown_risk
from tests.fixtures.evaluation import FrameSpec, RunBuilder, levels


def actor(
    actor_id: int, x: float, y: float, cls: ObjectClass = ObjectClass.VEHICLE
) -> ReferenceActor:
    return ReferenceActor(
        actor_id=f"actor-{actor_id}",
        simulator_actor_id=actor_id,
        object_class=cls,
        position=Vector3(x=x, y=y, z=0.0),
        reference_velocity=None,
        dimensions=Dimensions(length=4.0, width=2.0, height=1.5),
        distance_m=math.hypot(x, y),
        eligible=True,
    )


def candidate(
    candidate_id: int, x: float, y: float, cls: ObjectClass = ObjectClass.VEHICLE
) -> Candidate:
    return Candidate(candidate_id=candidate_id, position=Vector3(x=x, y=y, z=0.0), object_class=cls)


class TestDistribution:
    def test_an_empty_set_has_no_statistic_and_no_zero(self) -> None:
        d = Distribution.of([])
        assert d.count == 0
        assert d.mean is None and d.median is None and d.rmse is None
        assert d.min is None and d.max is None and d.p95 is None

    def test_known_values(self) -> None:
        d = Distribution.of([3.0, 4.0])
        assert d.count == 2
        assert d.mean == pytest.approx(3.5)
        assert d.median == pytest.approx(3.5)
        assert d.rmse == pytest.approx(math.sqrt(12.5))
        assert d.min == 3.0 and d.max == 4.0

    def test_p95_needs_enough_samples(self) -> None:
        few = Distribution.of([1.0] * (MIN_SAMPLES_FOR_P95 - 1))
        assert few.p95 is None
        assert few.p95_note is not None and str(MIN_SAMPLES_FOR_P95) in few.p95_note
        enough = Distribution.of([float(i) for i in range(MIN_SAMPLES_FOR_P95 + 1)])
        assert enough.p95 == pytest.approx(19.0)  # numpy linear interpolation of 0..20
        assert enough.p95_note is None

    def test_non_finite_values_are_refused(self) -> None:
        with pytest.raises(ValueError, match="non-finite"):
            Distribution.of([1.0, float("nan")])


class TestConfiguration:
    def test_defaults_are_labelled_baseline(self) -> None:
        config = EvaluationConfiguration()
        assert config.is_baseline
        assert config.primary_gate_m in config.match_gates_m

    def test_gates_are_sorted_and_distinct(self) -> None:
        config = EvaluationConfiguration(match_gates_m=[4.0, 1.0, 2.0], primary_gate_m=2.0)
        assert config.match_gates_m == [1.0, 2.0, 4.0]
        with pytest.raises(ValidationError, match="distinct"):
            EvaluationConfiguration(match_gates_m=[1.0, 1.0])

    def test_the_primary_gate_must_be_one_of_the_gates(self) -> None:
        with pytest.raises(ValidationError, match="primary_gate_m"):
            EvaluationConfiguration(match_gates_m=[1.0, 2.0], primary_gate_m=3.0)

    def test_a_non_positive_gate_is_refused(self) -> None:
        with pytest.raises(ValidationError, match="positive"):
            EvaluationConfiguration(match_gates_m=[0.0, 2.0], primary_gate_m=2.0)


class TestMatching:
    def test_a_candidate_on_the_actor_matches_with_zero_error(self) -> None:
        result = match_frame([candidate(0, 10.0, 0.0)], [actor(5, 10.0, 0.0)], gate_m=1.0)
        assert len(result.matches) == 1
        assert result.matches[0].distance_planar_m == 0.0
        assert result.matches[0].class_agreed
        assert result.unmatched_actor_ids == () and result.unmatched_candidate_ids == ()

    def test_the_gate_excludes_a_far_candidate(self) -> None:
        result = match_frame([candidate(0, 12.0, 0.0)], [actor(5, 10.0, 0.0)], gate_m=1.0)
        assert result.matches == ()
        assert result.unmatched_actor_ids == (5,)
        assert result.unmatched_candidate_ids == (0,)

    def test_the_nearest_candidate_wins(self) -> None:
        result = match_frame(
            [candidate(0, 10.8, 0.0), candidate(1, 10.2, 0.0)], [actor(5, 10.0, 0.0)], gate_m=2.0
        )
        assert result.matches[0].candidate_id == 1
        assert result.unmatched_candidate_ids == (0,)

    def test_each_side_is_used_at_most_once(self) -> None:
        result = match_frame(
            [candidate(0, 10.0, 0.0)], [actor(5, 10.0, 0.0), actor(6, 10.5, 0.0)], gate_m=2.0
        )
        assert len(result.matches) == 1
        assert result.matches[0].simulator_actor_id == 5
        assert result.unmatched_actor_ids == (6,)

    def test_ties_break_on_id_so_input_order_does_not_matter(self) -> None:
        a = match_frame(
            [candidate(3, 9.5, 0.0), candidate(1, 10.5, 0.0)], [actor(5, 10.0, 0.0)], gate_m=2.0
        )
        b = match_frame(
            [candidate(1, 10.5, 0.0), candidate(3, 9.5, 0.0)], [actor(5, 10.0, 0.0)], gate_m=2.0
        )
        assert a.matches[0].candidate_id == b.matches[0].candidate_id == 1

    def test_a_class_disagreement_is_recorded_not_refused(self) -> None:
        result = match_frame(
            [candidate(0, 10.0, 0.0, ObjectClass.PEDESTRIAN)], [actor(5, 10.0, 0.0)], gate_m=1.0
        )
        assert len(result.matches) == 1
        assert not result.matches[0].class_agreed

    def test_the_3d_distance_includes_height(self) -> None:
        c = Candidate(
            candidate_id=0, position=Vector3(x=10.0, y=0.0, z=3.0), object_class=ObjectClass.VEHICLE
        )
        result = match_frame([c], [actor(5, 10.0, 4.0)], gate_m=5.0)
        assert result.matches[0].distance_planar_m == pytest.approx(4.0)
        assert result.matches[0].distance_3d_m == pytest.approx(5.0)

    def test_a_non_positive_gate_is_refused(self) -> None:
        with pytest.raises(ValueError, match="positive"):
            match_frame([], [], gate_m=0.0)

    def test_planar_distance(self) -> None:
        assert planar_distance(Vector3(x=0, y=0, z=9), Vector3(x=3, y=4, z=0)) == 5.0


class TestOrdering:
    def test_a_perfect_inverse_relation_scores_one(self) -> None:
        pairs, value = ordering_concordance([(30.0, 0.2), (20.0, 0.5), (10.0, 0.9)])
        assert pairs == 3 and value == 1.0

    def test_a_direct_relation_scores_zero(self) -> None:
        _, value = ordering_concordance([(10.0, 0.2), (20.0, 0.5), (30.0, 0.9)])
        assert value == 0.0

    def test_ties_say_nothing_and_are_dropped(self) -> None:
        pairs, value = ordering_concordance([(10.0, 0.5), (20.0, 0.5), (30.0, 0.1)])
        assert pairs == 2 and value == 1.0

    def test_fewer_than_two_usable_pairs_is_null_not_zero(self) -> None:
        assert ordering_concordance([(10.0, 0.5)]) == (0, None)
        assert ordering_concordance([]) == (0, None)


class TestAlerting:
    def test_unknown_never_alerts(self) -> None:
        assert not is_alerting(unknown_risk(), RiskLevel.LOW)

    def test_at_or_above_the_level_alerts(self) -> None:
        high = assessed().model_copy(update={"risk_level": RiskLevel.HIGH, "risk_score": 0.7})
        critical = assessed().model_copy(
            update={"risk_level": RiskLevel.CRITICAL, "risk_score": 0.9}
        )
        medium = assessed()
        assert is_alerting(high, RiskLevel.HIGH)
        assert is_alerting(critical, RiskLevel.HIGH)
        assert not is_alerting(medium, RiskLevel.HIGH)


class TestAlignment:
    def test_a_whole_number_of_frames_aligns(self) -> None:
        assert frames_ahead(0.25, 0.05) == 5
        assert frames_ahead(3.0, 0.05) == 60

    def test_a_fractional_offset_does_not_align_and_is_not_rounded(self) -> None:
        assert frames_ahead(0.3, 0.07) is None
        assert frames_ahead(0.125, 0.05) is None


class TestFragments:
    def test_no_frames_is_no_fragment(self) -> None:
        assert _fragments([]) == (0, 0)

    def test_consecutive_frames_are_one_fragment(self) -> None:
        assert _fragments([3, 4, 5]) == (1, 3)

    def test_a_gap_splits_fragments(self) -> None:
        assert _fragments([1, 2, 4, 5, 6]) == (2, 3)


class TestFrames:
    def test_ego_to_sensor_frame_is_a_translation(self) -> None:
        moved = ego_to_sensor_frame(Vector3(x=10.0, y=2.0, z=0.0), Vector3(x=1.0, y=0.0, z=1.8))
        assert (moved.x, moved.y, moved.z) == (9.0, 2.0, -1.8)


class TestTileLookup:
    def _plan(self) -> ResolutionPlan:
        record = (
            RunBuilder({"a": 5}).frame(FrameSpec(levels=levels((5, ResolutionLevel.HIGH)))).build()
        )
        return record.frames[0].outputs.plan

    def test_an_interior_point_finds_its_tile(self) -> None:
        plan = self._plan()
        tile = tile_for(plan, -5.0, -5.0)  # column 1, row 1 -> index 5
        assert tile is not None and tile.tile_index == 5 and tile.level is ResolutionLevel.HIGH

    def test_a_boundary_point_belongs_to_exactly_the_tile_that_owns_it(self) -> None:
        plan = self._plan()
        tile = tile_for(plan, -10.0, -10.0)  # lower edge of tile 5, upper edge of tile 0
        assert tile is not None and tile.tile_index == 5

    def test_the_map_edge_is_not_lost(self) -> None:
        plan = self._plan()
        tile = tile_for(plan, 20.0, 20.0)
        assert tile is not None and tile.tile_index == 15

    def test_outside_the_map_is_none(self) -> None:
        assert tile_for(self._plan(), 25.0, 0.0) is None
