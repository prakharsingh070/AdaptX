"""Resolution controller tests (Phase 8).

Every case has explicit bounds, an explicit tile size and explicit object
positions, so the region an object influences is arithmetic rather than a
guess. Nothing here is random.

The map used throughout is deliberately small - 40 m square, 10 m tiles, so
4x4 = 16 regions - which makes a level distribution something a reader can
check by hand.

Assessments are built directly rather than produced by the Phase 7 engine:
these tests exercise resolution policy, and routing them through risk scoring
would make a policy failure indistinguishable from a scoring one. The
integration suite covers the real chain.
"""

from __future__ import annotations

import pytest

from adaptx.config.settings import AdaptiveResolutionSettings, MapSettings
from adaptx.core.exceptions import InvalidPointCloudError
from adaptx.mapping.controller import HeuristicResolutionController, build_controller
from adaptx.models.adaptive_resolution import (
    DetailFactorName,
    ExclusionReason,
    ResolutionPlan,
    TileDecisionReason,
)
from adaptx.models.map import ResolutionContext, ResolutionLevel
from adaptx.models.resolution import ResolutionSource
from adaptx.models.risk_assessment import AssessmentStatus
from adaptx.models.tracking import TrackStatus
from tests.fixtures.assessments import (
    assessed,
    at_position,
    straight_trajectory,
    unknown_risk,
)

#: A 40 m square map divided into 10 m regions: 4x4 = 16 tiles.
UNIT_MAP = {
    "min_x_m": -20.0,
    "max_x_m": 20.0,
    "min_y_m": -20.0,
    "max_y_m": 20.0,
    "resolution_m": 0.5,
}


def controller(**overrides: object) -> HeuristicResolutionController:
    """A controller over the small unit map unless overridden."""
    map_overrides = {key: overrides.pop(key) for key in list(overrides) if key in UNIT_MAP}
    map_settings = MapSettings(**{**UNIT_MAP, **map_overrides})  # type: ignore[arg-type]
    return build_controller(map_settings, AdaptiveResolutionSettings(**overrides))  # type: ignore[arg-type]


def levels(plan: ResolutionPlan) -> dict[int, ResolutionLevel]:
    """Tile index to level, for compact assertions."""
    return {decision.tile_index: decision.level for decision in plan.decisions}


class TestTiling:
    def test_the_map_is_partitioned_into_regions(self) -> None:
        grid = controller().tile_grid()

        assert (grid.columns, grid.rows) == (4, 4)
        assert grid.tile_count == 16

    def test_every_region_receives_a_decision(self) -> None:
        plan = controller().plan([])

        assert plan.tile_count == 16
        assert [d.tile_index for d in plan.decisions] == list(range(16))

    def test_regions_cover_the_extent_without_gap_or_overlap(self) -> None:
        grid = controller().tile_grid()
        area = sum(
            grid.tile_bounds(index).size_x_m * grid.tile_bounds(index).size_y_m
            for index in range(grid.tile_count)
        )

        assert area == pytest.approx(40.0 * 40.0)

    def test_a_point_belongs_to_exactly_one_region(self) -> None:
        grid = controller().tile_grid()
        # Exactly on an interior region boundary.
        index = grid.locate(-10.0, -10.0)

        assert index is not None
        holding = [i for i in range(grid.tile_count) if grid.tile_bounds(i).contains(-10.0, -10.0)]
        assert holding == [index]

    def test_the_upper_edge_is_outside_the_map(self) -> None:
        grid = controller().tile_grid()

        assert grid.locate(20.0, 0.0) is None
        assert grid.locate(-20.0, -20.0) == 0

    def test_a_tiling_beyond_the_region_ceiling_is_rejected(self) -> None:
        with pytest.raises(InvalidPointCloudError) as error:
            controller(tile_size_m=1.0, max_tiles=16).tile_grid()

        assert "regions" in str(error.value)

    def test_an_extent_that_is_not_a_whole_number_of_regions_is_clipped(self) -> None:
        grid = controller(max_x_m=25.0, tile_size_m=10.0).tile_grid()
        last = grid.tile_bounds(grid.index_of(0, grid.columns - 1))

        assert last.max_x == 25.0
        assert last.size_x_m == pytest.approx(5.0)


class TestEmptyScene:
    def test_an_empty_scene_stays_at_the_base_level(self) -> None:
        """The reason adaptive mapping exists: spend nothing where nothing is."""
        plan = controller().plan([])

        assert set(levels(plan).values()) == {ResolutionLevel.LOW}
        assert plan.counts_by_level()["low"] == 16

    def test_an_uninfluenced_region_has_no_priority_rather_than_zero(self) -> None:
        """No evidence is not evidence of quiet, and the record says which."""
        decision = controller().plan([]).decisions[0]

        assert decision.detail_priority is None
        assert decision.factors == []
        assert TileDecisionReason.NO_OBJECT_INFLUENCE in decision.reasons

    def test_an_empty_scene_allocates_far_fewer_cells_than_a_uniform_fine_map(self) -> None:
        plan = controller().plan([])
        # The same extent at the finest level would be 400 x 400.
        finest = (40.0 / 0.1) ** 2

        assert plan.total_cell_count == 16 * 10 * 10
        assert plan.total_cell_count < finest


class TestHighPriorityRefinement:
    def test_a_close_high_risk_object_refines_its_own_region(self) -> None:
        """The central case: detail goes where the concern is."""
        assessment = assessed(
            track_id=1, risk_score=0.95, uncertainty=0.3, position=(5.0, 5.0, 0.0)
        )
        plan = controller().plan([assessment], tracks=[at_position((5.0, 5.0, 0.0), track_id=1)])

        grid = controller().tile_grid()
        occupied = grid.locate(5.0, 5.0)
        assert occupied is not None
        decision = plan.decision_for(occupied)

        assert decision is not None
        assert decision.level in (ResolutionLevel.HIGH, ResolutionLevel.CRITICAL)
        assert decision.detail_priority is not None
        assert decision.resolution_m < 0.5

    def test_distant_unrelated_regions_stay_coarse(self) -> None:
        assessment = assessed(
            track_id=1, risk_score=0.95, uncertainty=0.3, position=(5.0, 5.0, 0.0)
        )
        plan = controller().plan([assessment], tracks=[at_position((5.0, 5.0, 0.0), track_id=1)])

        grid = controller().tile_grid()
        far = grid.locate(-15.0, -15.0)
        assert far is not None
        decision = plan.decision_for(far)

        assert decision is not None
        assert decision.level is ResolutionLevel.LOW
        assert decision.influencing_track_ids == []

    def test_one_object_does_not_refine_the_whole_map(self) -> None:
        """Influence is bounded, or adaptive mapping is just a fine uniform map."""
        assessment = assessed(track_id=1, risk_score=1.0, uncertainty=1.0, position=(0.0, 0.0, 0.0))
        plan = controller().plan([assessment], tracks=[at_position((0.0, 0.0, 0.0), track_id=1)])

        coarse = sum(1 for level in levels(plan).values() if level is ResolutionLevel.LOW)
        assert coarse > 0

    def test_the_scored_region_records_which_track_influenced_it(self) -> None:
        assessment = assessed(track_id=7, risk_score=0.9, position=(5.0, 5.0, 0.0))
        plan = controller().plan([assessment], tracks=[at_position((5.0, 5.0, 0.0), track_id=7)])
        influenced = [d for d in plan.decisions if d.influencing_track_ids]

        assert influenced
        assert all(d.influencing_track_ids == [7] for d in influenced)


class TestLevelOrdering:
    def test_a_higher_risk_object_never_receives_a_coarser_level(self) -> None:
        def level_at(score: float) -> ResolutionLevel:
            plan = controller().plan(
                [assessed(track_id=1, risk_score=score, uncertainty=0.2, position=(5.0, 5.0, 0.0))],
                tracks=[at_position((5.0, 5.0, 0.0), track_id=1)],
            )
            grid = controller().tile_grid()
            index = grid.locate(5.0, 5.0)
            assert index is not None
            decision = plan.decision_for(index)
            assert decision is not None
            return decision.level

        sizes = [controller().cell_size_m(level_at(score)) for score in (0.05, 0.35, 0.65, 0.95)]
        assert sizes == sorted(sizes, reverse=True)

    def test_levels_stay_within_the_configured_vocabulary(self) -> None:
        plan = controller().plan(
            [assessed(track_id=1, risk_score=1.0, uncertainty=1.0, position=(0.0, 0.0, 0.0))],
            tracks=[at_position((0.0, 0.0, 0.0), track_id=1)],
        )
        allowed = set(controller().levels())
        sizes = set(controller().levels().values())

        assert {d.level for d in plan.decisions} <= allowed
        assert {d.resolution_m for d in plan.decisions} <= sizes

    def test_every_decision_is_labelled_adaptive(self) -> None:
        plan = controller().plan([])

        assert all(d.resolution.source is ResolutionSource.ADAPTIVE for d in plan.decisions)
        assert all(d.resolution.is_adaptive for d in plan.decisions)


class TestUnknownRisk:
    """``risk_score is None`` must never be read as low risk (ADR-038)."""

    def test_an_unknown_assessment_does_not_receive_the_coarsest_level(self) -> None:
        plan = controller().plan(
            [unknown_risk(track_id=1, uncertainty=0.9, position=(5.0, 5.0, 0.0))],
            tracks=[at_position((5.0, 5.0, 0.0), track_id=1, velocity=None)],
        )
        grid = controller().tile_grid()
        index = grid.locate(5.0, 5.0)
        assert index is not None
        decision = plan.decision_for(index)

        assert decision is not None
        assert decision.level is not ResolutionLevel.LOW

    def test_the_risk_factor_is_dropped_rather_than_scored_zero(self) -> None:
        """A dropped factor renormalises the mean; a zero would drag it down."""
        plan = controller().plan(
            [unknown_risk(track_id=1, uncertainty=0.9, position=(5.0, 5.0, 0.0))],
            tracks=[at_position((5.0, 5.0, 0.0), track_id=1, velocity=None)],
        )
        grid = controller().tile_grid()
        index = grid.locate(5.0, 5.0)
        assert index is not None
        decision = plan.decision_for(index)

        assert decision is not None
        assert DetailFactorName.RISK not in decision.factors
        assert decision.factor_scores.risk is None

    def test_the_cause_is_always_visible_in_the_decision(self) -> None:
        """Which tracks could not be scored is recorded whether or not it bound."""
        plan = controller().plan(
            [unknown_risk(track_id=3, position=(5.0, 5.0, 0.0))],
            tracks=[at_position((5.0, 5.0, 0.0), track_id=3, velocity=None)],
        )
        grid = controller().tile_grid()
        index = grid.locate(5.0, 5.0)
        assert index is not None
        decision = plan.decision_for(index)

        assert decision is not None
        assert decision.unknown_risk_track_ids == [3]
        assert "unscored" in decision.reason
        assert "risk factor dropped" in decision.reason

    def test_the_floor_is_recorded_only_when_it_actually_raises_the_level(self) -> None:
        """A quiet, well-observed unknown object is the case where the floor binds.

        With high uncertainty the region earns detail on its own and the floor
        changes nothing, so claiming it was floored would misdescribe the
        decision.
        """
        plan = controller().plan(
            [unknown_risk(track_id=3, uncertainty=0.05, position=(18.0, 18.0, 0.0))],
            tracks=[at_position((18.0, 18.0, 0.0), track_id=3, velocity=None)],
        )
        grid = controller().tile_grid()
        index = grid.locate(18.0, 18.0)
        assert index is not None
        decision = plan.decision_for(index)

        assert decision is not None
        assert decision.level is ResolutionLevel.MEDIUM
        assert TileDecisionReason.UNKNOWN_RISK_FLOOR in decision.reasons
        assert "level floored" in decision.reason

    def test_an_unknown_object_outranks_a_confidently_quiet_one(self) -> None:
        """The inversion ADR-032 exists to prevent, one phase later."""
        grid = controller().tile_grid()

        quiet = controller().plan(
            [assessed(track_id=1, risk_score=0.02, uncertainty=0.02, position=(5.0, 5.0, 0.0))],
            tracks=[at_position((5.0, 5.0, 0.0), track_id=1)],
        )
        unknown = controller().plan(
            [unknown_risk(track_id=1, uncertainty=0.9, position=(5.0, 5.0, 0.0))],
            tracks=[at_position((5.0, 5.0, 0.0), track_id=1, velocity=None)],
        )

        index = grid.locate(5.0, 5.0)
        assert index is not None
        quiet_decision = quiet.decision_for(index)
        unknown_decision = unknown.decision_for(index)
        assert quiet_decision is not None and unknown_decision is not None

        assert unknown_decision.resolution_m < quiet_decision.resolution_m

    def test_a_lost_track_is_excluded_rather_than_refined(self) -> None:
        """A terminated track is history; spending detail there maps the past."""
        lost = unknown_risk(
            track_id=1,
            position=(5.0, 5.0, 0.0),
            status=AssessmentStatus.TRACK_LOST,
            track_status=TrackStatus.LOST,
        )
        plan = controller().plan([lost], tracks=[at_position((5.0, 5.0, 0.0), track_id=1)])

        assert [e.reason for e in plan.excluded] == [ExclusionReason.TRACK_LOST]
        assert set(levels(plan).values()) == {ResolutionLevel.LOW}


class TestUncertaintyDrivesDetail:
    """The ADR-033 payoff: uncertainty is an input in its own right."""

    def test_a_low_risk_but_poorly_observed_region_receives_more_detail(self) -> None:
        position = (5.0, 5.0, 0.0)
        grid = controller().tile_grid()
        index = grid.locate(*position[:2])
        assert index is not None

        confident = controller().plan(
            [assessed(track_id=1, risk_score=0.2, uncertainty=0.05, position=position)],
            tracks=[at_position(position, track_id=1)],
        )
        vague = controller().plan(
            [assessed(track_id=1, risk_score=0.2, uncertainty=0.95, position=position)],
            tracks=[at_position(position, track_id=1)],
        )

        confident_decision = confident.decision_for(index)
        vague_decision = vague.decision_for(index)
        assert confident_decision is not None and vague_decision is not None
        assert vague_decision.detail_priority is not None
        assert confident_decision.detail_priority is not None

        assert vague_decision.detail_priority > confident_decision.detail_priority
        assert vague_decision.resolution_m <= confident_decision.resolution_m

    def test_uncertainty_is_never_summed_into_the_risk_factor(self) -> None:
        plan = controller().plan(
            [assessed(track_id=1, risk_score=0.4, uncertainty=0.9, position=(5.0, 5.0, 0.0))],
            tracks=[at_position((5.0, 5.0, 0.0), track_id=1)],
        )
        grid = controller().tile_grid()
        index = grid.locate(5.0, 5.0)
        assert index is not None
        decision = plan.decision_for(index)

        assert decision is not None
        assert decision.factor_scores.risk == pytest.approx(0.4)
        assert decision.factor_scores.uncertainty == pytest.approx(0.9)

    def test_high_uncertainty_does_not_refine_the_entire_map(self) -> None:
        plan = controller().plan(
            [assessed(track_id=1, risk_score=0.1, uncertainty=1.0, position=(5.0, 5.0, 0.0))],
            tracks=[at_position((5.0, 5.0, 0.0), track_id=1)],
        )

        assert sum(1 for level in levels(plan).values() if level is ResolutionLevel.LOW) > 0


class TestTrajectoryRelevance:
    def test_a_predicted_path_raises_the_priority_of_regions_it_crosses(self) -> None:
        """Predictive refinement: detail before arrival, not after."""
        position = (-15.0, 0.0, 0.0)
        trajectory = straight_trajectory((-15.0, 0.0), (8.0, 0.0), track_id=1)
        grid = controller().tile_grid()
        ahead = grid.locate(5.0, 0.0)
        assert ahead is not None

        without = controller().plan(
            [assessed(track_id=1, risk_score=0.5, position=position)],
            tracks=[at_position(position, track_id=1)],
        )
        with_path = controller().plan(
            [assessed(track_id=1, risk_score=0.5, position=position)],
            tracks=[at_position(position, track_id=1)],
            trajectories=[trajectory],
        )

        before = without.decision_for(ahead)
        after = with_path.decision_for(ahead)
        assert before is not None and after is not None

        assert before.detail_priority is None
        assert after.detail_priority is not None
        assert after.resolution_m <= before.resolution_m

    def test_the_trajectory_factor_is_recorded_separately(self) -> None:
        plan = controller().plan(
            [assessed(track_id=1, risk_score=0.5, position=(-15.0, 0.0, 0.0))],
            tracks=[at_position((-15.0, 0.0, 0.0), track_id=1)],
            trajectories=[straight_trajectory((-15.0, 0.0), (8.0, 0.0), track_id=1)],
        )
        grid = controller().tile_grid()
        ahead = grid.locate(5.0, 0.0)
        assert ahead is not None
        decision = plan.decision_for(ahead)

        assert decision is not None
        assert DetailFactorName.TRAJECTORY in decision.factors
        assert decision.factor_scores.trajectory is not None
        assert "predicted-motion relevance" in decision.reason

    def test_a_missing_trajectory_drops_the_factor_rather_than_scoring_zero(self) -> None:
        plan = controller().plan(
            [assessed(track_id=1, risk_score=0.5, position=(5.0, 5.0, 0.0))],
            tracks=[at_position((5.0, 5.0, 0.0), track_id=1)],
        )
        grid = controller().tile_grid()
        index = grid.locate(5.0, 5.0)
        assert index is not None
        decision = plan.decision_for(index)

        assert decision is not None
        assert DetailFactorName.TRAJECTORY not in decision.factors
        assert decision.factor_scores.trajectory is None

    def test_the_near_future_weighs_more_than_the_far_future(self) -> None:
        plan = controller().plan(
            [assessed(track_id=1, risk_score=0.5, position=(-15.0, 0.0, 0.0))],
            tracks=[at_position((-15.0, 0.0, 0.0), track_id=1)],
            trajectories=[straight_trajectory((-15.0, 0.0), (8.0, 0.0), track_id=1)],
        )
        grid = controller().tile_grid()
        soon, later = grid.locate(-5.0, 0.0), grid.locate(5.0, 0.0)
        assert soon is not None and later is not None
        soon_decision, later_decision = plan.decision_for(soon), plan.decision_for(later)

        assert soon_decision is not None and later_decision is not None
        assert soon_decision.factor_scores.trajectory is not None
        assert later_decision.factor_scores.trajectory is not None
        assert soon_decision.factor_scores.trajectory > later_decision.factor_scores.trajectory


class TestDensityAndMotion:
    def test_more_objects_in_a_region_raise_its_priority(self) -> None:
        grid = controller().tile_grid()
        index = grid.locate(5.0, 5.0)
        assert index is not None

        def priority(count: int) -> float:
            assessments = [
                assessed(track_id=i, risk_score=0.4, uncertainty=0.2, position=(5.0, 5.0, 0.0))
                for i in range(count)
            ]
            tracks = [at_position((5.0, 5.0, 0.0), track_id=i) for i in range(count)]
            decision = controller().plan(assessments, tracks=tracks).decision_for(index)
            assert decision is not None and decision.detail_priority is not None
            return decision.detail_priority

        assert priority(3) > priority(1)

    def test_density_saturates_rather_than_dominating(self) -> None:
        grid = controller().tile_grid()
        index = grid.locate(5.0, 5.0)
        assert index is not None

        def density(count: int) -> float:
            assessments = [
                assessed(track_id=i, risk_score=0.4, position=(5.0, 5.0, 0.0)) for i in range(count)
            ]
            tracks = [at_position((5.0, 5.0, 0.0), track_id=i) for i in range(count)]
            decision = controller().plan(assessments, tracks=tracks).decision_for(index)
            assert decision is not None and decision.factor_scores.density is not None
            return decision.factor_scores.density

        assert density(4) == pytest.approx(1.0)
        assert density(40) == pytest.approx(1.0)

    def test_an_unmeasured_speed_drops_the_motion_factor(self) -> None:
        """Unknown speed is not a standstill (ADR-023), one phase later."""
        plan = controller().plan(
            [assessed(track_id=1, risk_score=0.5, speed_mps=None, position=(5.0, 5.0, 0.0))],
            tracks=[at_position((5.0, 5.0, 0.0), track_id=1, velocity=None)],
        )
        grid = controller().tile_grid()
        index = grid.locate(5.0, 5.0)
        assert index is not None
        decision = plan.decision_for(index)

        assert decision is not None
        assert DetailFactorName.MOTION not in decision.factors
        assert decision.factor_scores.motion is None

    def test_a_measured_standstill_scores_zero_motion_rather_than_dropping_it(self) -> None:
        plan = controller().plan(
            [assessed(track_id=1, risk_score=0.5, speed_mps=0.0, position=(5.0, 5.0, 0.0))],
            tracks=[at_position((5.0, 5.0, 0.0), track_id=1, velocity=(0.0, 0.0, 0.0))],
        )
        grid = controller().tile_grid()
        index = grid.locate(5.0, 5.0)
        assert index is not None
        decision = plan.decision_for(index)

        assert decision is not None
        assert DetailFactorName.MOTION in decision.factors
        assert decision.factor_scores.motion == pytest.approx(0.0)

    def test_a_faster_object_raises_the_motion_factor(self) -> None:
        grid = controller().tile_grid()
        index = grid.locate(5.0, 5.0)
        assert index is not None

        def motion(speed: float) -> float:
            decision = (
                controller()
                .plan(
                    [
                        assessed(
                            track_id=1, risk_score=0.5, speed_mps=speed, position=(5.0, 5.0, 0.0)
                        )
                    ],
                    tracks=[at_position((5.0, 5.0, 0.0), track_id=1)],
                )
                .decision_for(index)
            )
            assert decision is not None and decision.factor_scores.motion is not None
            return decision.factor_scores.motion

        assert motion(12.0) > motion(3.0)


class TestProximity:
    def test_a_nearer_region_scores_higher_proximity(self) -> None:
        near = controller().plan(
            [assessed(track_id=1, risk_score=0.5, position=(2.0, 2.0, 0.0))],
            tracks=[at_position((2.0, 2.0, 0.0), track_id=1)],
        )
        far = controller().plan(
            [assessed(track_id=1, risk_score=0.5, position=(18.0, 18.0, 0.0))],
            tracks=[at_position((18.0, 18.0, 0.0), track_id=1)],
        )
        grid = controller().tile_grid()
        near_index, far_index = grid.locate(2.0, 2.0), grid.locate(18.0, 18.0)
        assert near_index is not None and far_index is not None

        near_decision = near.decision_for(near_index)
        far_decision = far.decision_for(far_index)
        assert near_decision is not None and far_decision is not None
        assert near_decision.factor_scores.proximity is not None
        assert far_decision.factor_scores.proximity is not None

        assert near_decision.factor_scores.proximity > far_decision.factor_scores.proximity

    def test_proximity_alone_does_not_refine_an_empty_region(self) -> None:
        """Being near the ego is not evidence that anything is happening."""
        plan = controller().plan([])
        grid = controller().tile_grid()
        nearest = grid.locate(-1.0, -1.0)
        assert nearest is not None
        decision = plan.decision_for(nearest)

        assert decision is not None
        assert decision.level is ResolutionLevel.LOW
        assert decision.factor_scores.proximity is None


class TestSelectResolutionContract:
    """The abstract ``ResolutionController`` methods, exercised directly."""

    def test_cell_size_comes_from_map_configuration(self) -> None:
        instance = controller()

        assert instance.cell_size_m(ResolutionLevel.LOW) == 1.0
        assert instance.cell_size_m(ResolutionLevel.CRITICAL) == 0.1

    def test_a_context_with_no_objects_takes_the_base_level(self) -> None:
        context = ResolutionContext(x=0.0, y=0.0, distance_from_ego_m=0.0)

        assert controller().select_resolution(context) is ResolutionLevel.LOW

    def test_a_null_risk_score_is_not_read_as_zero(self) -> None:
        scored = ResolutionContext(
            x=0.0,
            y=0.0,
            distance_from_ego_m=5.0,
            risk_score=0.0,
            uncertainty=0.9,
            object_count=1,
            has_unknown_risk=False,
        )
        unknown = ResolutionContext(
            x=0.0,
            y=0.0,
            distance_from_ego_m=5.0,
            risk_score=None,
            uncertainty=0.9,
            object_count=1,
            has_unknown_risk=True,
        )
        instance = controller()

        assert instance.evaluate(unknown).value != instance.evaluate(scored).value
        assert instance.evaluate(unknown).scores.risk is None
        assert instance.evaluate(scored).scores.risk == pytest.approx(0.0)

    def test_a_missing_factor_renormalises_the_remaining_weights(self) -> None:
        """One factor at 0.8 alone must score 0.8, not 0.8 times its weight."""
        context = ResolutionContext(
            x=0.0, y=0.0, distance_from_ego_m=100.0, risk_score=0.8, object_count=1
        )
        priority = controller(
            weight_uncertainty=0.0,
            weight_proximity=0.0,
            weight_trajectory=0.0,
            weight_density=0.0,
            weight_motion=0.0,
        ).evaluate(context)

        assert priority.value == pytest.approx(0.8)

    def test_the_band_boundaries_follow_configuration(self) -> None:
        instance = controller()

        assert instance.band(0.0) is ResolutionLevel.LOW
        assert instance.band(0.35) is ResolutionLevel.MEDIUM
        assert instance.band(0.60) is ResolutionLevel.HIGH
        assert instance.band(0.85) is ResolutionLevel.CRITICAL
        assert instance.band(1.0) is ResolutionLevel.CRITICAL


class TestHysteresis:
    """Resolution must not flicker (ADR-039)."""

    def _priority_sequence(
        self, instance: HeuristicResolutionController, scores: list[float]
    ) -> list[ResolutionLevel]:
        """Drive one region through a sequence of risk scores."""
        grid = instance.tile_grid()
        index = grid.locate(5.0, 5.0)
        assert index is not None

        observed: list[ResolutionLevel] = []
        for score in scores:
            plan = instance.plan(
                [assessed(track_id=1, risk_score=score, uncertainty=0.2, position=(5.0, 5.0, 0.0))],
                tracks=[at_position((5.0, 5.0, 0.0), track_id=1)],
            )
            decision = plan.decision_for(index)
            assert decision is not None
            observed.append(decision.level)
        return observed

    def test_jitter_around_a_threshold_does_not_flip_the_level(self) -> None:
        """The failure this phase exists to avoid, asserted directly."""
        instance = controller()
        observed = self._priority_sequence(
            instance, [0.61, 0.59, 0.60, 0.58, 0.61, 0.59, 0.60, 0.58]
        )

        assert len(set(observed)) == 1

    def test_a_stable_score_gives_a_stable_level(self) -> None:
        instance = controller()
        observed = self._priority_sequence(instance, [0.5] * 6)

        assert len(set(observed)) == 1

    def test_a_sustained_rise_upgrades_on_the_first_frame(self) -> None:
        """Refinement is immediate: delaying detail is the expensive mistake."""
        instance = controller()
        observed = self._priority_sequence(instance, [0.1, 0.99])

        assert instance.cell_size_m(observed[1]) < instance.cell_size_m(observed[0])

    def test_a_sustained_fall_eventually_downgrades(self) -> None:
        instance = controller()
        observed = self._priority_sequence(instance, [0.99, 0.01, 0.01, 0.01, 0.01, 0.01])

        assert instance.cell_size_m(observed[-1]) > instance.cell_size_m(observed[0])

    def test_a_downgrade_waits_for_the_dwell_time(self) -> None:
        instance = controller(min_dwell_frames=3)
        observed = self._priority_sequence(instance, [0.99, 0.01, 0.01, 0.01])

        # Frames 1 and 2 are held; the third consecutive proposal applies.
        assert observed[1] is observed[0]
        assert observed[2] is observed[0]
        assert observed[3] is not observed[0]

    def test_a_hold_is_recorded_rather_than_hidden(self) -> None:
        instance = controller()
        grid = instance.tile_grid()
        index = grid.locate(5.0, 5.0)
        assert index is not None

        instance.plan(
            [assessed(track_id=1, risk_score=0.99, position=(5.0, 5.0, 0.0))],
            tracks=[at_position((5.0, 5.0, 0.0), track_id=1)],
        )
        plan = instance.plan(
            [assessed(track_id=1, risk_score=0.01, position=(5.0, 5.0, 0.0))],
            tracks=[at_position((5.0, 5.0, 0.0), track_id=1)],
        )
        decision = plan.decision_for(index)

        assert decision is not None
        assert TileDecisionReason.DWELL_HOLD in decision.reasons
        assert "held" in decision.reason

    def test_a_disappearing_object_returns_the_region_to_the_base_level(self) -> None:
        """Stale detail must not persist forever."""
        instance = controller()
        grid = instance.tile_grid()
        index = grid.locate(5.0, 5.0)
        assert index is not None

        instance.plan(
            [assessed(track_id=1, risk_score=0.99, position=(5.0, 5.0, 0.0))],
            tracks=[at_position((5.0, 5.0, 0.0), track_id=1)],
        )
        for _ in range(6):
            plan = instance.plan([])

        decision = plan.decision_for(index)
        assert decision is not None
        assert decision.level is ResolutionLevel.LOW

    def test_a_change_is_reported_against_the_previous_level(self) -> None:
        instance = controller()
        grid = instance.tile_grid()
        index = grid.locate(5.0, 5.0)
        assert index is not None

        first = instance.plan([])
        second = instance.plan(
            [assessed(track_id=1, risk_score=0.99, position=(5.0, 5.0, 0.0))],
            tracks=[at_position((5.0, 5.0, 0.0), track_id=1)],
        )

        assert first.changed_tile_count == 0
        assert all(d.previous_level is None for d in first.decisions)

        decision = second.decision_for(index)
        assert decision is not None
        assert decision.previous_level is ResolutionLevel.LOW
        assert decision.changed is True

    def test_a_reset_forgets_every_remembered_level(self) -> None:
        instance = controller()
        instance.plan(
            [assessed(track_id=1, risk_score=0.99, position=(5.0, 5.0, 0.0))],
            tracks=[at_position((5.0, 5.0, 0.0), track_id=1)],
        )
        instance.reset()

        assert instance.frame_index == 0
        assert all(instance.current_level(i) is None for i in range(16))


class TestBudget:
    def test_the_cell_ceiling_is_respected(self) -> None:
        instance = controller(max_total_cells=5_000, max_fine_tiles=64)
        plan = instance.plan(
            [
                assessed(track_id=i, risk_score=1.0, uncertainty=1.0, position=(x, y, 0.0))
                for i, (x, y) in enumerate([(-15.0, -15.0), (5.0, 5.0), (15.0, -5.0)])
            ],
            tracks=[
                at_position((x, y, 0.0), track_id=i)
                for i, (x, y) in enumerate([(-15.0, -15.0), (5.0, 5.0), (15.0, -5.0)])
            ],
        )

        assert plan.total_cell_count <= 5_000
        assert plan.budget.total_cell_count == plan.total_cell_count

    def test_a_demotion_is_reported_rather_than_hidden(self) -> None:
        instance = controller(max_total_cells=2_000)
        plan = instance.plan(
            [assessed(track_id=1, risk_score=1.0, uncertainty=1.0, position=(5.0, 5.0, 0.0))],
            tracks=[at_position((5.0, 5.0, 0.0), track_id=1)],
        )

        assert plan.budget.demoted_tile_count > 0
        demoted = [d for d in plan.decisions if TileDecisionReason.BUDGET_DEMOTED in d.reasons]
        assert demoted
        assert all("budget" in d.reason for d in demoted)

    def test_the_fine_region_ceiling_is_respected(self) -> None:
        instance = controller(max_fine_tiles=1)
        plan = instance.plan(
            [
                assessed(track_id=i, risk_score=1.0, uncertainty=1.0, position=(x, y, 0.0))
                for i, (x, y) in enumerate([(-15.0, -15.0), (5.0, 5.0), (15.0, 15.0)])
            ],
            tracks=[
                at_position((x, y, 0.0), track_id=i)
                for i, (x, y) in enumerate([(-15.0, -15.0), (5.0, 5.0), (15.0, 15.0)])
            ],
        )
        fine = [d for d in plan.decisions if d.is_fine]

        assert len(fine) <= 1
        assert plan.budget.fine_tile_count == len(fine)

    def test_the_highest_priority_region_keeps_its_detail_when_the_budget_binds(self) -> None:
        """Give up detail where it matters least, not arbitrarily."""
        instance = controller(max_fine_tiles=1)
        positions = [(-15.0, -15.0), (5.0, 5.0)]
        plan = instance.plan(
            [
                assessed(track_id=0, risk_score=0.4, uncertainty=0.2, position=(-15.0, -15.0, 0.0)),
                assessed(track_id=1, risk_score=1.0, uncertainty=1.0, position=(5.0, 5.0, 0.0)),
            ],
            tracks=[at_position((x, y, 0.0), track_id=i) for i, (x, y) in enumerate(positions)],
        )
        fine = [d for d in plan.decisions if d.is_fine]
        grid = instance.tile_grid()
        important = grid.locate(5.0, 5.0)

        assert [d.tile_index for d in fine] == [important]

    def test_the_budget_report_states_whether_it_was_met(self) -> None:
        plan = controller().plan([])

        assert plan.budget.within_budget is True
        assert plan.budget.tile_count == 16


class TestAccounting:
    def test_every_assessment_is_influencing_or_excluded(self) -> None:
        assessments = [
            assessed(track_id=0, position=(5.0, 5.0, 0.0)),
            assessed(track_id=1, position=(500.0, 500.0, 0.0)),
        ]
        tracks = [
            at_position((5.0, 5.0, 0.0), track_id=0),
            at_position((500.0, 500.0, 0.0), track_id=1),
        ]
        plan = controller().plan(assessments, tracks=tracks)

        assert plan.considered_assessment_count == 2
        assert plan.influencing_assessment_count + len(plan.excluded) == 2

    def test_an_object_outside_the_map_is_excluded_with_a_reason(self) -> None:
        plan = controller().plan(
            [assessed(track_id=1, position=(500.0, 500.0, 0.0))],
            tracks=[at_position((500.0, 500.0, 0.0), track_id=1)],
        )

        assert [e.reason for e in plan.excluded] == [ExclusionReason.OUT_OF_BOUNDS]
        assert plan.excluded[0].track_id == 1

    def test_an_assessment_without_a_track_is_excluded_not_guessed(self) -> None:
        plan = controller().plan([assessed(track_id=9, position=(5.0, 5.0, 0.0))], tracks=[])

        assert [e.reason for e in plan.excluded] == [ExclusionReason.MISSING_POSITION]

    def test_assessments_beyond_the_limit_are_recorded_as_excluded(self) -> None:
        instance = controller(max_influencing_objects=2)
        assessments = [assessed(track_id=i, position=(5.0, 5.0, 0.0)) for i in range(5)]
        tracks = [at_position((5.0, 5.0, 0.0), track_id=i) for i in range(5)]
        plan = instance.plan(assessments, tracks=tracks)

        limited = [e for e in plan.excluded if e.reason is ExclusionReason.LIMIT_EXCEEDED]
        assert len(limited) == 3
        assert plan.considered_assessment_count == 5


class TestDeterminism:
    def test_identical_input_produces_identical_decisions(self) -> None:
        def run() -> list[tuple[int, str, float | None]]:
            instance = controller()
            plan = instance.plan(
                [
                    assessed(track_id=i, risk_score=0.5 + i * 0.1, position=(x, y, 0.0))
                    for i, (x, y) in enumerate([(5.0, 5.0), (-8.0, 3.0), (12.0, -14.0)])
                ],
                tracks=[
                    at_position((x, y, 0.0), track_id=i)
                    for i, (x, y) in enumerate([(5.0, 5.0), (-8.0, 3.0), (12.0, -14.0)])
                ],
            )
            return [(d.tile_index, d.level.value, d.detail_priority) for d in plan.decisions]

        assert run() == run()

    def test_the_order_assessments_arrive_in_does_not_matter(self) -> None:
        """No dependence on dictionary or argument ordering."""
        positions = [(5.0, 5.0), (-8.0, 3.0), (12.0, -14.0)]
        assessments = [
            assessed(track_id=i, risk_score=0.5 + i * 0.1, position=(x, y, 0.0))
            for i, (x, y) in enumerate(positions)
        ]
        tracks = [at_position((x, y, 0.0), track_id=i) for i, (x, y) in enumerate(positions)]

        forward = controller().plan(assessments, tracks=tracks)
        backward = controller().plan(list(reversed(assessments)), tracks=list(reversed(tracks)))

        assert [(d.tile_index, d.level) for d in forward.decisions] == [
            (d.tile_index, d.level) for d in backward.decisions
        ]

    def test_ties_are_broken_by_region_index(self) -> None:
        """Two identical regions must resolve the same way every run."""
        instance = controller(max_fine_tiles=1)
        positions = [(-15.0, -15.0), (15.0, 15.0)]
        plan = instance.plan(
            [
                assessed(track_id=i, risk_score=0.9, uncertainty=0.5, position=(x, y, 0.0))
                for i, (x, y) in enumerate(positions)
            ],
            tracks=[at_position((x, y, 0.0), track_id=i) for i, (x, y) in enumerate(positions)],
        )
        fine = [d.tile_index for d in plan.decisions if d.is_fine]

        assert len(fine) <= 1
        assert plan.decisions == sorted(plan.decisions, key=lambda d: d.tile_index)


class TestExplanations:
    def test_an_explanation_is_generated_from_the_computed_values(self) -> None:
        plan = controller().plan(
            [assessed(track_id=1, risk_score=0.7, uncertainty=0.4, position=(5.0, 5.0, 0.0))],
            tracks=[at_position((5.0, 5.0, 0.0), track_id=1)],
        )
        grid = controller().tile_grid()
        index = grid.locate(5.0, 5.0)
        assert index is not None
        decision = plan.decision_for(index)

        assert decision is not None
        assert "detail priority" in decision.reason
        assert "risk 0.70" in decision.reason

    @pytest.mark.parametrize(
        "forbidden", ["probability", "calibrated", "validated", "guaranteed", "safe"]
    )
    def test_an_explanation_never_overclaims(self, forbidden: str) -> None:
        """A heuristic has no right to these words."""
        plan = controller().plan(
            [
                assessed(track_id=0, risk_score=0.9, uncertainty=0.8, position=(5.0, 5.0, 0.0)),
                unknown_risk(track_id=1, position=(-12.0, 8.0, 0.0)),
            ],
            tracks=[
                at_position((5.0, 5.0, 0.0), track_id=0),
                at_position((-12.0, 8.0, 0.0), track_id=1, velocity=None),
            ],
            trajectories=[straight_trajectory((5.0, 5.0), (4.0, 0.0), track_id=0)],
        )

        for decision in plan.decisions:
            assert forbidden not in decision.reason.lower()
            assert forbidden not in decision.resolution.reason.lower()


class TestAdaptationDisabled:
    def test_disabling_adaptation_holds_every_region_at_the_base_level(self) -> None:
        plan = controller(enabled=False).plan(
            [assessed(track_id=1, risk_score=1.0, uncertainty=1.0, position=(5.0, 5.0, 0.0))],
            tracks=[at_position((5.0, 5.0, 0.0), track_id=1)],
        )

        assert set(levels(plan).values()) == {ResolutionLevel.LOW}

    def test_the_reason_says_adaptation_was_disabled(self) -> None:
        """Switched off must not look the same as nothing to do."""
        plan = controller(enabled=False).plan(
            [assessed(track_id=1, risk_score=1.0, position=(5.0, 5.0, 0.0))],
            tracks=[at_position((5.0, 5.0, 0.0), track_id=1)],
        )

        assert all(TileDecisionReason.ADAPTATION_DISABLED in d.reasons for d in plan.decisions)
        assert "adaptation disabled" in plan.decisions[0].reason


class TestPlanMetadata:
    def test_a_plan_records_its_controller_and_policy(self) -> None:
        plan = controller().plan([], frame_id=7, sensor_id="roof_lidar")

        assert plan.controller == "heuristic_resolution_controller_v1"
        assert plan.policy_model == "heuristic_weighted_detail_priority"
        assert plan.is_baseline is True
        assert plan.frame_id == 7
        assert plan.sensor_id == "roof_lidar"

    def test_a_plan_carries_the_configuration_that_produced_it(self) -> None:
        configuration = controller().plan([]).configuration

        assert configuration.tile_size_m == 10.0
        assert configuration.base_level is ResolutionLevel.LOW
        assert configuration.unknown_risk_min_level is ResolutionLevel.MEDIUM
        assert configuration.level_cell_sizes[ResolutionLevel.CRITICAL] == 0.1

    def test_the_frame_index_advances_and_is_reported(self) -> None:
        instance = controller()

        assert instance.plan([]).frame_index == 1
        assert instance.plan([]).frame_index == 2

    def test_the_measured_duration_is_recorded(self) -> None:
        plan = controller().plan([])

        assert plan.duration_ms >= 0.0
