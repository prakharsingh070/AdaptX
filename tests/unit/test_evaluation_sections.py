"""Evaluation sections on hand-built runs (Phase 11).

Each test builds a small run record whose right answer is known exactly, then
asks one section of the evaluator for it. A perfect pipeline scores
perfectly; an empty one scores nothing; a missing quantity is null with a
reason, never zero.
"""

from __future__ import annotations

import pytest

from adaptx.evaluation import (
    EvaluationConfiguration,
    EvaluationInputError,
    EvaluationReport,
    MetricStatus,
    compare_reports,
    evaluate,
    load_dataset,
    render,
)
from adaptx.evaluation.adaptive import evaluate_adaptive
from adaptx.evaluation.dataset import EvaluationDataset
from adaptx.evaluation.mapping import evaluate_mapping
from adaptx.evaluation.matching import FrameMatching
from adaptx.evaluation.prediction import evaluate_prediction
from adaptx.evaluation.resource import evaluate_resource
from adaptx.evaluation.risk import evaluate_risk
from adaptx.evaluation.tracking import evaluate_detection, evaluate_tracking, match_tracks
from adaptx.models.adaptive_resolution import TileDecisionReason
from adaptx.models.common import DataSource, ObjectClass, Vector3
from adaptx.models.map import ResolutionLevel
from adaptx.models.prediction import PredictionStatus
from adaptx.models.risk import RiskLevel
from adaptx.models.tracking import TrackStatus
from adaptx.scenarios.models import ScenarioState
from tests.fixtures.evaluation import (
    TIMESTEP_S,
    ActorSpec,
    FrameSpec,
    RiskSpec,
    RunBuilder,
    TrackSpec,
    levels,
    tile_index_for,
)

CONFIG = EvaluationConfiguration()


def approach(frames: int = 6, *, offset_m: float = 0.0, speed_mps: float = -8.0) -> RunBuilder:
    """One actor closing along +x at ``speed_mps``, tracked with a fixed offset."""
    builder = RunBuilder({"target": 5})
    for i in range(frames):
        x = 10.0 + speed_mps * i * TIMESTEP_S
        builder.frame(
            FrameSpec(
                tracks=[TrackSpec(0, x + offset_m, 0.0, velocity=(speed_mps, 0.0))],
                actors=[ActorSpec(5, x, 0.0)],
            )
        )
    return builder


def sections(
    builder: RunBuilder, config: EvaluationConfiguration = CONFIG
) -> tuple[EvaluationDataset, dict[int, FrameMatching]]:
    dataset = load_dataset(builder.build(), include_map_actors=config.include_map_actors)
    matching = match_tracks(dataset, gate_m=config.primary_gate_m)
    return dataset, matching


class TestDataset:
    def test_the_reference_velocity_is_the_finite_difference_not_the_recorded_one(self) -> None:
        dataset, _ = sections(approach(3))
        first, second = dataset.frames[0].actors[0], dataset.frames[1].actors[0]
        assert first.reference_velocity is None  # null on the first frame, not zero
        assert second.reference_velocity is not None
        assert second.reference_velocity.x == pytest.approx(-8.0)
        assert second.reference_velocity.y == pytest.approx(0.0)
        # The record's own velocity field says something else entirely.
        recorded = dataset.frames[1].truth.others()[0].velocity
        assert recorded.x == 0.0 and recorded.z == -5.0

    def test_ground_truth_is_moved_into_the_sensor_frame(self) -> None:
        builder = RunBuilder({"target": 5}).frame(
            FrameSpec(tracks=[TrackSpec(0, 10.0, 0.0)], actors=[ActorSpec(5, 10.0, 0.0, z=0.0)])
        )
        dataset, _ = sections(builder)
        actor = dataset.frames[0].actors[0]
        assert actor.position.z == pytest.approx(-1.8)  # ego origin is 1.8 m below the sensor
        assert actor.position.x == 10.0

    def test_only_scenario_actors_are_ground_truth_by_default(self) -> None:
        builder = RunBuilder({"target": 5}).frame(
            FrameSpec(
                tracks=[TrackSpec(0, 10.0, 0.0)],
                actors=[ActorSpec(5, 10.0, 0.0), ActorSpec(9, 15.0, 5.0, type_id="traffic.stop")],
            )
        )
        dataset, _ = sections(builder)
        assert [a.simulator_actor_id for a in dataset.frames[0].actors] == [5]
        everything = load_dataset(builder.build(), include_map_actors=True)
        assert [a.simulator_actor_id for a in everything.frames[0].actors] == [5, 9]
        assert everything.frames[0].actors[1].actor_id == "traffic.stop"

    def test_an_actor_beyond_the_map_bounds_is_ineligible_but_kept(self) -> None:
        builder = RunBuilder({"target": 5}).frame(
            FrameSpec(tracks=[], actors=[ActorSpec(5, 30.0, 0.0)])
        )
        dataset, _ = sections(builder)
        actor = dataset.frames[0].actors[0]
        assert not actor.eligible
        assert actor.ineligibility_reason == "outside the map bounds"
        assert dataset.frames[0].eligible_actors == ()

    def test_a_counts_only_record_cannot_be_evaluated(self) -> None:
        record = approach(2).build()
        stripped = record.model_copy(
            update={
                "frames": [f.model_copy(update={"outputs": None}) for f in record.frames],
            }
        )
        with pytest.raises(EvaluationInputError, match="not the pipeline outputs"):
            load_dataset(stripped)

    def test_a_record_with_no_processed_frame_cannot_be_evaluated(self) -> None:
        record = approach(2).build()
        empty = record.model_copy(
            update={
                "frames": [
                    f.model_copy(update={"outputs": None, "pipeline": None}) for f in record.frames
                ]
            }
        )
        with pytest.raises(EvaluationInputError, match="no processed frames"):
            load_dataset(empty)

    def test_the_ego_is_checked_for_motion(self) -> None:
        dataset, _ = sections(approach(3))
        assert dataset.ego_stationary is True


class TestTracking:
    def test_a_perfect_track_scores_perfectly(self) -> None:
        dataset, matching = sections(approach(6))
        result = evaluate_tracking(dataset, CONFIG, matching)
        assert result.status is MetricStatus.MEASURED
        for gate in result.gates:
            assert gate.match_rate == 1.0
            assert gate.eligible_pairs == 6 and gate.matched_pairs == 6
            assert gate.position_error_planar_m.max == pytest.approx(0.0)
            assert gate.class_agreement_rate == 1.0
        assert result.continuity[0].coverage == 1.0
        assert result.continuity[0].id_switches == 0
        assert result.continuity[0].fragments == 1
        assert result.unlabelled_track_frames == 0

    def test_a_constant_offset_is_measured_exactly(self) -> None:
        dataset, matching = sections(approach(4, offset_m=0.3))
        result = evaluate_tracking(dataset, CONFIG, matching)
        primary = next(g for g in result.gates if g.gate_m == CONFIG.primary_gate_m)
        assert primary.position_error_planar_m.mean == pytest.approx(0.3)
        assert primary.position_error_planar_m.rmse == pytest.approx(0.3)
        assert primary.position_error_3d_m.mean == pytest.approx(0.3)

    def test_the_gate_changes_the_answer_and_both_are_reported(self) -> None:
        dataset, matching = sections(approach(4, offset_m=1.5))
        result = evaluate_tracking(dataset, CONFIG, matching)
        by_gate = {g.gate_m: g for g in result.gates}
        assert by_gate[1.0].match_rate == 0.0
        assert by_gate[2.0].match_rate == 1.0
        assert by_gate[4.0].match_rate == 1.0

    def test_velocity_error_uses_the_finite_difference_reference(self) -> None:
        builder = RunBuilder({"target": 5})
        for i in range(3):
            x = 10.0 - 8.0 * i * TIMESTEP_S
            builder.frame(
                FrameSpec(
                    tracks=[TrackSpec(0, x, 0.0, velocity=(-6.0, 0.0))],
                    actors=[ActorSpec(5, x, 0.0)],
                )
            )
        dataset, matching = sections(builder)
        primary = next(
            g
            for g in evaluate_tracking(dataset, CONFIG, matching).gates
            if g.gate_m == CONFIG.primary_gate_m
        )
        assert primary.velocity_error_mps.count == 2  # no reference on frame 0
        assert primary.velocity_reference_missing_pairs == 1
        assert primary.velocity_error_mps.mean == pytest.approx(2.0)

    def test_a_null_velocity_is_counted_and_never_scored(self) -> None:
        builder = RunBuilder({"target": 5})
        for _ in range(3):
            builder.frame(
                FrameSpec(
                    tracks=[TrackSpec(0, 10.0, 0.0, velocity=None)],
                    actors=[ActorSpec(5, 10.0, 0.0)],
                )
            )
        dataset, matching = sections(builder)
        primary = next(
            g
            for g in evaluate_tracking(dataset, CONFIG, matching).gates
            if g.gate_m == CONFIG.primary_gate_m
        )
        assert primary.velocity_null_pairs == 3
        assert primary.velocity_error_mps.count == 0
        assert primary.velocity_error_mps.mean is None

    def test_an_identity_switch_and_a_gap_are_counted(self) -> None:
        builder = RunBuilder({"target": 5})
        for track_id in [0, 0, None, 1, 1]:
            tracks = [] if track_id is None else [TrackSpec(track_id, 10.0, 0.0)]
            builder.frame(FrameSpec(tracks=tracks, actors=[ActorSpec(5, 10.0, 0.0)]))
        dataset, matching = sections(builder)
        continuity = evaluate_tracking(dataset, CONFIG, matching).continuity[0]
        assert continuity.frames_eligible == 5 and continuity.frames_matched == 4
        assert continuity.coverage == pytest.approx(0.8)
        assert continuity.track_ids == [0, 1]
        assert continuity.id_switches == 1
        assert continuity.fragments == 2 and continuity.longest_fragment_frames == 2

    def test_an_empty_pipeline_output_scores_zero_not_null(self) -> None:
        builder = RunBuilder({"target": 5})
        for _ in range(3):
            builder.frame(FrameSpec(tracks=[], actors=[ActorSpec(5, 10.0, 0.0)]))
        dataset, matching = sections(builder)
        result = evaluate_tracking(dataset, CONFIG, matching)
        assert all(g.match_rate == 0.0 for g in result.gates)
        assert result.continuity[0].coverage == 0.0

    def test_no_ground_truth_actor_means_unavailable_not_zero(self) -> None:
        builder = RunBuilder({}).frame(FrameSpec(tracks=[TrackSpec(0, 10.0, 0.0)]))
        dataset, matching = sections(builder)
        result = evaluate_tracking(dataset, CONFIG, matching)
        assert result.status is MetricStatus.UNAVAILABLE
        assert result.gates == []

    def test_an_unmatched_track_is_unlabelled_not_false(self) -> None:
        builder = RunBuilder({"target": 5}).frame(
            FrameSpec(
                tracks=[TrackSpec(0, 10.0, 0.0), TrackSpec(7, -15.0, 15.0)],
                actors=[ActorSpec(5, 10.0, 0.0)],
            )
        )
        dataset, matching = sections(builder)
        result = evaluate_tracking(dataset, CONFIG, matching)
        assert result.unlabelled_track_frames == 1
        assert result.unlabelled_track_ids == 1
        assert not hasattr(result, "false_positives")
        assert not hasattr(result, "precision")

    def test_a_track_on_an_ineligible_actor_is_not_unlabelled(self) -> None:
        builder = RunBuilder({"target": 5}).frame(
            FrameSpec(tracks=[TrackSpec(0, 30.0, 0.0)], actors=[ActorSpec(5, 30.0, 0.0)])
        )
        dataset, matching = sections(builder)
        result = evaluate_tracking(dataset, CONFIG, matching)
        assert result.status is MetricStatus.UNAVAILABLE  # nothing eligible
        assert result.unlabelled_track_frames == 0

    def test_the_matched_status_distribution_is_recorded(self) -> None:
        builder = RunBuilder({"target": 5})
        for status in (TrackStatus.TENTATIVE, TrackStatus.CONFIRMED, TrackStatus.COASTING):
            builder.frame(
                FrameSpec(
                    tracks=[TrackSpec(0, 10.0, 0.0, status=status)],
                    actors=[ActorSpec(5, 10.0, 0.0)],
                )
            )
        dataset, matching = sections(builder)
        counts = evaluate_tracking(dataset, CONFIG, matching).continuity[0].matched_status_counts
        assert counts == {"tentative": 1, "confirmed": 1, "coasting": 1}


class TestDetection:
    def test_recall_counts_frames_with_a_detection_at_the_actor(self) -> None:
        builder = RunBuilder({"target": 5})
        builder.frame(FrameSpec(detections=[(10.0, 0.0)], actors=[ActorSpec(5, 10.0, 0.0)]))
        builder.frame(FrameSpec(detections=[], actors=[ActorSpec(5, 10.0, 0.0)]))
        builder.frame(FrameSpec(detections=[(10.0, 0.5)], actors=[ActorSpec(5, 10.0, 0.0)]))
        dataset, _ = sections(builder)
        result = evaluate_detection(dataset, CONFIG)
        by_gate = {g.gate_m: g for g in result.gates}
        assert by_gate[1.0].match_rate == pytest.approx(2 / 3)
        assert by_gate[1.0].position_error_planar_m.mean == pytest.approx(0.25)


class TestPrediction:
    def _run(self, error_m: float, frames: int = 12) -> RunBuilder:
        """A track predicted with constant velocity; the actor follows exactly."""
        builder = RunBuilder({"target": 5})
        speed = -8.0
        for i in range(frames):
            x = 10.0 + speed * i * TIMESTEP_S
            trajectory = [(0.0, x, 0.0)] + [
                (k * 0.25, x + speed * k * 0.25 + error_m, 0.0) for k in (1, 2)
            ]
            builder.frame(
                FrameSpec(
                    tracks=[TrackSpec(0, x, 0.0, velocity=(speed, 0.0))],
                    actors=[ActorSpec(5, x, 0.0)],
                    trajectories={0: trajectory},
                )
            )
        return builder

    def test_an_exact_prediction_has_zero_ade_and_fde(self) -> None:
        dataset, matching = sections(self._run(0.0))
        result = evaluate_prediction(dataset, CONFIG, matching)
        assert result.ade_m.max == pytest.approx(0.0)
        assert result.fde_m.max == pytest.approx(0.0)

    def test_a_known_error_is_measured_exactly(self) -> None:
        dataset, matching = sections(self._run(1.0))
        result = evaluate_prediction(dataset, CONFIG, matching)
        assert result.ade_m.mean == pytest.approx(1.0)
        assert result.fde_m.mean == pytest.approx(1.0)
        assert set(result.error_by_offset_m) == {"0.25", "0.50"}

    def test_only_trajectories_with_future_ground_truth_are_evaluated(self) -> None:
        # 12 frames; offsets reach 10 frames ahead, so frames 0..1 evaluate fully,
        # frames 2..6 partially (only t+0.25 = 5 frames), frames 7..11 not at all.
        dataset, matching = sections(self._run(0.0))
        result = evaluate_prediction(dataset, CONFIG, matching)
        assert result.trajectories_in_record == 12
        assert result.trajectories_evaluated == 7
        assert result.skipped == {"no_ground_truth_within_run": 5}
        assert result.status is MetricStatus.PARTIAL
        assert result.horizon_coverage.min == pytest.approx(0.5)
        assert result.horizon_coverage.max == pytest.approx(1.0)

    def test_the_start_point_is_not_prediction_error(self) -> None:
        builder = RunBuilder({"target": 5})
        for _ in range(6):
            builder.frame(
                FrameSpec(
                    tracks=[TrackSpec(0, 10.5, 0.0)],
                    actors=[ActorSpec(5, 10.0, 0.0)],
                    trajectories={0: [(0.0, 10.5, 0.0), (0.25, 10.0, 0.0)]},
                )
            )
        dataset, matching = sections(builder)
        result = evaluate_prediction(dataset, CONFIG, matching)
        assert result.ade_m.mean == pytest.approx(0.0)  # the 0.5 m at t+0 is tracking error

    def test_offsets_off_the_frame_grid_are_skipped_not_interpolated(self) -> None:
        builder = RunBuilder({"target": 5})
        for _ in range(6):
            builder.frame(
                FrameSpec(
                    tracks=[TrackSpec(0, 10.0, 0.0)],
                    actors=[ActorSpec(5, 10.0, 0.0)],
                    trajectories={0: [(0.0, 10.0, 0.0), (0.03, 10.0, 0.0)]},
                )
            )
        dataset, matching = sections(builder)
        result = evaluate_prediction(dataset, CONFIG, matching)
        assert result.trajectories_evaluated == 0
        assert result.skipped == {"offsets_not_on_frame_boundary": 6}
        assert result.status is MetricStatus.UNAVAILABLE

    def test_a_trajectory_for_an_unmatched_track_is_skipped(self) -> None:
        builder = RunBuilder({"target": 5})
        for _ in range(6):
            builder.frame(
                FrameSpec(
                    tracks=[TrackSpec(0, -15.0, 15.0)],
                    actors=[ActorSpec(5, 10.0, 0.0)],
                    trajectories={0: [(0.0, -15.0, 15.0), (0.25, -15.0, 15.0)]},
                )
            )
        dataset, matching = sections(builder)
        result = evaluate_prediction(dataset, CONFIG, matching)
        assert result.skipped == {"track_unmatched_on_source_frame": 6}

    def test_predictor_skips_are_carried_from_the_record(self) -> None:
        builder = RunBuilder({"target": 5}).frame(
            FrameSpec(
                tracks=[TrackSpec(0, 10.0, 0.0, velocity=None)],
                actors=[ActorSpec(5, 10.0, 0.0)],
                skipped={0: PredictionStatus.INSUFFICIENT_VELOCITY},
            )
        )
        dataset, matching = sections(builder)
        result = evaluate_prediction(dataset, CONFIG, matching)
        assert result.status is MetricStatus.NOT_APPLICABLE
        assert result.predictor_skips == {"insufficient_velocity": 1}
        assert result.ade_m.count == 0 and result.ade_m.mean is None


class TestRisk:
    def _run(self, levels_by_frame: list[RiskLevel], *, distance: float = 15.0) -> RunBuilder:
        builder = RunBuilder({"target": 5})
        for level in levels_by_frame:
            score = (
                None
                if level is RiskLevel.UNKNOWN
                else {"low": 0.2, "medium": 0.5, "high": 0.7, "critical": 0.9}[level.value]
            )
            builder.frame(
                FrameSpec(
                    tracks=[TrackSpec(0, distance, 0.0)],
                    actors=[ActorSpec(5, distance, 0.0)],
                    risk=[RiskSpec(0, level, score)],
                )
            )
        return builder

    def test_alert_recall_over_event_frames(self) -> None:
        dataset, matching = sections(
            self._run([RiskLevel.LOW, RiskLevel.HIGH, RiskLevel.HIGH, RiskLevel.MEDIUM])
        )
        actor = evaluate_risk(dataset, CONFIG, matching).actors[0]
        assert actor.event_frames == 4  # 15 m is inside the 20 m band
        assert actor.alert_recall == pytest.approx(0.5)
        assert actor.risk_level_counts == {"low": 1, "high": 2, "medium": 1}

    def test_unknown_is_counted_and_never_alerts_or_scores(self) -> None:
        dataset, matching = sections(self._run([RiskLevel.UNKNOWN, RiskLevel.UNKNOWN]))
        actor = evaluate_risk(dataset, CONFIG, matching).actors[0]
        assert actor.unknown_risk_frames == 2
        assert actor.scored_frames == 0
        assert actor.alert_recall == 0.0
        assert actor.ordering_concordance is None
        assert actor.risk_level_counts == {"unknown": 2}

    def test_lead_time_is_positive_when_the_alert_precedes_the_event(self) -> None:
        builder = RunBuilder({"target": 5})
        distances = [19.0, 15.0, 12.0, 9.0, 7.0]  # all inside the 20 m test map
        for i, distance in enumerate(distances):
            level = RiskLevel.HIGH if i >= 1 else RiskLevel.LOW
            builder.frame(
                FrameSpec(
                    tracks=[TrackSpec(0, distance, 0.0)],
                    actors=[ActorSpec(5, distance, 0.0)],
                    risk=[RiskSpec(0, level, 0.7 if level is RiskLevel.HIGH else 0.2)],
                )
            )
        config = EvaluationConfiguration(proximity_event_m=10.0)
        dataset, matching = sections(builder, config)
        actor = evaluate_risk(dataset, config, matching).actors[0]
        assert actor.first_event_time_s == pytest.approx(3 * TIMESTEP_S)
        assert actor.first_alert_time_s == pytest.approx(1 * TIMESTEP_S)
        assert actor.lead_time_s == pytest.approx(2 * TIMESTEP_S)
        assert actor.early_alert_frames == 2  # alerting at 15 m and 12 m, outside the band

    def test_lead_time_is_negative_when_the_alert_comes_late(self) -> None:
        builder = RunBuilder({"target": 5})
        for i, distance in enumerate([19.0, 18.0, 17.0]):
            level = RiskLevel.HIGH if i == 2 else RiskLevel.LOW
            builder.frame(
                FrameSpec(
                    tracks=[TrackSpec(0, distance, 0.0)],
                    actors=[ActorSpec(5, distance, 0.0)],
                    risk=[RiskSpec(0, level, 0.7 if level is RiskLevel.HIGH else 0.2)],
                )
            )
        dataset, matching = sections(builder)
        actor = evaluate_risk(dataset, CONFIG, matching).actors[0]
        assert actor.lead_time_s == pytest.approx(-2 * TIMESTEP_S)

    def test_ordering_concordance_reads_the_score_against_distance(self) -> None:
        builder = RunBuilder({"target": 5})
        for distance, score in [(18.0, 0.2), (14.0, 0.5), (10.0, 0.9)]:
            builder.frame(
                FrameSpec(
                    tracks=[TrackSpec(0, distance, 0.0)],
                    actors=[ActorSpec(5, distance, 0.0)],
                    risk=[RiskSpec(0, RiskLevel.MEDIUM, score)],
                )
            )
        dataset, matching = sections(builder)
        actor = evaluate_risk(dataset, CONFIG, matching).actors[0]
        assert actor.ordering_pairs == 3 and actor.ordering_concordance == 1.0

    def test_no_event_is_partial_with_a_reason(self) -> None:
        dataset, matching = sections(self._run([RiskLevel.LOW] * 3, distance=15.0), CONFIG)
        config = EvaluationConfiguration(proximity_event_m=5.0)
        result = evaluate_risk(dataset, config, matching)
        assert result.status is MetricStatus.PARTIAL
        assert result.reason is not None and "5.0 m" in result.reason
        assert result.actors[0].alert_recall is None
        assert result.actors[0].lead_time_s is None

    def test_alerts_on_unlabelled_tracks_are_counted_only(self) -> None:
        builder = RunBuilder({"target": 5}).frame(
            FrameSpec(
                tracks=[TrackSpec(0, 15.0, 0.0), TrackSpec(3, -15.0, 15.0)],
                actors=[ActorSpec(5, 15.0, 0.0)],
                risk=[RiskSpec(0, RiskLevel.LOW, 0.2), RiskSpec(3, RiskLevel.CRITICAL, 0.95)],
            )
        )
        dataset, matching = sections(builder)
        result = evaluate_risk(dataset, CONFIG, matching)
        assert result.unlabelled_alert_track_frames == 1
        assert result.actors[0].alert_frames_in_event == 0

    def test_no_collision_label_exists(self) -> None:
        dataset, matching = sections(self._run([RiskLevel.LOW]))
        assert evaluate_risk(dataset, CONFIG, matching).collision_labels_present is False


class TestMapping:
    def test_accuracy_is_unavailable_with_the_reason_and_workload_is_measured(self) -> None:
        dataset, _ = sections(approach(3))
        result = evaluate_mapping(dataset)
        assert result.status is MetricStatus.MEASURED
        assert result.accuracy_status is MetricStatus.UNAVAILABLE
        assert "not evaluated in Phase 11" in result.accuracy_reason
        assert result.fixed is not None and result.adaptive is not None
        assert result.fixed.total_cells.mean == 6400  # 40 m / 0.5 m squared
        assert result.adaptive.total_cells.mean == 16 * 100  # sixteen LOW tiles of 1 m
        assert result.fixed_resolution_m == 0.5


class TestAdaptive:
    def test_the_actor_tile_level_is_told_apart_from_the_rest(self) -> None:
        builder = RunBuilder({"target": 5})
        tile = tile_index_for(10.0, 0.0)
        for _ in range(3):
            builder.frame(
                FrameSpec(
                    tracks=[TrackSpec(0, 10.0, 0.0)],
                    actors=[ActorSpec(5, 10.0, 0.0)],
                    levels=levels((tile, ResolutionLevel.HIGH)),
                )
            )
        dataset, matching = sections(builder)
        result = evaluate_adaptive(dataset, CONFIG, matching)
        assert result.actor_tile_levels == {"high": 3}
        assert result.other_tile_levels == {"low": 45}
        assert result.actor_tile_resolution_m.mean == pytest.approx(0.2)
        assert result.other_tile_resolution_m.mean == pytest.approx(1.0)
        assert result.cell_ratio.mean == pytest.approx((15 * 100 + 2500) / 6400)

    def test_refinement_lead_is_negative_when_the_tile_refines_after_arrival(self) -> None:
        builder = RunBuilder({"target": 5})
        tile = tile_index_for(10.0, 0.0)
        for i in range(4):
            builder.frame(
                FrameSpec(
                    tracks=[TrackSpec(0, 10.0, 0.0)],
                    actors=[ActorSpec(5, 10.0, 0.0)],
                    levels=levels((tile, ResolutionLevel.MEDIUM)) if i >= 2 else None,
                )
            )
        dataset, matching = sections(builder)
        result = evaluate_adaptive(dataset, CONFIG, matching)
        assert len(result.refinements) == 1
        assert result.refinements[0].entered_frame == 0
        assert result.refinements[0].refined_frame == 2
        assert result.refinements[0].lead_frames == -2
        assert result.entries_never_refined == 0

    def test_refinement_lead_is_positive_when_the_tile_was_refined_before_arrival(self) -> None:
        builder = RunBuilder({"target": 5})
        near, far = tile_index_for(10.0, 0.0), tile_index_for(-10.0, 0.0)
        for i in range(4):
            x = -10.0 if i < 2 else 10.0
            builder.frame(
                FrameSpec(
                    tracks=[TrackSpec(0, x, 0.0)],
                    actors=[ActorSpec(5, x, 0.0)],
                    levels=levels((near, ResolutionLevel.MEDIUM)),
                )
            )
        dataset, matching = sections(builder)
        result = evaluate_adaptive(dataset, CONFIG, matching)
        by_tile = {r.tile_index: r for r in result.refinements}
        assert by_tile[near].lead_frames == 2  # refined on frame 0, entered on frame 2
        assert by_tile[far].lead_frames is None
        assert result.entries_never_refined == 1

    def test_an_actor_flickering_across_a_tile_boundary_enters_each_tile_once(self) -> None:
        """Found live: an actor placed exactly on a tile boundary flips tile with
        the simulator's jitter; every flip must not count as a new arrival."""
        builder = RunBuilder({"target": 5})
        for i in range(6):
            y = 0.0 if i % 2 == 0 else -0.001  # either side of the y = 0 tile edge
            builder.frame(FrameSpec(tracks=[TrackSpec(0, 5.0, y)], actors=[ActorSpec(5, 5.0, y)]))
        dataset, matching = sections(builder)
        result = evaluate_adaptive(dataset, CONFIG, matching)
        assert len(result.refinements) == 2
        assert sorted(r.entered_frame for r in result.refinements) == [0, 1]

    def test_churn_and_reversals(self) -> None:
        builder = RunBuilder({"target": 5})
        sequence = [
            ResolutionLevel.LOW,
            ResolutionLevel.MEDIUM,  # transition
            ResolutionLevel.LOW,  # transition back within the window: a reversal
            ResolutionLevel.LOW,
            ResolutionLevel.LOW,
            ResolutionLevel.LOW,
            ResolutionLevel.MEDIUM,  # transition
            ResolutionLevel.MEDIUM,
            ResolutionLevel.MEDIUM,
            ResolutionLevel.MEDIUM,
            ResolutionLevel.MEDIUM,
            ResolutionLevel.LOW,  # transition back after five frames: no reversal
        ]
        for level in sequence:
            builder.frame(
                FrameSpec(
                    tracks=[TrackSpec(0, 10.0, 0.0)],
                    actors=[ActorSpec(5, 10.0, 0.0)],
                    levels=levels((3, level)),
                )
            )
        dataset, matching = sections(builder)
        churn = evaluate_adaptive(dataset, CONFIG, matching).churn
        assert churn is not None
        assert churn.total_transitions == 4
        assert churn.tiles_ever_changed == 1
        assert churn.frames_with_any_change == 4
        assert churn.min_dwell_frames == 3
        assert churn.reversals == 1

    def test_holds_and_budget_are_read_from_the_plan(self) -> None:
        builder = RunBuilder({"target": 5})
        builder.frame(
            FrameSpec(
                tracks=[TrackSpec(0, 10.0, 0.0)],
                actors=[ActorSpec(5, 10.0, 0.0)],
                holds={1: TileDecisionReason.HYSTERESIS_HOLD, 2: TileDecisionReason.DWELL_HOLD},
                within_budget=False,
                demoted=3,
            )
        )
        dataset, matching = sections(builder)
        result = evaluate_adaptive(dataset, CONFIG, matching)
        assert result.churn is not None
        assert result.churn.hysteresis_holds == 1 and result.churn.dwell_holds == 1
        assert result.frames_over_budget == 1 and result.demoted_tiles_total == 3

    def test_detail_is_grouped_by_the_matched_risk_level_with_unknown_apart(self) -> None:
        builder = RunBuilder({"target": 5})
        tile = tile_index_for(10.0, 0.0)
        for spec in (RiskSpec(0, RiskLevel.HIGH, 0.7), RiskSpec(0, RiskLevel.UNKNOWN, None)):
            builder.frame(
                FrameSpec(
                    tracks=[TrackSpec(0, 10.0, 0.0)],
                    actors=[ActorSpec(5, 10.0, 0.0)],
                    risk=[spec],
                    levels=levels((tile, ResolutionLevel.HIGH)),
                )
            )
        dataset, matching = sections(builder)
        groups = evaluate_adaptive(dataset, CONFIG, matching).actor_tile_resolution_by_risk_m
        assert set(groups) == {"high", "unknown"}
        assert groups["unknown"].count == 1

    def test_duration_ratios_are_paired_per_frame(self) -> None:
        builder = RunBuilder({"target": 5}).frame(
            FrameSpec(
                tracks=[TrackSpec(0, 10.0, 0.0)],
                actors=[ActorSpec(5, 10.0, 0.0)],
                fixed_duration_ms=2.0,
                adaptive_duration_ms=6.0,
                controller_duration_ms=4.0,
            )
        )
        dataset, matching = sections(builder)
        result = evaluate_adaptive(dataset, CONFIG, matching)
        assert result.mapping_duration_ratio.mean == pytest.approx(3.0)
        assert result.total_duration_ratio.mean == pytest.approx(5.0)


class TestResource:
    def test_stage_timings_are_summarised_and_memory_is_unavailable(self) -> None:
        dataset, _ = sections(approach(3))
        result = evaluate_resource(dataset)
        assert result.status is MetricStatus.MEASURED
        assert set(result.stage_ms) >= {"processing", "detection", "tracking", "mapping"}
        assert result.stage_ms["processing"].median == pytest.approx(5.0)
        assert result.peak_memory_status is MetricStatus.UNAVAILABLE
        assert "not sampled" in result.peak_memory_reason


class TestReport:
    def test_the_report_round_trips_through_json(self) -> None:
        report = evaluate(approach(6).build(), git_commit="abc123")
        rebuilt = EvaluationReport.model_validate_json(report.model_dump_json())
        assert rebuilt == report

    def test_the_report_is_simulation_evidence_with_its_limitations(self) -> None:
        report = evaluate(approach(3).build())
        assert report.source is DataSource.SIMULATION
        assert any("Not safety validation" in line for line in report.limitations)
        assert any("Not collision-probability" in line for line in report.limitations)
        assert any("Not real-world" in line for line in report.limitations)

    def test_evaluation_is_deterministic(self) -> None:
        record = approach(6, offset_m=0.3).build()
        first = evaluate(record, git_commit="x")
        second = evaluate(record, git_commit="x")
        comparison = compare_reports(first, second)
        assert comparison.repeatable
        assert comparison.differences == []
        assert first.evaluation_id != second.evaluation_id  # identity differs, content does not

    def test_a_content_difference_is_found_and_timing_is_ignored(self) -> None:
        first = evaluate(approach(6, offset_m=0.3).build(), git_commit="x")
        second = evaluate(approach(6, offset_m=0.4).build(), git_commit="x")
        comparison = compare_reports(first, second)
        assert not comparison.repeatable
        assert any("position_error_planar_m" in d for d in comparison.differences)
        assert any(path.startswith("resource") for path in comparison.timing_fields_ignored)

    def test_an_incomplete_run_is_flagged(self) -> None:
        report = evaluate(approach(3).build(state=ScenarioState.FAILED))
        assert any("did not complete" in warning for warning in report.warnings)

    def test_the_configuration_and_pipeline_configuration_are_recorded(self) -> None:
        report = evaluate(approach(3).build())
        assert report.configuration == EvaluationConfiguration()
        assert set(report.pipeline_configuration) == {
            "tracking",
            "prediction",
            "risk",
            "mapping",
            "adaptive_resolution",
        }
        assert report.sensor is not None and report.sensor_mount == Vector3(x=0.0, y=0.0, z=1.8)

    def test_the_text_report_prints_reasons_not_zeros_for_missing_metrics(self) -> None:
        builder = RunBuilder({"target": 5}).frame(
            FrameSpec(
                tracks=[TrackSpec(0, 10.0, 0.0, velocity=None)], actors=[ActorSpec(5, 10.0, 0.0)]
            )
        )
        text = render(evaluate(builder.build()))
        assert "ADAPT-X PHASE 11 EVALUATION" in text
        assert "velocity error:        n/a (no samples)" in text
        assert "ADE: n/a (no samples)" in text
        assert "map accuracy: unavailable" in text
        assert "peak memory: unavailable" in text
        assert "LIMITATIONS" in text
        assert "not false positives" in text

    def test_the_report_models_carry_no_precision_or_accuracy_field(self) -> None:
        """Precision over unlabelled tracks and 'accuracy' would be fabricated."""
        names = " ".join(
            " ".join(model.model_fields)
            for model in (
                EvaluationReport,
                *(type(s) for s in evaluate(approach(2).build()).sections.values()),
            )
        )
        for forbidden in ("precision", "accuracy_percent", "false_positive", "mota", "motp"):
            assert forbidden not in names


class TestDatasetFrames:
    def test_ineligible_actor_frames_produce_a_tracking_warning(self) -> None:
        builder = RunBuilder({"target": 5})
        builder.frame(FrameSpec(tracks=[TrackSpec(0, 10.0, 0.0)], actors=[ActorSpec(5, 10.0, 0.0)]))
        builder.frame(FrameSpec(tracks=[], actors=[ActorSpec(5, 30.0, 0.0)]))
        dataset, matching = sections(builder)
        result = evaluate_tracking(dataset, CONFIG, matching)
        assert any("ineligible" in warning for warning in result.warnings)
        assert result.gates[0].eligible_pairs == 1

    def test_a_class_disagreement_lowers_agreement_not_the_match(self) -> None:
        builder = RunBuilder({"target": 5}).frame(
            FrameSpec(
                tracks=[TrackSpec(0, 10.0, 0.0, object_class=ObjectClass.UNKNOWN)],
                actors=[ActorSpec(5, 10.0, 0.0)],
            )
        )
        dataset, matching = sections(builder)
        gate = evaluate_tracking(dataset, CONFIG, matching).gates[0]
        assert gate.match_rate == 1.0 and gate.class_agreement_rate == 0.0
