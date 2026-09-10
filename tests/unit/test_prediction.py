"""Constant-velocity predictor tests (Phase 5).

Every case has an explicit position, an explicit velocity and an explicit
horizon, so the position a test expects is arithmetic rather than a guess.
Nothing here is random, and nothing is asserted loosely where the model gives
one correct answer: a track at x=10 moving at 2 m/s is at x=11 at t+0.5 and
nowhere else.

Tracks are constructed directly rather than produced by the tracker. These
tests exercise prediction; routing them through association would make a
prediction failure indistinguishable from a tracking one.
"""

from __future__ import annotations

import math
from datetime import timedelta
from itertools import pairwise

import pytest
from pydantic import ValidationError

from adaptx.config.settings import PredictionSettings
from adaptx.models.common import CoordinateFrame, DataSource, ObjectClass, Vector3
from adaptx.models.prediction import PredictionStatus
from adaptx.models.tracking import TrackStatus
from adaptx.prediction.constant_velocity import (
    MODEL_NAME,
    ConstantVelocityPredictor,
    build_predictor,
)
from tests.fixtures.sequences import EPOCH, track


def predictor(**overrides: object) -> ConstantVelocityPredictor:
    """A predictor whose configuration starts at the defaults."""
    return ConstantVelocityPredictor(PredictionSettings(**overrides))  # type: ignore[arg-type]


def offsets_of(trajectory: object) -> list[float]:
    return [point.time_offset_s for point in trajectory.points]  # type: ignore[attr-defined]


class TestConstantVelocityArithmetic:
    """The model is ``p + v*t``. These are exact, not approximate."""

    def test_the_documented_worked_example(self) -> None:
        """position 10, velocity 2 m/s: t+0.5=11, t+1=12, t+2=14, t+3=16."""
        result = predictor().predict([track((10.0, 0.0, 0.0), (2.0, 0.0, 0.0))], EPOCH)
        points = {p.time_offset_s: p.position.x for p in result.trajectories[0].points}

        assert points[0.0] == 10.0
        assert points[0.5] == 11.0
        assert points[1.0] == 12.0
        assert points[2.0] == 14.0
        assert points[3.0] == 16.0

    def test_motion_along_positive_x(self) -> None:
        result = predictor().predict([track((0.0, 0.0, 0.0), (4.0, 0.0, 0.0))], EPOCH)
        for point in result.trajectories[0].points:
            assert point.position.x == pytest.approx(4.0 * point.time_offset_s)
            assert point.position.y == 0.0
            assert point.position.z == 0.0

    def test_motion_along_negative_x(self) -> None:
        result = predictor().predict([track((0.0, 0.0, 0.0), (-3.0, 0.0, 0.0))], EPOCH)
        last = result.trajectories[0].points[-1]

        assert last.time_offset_s == 3.0
        assert last.position.x == pytest.approx(-9.0)

    def test_motion_along_positive_y(self) -> None:
        result = predictor().predict([track((5.0, 1.0, 0.0), (0.0, 2.0, 0.0))], EPOCH)
        last = result.trajectories[0].points[-1]

        assert last.position.x == 5.0
        assert last.position.y == pytest.approx(1.0 + 6.0)
        assert last.position.z == 0.0

    def test_diagonal_motion_advances_every_axis_independently(self) -> None:
        result = predictor().predict([track((0.0, 0.0, 0.0), (1.0, 2.0, 3.0))], EPOCH)
        point = next(p for p in result.trajectories[0].points if p.time_offset_s == 2.0)

        assert point.position.x == pytest.approx(2.0)
        assert point.position.y == pytest.approx(4.0)
        assert point.position.z == pytest.approx(6.0)

    def test_vertical_motion_is_predicted_like_any_other_axis(self) -> None:
        """+z is up (ADR-009); the baseline has no ground constraint."""
        result = predictor().predict([track((0.0, 0.0, 1.0), (0.0, 0.0, -0.5))], EPOCH)
        last = result.trajectories[0].points[-1]

        assert last.position.z == pytest.approx(1.0 - 1.5)

    def test_every_point_carries_the_constant_velocity_it_was_built_from(self) -> None:
        result = predictor().predict([track((0.0, 0.0, 0.0), (1.5, -2.5, 0.0))], EPOCH)
        for point in result.trajectories[0].points:
            assert point.velocity is not None
            assert point.velocity.x == 1.5
            assert point.velocity.y == -2.5

    def test_no_output_value_is_nan_or_infinite(self) -> None:
        result = predictor().predict(
            [track((10.0, -3.0, 0.5), (12.0, -7.5, 0.25))],
            EPOCH,
        )
        for point in result.trajectories[0].points:
            for value in (
                point.position.x,
                point.position.y,
                point.position.z,
                point.confidence,
                point.position_uncertainty_m,
                point.time_offset_s,
            ):
                assert math.isfinite(value)


class TestVelocitySemantics:
    """``None`` is not zero (ADR-023). The distinction is the point of Phase 5."""

    def test_unknown_velocity_produces_no_trajectory(self) -> None:
        result = predictor().predict([track(velocity=None)], EPOCH)

        assert result.trajectories == []
        assert result.skipped_track_count == 1
        assert result.skipped[0].status is PredictionStatus.INSUFFICIENT_VELOCITY

    def test_unknown_velocity_records_a_reason_rather_than_going_silent(self) -> None:
        result = predictor().predict([track(velocity=None)], EPOCH)
        reason = result.skipped[0].reason

        assert "no measured velocity" in reason
        assert "standstill" in reason

    def test_a_measured_standstill_produces_a_stationary_trajectory(self) -> None:
        """Zero is a measurement. It is not the same as unknown."""
        result = predictor().predict([track((7.0, 2.0, 0.0), (0.0, 0.0, 0.0))], EPOCH)
        trajectory = result.trajectories[0]

        assert result.skipped == []
        assert len(trajectory.points) == 13
        for point in trajectory.points:
            assert point.position.x == 7.0
            assert point.position.y == 2.0
            assert point.position.z == 0.0

    def test_a_stationary_trajectory_still_grows_uncertain(self) -> None:
        """A standstill can end. Certainty about it must not be claimed."""
        result = predictor().predict([track(velocity=(0.0, 0.0, 0.0))], EPOCH)
        points = result.trajectories[0].points

        assert points[-1].position_uncertainty_m > points[0].position_uncertainty_m

    def test_excessive_speed_is_rejected_not_clipped(self) -> None:
        result = predictor(max_speed_mps=30.0).predict(
            [track((0.0, 0.0, 0.0), (100.0, 0.0, 0.0))], EPOCH
        )

        assert result.trajectories == []
        assert result.skipped[0].status is PredictionStatus.INVALID_VELOCITY
        assert "rejected rather than clipped" in result.skipped[0].reason

    def test_speed_exactly_at_the_bound_is_accepted(self) -> None:
        result = predictor(max_speed_mps=30.0).predict(
            [track((0.0, 0.0, 0.0), (30.0, 0.0, 0.0))], EPOCH
        )

        assert result.predicted_track_count == 1

    def test_the_smoothed_velocity_is_used_not_the_raw_one(self) -> None:
        """Single-frame noise is what should not be projected forward."""
        subject = track((0.0, 0.0, 0.0), (2.0, 0.0, 0.0)).model_copy(
            update={"observed_velocity": Vector3(x=50.0, y=0.0, z=0.0)}
        )
        result = predictor().predict([subject], EPOCH)
        last = result.trajectories[0].points[-1]

        assert last.position.x == pytest.approx(6.0)

    def test_heading_does_not_steer_the_extrapolation(self) -> None:
        """Velocity is the authoritative motion vector, not heading."""
        subject = track((0.0, 0.0, 0.0), (3.0, 0.0, 0.0)).model_copy(
            update={"heading_rad": math.pi / 2}
        )
        result = predictor().predict([subject], EPOCH)
        last = result.trajectories[0].points[-1]

        assert last.position.x == pytest.approx(9.0)
        assert last.position.y == 0.0


class TestTrackLifecycle:
    def test_a_confirmed_track_is_predicted_normally(self) -> None:
        result = predictor().predict([track(status=TrackStatus.CONFIRMED)], EPOCH)

        assert result.trajectories[0].status is PredictionStatus.PREDICTED
        assert result.trajectories[0].observation_age_s == 0.0

    def test_a_tentative_track_with_velocity_is_predicted(self) -> None:
        result = predictor().predict([track(status=TrackStatus.TENTATIVE, hits=1)], EPOCH)

        assert result.predicted_track_count == 1

    def test_a_tentative_track_scores_below_an_established_one(self) -> None:
        """Fewer observations is less evidence, and must show as less confidence."""
        settings = PredictionSettings(confidence_hits_full=3)
        tentative = ConstantVelocityPredictor(settings).predict(
            [track(status=TrackStatus.TENTATIVE, hits=1)], EPOCH
        )
        confirmed = ConstantVelocityPredictor(settings).predict(
            [track(status=TrackStatus.CONFIRMED, hits=9)], EPOCH
        )

        assert tentative.trajectories[0].confidence < confirmed.trajectories[0].confidence
        assert tentative.trajectories[0].confidence == pytest.approx(1 / 3)
        assert confirmed.trajectories[0].confidence == 1.0

    def test_a_tentative_track_without_velocity_is_skipped(self) -> None:
        result = predictor().predict(
            [track(status=TrackStatus.TENTATIVE, hits=1, velocity=None)], EPOCH
        )

        assert result.skipped[0].status is PredictionStatus.INSUFFICIENT_VELOCITY

    def test_a_coasting_track_is_marked_as_extrapolated(self) -> None:
        result = predictor().predict(
            [track(status=TrackStatus.COASTING, missed_frames=2, last_seen_offset_s=0.2)],
            EPOCH,
        )
        trajectory = result.trajectories[0]

        assert trajectory.status is PredictionStatus.EXTRAPOLATED
        assert trajectory.is_extrapolated_from_stale_observation is True
        assert trajectory.observation_age_s == pytest.approx(0.2)

    def test_a_coasting_track_starts_from_where_it_would_be_now(self) -> None:
        """Its stored position is stale by a measured interval; that is not ignored."""
        result = predictor().predict(
            [
                track(
                    (10.0, 0.0, 0.0),
                    (2.0, 0.0, 0.0),
                    status=TrackStatus.COASTING,
                    missed_frames=1,
                    last_seen_offset_s=0.5,
                )
            ],
            EPOCH,
        )
        points = {p.time_offset_s: p.position.x for p in result.trajectories[0].points}

        assert points[0.0] == pytest.approx(11.0)
        assert points[1.0] == pytest.approx(13.0)

    def test_a_coasting_track_is_more_uncertain_than_a_fresh_one(self) -> None:
        fresh = predictor().predict([track()], EPOCH)
        coasting = predictor().predict(
            [track(status=TrackStatus.COASTING, missed_frames=2, last_seen_offset_s=0.4)],
            EPOCH,
        )

        assert (
            coasting.trajectories[0].points[0].position_uncertainty_m
            > fresh.trajectories[0].points[0].position_uncertainty_m
        )
        assert coasting.trajectories[0].confidence < fresh.trajectories[0].confidence

    def test_a_lost_track_publishes_no_active_prediction(self) -> None:
        result = predictor().predict([track(status=TrackStatus.LOST)], EPOCH)

        assert result.trajectories == []
        assert result.skipped[0].status is PredictionStatus.TRACK_LOST

    def test_an_observation_older_than_the_horizon_is_skipped(self) -> None:
        result = predictor(horizon_s=3.0).predict(
            [track(status=TrackStatus.COASTING, last_seen_offset_s=5.0)], EPOCH
        )

        assert result.trajectories == []
        assert result.skipped[0].status is PredictionStatus.STALE_OBSERVATION

    def test_a_last_seen_time_in_the_future_is_treated_as_no_age(self) -> None:
        """A negative age would mean predicting back from an unobserved future."""
        result = predictor().predict([track(last_seen_offset_s=-1.0)], EPOCH)

        assert result.trajectories[0].observation_age_s == 0.0

    def test_a_track_never_seen_is_treated_as_no_age(self) -> None:
        subject = track().model_copy(update={"last_seen": None})
        result = predictor().predict([subject], EPOCH)

        assert result.trajectories[0].observation_age_s == 0.0


class TestHorizonAndInterval:
    def test_the_default_span_produces_thirteen_points(self) -> None:
        """3.0 s at 0.25 s, t+0 inclusive."""
        result = predictor().predict([track()], EPOCH)

        assert len(result.trajectories[0].points) == 13
        assert result.configuration.points_per_trajectory == 13

    def test_offsets_run_from_zero_to_the_horizon_in_exact_steps(self) -> None:
        result = predictor(horizon_s=1.0, interval_s=0.25).predict([track()], EPOCH)

        assert offsets_of(result.trajectories[0]) == [0.0, 0.25, 0.5, 0.75, 1.0]

    def test_a_configured_horizon_is_honoured(self) -> None:
        result = predictor(horizon_s=5.0, interval_s=1.0).predict([track()], EPOCH)
        trajectory = result.trajectories[0]

        assert trajectory.horizon_s == 5.0
        assert offsets_of(trajectory) == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]

    def test_a_configured_interval_is_honoured(self) -> None:
        result = predictor(horizon_s=1.0, interval_s=0.5).predict([track()], EPOCH)

        assert result.trajectories[0].timestep_s == 0.5
        assert offsets_of(result.trajectories[0]) == [0.0, 0.5, 1.0]

    def test_a_per_call_override_beats_the_configuration(self) -> None:
        result = predictor(horizon_s=3.0).predict([track()], EPOCH, horizon_s=1.0, timestep_s=0.5)

        assert result.trajectories[0].horizon_s == 1.0
        assert offsets_of(result.trajectories[0]) == [0.0, 0.5, 1.0]

    def test_an_interval_equal_to_the_horizon_gives_two_points(self) -> None:
        result = predictor(horizon_s=2.0, interval_s=2.0).predict([track()], EPOCH)

        assert offsets_of(result.trajectories[0]) == [0.0, 2.0]

    def test_no_point_ever_exceeds_the_horizon(self) -> None:
        """Holds even when the horizon is not an exact multiple of the interval."""
        result = predictor(horizon_s=1.0, interval_s=0.3).predict([track()], EPOCH)
        offsets = offsets_of(result.trajectories[0])

        assert max(offsets) <= 1.0
        assert offsets == sorted(offsets)

    def test_points_are_ordered_by_increasing_time(self) -> None:
        result = predictor().predict([track()], EPOCH)
        offsets = offsets_of(result.trajectories[0])

        assert offsets == sorted(offsets)
        assert offsets[0] == 0.0

    def test_absolute_point_times_are_derived_from_the_source_time(self) -> None:
        """Future times are arithmetic, never taken from a wall clock."""
        result = predictor().predict([track()], EPOCH)

        assert result.timestamp == EPOCH
        for point in result.trajectories[0].points:
            assert point.timestamp == EPOCH + timedelta(seconds=point.time_offset_s)

    def test_a_non_positive_override_is_rejected(self) -> None:
        with pytest.raises(ValueError, match="horizon_s"):
            predictor().predict([track()], EPOCH, horizon_s=0.0)
        with pytest.raises(ValueError, match="timestep_s"):
            predictor().predict([track()], EPOCH, timestep_s=-1.0)

    def test_an_interval_longer_than_the_horizon_is_rejected(self) -> None:
        with pytest.raises(ValueError, match="must be <= horizon_s"):
            predictor().predict([track()], EPOCH, horizon_s=1.0, timestep_s=2.0)

    def test_a_non_finite_override_is_rejected(self) -> None:
        with pytest.raises(ValueError, match="finite"):
            predictor().predict([track()], EPOCH, horizon_s=math.inf)


class TestUncertainty:
    def test_uncertainty_starts_at_the_configured_floor(self) -> None:
        result = predictor(base_uncertainty_m=0.5).predict([track()], EPOCH)

        assert result.trajectories[0].points[0].position_uncertainty_m == 0.5

    def test_uncertainty_grows_strictly_with_time_offset(self) -> None:
        result = predictor().predict([track()], EPOCH)
        values = [p.position_uncertainty_m for p in result.trajectories[0].points]

        assert values == sorted(values)
        assert all(later > earlier for earlier, later in pairwise(values))

    def test_uncertainty_follows_the_documented_formula(self) -> None:
        """base + growth * t, exactly."""
        result = predictor(base_uncertainty_m=0.5, uncertainty_growth_mps=0.5).predict(
            [track()], EPOCH
        )
        points = {p.time_offset_s: p.position_uncertainty_m for p in result.trajectories[0].points}

        assert points[0.0] == pytest.approx(0.5)
        assert points[1.0] == pytest.approx(1.0)
        assert points[2.0] == pytest.approx(1.5)
        assert points[3.0] == pytest.approx(2.0)

    def test_growth_rate_is_configurable(self) -> None:
        slow = predictor(uncertainty_growth_mps=0.1).predict([track()], EPOCH)
        fast = predictor(uncertainty_growth_mps=2.0).predict([track()], EPOCH)

        assert (
            fast.trajectories[0].points[-1].position_uncertainty_m
            > slow.trajectories[0].points[-1].position_uncertainty_m
        )

    def test_point_confidence_decays_as_uncertainty_grows(self) -> None:
        result = predictor().predict([track()], EPOCH)
        values = [p.confidence for p in result.trajectories[0].points]

        assert all(later < earlier for earlier, later in pairwise(values))
        assert all(0.0 <= value <= 1.0 for value in values)

    def test_the_first_point_carries_the_full_track_confidence(self) -> None:
        result = predictor().predict([track(hits=9)], EPOCH)
        trajectory = result.trajectories[0]

        assert trajectory.points[0].confidence == pytest.approx(trajectory.confidence)

    def test_the_result_labels_its_uncertainty_model_as_heuristic(self) -> None:
        result = predictor().predict([track()], EPOCH)

        assert result.uncertainty_model == "heuristic_linear_growth"
        assert result.is_baseline is True


class TestMultipleTracksAndLimits:
    def test_every_track_gets_its_own_trajectory(self) -> None:
        tracks = [
            track((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), track_id=0),
            track((20.0, 5.0, 0.0), (0.0, -2.0, 0.0), track_id=1),
            track((-10.0, 0.0, 0.0), (0.0, 0.0, 0.0), track_id=2),
        ]
        result = predictor().predict(tracks, EPOCH)

        assert result.predicted_track_ids == [0, 1, 2]
        assert result.considered_track_count == 3

    def test_tracks_beyond_the_limit_are_recorded_not_dropped(self) -> None:
        tracks = [track(track_id=index) for index in range(5)]
        result = predictor(max_tracks=2).predict(tracks, EPOCH)

        assert result.predicted_track_count == 2
        assert result.skipped_track_count == 3
        assert all(s.status is PredictionStatus.LIMIT_EXCEEDED for s in result.skipped)

    def test_a_mixed_scene_accounts_for_every_track(self) -> None:
        tracks = [
            track(track_id=0),
            track(track_id=1, velocity=None),
            track(track_id=2, status=TrackStatus.LOST),
            track(track_id=3, velocity=(0.0, 0.0, 0.0)),
        ]
        result = predictor().predict(tracks, EPOCH)

        assert result.considered_track_count == 4
        assert result.predicted_track_count == 2
        assert result.skipped_track_count == 2
        assert result.counts_by_status() == {
            "predicted": 2,
            "insufficient_velocity": 1,
            "track_lost": 1,
        }

    def test_an_empty_track_list_produces_an_empty_result(self) -> None:
        result = predictor().predict([], EPOCH)

        assert result.trajectories == []
        assert result.skipped == []
        assert result.considered_track_count == 0
        assert result.predicted_point_count == 0

    def test_a_scene_where_nothing_is_eligible_is_not_an_empty_scene(self) -> None:
        """ "Saw nothing" and "could predict nothing" must stay distinguishable."""
        result = predictor().predict(
            [track(track_id=0, velocity=None), track(track_id=1, velocity=None)], EPOCH
        )

        assert result.trajectories == []
        assert result.considered_track_count == 2
        assert result.skipped_track_count == 2

    def test_a_trajectory_can_be_looked_up_by_track_id(self) -> None:
        result = predictor().predict([track(track_id=4), track(track_id=9)], EPOCH)

        assert result.trajectory_for(9) is not None
        assert result.trajectory_for(9).track_id == 9  # type: ignore[union-attr]
        assert result.trajectory_for(77) is None

    def test_the_total_point_count_is_reported(self) -> None:
        result = predictor(horizon_s=1.0, interval_s=0.5).predict(
            [track(track_id=0), track(track_id=1)], EPOCH
        )

        assert result.predicted_point_count == 6


class TestResultAccounting:
    def test_considered_equals_predicted_plus_skipped(self) -> None:
        tracks = [track(track_id=0), track(track_id=1, velocity=None)]
        result = predictor().predict(tracks, EPOCH)

        assert result.considered_track_count == (
            result.predicted_track_count + result.skipped_track_count
        )

    def test_a_result_that_does_not_add_up_is_rejected(self) -> None:
        from adaptx.models.prediction_result import PredictionConfiguration, PredictionResult

        with pytest.raises(ValidationError, match="must equal"):
            PredictionResult(
                frame_id=0,
                sensor_id="test",
                predictor="x",
                model_name="constant_velocity",
                considered_track_count=5,
                duration_ms=0.0,
                configuration=PredictionConfiguration(
                    horizon_s=3.0,
                    interval_s=0.25,
                    max_tracks=256,
                    max_speed_mps=80.0,
                    base_uncertainty_m=0.5,
                    uncertainty_growth_mps=0.5,
                    confidence_hits_full=3,
                ),
            )

    def test_the_result_carries_provenance_and_identity(self) -> None:
        result = predictor().predict([track()], EPOCH, frame_id=42, sensor_id="lidar_top")

        assert result.frame_id == 42
        assert result.sensor_id == "lidar_top"
        assert result.predictor == "constant_velocity_v1"
        assert result.model_name == MODEL_NAME

    def test_the_result_carries_a_configuration_snapshot(self) -> None:
        result = predictor(horizon_s=2.0, interval_s=0.5, max_tracks=8).predict([track()], EPOCH)

        assert result.configuration.horizon_s == 2.0
        assert result.configuration.interval_s == 0.5
        assert result.configuration.max_tracks == 8

    def test_the_duration_is_measured_not_estimated(self) -> None:
        result = predictor().predict([track(track_id=i) for i in range(20)], EPOCH)

        assert result.duration_ms > 0.0
        assert math.isfinite(result.duration_ms)

    def test_coordinate_frame_and_source_come_from_the_track(self) -> None:
        """Provenance propagates: a synthetic track cannot yield a live-sensor path."""
        subject = track(coordinate_frame=CoordinateFrame.WORLD, source=DataSource.SIMULATION)
        result = predictor().predict([subject], EPOCH)

        assert result.trajectories[0].coordinate_frame is CoordinateFrame.WORLD
        assert result.trajectories[0].source is DataSource.SIMULATION

    def test_the_predictor_never_writes_a_prediction_onto_the_track(self) -> None:
        subject = track((10.0, 0.0, 0.0), (2.0, 0.0, 0.0))
        predictor().predict([subject], EPOCH)

        assert subject.position.x == 10.0
        assert subject.predicted_position is None


class TestDeterminism:
    def test_the_same_input_always_produces_the_same_output(self) -> None:
        tracks = [track(track_id=0), track(track_id=1, velocity=(1.0, 1.0, 0.0))]
        first = predictor().predict(tracks, EPOCH)
        second = predictor().predict(tracks, EPOCH)

        assert first.model_dump(exclude={"duration_ms"}) == second.model_dump(
            exclude={"duration_ms"}
        )

    def test_repeated_calls_on_one_predictor_do_not_drift(self) -> None:
        """The predictor is stateless; a tenth call must match the first."""
        stage = predictor()
        first = stage.predict([track()], EPOCH)
        for _ in range(9):
            latest = stage.predict([track()], EPOCH)

        assert [p.position.x for p in latest.trajectories[0].points] == [
            p.position.x for p in first.trajectories[0].points
        ]

    def test_track_order_does_not_change_any_trajectory(self) -> None:
        a = track(track_id=0, velocity=(1.0, 0.0, 0.0))
        b = track(track_id=1, velocity=(0.0, 3.0, 0.0))
        forward = predictor().predict([a, b], EPOCH)
        reverse = predictor().predict([b, a], EPOCH)

        assert forward.trajectory_for(1).points[-1].position.y == pytest.approx(  # type: ignore[union-attr]
            reverse.trajectory_for(1).points[-1].position.y  # type: ignore[union-attr]
        )


class TestConfiguration:
    def test_the_defaults_are_the_documented_engineering_values(self) -> None:
        settings = PredictionSettings()

        assert settings.horizon_s == 3.0
        assert settings.interval_s == 0.25
        assert settings.max_tracks == 256
        assert settings.max_speed_mps == 80.0
        assert settings.base_uncertainty_m == 0.5
        assert settings.uncertainty_growth_mps == 0.5

    def test_an_interval_longer_than_the_horizon_is_rejected(self) -> None:
        with pytest.raises(ValidationError, match="interval_s must be <= "):
            PredictionSettings(horizon_s=1.0, interval_s=2.0)

    @pytest.mark.parametrize(
        ("field", "value"),
        [
            ("horizon_s", 0.0),
            ("horizon_s", -1.0),
            ("interval_s", 0.0),
            ("max_tracks", 0),
            ("max_speed_mps", 0.0),
            ("base_uncertainty_m", 0.0),
            ("uncertainty_growth_mps", -0.1),
            ("confidence_hits_full", 0),
        ],
    )
    def test_out_of_range_configuration_is_rejected(self, field: str, value: float) -> None:
        with pytest.raises(ValidationError):
            PredictionSettings(**{field: value})  # type: ignore[arg-type]

    def test_the_factory_builds_the_configured_predictor(self) -> None:
        built = build_predictor(PredictionSettings(horizon_s=4.0))

        assert isinstance(built, ConstantVelocityPredictor)
        assert built.configuration.horizon_s == 4.0

    def test_the_predictor_is_labelled_a_baseline(self) -> None:
        assert ConstantVelocityPredictor(PredictionSettings()).is_baseline is True
        assert MODEL_NAME == "constant_velocity"


class TestMalformedInput:
    def test_a_track_class_does_not_change_the_prediction(self) -> None:
        """The baseline is class-agnostic; claiming otherwise would be untrue."""
        vehicle = predictor().predict(
            [track(object_class=ObjectClass.VEHICLE, velocity=(2.0, 0.0, 0.0))], EPOCH
        )
        pedestrian = predictor().predict(
            [track(object_class=ObjectClass.PEDESTRIAN, velocity=(2.0, 0.0, 0.0))], EPOCH
        )

        assert [p.position.x for p in vehicle.trajectories[0].points] == [
            p.position.x for p in pedestrian.trajectories[0].points
        ]

    def test_a_non_finite_velocity_cannot_reach_the_predictor(self) -> None:
        """``Vector3`` rejects it at the contract boundary."""
        with pytest.raises(ValidationError, match="finite"):
            Vector3(x=math.nan, y=0.0, z=0.0)

    def test_a_very_small_interval_still_respects_the_horizon(self) -> None:
        result = predictor(horizon_s=0.1, interval_s=0.001).predict([track()], EPOCH)
        offsets = offsets_of(result.trajectories[0])

        assert len(offsets) == 101
        assert max(offsets) <= 0.1
