"""Heuristic risk and uncertainty engine tests (Phase 7).

Every case has an explicit position, an explicit velocity and an explicit
configuration, so the score a test expects is arithmetic rather than a guess.
Nothing here is random.

Tracks are constructed directly rather than produced by the tracker: these
tests exercise risk, and routing them through association would make a risk
failure indistinguishable from a tracking one.
"""

from __future__ import annotations

import math
from datetime import UTC, datetime, timedelta
from itertools import pairwise

import numpy as np
import pytest
from pydantic import ValidationError

from adaptx.config.settings import MapSettings, RiskSettings
from adaptx.mapping.grid_mapper import FixedResolutionMapper
from adaptx.models.common import CoordinateFrame, DataSource, ObjectClass, Vector3
from adaptx.models.point_cloud import PointCloudFrame
from adaptx.models.prediction import PredictedTrajectory, TrajectoryPoint
from adaptx.models.risk import RiskFactors, RiskLevel
from adaptx.models.risk_assessment import (
    AssessmentStatus,
    MapObservation,
    RiskAssessment,
    RiskAssessmentResult,
    RiskFactorName,
    UncertaintyReason,
)
from adaptx.models.tracking import TrackedObject, TrackStatus
from adaptx.models.vehicle import VehicleState
from adaptx.risk.heuristic import HeuristicRiskEngine, build_risk_engine

EPOCH = datetime(2026, 1, 1, 12, 0, 0, tzinfo=UTC)


def engine(**overrides: object) -> HeuristicRiskEngine:
    """An engine whose configuration starts at the defaults."""
    return HeuristicRiskEngine(RiskSettings(**overrides))  # type: ignore[arg-type]


def track(
    x: float = 10.0,
    y: float = 0.0,
    velocity: tuple[float, float, float] | None = (0.0, 0.0, 0.0),
    *,
    track_id: int = 0,
    status: TrackStatus = TrackStatus.CONFIRMED,
    object_class: ObjectClass = ObjectClass.VEHICLE,
    confidence: float = 0.8,
    last_seen_offset_s: float = 0.0,
    missed_frames: int = 0,
) -> TrackedObject:
    """A tracked object with an explicit state."""
    return TrackedObject(
        timestamp=EPOCH,
        track_id=track_id,
        object_class=object_class,
        status=status,
        position=Vector3(x=x, y=y, z=0.0),
        velocity=(
            None if velocity is None else Vector3(x=velocity[0], y=velocity[1], z=velocity[2])
        ),
        point_count=500,
        hits=5,
        first_seen=EPOCH - timedelta(seconds=1.0),
        confidence=confidence,
        age_frames=5,
        missed_frames=missed_frames,
        last_seen=EPOCH - timedelta(seconds=last_seen_offset_s),
        coordinate_frame=CoordinateFrame.EGO,
        source=DataSource.SYNTHETIC_TEST,
    )


def trajectory(
    *positions: tuple[float, float],
    track_id: int = 0,
    uncertainty_m: float = 0.5,
    horizon_s: float = 3.0,
) -> PredictedTrajectory:
    """A predicted path through the given planar positions, 1 s apart."""
    points = [
        TrajectoryPoint(
            timestamp=EPOCH + timedelta(seconds=index),
            time_offset_s=float(index),
            position=Vector3(x=x, y=y, z=0.0),
            confidence=0.9,
            position_uncertainty_m=uncertainty_m,
        )
        for index, (x, y) in enumerate(positions)
    ]
    return PredictedTrajectory(
        timestamp=EPOCH,
        track_id=track_id,
        horizon_s=horizon_s,
        timestep_s=1.0,
        points=points,
        confidence=0.9,
        predictor_name="test",
        source=DataSource.SYNTHETIC_TEST,
    )


def spatial_map_with(points: list[list[float]]) -> object:
    """A small Phase 6 map built from real geometry."""
    mapper = FixedResolutionMapper(
        MapSettings(min_x_m=-20.0, max_x_m=20.0, min_y_m=-20.0, max_y_m=20.0, resolution_m=1.0)
    )
    frame = PointCloudFrame(
        timestamp=EPOCH,
        frame_id=0,
        sensor_id="lidar",
        points=np.asarray(points, dtype=np.float64).reshape(-1, 3),
        source=DataSource.SYNTHETIC_TEST,
    )
    return mapper.build(frame)


class TestProximityFactor:
    def test_proximity_is_one_at_and_inside_the_near_bound(self) -> None:
        for distance in (0.5, 3.0, 5.0):
            result = engine().assess(track(x=distance))
            assert result.factor_scores.proximity == 1.0

    def test_proximity_is_zero_at_and_beyond_the_far_bound(self) -> None:
        for distance in (40.0, 60.0, 200.0):
            result = engine().assess(track(x=distance))
            assert result.factor_scores.proximity == 0.0

    def test_proximity_falls_off_linearly_between_the_bounds(self) -> None:
        """near=5, far=40: the midpoint 22.5 m must score exactly 0.5."""
        result = engine().assess(track(x=22.5))
        assert result.factor_scores.proximity == pytest.approx(0.5)

    def test_proximity_is_continuous_with_distance(self) -> None:
        """A small step in distance must never produce a jump in risk."""
        scores = [
            engine().assess(track(x=d)).factor_scores.proximity for d in np.arange(1.0, 45.0, 0.5)
        ]
        for earlier, later in pairwise(scores):
            assert later <= earlier
            assert abs(later - earlier) < 0.05

    def test_distance_is_planar_and_ignores_height(self) -> None:
        flat = engine().assess(track(x=10.0, y=0.0))
        raised = engine().assess(
            track(x=10.0, y=0.0).model_copy(update={"position": Vector3(x=10.0, y=0.0, z=8.0)})
        )
        assert flat.distance_m == pytest.approx(raised.distance_m)

    def test_a_nearer_object_is_never_less_risky_than_a_far_one(self) -> None:
        near = engine().assess(track(x=6.0, velocity=(0.0, 0.0, 0.0)))
        far = engine().assess(track(x=35.0, velocity=(0.0, 0.0, 0.0)))
        assert near.risk_score is not None and far.risk_score is not None
        assert near.risk_score > far.risk_score


class TestClosingSpeed:
    def test_an_approaching_object_has_positive_closing_speed(self) -> None:
        result = engine().assess(track(x=10.0, velocity=(-8.0, 0.0, 0.0)))
        assert result.closing_speed_mps == pytest.approx(8.0)

    def test_a_receding_object_has_negative_closing_speed(self) -> None:
        result = engine().assess(track(x=10.0, velocity=(8.0, 0.0, 0.0)))
        assert result.closing_speed_mps == pytest.approx(-8.0)

    def test_a_receding_object_contributes_nothing_to_risk(self) -> None:
        result = engine().assess(track(x=10.0, velocity=(8.0, 0.0, 0.0)))
        assert result.factor_scores.relative_velocity == 0.0

    def test_lateral_motion_is_not_closing(self) -> None:
        """Moving across the line of sight neither approaches nor recedes."""
        result = engine().assess(track(x=10.0, y=0.0, velocity=(0.0, 5.0, 0.0)))
        assert result.closing_speed_mps == pytest.approx(0.0)

    def test_closing_speed_saturates_at_the_configured_bound(self) -> None:
        result = engine(closing_speed_high_mps=10.0).assess(
            track(x=10.0, velocity=(-50.0, 0.0, 0.0))
        )
        assert result.factor_scores.relative_velocity == 1.0

    def test_an_extremely_fast_object_stays_within_bounds(self) -> None:
        result = engine().assess(track(x=10.0, velocity=(-500.0, 0.0, 0.0)))
        assert result.risk_score is not None
        assert 0.0 <= result.risk_score <= 1.0

    def test_a_fast_approaching_object_outranks_a_stationary_one_at_equal_range(self) -> None:
        moving = engine().assess(track(x=15.0, velocity=(-14.0, 0.0, 0.0)))
        still = engine().assess(track(x=15.0, velocity=(0.0, 0.0, 0.0)))
        assert moving.risk_score is not None and still.risk_score is not None
        assert moving.risk_score > still.risk_score


class TestUnknownVelocitySemantics:
    """``None`` is not zero (ADR-023). This is the distinction Phase 7 must keep."""

    def test_unknown_velocity_reports_no_closing_speed(self) -> None:
        result = engine().assess(track(velocity=None))

        assert result.closing_speed_mps is None
        assert result.speed_mps is None

    def test_unknown_velocity_drops_the_factor_rather_than_zeroing_it(self) -> None:
        result = engine().assess(track(velocity=None))

        assert RiskFactorName.CLOSING_SPEED not in result.factors
        assert result.factor_scores.relative_velocity is None

    def test_unknown_velocity_and_measured_zero_produce_different_scores(self) -> None:
        """The whole point: an unmeasured object must not look like a calm one."""
        unknown = engine().assess(track(x=10.0, velocity=None))
        standstill = engine().assess(track(x=10.0, velocity=(0.0, 0.0, 0.0)))

        assert unknown.risk_score != standstill.risk_score
        assert unknown.risk_score is not None and standstill.risk_score is not None
        # Dropping the zero-valued approach term leaves the mean higher.
        assert unknown.risk_score > standstill.risk_score

    def test_a_measured_standstill_reports_zero_not_null(self) -> None:
        result = engine().assess(track(velocity=(0.0, 0.0, 0.0)))

        assert result.closing_speed_mps == 0.0
        assert result.speed_mps == 0.0
        assert RiskFactorName.CLOSING_SPEED in result.factors

    def test_unknown_velocity_raises_uncertainty(self) -> None:
        unknown = engine().assess(track(velocity=None))
        known = engine().assess(track(velocity=(0.0, 0.0, 0.0)))

        assert UncertaintyReason.UNKNOWN_VELOCITY in unknown.uncertainty.reasons
        assert unknown.uncertainty.velocity_known is False
        assert known.uncertainty.velocity_known is True
        assert unknown.uncertainty.score > known.uncertainty.score

    def test_the_explanation_says_velocity_was_never_measured(self) -> None:
        result = engine().assess(track(velocity=None))
        assert "never measured" in result.reason


class TestTrajectoryRelevance:
    def test_an_approaching_path_reports_its_closest_point(self) -> None:
        result = engine().assess(
            track(x=30.0, velocity=(-10.0, 0.0, 0.0)),
            trajectory=trajectory((30.0, 0.0), (20.0, 0.0), (10.0, 0.0), (2.0, 0.0)),
        )
        assert result.trajectory is not None
        assert result.trajectory.min_distance_m == pytest.approx(2.0)
        assert result.trajectory.time_to_min_distance_s == pytest.approx(3.0)
        assert result.trajectory.is_approaching is True

    def test_a_receding_path_is_not_approaching(self) -> None:
        result = engine().assess(
            track(x=10.0, velocity=(10.0, 0.0, 0.0)),
            trajectory=trajectory((10.0, 0.0), (20.0, 0.0), (30.0, 0.0)),
        )
        assert result.trajectory is not None
        assert result.trajectory.min_distance_m == pytest.approx(10.0)
        assert result.trajectory.is_approaching is False

    def test_the_predicted_factor_raises_risk_for_an_approaching_path(self) -> None:
        far = track(x=30.0, velocity=(-10.0, 0.0, 0.0))
        without = engine().assess(far)
        with_path = engine().assess(
            far, trajectory=trajectory((30.0, 0.0), (20.0, 0.0), (10.0, 0.0), (1.0, 0.0))
        )
        assert without.risk_score is not None and with_path.risk_score is not None
        assert with_path.risk_score > without.risk_score

    def test_prediction_uncertainty_is_carried_through_not_recomputed(self) -> None:
        result = engine().assess(
            track(x=10.0, velocity=(-5.0, 0.0, 0.0)),
            trajectory=trajectory((10.0, 0.0), (5.0, 0.0), uncertainty_m=1.75),
        )
        assert result.trajectory is not None
        assert result.trajectory.uncertainty_at_min_m == 1.75

    def test_wide_prediction_uncertainty_is_recorded_as_a_reason(self) -> None:
        result = engine(predicted_proximity_near_m=1.0).assess(
            track(velocity=(-5.0, 0.0, 0.0)),
            trajectory=trajectory((10.0, 0.0), (5.0, 0.0), uncertainty_m=9.0),
        )
        assert UncertaintyReason.WIDE_PREDICTION_UNCERTAINTY in result.uncertainty.reasons

    def test_no_trajectory_drops_the_factor_and_raises_uncertainty(self) -> None:
        result = engine().assess(track(velocity=(0.0, 0.0, 0.0)))

        assert result.trajectory is None
        assert RiskFactorName.PREDICTED_PROXIMITY not in result.factors
        assert result.factor_scores.trajectory_overlap is None
        assert UncertaintyReason.NO_PREDICTION in result.uncertainty.reasons
        assert result.uncertainty.prediction_available is False

    def test_ties_in_distance_pick_the_earliest_point(self) -> None:
        """Deterministic tie-breaking, so the same path always reports the same time."""
        result = engine().assess(
            track(x=10.0, velocity=(0.0, 0.0, 0.0)),
            trajectory=trajectory((5.0, 0.0), (5.0, 0.0), (5.0, 0.0)),
        )
        assert result.trajectory is not None
        assert result.trajectory.time_to_min_distance_s == 0.0


class TestMapContext:
    def test_an_occupied_cell_is_reported_observed(self) -> None:
        result = engine().assess(
            track(x=3.0, y=3.0),
            spatial_map=spatial_map_with([[3.2, 3.2, 1.0], [3.4, 3.1, 2.0]]),  # type: ignore[arg-type]
        )
        assert result.map_context.observation is MapObservation.OBSERVED_OCCUPIED
        assert result.map_context.point_count == 2
        assert result.map_context.max_height_m == 2.0

    def test_an_empty_cell_is_unobserved_not_free(self) -> None:
        """Absence of returns is not evidence of free space (ADR-031, ADR-034)."""
        result = engine().assess(
            track(x=3.0, y=3.0),
            spatial_map=spatial_map_with([[-10.0, -10.0, 1.0]]),  # type: ignore[arg-type]
        )
        assert result.map_context.observation is MapObservation.OBSERVED_EMPTY
        assert result.map_context.point_count == 0
        assert result.map_context.is_observed is False

    def test_an_empty_cell_never_lowers_risk(self) -> None:
        """The critical safety property of the map component."""
        subject = track(x=3.0, y=3.0, velocity=(0.0, 0.0, 0.0))
        without_map = engine().assess(subject)
        empty_cell = engine().assess(
            subject,
            spatial_map=spatial_map_with([[-10.0, -10.0, 1.0]]),  # type: ignore[arg-type]
        )
        assert without_map.risk_score is not None and empty_cell.risk_score is not None
        assert empty_cell.risk_score >= without_map.risk_score

    def test_an_empty_cell_raises_uncertainty_instead(self) -> None:
        result = engine().assess(
            track(x=3.0, y=3.0),
            spatial_map=spatial_map_with([[-10.0, -10.0, 1.0]]),  # type: ignore[arg-type]
        )
        assert UncertaintyReason.UNOBSERVED_MAP_CONTEXT in result.uncertainty.reasons

    def test_an_object_outside_the_map_is_reported_out_of_bounds(self) -> None:
        result = engine().assess(
            track(x=500.0),
            spatial_map=spatial_map_with([[0.0, 0.0, 1.0]]),  # type: ignore[arg-type]
        )
        assert result.map_context.observation is MapObservation.OUT_OF_BOUNDS
        assert result.map_context.point_count is None

    def test_an_object_on_the_map_boundary_does_not_crash(self) -> None:
        for x in (-20.0, 19.999, 20.0):
            result = engine().assess(
                track(x=x, y=0.0),
                spatial_map=spatial_map_with([[0.0, 0.0, 1.0]]),  # type: ignore[arg-type]
            )
            assert result.map_context.observation in set(MapObservation)

    def test_no_map_is_reported_and_raises_uncertainty(self) -> None:
        result = engine().assess(track())

        assert result.map_context.observation is MapObservation.NO_MAP
        assert UncertaintyReason.NO_MAP in result.uncertainty.reasons

    def test_a_completely_empty_map_is_handled(self) -> None:
        result = engine().assess(track(x=3.0), spatial_map=spatial_map_with([]))  # type: ignore[arg-type]
        assert result.map_context.observation is MapObservation.OBSERVED_EMPTY


class TestLifecycle:
    def test_a_confirmed_track_is_assessed_normally(self) -> None:
        result = engine().assess(track(status=TrackStatus.CONFIRMED))

        assert result.status is AssessmentStatus.ASSESSED
        assert result.risk_score is not None

    def test_a_tentative_track_is_assessed_with_raised_uncertainty(self) -> None:
        tentative = engine().assess(track(status=TrackStatus.TENTATIVE))
        confirmed = engine().assess(track(status=TrackStatus.CONFIRMED))

        assert tentative.status is AssessmentStatus.ASSESSED
        assert UncertaintyReason.TENTATIVE_TRACK in tentative.uncertainty.reasons
        assert tentative.uncertainty.score > confirmed.uncertainty.score

    def test_a_coasting_track_is_assessed_with_raised_uncertainty(self) -> None:
        coasting = engine().assess(track(status=TrackStatus.COASTING, missed_frames=2))
        confirmed = engine().assess(track(status=TrackStatus.CONFIRMED))

        assert coasting.status is AssessmentStatus.ASSESSED
        assert UncertaintyReason.COASTING_TRACK in coasting.uncertainty.reasons
        assert coasting.uncertainty.score > confirmed.uncertainty.score

    def test_a_lost_track_is_not_scored(self) -> None:
        """A terminated track is history, not a present concern."""
        result = engine().assess(track(x=2.0, velocity=(-20.0, 0.0, 0.0), status=TrackStatus.LOST))

        assert result.status is AssessmentStatus.TRACK_LOST
        assert result.risk_level is RiskLevel.UNKNOWN
        assert result.risk_score is None
        assert "terminated" in result.reason

    def test_the_track_lifecycle_is_reported_on_the_assessment(self) -> None:
        result = engine().assess(track(status=TrackStatus.COASTING))
        assert result.track_status is TrackStatus.COASTING


class TestStaleObservation:
    def test_a_fresh_observation_is_not_stale(self) -> None:
        result = engine().assess(track(last_seen_offset_s=0.0))

        assert result.uncertainty.is_stale is False
        assert result.uncertainty.observation_age_s == 0.0
        assert UncertaintyReason.STALE_OBSERVATION not in result.uncertainty.reasons

    def test_an_old_observation_is_stale_and_raises_uncertainty(self) -> None:
        result = engine(stale_observation_s=0.5).assess(track(last_seen_offset_s=2.0))

        assert result.uncertainty.is_stale is True
        assert result.uncertainty.observation_age_s == pytest.approx(2.0)
        assert UncertaintyReason.STALE_OBSERVATION in result.uncertainty.reasons

    def test_staleness_is_measured_against_the_supplied_timestamp(self) -> None:
        subject = track(last_seen_offset_s=0.0)
        later = engine(stale_observation_s=0.5).assess(
            subject, timestamp=EPOCH + timedelta(seconds=3.0)
        )
        assert later.uncertainty.observation_age_s == pytest.approx(3.0)
        assert later.uncertainty.is_stale is True

    def test_a_future_observation_is_treated_as_no_age(self) -> None:
        """A negative age would describe an observation that has not happened."""
        result = engine().assess(track(last_seen_offset_s=-5.0))
        assert result.uncertainty.observation_age_s == 0.0

    def test_a_track_never_seen_is_treated_as_no_age(self) -> None:
        subject = track().model_copy(update={"last_seen": None})
        result = engine().assess(subject)
        assert result.uncertainty.observation_age_s == 0.0


class TestConfidenceSemantics:
    def test_low_track_confidence_raises_uncertainty(self) -> None:
        low = engine(low_track_confidence=0.5).assess(track(confidence=0.2))
        high = engine(low_track_confidence=0.5).assess(track(confidence=0.95))

        assert UncertaintyReason.LOW_TRACK_CONFIDENCE in low.uncertainty.reasons
        assert UncertaintyReason.LOW_TRACK_CONFIDENCE not in high.uncertainty.reasons
        assert low.uncertainty.score > high.uncertainty.score

    def test_confidence_is_carried_through_unchanged(self) -> None:
        """Phase 3's geometric fit score, not reinterpreted as a probability."""
        result = engine().assess(track(confidence=0.37))
        assert result.uncertainty.track_confidence == 0.37

    def test_confidence_does_not_change_the_risk_score(self) -> None:
        """Risk and uncertainty stay separate quantities (ADR-033)."""
        low = engine().assess(track(confidence=0.1))
        high = engine().assess(track(confidence=0.99))
        assert low.risk_score == high.risk_score


class TestUncertaintyModel:
    def test_a_fully_observed_track_with_context_is_confident(self) -> None:
        result = engine().assess(
            track(velocity=(0.0, 0.0, 0.0), confidence=0.95),
            trajectory=trajectory((10.0, 0.0), (10.0, 0.0)),
            spatial_map=spatial_map_with([[10.2, 0.2, 1.0]]),  # type: ignore[arg-type]
        )
        assert result.uncertainty.reasons == []
        assert result.uncertainty.score == 0.0
        assert result.uncertainty.is_confident is True

    def test_uncertainty_stays_bounded_when_everything_is_missing(self) -> None:
        result = engine(stale_observation_s=0.1).assess(
            track(
                velocity=None,
                confidence=0.0,
                status=TrackStatus.COASTING,
                last_seen_offset_s=10.0,
            )
        )
        assert result.uncertainty.score == 1.0
        assert len(result.uncertainty.reasons) >= 5

    def test_uncertainty_is_never_folded_into_the_risk_score(self) -> None:
        """Two tracks identical except for observability must score the same risk."""
        clean = engine().assess(track(x=10.0, velocity=(0.0, 0.0, 0.0), confidence=0.95))
        murky = engine(stale_observation_s=0.1).assess(
            track(
                x=10.0,
                velocity=(0.0, 0.0, 0.0),
                confidence=0.05,
                status=TrackStatus.TENTATIVE,
                last_seen_offset_s=5.0,
            )
        )
        assert clean.risk_score == murky.risk_score
        assert murky.uncertainty.score > clean.uncertainty.score

    def test_the_reasons_stay_visible_beside_the_scalar(self) -> None:
        result = engine().assess(track(velocity=None))
        assert result.uncertainty.reasons
        assert result.factor_scores.uncertainty == result.uncertainty.score


class TestRiskLevels:
    @pytest.mark.parametrize(
        ("score", "expected"),
        [
            (0.0, RiskLevel.LOW),
            (0.34, RiskLevel.LOW),
            (0.35, RiskLevel.MEDIUM),
            (0.59, RiskLevel.MEDIUM),
            (0.60, RiskLevel.HIGH),
            (0.84, RiskLevel.HIGH),
            (0.85, RiskLevel.CRITICAL),
            (1.0, RiskLevel.CRITICAL),
        ],
    )
    def test_classification_uses_the_configured_thresholds(
        self, score: float, expected: RiskLevel
    ) -> None:
        assert engine().classify(score) is expected

    def test_classification_rejects_a_score_outside_the_scale(self) -> None:
        with pytest.raises(ValueError, match=r"must be in \[0, 1\]"):
            engine().classify(1.5)

    def test_a_close_fast_approaching_object_reaches_a_high_band(self) -> None:
        result = engine().assess(
            track(x=4.0, velocity=(-20.0, 0.0, 0.0)),
            trajectory=trajectory((4.0, 0.0), (0.5, 0.0)),
        )
        assert result.risk_level in (RiskLevel.HIGH, RiskLevel.CRITICAL)

    def test_a_distant_stationary_object_is_low(self) -> None:
        result = engine().assess(track(x=38.0, velocity=(0.0, 0.0, 0.0)))
        assert result.risk_level is RiskLevel.LOW

    def test_unknown_is_not_a_point_on_the_scored_scale(self) -> None:
        from adaptx.models.risk import SCORED_RISK_LEVELS

        assert RiskLevel.UNKNOWN not in SCORED_RISK_LEVELS
        assert len(SCORED_RISK_LEVELS) == 4


class TestUnknownAssessment:
    def test_an_unknown_assessment_carries_no_score(self) -> None:
        result = engine().assess(track(status=TrackStatus.LOST))

        assert result.risk_score is None
        assert result.is_assessed is False

    def test_a_score_is_required_for_a_scored_level(self) -> None:
        with pytest.raises(ValidationError, match="requires a risk_score"):
            RiskAssessment(
                track_id=0,
                status=AssessmentStatus.ASSESSED,
                risk_level=RiskLevel.HIGH,
                risk_score=None,
                distance_m=1.0,
                track_status=TrackStatus.CONFIRMED,
                object_class="vehicle",
                map_context=engine().assess(track()).map_context,
                uncertainty=engine().assess(track()).uncertainty,
                factor_scores=RiskFactors(),
                reason="x",
            )

    def test_an_unknown_assessment_must_not_carry_a_score(self) -> None:
        with pytest.raises(ValidationError, match="must not carry a risk_score"):
            RiskAssessment(
                track_id=0,
                status=AssessmentStatus.TRACK_LOST,
                risk_level=RiskLevel.UNKNOWN,
                risk_score=0.5,
                distance_m=1.0,
                track_status=TrackStatus.LOST,
                object_class="vehicle",
                map_context=engine().assess(track()).map_context,
                uncertainty=engine().assess(track()).uncertainty,
                factor_scores=RiskFactors(),
                reason="x",
            )

    def test_an_assessed_track_cannot_be_unknown(self) -> None:
        with pytest.raises(ValidationError):
            RiskAssessment(
                track_id=0,
                status=AssessmentStatus.ASSESSED,
                risk_level=RiskLevel.UNKNOWN,
                risk_score=None,
                distance_m=1.0,
                track_status=TrackStatus.CONFIRMED,
                object_class="vehicle",
                map_context=engine().assess(track()).map_context,
                uncertainty=engine().assess(track()).uncertainty,
                factor_scores=RiskFactors(),
                reason="x",
            )

    def test_zero_weights_on_every_factor_cannot_be_configured(self) -> None:
        with pytest.raises(ValidationError, match="must not all be zero"):
            RiskSettings(
                weight_proximity=0.0,
                weight_closing_speed=0.0,
                weight_predicted_proximity=0.0,
            )


class TestObjectClass:
    @pytest.mark.parametrize(
        "object_class",
        [
            ObjectClass.PEDESTRIAN,
            ObjectClass.CYCLIST,
            ObjectClass.VEHICLE,
            ObjectClass.OBSTACLE,
            ObjectClass.UNKNOWN,
        ],
    )
    def test_every_class_is_assessable(self, object_class: ObjectClass) -> None:
        result = engine().assess(track(object_class=object_class))

        assert result.status is AssessmentStatus.ASSESSED
        assert result.object_class == object_class.value

    def test_class_alone_does_not_determine_risk(self) -> None:
        """A far pedestrian is less concerning than a closing vehicle."""
        pedestrian = engine().assess(
            track(x=35.0, object_class=ObjectClass.PEDESTRIAN, velocity=(0.0, 0.0, 0.0))
        )
        vehicle = engine().assess(
            track(x=8.0, object_class=ObjectClass.VEHICLE, velocity=(-14.0, 0.0, 0.0))
        )
        assert pedestrian.risk_score is not None and vehicle.risk_score is not None
        assert vehicle.risk_score > pedestrian.risk_score

    def test_the_class_appears_in_the_explanation(self) -> None:
        result = engine().assess(track(object_class=ObjectClass.CYCLIST))
        assert "cyclist" in result.reason


class TestExplanation:
    def test_the_explanation_reports_the_level_and_distance(self) -> None:
        result = engine().assess(track(x=12.3, velocity=(0.0, 0.0, 0.0)))

        assert result.reason.startswith(result.risk_level.value.capitalize())
        assert "12.3 m" in result.reason

    def test_the_explanation_reports_a_measured_approach(self) -> None:
        result = engine().assess(track(x=10.0, velocity=(-6.0, 0.0, 0.0)))
        assert "closing at 6.0 m/s" in result.reason

    def test_the_explanation_reports_a_predicted_approach(self) -> None:
        result = engine().assess(
            track(x=20.0, velocity=(-5.0, 0.0, 0.0)),
            trajectory=trajectory((20.0, 0.0), (3.0, 0.0)),
        )
        assert "predicted to pass within 3.0 m" in result.reason

    def test_the_explanation_says_when_a_cell_is_unobserved(self) -> None:
        result = engine().assess(
            track(x=3.0, y=3.0),
            spatial_map=spatial_map_with([[-10.0, -10.0, 1.0]]),  # type: ignore[arg-type]
        )
        assert "unobserved rather than clear" in result.reason

    @pytest.mark.parametrize(
        "overclaim",
        [
            "probability",
            "guaranteed",
            "will collide",
            "collision detected",
            "safe",
            "validated",
            "calibrated",
        ],
    )
    def test_the_explanation_never_overclaims(self, overclaim: str) -> None:
        """The engine describes what it measured, never what it cannot know."""
        result = engine().assess(
            track(velocity=(-9.0, 0.0, 0.0)),
            trajectory=trajectory((10.0, 0.0), (1.0, 0.0)),
        )
        assert overclaim not in result.reason.lower()


class TestFactors:
    def test_only_computed_factors_are_listed(self) -> None:
        result = engine().assess(track(velocity=None))

        assert result.factors == [RiskFactorName.PROXIMITY]

    def test_all_three_factors_are_listed_when_available(self) -> None:
        result = engine().assess(
            track(velocity=(-5.0, 0.0, 0.0)),
            trajectory=trajectory((10.0, 0.0), (5.0, 0.0)),
        )
        assert set(result.factors) == {
            RiskFactorName.PROXIMITY,
            RiskFactorName.CLOSING_SPEED,
            RiskFactorName.PREDICTED_PROXIMITY,
        }

    def test_a_zero_weighted_factor_is_not_listed(self) -> None:
        result = engine(weight_closing_speed=0.0).assess(track(velocity=(-5.0, 0.0, 0.0)))
        assert RiskFactorName.CLOSING_SPEED not in result.factors

    def test_dropping_a_factor_renormalises_rather_than_zeroing(self) -> None:
        """proximity=1 alone must score 1.0, not 0.5, when the other factor is unknown."""
        result = engine(
            weight_proximity=0.5, weight_closing_speed=0.5, weight_predicted_proximity=0.0
        ).assess(track(x=1.0, velocity=None))

        assert result.factor_scores.proximity == 1.0
        assert result.risk_score == pytest.approx(1.0)


class TestAggregateResult:
    def test_every_track_appears_in_the_result(self) -> None:
        tracks = [track(track_id=i, x=float(5 + i * 5)) for i in range(4)]
        result = engine().assess_many(tracks)

        assert result.considered_track_count == 4
        assert len(result.assessments) == 4

    def test_the_aggregate_is_a_maximum_not_a_mean(self) -> None:
        """One critical object must not vanish behind many quiet ones (ADR-035)."""
        tracks = [track(track_id=i, x=39.0, velocity=(0.0, 0.0, 0.0)) for i in range(10)]
        tracks.append(track(track_id=99, x=1.0, velocity=(-30.0, 0.0, 0.0)))
        result = engine().assess_many(tracks)

        assert result.highest_risk_level in (RiskLevel.HIGH, RiskLevel.CRITICAL)

    def test_counts_by_level_include_unknown(self) -> None:
        tracks = [
            track(track_id=0, x=1.0, velocity=(-30.0, 0.0, 0.0)),
            track(track_id=1, x=39.0, velocity=(0.0, 0.0, 0.0)),
            track(track_id=2, status=TrackStatus.LOST),
        ]
        result = engine().assess_many(tracks)
        counts = result.counts_by_level()

        assert counts.get("unknown") == 1
        assert result.unknown_count == 1
        assert sum(counts.values()) == 3

    def test_an_empty_scene_reports_unknown_not_low(self) -> None:
        """ "Nothing is risky" and "nothing was assessed" are different."""
        result = engine().assess_many([])

        assert result.considered_track_count == 0
        assert result.highest_risk_level is RiskLevel.UNKNOWN
        assert result.highest_risk_score is None
        assert result.max_uncertainty is None

    def test_a_scene_of_only_lost_tracks_reports_unknown(self) -> None:
        result = engine().assess_many(
            [track(track_id=i, status=TrackStatus.LOST) for i in range(3)]
        )
        assert result.highest_risk_level is RiskLevel.UNKNOWN
        assert result.unknown_count == 3

    def test_one_object_is_handled(self) -> None:
        result = engine().assess_many([track()])
        assert result.considered_track_count == 1

    def test_many_objects_are_handled(self) -> None:
        result = engine().assess_many([track(track_id=i, x=float(i % 40 + 1)) for i in range(200)])
        assert result.considered_track_count == 200

    def test_the_per_call_limit_bounds_the_result(self) -> None:
        result = engine(max_assessed_tracks=10).assess_many([track(track_id=i) for i in range(50)])
        assert result.considered_track_count == 10

    def test_trajectories_are_matched_by_track_id(self) -> None:
        tracks = [track(track_id=0, x=20.0), track(track_id=1, x=20.0)]
        result = engine().assess_many(
            tracks, trajectories=[trajectory((20.0, 0.0), (2.0, 0.0), track_id=1)]
        )
        by_id = {a.track_id: a for a in result.assessments}

        assert by_id[0].trajectory is None
        assert by_id[1].trajectory is not None

    def test_accounting_is_enforced_by_the_contract(self) -> None:
        with pytest.raises(ValidationError, match="must equal"):
            RiskAssessmentResult(
                frame_id=0,
                sensor_id="x",
                engine="e",
                considered_track_count=5,
                duration_ms=0.0,
                configuration=engine().configuration,
            )

    def test_the_duration_is_measured(self) -> None:
        result = engine().assess_many([track(track_id=i) for i in range(20)])
        assert result.duration_ms > 0.0
        assert math.isfinite(result.duration_ms)


class TestDeterminism:
    def test_the_same_scene_always_produces_the_same_result(self) -> None:
        tracks = [
            track(track_id=i, x=float(i + 3), velocity=(-float(i), 0.0, 0.0)) for i in range(5)
        ]
        first = engine().assess_many(tracks)
        second = engine().assess_many(tracks)

        assert first.model_dump(exclude={"duration_ms"}) == second.model_dump(
            exclude={"duration_ms"}
        )

    def test_identical_distances_produce_identical_scores(self) -> None:
        a = engine().assess(track(track_id=0, x=10.0, velocity=(0.0, 0.0, 0.0)))
        b = engine().assess(track(track_id=1, x=10.0, velocity=(0.0, 0.0, 0.0)))
        assert a.risk_score == b.risk_score
        assert a.risk_level is b.risk_level

    def test_track_order_does_not_change_any_assessment(self) -> None:
        a = track(track_id=0, x=8.0, velocity=(-3.0, 0.0, 0.0))
        b = track(track_id=1, x=25.0, velocity=(-9.0, 0.0, 0.0))
        forward = {x.track_id: x.risk_score for x in engine().assess_many([a, b]).assessments}
        reverse = {x.track_id: x.risk_score for x in engine().assess_many([b, a]).assessments}
        assert forward == reverse

    def test_repeated_calls_on_one_engine_do_not_drift(self) -> None:
        stage = engine()
        tracks = [track(x=12.0, velocity=(-4.0, 0.0, 0.0))]
        first = stage.assess_many(tracks)
        for _ in range(9):
            latest = stage.assess_many(tracks)
        assert first.assessments[0].risk_score == latest.assessments[0].risk_score


class TestEdgeCases:
    def test_an_object_at_the_ego_origin_does_not_divide_by_zero(self) -> None:
        result = engine().assess(track(x=0.0, y=0.0, velocity=(-5.0, 0.0, 0.0)))

        assert result.distance_m == 0.0
        assert result.closing_speed_mps is None
        assert result.risk_score is not None

    def test_a_non_finite_position_cannot_reach_the_engine(self) -> None:
        with pytest.raises(ValidationError, match="finite"):
            Vector3(x=math.nan, y=0.0, z=0.0)

    def test_an_ego_state_moves_the_reference_point(self) -> None:
        ego = VehicleState(timestamp=EPOCH, position=Vector3(x=10.0, y=0.0, z=0.0))
        result = engine().assess(track(x=12.0), ego_state=ego)
        assert result.distance_m == pytest.approx(2.0)

    def test_duplicate_track_ids_are_each_assessed(self) -> None:
        """The engine does not deduplicate; it reports what it was given."""
        result = engine().assess_many([track(track_id=7, x=5.0), track(track_id=7, x=30.0)])
        assert result.considered_track_count == 2

    def test_every_output_value_is_finite(self) -> None:
        result = engine().assess(
            track(x=7.3, velocity=(-11.7, 3.2, 0.0)),
            trajectory=trajectory((7.3, 0.0), (1.1, 0.4)),
        )
        assert result.risk_score is not None and math.isfinite(result.risk_score)
        assert math.isfinite(result.distance_m)
        assert result.closing_speed_mps is not None and math.isfinite(result.closing_speed_mps)
        assert math.isfinite(result.uncertainty.score)


class TestEngineContract:
    def test_the_engine_is_labelled_a_baseline(self) -> None:
        assert engine().is_baseline is True
        assert engine().name == "heuristic_risk_v1"

    def test_the_factory_builds_the_configured_engine(self) -> None:
        built = build_risk_engine(RiskSettings(proximity_near_m=9.0))
        assert isinstance(built, HeuristicRiskEngine)
        assert built.configuration.proximity_near_m == 9.0

    def test_evaluate_satisfies_the_phase_one_contract(self) -> None:
        field = engine().evaluate([track(x=5.0, velocity=(0.0, 0.0, 0.0))])

        assert field.engine == "heuristic_risk_v1"
        assert field.is_baseline is True
        assert len(field.object_risks) == 1
        assert field.cells == []

    def test_evaluate_omits_unscored_tracks_rather_than_inventing_a_score(self) -> None:
        field = engine().evaluate([track(status=TrackStatus.LOST)])
        assert field.object_risks == []

    def test_the_configuration_snapshot_travels_with_the_result(self) -> None:
        result = engine(proximity_near_m=3.0, threshold_high=0.7).assess_many([track()])

        assert result.configuration.proximity_near_m == 3.0
        assert result.configuration.threshold_high == 0.7

    def test_the_result_is_labelled_heuristic_and_not_calibrated(self) -> None:
        result = engine().assess_many([track()])

        assert result.is_baseline is True
        assert result.scoring_model == "heuristic_weighted_factors"

    def test_provenance_is_carried_from_the_track(self) -> None:
        result = engine().assess(track())
        assert result.source is DataSource.SYNTHETIC_TEST


class TestConfigurationValidation:
    def test_the_defaults_are_the_documented_baseline_values(self) -> None:
        settings = RiskSettings()

        assert settings.proximity_near_m == 5.0
        assert settings.proximity_far_m == 40.0
        assert settings.closing_speed_high_mps == 15.0
        assert settings.stale_observation_s == 0.5
        assert settings.total_weight == pytest.approx(1.0)

    def test_inverted_proximity_bounds_are_rejected(self) -> None:
        with pytest.raises(ValidationError, match="proximity_near_m"):
            RiskSettings(proximity_near_m=50.0, proximity_far_m=10.0)

    def test_unordered_thresholds_are_rejected(self) -> None:
        with pytest.raises(ValidationError, match="medium < high < critical"):
            RiskSettings(threshold_medium=0.9, threshold_high=0.5, threshold_critical=0.95)

    @pytest.mark.parametrize(
        ("field", "value"),
        [
            ("proximity_near_m", 0.0),
            ("proximity_far_m", -1.0),
            ("closing_speed_high_mps", 0.0),
            ("stale_observation_s", 0.0),
            ("low_track_confidence", 1.5),
            ("weight_proximity", -0.1),
            ("max_assessed_tracks", 0),
        ],
    )
    def test_out_of_range_configuration_is_rejected(self, field: str, value: float) -> None:
        with pytest.raises(ValidationError):
            RiskSettings(**{field: value})  # type: ignore[arg-type]


class TestPhaseEightBoundary:
    """Phase 7 must not decide spatial resolution (ADR-036)."""

    def test_an_assessment_carries_no_resolution(self) -> None:
        result = engine().assess(track())
        dumped = result.model_dump()

        for forbidden in ("resolution_m", "resolution", "cell_size_m", "resolution_level"):
            assert forbidden not in dumped

    def test_a_result_carries_no_resolution(self) -> None:
        dumped = engine().assess_many([track()]).model_dump()

        assert "resolution" not in dumped
        assert "resolution_m" not in dumped.get("configuration", {})

    def test_the_engine_module_imports_no_resolution_type(self) -> None:
        import adaptx.risk.heuristic as module

        source = module.__doc__ or ""
        assert "does **not** choose spatial resolution" in source
        for forbidden in ("ResolutionController", "ResolutionDecision", "MapSettings"):
            assert not hasattr(module, forbidden)
