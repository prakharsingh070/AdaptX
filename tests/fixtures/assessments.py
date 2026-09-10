"""Deterministic risk assessments and trajectories for Phase 8 tests.

Builds :class:`~adaptx.models.risk_assessment.RiskAssessment` instances
directly rather than running the Phase 7 engine, so a resolution test
exercises the resolution policy and nothing else. If these went through the
engine, a controller failure and a risk-scoring failure would be
indistinguishable - the integration suite covers the real chain.

Building them by hand is also the only way to reach the cases that matter
most here: an assessment with ``risk_score = None`` is easy to construct and
awkward to provoke, and it is precisely the input the controller must not
coerce to zero.

Coordinate convention (ADR-009): +x forward, +y left, +z up, metres, ego
reference at the origin.
"""

from __future__ import annotations

import math
from datetime import UTC, datetime

from adaptx.models.common import ObjectClass, Vector3
from adaptx.models.prediction import PredictedTrajectory, PredictionStatus, TrajectoryPoint
from adaptx.models.risk import RiskFactors, RiskLevel
from adaptx.models.risk_assessment import (
    AssessmentStatus,
    MapContext,
    MapObservation,
    RiskAssessment,
    TrajectoryRelevance,
    UncertaintyBreakdown,
)
from adaptx.models.tracking import TrackedObject, TrackStatus
from tests.fixtures.sequences import EPOCH, track

__all__ = [
    "EPOCH",
    "assessed",
    "at_position",
    "straight_trajectory",
    "unknown_risk",
]


def _uncertainty(
    score: float,
    *,
    velocity_known: bool = True,
    prediction_available: bool = True,
    track_confidence: float = 0.8,
    observation_age_s: float = 0.0,
    is_stale: bool = False,
) -> UncertaintyBreakdown:
    """A breakdown with an explicit score and no invented reasons."""
    return UncertaintyBreakdown(
        score=score,
        reasons=[],
        observation_age_s=observation_age_s,
        is_stale=is_stale,
        velocity_known=velocity_known,
        prediction_available=prediction_available,
        track_confidence=track_confidence,
    )


def assessed(
    *,
    track_id: int = 0,
    risk_score: float = 0.5,
    uncertainty: float = 0.2,
    distance_m: float | None = None,
    position: tuple[float, float, float] = (10.0, 0.0, 0.0),
    speed_mps: float | None = 2.0,
    closing_speed_mps: float | None = 2.0,
    object_class: ObjectClass = ObjectClass.VEHICLE,
    track_status: TrackStatus = TrackStatus.CONFIRMED,
    trajectory: TrajectoryRelevance | None = None,
    observation: MapObservation = MapObservation.OBSERVED_OCCUPIED,
    timestamp: datetime | None = None,
) -> RiskAssessment:
    """A scored assessment with every quantity stated explicitly.

    ``distance_m`` defaults to the planar distance implied by ``position``, so
    the assessment and the track it describes cannot silently disagree.
    """
    planar = math.hypot(position[0], position[1]) if distance_m is None else distance_m
    return RiskAssessment(
        timestamp=timestamp if timestamp is not None else EPOCH,
        track_id=track_id,
        status=AssessmentStatus.ASSESSED,
        risk_level=RiskLevel.MEDIUM,
        risk_score=risk_score,
        distance_m=planar,
        closing_speed_mps=closing_speed_mps,
        speed_mps=speed_mps,
        track_status=track_status,
        object_class=object_class.value,
        trajectory=trajectory,
        map_context=MapContext(observation=observation),
        uncertainty=_uncertainty(uncertainty),
        factors=[],
        factor_scores=RiskFactors(),
        reason="synthetic test assessment",
    )


def unknown_risk(
    *,
    track_id: int = 0,
    uncertainty: float = 0.9,
    position: tuple[float, float, float] = (10.0, 0.0, 0.0),
    distance_m: float | None = None,
    speed_mps: float | None = None,
    status: AssessmentStatus = AssessmentStatus.INSUFFICIENT_DATA,
    track_status: TrackStatus = TrackStatus.TENTATIVE,
    timestamp: datetime | None = None,
) -> RiskAssessment:
    """An assessment that could not be scored: ``UNKNOWN`` with a **null** score.

    The contract enforces the pairing, so this cannot accidentally be built
    with a number in it. It is the input the controller must never read as
    low risk.
    """
    planar = math.hypot(position[0], position[1]) if distance_m is None else distance_m
    return RiskAssessment(
        timestamp=timestamp if timestamp is not None else EPOCH,
        track_id=track_id,
        status=status,
        risk_level=RiskLevel.UNKNOWN,
        risk_score=None,
        distance_m=planar,
        closing_speed_mps=None,
        speed_mps=speed_mps,
        track_status=track_status,
        object_class=ObjectClass.UNKNOWN.value,
        trajectory=None,
        map_context=MapContext(observation=MapObservation.OBSERVED_EMPTY),
        uncertainty=_uncertainty(
            uncertainty, velocity_known=False, prediction_available=False, track_confidence=0.3
        ),
        factors=[],
        factor_scores=RiskFactors(),
        reason="synthetic test assessment: nothing could be computed",
    )


def at_position(
    position: tuple[float, float, float],
    *,
    track_id: int = 0,
    velocity: tuple[float, float, float] | None = (2.0, 0.0, 0.0),
    status: TrackStatus = TrackStatus.CONFIRMED,
    object_class: ObjectClass = ObjectClass.VEHICLE,
) -> TrackedObject:
    """A track at a known location, which is where spatial influence comes from."""
    return track(
        position,
        velocity,
        track_id=track_id,
        status=status,
        object_class=object_class,
    )


def straight_trajectory(
    start: tuple[float, float],
    velocity: tuple[float, float],
    *,
    track_id: int = 0,
    horizon_s: float = 3.0,
    timestep_s: float = 0.5,
    uncertainty_m: float = 0.5,
    timestamp: datetime | None = None,
) -> PredictedTrajectory:
    """A constant-velocity path from ``start``, sampled ``t+0`` to the horizon.

    Mirrors what the Phase 5 predictor produces, without running it: the points
    are exactly ``start + velocity * t``, so the regions a corridor should
    reach are arithmetic rather than a guess.
    """
    points: list[TrajectoryPoint] = []
    steps = round(horizon_s / timestep_s)
    for index in range(steps + 1):
        offset = index * timestep_s
        points.append(
            TrajectoryPoint(
                timestamp=timestamp if timestamp is not None else EPOCH,
                time_offset_s=offset,
                position=Vector3(
                    x=start[0] + velocity[0] * offset,
                    y=start[1] + velocity[1] * offset,
                    z=0.0,
                ),
                velocity=Vector3(x=velocity[0], y=velocity[1], z=0.0),
                confidence=0.8,
                position_uncertainty_m=uncertainty_m,
            )
        )
    return PredictedTrajectory(
        timestamp=timestamp if timestamp is not None else EPOCH,
        track_id=track_id,
        horizon_s=horizon_s,
        timestep_s=timestep_s,
        points=points,
        confidence=0.8,
        predictor_name="constant_velocity_v1",
        status=PredictionStatus.PREDICTED,
        observation_age_s=0.0,
    )


def utc(seconds: float) -> datetime:
    """A timestamp ``seconds`` after the fixed epoch, in UTC."""
    return datetime.fromtimestamp(EPOCH.timestamp() + seconds, tz=UTC)
