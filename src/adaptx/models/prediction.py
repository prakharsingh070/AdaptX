"""Trajectory-prediction contracts.

Predicted positions are always distinguishable from measured positions: they
live in :class:`PredictedTrajectory`, never in
:class:`adaptx.models.tracking.TrackedObject`.

Field naming
------------
``timestamp`` is the *source* time a trajectory was predicted from - the frame
whose tracking result produced it - and every point's ``time_offset_s`` is
relative to it. ``horizon_s`` and ``predictor_name`` carry what other projects
might call ``prediction_horizon_s`` and ``model_name``; the shorter names are
kept because these contracts predate Phase 5 and renaming them would break
consumers for no gain.
"""

from __future__ import annotations

from enum import StrEnum

from pydantic import Field, model_validator

from adaptx.models.common import (
    CoordinateFrame,
    DataSource,
    TimestampedModel,
    Vector3,
)


class PredictionStatus(StrEnum):
    """Outcome of attempting to predict one track (Phase 5).

    Two of these accompany a trajectory that was produced; the rest explain why
    one was not, and appear on
    :class:`~adaptx.models.prediction_result.SkippedTrack`. A caller can always
    tell "nothing was predicted" from "prediction was attempted and declined",
    the same distinction ADR-022 drew for detection.

    ``PREDICTED``
        Extrapolated from a velocity measured in the current frame.
    ``EXTRAPOLATED``
        Produced for a coasting track, from an observation older than the
        prediction time. Marked separately because the starting position is
        itself already an extrapolation, not a fresh measurement.
    ``INSUFFICIENT_VELOCITY``
        The track has no measured velocity. Null is not zero (ADR-023), so no
        trajectory is emitted rather than a fabricated standstill.
    ``INVALID_VELOCITY``
        Measured speed exceeds the configured sanity bound. The value is
        rejected, never clipped: a clipped velocity is a number no sensor
        produced.
    ``STALE_OBSERVATION``
        The last observation is older than the prediction horizon, so the
        output would be more gap-filling than prediction.
    ``TRACK_LOST``
        The track is terminated and no active prediction is published for it.
    ``LIMIT_EXCEEDED``
        The configured per-call track limit was reached before this track.
    """

    PREDICTED = "predicted"
    EXTRAPOLATED = "extrapolated"
    INSUFFICIENT_VELOCITY = "insufficient_velocity"
    INVALID_VELOCITY = "invalid_velocity"
    STALE_OBSERVATION = "stale_observation"
    TRACK_LOST = "track_lost"
    LIMIT_EXCEEDED = "limit_exceeded"


#: Statuses that accompany an emitted trajectory rather than a skipped track.
PRODUCED_STATUSES = frozenset({PredictionStatus.PREDICTED, PredictionStatus.EXTRAPOLATED})


class TrajectoryPoint(TimestampedModel):
    """A single predicted state at ``time_offset_s`` after the prediction time."""

    time_offset_s: float = Field(ge=0.0, description="Seconds ahead of the prediction time.")
    position: Vector3
    velocity: Vector3 | None = None
    confidence: float = Field(ge=0.0, le=1.0)
    position_uncertainty_m: float = Field(
        default=0.0, ge=0.0, description="1-sigma positional uncertainty in metres."
    )


class PredictedTrajectory(TimestampedModel):
    """Predicted future path of one tracked object.

    ``timestamp`` is the source time the prediction was made from; each point's
    ``time_offset_s`` is measured forward from it, so absolute future times are
    derived arithmetically and never from a wall clock.
    """

    track_id: int = Field(ge=0)
    horizon_s: float = Field(gt=0.0, description="Prediction horizon in seconds.")
    timestep_s: float = Field(gt=0.0, description="Spacing between trajectory points.")
    points: list[TrajectoryPoint] = Field(min_length=1)
    confidence: float = Field(ge=0.0, le=1.0, description="Overall trajectory confidence.")
    predictor_name: str = Field(
        min_length=1, description="Predictor that produced this trajectory, for traceability."
    )
    status: PredictionStatus = Field(
        default=PredictionStatus.PREDICTED,
        description=(
            "How this trajectory was produced. EXTRAPOLATED marks a coasting "
            "track, whose starting position is older than the prediction time."
        ),
    )
    observation_age_s: float = Field(
        default=0.0,
        ge=0.0,
        description=(
            "Measured seconds between the last observation of this track and "
            "the prediction time. 0.0 for a track seen in the current frame; "
            "positive for a coasting track, whose whole trajectory is shifted "
            "by it."
        ),
    )
    coordinate_frame: CoordinateFrame = CoordinateFrame.EGO
    source: DataSource = DataSource.LIVE_SENSOR

    @model_validator(mode="after")
    def _check_horizon(self) -> PredictedTrajectory:
        last_offset = self.points[-1].time_offset_s
        if last_offset > self.horizon_s:
            raise ValueError(
                f"trajectory point at t+{last_offset}s exceeds horizon {self.horizon_s}s"
            )
        offsets = [point.time_offset_s for point in self.points]
        if offsets != sorted(offsets):
            raise ValueError("trajectory points must be ordered by increasing time_offset_s")
        return self

    @property
    def is_extrapolated_from_stale_observation(self) -> bool:
        """Whether this trajectory starts from an observation older than its source time."""
        return self.status is PredictionStatus.EXTRAPOLATED
