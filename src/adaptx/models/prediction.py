"""Trajectory-prediction contracts.

Predicted positions are always distinguishable from measured positions: they
live in :class:`PredictedTrajectory`, never in
:class:`adaptx.models.tracking.TrackedObject`.
"""

from __future__ import annotations

from pydantic import Field, model_validator

from adaptx.models.common import (
    CoordinateFrame,
    DataSource,
    TimestampedModel,
    Vector3,
)


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
    """Predicted future path of one tracked object."""

    track_id: int = Field(ge=0)
    horizon_s: float = Field(gt=0.0, description="Prediction horizon in seconds.")
    timestep_s: float = Field(gt=0.0, description="Spacing between trajectory points.")
    points: list[TrajectoryPoint] = Field(min_length=1)
    confidence: float = Field(ge=0.0, le=1.0, description="Overall trajectory confidence.")
    predictor_name: str = Field(
        min_length=1, description="Predictor that produced this trajectory, for traceability."
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
