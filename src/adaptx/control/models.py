"""Contracts of the baseline ego controller (live simulation extension).

A :class:`ControlCommand` is what the policy decided for one frame and why;
:class:`EgoObservation` is what it decided from. Neither carries a CARLA
type, and neither carries ground truth about other actors: the observation
is the ego's own odometry plus the map's lane geometry under it, and the
policy reads the perception stack's outputs for everything else.
"""

from __future__ import annotations

from enum import StrEnum

from pydantic import Field, model_validator

from adaptx.models.common import AdaptXModel, DataSource
from adaptx.models.risk import RiskLevel


class ControllerState(StrEnum):
    """What the governor is doing, coarsely."""

    CRUISING = "CRUISING"
    SLOWING = "SLOWING"
    HOLDING = "HOLDING"
    EMERGENCY_BRAKING = "EMERGENCY_BRAKING"
    STOPPED = "STOPPED"
    RESUMING = "RESUMING"


class EgoObservation(AdaptXModel):
    """The ego's own state as the controller sees it.

    ``speed_mps`` and ``heading_rad`` are the simulator's report of the ego
    (odometry, labelled ``SIMULATION``); ``lane_heading_error_rad`` and
    ``lane_offset_m`` come from the map's lane centre ahead of the ego, which
    is road geometry, not a perceived object. None of this says anything about
    other actors.
    """

    speed_mps: float = Field(ge=0.0)
    heading_rad: float
    lane_heading_error_rad: float | None = Field(
        default=None,
        description="Yaw from the ego heading to the lane centre ahead; null when unknown.",
    )
    lane_offset_m: float | None = Field(
        default=None, description="Signed lateral offset from the lane centre (+ = left)."
    )
    source: DataSource = DataSource.SIMULATION


class ControlCommand(AdaptXModel):
    """One frame's actuation and the reasoning behind it.

    ``throttle``, ``brake`` and ``steer`` are the normalised commands a
    vehicle interface applies; the rest records what produced them so the
    dashboard can show *why* the ego slowed, not just that it did.
    """

    throttle: float = Field(ge=0.0, le=1.0)
    brake: float = Field(ge=0.0, le=1.0)
    steer: float = Field(
        ge=-1.0,
        le=1.0,
        description=(
            "Normalised steer in the ADAPT-X convention: positive turns LEFT (+Y). "
            "The CARLA adapter flips the sign once at the boundary (ADR-043)."
        ),
    )
    target_speed_mps: float = Field(ge=0.0, description="The speed the rules chose.")
    setpoint_mps: float = Field(
        ge=0.0,
        description=(
            "The speed the pedals follow: the target, rate-limited on the way up by "
            "the acceleration limit, applied at once on the way down."
        ),
    )
    state: ControllerState
    governing_level: RiskLevel = Field(
        description=(
            "The highest risk level among objects in the ego's path, which sets the "
            "target speed; LOW when nothing is in the path."
        )
    )
    scene_level: RiskLevel = Field(
        description=(
            "The highest risk level over every assessed object, as the risk engine "
            "reported it. Shown, not acted on, when the object is beside the path."
        )
    )
    governing_track_id: int | None = Field(
        default=None, description="The in-path track that set the target, if any."
    )
    nearest_in_path_m: float | None = Field(
        default=None,
        ge=0.0,
        description="Distance of the closest in-path object, from its risk assessment.",
    )
    reason: str = Field(min_length=1)
    is_baseline: bool = Field(
        default=True,
        description="Always true: this is a simulation baseline, not a validated controller.",
    )

    @model_validator(mode="after")
    def _one_pedal(self) -> ControlCommand:
        if self.throttle > 0.0 and self.brake > 0.0:
            raise ValueError("a command applies throttle or brake, never both")
        return self


class ControlConfiguration(AdaptXModel):
    """Snapshot of the governor's parameters, for the record and the dashboard."""

    policy: str = Field(min_length=1)
    max_speed_mps: float
    medium_speed_mps: float
    high_speed_mps: float
    unknown_speed_mps: float
    min_safe_distance_m: float
    emergency_distance_m: float
    max_acceleration_mps2: float
    max_deceleration_mps2: float
    steering_limit: float
    path_half_width_m: float
    resume_dwell_frames: int
    is_baseline: bool = True


__all__ = ["ControlCommand", "ControlConfiguration", "ControllerState", "EgoObservation"]
