"""Ego-vehicle state contract."""

from __future__ import annotations

import math

from pydantic import Field, field_validator

from adaptx.models.common import (
    CoordinateFrame,
    DataSource,
    Dimensions,
    TimestampedModel,
    Vector3,
)


class VehicleState(TimestampedModel):
    """Kinematic state of the ego vehicle at a point in time.

    Units: position in metres, velocity in m/s, acceleration in m/s^2, heading
    in radians counter-clockwise from the +x axis of ``coordinate_frame``.
    """

    position: Vector3 = Field(default_factory=Vector3)
    velocity: Vector3 = Field(default_factory=Vector3)
    acceleration: Vector3 = Field(default_factory=Vector3)
    heading_rad: float = 0.0
    dimensions: Dimensions = Field(
        default_factory=lambda: Dimensions(length=4.5, width=1.8, height=1.5),
        description="Ego footprint; the default is a generic passenger-car size.",
    )
    coordinate_frame: CoordinateFrame = CoordinateFrame.WORLD
    source: DataSource = DataSource.LIVE_SENSOR

    @field_validator("heading_rad")
    @classmethod
    def _require_finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("heading_rad must be finite")
        return value

    @property
    def speed_mps(self) -> float:
        """Scalar speed in metres per second."""
        return self.velocity.magnitude
