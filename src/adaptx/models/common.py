"""Shared primitives for every ADAPT-X data contract.

Contract rules (see ``docs/knowledge-base/17_data-schema.md``):

* every public model carries a ``schema_version``;
* every model that describes observed state carries a timezone-aware UTC
  ``timestamp`` and a ``coordinate_frame``;
* every model that can originate from something other than a live sensor
  carries a ``source`` so simulated, replayed and synthetic data are never
  mistaken for measurements;
* units are metres, metres/second, metres/second^2, radians and seconds unless
  a field name states otherwise.
"""

from __future__ import annotations

import math
from datetime import UTC, datetime
from enum import StrEnum

from pydantic import BaseModel, ConfigDict, Field, field_validator

#: Version of the ADAPT-X data contracts. Bump on any breaking field change.
SCHEMA_VERSION = "1.0"


def utc_now() -> datetime:
    """Current time as a timezone-aware UTC datetime."""
    return datetime.now(tz=UTC)


class CoordinateFrame(StrEnum):
    """Reference frame a spatial quantity is expressed in."""

    LIDAR = "lidar"
    EGO = "ego"
    WORLD = "world"
    MAP = "map"


class DataSource(StrEnum):
    """Provenance of a piece of data.

    ADAPT-X never presents simulated, replayed or synthetic values as sensor
    measurements; this field makes the distinction explicit end to end.
    """

    LIVE_SENSOR = "live_sensor"
    SIMULATION = "simulation"
    REPLAY = "replay"
    SYNTHETIC_TEST = "synthetic_test"
    UNAVAILABLE = "unavailable"


class ObjectClass(StrEnum):
    """Object classes ADAPT-X reasons about."""

    VEHICLE = "vehicle"
    PEDESTRIAN = "pedestrian"
    CYCLIST = "cyclist"
    OBSTACLE = "obstacle"
    UNKNOWN = "unknown"


class AdaptXModel(BaseModel):
    """Base class for ADAPT-X data contracts.

    Data models hold state and validation only. Business logic belongs in the
    perception, mapping, risk, tracking and prediction modules.
    """

    model_config = ConfigDict(extra="forbid", validate_assignment=True)


class TimestampedModel(AdaptXModel):
    """A model describing state observed at a point in time."""

    schema_version: str = SCHEMA_VERSION
    timestamp: datetime = Field(default_factory=utc_now)

    @field_validator("timestamp")
    @classmethod
    def _require_timezone(cls, value: datetime) -> datetime:
        """Reject naive datetimes and normalise to UTC."""
        if value.tzinfo is None:
            raise ValueError("timestamp must be timezone-aware (use UTC)")
        return value.astimezone(UTC)


class Vector3(AdaptXModel):
    """A 3D vector in metres (position) or metres/second (velocity)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @field_validator("x", "y", "z")
    @classmethod
    def _require_finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("vector components must be finite")
        return value

    @property
    def magnitude(self) -> float:
        """Euclidean norm."""
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def distance_to(self, other: Vector3) -> float:
        """Euclidean distance to ``other``."""
        return math.sqrt(
            (self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2
        )


class Dimensions(AdaptXModel):
    """Axis-aligned extent of a body in metres."""

    length: float = Field(gt=0)
    width: float = Field(gt=0)
    height: float = Field(gt=0)


class BoundingBox3D(AdaptXModel):
    """Oriented 3D bounding box.

    ``yaw_rad`` is the rotation about the z axis of the frame the box is
    expressed in, measured counter-clockwise from the +x axis.
    """

    center: Vector3
    dimensions: Dimensions
    yaw_rad: float = 0.0

    @field_validator("yaw_rad")
    @classmethod
    def _require_finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("yaw_rad must be finite")
        return value

    @property
    def volume_m3(self) -> float:
        """Box volume in cubic metres."""
        return self.dimensions.length * self.dimensions.width * self.dimensions.height
