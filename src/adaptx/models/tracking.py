"""Tracked-object contract (multi-frame perception output)."""

from __future__ import annotations

import math
from datetime import UTC, datetime
from enum import StrEnum

from pydantic import Field, field_validator

from adaptx.models.common import (
    CoordinateFrame,
    DataSource,
    ObjectClass,
    TimestampedModel,
    Vector3,
)


class TrackStatus(StrEnum):
    """Lifecycle state of a track.

    See ``docs/knowledge-base/08_tracking.md``. Transition rules are the
    responsibility of the tracker implementation, which does not exist yet.
    """

    TENTATIVE = "tentative"
    CONFIRMED = "confirmed"
    COASTING = "coasting"
    LOST = "lost"


class TrackedObject(TimestampedModel):
    """An object with persistent identity across frames.

    ``track_id`` is stable for the lifetime of the track only; it is not a
    global identity (``docs/knowledge-base/17_data-schema.md``).
    """

    track_id: int = Field(ge=0)
    object_class: ObjectClass = ObjectClass.UNKNOWN
    status: TrackStatus = TrackStatus.TENTATIVE
    position: Vector3
    velocity: Vector3 = Field(default_factory=Vector3)
    acceleration: Vector3 = Field(default_factory=Vector3)
    heading_rad: float = 0.0
    confidence: float = Field(ge=0.0, le=1.0)
    uncertainty: float = Field(
        default=0.0,
        ge=0.0,
        le=1.0,
        description="Normalised state uncertainty; distinct from risk.",
    )
    age_frames: int = Field(default=1, ge=0, description="Frames since track creation.")
    missed_frames: int = Field(
        default=0, ge=0, description="Consecutive frames without an associated detection."
    )
    last_seen: datetime | None = Field(
        default=None,
        description="Timestamp of the last associated detection; null while never seen.",
    )
    coordinate_frame: CoordinateFrame = CoordinateFrame.EGO
    source: DataSource = DataSource.LIVE_SENSOR

    @field_validator("heading_rad")
    @classmethod
    def _require_finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("heading_rad must be finite")
        return value

    @field_validator("last_seen")
    @classmethod
    def _require_last_seen_timezone(cls, value: datetime | None) -> datetime | None:
        if value is not None and value.tzinfo is None:
            raise ValueError("last_seen must be timezone-aware (use UTC)")
        return value.astimezone(UTC) if value is not None else None

    @property
    def speed_mps(self) -> float:
        """Scalar speed in metres per second."""
        return self.velocity.magnitude
