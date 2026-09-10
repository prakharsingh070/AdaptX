"""Tracked-object contract (multi-frame perception output)."""

from __future__ import annotations

import math
from datetime import UTC, datetime
from enum import StrEnum

from pydantic import Field, field_validator

from adaptx.models.common import (
    BoundingBox3D,
    CoordinateFrame,
    DataSource,
    ObjectClass,
    TimestampedModel,
    Vector3,
)


class TrackStatus(StrEnum):
    """Lifecycle state of a track.

    See ``docs/knowledge-base/08_tracking.md``. The four states cover the whole
    lifecycle, so no new ones were added in Phase 4:

    ``TENTATIVE``
        Newly created, not yet seen often enough to trust.
    ``CONFIRMED``
        Associated with a detection in the current frame.
    ``COASTING``
        Alive but unmatched this frame - occluded, missed by the detector, or
        filtered out. Still aged and still reported.
    ``LOST``
        Terminated. Missed too many consecutive frames; the id is retired and
        never reissued.
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
    previous_position: Vector3 | None = Field(
        default=None, description="Position at the previous update; null on the first."
    )
    velocity: Vector3 | None = Field(
        default=None,
        description=(
            "Smoothed velocity in m/s, or null when it is not yet observable. "
            "A single observation cannot show motion, so a new track reports "
            "null rather than a zero vector - zero would claim a measured "
            "standstill (ADR-023)."
        ),
    )
    observed_velocity: Vector3 | None = Field(
        default=None,
        description="Raw frame-to-frame velocity before smoothing, or null if unknown.",
    )
    acceleration: Vector3 | None = Field(
        default=None,
        description=(
            "Change in velocity per second, or null. Requires two consecutive "
            "velocity observations, so it stays null until then."
        ),
    )
    heading_rad: float | None = Field(
        default=None,
        description=(
            "Direction of travel, counter-clockwise from +x. Null below the "
            "configured speed floor, where a heading would describe noise "
            "rather than travel."
        ),
    )
    predicted_position: Vector3 | None = Field(
        default=None,
        description=(
            "Where the tracker expected this track before matching, used for "
            "association gating. Tracker state, NOT an observation and NOT a "
            "trajectory prediction - that is a later phase."
        ),
    )
    bounding_box: BoundingBox3D | None = Field(
        default=None, description="Geometry from the most recent associated detection."
    )
    point_count: int = Field(
        default=0, ge=0, description="Points in the most recent associated detection."
    )
    hits: int = Field(
        default=0, ge=0, description="Detections associated with this track in total."
    )
    first_seen: datetime | None = Field(
        default=None, description="Timestamp of the detection that created this track."
    )
    confidence: float = Field(
        ge=0.0,
        le=1.0,
        description=(
            "Geometric fit score of the most recent associated detection. As in "
            "Phase 3 this is not a probability that the class is correct."
        ),
    )
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
    def _require_finite(cls, value: float | None) -> float | None:
        if value is not None and not math.isfinite(value):
            raise ValueError("heading_rad must be finite")
        return value

    @field_validator("last_seen")
    @classmethod
    def _require_last_seen_timezone(cls, value: datetime | None) -> datetime | None:
        if value is not None and value.tzinfo is None:
            raise ValueError("last_seen must be timezone-aware (use UTC)")
        return value.astimezone(UTC) if value is not None else None

    @property
    def speed_mps(self) -> float | None:
        """Scalar speed in m/s, or ``None`` while velocity is unobserved.

        Null rather than 0.0: a track seen once has no measured speed, and
        reporting zero would be indistinguishable from a measured standstill.
        """
        return None if self.velocity is None else self.velocity.magnitude

    @property
    def is_moving(self) -> bool | None:
        """Whether the track has measurable motion, or ``None`` if unknown."""
        speed = self.speed_mps
        return None if speed is None else speed > 0.0
