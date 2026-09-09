"""Detected-object contract (single-frame perception output)."""

from __future__ import annotations

from pydantic import Field

from adaptx.models.common import (
    BoundingBox3D,
    CoordinateFrame,
    DataSource,
    ObjectClass,
    TimestampedModel,
    Vector3,
)


class DetectedObject(TimestampedModel):
    """An object detected in a single point-cloud frame.

    A detection has no temporal identity: ``object_id`` is unique within its
    frame only. Persistent identity is produced by the tracking module as
    :class:`adaptx.models.tracking.TrackedObject`.
    """

    object_id: int = Field(ge=0, description="Identifier unique within `frame_id`.")
    frame_id: int = Field(ge=0)
    object_class: ObjectClass = ObjectClass.UNKNOWN
    position: Vector3 = Field(description="Centroid of the detection.")
    velocity: Vector3 | None = Field(
        default=None,
        description="Instantaneous velocity if the detector estimates one, else null.",
    )
    bounding_box: BoundingBox3D
    confidence: float = Field(ge=0.0, le=1.0, description="Detector confidence in [0, 1].")
    point_count: int = Field(default=0, ge=0, description="Points supporting the detection.")
    coordinate_frame: CoordinateFrame = CoordinateFrame.EGO
    source: DataSource = DataSource.LIVE_SENSOR
