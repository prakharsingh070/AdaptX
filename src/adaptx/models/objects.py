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
    bounding_box: BoundingBox3D = Field(
        description="Axis-aligned in the current detector; `yaw_rad` is 0."
    )
    confidence: float = Field(
        ge=0.0,
        le=1.0,
        description=(
            "How well the cluster geometry fits the assigned class, in [0, 1]. "
            "This is a geometric fit score, NOT a probability that the "
            "classification is correct: no labelled data exists to calibrate one. "
            "0.0 when the class is UNKNOWN, because nothing was matched."
        ),
    )
    point_count: int = Field(default=0, ge=0, description="Points supporting the detection.")
    distance_m: float = Field(
        default=0.0,
        ge=0.0,
        description="Euclidean distance from the sensor origin to the centroid.",
    )
    classifier: str = Field(
        default="",
        description=(
            "Identifier of the classifier that assigned `object_class`, so a "
            "baseline heuristic is never mistaken for a trained model."
        ),
    )
    is_baseline_classification: bool = Field(
        default=True,
        description="True while classification comes from a geometric heuristic.",
    )
    coordinate_frame: CoordinateFrame = CoordinateFrame.EGO
    source: DataSource = DataSource.LIVE_SENSOR

    @property
    def extents(self) -> tuple[float, float, float]:
        """Axis-aligned extents (x, y, z) in metres."""
        dimensions = self.bounding_box.dimensions
        return (dimensions.length, dimensions.width, dimensions.height)
