"""Point-cloud data contracts.

Points are held as a NumPy array of shape ``(N, C)`` where the first three
columns are ``x``, ``y``, ``z`` in metres and an optional fourth column is
intensity. The column layout is described by :attr:`PointCloudFrame.fields`.
"""

from __future__ import annotations

from typing import Any

import numpy as np
from pydantic import ConfigDict, Field, field_validator, model_validator

from adaptx.models.common import (
    AdaptXModel,
    CoordinateFrame,
    DataSource,
    TimestampedModel,
)

#: Column layouts accepted by :class:`PointCloudFrame`.
XYZ_FIELDS: tuple[str, ...] = ("x", "y", "z")
XYZI_FIELDS: tuple[str, ...] = ("x", "y", "z", "intensity")


class PointCloudBounds(AdaptXModel):
    """Axis-aligned bounds of a point cloud, in metres."""

    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_z: float
    max_z: float

    @property
    def extent(self) -> tuple[float, float, float]:
        """Size along each axis."""
        return (
            self.max_x - self.min_x,
            self.max_y - self.min_y,
            self.max_z - self.min_z,
        )


class PointCloudFrame(TimestampedModel):
    """A single timestamped LiDAR frame.

    Validation performed here is structural only: shape, dtype, finiteness and
    column layout. Geometric processing (ground segmentation, downsampling,
    clustering) is the responsibility of a
    :class:`adaptx.perception.interfaces.LiDARProcessor` implementation and is
    not implemented in Phase 1.
    """

    model_config = ConfigDict(
        extra="forbid", validate_assignment=True, arbitrary_types_allowed=True
    )

    frame_id: int = Field(ge=0, description="Monotonic frame counter from the source.")
    sensor_id: str = Field(min_length=1, description="Identifier of the emitting sensor.")
    points: np.ndarray = Field(description="Array of shape (N, 3) or (N, 4), float32/float64.")
    fields: tuple[str, ...] = Field(
        default=XYZ_FIELDS, description="Column names of `points`, in order."
    )
    coordinate_frame: CoordinateFrame = CoordinateFrame.LIDAR
    source: DataSource = DataSource.LIVE_SENSOR

    @field_validator("points")
    @classmethod
    def _validate_points(cls, value: np.ndarray) -> np.ndarray:
        if not isinstance(value, np.ndarray):
            raise ValueError("points must be a numpy.ndarray")
        if value.ndim != 2:
            raise ValueError(f"points must be 2-dimensional, got ndim={value.ndim}")
        if value.shape[1] not in (3, 4):
            raise ValueError(
                f"points must have 3 (x,y,z) or 4 (x,y,z,intensity) columns, got {value.shape[1]}"
            )
        if not np.issubdtype(value.dtype, np.floating):
            raise ValueError(f"points must have a floating dtype, got {value.dtype}")
        if value.size and not np.isfinite(value).all():
            raise ValueError("points must not contain NaN or infinite values")
        return value

    @model_validator(mode="after")
    def _validate_field_layout(self) -> PointCloudFrame:
        expected = XYZ_FIELDS if self.points.shape[1] == 3 else XYZI_FIELDS
        if self.fields != expected:
            raise ValueError(
                f"fields {self.fields} do not match a {self.points.shape[1]}-column "
                f"point array (expected {expected})"
            )
        return self

    @property
    def point_count(self) -> int:
        """Number of points in the frame."""
        return int(self.points.shape[0])

    @property
    def has_intensity(self) -> bool:
        """True when the frame carries an intensity column."""
        return self.points.shape[1] == 4

    def bounds(self) -> PointCloudBounds | None:
        """Axis-aligned bounds, or ``None`` for an empty frame."""
        if self.point_count == 0:
            return None
        xyz = self.points[:, :3]
        minimum = xyz.min(axis=0)
        maximum = xyz.max(axis=0)
        return PointCloudBounds(
            min_x=float(minimum[0]),
            max_x=float(maximum[0]),
            min_y=float(minimum[1]),
            max_y=float(maximum[1]),
            min_z=float(minimum[2]),
            max_z=float(maximum[2]),
        )

    def summary(self) -> PointCloudSummary:
        """Metadata-only view of this frame, safe to serialise to JSON."""
        return PointCloudSummary(
            schema_version=self.schema_version,
            timestamp=self.timestamp,
            frame_id=self.frame_id,
            sensor_id=self.sensor_id,
            point_count=self.point_count,
            fields=self.fields,
            coordinate_frame=self.coordinate_frame,
            source=self.source,
            bounds=self.bounds(),
        )

    @classmethod
    def from_sequence(
        cls,
        points: list[list[float]] | list[tuple[float, ...]],
        **kwargs: Any,
    ) -> PointCloudFrame:
        """Build a frame from nested sequences (used by the JSON API layer).

        Raises ``ValueError`` for ragged or non-numeric input, which the API
        layer converts into a 422 response.
        """
        try:
            array = np.asarray(points, dtype=np.float64)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"points are not a numeric 2D array: {exc}") from exc
        if array.ndim == 1 and array.size == 0:
            array = array.reshape(0, 3)
        columns = array.shape[1] if array.ndim == 2 else 0
        kwargs.setdefault("fields", XYZI_FIELDS if columns == 4 else XYZ_FIELDS)
        return cls(points=array, **kwargs)


class PointCloudSummary(TimestampedModel):
    """Serialisable metadata describing a frame, without the point payload."""

    frame_id: int
    sensor_id: str
    point_count: int
    fields: tuple[str, ...]
    coordinate_frame: CoordinateFrame
    source: DataSource
    bounds: PointCloudBounds | None = None
