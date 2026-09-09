"""Point-cloud data contracts.

Points are held as a NumPy array of shape ``(N, C)`` where the first three
columns are ``x``, ``y``, ``z`` in metres and an optional fourth column is
intensity. The column layout is described by :attr:`PointCloudFrame.fields`.
"""

from __future__ import annotations

from typing import Any, Self

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


class BasePointCloudFrame(TimestampedModel):
    """Shared structure and metadata for a timestamped LiDAR frame.

    Validation here is *structural* only: array type, dimensionality, column
    count, floating dtype and column layout. Whether non-finite values are
    permitted is decided by the subclass:

    * :class:`RawPointCloudFrame` allows them - real sensors emit NaN for a
      non-return.
    * :class:`PointCloudFrame` forbids them, so holding one is a guarantee
      that every coordinate is finite.

    Geometric processing beyond Phase 2A preprocessing (voxelisation, ground
    segmentation, clustering) is the responsibility of a
    :class:`adaptx.perception.interfaces.LiDARProcessor` implementation and is
    not implemented.
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
    def _validate_points_structure(cls, value: np.ndarray) -> np.ndarray:
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
        return value

    @model_validator(mode="after")
    def _validate_field_layout(self) -> BasePointCloudFrame:
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
        """Axis-aligned bounds, or ``None`` when there is nothing to bound.

        Bounds are computed over points whose x, y and z are all finite. A
        :class:`PointCloudFrame` is finite by construction, so this is exactly
        its extent; for a :class:`RawPointCloudFrame` the non-finite points are
        excluded, because an infinity would otherwise propagate into a bound
        and serialise as a null, which reads as missing data rather than as the
        infinity it actually was.

        Returns ``None`` for an empty frame or one with no finite point.
        """
        if self.point_count == 0:
            return None
        xyz = self.points[:, :3]
        finite = np.isfinite(xyz).all(axis=1)
        if not finite.any():
            return None
        if not finite.all():
            xyz = xyz[finite]
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
    ) -> Self:
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


class RawPointCloudFrame(BasePointCloudFrame):
    """An unprocessed frame as it arrives from a sensor or simulator.

    Structurally valid but **may contain NaN or infinite coordinates**: a
    scanner reports a non-return that way, and dropping those points is the
    job of the preprocessing pipeline
    (:class:`adaptx.perception.preprocessing.PointCloudPreprocessor`), which
    counts them rather than hiding them.
    """


class PointCloudFrame(BasePointCloudFrame):
    """A structurally valid frame whose coordinates are all finite.

    This is the contract every downstream module consumes. Constructing one
    with NaN or infinite values fails, so a `PointCloudFrame` in hand is proof
    the data has been validated.
    """

    @field_validator("points")
    @classmethod
    def _require_finite_points(cls, value: np.ndarray) -> np.ndarray:
        if value.size and not np.isfinite(value).all():
            raise ValueError("points must not contain NaN or infinite values")
        return value


class PointCloudSummary(TimestampedModel):
    """Serialisable metadata describing a frame, without the point payload."""

    frame_id: int
    sensor_id: str
    point_count: int
    fields: tuple[str, ...]
    coordinate_frame: CoordinateFrame
    source: DataSource
    bounds: PointCloudBounds | None = None
