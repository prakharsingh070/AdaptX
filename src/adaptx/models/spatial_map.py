"""2.5D spatial map contracts (Phase 6).

A 2.5D map indexes space by ``x``/``y`` and summarises ``z`` as height
statistics per cell, rather than storing a volumetric grid
(``docs/knowledge-base/05_2.5d-mapping.md``, ADR-028).

Dense arrays, not cell objects
------------------------------
The grid is held as NumPy arrays rather than a list of per-cell models. A
0.25 m map over a 120 m square is 230,400 cells; one validated Pydantic object
each would cost more than the mapping itself and would be unusable on every
frame. Storing an array inside a contract follows the precedent already set by
:class:`~adaptx.models.point_cloud.BasePointCloudFrame`, which holds its points
the same way.

:meth:`SpatialMap.to_adaptive_map` projects **occupied cells only** into the
pre-existing :class:`~adaptx.models.map.AdaptiveMap` contract for consumers
that want individual cells.

Unobserved is not zero
----------------------
A cell with no points has no measured height. Its ``min``/``max``/``mean``
entries are ``NaN``, never ``0.0`` - zero is a real height in this coordinate
frame (ADR-031), and the same reasoning as ADR-023 applies: an unmeasured value
must not be indistinguishable from a measured one.

Frame-local
-----------
A ``SpatialMap`` describes exactly one frame. Nothing accumulates between
frames; this is not a persistent world map and not SLAM (ADR-030).
"""

from __future__ import annotations

import math
from enum import StrEnum

import numpy as np
from pydantic import ConfigDict, Field, field_validator, model_validator

from adaptx.models.common import (
    AdaptXModel,
    CoordinateFrame,
    DataSource,
    TimestampedModel,
    Vector3,
)
from adaptx.models.map import AdaptiveMap, AdaptiveMapCell, OccupancyState, ResolutionLevel
from adaptx.models.resolution import ResolutionDecision

#: Cap on cells returned by :meth:`SpatialMap.to_adaptive_map` unless raised by
#: the caller. A full 0.25 m map would otherwise serialise hundreds of
#: thousands of objects into one response.
DEFAULT_MAX_PROJECTED_CELLS = 20_000


class MapBounds(AdaptXModel):
    """Rectangular extent of a map in the XY plane, in metres.

    Half-open on the upper edge: a point at exactly ``max_x`` or ``max_y`` lies
    **outside** the map (ADR-028). Without that rule such a point would index
    one cell past the last column, and clamping it inward would place a
    measurement in a cell it does not belong to.
    """

    min_x: float
    max_x: float
    min_y: float
    max_y: float

    @field_validator("min_x", "max_x", "min_y", "max_y")
    @classmethod
    def _require_finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("map bounds must be finite")
        return value

    @model_validator(mode="after")
    def _check_ordering(self) -> MapBounds:
        if self.min_x >= self.max_x:
            raise ValueError(f"min_x ({self.min_x}) must be < max_x ({self.max_x})")
        if self.min_y >= self.max_y:
            raise ValueError(f"min_y ({self.min_y}) must be < max_y ({self.max_y})")
        return self

    @property
    def size_x_m(self) -> float:
        """Extent along x in metres."""
        return self.max_x - self.min_x

    @property
    def size_y_m(self) -> float:
        """Extent along y in metres."""
        return self.max_y - self.min_y

    def contains(self, x: float, y: float) -> bool:
        """Whether ``(x, y)`` falls inside the map, upper edges exclusive."""
        return self.min_x <= x < self.max_x and self.min_y <= y < self.max_y


class MappingConfiguration(AdaptXModel):
    """Effective mapping configuration that produced a result.

    Carried on every map so a record is self-describing - the same rule
    processing, detection, tracking and prediction already follow. A plain
    model rather than a reference to
    :class:`~adaptx.config.settings.MapSettings`, keeping :mod:`adaptx.models`
    free of any dependency on the configuration layer.
    """

    resolution_m: float
    bounds: MapBounds
    max_cells: int


class MapAccounting(AdaptXModel):
    """Where every input point went, and what the grid ended up holding.

    ``input_point_count == mapped_point_count + out_of_bounds_point_count``
    is enforced. A point is never silently dropped: if it did not land in a
    cell, it is counted as out of bounds and can be explained.
    """

    input_point_count: int = Field(ge=0)
    mapped_point_count: int = Field(ge=0)
    out_of_bounds_point_count: int = Field(ge=0)
    occupied_cell_count: int = Field(ge=0)
    total_cell_count: int = Field(ge=1)

    @model_validator(mode="after")
    def _check_accounting(self) -> MapAccounting:
        accounted = self.mapped_point_count + self.out_of_bounds_point_count
        if self.input_point_count != accounted:
            raise ValueError(
                f"input_point_count ({self.input_point_count}) must equal mapped "
                f"({self.mapped_point_count}) + out_of_bounds "
                f"({self.out_of_bounds_point_count})"
            )
        if self.occupied_cell_count > self.total_cell_count:
            raise ValueError(
                f"occupied_cell_count ({self.occupied_cell_count}) cannot exceed "
                f"total_cell_count ({self.total_cell_count})"
            )
        return self

    @property
    def occupancy_ratio(self) -> float:
        """Fraction of cells holding at least one point, in ``[0, 1]``."""
        return self.occupied_cell_count / self.total_cell_count

    @property
    def out_of_bounds_ratio(self) -> float | None:
        """Fraction of input points that fell outside the map.

        ``None`` for an empty frame rather than a fabricated 0.0: no points
        means the question has no answer.
        """
        if self.input_point_count == 0:
            return None
        return self.out_of_bounds_point_count / self.input_point_count


class SpatialMapSummary(TimestampedModel):
    """Compact description of a map, without the grid itself.

    This is what the API, telemetry and status endpoints carry. The arrays stay
    where they are; sending a quarter of a million cells down a status channel
    would be frame data in the wrong place.
    """

    frame_id: int = Field(ge=0)
    sensor_id: str = Field(min_length=1)
    mapper: str = Field(min_length=1, description="Identifier of the mapper that ran.")
    is_adaptive: bool = Field(
        description="False for the Phase 6 fixed-resolution baseline (ADR-003)."
    )
    resolution: ResolutionDecision
    bounds: MapBounds
    width: int = Field(ge=1, description="Cells along x.")
    height: int = Field(ge=1, description="Cells along y.")
    accounting: MapAccounting
    duration_ms: float = Field(ge=0.0, description="Whole mapping pass, measured.")
    configuration: MappingConfiguration

    @property
    def occupancy_ratio(self) -> float:
        """Fraction of cells holding at least one point."""
        return self.accounting.occupancy_ratio


class SpatialMapCellField(StrEnum):
    """Names of the per-cell arrays a :class:`SpatialMap` holds."""

    POINT_COUNT = "point_count"
    MIN_HEIGHT = "min_height_m"
    MAX_HEIGHT = "max_height_m"
    MEAN_HEIGHT = "mean_height_m"


class SpatialMap(TimestampedModel):
    """A frame-local 2.5D grid: occupancy and height statistics per cell.

    Arrays are indexed ``[row, column]`` where ``row`` steps along **y** and
    ``column`` along **x**, so ``shape == (height, width)``. That matches the
    NumPy convention for images and keeps ``width``/``height`` meaning cells
    along x and y respectively.

    Height arrays hold ``NaN`` wherever ``point_count == 0``: unobserved space
    has no measured height (ADR-031).
    """

    model_config = ConfigDict(
        extra="forbid", validate_assignment=True, arbitrary_types_allowed=True
    )

    frame_id: int = Field(ge=0)
    sensor_id: str = Field(min_length=1)
    mapper: str = Field(min_length=1)
    is_adaptive: bool = Field(
        default=False, description="False for the Phase 6 fixed-resolution baseline."
    )

    resolution: ResolutionDecision
    bounds: MapBounds
    width: int = Field(ge=1, description="Cells along x.")
    height: int = Field(ge=1, description="Cells along y.")

    point_count: np.ndarray = Field(description="(height, width) int64 points per cell.")
    min_height_m: np.ndarray = Field(description="(height, width) float64, NaN where empty.")
    max_height_m: np.ndarray = Field(description="(height, width) float64, NaN where empty.")
    mean_height_m: np.ndarray = Field(description="(height, width) float64, NaN where empty.")

    accounting: MapAccounting
    duration_ms: float = Field(ge=0.0)
    configuration: MappingConfiguration
    coordinate_frame: CoordinateFrame = CoordinateFrame.EGO
    source: DataSource = DataSource.LIVE_SENSOR

    @field_validator("point_count")
    @classmethod
    def _validate_counts(cls, value: np.ndarray) -> np.ndarray:
        if not isinstance(value, np.ndarray):
            raise ValueError("point_count must be a numpy.ndarray")
        if value.ndim != 2:
            raise ValueError(f"point_count must be 2-dimensional, got ndim={value.ndim}")
        if not np.issubdtype(value.dtype, np.integer):
            raise ValueError(f"point_count must have an integer dtype, got {value.dtype}")
        if value.size and int(value.min()) < 0:
            raise ValueError("point_count cannot be negative")
        return value

    @field_validator("min_height_m", "max_height_m", "mean_height_m")
    @classmethod
    def _validate_heights(cls, value: np.ndarray) -> np.ndarray:
        if not isinstance(value, np.ndarray):
            raise ValueError("height arrays must be numpy.ndarray")
        if value.ndim != 2:
            raise ValueError(f"height arrays must be 2-dimensional, got ndim={value.ndim}")
        if not np.issubdtype(value.dtype, np.floating):
            raise ValueError(f"height arrays must have a floating dtype, got {value.dtype}")
        if value.size and np.isinf(value).any():
            raise ValueError("height arrays must not contain infinities; use NaN for unobserved")
        return value

    @model_validator(mode="after")
    def _check_shapes(self) -> SpatialMap:
        expected = (self.height, self.width)
        for name in (
            "point_count",
            "min_height_m",
            "max_height_m",
            "mean_height_m",
        ):
            array: np.ndarray = getattr(self, name)
            if array.shape != expected:
                raise ValueError(f"{name} has shape {array.shape}, expected {expected}")
        if self.accounting.total_cell_count != self.width * self.height:
            raise ValueError(
                f"total_cell_count ({self.accounting.total_cell_count}) must equal "
                f"width * height ({self.width * self.height})"
            )
        return self

    # -- derived views -----------------------------------------------------
    @property
    def occupancy(self) -> np.ndarray:
        """Boolean ``(height, width)`` grid: a cell is occupied iff it holds a point.

        Binary and deterministic (ADR-031). Not a probability, not Bayesian and
        not fused over time; those would each need evidence Phase 6 does not
        collect.
        """
        return self.point_count > 0

    @property
    def occupied_cell_count(self) -> int:
        """Cells holding at least one point."""
        return self.accounting.occupied_cell_count

    @property
    def occupancy_ratio(self) -> float:
        """Fraction of cells holding at least one point."""
        return self.accounting.occupancy_ratio

    @property
    def resolution_m(self) -> float:
        """Cell edge length in metres."""
        return self.resolution.resolution_m

    def cell_centre(self, row: int, column: int) -> tuple[float, float]:
        """Centre coordinates in metres of the cell at ``[row, column]``."""
        size = self.resolution.resolution_m
        return (
            self.bounds.min_x + (column + 0.5) * size,
            self.bounds.min_y + (row + 0.5) * size,
        )

    def summary(self) -> SpatialMapSummary:
        """Compact description, without the grid."""
        return SpatialMapSummary(
            timestamp=self.timestamp,
            frame_id=self.frame_id,
            sensor_id=self.sensor_id,
            mapper=self.mapper,
            is_adaptive=self.is_adaptive,
            resolution=self.resolution,
            bounds=self.bounds,
            width=self.width,
            height=self.height,
            accounting=self.accounting,
            duration_ms=self.duration_ms,
            configuration=self.configuration,
        )

    def to_adaptive_map(
        self, *, max_cells: int = DEFAULT_MAX_PROJECTED_CELLS
    ) -> tuple[AdaptiveMap, bool]:
        """Project **occupied cells only** into the ``AdaptiveMap`` contract.

        Empty cells are omitted rather than emitted as unknown: a 0.25 m map is
        overwhelmingly empty, and serialising a quarter of a million cells to
        say "nothing here" would swamp any consumer.

        ``AdaptiveMapCell.occupancy`` is populated 1.0 for every projected
        cell, and ``occupancy_state`` is ``OCCUPIED``. The float field predates
        Phase 6 and is documented as a probability; the baseline produces only
        binary values, so it is never given an intermediate one (ADR-031).
        ``risk_score`` and ``uncertainty`` are left at their defaults - nothing
        computes them yet, and filling them would be an invention.

        Cells are ordered by ``(row, column)`` so the projection is
        deterministic.

        Args:
            max_cells: Upper bound on projected cells.

        Returns:
            ``(map, truncated)`` - the projection, and whether it was cut short.
        """
        rows, columns = np.nonzero(self.point_count)
        truncated = bool(rows.size > max_cells)
        if truncated:
            rows, columns = rows[:max_cells], columns[:max_cells]

        size = self.resolution.resolution_m
        counts = self.point_count[rows, columns]
        minimums = self.min_height_m[rows, columns]
        maximums = self.max_height_m[rows, columns]
        means = self.mean_height_m[rows, columns]

        cells = [
            AdaptiveMapCell(
                timestamp=self.timestamp,
                x=self.bounds.min_x + (int(column) + 0.5) * size,
                y=self.bounds.min_y + (int(row) + 0.5) * size,
                resolution_m=size,
                resolution_level=_nearest_level(size),
                occupancy=1.0,
                occupancy_state=OccupancyState.OCCUPIED,
                height_m=_finite_or_none(means[index]),
                height_min_m=_finite_or_none(minimums[index]),
                height_max_m=_finite_or_none(maximums[index]),
                point_count=int(counts[index]),
                coordinate_frame=self.coordinate_frame,
                source=self.source,
            )
            for index, (row, column) in enumerate(zip(rows, columns, strict=True))
        ]

        return (
            AdaptiveMap(
                timestamp=self.timestamp,
                frame_id=self.frame_id,
                is_adaptive=self.is_adaptive,
                origin=Vector3(x=self.bounds.min_x, y=self.bounds.min_y, z=0.0),
                range_m=max(self.bounds.size_x_m, self.bounds.size_y_m),
                cells=cells,
                coordinate_frame=self.coordinate_frame,
                source=self.source,
            ),
            truncated,
        )


def _finite_or_none(value: float) -> float | None:
    """Convert a NaN height to ``None`` for contracts that express it that way."""
    numeric = float(value)
    return None if math.isnan(numeric) else numeric


#: Cell size in metres bound to each resolution level by Phase 1 defaults. Used
#: only to label a projected cell; the mapper itself never consults it.
_DEFAULT_LEVEL_SIZES: dict[ResolutionLevel, float] = {
    ResolutionLevel.LOW: 1.0,
    ResolutionLevel.MEDIUM: 0.5,
    ResolutionLevel.HIGH: 0.2,
    ResolutionLevel.CRITICAL: 0.1,
}


def _nearest_level(resolution_m: float) -> ResolutionLevel:
    """Label a cell size with the closest configured resolution level.

    A label for display only. Phase 6 applies one uniform size, so this
    describes the size that was used - it is not a resolution *decision*, which
    belongs to a controller that does not exist (ADR-029).
    """
    return min(
        _DEFAULT_LEVEL_SIZES,
        key=lambda level: (abs(_DEFAULT_LEVEL_SIZES[level] - resolution_m), level.value),
    )
