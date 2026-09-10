"""Tiled adaptive 2.5D map contracts (Phase 8).

How one map holds several resolutions
-------------------------------------
A single dense NumPy grid has exactly one cell size, so an adaptive map cannot
be one. The map extent is instead partitioned into fixed-size square **tiles**,
and each tile owns its own dense sub-grid at its own cell size (ADR-037)::

    map bounds
      +-- tile (10 m)  level LOW       -> 10 x 10 cells at 1.0 m
      +-- tile (10 m)  level CRITICAL  -> 100 x 100 cells at 0.1 m
      +-- ...

Tiles partition the extent exactly: they are anchored at the map lower corner,
half-open on their upper edges, and the last row and column are clipped to the
map bounds. Every point inside the map therefore belongs to exactly one tile
and exactly one cell of it - no gap, no double coverage.

Relationship to Phase 6
-----------------------
:class:`~adaptx.models.spatial_map.SpatialMap` is unchanged and remains the
fixed-resolution baseline (ADR-003). A tile here holds the same per-cell
quantities a ``SpatialMap`` cell holds - point count, min/max/mean height, NaN
where unobserved (ADR-031) - and :class:`MapAccounting` and
:class:`MapBounds` are reused rather than re-declared.

Unobserved is not free
----------------------
A cell with no returns is *unobserved*: it may be empty, or it may be occluded,
and the map cannot tell the difference. Adaptive tiling changes none of that;
it only changes how finely the question is asked.
"""

from __future__ import annotations

import math

import numpy as np
from pydantic import ConfigDict, Field, field_validator, model_validator

from adaptx.models.adaptive_resolution import LEVEL_ORDER, ResolutionPlan
from adaptx.models.common import (
    AdaptXModel,
    CoordinateFrame,
    DataSource,
    TimestampedModel,
    Vector3,
)
from adaptx.models.map import AdaptiveMap, AdaptiveMapCell, OccupancyState, ResolutionLevel
from adaptx.models.spatial_map import (
    DEFAULT_MAX_PROJECTED_CELLS,
    MapAccounting,
    MapBounds,
)

#: Bytes each cell occupies across the four dense arrays: one int64 and three
#: float64. Arithmetic, not a measurement.
BYTES_PER_CELL = 8 * 4


class MapTile(TimestampedModel):
    """One tile of the adaptive map: a dense sub-grid at a single cell size.

    Arrays are indexed ``[row, column]`` where ``row`` steps along **y** and
    ``column`` along **x**, matching :class:`~adaptx.models.spatial_map.SpatialMap`.

    Height arrays hold ``NaN`` wherever ``point_count == 0``: unobserved space
    has no measured height (ADR-031).
    """

    model_config = ConfigDict(
        extra="forbid", validate_assignment=True, arbitrary_types_allowed=True
    )

    tile_index: int = Field(ge=0, description="Row-major index within the tile grid.")
    tile_row: int = Field(ge=0)
    tile_column: int = Field(ge=0)
    bounds: MapBounds = Field(description="Extent of this tile, clipped to the map bounds.")

    level: ResolutionLevel
    resolution_m: float = Field(gt=0.0, description="Cell edge length inside this tile.")
    width: int = Field(ge=1, description="Cells along x.")
    height: int = Field(ge=1, description="Cells along y.")

    point_count: np.ndarray = Field(description="(height, width) int64 points per cell.")
    min_height_m: np.ndarray = Field(description="(height, width) float64, NaN where empty.")
    max_height_m: np.ndarray = Field(description="(height, width) float64, NaN where empty.")
    mean_height_m: np.ndarray = Field(description="(height, width) float64, NaN where empty.")

    mapped_point_count: int = Field(ge=0, description="Points that landed in this tile.")
    occupied_cell_count: int = Field(ge=0)

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
    def _check_shapes(self) -> MapTile:
        expected = (self.height, self.width)
        for name in ("point_count", "min_height_m", "max_height_m", "mean_height_m"):
            array: np.ndarray = getattr(self, name)
            if array.shape != expected:
                raise ValueError(f"{name} has shape {array.shape}, expected {expected}")
        if self.occupied_cell_count > self.width * self.height:
            raise ValueError(
                f"occupied_cell_count ({self.occupied_cell_count}) cannot exceed the "
                f"{self.width * self.height} cells of the tile"
            )
        return self

    @property
    def occupancy(self) -> np.ndarray:
        """Boolean ``(height, width)`` grid: a cell is occupied iff it holds a point.

        Binary and deterministic (ADR-031), exactly as in Phase 6. Not a
        probability, not fused over time, and **not** a statement that an empty
        cell is free space.
        """
        return self.point_count > 0

    @property
    def cell_count(self) -> int:
        """Cells this tile allocates."""
        return self.width * self.height

    def cell_centre(self, row: int, column: int) -> tuple[float, float]:
        """Centre coordinates in metres of the cell at ``[row, column]``."""
        return (
            self.bounds.min_x + (column + 0.5) * self.resolution_m,
            self.bounds.min_y + (row + 0.5) * self.resolution_m,
        )


class AdaptiveSpatialMapSummary(TimestampedModel):
    """Compact description of an adaptive map, without any grid.

    This is what the API, telemetry and status endpoints carry. A tiled map at
    mixed resolutions can hold hundreds of thousands of cells; sending them
    down a status channel would be frame data in the wrong place.
    """

    frame_id: int = Field(ge=0)
    sensor_id: str = Field(min_length=1)
    mapper: str = Field(min_length=1)
    controller: str = Field(min_length=1)
    is_adaptive: bool = Field(description="True: resolution varies by region (ADR-003).")

    bounds: MapBounds
    tile_size_m: float = Field(gt=0.0)
    tile_count: int = Field(ge=1)

    accounting: MapAccounting
    tiles_by_level: dict[str, int] = Field(description="Tiles at each resolution level.")
    cells_by_level: dict[str, int] = Field(description="Cells allocated at each level.")
    finest_resolution_m: float = Field(gt=0.0)
    coarsest_resolution_m: float = Field(gt=0.0)
    area_weighted_resolution_m: float = Field(
        gt=0.0,
        description=(
            "Mean cell size weighted by the ground area each tile covers. "
            "Arithmetic over the tiling, not a measurement."
        ),
    )
    changed_tile_count: int = Field(ge=0, description="Tiles whose level changed this frame.")

    grid_bytes: int = Field(
        ge=0,
        description=(
            "Exact size of the dense arrays: one int64 and three float64 per "
            "cell. Arithmetic, not a measurement."
        ),
    )
    controller_duration_ms: float = Field(ge=0.0, description="Resolution planning, measured.")
    mapping_duration_ms: float = Field(ge=0.0, description="Grid construction, measured.")

    @property
    def total_duration_ms(self) -> float:
        """Planning plus mapping."""
        return self.controller_duration_ms + self.mapping_duration_ms

    @property
    def occupancy_ratio(self) -> float:
        """Fraction of cells holding at least one point."""
        return self.accounting.occupancy_ratio


class AdaptiveSpatialMap(TimestampedModel):
    """A frame-local 2.5D map whose resolution varies by region.

    Frame-local exactly as Phase 6 is (ADR-030): a call builds a whole map from
    one frame and no occupancy accumulates. The **only** thing that persists
    between frames is the resolution controller stabilisation state, which
    lives in the controller and never in a map.
    """

    model_config = ConfigDict(
        extra="forbid", validate_assignment=True, arbitrary_types_allowed=True
    )

    frame_id: int = Field(ge=0)
    sensor_id: str = Field(min_length=1)
    mapper: str = Field(min_length=1)
    is_adaptive: bool = Field(default=True)

    bounds: MapBounds
    tile_size_m: float = Field(gt=0.0)
    tiles: list[MapTile] = Field(min_length=1)
    plan: ResolutionPlan = Field(description="The decisions this map applied.")

    accounting: MapAccounting
    duration_ms: float = Field(ge=0.0, description="Grid construction only, measured.")
    coordinate_frame: CoordinateFrame = CoordinateFrame.EGO
    source: DataSource = DataSource.LIVE_SENSOR

    @model_validator(mode="after")
    def _check_tiles(self) -> AdaptiveSpatialMap:
        indices = [tile.tile_index for tile in self.tiles]
        if indices != sorted(indices):
            raise ValueError("tiles must be ordered by tile_index")
        if len(set(indices)) != len(indices):
            raise ValueError("a tile may appear at most once in a map")
        if indices != [decision.tile_index for decision in self.plan.decisions]:
            raise ValueError("map tiles and plan decisions must cover the same tiles in order")

        cells = sum(tile.cell_count for tile in self.tiles)
        if self.accounting.total_cell_count != cells:
            raise ValueError(
                f"total_cell_count ({self.accounting.total_cell_count}) must equal the "
                f"{cells} cells the tiles allocate"
            )
        occupied = sum(tile.occupied_cell_count for tile in self.tiles)
        if self.accounting.occupied_cell_count != occupied:
            raise ValueError(
                f"occupied_cell_count ({self.accounting.occupied_cell_count}) must equal the "
                f"{occupied} occupied cells the tiles hold"
            )
        mapped = sum(tile.mapped_point_count for tile in self.tiles)
        if self.accounting.mapped_point_count != mapped:
            raise ValueError(
                f"mapped_point_count ({self.accounting.mapped_point_count}) must equal the "
                f"{mapped} points the tiles hold"
            )
        return self

    # -- derived views -----------------------------------------------------
    @property
    def tile_count(self) -> int:
        """Tiles in the map."""
        return len(self.tiles)

    @property
    def occupancy_ratio(self) -> float:
        """Fraction of cells holding at least one point."""
        return self.accounting.occupancy_ratio

    @property
    def grid_bytes(self) -> int:
        """Exact size of the dense arrays. Arithmetic, not a measurement."""
        return self.accounting.total_cell_count * BYTES_PER_CELL

    @property
    def finest_resolution_m(self) -> float:
        """Smallest cell size present."""
        return min(tile.resolution_m for tile in self.tiles)

    @property
    def coarsest_resolution_m(self) -> float:
        """Largest cell size present."""
        return max(tile.resolution_m for tile in self.tiles)

    def tiles_by_level(self) -> dict[str, int]:
        """Tiles at each level, including levels with none."""
        counts = {level.value: 0 for level in LEVEL_ORDER}
        for tile in self.tiles:
            counts[tile.level.value] += 1
        return counts

    def cells_by_level(self) -> dict[str, int]:
        """Cells allocated at each level, including levels with none."""
        cells = {level.value: 0 for level in LEVEL_ORDER}
        for tile in self.tiles:
            cells[tile.level.value] += tile.cell_count
        return cells

    def area_weighted_resolution_m(self) -> float:
        """Mean cell size weighted by the ground area each tile covers.

        Area weighting rather than a plain mean over tiles, because the last
        row and column are clipped by the map bounds and cover less ground.
        """
        total_area = 0.0
        weighted = 0.0
        for tile in self.tiles:
            area = tile.bounds.size_x_m * tile.bounds.size_y_m
            total_area += area
            weighted += area * tile.resolution_m
        return weighted / total_area

    def tile_at(self, x: float, y: float) -> MapTile | None:
        """The tile containing ``(x, y)``, or ``None`` when outside the map.

        Linear rather than arithmetic on purpose: a map may omit no tile, but
        this stays correct if one is ever filtered out, and it is used for
        inspection rather than in the mapping hot path.
        """
        for tile in self.tiles:
            if tile.bounds.contains(x, y):
                return tile
        return None

    def summary(self, *, controller_duration_ms: float | None = None) -> AdaptiveSpatialMapSummary:
        """Compact description, without any grid.

        Args:
            controller_duration_ms: Planning time to report. Defaults to the
                duration recorded on the plan.
        """
        planning = (
            self.plan.duration_ms if controller_duration_ms is None else controller_duration_ms
        )
        return AdaptiveSpatialMapSummary(
            timestamp=self.timestamp,
            frame_id=self.frame_id,
            sensor_id=self.sensor_id,
            mapper=self.mapper,
            controller=self.plan.controller,
            is_adaptive=self.is_adaptive,
            bounds=self.bounds,
            tile_size_m=self.tile_size_m,
            tile_count=self.tile_count,
            accounting=self.accounting,
            tiles_by_level=self.tiles_by_level(),
            cells_by_level=self.cells_by_level(),
            finest_resolution_m=self.finest_resolution_m,
            coarsest_resolution_m=self.coarsest_resolution_m,
            area_weighted_resolution_m=self.area_weighted_resolution_m(),
            changed_tile_count=self.plan.changed_tile_count,
            grid_bytes=self.grid_bytes,
            controller_duration_ms=planning,
            mapping_duration_ms=self.duration_ms,
        )

    def to_adaptive_map(
        self, *, max_cells: int = DEFAULT_MAX_PROJECTED_CELLS
    ) -> tuple[AdaptiveMap, bool]:
        """Project **occupied cells only** into the ``AdaptiveMap`` contract.

        This is where the pre-existing per-cell ``resolution_m`` and
        ``resolution_level`` fields finally carry real, varying values: a cell
        from a CRITICAL tile reports its own fine cell size, and one from a LOW
        tile reports its coarse one.

        Empty cells are omitted rather than emitted as unknown, exactly as in
        Phase 6: a fine tile is overwhelmingly empty, and serialising those
        cells would say nothing at great length.

        ``risk_score`` and ``uncertainty`` are left at their defaults. Risk is
        object-level (Phase 7); no per-cell risk field exists, and filling
        them from a tile decision would present a region priority as a cell
        measurement.

        Cells are ordered by ``(tile_index, row, column)`` so the projection is
        deterministic.

        Args:
            max_cells: Upper bound on projected cells.

        Returns:
            ``(map, truncated)`` - the projection, and whether it was cut short.
        """
        cells: list[AdaptiveMapCell] = []
        truncated = False

        for tile in self.tiles:
            if len(cells) >= max_cells:
                truncated = truncated or tile.occupied_cell_count > 0
                continue
            rows, columns = np.nonzero(tile.point_count)
            remaining = max_cells - len(cells)
            if rows.size > remaining:
                truncated = True
                rows, columns = rows[:remaining], columns[:remaining]

            counts = tile.point_count[rows, columns]
            minimums = tile.min_height_m[rows, columns]
            maximums = tile.max_height_m[rows, columns]
            means = tile.mean_height_m[rows, columns]

            for index, (row, column) in enumerate(zip(rows, columns, strict=True)):
                x, y = tile.cell_centre(int(row), int(column))
                cells.append(
                    AdaptiveMapCell(
                        timestamp=self.timestamp,
                        x=x,
                        y=y,
                        resolution_m=tile.resolution_m,
                        resolution_level=tile.level,
                        occupancy=1.0,
                        occupancy_state=OccupancyState.OCCUPIED,
                        height_m=_finite_or_none(means[index]),
                        height_min_m=_finite_or_none(minimums[index]),
                        height_max_m=_finite_or_none(maximums[index]),
                        point_count=int(counts[index]),
                        coordinate_frame=self.coordinate_frame,
                        source=self.source,
                    )
                )

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


class MappingVariantMetrics(AdaptXModel):
    """What one mapping variant cost and produced on a given frame.

    Every field is measured or exact arithmetic over what was built. Nothing
    is estimated.
    """

    variant: str = Field(min_length=1, description="'fixed' or 'adaptive'.")
    mapper: str = Field(min_length=1)
    is_adaptive: bool

    uniform_resolution_m: float | None = Field(
        default=None,
        gt=0.0,
        description="Cell size, when one applies everywhere. Null for an adaptive map.",
    )
    finest_resolution_m: float = Field(gt=0.0)
    coarsest_resolution_m: float = Field(gt=0.0)

    total_cell_count: int = Field(ge=1)
    occupied_cell_count: int = Field(ge=0)
    mapped_point_count: int = Field(ge=0)
    out_of_bounds_point_count: int = Field(ge=0)
    grid_bytes: int = Field(ge=0)
    duration_ms: float = Field(ge=0.0, description="Grid construction only, measured.")

    @property
    def occupancy_ratio(self) -> float:
        """Fraction of cells holding at least one point."""
        return self.occupied_cell_count / self.total_cell_count


class MappingComparison(AdaptXModel):
    """A fixed-resolution and an adaptive map built over the **same** frame.

    The point of ADAPT-X is that detail should not be spread uniformly. This
    contract is how that claim is checked rather than asserted: both variants
    see identical input, and the cells each spends on high-priority and
    low-priority regions are counted.

    A ratio here is arithmetic over two measurements, never a claim of
    improvement on its own. A comparison where the adaptive map costs *more*
    is a valid result and is reported the same way.
    """

    fixed: MappingVariantMetrics
    adaptive: MappingVariantMetrics

    high_priority_tile_count: int = Field(
        ge=0, description="Tiles whose priority reached the configured HIGH threshold."
    )
    low_priority_tile_count: int = Field(
        ge=0, description="Tiles below the MEDIUM threshold, or with no influence at all."
    )
    cells_on_high_priority_tiles: int = Field(
        ge=0, description="Adaptive cells spent where priority reached HIGH."
    )
    cells_on_low_priority_tiles: int = Field(
        ge=0, description="Adaptive cells spent on the least important regions."
    )

    notes: str = Field(
        default=(
            "Both variants mapped the same processed frame. Cell counts and byte "
            "figures are exact arithmetic; durations are measured on this machine. "
            "Nothing here measures map correctness - no labelled reference map "
            "exists - and nothing here is a real-world performance claim."
        )
    )

    @property
    def cell_ratio(self) -> float:
        """Adaptive cells divided by fixed cells.

        Below 1.0 means the adaptive map allocated fewer cells than the uniform
        baseline it was compared against; above 1.0 means it allocated more.
        """
        return self.adaptive.total_cell_count / self.fixed.total_cell_count

    @property
    def byte_ratio(self) -> float:
        """Adaptive grid bytes divided by fixed grid bytes."""
        return self.adaptive.grid_bytes / self.fixed.grid_bytes

    @property
    def duration_ratio(self) -> float | None:
        """Adaptive mapping time divided by fixed mapping time.

        ``None`` when the fixed pass measured zero, rather than an infinity.
        """
        if self.fixed.duration_ms <= 0.0:
            return None
        return self.adaptive.duration_ms / self.fixed.duration_ms

    @property
    def high_priority_cell_share(self) -> float | None:
        """Share of adaptive cells spent on high-priority regions.

        ``None`` when no tile reached the HIGH threshold - the question has no
        answer rather than an answer of zero.
        """
        if self.high_priority_tile_count == 0:
            return None
        return self.cells_on_high_priority_tiles / self.adaptive.total_cell_count


def _finite_or_none(value: float) -> float | None:
    """Convert a NaN height to ``None`` for contracts that express it that way."""
    numeric = float(value)
    return None if math.isnan(numeric) else numeric
