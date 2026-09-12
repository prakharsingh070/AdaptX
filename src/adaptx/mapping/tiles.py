"""Tile geometry for region-adaptive mapping (Phase 8).

One tiling, used by both sides
------------------------------
The resolution controller decides *per region* and the adaptive mapper builds
*per region*. If each derived its own tile boundaries they could disagree by a
rounding step, and a point would fall into a tile no decision covered. They
share this module instead, so the partition is defined exactly once.

Boundary convention
-------------------
Identical to Phase 6 cells (ADR-028), one level up: tiles are anchored at the
map lower corner and are **half-open** on their upper edges::

    column = floor((x - min_x) / tile_size)
    row    = floor((y - min_y) / tile_size)

A point exactly on ``min_x``/``min_y`` belongs to the first tile; a point
exactly on ``max_x``/``max_y`` is outside the map. A point on an interior tile
boundary belongs to the tile above/right of it, and to that one only - so
tiles partition the extent with no gap and no double coverage.

The last row and column are **clipped** to the map bounds, so a map whose
extent is not a whole number of tiles still ends exactly at its bounds rather
than covering ground it does not have.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from adaptx.core.exceptions import InvalidPointCloudError
from adaptx.models.spatial_map import MapBounds

#: Stage name used in quantisation errors, matching the Phase 2B convention.
STAGE = "adaptive_mapping"


#: Grids by geometry; a session uses one, tests a handful.
_GRIDS: dict[tuple[float, float, float, float, float, int, int], TileGrid] = {}
_GRID_CACHE_SIZE = 32


@dataclass(frozen=True, slots=True)
class TileGrid:
    """A partition of a map extent into fixed-size square regions.

    Immutable: a tiling is a property of the bounds and the tile size, and
    nothing should be able to reshape one after a plan has been built against
    it.
    """

    bounds: MapBounds
    tile_size_m: float
    columns: int
    rows: int
    # Geometry memos. A tile's extent and cell shape are functions of the
    # (immutable) grid alone, yet the controller and mapper asked for them
    # about 1,200 times per frame on a 144-tile map - 8-9 times per tile -
    # and each answer built a MapBounds model. Measured in the live loop
    # (Experiment 015) as a leading cost of the Phase 8 stage; memoising them
    # changes no value. Excluded from equality and repr.
    _tile_bounds: dict[int, MapBounds] = field(default_factory=dict, repr=False, compare=False)
    _cell_shapes: dict[tuple[int, float], tuple[int, int]] = field(
        default_factory=dict, repr=False, compare=False
    )

    @classmethod
    def over(cls, bounds: MapBounds, tile_size_m: float, *, max_tiles: int) -> TileGrid:
        """Build the tiling of ``bounds`` at ``tile_size_m``.

        Args:
            bounds: Map extent to partition.
            tile_size_m: Edge length of one square tile, in metres.
            max_tiles: Ceiling on the number of tiles, checked **before** any
                per-tile work, so an unusable combination is rejected rather
                than attempted (the ADR-028 rule, applied to regions).

        Raises:
            InvalidPointCloudError: the tile size is not finite and positive,
                or the tiling would exceed ``max_tiles``.
        """
        if not math.isfinite(tile_size_m) or tile_size_m <= 0.0:
            raise InvalidPointCloudError(
                f"tile size must be finite and positive, got {tile_size_m}",
                details={"tile_size_m": tile_size_m},
            )

        columns = max(1, math.ceil(bounds.size_x_m / tile_size_m))
        rows = max(1, math.ceil(bounds.size_y_m / tile_size_m))
        count = columns * rows
        if count > max_tiles:
            raise InvalidPointCloudError(
                f"tiling the map at {tile_size_m} m would create {count} regions, above the "
                f"configured limit of {max_tiles}; use a larger tile size or tighter bounds",
                details={
                    "tile_size_m": tile_size_m,
                    "columns": columns,
                    "rows": rows,
                    "tiles": count,
                    "max_tiles": max_tiles,
                },
            )
        # One grid object per distinct geometry, so the memos above survive
        # across frames: the controller and mapper rebuild "their" grid from
        # settings on every call, and the settings do not change mid-session.
        key = (bounds.min_x, bounds.max_x, bounds.min_y, bounds.max_y, tile_size_m, columns, rows)
        cached = _GRIDS.get(key)
        if cached is None:
            cached = cls(bounds=bounds, tile_size_m=tile_size_m, columns=columns, rows=rows)
            if len(_GRIDS) >= _GRID_CACHE_SIZE:
                _GRIDS.clear()
            _GRIDS[key] = cached
        return cached

    # -- shape -------------------------------------------------------------
    @property
    def tile_count(self) -> int:
        """Number of tiles in the partition."""
        return self.columns * self.rows

    def index_of(self, row: int, column: int) -> int:
        """Row-major index of the tile at ``(row, column)``."""
        return row * self.columns + column

    def row_column(self, tile_index: int) -> tuple[int, int]:
        """``(row, column)`` of a row-major tile index."""
        return divmod(tile_index, self.columns)

    # -- geometry ----------------------------------------------------------
    def tile_bounds(self, tile_index: int) -> MapBounds:
        """Extent of one tile, clipped to the map bounds. Memoised per grid."""
        cached = self._tile_bounds.get(tile_index)
        if cached is not None:
            return cached
        row, column = self.row_column(tile_index)
        min_x = self.bounds.min_x + column * self.tile_size_m
        min_y = self.bounds.min_y + row * self.tile_size_m
        bounds = MapBounds(
            min_x=min_x,
            max_x=min(min_x + self.tile_size_m, self.bounds.max_x),
            min_y=min_y,
            max_y=min(min_y + self.tile_size_m, self.bounds.max_y),
        )
        self._tile_bounds[tile_index] = bounds
        return bounds

    def tile_centre(self, tile_index: int) -> tuple[float, float]:
        """Centre of one tile in metres, accounting for clipping at the edges."""
        tile = self.tile_bounds(tile_index)
        return (
            (tile.min_x + tile.max_x) / 2.0,
            (tile.min_y + tile.max_y) / 2.0,
        )

    def cell_shape(self, tile_index: int, resolution_m: float) -> tuple[int, int]:
        """``(height, width)`` in cells for one tile at ``resolution_m``.

        Derived from the tile clipped extent, so a clipped edge tile allocates
        only the cells it needs rather than a full tile worth.
        """
        key = (tile_index, resolution_m)
        cached = self._cell_shapes.get(key)
        if cached is None:
            cached = cells_for(self.tile_bounds(tile_index), resolution_m)
            self._cell_shapes[key] = cached
        return cached

    # -- lookup ------------------------------------------------------------
    def locate(self, x: float, y: float) -> int | None:
        """Tile index containing ``(x, y)``, or ``None`` when outside the map."""
        if not self.bounds.contains(x, y):
            return None
        column = min(int((x - self.bounds.min_x) // self.tile_size_m), self.columns - 1)
        row = min(int((y - self.bounds.min_y) // self.tile_size_m), self.rows - 1)
        return self.index_of(row, column)

    def locate_many(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """Tile index for each coordinate pair; ``-1`` for points outside the map.

        Vectorised counterpart of :meth:`locate`, using the same half-open
        rule. Ceil-derived dimensions can leave the final tile partly outside
        the bounds, so indices are clipped to the last row and column - the
        same guard the Phase 6 mapper applies to cells.
        """
        inside = (
            (xs >= self.bounds.min_x)
            & (xs < self.bounds.max_x)
            & (ys >= self.bounds.min_y)
            & (ys < self.bounds.max_y)
        )
        indices = np.full(xs.shape[0], -1, dtype=np.int64)
        if not np.any(inside):
            return indices

        columns = np.floor((xs[inside] - self.bounds.min_x) / self.tile_size_m)
        rows = np.floor((ys[inside] - self.bounds.min_y) / self.tile_size_m)
        if columns.size and not np.isfinite(columns).all():
            raise InvalidPointCloudError(
                f"{STAGE}: a coordinate could not be assigned to a region",
                details={"stage": STAGE, "tile_size_m": self.tile_size_m},
            )
        columns = np.clip(columns.astype(np.int64), 0, self.columns - 1)
        rows = np.clip(rows.astype(np.int64), 0, self.rows - 1)
        indices[inside] = rows * self.columns + columns
        return indices

    def indices_within(self, x: float, y: float, radius_m: float) -> list[int]:
        """Tiles whose extent lies within ``radius_m`` of the point ``(x, y)``.

        A tile qualifies when the closest point of its extent is within the
        radius, so influence reaches a tile the object merely borders rather
        than only the one holding its centre.

        Returns indices in ascending order, which is what makes an influence
        pass deterministic regardless of how the objects were ordered.
        """
        if radius_m < 0.0 or not math.isfinite(radius_m):
            return []

        first_column = math.floor((x - radius_m - self.bounds.min_x) / self.tile_size_m)
        last_column = math.floor((x + radius_m - self.bounds.min_x) / self.tile_size_m)
        first_row = math.floor((y - radius_m - self.bounds.min_y) / self.tile_size_m)
        last_row = math.floor((y + radius_m - self.bounds.min_y) / self.tile_size_m)

        first_column = max(first_column, 0)
        first_row = max(first_row, 0)
        last_column = min(last_column, self.columns - 1)
        last_row = min(last_row, self.rows - 1)

        found: list[int] = []
        for row in range(first_row, last_row + 1):
            for column in range(first_column, last_column + 1):
                index = self.index_of(row, column)
                if self.distance_to(index, x, y) <= radius_m:
                    found.append(index)
        return found

    def distance_to(self, tile_index: int, x: float, y: float) -> float:
        """Planar distance from ``(x, y)`` to the nearest point of a tile extent."""
        tile = self.tile_bounds(tile_index)
        dx = max(tile.min_x - x, 0.0, x - tile.max_x)
        dy = max(tile.min_y - y, 0.0, y - tile.max_y)
        return math.hypot(dx, dy)

    def total_cells(self, resolutions_m: dict[int, float]) -> int:
        """Cells the whole tiling would allocate given a cell size per tile."""
        total = 0
        for tile_index, resolution_m in resolutions_m.items():
            height, width = self.cell_shape(tile_index, resolution_m)
            total += height * width
        return total


def cells_for(bounds: MapBounds, resolution_m: float) -> tuple[int, int]:
    """``(height, width)`` in cells covering ``bounds`` at ``resolution_m``.

    Split out from :meth:`TileGrid.cell_shape` so a caller that already holds a
    tile extent does not rebuild one to ask its size. Rounding before the ceil
    absorbs the float error in an exact division such as 10 / 0.1, which would
    otherwise allocate an extra row of cells that no point can reach.
    """
    width = max(1, math.ceil(round(bounds.size_x_m / resolution_m, 9)))
    height = max(1, math.ceil(round(bounds.size_y_m / resolution_m, 9)))
    return height, width
