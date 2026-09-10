"""Deterministic region-adaptive 2.5D mapping (Phase 8).

Applies a :class:`~adaptx.models.adaptive_resolution.ResolutionPlan` to a
processed frame, building one dense sub-grid per region at that region own
cell size::

    frame + plan -> bounds check -> tile assignment -> per-tile binning
                 -> AdaptiveSpatialMap

What this module must never do
------------------------------
It does **not** choose resolution. Every cell size it uses comes from the plan
it was handed, and it is never given tracks, trajectories, risk or uncertainty,
so a risk-aware choice is structurally impossible here (ADR-029, ADR-037).

It also does no preprocessing, for the same reason the Phase 6 mapper does
none: repeating validation or filtering would create a second path that could
drift from the first.

Relationship to the Phase 6 mapper
----------------------------------
The binning arithmetic is identical, applied once per tile rather than once per
map: half-open cells anchored at the **tile** lower corner, quantised through
:func:`adaptx.perception.grid.cell_indices` so the int64 overflow guard added
in Phase 2B still applies, and ``fmin``/``fmax`` accumulation that leaves an
untouched cell holding the NaN it was initialised with (ADR-031).

:class:`~adaptx.mapping.grid_mapper.FixedResolutionMapper` is untouched and
remains the baseline this must be measured against (ADR-003).

Point accounting
----------------
Tiles partition the map exactly (see :mod:`adaptx.mapping.tiles`), so a point
inside the bounds lands in exactly one tile and one cell of it. A point outside
the bounds is counted as out of bounds, never silently dropped:
``input == mapped + out_of_bounds`` is enforced by the contract.
"""

from __future__ import annotations

import time

import numpy as np

from adaptx.config.settings import AdaptiveResolutionSettings, MapSettings
from adaptx.core.exceptions import InvalidPointCloudError
from adaptx.core.logging import get_logger
from adaptx.mapping.interfaces import RegionAdaptiveMapper
from adaptx.mapping.tiles import STAGE, TileGrid, cells_for
from adaptx.models.adaptive_map import AdaptiveSpatialMap, MapTile
from adaptx.models.adaptive_resolution import ResolutionPlan, TileResolutionDecision
from adaptx.models.point_cloud import PointCloudFrame
from adaptx.models.spatial_map import MapAccounting, MapBounds
from adaptx.perception.grid import cell_indices

logger = get_logger(__name__)


class TiledAdaptiveMapper(RegionAdaptiveMapper):
    """Bins a processed frame into a tiled grid at per-region resolutions."""

    name = "tiled_adaptive_mapper_v1"
    #: True: resolution varies by region (ADR-003).
    is_adaptive = True

    def __init__(self, settings: MapSettings, adaptive: AdaptiveResolutionSettings) -> None:
        self._settings = settings
        self._adaptive = adaptive

    # -- state -------------------------------------------------------------
    @property
    def bounds(self) -> MapBounds:
        """Configured map extent, the same one the fixed mapper covers."""
        return MapBounds(
            min_x=self._settings.min_x_m,
            max_x=self._settings.max_x_m,
            min_y=self._settings.min_y_m,
            max_y=self._settings.max_y_m,
        )

    def tile_grid(self) -> TileGrid:
        """The region partition this mapper builds over."""
        return TileGrid.over(
            self.bounds,
            self._adaptive.tile_size_m,
            max_tiles=self._adaptive.max_tiles,
        )

    def reset(self) -> None:
        """No-op: the mapper holds no state between frames (ADR-030).

        Declared by the contract and kept deliberately empty rather than
        removed, so a future accumulating mapper can implement it without
        changing callers. Being a no-op is the guarantee, not an oversight.
        The stabilisation state that *does* persist belongs to the controller.
        """

    # -- mapping -----------------------------------------------------------
    def build(self, frame: PointCloudFrame, plan: ResolutionPlan) -> AdaptiveSpatialMap:
        """Build a complete tiled map from one frame.

        Args:
            frame: A processed frame from the Phase 2 pipeline. Its coordinates
                are already guaranteed finite by the contract.
            plan: One decision per region, covering the whole map.

        Returns:
            A map whose tiles hold per-cell point counts and height statistics
            at the resolution the plan chose for each region.

        Raises:
            InvalidPointCloudError: the plan does not cover this mapper tiling,
                or the total cell count exceeds the configured ceiling.
        """
        started = time.perf_counter()
        grid = self.tile_grid()
        decisions = self._validated_decisions(grid, plan)

        points = frame.points
        input_count = int(points.shape[0])
        assignments = self._assign(grid, points)

        tiles: list[MapTile] = []
        mapped_total = 0
        occupied_total = 0
        cell_total = 0

        for decision in decisions:
            indices = assignments.get(decision.tile_index)
            tile = self._build_tile(frame, decision, indices)
            tiles.append(tile)
            mapped_total += tile.mapped_point_count
            occupied_total += tile.occupied_cell_count
            cell_total += tile.cell_count

        duration_ms = (time.perf_counter() - started) * 1000.0
        logger.debug(
            "adaptive map built",
            extra={
                "context": {
                    "frame_id": frame.frame_id,
                    "tiles": len(tiles),
                    "cells": cell_total,
                    "input_points": input_count,
                    "mapped_points": mapped_total,
                    "occupied_cells": occupied_total,
                    "duration_ms": round(duration_ms, 3),
                }
            },
        )

        return AdaptiveSpatialMap(
            timestamp=frame.timestamp,
            frame_id=frame.frame_id,
            sensor_id=frame.sensor_id,
            mapper=self.name,
            is_adaptive=self.is_adaptive,
            bounds=grid.bounds,
            tile_size_m=grid.tile_size_m,
            tiles=tiles,
            plan=plan,
            accounting=MapAccounting(
                input_point_count=input_count,
                mapped_point_count=mapped_total,
                out_of_bounds_point_count=input_count - mapped_total,
                occupied_cell_count=occupied_total,
                total_cell_count=cell_total,
            ),
            duration_ms=duration_ms,
            coordinate_frame=frame.coordinate_frame,
            source=frame.source,
        )

    # -- internals ---------------------------------------------------------
    def _validated_decisions(
        self, grid: TileGrid, plan: ResolutionPlan
    ) -> list[TileResolutionDecision]:
        """Check that the plan covers exactly this tiling, and bound the cells.

        A plan built against different bounds or a different tile size would
        silently map points into the wrong regions, so the mismatch is rejected
        rather than absorbed.
        """
        expected = grid.tile_count
        if len(plan.decisions) != expected:
            raise InvalidPointCloudError(
                f"{STAGE}: the plan covers {len(plan.decisions)} regions but the map has "
                f"{expected}; the plan was built against a different tiling",
                details={
                    "stage": STAGE,
                    "plan_tiles": len(plan.decisions),
                    "map_tiles": expected,
                    "tile_size_m": grid.tile_size_m,
                },
            )
        if plan.configuration.tile_size_m != grid.tile_size_m:
            raise InvalidPointCloudError(
                f"{STAGE}: the plan was built at a tile size of "
                f"{plan.configuration.tile_size_m} m but the mapper uses {grid.tile_size_m} m",
                details={
                    "stage": STAGE,
                    "plan_tile_size_m": plan.configuration.tile_size_m,
                    "mapper_tile_size_m": grid.tile_size_m,
                },
            )

        total = 0
        for position, decision in enumerate(plan.decisions):
            if decision.tile_index != position:
                raise InvalidPointCloudError(
                    f"{STAGE}: the plan must carry one decision per region in index order; "
                    f"position {position} holds tile {decision.tile_index}",
                    details={"stage": STAGE, "position": position},
                )
            expected_bounds = grid.tile_bounds(decision.tile_index)
            if decision.bounds != expected_bounds:
                raise InvalidPointCloudError(
                    f"{STAGE}: region {decision.tile_index} declares an extent the tiling "
                    f"does not produce; the plan was built against different bounds",
                    details={"stage": STAGE, "tile_index": decision.tile_index},
                )
            height, width = cells_for(expected_bounds, decision.resolution_m)
            if (height, width) != (decision.cell_height, decision.cell_width):
                raise InvalidPointCloudError(
                    f"{STAGE}: region {decision.tile_index} declares "
                    f"{decision.cell_height}x{decision.cell_width} cells but its extent at "
                    f"{decision.resolution_m} m needs {height}x{width}",
                    details={"stage": STAGE, "tile_index": decision.tile_index},
                )
            total += height * width

        if total > self._settings.max_cells:
            raise InvalidPointCloudError(
                f"{STAGE}: the plan would allocate {total} cells, above the configured limit "
                f"of {self._settings.max_cells}; coarsen the plan or tighten the bounds",
                details={
                    "stage": STAGE,
                    "cells": total,
                    "max_cells": self._settings.max_cells,
                },
            )
        return list(plan.decisions)

    def _assign(self, grid: TileGrid, points: np.ndarray) -> dict[int, np.ndarray]:
        """Group point row indices by the tile that holds them.

        Sorted once rather than filtered per tile: a per-tile mask would sweep
        the whole cloud for every region, which is the difference between one
        pass and several hundred.
        """
        if points.shape[0] == 0:
            return {}

        tiles = grid.locate_many(points[:, 0], points[:, 1])
        inside = tiles >= 0
        if not np.any(inside):
            return {}

        rows = np.nonzero(inside)[0]
        keys = tiles[rows]
        order = np.argsort(keys, kind="stable")
        rows = rows[order]
        keys = keys[order]

        boundaries = np.searchsorted(keys, np.arange(grid.tile_count + 1), side="left")
        grouped: dict[int, np.ndarray] = {}
        for tile_index in range(grid.tile_count):
            start, stop = int(boundaries[tile_index]), int(boundaries[tile_index + 1])
            if stop > start:
                grouped[tile_index] = rows[start:stop]
        return grouped

    def _build_tile(
        self,
        frame: PointCloudFrame,
        decision: TileResolutionDecision,
        rows: np.ndarray | None,
    ) -> MapTile:
        """Bin one region point set into its own dense sub-grid.

        The extent comes from the decision rather than being recomputed:
        :meth:`_validated_decisions` has already checked it against the tiling,
        and rebuilding it per tile was measurably the dominant cost of a pass
        (Experiment 007).
        """
        bounds = decision.bounds
        height, width = decision.cell_height, decision.cell_width
        size = decision.resolution_m
        cell_total = height * width

        minimums_flat = np.full(cell_total, np.nan, dtype=np.float64)
        maximums_flat = np.full(cell_total, np.nan, dtype=np.float64)
        means_flat = np.full(cell_total, np.nan, dtype=np.float64)

        if rows is None or rows.size == 0:
            counts_flat = np.zeros(cell_total, dtype=np.int64)
            mapped = 0
        else:
            selected = frame.points[rows]
            mapped = int(selected.shape[0])

            local = np.empty((mapped, 2), dtype=np.float64)
            local[:, 0] = selected[:, 0] - bounds.min_x
            local[:, 1] = selected[:, 1] - bounds.min_y
            indices = cell_indices(local, size, stage=STAGE)

            # Ceil-derived dimensions can leave the final cell partly outside
            # the tile; clip guards the exact-boundary rounding case rather
            # than letting an index run past the array.
            columns = np.clip(indices[:, 0], 0, width - 1)
            cell_rows = np.clip(indices[:, 1], 0, height - 1)
            flat = cell_rows * width + columns
            heights = selected[:, 2].astype(np.float64, copy=False)

            counts_flat = np.bincount(flat, minlength=cell_total).astype(np.int64, copy=False)
            sums_flat = np.bincount(flat, weights=heights, minlength=cell_total)
            np.fmin.at(minimums_flat, flat, heights)
            np.fmax.at(maximums_flat, flat, heights)
            occupied = counts_flat > 0
            means_flat[occupied] = sums_flat[occupied] / counts_flat[occupied]

        counts = counts_flat.reshape(height, width)
        return MapTile(
            timestamp=frame.timestamp,
            tile_index=decision.tile_index,
            tile_row=decision.tile_row,
            tile_column=decision.tile_column,
            bounds=bounds,
            level=decision.level,
            resolution_m=size,
            width=width,
            height=height,
            point_count=counts,
            min_height_m=minimums_flat.reshape(height, width),
            max_height_m=maximums_flat.reshape(height, width),
            mean_height_m=means_flat.reshape(height, width),
            mapped_point_count=mapped,
            occupied_cell_count=int(np.count_nonzero(counts)),
        )


def build_adaptive_mapper(
    settings: MapSettings, adaptive: AdaptiveResolutionSettings
) -> TiledAdaptiveMapper:
    """Construct the configured region-adaptive mapper."""
    return TiledAdaptiveMapper(settings, adaptive)
