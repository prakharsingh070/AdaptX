"""Deterministic fixed-resolution 2.5D mapping (Phase 6).

Consumes the :class:`~adaptx.models.point_cloud.PointCloudFrame` output of
Phase 2 and bins it into a bounded XY grid, summarising ``z`` per cell:

    frame + resolution -> bounds check -> cell indices -> accumulate -> SpatialMap

**This is a deterministic fixed-resolution baseline.** One cell size applies
everywhere. It is the reference the adaptive mapper must eventually beat
(ADR-003), and it is deliberately the least clever thing that can be measured.

What this module must never do
------------------------------
It does **not** choose its own resolution. It receives a
:class:`~adaptx.models.resolution.ResolutionDecision` and applies it. It is
never handed tracks, predicted trajectories, risk or uncertainty, so it cannot
make a risk-aware choice even by accident (ADR-029). When an adaptive
controller exists it will produce a different decision through the same
contract, and this module will not change.

It also does no preprocessing. Validation, NaN removal, ROI, range, voxel,
ground and noise all happened upstream; repeating any of it here would create a
second filtering path that could drift from the first.

Frame-local
-----------
Every call builds a complete map from one frame. Nothing carries over. This is
not SLAM, not a persistent world map, and there is no state for frame ``N`` to
leak into frame ``N+1`` (ADR-030).

Indexing
--------
Cells are anchored at the map's lower corner and are half-open (ADR-028)::

    column = floor((x - min_x) / resolution)
    row    = floor((y - min_y) / resolution)

A point exactly on ``min_x``/``min_y`` is the first cell; a point exactly on
``max_x``/``max_y`` is **out of bounds**, because it would index one cell past
the last column. Negative coordinates are ordinary - the bounds are explicit,
so nothing assumes the map starts at the origin.

Quantisation goes through :func:`adaptx.perception.grid.cell_indices`, which
carries the int64 overflow guard added in Phase 2B, rather than a second
`astype(int64)` that could silently wrap.
"""

from __future__ import annotations

import time

import numpy as np

from adaptx.config.settings import MapSettings
from adaptx.core.exceptions import InvalidPointCloudError
from adaptx.core.logging import get_logger
from adaptx.mapping.interfaces import AdaptiveMapper
from adaptx.models.point_cloud import PointCloudFrame
from adaptx.models.resolution import ResolutionDecision
from adaptx.models.spatial_map import (
    MapAccounting,
    MapBounds,
    MappingConfiguration,
    SpatialMap,
)
from adaptx.perception.grid import cell_indices

logger = get_logger(__name__)

#: Stage name used in quantisation errors, matching the Phase 2B convention.
_STAGE = "mapping"


class FixedResolutionMapper(AdaptiveMapper):
    """Bins a processed frame into a bounded, uniform-resolution 2.5D grid."""

    name = "fixed_resolution_mapper_v1"
    #: False: one uniform cell size, not a risk-aware allocation (ADR-003).
    is_adaptive = False

    def __init__(self, settings: MapSettings) -> None:
        self._settings = settings

    # -- state -------------------------------------------------------------
    @property
    def bounds(self) -> MapBounds:
        """Configured map extent."""
        return MapBounds(
            min_x=self._settings.min_x_m,
            max_x=self._settings.max_x_m,
            min_y=self._settings.min_y_m,
            max_y=self._settings.max_y_m,
        )

    def configuration(self, resolution_m: float) -> MappingConfiguration:
        """Snapshot of the settings that shape one mapping pass."""
        return MappingConfiguration(
            resolution_m=resolution_m,
            bounds=self.bounds,
            max_cells=self._settings.max_cells,
        )

    def default_resolution(self) -> ResolutionDecision:
        """The configured fixed-resolution decision.

        Phase 6 has no controller, so this stands in for one: a constant read
        from configuration, labelled as such.
        """
        return ResolutionDecision.fixed(self._settings.resolution_m)

    def grid_shape(self, resolution_m: float) -> tuple[int, int]:
        """``(height, width)`` in cells for ``resolution_m``, validated.

        Computed **before** any array is allocated, so an unusable
        bounds/resolution combination is rejected rather than attempted
        (ADR-028).

        Raises:
            InvalidPointCloudError: the resolution is outside configured limits,
                or the grid would exceed ``max_cells``.
        """
        settings = self._settings
        if not np.isfinite(resolution_m) or resolution_m <= 0.0:
            raise InvalidPointCloudError(
                f"mapping resolution must be finite and positive, got {resolution_m}",
                details={"resolution_m": resolution_m},
            )
        if resolution_m < settings.min_resolution_m or resolution_m > settings.max_resolution_m:
            raise InvalidPointCloudError(
                f"mapping resolution {resolution_m} m is outside the configured range "
                f"[{settings.min_resolution_m}, {settings.max_resolution_m}] m",
                details={
                    "resolution_m": resolution_m,
                    "min_resolution_m": settings.min_resolution_m,
                    "max_resolution_m": settings.max_resolution_m,
                },
            )

        bounds = self.bounds
        width = int(np.ceil(bounds.size_x_m / resolution_m))
        height = int(np.ceil(bounds.size_y_m / resolution_m))
        if width < 1 or height < 1:
            raise InvalidPointCloudError(
                f"mapping bounds produce an empty grid at {resolution_m} m",
                details={"width": width, "height": height},
            )

        cells = width * height
        if cells > settings.max_cells:
            raise InvalidPointCloudError(
                f"mapping grid would allocate {cells} cells at {resolution_m} m, above the "
                f"configured limit of {settings.max_cells}; use a coarser resolution or "
                f"tighter bounds",
                details={
                    "resolution_m": resolution_m,
                    "width": width,
                    "height": height,
                    "cells": cells,
                    "max_cells": settings.max_cells,
                },
            )
        return height, width

    def reset(self) -> None:
        """No-op: the mapper holds no state between frames (ADR-030).

        Declared by the :class:`~adaptx.mapping.interfaces.AdaptiveMapper`
        contract and kept deliberately empty rather than removed, so a future
        accumulating mapper can implement it without changing callers. Being a
        no-op is the guarantee, not an oversight.
        """

    # -- mapping -----------------------------------------------------------
    def build(
        self, frame: PointCloudFrame, resolution: ResolutionDecision | None = None
    ) -> SpatialMap:
        """Build a complete 2.5D map from one frame.

        Args:
            frame: A processed frame from the Phase 2 pipeline. Its coordinates
                are already guaranteed finite by the contract.
            resolution: The resolution to apply. Defaults to the configured
                fixed value.

        Returns:
            A map covering the configured bounds, with per-cell point counts and
            height statistics.

        Raises:
            InvalidPointCloudError: the resolution is out of range, or the grid
                would exceed the configured cell limit.
        """
        started = time.perf_counter()
        decision = resolution if resolution is not None else self.default_resolution()
        size = decision.resolution_m
        height, width = self.grid_shape(size)
        bounds = self.bounds

        cell_total = width * height
        points = frame.points
        input_count = int(points.shape[0])
        mapped_count = 0
        flat: np.ndarray | None = None
        heights: np.ndarray | None = None

        if input_count:
            x = points[:, 0]
            y = points[:, 1]
            z = points[:, 2]

            # Half-open on the upper edge, so a point at max_x belongs to no cell.
            inside = (
                (x >= bounds.min_x) & (x < bounds.max_x) & (y >= bounds.min_y) & (y < bounds.max_y)
            )
            mapped_count = int(np.count_nonzero(inside))

            if mapped_count:
                local = np.empty((mapped_count, 2), dtype=np.float64)
                local[:, 0] = x[inside] - bounds.min_x
                local[:, 1] = y[inside] - bounds.min_y
                indices = cell_indices(local, size, stage=_STAGE)

                # Ceil-derived dimensions can leave the final cell partly
                # outside the bounds; clip guards the exact-boundary rounding
                # case rather than letting an index run past the array.
                columns = np.clip(indices[:, 0], 0, width - 1)
                rows = np.clip(indices[:, 1], 0, height - 1)
                flat = rows * width + columns
                heights = z[inside].astype(np.float64, copy=False)

        # One allocation per output array, whether or not the frame had points.
        # `fmin`/`fmax` ignore NaN, so an untouched cell keeps the NaN it was
        # initialised with and no second pass over the grid is needed.
        minimums_flat = np.full(cell_total, np.nan, dtype=np.float64)
        maximums_flat = np.full(cell_total, np.nan, dtype=np.float64)
        means_flat = np.full(cell_total, np.nan, dtype=np.float64)

        if flat is not None and heights is not None:
            counts_flat = np.bincount(flat, minlength=cell_total).astype(np.int64, copy=False)
            sums_flat = np.bincount(flat, weights=heights, minlength=cell_total)
            np.fmin.at(minimums_flat, flat, heights)
            np.fmax.at(maximums_flat, flat, heights)
            occupied = counts_flat > 0
            means_flat[occupied] = sums_flat[occupied] / counts_flat[occupied]
        else:
            counts_flat = np.zeros(cell_total, dtype=np.int64)

        counts = counts_flat.reshape(height, width)
        minimums = minimums_flat.reshape(height, width)
        maximums = maximums_flat.reshape(height, width)
        means = means_flat.reshape(height, width)

        occupied_cells = int(np.count_nonzero(counts))
        duration_ms = (time.perf_counter() - started) * 1000.0

        logger.debug(
            "map built",
            extra={
                "context": {
                    "frame_id": frame.frame_id,
                    "resolution_m": size,
                    "grid": f"{width}x{height}",
                    "input_points": input_count,
                    "mapped_points": mapped_count,
                    "occupied_cells": occupied_cells,
                    "duration_ms": round(duration_ms, 3),
                }
            },
        )

        return SpatialMap(
            timestamp=frame.timestamp,
            frame_id=frame.frame_id,
            sensor_id=frame.sensor_id,
            mapper=self.name,
            is_adaptive=self.is_adaptive,
            resolution=decision,
            bounds=bounds,
            width=width,
            height=height,
            point_count=counts,
            min_height_m=minimums,
            max_height_m=maximums,
            mean_height_m=means,
            accounting=MapAccounting(
                input_point_count=input_count,
                mapped_point_count=mapped_count,
                out_of_bounds_point_count=input_count - mapped_count,
                occupied_cell_count=occupied_cells,
                total_cell_count=width * height,
            ),
            duration_ms=duration_ms,
            configuration=self.configuration(size),
            coordinate_frame=frame.coordinate_frame,
            source=frame.source,
        )


def build_mapper(settings: MapSettings) -> FixedResolutionMapper:
    """Construct the configured baseline mapper."""
    return FixedResolutionMapper(settings)
