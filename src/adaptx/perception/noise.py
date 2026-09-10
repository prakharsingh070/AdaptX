"""Baseline noise / outlier filtering (Phase 2B).

**This is a baseline.** It removes points that have too few neighbours nearby -
the signature of a spurious return, a dust or rain hit, or an isolated
measurement error.

Method: grid neighbour count
----------------------------
Space is divided into cubic cells of ``noise_cell_size_m``. A point's
neighbours are counted over the **3x3x3 block of cells centred on its own
cell** - a cube of side ``3 x cell size``. A point with fewer than
``noise_min_neighbors`` neighbours (itself excluded) is dropped.

This approximates radius outlier removal. It is deliberately *not* called a
radius filter, because the region tested is a cube, not a sphere, and a point
near a cell corner sees a different slice of space than one at a cell centre.
Exact radius or statistical (k-nearest-neighbour) outlier removal needs a
spatial index such as ``scipy.spatial.cKDTree``; that dependency is not
currently justified (ADR-014), and this stage is honest about being the
cheaper approximation rather than claiming to be the real thing.

Determinism: the result depends only on the input array and the configuration.
No sampling, no randomness.
"""

from __future__ import annotations

import itertools

import numpy as np

from adaptx.config.settings import LiDARSettings
from adaptx.perception.grid import cell_indices, cell_keys


class NoiseFilter:
    """Drops points with too few nearby neighbours. Baseline implementation."""

    name = "grid_neighbour_count_v1"
    #: True for comparison baselines, so results are never reported as final.
    is_baseline = True

    def __init__(self, settings: LiDARSettings) -> None:
        self._settings = settings

    def keep_mask(self, points: np.ndarray) -> np.ndarray:
        """Return a boolean mask that is True for points to keep.

        Args:
            points: ``(N, 3)`` or ``(N, 4)`` array of finite coordinates.

        Returns:
            ``(N,)`` boolean array; True means the point survives.

        Raises:
            InvalidPointCloudError: the cloud spans so many cells that the
                internal cell encoding would overflow. Filter by range or ROI
                first, or increase ``noise_cell_size_m``.
        """
        settings = self._settings
        count = int(points.shape[0])
        if count == 0:
            return np.zeros(0, dtype=bool)
        if settings.noise_min_neighbors == 0:
            return np.ones(count, dtype=bool)

        cell = cell_indices(points[:, :3], settings.noise_cell_size_m, stage=self.name)
        # margin=1 reserves a one-cell border, so stepping to a neighbouring
        # cell can never wrap into an adjacent row of the encoded grid.
        keys, strides = cell_keys(cell, stage=self.name, margin=1)

        occupied_keys, occupancy = np.unique(keys, return_counts=True)

        neighbours = np.zeros(count, dtype=np.int64)
        for offset in itertools.product((-1, 0, 1), repeat=3):
            delta = sum(step * stride for step, stride in zip(offset, strides, strict=True))
            position = np.searchsorted(occupied_keys, keys + delta)
            np.clip(position, 0, occupied_keys.size - 1, out=position)
            matched = occupied_keys[position] == keys + delta
            neighbours += np.where(matched, occupancy[position], 0)

        # The (0, 0, 0) offset counted the point itself.
        return (neighbours - 1) >= settings.noise_min_neighbors
