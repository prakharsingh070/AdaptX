"""Voxel downsampling (Phase 2B).

Reduces redundant points by keeping **one** point per occupied voxel of a
regular 3D grid.

Representative point
--------------------
The point kept for a voxel is the **real measured point closest to that
voxel's centroid**. The centroid itself is deliberately *not* emitted: it is a
coordinate no sensor ever returned, and ADAPT-X does not manufacture sensor
values (``docs/knowledge-base/20-constraints.md``). Keeping a genuine return
also keeps its intensity truthful, which averaging would not.

Ties - two points equidistant from the centroid - are broken by the lower index
in the input array, so the result never depends on sort implementation details.

Grid alignment
--------------
The grid is anchored at the coordinate-frame origin (0, 0, 0), *not* at the
cloud's bounding box. An origin-anchored grid gives the same voxel boundaries
for every frame, so two frames of the same scene downsample consistently and
results stay comparable across a run. A bbox-anchored grid would shift with the
data.

Voxel index of a point is ``floor(p / voxel_size)`` per axis, so a point
exactly on a voxel boundary belongs to the voxel above it.

Not implemented here
--------------------
No averaging, no normal estimation, no adaptive voxel size. Voxel size is
uniform and configured; the *adaptive* resolution idea at the heart of ADAPT-X
is a later phase and is not approximated here.
"""

from __future__ import annotations

import numpy as np

from adaptx.config.settings import LiDARSettings
from adaptx.perception.grid import cell_indices, cell_keys


class VoxelDownsampler:
    """Keeps one real point per occupied voxel."""

    name = "voxel_downsample"

    def __init__(self, settings: LiDARSettings) -> None:
        self._settings = settings

    @property
    def voxel_size_m(self) -> float:
        """Configured voxel edge length in metres."""
        return self._settings.voxel_size_m

    def select(self, points: np.ndarray) -> np.ndarray:
        """Return the indices of the points to keep, ascending.

        Indices are returned rather than a filtered array so the caller can
        combine this stage with others and allocate the output once. The
        ascending order preserves the input ordering of the survivors.

        Args:
            points: ``(N, 3)`` or ``(N, 4)`` array of finite coordinates.

        Returns:
            ``(K,)`` int array of kept indices, ``K`` = number of occupied voxels.
        """
        count = int(points.shape[0])
        if count == 0:
            return np.empty(0, dtype=np.intp)

        xyz = points[:, :3]
        voxel_index = cell_indices(xyz, self._settings.voxel_size_m, stage=self.name)

        # Group by an exact integer key rather than by row: same grouping,
        # markedly cheaper, and the encoding is bijective so there is no
        # collision risk.
        keys, _ = cell_keys(voxel_index, stage=self.name)
        _, inverse = np.unique(keys, return_inverse=True)
        inverse = inverse.ravel()
        voxel_count = int(inverse.max()) + 1

        # Per-voxel centroid, accumulated with bincount rather than a Python loop.
        occupancy = np.bincount(inverse, minlength=voxel_count).astype(np.float64)
        centroids = np.empty((voxel_count, 3), dtype=np.float64)
        for axis in range(3):
            centroids[:, axis] = (
                np.bincount(inverse, weights=xyz[:, axis], minlength=voxel_count) / occupancy
            )

        offsets = xyz - centroids[inverse]
        distance_squared = np.einsum("ij,ij->i", offsets, offsets)

        # Group the points by voxel with a stable integer sort. Profiling showed
        # the obvious formulation - a lexsort keyed on (index, distance, voxel) -
        # to be the single most expensive operation in the pipeline, roughly a
        # quarter of its total time at 400k points. A stable sort on one integer
        # key uses radix and is about twice as fast for the same grouping.
        order = np.argsort(inverse, kind="stable")
        grouped = inverse[order]
        group_start = np.flatnonzero(np.concatenate(([True], grouped[1:] != grouped[:-1])))
        group_size = np.diff(np.concatenate((group_start, [count])))

        # Smallest distance within each voxel, then the positions attaining it.
        grouped_distance = distance_squared[order]
        group_best = np.minimum.reduceat(grouped_distance, group_start)
        is_best = grouped_distance == np.repeat(group_best, group_size)

        # The stable sort preserved input order inside each group, so the first
        # position attaining the minimum is the lowest-index tie-break.
        best_positions = np.flatnonzero(is_best)
        best_group = np.repeat(np.arange(group_start.size), group_size)[best_positions]
        first_in_group = np.concatenate(([True], best_group[1:] != best_group[:-1]))

        return np.sort(order[best_positions[first_in_group]])
