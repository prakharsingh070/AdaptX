"""Spatial clustering of non-ground points (Phase 3).

Method: grid connected components
---------------------------------
Space is divided into cubic cells of ``cluster_tolerance_m``. Two occupied
cells that touch - anywhere in the 3x3x3 block, faces, edges or corners - are
joined, and each connected group of cells becomes one cluster. Every point in
those cells belongs to that cluster.

This is an **approximation of Euclidean clustering**, and the module is named
for what it does rather than borrowing the name of an algorithm it is not:

* It can **merge** two objects whose points land in touching cells even when no
  pair of points is actually within the tolerance. Two pedestrians standing
  half a metre apart may come back as one cluster.
* It cannot **split** points that share a cell, whatever their true separation
  inside it.
* It has no notion of core points, so it is not DBSCAN. Sparse clusters are
  removed afterwards by the point-count filter rather than by a density rule.

Exact Euclidean clustering needs pairwise distances between candidate
neighbours, which in practice means a spatial index - ``scipy.spatial.cKDTree``
or equivalent. That dependency is not yet justified (ADR-014 made the same call
for noise filtering), and this approximation is adequate for a baseline whose
job is to be replaceable.

Determinism
-----------
Cluster labels are assigned in ascending order of first appearance in the input
array, so the same points in the same order always produce the same labels. No
randomness anywhere.

Cost
----
``O(M x 26)`` edge construction over ``M`` occupied cells, then label
propagation with pointer jumping, which converges in about ``log`` of the
longest cluster's diameter. Memory is dominated by the edge list, at worst
``26 x M`` index pairs; in practice detection runs on a voxel-downsampled cloud
where ``M`` is far below the raw point count.
"""

from __future__ import annotations

import itertools

import numpy as np

from adaptx.config.settings import DetectionSettings
from adaptx.perception.grid import cell_indices, cell_keys


class GridConnectedComponentClusterer:
    """Groups points into clusters by connectivity on a regular grid."""

    name = "grid_connected_components_v1"
    #: True for comparison baselines, so results are never reported as final.
    is_baseline = True

    def __init__(self, settings: DetectionSettings) -> None:
        self._settings = settings

    def cluster(self, points: np.ndarray) -> np.ndarray:
        """Return a cluster label per point.

        Args:
            points: ``(N, 3)`` or ``(N, 4)`` array of finite coordinates.

        Returns:
            ``(N,)`` int array of labels in ``[0, K)``. Labels are dense and
            ordered by each cluster's first appearance in the input.
        """
        count = int(points.shape[0])
        if count == 0:
            return np.empty(0, dtype=np.int64)

        cell = cell_indices(points[:, :3], self._settings.cluster_tolerance_m, stage=self.name)
        # margin=1 reserves a border so stepping to a neighbouring cell can
        # never wrap into an adjacent row of the encoded grid.
        keys, strides = cell_keys(cell, stage=self.name, margin=1)

        occupied, inverse = np.unique(keys, return_inverse=True)
        inverse = inverse.ravel()

        source, target = self._cell_edges(occupied, strides)
        cell_label = _connected_components(source, target, occupied.size)

        return _densify(cell_label[inverse])

    @staticmethod
    def _cell_edges(
        occupied: np.ndarray, strides: tuple[int, ...]
    ) -> tuple[np.ndarray, np.ndarray]:
        """Edges between occupied cells that touch in the 3x3x3 neighbourhood.

        Both directions of every adjacency appear, because each offset and its
        negation are both visited. Label propagation needs that symmetry.
        """
        index = np.arange(occupied.size)
        sources: list[np.ndarray] = []
        targets: list[np.ndarray] = []

        for offset in itertools.product((-1, 0, 1), repeat=3):
            if offset == (0, 0, 0):
                continue
            delta = sum(step * stride for step, stride in zip(offset, strides, strict=True))
            wanted = occupied + delta
            position = np.searchsorted(occupied, wanted)
            np.clip(position, 0, occupied.size - 1, out=position)
            matched = occupied[position] == wanted
            if matched.any():
                sources.append(index[matched])
                targets.append(position[matched])

        if not sources:
            return np.empty(0, dtype=np.int64), np.empty(0, dtype=np.int64)
        return np.concatenate(sources), np.concatenate(targets)


def _connected_components(source: np.ndarray, target: np.ndarray, count: int) -> np.ndarray:
    """Label connected components of an undirected graph, vectorised.

    Each node starts as its own label and repeatedly takes the smallest label
    among its neighbours; pointer jumping (``label = label[label]``) flattens
    the resulting forest each round so the next propagation travels further.
    Labels only ever decrease and are bounded below, so this terminates.

    The edge list is sorted by source **once** and reused, which is what makes
    each round a single ``reduceat`` rather than a re-scan of the neighbourhood.
    """
    labels = np.arange(count, dtype=np.int64)
    if source.size == 0:
        return labels

    order = np.argsort(source, kind="stable")
    source_sorted = source[order]
    target_sorted = target[order]
    group_start = np.flatnonzero(np.concatenate(([True], source_sorted[1:] != source_sorted[:-1])))
    group_source = source_sorted[group_start]

    while True:
        neighbour_min = np.minimum.reduceat(labels[target_sorted], group_start)
        updated = labels.copy()
        updated[group_source] = np.minimum(labels[group_source], neighbour_min)

        while True:
            jumped = updated[updated]
            if np.array_equal(jumped, updated):
                break
            updated = jumped

        if np.array_equal(updated, labels):
            return labels
        labels = updated


def _densify(labels: np.ndarray) -> np.ndarray:
    """Renumber arbitrary labels to ``[0, K)`` by first appearance.

    Ordering by first appearance rather than by raw label value is what makes
    cluster ids stable and readable: cluster 0 is the one whose first point
    comes first in the input.
    """
    if labels.size == 0:
        return labels
    _, first_index, inverse = np.unique(labels, return_index=True, return_inverse=True)
    rank = np.empty(first_index.size, dtype=np.int64)
    rank[np.argsort(first_index, kind="stable")] = np.arange(first_index.size)
    return rank[inverse.ravel()]
