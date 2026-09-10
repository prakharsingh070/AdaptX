"""Baseline ground segmentation (Phase 2B).

**This is a baseline, not production ground segmentation**, and nothing in
ADAPT-X should be described as production-grade for autonomous driving on the
strength of it. Its failure modes are listed below rather than hidden.

Method: local lowest point per xy cell
--------------------------------------
The xy plane is divided into square cells of ``ground_cell_size_m``. Within a
cell, the lowest point is taken as the local ground level, and every point
within ``ground_height_tolerance_m`` above that level is classified as ground.

Chosen over a single global height threshold because a global threshold assumes
both a flat world and a known sensor mount height: on any incline it either
keeps a wall of ground points or erases the road. Working per cell means the
ground level is *discovered* locally, so a slope is handled and **no sensor
mount height is required** - which is also why Phase 2B needs no lidar-to-ego
coordinate transform (ADR-013).

The only frame assumption is that **+z is up** (ADR-009), i.e. the sensor is
mounted roughly level. A significantly rolled or pitched mount would need a
real transform first; that stage does not exist and is not approximated here.

Known failure modes
-------------------
* A cell containing only object returns - a car roof with no road visible
  beneath it - has its lowest points classified as ground. The optional
  ``ground_max_height_m`` ceiling limits this when the mount height is known;
  it is disabled by default because it reintroduces the mount assumption.
* A cell spanning a kerb or a steep slope keeps the higher side as non-ground.
* Overhanging structure (a bridge, a tunnel roof) is non-ground, correctly, but
  only because it is far above the cell minimum.
* Cell size trades off against slope: larger cells are more robust to sparse
  returns but less able to follow a gradient.
"""

from __future__ import annotations

import numpy as np

from adaptx.config.settings import LiDARSettings
from adaptx.perception.grid import cell_indices, cell_keys


class GroundSegmenter:
    """Classifies points as ground or non-ground. Baseline implementation."""

    name = "ground_lowest_point_v1"
    #: True for comparison baselines, so results are never reported as final.
    is_baseline = True

    def __init__(self, settings: LiDARSettings) -> None:
        self._settings = settings

    def ground_mask(self, points: np.ndarray) -> np.ndarray:
        """Return a boolean mask that is True for ground points.

        Args:
            points: ``(N, 3)`` or ``(N, 4)`` array of finite coordinates.

        Returns:
            ``(N,)`` boolean array; True means the point was classified as
            ground.
        """
        settings = self._settings
        count = int(points.shape[0])
        if count == 0:
            return np.zeros(0, dtype=bool)

        xy = points[:, :2]
        z = points[:, 2]

        cell_index = cell_indices(xy, settings.ground_cell_size_m, stage=self.name)
        keys, _ = cell_keys(cell_index, stage=self.name)
        _, inverse = np.unique(keys, return_inverse=True)
        inverse = inverse.ravel()
        cell_count = int(inverse.max()) + 1

        # Lowest z per cell, without a Python loop and without ufunc.at, which
        # is markedly slower: sort by (cell, z) and take the first row of each
        # cell group.
        order = np.lexsort((z, inverse))
        grouped = inverse[order]
        is_first = np.empty(count, dtype=bool)
        is_first[0] = True
        np.not_equal(grouped[1:], grouped[:-1], out=is_first[1:])

        cell_floor = np.empty(cell_count, dtype=np.float64)
        cell_floor[grouped[is_first]] = z[order[is_first]]

        floor_per_point = cell_floor[inverse]
        mask = z <= floor_per_point + settings.ground_height_tolerance_m

        if settings.ground_max_height_m is not None:
            # A cell whose lowest point is already above the ceiling cannot be
            # showing ground at all, so nothing in it is ground.
            mask &= floor_per_point <= settings.ground_max_height_m

        return mask
