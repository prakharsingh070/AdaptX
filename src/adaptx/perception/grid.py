"""Shared grid quantisation for the Phase 2B stages.

Voxel downsampling, ground segmentation and noise filtering all begin by
mapping coordinates onto a regular integer grid anchored at the coordinate
frame origin. That single step is factored out here so the overflow guard
exists once rather than three times.

Why the guard matters: ``np.floor(x / size).astype(np.int64)`` does **not**
raise when the quotient exceeds the int64 range. NumPy emits a runtime warning
and produces ``INT64_MIN``, which would silently place far-apart points in the
same cell - a wrong answer that looks like a right one. ADAPT-X does not hide
errors (``docs/knowledge-base/20-constraints.md``), so an input that cannot be
quantised is rejected with an explanation instead.

In the pipeline this is unreachable, because ROI and range filtering bound the
coordinates first. It is reachable when a stage is used on its own.
"""

from __future__ import annotations

import numpy as np

from adaptx.core.exceptions import InvalidPointCloudError

#: Largest magnitude a cell index may take before int64 quantisation is unsafe.
MAX_CELL_INDEX = 2.0**62


def cell_indices(coords: np.ndarray, cell_size_m: float, *, stage: str) -> np.ndarray:
    """Quantise ``coords`` to integer grid indices anchored at the origin.

    Args:
        coords: ``(N, D)`` array of finite coordinates in metres.
        cell_size_m: Positive cell edge length.
        stage: Stage name, used in the error message.

    Returns:
        ``(N, D)`` int64 array of cell indices, ``floor(coord / cell_size_m)``.

    Raises:
        InvalidPointCloudError: a coordinate is so large relative to the cell
            size that the index would overflow int64.
    """
    scaled = np.floor(coords / cell_size_m)
    if scaled.size:
        # One pass covers both faults. `magnitude` is NaN if any coordinate was
        # NaN, and the negated comparison catches that as well as an overflow,
        # because every comparison against NaN is False.
        magnitude = np.abs(scaled).max()
        if not magnitude <= MAX_CELL_INDEX:
            raise InvalidPointCloudError(
                f"{stage}: point cloud spans too many grid cells at cell size "
                f"{cell_size_m} m, or contains a non-finite coordinate; restrict it "
                "by range or ROI first, or use a larger cell",
                details={
                    "stage": stage,
                    "cell_size_m": cell_size_m,
                    "max_cell_index": float(magnitude),
                },
            )
    return scaled.astype(np.int64)


def cell_keys(
    indices: np.ndarray, *, stage: str, margin: int = 0
) -> tuple[np.ndarray, tuple[int, ...]]:
    """Encode integer cell indices as unique 1D int64 keys.

    Grouping points by cell needs ``np.unique``. Called on a ``(N, D)`` array
    with ``axis=0`` that sorts *rows*, which profiling showed to dominate the
    pipeline - about 60% of its runtime at 100k points. Collapsing each cell to
    one integer first lets the scalar ``np.unique`` do the work instead, which
    is several times faster for the same result.

    The encoding is affine, so stepping one cell along an axis is a constant
    shift of the key. Callers that need neighbouring cells (the noise filter)
    use that: pass ``margin=1`` to reserve a one-cell border, which guarantees a
    +/-1 step can never wrap into an adjacent row of the grid.

    Args:
        indices: ``(N, D)`` int64 cell indices from :func:`cell_indices`.
        stage: Stage name, used in the error message.
        margin: Cells of empty border to reserve on every side.

    Returns:
        ``(keys, strides)`` - the ``(N,)`` key array, and the per-axis stride
        used to encode it, so a caller can compute neighbour offsets.

    Raises:
        InvalidPointCloudError: the grid spans more cells than an int64 key can
            distinguish.
    """
    shifted = indices - indices.min(axis=0) + margin
    dims = shifted.max(axis=0) + 1 + margin

    span = 1.0
    for size in dims:
        span *= float(size)
    if span > MAX_CELL_INDEX:
        raise InvalidPointCloudError(
            f"{stage}: point cloud spans too many grid cells; restrict it by range "
            "or ROI first, or use a larger cell size",
            details={"stage": stage, "grid_dimensions": [int(d) for d in dims]},
        )

    strides: list[int] = []
    stride = 1
    for size in dims:
        strides.append(stride)
        stride *= int(size)

    keys = np.zeros(shifted.shape[0], dtype=np.int64)
    for axis, axis_stride in enumerate(strides):
        keys += shifted[:, axis] * axis_stride
    return keys, tuple(strides)
