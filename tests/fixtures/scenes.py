"""Deterministic synthetic scenes for object-detection tests.

Every scene is built from explicit geometry — a shell of points on a box, a
plane, a scatter with a fixed seed — so the expected number of clusters and
their approximate dimensions can be reasoned about by hand.

These are test fixtures, not a sensor model. Points sit on perfect surfaces
with no occlusion, no beam pattern and no range-dependent sparsity, so a real
scan looks nothing like this. They exist to exercise clustering, filtering,
geometry and classification against shapes whose answers are known.

Coordinate convention (ADR-009): +x forward, +y left, +z up, metres, origin at
the sensor.
"""

from __future__ import annotations

import numpy as np

from adaptx.models.common import CoordinateFrame, DataSource
from adaptx.models.point_cloud import PointCloudFrame

#: Seed for the few scenes that need scatter. Fixed so runs are identical.
SEED = 20260101

#: Ground height below the sensor, matching the benchmark datasets.
GROUND_Z_M = -1.8


def box_shell(
    centre: tuple[float, float, float],
    size: tuple[float, float, float],
    *,
    spacing_m: float = 0.15,
) -> np.ndarray:
    """Points covering the surface of an axis-aligned box.

    A shell rather than a solid: a LiDAR sees surfaces, and it keeps the point
    count proportional to area rather than volume.

    Sampling is at a fixed **spacing**, not a fixed count per axis, so a large
    box stays dense enough for clustering to connect it. A fixed count would
    make a 40 m wall sparser than the cluster tolerance and split it into
    fragments — which says nothing about the code under test.

    Args:
        centre: Box centre (x, y, z) in metres.
        size: Full extents (length, width, height) in metres.
        spacing_m: Approximate distance between adjacent surface points.

    Returns:
        ``(N, 3)`` array whose extents equal ``size`` exactly.
    """
    length, width, height = size

    def samples(extent: float) -> np.ndarray:
        count = max(2, int(np.ceil(extent / spacing_m)) + 1)
        return np.linspace(-0.5, 0.5, count) * extent

    xs, ys, zs = samples(length), samples(width), samples(height)
    faces: list[np.ndarray] = []

    grid_x, grid_y = np.meshgrid(xs, ys, indexing="ij")
    for sign in (-0.5, 0.5):
        faces.append(
            np.column_stack([grid_x.ravel(), grid_y.ravel(), np.full(grid_x.size, sign * height)])
        )

    grid_x, grid_z = np.meshgrid(xs, zs, indexing="ij")
    for sign in (-0.5, 0.5):
        faces.append(
            np.column_stack([grid_x.ravel(), np.full(grid_x.size, sign * width), grid_z.ravel()])
        )

    grid_y, grid_z = np.meshgrid(ys, zs, indexing="ij")
    for sign in (-0.5, 0.5):
        faces.append(
            np.column_stack([np.full(grid_y.size, sign * length), grid_y.ravel(), grid_z.ravel()])
        )

    return np.vstack(faces) + np.array(centre)


def vehicle(centre: tuple[float, float, float] = (12.0, -3.0, GROUND_Z_M + 0.8)) -> np.ndarray:
    """A vehicle-sized box: 4.5 m x 1.9 m x 1.6 m."""
    return box_shell(centre, (4.5, 1.9, 1.6))


def pedestrian(centre: tuple[float, float, float] = (8.0, 4.0, GROUND_Z_M + 0.9)) -> np.ndarray:
    """A pedestrian-sized box: 0.6 m x 0.5 m x 1.75 m."""
    return box_shell(centre, (0.6, 0.5, 1.75))


def cyclist(centre: tuple[float, float, float] = (15.0, 2.0, GROUND_Z_M + 0.8)) -> np.ndarray:
    """A bicycle-and-rider-sized box: 1.8 m x 0.6 m x 1.7 m."""
    return box_shell(centre, (1.8, 0.6, 1.7))


def low_obstacle(centre: tuple[float, float, float] = (6.0, -6.0, GROUND_Z_M + 0.3)) -> np.ndarray:
    """A low obstacle: 0.8 m x 0.8 m x 0.6 m."""
    return box_shell(centre, (0.8, 0.8, 0.6))


def ambiguous(centre: tuple[float, float, float] = (20.0, 0.0, GROUND_Z_M + 1.0)) -> np.ndarray:
    """Geometry that fits no band: tall, long and wide at once (8 m x 3 m x 2 m)."""
    return box_shell(centre, (8.0, 3.0, 2.0))


def noise_speck(centre: tuple[float, float, float] = (30.0, 10.0, 0.0)) -> np.ndarray:
    """Three points — far below any sensible minimum cluster size."""
    cx, cy, cz = centre
    return np.array([[cx, cy, cz], [cx + 0.02, cy, cz], [cx, cy + 0.02, cz]])


def ground_plane(extent_m: float = 30.0, *, spacing_m: float = 0.3) -> np.ndarray:
    """A dense flat plane at ground height.

    Regular rather than random: a real ground return is a continuous surface,
    and a sparse scatter would fragment into specks that test nothing.
    """
    axis_x = np.arange(-extent_m / 2, extent_m / 2, spacing_m)
    axis_y = np.arange(-extent_m / 3, extent_m / 3, spacing_m)
    grid_x, grid_y = np.meshgrid(axis_x, axis_y, indexing="ij")
    return np.column_stack([grid_x.ravel(), grid_y.ravel(), np.full(grid_x.size, GROUND_Z_M)])


def frame(
    points: np.ndarray, *, frame_id: int = 1, sensor_id: str = "test_lidar"
) -> PointCloudFrame:
    """Wrap points in a validated frame labelled synthetic."""
    return PointCloudFrame(
        frame_id=frame_id,
        sensor_id=sensor_id,
        points=np.ascontiguousarray(points, dtype=np.float64),
        coordinate_frame=CoordinateFrame.EGO,
        source=DataSource.SYNTHETIC_TEST,
    )


def empty_frame() -> PointCloudFrame:
    """A frame with no points at all."""
    return PointCloudFrame(
        frame_id=0,
        sensor_id="test_lidar",
        points=np.empty((0, 3), dtype=np.float64),
        source=DataSource.SYNTHETIC_TEST,
    )
