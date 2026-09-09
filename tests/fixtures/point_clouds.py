"""Deterministic point-cloud builders for tests.

These produce synthetic geometry for exercising contracts and endpoints. They
are test scaffolding: every frame is labelled
:attr:`~adaptx.models.common.DataSource.SYNTHETIC_TEST` and must never be used
to stand in for sensor data.
"""

from __future__ import annotations

import numpy as np

from adaptx.models.common import CoordinateFrame, DataSource
from adaptx.models.point_cloud import PointCloudFrame

#: Fixed seed so generated clouds are identical on every run.
SEED = 20260101


def make_points(count: int = 100, *, columns: int = 3, seed: int = SEED) -> np.ndarray:
    """Return a reproducible ``(count, columns)`` float array in a 20 m box."""
    rng = np.random.default_rng(seed)
    return rng.uniform(-10.0, 10.0, size=(count, columns))


def make_frame(
    count: int = 100,
    *,
    frame_id: int = 0,
    columns: int = 3,
    source: DataSource = DataSource.SYNTHETIC_TEST,
    sensor_id: str = "test_lidar",
) -> PointCloudFrame:
    """Return a reproducible synthetic :class:`PointCloudFrame`."""
    return PointCloudFrame(
        frame_id=frame_id,
        sensor_id=sensor_id,
        points=make_points(count, columns=columns),
        fields=("x", "y", "z", "intensity") if columns == 4 else ("x", "y", "z"),
        coordinate_frame=CoordinateFrame.LIDAR,
        source=source,
    )


def frame_request_body(
    count: int = 10,
    *,
    frame_id: int = 0,
    source: DataSource = DataSource.SYNTHETIC_TEST,
) -> dict[str, object]:
    """Return a JSON body for ``POST /api/v1/lidar/frame``."""
    return {
        "frame_id": frame_id,
        "sensor_id": "test_lidar",
        "source": source.value,
        "points": make_points(count).tolist(),
        "coordinate_frame": CoordinateFrame.LIDAR.value,
    }
