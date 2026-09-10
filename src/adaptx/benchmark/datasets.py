"""Deterministic synthetic point clouds for benchmarking and sanity checks.

**Everything in this module is synthetic.** No real LiDAR recording is used,
and every frame produced here is labelled
:attr:`~adaptx.models.common.DataSource.SYNTHETIC_TEST`. Results measured on
this data describe how fast the code runs on *these shapes*; they say nothing
about accuracy, and nothing about real-world autonomous-driving performance.

The geometry is a crude stand-in for a road scene - a flat plane, some boxes,
a little sparse clutter - not a sensor model. It has no beam divergence, no
incidence-angle falloff, no occlusion, no ring structure and no intensity
physics. It exists so the pipeline has structure to work on: a ground plane for
segmentation to find, dense surfaces for voxelisation to reduce, and isolated
returns for the noise filter to catch.

Reproducibility: every scenario is generated from a fixed seed with
:func:`numpy.random.default_rng`, so a given (scenario, seed) pair always
produces the identical array. Swap in a recorded dataset here when one exists.
"""

from __future__ import annotations

from enum import StrEnum

import numpy as np
from pydantic import Field

from adaptx.models.common import AdaptXModel, CoordinateFrame, DataSource
from adaptx.models.point_cloud import RawPointCloudFrame

#: Default seed. Fixed so benchmark runs are comparable across machines and days.
DEFAULT_SEED = 20260101

#: Nominal height of the synthetic ground plane below the sensor, in metres.
GROUND_Z_M = -1.8


class DatasetScenario(StrEnum):
    """A named synthetic dataset.

    The first three are the size ladder for performance measurement. The rest
    are shaped to exercise a specific edge case rather than to be timed.
    """

    SMALL = "small"
    MEDIUM = "medium"
    LARGE = "large"

    EMPTY = "empty"
    DENSE = "dense"
    EXTREME_COORDINATES = "extreme_coordinates"
    ALL_GROUND = "all_ground"
    NO_GROUND = "no_ground"
    NOISY = "noisy"


#: Scenarios that form the performance size ladder.
SIZE_LADDER: tuple[DatasetScenario, ...] = (
    DatasetScenario.SMALL,
    DatasetScenario.MEDIUM,
    DatasetScenario.LARGE,
)

#: Scenarios that exist to probe edge-case behaviour, not to be timed.
EDGE_CASES: tuple[DatasetScenario, ...] = (
    DatasetScenario.EMPTY,
    DatasetScenario.DENSE,
    DatasetScenario.EXTREME_COORDINATES,
    DatasetScenario.ALL_GROUND,
    DatasetScenario.NO_GROUND,
    DatasetScenario.NOISY,
)


class DatasetDescription(AdaptXModel):
    """What a generated dataset contains, recorded alongside its results."""

    scenario: DatasetScenario
    seed: int
    point_count: int = Field(ge=0)
    description: str
    #: Always true here. Present so a report can never omit the distinction.
    synthetic: bool = True
    ground_truth_available: bool = Field(
        default=False,
        description=(
            "Whether labels exist to score accuracy against. False for every "
            "scenario here: these measure speed and behaviour, not accuracy."
        ),
    )


_DESCRIPTIONS: dict[DatasetScenario, str] = {
    DatasetScenario.SMALL: (
        "Road plane, three boxes, sparse clutter and a few non-returns. ~10k points."
    ),
    DatasetScenario.MEDIUM: (
        "Same composition at ~100k points, the order of a single automotive scan."
    ),
    DatasetScenario.LARGE: ("Same composition at ~400k points, a dense or multi-sensor frame."),
    DatasetScenario.EMPTY: "No points at all.",
    DatasetScenario.DENSE: (
        "50k points packed into a 1 m cube, so one voxel holds almost everything."
    ),
    DatasetScenario.EXTREME_COORDINATES: (
        "Points at the edge of and far beyond the configured range and ROI."
    ),
    DatasetScenario.ALL_GROUND: "A single flat plane and nothing above it.",
    DatasetScenario.NO_GROUND: (
        "A raised slab with no return beneath it; the lowest points are not road."
    ),
    DatasetScenario.NOISY: ("A road plane plus 20% isolated returns scattered through the volume."),
}

_SIZES: dict[DatasetScenario, int] = {
    DatasetScenario.SMALL: 10_000,
    DatasetScenario.MEDIUM: 100_000,
    DatasetScenario.LARGE: 400_000,
}


def _road_scene(rng: np.random.Generator, count: int) -> np.ndarray:
    """A flat road with boxes on it, sparse clutter and some non-returns."""
    ground_count = int(count * 0.70)
    box_count = int(count * 0.22)
    clutter_count = int(count * 0.06)
    invalid_count = count - ground_count - box_count - clutter_count

    ground = np.empty((ground_count, 3))
    ground[:, 0] = rng.uniform(-40.0, 70.0, ground_count)
    ground[:, 1] = rng.uniform(-30.0, 30.0, ground_count)
    # A little roughness so the plane is not perfectly degenerate.
    ground[:, 2] = GROUND_Z_M + rng.normal(0.0, 0.02, ground_count)

    # Three boxes standing on the road, at different distances.
    centres = np.array([[12.0, -3.0], [25.0, 4.0], [45.0, -8.0]])
    assignment = rng.integers(0, len(centres), box_count)
    boxes = np.empty((box_count, 3))
    boxes[:, :2] = centres[assignment] + rng.uniform(-1.0, 1.0, (box_count, 2))
    boxes[:, 2] = GROUND_Z_M + rng.uniform(0.1, 1.8, box_count)

    clutter = np.empty((clutter_count, 3))
    clutter[:, 0] = rng.uniform(-40.0, 70.0, clutter_count)
    clutter[:, 1] = rng.uniform(-30.0, 30.0, clutter_count)
    clutter[:, 2] = rng.uniform(GROUND_Z_M, 3.0, clutter_count)

    invalid = np.full((invalid_count, 3), np.nan)

    return np.vstack([ground, boxes, clutter, invalid])


def _generate_points(scenario: DatasetScenario, rng: np.random.Generator) -> np.ndarray:
    if scenario in _SIZES:
        return _road_scene(rng, _SIZES[scenario])

    if scenario is DatasetScenario.EMPTY:
        return np.empty((0, 3), dtype=np.float64)

    if scenario is DatasetScenario.DENSE:
        return rng.uniform(0.0, 1.0, (50_000, 3)) + np.array([10.0, 0.0, GROUND_Z_M])

    if scenario is DatasetScenario.EXTREME_COORDINATES:
        return np.array(
            [
                [0.5, 0.0, 0.0],  # exactly at the default minimum range
                [100.0, 0.0, 0.0],  # exactly at the default maximum range
                [0.0, 0.0, 0.0],  # the sensor origin itself
                [1.0e6, 1.0e6, 1.0e6],  # far outside anything configured
                [-1.0e6, 0.0, 0.0],
                [80.0, 0.0, 0.0],  # exactly on the default ROI +x face
                [-50.0, 0.0, 0.0],  # exactly on the default ROI -x face
                [1.0e-9, 1.0e-9, 1.0e-9],  # sub-millimetre, near the origin
            ]
        )

    if scenario is DatasetScenario.ALL_GROUND:
        points = np.empty((20_000, 3))
        points[:, 0] = rng.uniform(-30.0, 60.0, 20_000)
        points[:, 1] = rng.uniform(-25.0, 25.0, 20_000)
        points[:, 2] = GROUND_Z_M + rng.normal(0.0, 0.01, 20_000)
        return points

    if scenario is DatasetScenario.NO_GROUND:
        # A slab well above the road, with nothing returned beneath it.
        points = np.empty((20_000, 3))
        points[:, 0] = rng.uniform(10.0, 20.0, 20_000)
        points[:, 1] = rng.uniform(-5.0, 5.0, 20_000)
        points[:, 2] = rng.uniform(1.0, 2.0, 20_000)
        return points

    if scenario is DatasetScenario.NOISY:
        clean = _road_scene(rng, 40_000)
        noise = np.empty((10_000, 3))
        noise[:, 0] = rng.uniform(-40.0, 70.0, 10_000)
        noise[:, 1] = rng.uniform(-30.0, 30.0, 10_000)
        noise[:, 2] = rng.uniform(-2.0, 4.0, 10_000)
        return np.vstack([clean, noise])

    raise ValueError(f"unknown scenario: {scenario}")  # pragma: no cover


def generate_dataset(
    scenario: DatasetScenario, *, seed: int = DEFAULT_SEED, frame_id: int = 0
) -> tuple[RawPointCloudFrame, DatasetDescription]:
    """Build a synthetic frame and the description that must travel with it.

    Args:
        scenario: Which named dataset to build.
        seed: Generator seed. The same (scenario, seed) always yields the same
            array.
        frame_id: Frame identifier to stamp on the produced frame.

    Returns:
        The frame - always labelled ``SYNTHETIC_TEST`` - and its description.
    """
    rng = np.random.default_rng(seed)
    points = _generate_points(scenario, rng)

    frame = RawPointCloudFrame(
        frame_id=frame_id,
        sensor_id=f"synthetic_{scenario.value}",
        points=points,
        coordinate_frame=CoordinateFrame.LIDAR,
        source=DataSource.SYNTHETIC_TEST,
    )
    description = DatasetDescription(
        scenario=scenario,
        seed=seed,
        point_count=frame.point_count,
        description=_DESCRIPTIONS[scenario],
    )
    return frame, description
