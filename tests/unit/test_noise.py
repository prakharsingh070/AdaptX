"""Baseline noise / outlier filtering tests (Phase 2B).

The stage counts a point's neighbours over the 3x3x3 block of cells around it
and drops points with too few. Every cloud below is small enough to count
neighbours by hand.
"""

from __future__ import annotations

import numpy as np
import pytest

from adaptx.config.settings import LiDARSettings
from adaptx.core.exceptions import InvalidPointCloudError
from adaptx.perception.noise import NoiseFilter


def noise_filter(cell_size_m: float = 1.0, min_neighbors: int = 2) -> NoiseFilter:
    return NoiseFilter(
        LiDARSettings(
            noise_enabled=True,
            noise_cell_size_m=cell_size_m,
            noise_min_neighbors=min_neighbors,
        )
    )


def array(points: list[list[float]]) -> np.ndarray:
    return np.array(points, dtype=np.float64)


def cluster(count: int, origin: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> list[list[float]]:
    """A tight deterministic clump of points inside a single cell."""
    return [[origin[0] + i * 0.01, origin[1], origin[2]] for i in range(count)]


class TestDegenerateInput:
    def test_empty_cloud(self) -> None:
        mask = noise_filter().keep_mask(np.empty((0, 3), dtype=np.float64))
        assert mask.shape == (0,)
        assert mask.dtype == bool

    def test_single_point_has_no_neighbours(self) -> None:
        assert noise_filter(min_neighbors=1).keep_mask(array([[0.0, 0.0, 0.0]])).tolist() == [False]

    def test_single_point_survives_a_zero_threshold(self) -> None:
        assert noise_filter(min_neighbors=0).keep_mask(array([[0.0, 0.0, 0.0]])).tolist() == [True]


class TestOutlierRemoval:
    def test_isolated_point_is_removed_and_the_cluster_survives(self) -> None:
        points = array([*cluster(5), [50.0, 50.0, 50.0]])
        mask = noise_filter(min_neighbors=2).keep_mask(points)

        assert mask[:5].all()
        assert not mask[5]

    def test_dense_cluster_is_kept_entirely(self) -> None:
        points = array(cluster(10))
        assert noise_filter(min_neighbors=3).keep_mask(points).all()

    def test_several_isolated_points_are_all_removed(self) -> None:
        points = array([*cluster(6), [100.0, 0.0, 0.0], [0.0, 100.0, 0.0], [0.0, 0.0, 100.0]])
        mask = noise_filter(min_neighbors=2).keep_mask(points)

        assert mask[:6].all()
        assert not mask[6:].any()

    def test_a_pair_is_removed_when_the_threshold_exceeds_it(self) -> None:
        """Two points are each other's only neighbour."""
        points = array([[0.0, 0.0, 0.0], [0.05, 0.0, 0.0]])
        assert noise_filter(min_neighbors=1).keep_mask(points).all()
        assert not noise_filter(min_neighbors=2).keep_mask(points).any()


class TestNeighbourhood:
    def test_neighbours_are_found_across_a_cell_boundary(self) -> None:
        """The 3x3x3 block is why a cluster split by a boundary still survives."""
        points = array([[0.99, 0.0, 0.0], [1.01, 0.0, 0.0], [1.02, 0.0, 0.0]])
        assert noise_filter(cell_size_m=1.0, min_neighbors=2).keep_mask(points).all()

    def test_points_beyond_the_block_are_not_neighbours(self) -> None:
        """A point three cells away falls outside the 3x3x3 block."""
        points = array([*cluster(4), [3.5, 0.0, 0.0]])
        assert not noise_filter(cell_size_m=1.0, min_neighbors=2).keep_mask(points)[4]

    def test_negative_coordinates_are_handled(self) -> None:
        points = array([*cluster(4, origin=(-10.0, -10.0, -10.0)), [-50.0, 0.0, 0.0]])
        mask = noise_filter(min_neighbors=2).keep_mask(points)

        assert mask[:4].all()
        assert not mask[4]


class TestConfiguration:
    def test_raising_the_threshold_removes_more(self) -> None:
        points = array([*cluster(4), [20.0, 0.0, 0.0]])

        assert int(noise_filter(min_neighbors=1).keep_mask(points).sum()) == 4
        assert int(noise_filter(min_neighbors=3).keep_mask(points).sum()) == 4
        assert int(noise_filter(min_neighbors=4).keep_mask(points).sum()) == 0

    def test_a_larger_cell_finds_more_neighbours(self) -> None:
        points = array([[0.0, 0.0, 0.0], [5.0, 0.0, 0.0], [10.0, 0.0, 0.0]])

        # 1 m cells: everything is isolated.
        assert not noise_filter(cell_size_m=1.0, min_neighbors=1).keep_mask(points).any()
        # 5 m cells: each point now has a neighbour in the adjacent cell.
        assert noise_filter(cell_size_m=5.0, min_neighbors=1).keep_mask(points).all()

    def test_cell_size_must_be_positive(self) -> None:
        from pydantic import ValidationError

        with pytest.raises(ValidationError):
            LiDARSettings(noise_cell_size_m=0.0)

    def test_threshold_cannot_be_negative(self) -> None:
        from pydantic import ValidationError

        with pytest.raises(ValidationError):
            LiDARSettings(noise_min_neighbors=-1)


class TestGuards:
    def test_an_absurdly_spread_cloud_is_rejected_clearly(self) -> None:
        """Rather than overflow the internal cell encoding, say what to do."""
        points = array([[0.0, 0.0, 0.0], [1e300, 1e300, 1e300]])
        with pytest.raises(InvalidPointCloudError, match="too many grid cells"):
            noise_filter(cell_size_m=0.05).keep_mask(points)

    def test_filter_declares_itself_a_baseline(self) -> None:
        assert noise_filter().is_baseline is True


class TestDeterminism:
    def test_repeated_calls_agree(self) -> None:
        points = array([*cluster(5), [30.0, 0.0, 0.0]])
        stage = noise_filter()
        assert stage.keep_mask(points).tolist() == stage.keep_mask(points).tolist()

    def test_result_is_stable_for_a_larger_seeded_cloud(self) -> None:
        rng = np.random.default_rng(20260101)
        points = rng.uniform(-20.0, 20.0, size=(500, 3))
        stage = noise_filter(cell_size_m=2.0, min_neighbors=2)
        assert np.array_equal(stage.keep_mask(points), stage.keep_mask(points))
