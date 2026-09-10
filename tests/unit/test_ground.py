"""Baseline ground segmentation tests (Phase 2B).

Hand-built synthetic geometry only. The method is a per-cell lowest point plus
a height tolerance, so every expectation below is arithmetic on the cell
minimum.

Convention: +z is up (ADR-009). The sensor sits at the origin, so a road under
the vehicle has negative z.
"""

from __future__ import annotations

import numpy as np
import pytest

from adaptx.config.settings import LiDARSettings
from adaptx.perception.ground import GroundSegmenter


def segmenter(**overrides: float | None) -> GroundSegmenter:
    base: dict[str, float | bool | None] = {
        "ground_enabled": True,
        "ground_cell_size_m": 1.0,
        "ground_height_tolerance_m": 0.2,
    }
    base.update(overrides)
    return GroundSegmenter(LiDARSettings(**base))


def array(points: list[list[float]]) -> np.ndarray:
    return np.array(points, dtype=np.float64)


class TestDegenerateInput:
    def test_empty_cloud(self) -> None:
        mask = segmenter().ground_mask(np.empty((0, 3), dtype=np.float64))
        assert mask.shape == (0,)
        assert mask.dtype == bool

    def test_single_point_is_its_own_ground(self) -> None:
        """With nothing lower in its cell, one point is the local ground."""
        assert segmenter().ground_mask(array([[0.0, 0.0, -1.8]])).tolist() == [True]

    def test_all_points_coplanar_are_all_ground(self) -> None:
        points = array([[float(i), 0.0, -1.8] for i in range(5)])
        assert segmenter().ground_mask(points).all()


class TestFlatGround:
    def test_flat_road_is_ground_and_an_object_is_not(self) -> None:
        points = array(
            [
                [0.2, 0.2, -1.80],  # road
                [0.4, 0.4, -1.75],  # road, within tolerance
                [0.6, 0.6, -0.50],  # object well above
            ]
        )
        assert segmenter().ground_mask(points).tolist() == [True, True, False]

    def test_elevated_object_across_several_cells(self) -> None:
        road = [[float(x) + 0.5, 0.5, -1.8] for x in range(4)]
        box = [[float(x) + 0.5, 0.5, -0.3] for x in range(4)]
        mask = segmenter().ground_mask(array(road + box))

        assert mask[:4].all()
        assert not mask[4:].any()


class TestTolerance:
    def test_point_exactly_at_the_tolerance_is_ground(self) -> None:
        """The bound is inclusive: floor + tolerance still counts as ground."""
        points = array([[0.5, 0.5, -2.0], [0.5, 0.5, -1.8]])
        assert segmenter(ground_height_tolerance_m=0.2).ground_mask(points).tolist() == [
            True,
            True,
        ]

    def test_point_just_above_the_tolerance_is_not_ground(self) -> None:
        points = array([[0.5, 0.5, -2.0], [0.5, 0.5, -1.79]])
        assert segmenter(ground_height_tolerance_m=0.2).ground_mask(points).tolist() == [
            True,
            False,
        ]

    def test_zero_tolerance_keeps_only_the_lowest_point(self) -> None:
        points = array([[0.5, 0.5, -2.0], [0.5, 0.5, -1.99]])
        assert segmenter(ground_height_tolerance_m=0.0).ground_mask(points).tolist() == [
            True,
            False,
        ]

    def test_a_large_tolerance_swallows_everything(self) -> None:
        points = array([[0.5, 0.5, -2.0], [0.5, 0.5, 3.0]])
        assert segmenter(ground_height_tolerance_m=10.0).ground_mask(points).all()


class TestCellLocality:
    def test_each_cell_finds_its_own_ground_level(self) -> None:
        """A point high in one cell can still be ground in a cell that is higher."""
        points = array(
            [
                [0.5, 0.5, -2.0],  # cell (0,0) floor
                [1.5, 0.5, -1.0],  # cell (1,0) floor - higher, still ground
            ]
        )
        assert segmenter().ground_mask(points).tolist() == [True, True]

    def test_a_slope_is_followed_across_cells(self) -> None:
        """A global height threshold would fail this; per-cell minima do not."""
        points = array([[float(x) + 0.5, 0.5, -2.0 + 0.5 * x] for x in range(6)])
        assert segmenter().ground_mask(points).all()

    def test_cell_size_changes_the_grouping(self) -> None:
        # Two points 1 m apart in x, differing 1 m in height.
        points = array([[0.5, 0.5, -2.0], [1.5, 0.5, -1.0]])

        # Separate cells: each is its own ground.
        assert segmenter(ground_cell_size_m=1.0).ground_mask(points).tolist() == [True, True]
        # One shared cell: the higher point is now an object above the floor.
        assert segmenter(ground_cell_size_m=10.0).ground_mask(points).tolist() == [True, False]

    def test_cell_size_must_be_positive(self) -> None:
        from pydantic import ValidationError

        with pytest.raises(ValidationError):
            LiDARSettings(ground_cell_size_m=0.0)


class TestOptionalCeiling:
    def test_ceiling_disabled_by_default(self) -> None:
        """Without a ceiling, an elevated surface becomes its own ground."""
        points = array([[0.5, 0.5, 2.0], [0.5, 0.5, 2.1]])
        assert LiDARSettings().ground_max_height_m is None
        assert segmenter().ground_mask(points).tolist() == [True, True]

    def test_ceiling_rejects_a_cell_whose_floor_is_too_high(self) -> None:
        """This is the documented failure mode the ceiling exists to limit."""
        points = array([[0.5, 0.5, 2.0], [0.5, 0.5, 2.1]])
        mask = segmenter(ground_max_height_m=0.0).ground_mask(points)
        assert not mask.any()

    def test_ceiling_keeps_a_genuine_low_cell(self) -> None:
        points = array([[0.5, 0.5, -1.8], [0.5, 0.5, 2.0]])
        assert segmenter(ground_max_height_m=0.0).ground_mask(points).tolist() == [True, False]


class TestKnownLimitations:
    def test_object_only_cell_misclassifies_its_lowest_point(self) -> None:
        """Documented weakness: with no road return beneath it, a car roof
        becomes the local ground. Asserted so the limitation stays visible."""
        points = array([[0.5, 0.5, 0.0], [0.5, 0.5, 1.0]])
        assert segmenter().ground_mask(points).tolist() == [True, False]

    def test_segmenter_declares_itself_a_baseline(self) -> None:
        assert segmenter().is_baseline is True
        assert "v1" in segmenter().name


class TestDeterminism:
    def test_repeated_calls_agree(self) -> None:
        points = array([[0.5, 0.5, -2.0], [0.5, 0.5, -1.0], [1.5, 1.5, -1.9]])
        stage = segmenter()
        assert stage.ground_mask(points).tolist() == stage.ground_mask(points).tolist()

    def test_result_is_stable_for_a_larger_seeded_cloud(self) -> None:
        rng = np.random.default_rng(20260101)
        points = rng.uniform(-10.0, 10.0, size=(400, 3))
        stage = segmenter()
        assert np.array_equal(stage.ground_mask(points), stage.ground_mask(points))
