"""Voxel downsampling tests (Phase 2B).

Every cloud is hand-written and small enough that the expected survivor of each
voxel can be worked out by hand. Nothing is random.

Convention under test: the grid is anchored at the origin, a voxel index is
``floor(p / voxel_size)``, and the point kept per voxel is the real measured
point nearest that voxel's centroid, ties broken by lower input index.
"""

from __future__ import annotations

import numpy as np
import pytest

from adaptx.config.settings import LiDARSettings
from adaptx.perception.voxel import VoxelDownsampler


def downsampler(voxel_size_m: float = 1.0) -> VoxelDownsampler:
    return VoxelDownsampler(LiDARSettings(voxel_size_m=voxel_size_m, voxel_enabled=True))


def array(points: list[list[float]]) -> np.ndarray:
    return np.array(points, dtype=np.float64)


class TestDegenerateInput:
    def test_empty_cloud(self) -> None:
        kept = downsampler().select(np.empty((0, 3), dtype=np.float64))
        assert kept.shape == (0,)

    def test_single_point_is_kept(self) -> None:
        kept = downsampler().select(array([[1.0, 2.0, 3.0]]))
        assert kept.tolist() == [0]

    def test_all_points_identical_collapse_to_one(self) -> None:
        points = array([[1.0, 1.0, 1.0]] * 5)
        assert downsampler().select(points).shape == (1,)


class TestVoxelGrouping:
    def test_points_in_the_same_voxel_collapse_to_one(self) -> None:
        # All within the voxel spanning [0,1) on every axis.
        points = array([[0.1, 0.1, 0.1], [0.2, 0.2, 0.2], [0.9, 0.9, 0.9]])
        assert downsampler(1.0).select(points).shape == (1,)

    def test_points_in_different_voxels_are_all_kept(self) -> None:
        points = array([[0.5, 0.5, 0.5], [1.5, 0.5, 0.5], [0.5, 1.5, 0.5], [0.5, 0.5, 1.5]])
        assert downsampler(1.0).select(points).tolist() == [0, 1, 2, 3]

    def test_separation_on_each_axis_independently(self) -> None:
        for axis in range(3):
            far = [0.5, 0.5, 0.5]
            far[axis] = 5.5
            points = array([[0.5, 0.5, 0.5], far])
            assert downsampler(1.0).select(points).shape == (2,), f"axis {axis}"

    def test_negative_coordinates_group_correctly(self) -> None:
        # floor(-0.5) == -1 and floor(-1.5) == -2, so these are different voxels.
        points = array([[-0.5, 0.0, 0.0], [-1.5, 0.0, 0.0]])
        assert downsampler(1.0).select(points).shape == (2,)

    def test_points_straddling_zero_are_separated(self) -> None:
        """floor puts -0.1 in voxel -1 and +0.1 in voxel 0."""
        points = array([[-0.1, 0.0, 0.0], [0.1, 0.0, 0.0]])
        assert downsampler(1.0).select(points).shape == (2,)


class TestBoundaryBehaviour:
    def test_point_exactly_on_a_boundary_belongs_to_the_upper_voxel(self) -> None:
        """floor(1.0 / 1.0) == 1, so 1.0 sits with 1.5, not with 0.5."""
        points = array([[0.5, 0.0, 0.0], [1.0, 0.0, 0.0], [1.5, 0.0, 0.0]])
        kept = downsampler(1.0).select(points)

        assert kept.shape == (2,)
        assert 0 in kept.tolist()  # the lone occupant of voxel 0

    def test_origin_anchored_grid_not_bbox_anchored(self) -> None:
        """Shifting the cloud changes voxel membership; a bbox grid would not."""
        near_origin = array([[0.4, 0.0, 0.0], [0.6, 0.0, 0.0]])
        straddling = array([[0.9, 0.0, 0.0], [1.1, 0.0, 0.0]])

        assert downsampler(1.0).select(near_origin).shape == (1,)
        assert downsampler(1.0).select(straddling).shape == (2,)


class TestRepresentativePoint:
    def test_kept_point_is_a_real_input_point(self) -> None:
        """The centroid is never emitted: no coordinate is manufactured."""
        points = array([[0.1, 0.1, 0.1], [0.9, 0.9, 0.9]])
        kept = downsampler(1.0).select(points)

        assert kept.shape == (1,)
        assert points[kept][0].tolist() in points.tolist()

    def test_kept_point_is_the_one_nearest_the_centroid(self) -> None:
        # Centroid of x = 0.1, 0.5, 0.9 is 0.5, so index 1 wins.
        points = array([[0.1, 0.0, 0.0], [0.5, 0.0, 0.0], [0.9, 0.0, 0.0]])
        assert downsampler(1.0).select(points).tolist() == [1]

    def test_ties_are_broken_by_lower_input_index(self) -> None:
        # 0.25 and 0.75 are exact in binary floating point, so the centroid is
        # exactly 0.5 and both points are exactly 0.25 away - a genuine tie.
        points = array([[0.75, 0.0, 0.0], [0.25, 0.0, 0.0]])
        assert downsampler(1.0).select(points).tolist() == [0]

    def test_intensity_of_the_kept_point_is_untouched(self) -> None:
        """Intensity is carried, never averaged: it belongs to the kept return."""
        # Centroid x = 0.5, so the middle point wins outright.
        points = array([[0.25, 0.0, 0.0, 0.9], [0.5, 0.0, 0.0, 0.1], [0.75, 0.0, 0.0, 0.4]])
        kept = downsampler(1.0).select(points)

        assert kept.tolist() == [1]
        assert points[kept][0, 3] == 0.1

    def test_indices_are_returned_in_ascending_order(self) -> None:
        points = array([[5.5, 0.0, 0.0], [0.5, 0.0, 0.0], [2.5, 0.0, 0.0]])
        kept = downsampler(1.0).select(points)
        assert kept.tolist() == sorted(kept.tolist())


class TestVoxelSize:
    @pytest.mark.parametrize("size", [0.05, 0.10, 0.20, 0.50])
    def test_documented_sizes_are_accepted(self, size: float) -> None:
        points = array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        assert downsampler(size).select(points).shape == (2,)

    def test_a_larger_voxel_keeps_fewer_points(self) -> None:
        points = array([[float(i) * 0.1, 0.0, 0.0] for i in range(10)])

        fine = downsampler(0.05).select(points).shape[0]
        coarse = downsampler(0.5).select(points).shape[0]
        very_coarse = downsampler(10.0).select(points).shape[0]

        assert fine == 10
        assert coarse == 2
        assert very_coarse == 1
        assert fine > coarse > very_coarse

    def test_voxel_size_must_be_positive(self) -> None:
        from pydantic import ValidationError

        with pytest.raises(ValidationError):
            LiDARSettings(voxel_size_m=0.0)


class TestDeterminism:
    def test_repeated_calls_agree(self) -> None:
        points = array([[0.1, 0.2, 0.3], [0.4, 0.5, 0.6], [3.1, 3.2, 3.3], [3.4, 3.5, 3.6]])
        stage = downsampler(1.0)
        assert stage.select(points).tolist() == stage.select(points).tolist()

    def test_two_instances_agree(self) -> None:
        points = array([[0.1, 0.0, 0.0], [0.6, 0.0, 0.0], [2.2, 0.0, 0.0]])
        assert downsampler(1.0).select(points).tolist() == downsampler(1.0).select(points).tolist()

    def test_result_is_stable_for_a_larger_seeded_cloud(self) -> None:
        """Seeded only to get a repeatable cloud; the assertion is self-consistency."""
        rng = np.random.default_rng(20260101)
        points = rng.uniform(-5.0, 5.0, size=(500, 3))
        stage = downsampler(0.5)
        assert np.array_equal(stage.select(points), stage.select(points))
