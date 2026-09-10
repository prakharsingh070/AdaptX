"""Tiled adaptive mapper tests (Phase 8).

Every case has explicit bounds, an explicit tiling, an explicit plan and
explicit point coordinates, so the cell a point lands in is arithmetic rather
than a guess. Nothing here is random.

Frames are constructed directly rather than produced by the Phase 2 pipeline,
and plans are built by driving the real controller with hand-made assessments:
these tests exercise the mapper, and routing points through preprocessing would
make a mapping failure indistinguishable from a filtering one.

The map used throughout is 40 m square with 10 m regions, so 4x4 = 16 tiles.
"""

from __future__ import annotations

import math
from datetime import UTC, datetime

import numpy as np
import pytest

from adaptx.config.settings import AdaptiveResolutionSettings, MapSettings
from adaptx.core.exceptions import InvalidPointCloudError
from adaptx.mapping.adaptive_mapper import TiledAdaptiveMapper, build_adaptive_mapper
from adaptx.mapping.comparison import compare, partition_by_priority
from adaptx.mapping.controller import HeuristicResolutionController
from adaptx.mapping.grid_mapper import FixedResolutionMapper
from adaptx.models.adaptive_resolution import ResolutionPlan
from adaptx.models.common import CoordinateFrame, DataSource
from adaptx.models.map import OccupancyState, ResolutionLevel
from adaptx.models.point_cloud import PointCloudFrame
from tests.fixtures.assessments import assessed, at_position

EPOCH = datetime(2026, 1, 1, 12, 0, 0, tzinfo=UTC)

UNIT_MAP = {
    "min_x_m": -20.0,
    "max_x_m": 20.0,
    "min_y_m": -20.0,
    "max_y_m": 20.0,
    "resolution_m": 0.5,
}


def settings(**overrides: object) -> tuple[MapSettings, AdaptiveResolutionSettings]:
    """Map and adaptive settings over the small unit map."""
    map_overrides = {key: overrides.pop(key) for key in list(overrides) if key in UNIT_MAP}
    return (
        MapSettings(**{**UNIT_MAP, **map_overrides}),  # type: ignore[arg-type]
        AdaptiveResolutionSettings(**overrides),  # type: ignore[arg-type]
    )


def mapper(**overrides: object) -> TiledAdaptiveMapper:
    """A mapper over the small unit map unless overridden."""
    return build_adaptive_mapper(*settings(**overrides))


def controller(**overrides: object) -> HeuristicResolutionController:
    """A controller over the same map, so plans and maps always agree."""
    return HeuristicResolutionController(*settings(**overrides))


def frame(points: list[list[float]], *, frame_id: int = 0) -> PointCloudFrame:
    """A processed frame holding exactly these points."""
    array = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    return PointCloudFrame(
        timestamp=EPOCH,
        frame_id=frame_id,
        sensor_id="roof_lidar",
        points=array,
        coordinate_frame=CoordinateFrame.EGO,
        source=DataSource.SYNTHETIC_TEST,
    )


def empty_frame() -> PointCloudFrame:
    """A frame with no points at all."""
    return frame([])


def uniform_plan(
    level: ResolutionLevel = ResolutionLevel.LOW, **overrides: object
) -> ResolutionPlan:
    """A plan holding every region at one level, for arithmetic that is easy to check."""
    instance = controller(base_level=level, enabled=False, **overrides)
    return instance.plan([])


def mixed_plan(**overrides: object) -> ResolutionPlan:
    """A plan with a genuinely refined region beside coarse ones."""
    instance = controller(**overrides)
    return instance.plan(
        [assessed(track_id=1, risk_score=0.95, uncertainty=0.6, position=(5.0, 5.0, 0.0))],
        tracks=[at_position((5.0, 5.0, 0.0), track_id=1)],
    )


class TestBasicMapping:
    def test_it_builds_one_tile_per_region(self) -> None:
        result = mapper().build(empty_frame(), uniform_plan())

        assert result.tile_count == 16
        assert [tile.tile_index for tile in result.tiles] == list(range(16))

    def test_a_point_lands_in_the_expected_cell(self) -> None:
        """1 m cells anchored at the tile lower corner: pure arithmetic."""
        result = mapper().build(frame([[-19.5, -19.5, 1.0]]), uniform_plan())
        tile = result.tiles[0]

        assert tile.point_count[0, 0] == 1
        assert tile.mapped_point_count == 1
        assert result.accounting.occupied_cell_count == 1

    def test_the_map_is_labelled_adaptive(self) -> None:
        result = mapper().build(empty_frame(), uniform_plan())

        assert result.is_adaptive is True
        assert result.mapper == "tiled_adaptive_mapper_v1"

    def test_it_records_the_plan_that_produced_it(self) -> None:
        plan = mixed_plan()
        result = mapper().build(empty_frame(), plan)

        assert result.plan.controller == plan.controller
        assert result.plan.tile_count == result.tile_count


class TestMixedResolutions:
    def test_one_map_genuinely_holds_several_cell_sizes(self) -> None:
        """The whole point of the phase, asserted at the representation."""
        result = mapper().build(empty_frame(), mixed_plan())
        sizes = {tile.resolution_m for tile in result.tiles}

        assert len(sizes) > 1
        assert result.finest_resolution_m < result.coarsest_resolution_m

    def test_a_refined_region_holds_more_cells_than_a_coarse_one(self) -> None:
        result = mapper().build(empty_frame(), mixed_plan())
        fine = min(result.tiles, key=lambda tile: tile.resolution_m)
        coarse = max(result.tiles, key=lambda tile: tile.resolution_m)

        assert fine.cell_count > coarse.cell_count

    def test_each_tile_allocates_the_cells_its_extent_needs(self) -> None:
        result = mapper().build(empty_frame(), mixed_plan())

        for tile in result.tiles:
            expected_width = math.ceil(tile.bounds.size_x_m / tile.resolution_m)
            expected_height = math.ceil(tile.bounds.size_y_m / tile.resolution_m)
            assert (tile.width, tile.height) == (expected_width, expected_height)

    def test_the_level_distribution_is_reported(self) -> None:
        result = mapper().build(empty_frame(), mixed_plan())
        tiles = result.tiles_by_level()

        assert sum(tiles.values()) == 16
        assert tiles["low"] > 0


class TestPointAccounting:
    def test_every_point_is_mapped_or_out_of_bounds(self) -> None:
        points = [[-19.5, -19.5, 0.0], [0.5, 0.5, 1.0], [100.0, 0.0, 0.0], [0.0, -100.0, 0.0]]
        result = mapper().build(frame(points), uniform_plan())
        accounting = result.accounting

        assert accounting.input_point_count == 4
        assert accounting.mapped_point_count == 2
        assert accounting.out_of_bounds_point_count == 2
        assert (
            accounting.input_point_count
            == accounting.mapped_point_count + accounting.out_of_bounds_point_count
        )

    def test_accounting_holds_across_mixed_resolutions(self) -> None:
        points = [[x * 0.5 - 19.0, y * 0.5 - 19.0, 0.0] for x in range(40) for y in range(40)]
        result = mapper().build(frame(points), mixed_plan())

        assert result.accounting.mapped_point_count == sum(
            tile.mapped_point_count for tile in result.tiles
        )
        assert result.accounting.input_point_count == len(points)

    def test_no_point_is_counted_twice(self) -> None:
        points = [[-10.0, -10.0, 0.0], [0.0, 0.0, 0.0], [10.0, 10.0, 0.0]]
        result = mapper().build(frame(points), mixed_plan())
        total = sum(int(tile.point_count.sum()) for tile in result.tiles)

        assert total == result.accounting.mapped_point_count == 3

    def test_an_empty_frame_maps_nothing_but_still_allocates_the_grid(self) -> None:
        result = mapper().build(empty_frame(), uniform_plan())

        assert result.accounting.input_point_count == 0
        assert result.accounting.mapped_point_count == 0
        assert result.accounting.occupied_cell_count == 0
        assert result.accounting.total_cell_count == 16 * 10 * 10
        assert result.accounting.out_of_bounds_ratio is None


class TestBoundaryConsistency:
    def test_a_point_on_an_interior_region_boundary_belongs_to_one_region(self) -> None:
        """Half-open regions: the boundary belongs to the tile above and right."""
        result = mapper().build(frame([[-10.0, -10.0, 0.0]]), uniform_plan())
        holding = [tile for tile in result.tiles if int(tile.point_count.sum()) > 0]

        assert len(holding) == 1
        assert result.accounting.mapped_point_count == 1

    def test_a_point_on_the_lower_edge_is_inside(self) -> None:
        result = mapper().build(frame([[-20.0, -20.0, 0.0]]), uniform_plan())

        assert result.accounting.mapped_point_count == 1
        assert result.tiles[0].point_count[0, 0] == 1

    def test_a_point_on_the_upper_edge_is_outside(self) -> None:
        result = mapper().build(frame([[20.0, 0.0, 0.0], [0.0, 20.0, 0.0]]), uniform_plan())

        assert result.accounting.mapped_point_count == 0
        assert result.accounting.out_of_bounds_point_count == 2

    def test_negative_coordinates_are_ordinary(self) -> None:
        result = mapper().build(frame([[-15.0, -15.0, -2.0]]), uniform_plan())
        tile = result.tiles[0]

        assert tile.mapped_point_count == 1
        assert tile.min_height_m[tile.point_count > 0][0] == pytest.approx(-2.0)

    def test_points_across_every_region_boundary_are_all_accounted_for(self) -> None:
        """Sweep the whole extent: no gap between regions, no double coverage."""
        coordinates = [-20.0 + step * 0.5 for step in range(80)]
        points = [[x, y, 0.0] for x in coordinates for y in coordinates]
        result = mapper().build(frame(points), mixed_plan())

        assert result.accounting.mapped_point_count == len(points)
        assert result.accounting.out_of_bounds_point_count == 0
        assert sum(int(tile.point_count.sum()) for tile in result.tiles) == len(points)

    def test_a_clipped_edge_region_still_maps_its_points(self) -> None:
        map_settings, adaptive = settings(max_x_m=25.0)
        instance = TiledAdaptiveMapper(map_settings, adaptive)
        plan = HeuristicResolutionController(map_settings, adaptive).plan([])
        result = instance.build(frame([[24.0, 0.0, 0.0]]), plan)

        assert result.accounting.mapped_point_count == 1


class TestHeightStatistics:
    def test_it_records_min_max_and_mean_height_per_cell(self) -> None:
        points = [[-19.5, -19.5, 1.0], [-19.4, -19.4, 3.0]]
        result = mapper().build(frame(points), uniform_plan())
        tile = result.tiles[0]

        assert tile.point_count[0, 0] == 2
        assert tile.min_height_m[0, 0] == pytest.approx(1.0)
        assert tile.max_height_m[0, 0] == pytest.approx(3.0)
        assert tile.mean_height_m[0, 0] == pytest.approx(2.0)

    def test_an_unobserved_cell_reports_nan_never_zero(self) -> None:
        """Zero is a real height in this frame (ADR-031)."""
        result = mapper().build(frame([[-19.5, -19.5, 1.0]]), uniform_plan())
        tile = result.tiles[0]

        assert math.isnan(tile.min_height_m[5, 5])
        assert math.isnan(tile.max_height_m[5, 5])
        assert math.isnan(tile.mean_height_m[5, 5])

    def test_a_measured_zero_height_stays_zero(self) -> None:
        result = mapper().build(frame([[-19.5, -19.5, 0.0]]), uniform_plan())
        tile = result.tiles[0]

        assert tile.point_count[0, 0] == 1
        assert tile.mean_height_m[0, 0] == pytest.approx(0.0)
        assert not math.isnan(tile.mean_height_m[0, 0])


class TestOccupancy:
    def test_occupancy_is_binary(self) -> None:
        result = mapper().build(frame([[-19.5, -19.5, 1.0]]), uniform_plan())
        tile = result.tiles[0]

        assert tile.occupancy.dtype == bool
        assert tile.occupancy[0, 0] is np.True_
        assert not tile.occupancy[5, 5]

    def test_an_empty_cell_is_unobserved_not_free(self) -> None:
        """Preserved from Phase 6: absence of returns is not evidence of space."""
        result = mapper().build(frame([[-19.5, -19.5, 1.0]]), uniform_plan())
        projected, _ = result.to_adaptive_map()

        assert all(cell.occupancy_state is OccupancyState.OCCUPIED for cell in projected.cells)
        assert len(projected.cells) == 1


class TestProjection:
    def test_projected_cells_carry_their_own_resolution(self) -> None:
        """Where the per-cell resolution field finally becomes real."""
        plan = mixed_plan()
        fine_tile = max((d for d in plan.decisions), key=lambda d: -d.resolution_m)
        x = (fine_tile.bounds.min_x + fine_tile.bounds.max_x) / 2
        y = (fine_tile.bounds.min_y + fine_tile.bounds.max_y) / 2

        result = mapper().build(frame([[x, y, 1.0], [-19.5, -19.5, 1.0]]), plan)
        projected, _ = result.to_adaptive_map()
        sizes = {cell.resolution_m for cell in projected.cells}

        assert len(projected.cells) == 2
        assert len(sizes) == 2

    def test_a_projected_cell_reports_the_level_of_its_region(self) -> None:
        result = mapper().build(frame([[-19.5, -19.5, 1.0]]), uniform_plan(ResolutionLevel.MEDIUM))
        projected, _ = result.to_adaptive_map()

        assert projected.cells[0].resolution_level is ResolutionLevel.MEDIUM
        assert projected.cells[0].resolution_m == pytest.approx(0.5)

    def test_empty_cells_are_omitted(self) -> None:
        result = mapper().build(frame([[-19.5, -19.5, 1.0]]), uniform_plan())
        projected, truncated = result.to_adaptive_map()

        assert len(projected.cells) == 1
        assert truncated is False

    def test_truncation_is_reported_never_silent(self) -> None:
        points = [[-19.5 + i * 1.0, -19.5, 1.0] for i in range(8)]
        result = mapper().build(frame(points), uniform_plan())
        projected, truncated = result.to_adaptive_map(max_cells=3)

        assert len(projected.cells) == 3
        assert truncated is True

    def test_risk_is_not_invented_on_a_projected_cell(self) -> None:
        """No per-cell risk field exists; a region priority is not a cell measurement."""
        result = mapper().build(frame([[5.0, 5.0, 1.0]]), mixed_plan())
        projected, _ = result.to_adaptive_map()

        assert all(cell.risk_score == 0.0 for cell in projected.cells)
        assert all(cell.uncertainty == 0.0 for cell in projected.cells)


class TestPlanValidation:
    def test_a_plan_built_for_a_different_tiling_is_rejected(self) -> None:
        plan = controller(tile_size_m=20.0).plan([])

        with pytest.raises(InvalidPointCloudError) as error:
            mapper(tile_size_m=10.0).build(empty_frame(), plan)

        assert "tiling" in str(error.value) or "tile size" in str(error.value)

    def test_a_plan_beyond_the_cell_ceiling_is_rejected(self) -> None:
        map_settings, adaptive = settings()
        map_settings = map_settings.model_copy(update={"max_cells": 100})
        plan = HeuristicResolutionController(map_settings, adaptive).plan([])

        with pytest.raises(InvalidPointCloudError) as error:
            TiledAdaptiveMapper(map_settings, adaptive).build(empty_frame(), plan)

        assert "cells" in str(error.value)


class TestDeterminism:
    def test_the_same_frame_and_plan_produce_the_same_map(self) -> None:
        points = [[-15.0, -15.0, 1.0], [5.0, 5.0, 2.0], [12.0, -8.0, 0.5]]
        plan = mixed_plan()
        first = mapper().build(frame(points), plan)
        second = mapper().build(frame(points), plan)

        for left, right in zip(first.tiles, second.tiles, strict=True):
            assert np.array_equal(left.point_count, right.point_count)
            assert left.resolution_m == right.resolution_m

    def test_point_order_does_not_change_the_result(self) -> None:
        points = [[-15.0, -15.0, 1.0], [5.0, 5.0, 2.0], [12.0, -8.0, 0.5]]
        plan = uniform_plan()
        forward = mapper().build(frame(points), plan)
        backward = mapper().build(frame(list(reversed(points))), plan)

        for left, right in zip(forward.tiles, backward.tiles, strict=True):
            assert np.array_equal(left.point_count, right.point_count)


class TestFrameLocalLifecycle:
    def test_nothing_accumulates_between_frames(self) -> None:
        instance = mapper()
        plan = uniform_plan()
        instance.build(frame([[-19.5, -19.5, 1.0]]), plan)
        second = instance.build(empty_frame(), plan)

        assert second.accounting.occupied_cell_count == 0

    def test_reset_is_a_documented_no_op(self) -> None:
        instance = mapper()
        plan = uniform_plan()
        before = instance.build(frame([[-19.5, -19.5, 1.0]]), plan)
        instance.reset()
        after = instance.build(frame([[-19.5, -19.5, 1.0]]), plan)

        assert before.accounting.occupied_cell_count == after.accounting.occupied_cell_count


class TestMemoryAndSummary:
    def test_grid_bytes_are_exact_arithmetic(self) -> None:
        result = mapper().build(empty_frame(), uniform_plan())

        assert result.grid_bytes == result.accounting.total_cell_count * 32

    def test_the_summary_carries_no_arrays(self) -> None:
        summary = mapper().build(empty_frame(), mixed_plan()).summary()
        payload = summary.model_dump(mode="json")

        assert "tiles" not in payload
        assert "point_count" not in payload
        assert summary.tile_count == 16
        assert summary.is_adaptive is True

    def test_the_area_weighted_resolution_lies_between_the_extremes(self) -> None:
        result = mapper().build(empty_frame(), mixed_plan())
        average = result.area_weighted_resolution_m()

        assert result.finest_resolution_m <= average <= result.coarsest_resolution_m


class TestComparison:
    def _fixed(self, points: list[list[float]], resolution_m: float = 0.5) -> object:
        map_settings, _ = settings()
        instance = FixedResolutionMapper(map_settings)
        from adaptx.models.resolution import ResolutionDecision

        return instance.build(frame(points), ResolutionDecision.fixed(resolution_m))

    def test_a_comparison_reports_both_workloads(self) -> None:
        points = [[5.0, 5.0, 1.0], [-15.0, -15.0, 0.0]]
        fixed = self._fixed(points)
        adaptive = mapper().build(frame(points), mixed_plan())
        result = compare(fixed, adaptive)  # type: ignore[arg-type]

        assert result.fixed.is_adaptive is False
        assert result.adaptive.is_adaptive is True
        assert result.fixed.uniform_resolution_m == pytest.approx(0.5)
        assert result.adaptive.uniform_resolution_m is None

    def test_a_comparison_over_different_input_is_refused(self) -> None:
        """A ratio between two different scenes would be meaningless."""
        fixed = self._fixed([[5.0, 5.0, 1.0]])
        adaptive = mapper().build(frame([[5.0, 5.0, 1.0], [1.0, 1.0, 0.0]]), mixed_plan())

        with pytest.raises(ValueError, match="identical input"):
            compare(fixed, adaptive)  # type: ignore[arg-type]

    def test_cells_are_attributed_to_priority_groups(self) -> None:
        plan = mixed_plan()
        high, low = partition_by_priority(plan)
        adaptive = mapper().build(empty_frame(), plan)
        result = compare(self._fixed([]), adaptive)  # type: ignore[arg-type]

        assert result.high_priority_tile_count == len(high)
        assert result.low_priority_tile_count == len(low)
        assert result.cells_on_high_priority_tiles + result.cells_on_low_priority_tiles <= (
            result.adaptive.total_cell_count
        )

    def test_an_empty_scene_costs_fewer_cells_than_the_uniform_baseline(self) -> None:
        """Measured, not assumed: the coarse plan really is cheaper here."""
        fixed = self._fixed([])
        adaptive = mapper().build(empty_frame(), uniform_plan())
        result = compare(fixed, adaptive)  # type: ignore[arg-type]

        assert result.cell_ratio < 1.0
        assert result.byte_ratio < 1.0

    def test_the_high_priority_share_is_null_when_nothing_is_high(self) -> None:
        adaptive = mapper().build(empty_frame(), uniform_plan())
        result = compare(self._fixed([]), adaptive)  # type: ignore[arg-type]

        assert result.high_priority_cell_share is None
