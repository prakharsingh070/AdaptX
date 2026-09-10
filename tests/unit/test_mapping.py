"""Fixed-resolution 2.5D mapper tests (Phase 6).

Every case has explicit bounds, an explicit resolution and explicit point
coordinates, so the cell a point lands in is arithmetic rather than a guess.
Nothing here is random.

Frames are constructed directly rather than produced by the Phase 2 pipeline:
these tests exercise mapping, and routing them through preprocessing would make
a mapping failure indistinguishable from a filtering one. The integration
suite covers the real chain.
"""

from __future__ import annotations

import math
from datetime import UTC, datetime

import numpy as np
import pytest
from pydantic import ValidationError

from adaptx.config.settings import MapSettings
from adaptx.core.exceptions import InvalidPointCloudError
from adaptx.mapping.grid_mapper import FixedResolutionMapper, build_mapper
from adaptx.models.common import CoordinateFrame, DataSource
from adaptx.models.map import OccupancyState
from adaptx.models.point_cloud import PointCloudFrame
from adaptx.models.resolution import (
    FIXED_BASELINE_REASON,
    ResolutionDecision,
    ResolutionSource,
)
from adaptx.models.spatial_map import MapAccounting, MapBounds, SpatialMap

EPOCH = datetime(2026, 1, 1, 12, 0, 0, tzinfo=UTC)

#: A small square map with 1 m cells: 4x4 = 16 cells over [0, 4) x [0, 4).
UNIT_SETTINGS = {
    "min_x_m": 0.0,
    "max_x_m": 4.0,
    "min_y_m": 0.0,
    "max_y_m": 4.0,
    "resolution_m": 1.0,
}


def mapper(**overrides: object) -> FixedResolutionMapper:
    """A mapper over the small unit grid unless overridden."""
    return FixedResolutionMapper(MapSettings(**{**UNIT_SETTINGS, **overrides}))  # type: ignore[arg-type]


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
    """A valid frame with no points."""
    return frame([])


class TestBasicMapping:
    def test_a_single_point_occupies_exactly_one_cell(self) -> None:
        result = mapper().build(frame([[0.5, 0.5, 2.0]]))

        assert result.occupied_cell_count == 1
        assert result.point_count[0, 0] == 1
        assert int(result.point_count.sum()) == 1

    def test_points_in_different_cells_stay_separate(self) -> None:
        result = mapper().build(frame([[0.5, 0.5, 1.0], [2.5, 3.5, 2.0]]))

        assert result.occupied_cell_count == 2
        assert result.point_count[0, 0] == 1
        assert result.point_count[3, 2] == 1

    def test_points_in_the_same_cell_accumulate(self) -> None:
        result = mapper().build(frame([[0.1, 0.1, 1.0], [0.9, 0.9, 2.0], [0.5, 0.2, 3.0]]))

        assert result.occupied_cell_count == 1
        assert result.point_count[0, 0] == 3

    def test_rows_step_along_y_and_columns_along_x(self) -> None:
        """Arrays are [row, column] with row on y - stated, then asserted."""
        result = mapper().build(frame([[3.5, 0.5, 0.0]]))

        assert result.point_count[0, 3] == 1
        assert result.point_count[3, 0] == 0

    def test_the_grid_covers_the_configured_bounds(self) -> None:
        result = mapper().build(empty_frame())

        assert (result.width, result.height) == (4, 4)
        assert result.accounting.total_cell_count == 16
        assert result.bounds == MapBounds(min_x=0.0, max_x=4.0, min_y=0.0, max_y=4.0)


class TestOccupancy:
    def test_a_cell_is_occupied_exactly_when_it_holds_a_point(self) -> None:
        result = mapper().build(frame([[0.5, 0.5, 1.0]]))

        assert result.occupancy[0, 0] is np.True_
        assert result.occupancy[1, 1] is np.False_
        assert result.occupancy.dtype == np.bool_

    def test_occupancy_is_derived_from_counts_not_stored_separately(self) -> None:
        """Binary by construction: there is no probability to drift from the count."""
        result = mapper().build(frame([[0.5, 0.5, 1.0], [2.5, 2.5, 1.0]]))

        assert np.array_equal(result.occupancy, result.point_count > 0)

    def test_occupancy_ratio_is_occupied_over_total(self) -> None:
        result = mapper().build(frame([[0.5, 0.5, 1.0], [2.5, 2.5, 1.0]]))

        assert result.occupancy_ratio == pytest.approx(2 / 16)

    def test_an_empty_map_has_zero_occupancy_ratio(self) -> None:
        assert mapper().build(empty_frame()).occupancy_ratio == 0.0


class TestHeightStatistics:
    def test_min_max_and_mean_are_computed_from_the_points(self) -> None:
        result = mapper().build(frame([[0.5, 0.5, 1.0], [0.6, 0.6, 3.0], [0.7, 0.7, 5.0]]))

        assert result.min_height_m[0, 0] == 1.0
        assert result.max_height_m[0, 0] == 5.0
        assert result.mean_height_m[0, 0] == pytest.approx(3.0)

    def test_a_single_point_makes_min_max_and_mean_equal(self) -> None:
        result = mapper().build(frame([[0.5, 0.5, 2.5]]))

        assert result.min_height_m[0, 0] == 2.5
        assert result.max_height_m[0, 0] == 2.5
        assert result.mean_height_m[0, 0] == 2.5

    def test_negative_heights_are_ordinary(self) -> None:
        """Ground sits below the sensor, so negative z is the common case."""
        result = mapper().build(frame([[0.5, 0.5, -1.8], [0.6, 0.6, -1.2]]))

        assert result.min_height_m[0, 0] == -1.8
        assert result.max_height_m[0, 0] == -1.2
        assert result.mean_height_m[0, 0] == pytest.approx(-1.5)

    def test_an_empty_cell_reports_nan_not_zero(self) -> None:
        """Zero is a real height here. An unobserved cell must not claim it."""
        result = mapper().build(frame([[0.5, 0.5, 0.0]]))

        assert result.min_height_m[0, 0] == 0.0
        assert math.isnan(result.min_height_m[2, 2])
        assert math.isnan(result.max_height_m[2, 2])
        assert math.isnan(result.mean_height_m[2, 2])

    def test_every_empty_cell_of_an_empty_map_is_nan(self) -> None:
        result = mapper().build(empty_frame())

        assert np.isnan(result.min_height_m).all()
        assert np.isnan(result.mean_height_m).all()

    def test_heights_are_never_infinite(self) -> None:
        result = mapper().build(frame([[0.5, 0.5, 1.0]]))

        for array in (result.min_height_m, result.max_height_m, result.mean_height_m):
            assert not np.isinf(array).any()


class TestSpatialIndexing:
    def test_a_point_on_the_lower_bound_is_the_first_cell(self) -> None:
        result = mapper().build(frame([[0.0, 0.0, 1.0]]))

        assert result.point_count[0, 0] == 1
        assert result.accounting.out_of_bounds_point_count == 0

    def test_a_point_on_the_upper_bound_is_out_of_bounds(self) -> None:
        """Cells are half-open; max_x would index one cell past the last."""
        result = mapper().build(frame([[4.0, 1.0, 1.0], [1.0, 4.0, 1.0]]))

        assert result.occupied_cell_count == 0
        assert result.accounting.out_of_bounds_point_count == 2

    def test_a_point_just_inside_the_upper_bound_is_the_last_cell(self) -> None:
        result = mapper().build(frame([[3.999, 3.999, 1.0]]))

        assert result.point_count[3, 3] == 1

    def test_a_cell_boundary_belongs_to_the_upper_cell(self) -> None:
        """floor() puts x=1.0 in column 1, not column 0."""
        result = mapper().build(frame([[1.0, 0.5, 1.0]]))

        assert result.point_count[0, 1] == 1
        assert result.point_count[0, 0] == 0

    def test_negative_coordinates_are_supported(self) -> None:
        stage = mapper(min_x_m=-4.0, max_x_m=4.0, min_y_m=-4.0, max_y_m=4.0)
        result = stage.build(frame([[-3.5, -3.5, 1.0], [3.5, 3.5, 2.0]]))

        assert (result.width, result.height) == (8, 8)
        assert result.point_count[0, 0] == 1
        assert result.point_count[7, 7] == 1

    def test_the_origin_falls_where_the_bounds_put_it(self) -> None:
        stage = mapper(min_x_m=-4.0, max_x_m=4.0, min_y_m=-4.0, max_y_m=4.0)
        result = stage.build(frame([[0.0, 0.0, 1.0]]))

        assert result.point_count[4, 4] == 1

    def test_cell_centres_are_reported_at_the_middle_of_the_cell(self) -> None:
        result = mapper().build(empty_frame())

        assert result.cell_centre(0, 0) == (0.5, 0.5)
        assert result.cell_centre(3, 3) == (3.5, 3.5)


class TestResolution:
    @pytest.mark.parametrize(
        ("resolution", "expected"),
        [(1.0, (4, 4)), (0.5, (8, 8)), (0.25, (16, 16)), (2.0, (2, 2))],
    )
    def test_resolution_sets_the_grid_dimensions(
        self, resolution: float, expected: tuple[int, int]
    ) -> None:
        result = mapper(resolution_m=resolution).build(empty_frame())

        assert (result.width, result.height) == expected

    def test_a_finer_resolution_separates_points_a_coarse_one_merges(self) -> None:
        points = [[0.1, 0.1, 1.0], [0.9, 0.9, 2.0]]

        coarse = mapper(resolution_m=1.0).build(frame(points))
        fine = mapper(resolution_m=0.25).build(frame(points))

        assert coarse.occupied_cell_count == 1
        assert fine.occupied_cell_count == 2

    def test_the_default_resolution_comes_from_configuration(self) -> None:
        decision = mapper(resolution_m=0.5).default_resolution()

        assert decision.resolution_m == 0.5
        assert decision.source is ResolutionSource.FIXED
        assert decision.reason == FIXED_BASELINE_REASON

    def test_a_supplied_resolution_overrides_the_configured_one(self) -> None:
        override = ResolutionDecision.override(0.5, reason="test sweep", requested_by="unit test")
        result = mapper(resolution_m=1.0).build(empty_frame(), override)

        assert result.resolution_m == 0.5
        assert (result.width, result.height) == (8, 8)
        assert result.resolution.source is ResolutionSource.OVERRIDE

    def test_a_resolution_below_the_configured_floor_is_rejected(self) -> None:
        with pytest.raises(InvalidPointCloudError, match="outside the configured range"):
            mapper(min_resolution_m=0.5).build(empty_frame(), ResolutionDecision.fixed(0.1))

    def test_a_resolution_above_the_configured_ceiling_is_rejected(self) -> None:
        with pytest.raises(InvalidPointCloudError, match="outside the configured range"):
            mapper(max_resolution_m=2.0).build(empty_frame(), ResolutionDecision.fixed(3.0))

    def test_a_non_positive_resolution_cannot_be_constructed(self) -> None:
        with pytest.raises(ValidationError):
            ResolutionDecision.fixed(0.0)

    def test_a_non_finite_resolution_is_rejected(self) -> None:
        with pytest.raises(ValidationError, match="finite"):
            ResolutionDecision(resolution_m=math.inf, reason="bad")


class TestBounds:
    def test_all_points_inside_are_all_mapped(self) -> None:
        result = mapper().build(frame([[0.5, 0.5, 1.0], [2.5, 2.5, 1.0]]))

        assert result.accounting.mapped_point_count == 2
        assert result.accounting.out_of_bounds_point_count == 0

    def test_all_points_outside_leave_an_empty_grid(self) -> None:
        result = mapper().build(frame([[99.0, 99.0, 1.0], [-5.0, 2.0, 1.0]]))

        assert result.occupied_cell_count == 0
        assert result.accounting.mapped_point_count == 0
        assert result.accounting.out_of_bounds_point_count == 2
        assert int(result.point_count.sum()) == 0

    def test_a_mixed_frame_maps_only_the_points_inside(self) -> None:
        result = mapper().build(
            frame([[0.5, 0.5, 1.0], [99.0, 0.5, 1.0], [2.5, 2.5, 1.0], [0.5, -9.0, 1.0]])
        )

        assert result.accounting.mapped_point_count == 2
        assert result.accounting.out_of_bounds_point_count == 2

    def test_out_of_bounds_points_do_not_corrupt_any_cell(self) -> None:
        clean = mapper().build(frame([[0.5, 0.5, 1.0]]))
        polluted = mapper().build(frame([[0.5, 0.5, 1.0], [500.0, 500.0, 900.0]]))

        assert np.array_equal(clean.point_count, polluted.point_count)
        assert polluted.max_height_m[0, 0] == 1.0

    def test_the_out_of_bounds_ratio_is_null_for_an_empty_frame(self) -> None:
        """No points means the question has no answer, so it is not answered."""
        assert mapper().build(empty_frame()).accounting.out_of_bounds_ratio is None

    def test_inverted_bounds_are_rejected_by_configuration(self) -> None:
        with pytest.raises(ValidationError, match="min_x_m"):
            MapSettings(min_x_m=5.0, max_x_m=-5.0)

    def test_inverted_bounds_are_rejected_by_the_contract(self) -> None:
        with pytest.raises(ValidationError, match="must be <"):
            MapBounds(min_x=1.0, max_x=0.0, min_y=0.0, max_y=1.0)


class TestMemorySafety:
    def test_a_grid_above_the_cell_limit_is_rejected_before_allocation(self) -> None:
        stage = mapper(min_resolution_m=0.01, max_cells=100)

        with pytest.raises(InvalidPointCloudError, match="above the configured limit"):
            stage.build(empty_frame(), ResolutionDecision.fixed(0.01))

    def test_the_limit_is_checked_by_configuration_too(self) -> None:
        with pytest.raises(ValidationError, match="max_cells"):
            MapSettings(
                min_x_m=-100.0,
                max_x_m=100.0,
                min_y_m=-100.0,
                max_y_m=100.0,
                resolution_m=0.05,
                min_resolution_m=0.01,
                max_cells=1000,
            )

    def test_grid_shape_is_available_without_building_a_map(self) -> None:
        """Dimensions are computed before arrays are allocated."""
        assert mapper(resolution_m=0.5).grid_shape(0.5) == (8, 8)


class TestEmptyInput:
    def test_an_empty_frame_produces_a_valid_map(self) -> None:
        result = mapper().build(empty_frame())

        assert isinstance(result, SpatialMap)
        assert (result.width, result.height) == (4, 4)
        assert result.resolution_m == 1.0
        assert result.occupied_cell_count == 0
        assert result.accounting.input_point_count == 0
        assert result.accounting.mapped_point_count == 0
        assert result.occupancy_ratio == 0.0

    def test_an_empty_frame_does_not_raise(self) -> None:
        mapper().build(empty_frame())

    def test_an_empty_map_still_carries_its_configuration(self) -> None:
        result = mapper().build(empty_frame())

        assert result.configuration.resolution_m == 1.0
        assert result.configuration.max_cells > 0


class TestDeterminism:
    def test_the_same_frame_always_produces_the_same_grid(self) -> None:
        points = [[0.5, 0.5, 1.0], [2.5, 1.5, 2.0], [3.5, 3.5, 3.0]]
        first = mapper().build(frame(points))
        second = mapper().build(frame(points))

        assert np.array_equal(first.point_count, second.point_count)
        assert np.array_equal(first.min_height_m, second.min_height_m, equal_nan=True)
        assert np.array_equal(first.mean_height_m, second.mean_height_m, equal_nan=True)

    def test_repeated_calls_on_one_mapper_do_not_drift(self) -> None:
        stage = mapper()
        points = [[0.5, 0.5, 1.0], [2.5, 2.5, 2.0]]
        first = stage.build(frame(points))
        for _ in range(9):
            latest = stage.build(frame(points))

        assert np.array_equal(first.point_count, latest.point_count)

    def test_a_second_mapper_agrees_with_the_first(self) -> None:
        points = [[0.5, 0.5, 1.0], [1.5, 2.5, 4.0]]

        assert np.array_equal(
            mapper().build(frame(points)).mean_height_m,
            mapper().build(frame(points)).mean_height_m,
            equal_nan=True,
        )


class TestAccounting:
    def test_input_equals_mapped_plus_out_of_bounds(self) -> None:
        result = mapper().build(frame([[0.5, 0.5, 1.0], [99.0, 0.5, 1.0], [2.5, 2.5, 1.0]]))
        accounting = result.accounting

        assert accounting.input_point_count == (
            accounting.mapped_point_count + accounting.out_of_bounds_point_count
        )

    def test_accounting_that_does_not_add_up_is_rejected(self) -> None:
        with pytest.raises(ValidationError, match="must equal"):
            MapAccounting(
                input_point_count=10,
                mapped_point_count=3,
                out_of_bounds_point_count=3,
                occupied_cell_count=1,
                total_cell_count=16,
            )

    def test_occupied_cells_cannot_exceed_total_cells(self) -> None:
        with pytest.raises(ValidationError, match="cannot exceed"):
            MapAccounting(
                input_point_count=0,
                mapped_point_count=0,
                out_of_bounds_point_count=0,
                occupied_cell_count=99,
                total_cell_count=16,
            )

    def test_the_counts_match_the_grid(self) -> None:
        result = mapper().build(frame([[0.5, 0.5, 1.0], [0.6, 0.6, 2.0], [2.5, 2.5, 3.0]]))

        assert int(result.point_count.sum()) == result.accounting.mapped_point_count
        assert int(np.count_nonzero(result.point_count)) == result.occupied_cell_count

    def test_the_duration_is_measured_not_estimated(self) -> None:
        result = mapper(resolution_m=0.25).build(frame([[0.5, 0.5, 1.0]]))

        assert result.duration_ms > 0.0
        assert math.isfinite(result.duration_ms)


class TestMetadata:
    def test_frame_identity_and_time_are_carried_through(self) -> None:
        result = mapper().build(frame([[0.5, 0.5, 1.0]], frame_id=42))

        assert result.frame_id == 42
        assert result.sensor_id == "roof_lidar"
        assert result.timestamp == EPOCH

    def test_provenance_is_carried_through(self) -> None:
        """Synthetic points must not become a live-sensor map."""
        result = mapper().build(frame([[0.5, 0.5, 1.0]]))

        assert result.source is DataSource.SYNTHETIC_TEST
        assert result.coordinate_frame is CoordinateFrame.EGO

    def test_the_map_is_labelled_a_fixed_resolution_baseline(self) -> None:
        result = mapper().build(empty_frame())

        assert result.is_adaptive is False
        assert result.mapper == "fixed_resolution_mapper_v1"
        assert result.resolution.is_adaptive is False

    def test_the_summary_reports_the_same_numbers_as_the_map(self) -> None:
        result = mapper().build(frame([[0.5, 0.5, 1.0]]))
        summary = result.summary()

        assert summary.width == result.width
        assert summary.height == result.height
        assert summary.accounting == result.accounting
        assert summary.occupancy_ratio == result.occupancy_ratio
        assert summary.is_adaptive is False

    def test_the_factory_builds_the_configured_mapper(self) -> None:
        built = build_mapper(MapSettings(**UNIT_SETTINGS))  # type: ignore[arg-type]

        assert isinstance(built, FixedResolutionMapper)
        assert built.is_adaptive is False


class TestFrameLocalLifecycle:
    def test_consecutive_frames_do_not_contaminate_each_other(self) -> None:
        """The whole point of frame-local mapping, asserted rather than assumed."""
        stage = mapper()
        stage.build(frame([[0.5, 0.5, 1.0]], frame_id=0))
        second = stage.build(frame([[2.5, 2.5, 5.0]], frame_id=1))

        assert second.point_count[0, 0] == 0
        assert second.point_count[2, 2] == 1
        assert second.occupied_cell_count == 1

    def test_a_frame_after_a_busy_one_is_not_inflated(self) -> None:
        stage = mapper()
        stage.build(frame([[float(i) * 0.1, 0.5, 1.0] for i in range(30)]))
        after = stage.build(empty_frame())

        assert after.occupied_cell_count == 0
        assert after.accounting.mapped_point_count == 0

    def test_reset_is_a_documented_no_op(self) -> None:
        """There is no accumulated state, so reset cannot change an output."""
        stage = mapper()
        before = stage.build(frame([[0.5, 0.5, 1.0]]))
        stage.reset()
        after = stage.build(frame([[0.5, 0.5, 1.0]]))

        assert np.array_equal(before.point_count, after.point_count)


class TestAdaptiveMapProjection:
    def test_only_occupied_cells_are_projected(self) -> None:
        result = mapper().build(frame([[0.5, 0.5, 1.0], [2.5, 2.5, 2.0]]))
        projected, truncated = result.to_adaptive_map()

        assert truncated is False
        assert projected.cell_count == 2
        assert all(cell.point_count > 0 for cell in projected.cells)

    def test_projected_cells_carry_binary_occupancy(self) -> None:
        result = mapper().build(frame([[0.5, 0.5, 1.0]]))
        projected, _ = result.to_adaptive_map()
        cell = projected.cells[0]

        assert cell.occupancy == 1.0
        assert cell.occupancy_state is OccupancyState.OCCUPIED

    def test_projected_cells_carry_measured_heights(self) -> None:
        result = mapper().build(frame([[0.5, 0.5, 1.0], [0.6, 0.6, 3.0]]))
        cell = result.to_adaptive_map()[0].cells[0]

        assert cell.height_min_m == 1.0
        assert cell.height_max_m == 3.0
        assert cell.height_m == pytest.approx(2.0)

    def test_projection_reports_truncation_rather_than_silently_cutting(self) -> None:
        result = mapper().build(frame([[0.5, 0.5, 1.0], [1.5, 1.5, 1.0], [2.5, 2.5, 1.0]]))
        projected, truncated = result.to_adaptive_map(max_cells=2)

        assert truncated is True
        assert projected.cell_count == 2

    def test_an_empty_map_projects_to_no_cells(self) -> None:
        projected, truncated = mapper().build(empty_frame()).to_adaptive_map()

        assert projected.cell_count == 0
        assert truncated is False

    def test_the_projection_is_not_labelled_adaptive(self) -> None:
        projected, _ = mapper().build(frame([[0.5, 0.5, 1.0]])).to_adaptive_map()

        assert projected.is_adaptive is False

    def test_risk_and_uncertainty_are_left_unset(self) -> None:
        """Nothing computes them yet; filling them would be an invention."""
        cell = mapper().build(frame([[0.5, 0.5, 1.0]])).to_adaptive_map()[0].cells[0]

        assert cell.risk_score == 0.0
        assert cell.uncertainty == 0.0


class TestResolutionDecisionContract:
    def test_the_fixed_helper_records_its_provenance(self) -> None:
        decision = ResolutionDecision.fixed(0.5)

        assert decision.source is ResolutionSource.FIXED
        assert decision.requested_by == "configuration"
        assert decision.is_adaptive is False

    def test_the_override_helper_is_distinguishable_from_configuration(self) -> None:
        decision = ResolutionDecision.override(
            0.25, reason="benchmark sweep", requested_by="benchmark"
        )

        assert decision.source is ResolutionSource.OVERRIDE
        assert decision.is_adaptive is False
        assert decision.reason == "benchmark sweep"

    def test_an_adaptive_source_would_be_flagged(self) -> None:
        """Reserved, not implemented - but the contract already distinguishes it."""
        decision = ResolutionDecision(
            resolution_m=0.25,
            source=ResolutionSource.ADAPTIVE,
            reason="reserved for a future controller",
        )

        assert decision.is_adaptive is True

    def test_a_reason_is_required(self) -> None:
        with pytest.raises(ValidationError):
            ResolutionDecision(resolution_m=0.5, reason="")


class TestSpatialMapContract:
    def test_mismatched_array_shapes_are_rejected(self) -> None:
        """A grid whose arrays disagree with its dimensions cannot exist."""
        with pytest.raises(ValidationError, match="expected"):
            SpatialMap(
                frame_id=0,
                sensor_id="x",
                mapper="m",
                resolution=ResolutionDecision.fixed(1.0),
                bounds=MapBounds(min_x=0.0, max_x=2.0, min_y=0.0, max_y=2.0),
                width=2,
                height=2,
                point_count=np.zeros((1, 1), dtype=np.int64),
                min_height_m=np.full((2, 2), np.nan),
                max_height_m=np.full((2, 2), np.nan),
                mean_height_m=np.full((2, 2), np.nan),
                accounting=MapAccounting(
                    input_point_count=0,
                    mapped_point_count=0,
                    out_of_bounds_point_count=0,
                    occupied_cell_count=0,
                    total_cell_count=4,
                ),
                duration_ms=0.0,
                configuration=mapper().configuration(1.0),
            )

    def test_an_infinite_height_is_rejected(self) -> None:
        with pytest.raises(ValidationError, match="infinities"):
            SpatialMap(
                frame_id=0,
                sensor_id="x",
                mapper="m",
                resolution=ResolutionDecision.fixed(1.0),
                bounds=MapBounds(min_x=0.0, max_x=1.0, min_y=0.0, max_y=1.0),
                width=1,
                height=1,
                point_count=np.zeros((1, 1), dtype=np.int64),
                min_height_m=np.full((1, 1), np.inf),
                max_height_m=np.full((1, 1), np.nan),
                mean_height_m=np.full((1, 1), np.nan),
                accounting=MapAccounting(
                    input_point_count=0,
                    mapped_point_count=0,
                    out_of_bounds_point_count=0,
                    occupied_cell_count=0,
                    total_cell_count=1,
                ),
                duration_ms=0.0,
                configuration=mapper().configuration(1.0),
            )

    def test_bounds_report_their_extent(self) -> None:
        bounds = MapBounds(min_x=-10.0, max_x=30.0, min_y=-5.0, max_y=5.0)

        assert bounds.size_x_m == 40.0
        assert bounds.size_y_m == 10.0
        assert bounds.contains(0.0, 0.0) is True
        assert bounds.contains(30.0, 0.0) is False
