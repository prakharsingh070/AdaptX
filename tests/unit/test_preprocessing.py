"""Phase 2A preprocessing pipeline tests.

All clouds here are small, hand-written and deterministic so every expected
count can be reasoned about by hand. Nothing is random.

Coordinate convention under test (ADR-009): +x forward, +y left, +z up,
metres, origin at the sensor. Bounds are inclusive at both ends.
"""

from __future__ import annotations

import itertools

import numpy as np
import pytest

from adaptx.config.settings import LiDARSettings
from adaptx.core.exceptions import InvalidPointCloudError
from adaptx.models.common import CoordinateFrame, DataSource
from adaptx.models.point_cloud import PointCloudFrame, RawPointCloudFrame
from adaptx.models.processing import ProcessingStage
from adaptx.perception.preprocessing import PointCloudPreprocessor

# A permissive configuration: only the filter under test constrains anything.
OPEN_SETTINGS = LiDARSettings(
    min_points=0,
    max_points=1000,
    min_range_m=0.0,
    max_range_m=1000.0,
    roi_x_min_m=-1000.0,
    roi_x_max_m=1000.0,
    roi_y_min_m=-1000.0,
    roi_y_max_m=1000.0,
    roi_z_min_m=-1000.0,
    roi_z_max_m=1000.0,
)


def make_raw(points: list[list[float]], **kwargs: object) -> RawPointCloudFrame:
    """Build a raw frame from an explicit list of points."""
    defaults: dict[str, object] = {
        "frame_id": 1,
        "sensor_id": "test_lidar",
        "source": DataSource.SYNTHETIC_TEST,
        "coordinate_frame": CoordinateFrame.LIDAR,
    }
    defaults.update(kwargs)
    return RawPointCloudFrame.from_sequence(points, **defaults)


def preprocessor(**overrides: float | int) -> PointCloudPreprocessor:
    """A preprocessor whose configuration starts permissive."""
    return PointCloudPreprocessor(OPEN_SETTINGS.model_copy(update=dict(overrides)))


class TestValidInput:
    def test_valid_cloud_passes_through_unchanged(self) -> None:
        points = [[1.0, 2.0, 0.5], [3.0, -1.0, 0.0], [10.0, 4.0, 1.0]]
        result = preprocessor().run(make_raw(points))

        assert result.metrics.output_point_count == 3
        assert np.array_equal(result.frame.points, np.array(points))
        assert isinstance(result.frame, PointCloudFrame)

    def test_point_order_is_preserved(self) -> None:
        points = [[5.0, 0.0, 0.0], [1.0, 0.0, 0.0], [3.0, 0.0, 0.0]]
        result = preprocessor().run(make_raw(points))
        assert result.frame.points[:, 0].tolist() == [5.0, 1.0, 3.0]

    def test_intensity_column_survives(self) -> None:
        frame = make_raw([[1.0, 0.0, 0.0, 0.7], [2.0, 0.0, 0.0, 0.2]])
        result = preprocessor().run(frame)

        assert result.frame.has_intensity is True
        assert result.frame.fields == ("x", "y", "z", "intensity")
        assert result.frame.points[:, 3].tolist() == [0.7, 0.2]

    def test_output_is_a_validated_finite_frame(self) -> None:
        result = preprocessor().run(make_raw([[1.0, 1.0, 1.0], [float("nan"), 0.0, 0.0]]))
        assert np.isfinite(result.frame.points).all()


class TestEmptyInput:
    def test_empty_cloud_is_handled(self) -> None:
        frame = RawPointCloudFrame(
            frame_id=0, sensor_id="s", points=np.empty((0, 3), dtype=np.float64)
        )
        result = preprocessor().run(frame)

        assert result.metrics.input_point_count == 0
        assert result.metrics.output_point_count == 0
        assert result.metrics.retention_ratio is None
        assert result.frame.point_count == 0
        assert result.frame.bounds() is None

    def test_empty_cloud_is_rejected_when_a_minimum_is_configured(self) -> None:
        frame = RawPointCloudFrame(
            frame_id=0, sensor_id="s", points=np.empty((0, 3), dtype=np.float64)
        )
        with pytest.raises(InvalidPointCloudError):
            preprocessor(min_points=1).run(frame)


class TestMalformedInput:
    """Structural faults are rejected by the frame contract, not silently fixed."""

    @pytest.mark.parametrize(
        "points",
        [
            np.zeros((3,)),  # not 2D
            np.zeros((2, 2)),  # too few columns
            np.zeros((2, 5)),  # too many columns
            np.zeros((2, 3), dtype=np.int32),  # not floating
        ],
    )
    def test_malformed_dimensions_are_rejected(self, points: np.ndarray) -> None:
        from pydantic import ValidationError

        with pytest.raises(ValidationError):
            RawPointCloudFrame(frame_id=0, sensor_id="s", points=points)

    def test_too_many_points_are_rejected(self) -> None:
        frame = make_raw([[float(i), 0.0, 0.0] for i in range(1, 11)])
        with pytest.raises(InvalidPointCloudError) as excinfo:
            preprocessor(max_points=5).run(frame)
        assert excinfo.value.details["max_points"] == 5

    def test_too_few_points_are_rejected(self) -> None:
        with pytest.raises(InvalidPointCloudError) as excinfo:
            preprocessor(min_points=5).run(make_raw([[1.0, 0.0, 0.0]]))
        assert excinfo.value.details["min_points"] == 5


class TestInvalidPointRemoval:
    def test_nan_points_are_removed_and_counted(self) -> None:
        frame = make_raw([[1.0, 0.0, 0.0], [float("nan"), 0.0, 0.0], [2.0, float("nan"), 0.0]])
        result = preprocessor().run(frame)

        assert result.metrics.input_point_count == 3
        assert result.metrics.invalid_point_count == 2
        assert result.metrics.output_point_count == 1
        assert result.frame.points.tolist() == [[1.0, 0.0, 0.0]]

    @pytest.mark.parametrize("bad", [float("inf"), float("-inf")])
    def test_infinite_points_are_removed(self, bad: float) -> None:
        result = preprocessor().run(make_raw([[1.0, 0.0, 0.0], [bad, 0.0, 0.0]]))

        assert result.metrics.invalid_point_count == 1
        assert result.metrics.output_point_count == 1

    def test_nan_in_the_intensity_column_invalidates_the_point(self) -> None:
        """A NaN intensity means an untrustworthy return, not a usable point."""
        frame = make_raw([[1.0, 0.0, 0.0, 0.5], [2.0, 0.0, 0.0, float("nan")]])
        result = preprocessor().run(frame)

        assert result.metrics.invalid_point_count == 1
        assert result.frame.points.tolist() == [[1.0, 0.0, 0.0, 0.5]]

    def test_all_invalid_cloud_yields_an_empty_frame(self) -> None:
        frame = make_raw([[float("nan")] * 3, [float("inf")] * 3, [0.0, float("nan"), 0.0]])
        result = preprocessor().run(frame)

        assert result.metrics.input_point_count == 3
        assert result.metrics.invalid_point_count == 3
        assert result.metrics.output_point_count == 0
        assert result.metrics.retention_ratio == 0.0
        assert result.frame.point_count == 0

    def test_invalid_points_are_not_attributed_to_later_stages(self) -> None:
        """A NaN point must be counted once, by the removal stage only."""
        frame = make_raw([[float("nan"), 0.0, 0.0], [5.0, 0.0, 0.0]])
        result = preprocessor(roi_x_max_m=10.0, max_range_m=10.0).run(frame)

        assert result.metrics.invalid_point_count == 1
        assert result.metrics.roi_rejected_count == 0
        assert result.metrics.range_rejected_count == 0


class TestROIFiltering:
    def test_points_outside_the_roi_are_removed(self) -> None:
        frame = make_raw(
            [
                [5.0, 0.0, 0.0],  # inside
                [50.0, 0.0, 0.0],  # beyond +x
                [-50.0, 0.0, 0.0],  # beyond -x
                [0.0, 50.0, 0.0],  # beyond +y
                [0.0, 0.0, 50.0],  # beyond +z
            ]
        )
        result = preprocessor(
            roi_x_min_m=-10.0,
            roi_x_max_m=10.0,
            roi_y_min_m=-10.0,
            roi_y_max_m=10.0,
            roi_z_min_m=-10.0,
            roi_z_max_m=10.0,
        ).run(frame)

        assert result.metrics.roi_rejected_count == 4
        assert result.frame.points.tolist() == [[5.0, 0.0, 0.0]]

    def test_points_exactly_on_roi_boundaries_are_kept(self) -> None:
        """Bounds are inclusive on both ends."""
        frame = make_raw(
            [
                [10.0, 0.0, 0.0],
                [-10.0, 0.0, 0.0],
                [0.0, 10.0, 0.0],
                [0.0, -10.0, 0.0],
                [0.0, 0.0, 10.0],
                [0.0, 0.0, -10.0],
            ]
        )
        result = preprocessor(
            roi_x_min_m=-10.0,
            roi_x_max_m=10.0,
            roi_y_min_m=-10.0,
            roi_y_max_m=10.0,
            roi_z_min_m=-10.0,
            roi_z_max_m=10.0,
        ).run(frame)

        assert result.metrics.roi_rejected_count == 0
        assert result.metrics.output_point_count == 6

    def test_point_just_outside_a_boundary_is_removed(self) -> None:
        frame = make_raw([[10.0, 0.0, 0.0], [10.001, 0.0, 0.0]])
        result = preprocessor(roi_x_max_m=10.0).run(frame)

        assert result.metrics.roi_rejected_count == 1
        assert result.frame.points.tolist() == [[10.0, 0.0, 0.0]]

    def test_asymmetric_roi_respects_the_forward_convention(self) -> None:
        """+x is forward: a forward-biased ROI keeps ahead and drops behind."""
        frame = make_raw([[20.0, 0.0, 0.0], [-20.0, 0.0, 0.0]])
        result = preprocessor(roi_x_min_m=-5.0, roi_x_max_m=40.0).run(frame)

        assert result.frame.points.tolist() == [[20.0, 0.0, 0.0]]

    def test_negative_coordinates_inside_the_roi_are_kept(self) -> None:
        frame = make_raw([[-3.0, -4.0, -1.0], [-2.0, -2.0, -0.5]])
        result = preprocessor(min_range_m=0.0).run(frame)
        assert result.metrics.output_point_count == 2


class TestRangeFiltering:
    def test_points_closer_than_the_minimum_are_removed(self) -> None:
        frame = make_raw([[0.1, 0.0, 0.0], [5.0, 0.0, 0.0]])
        result = preprocessor(min_range_m=1.0).run(frame)

        assert result.metrics.range_rejected_count == 1
        assert result.frame.points.tolist() == [[5.0, 0.0, 0.0]]

    def test_points_beyond_the_maximum_are_removed(self) -> None:
        frame = make_raw([[5.0, 0.0, 0.0], [500.0, 0.0, 0.0]])
        result = preprocessor(max_range_m=100.0).run(frame)

        assert result.metrics.range_rejected_count == 1
        assert result.frame.points.tolist() == [[5.0, 0.0, 0.0]]

    def test_points_exactly_on_range_boundaries_are_kept(self) -> None:
        # 3-4-5 triangle: distance is exactly 5.0.
        frame = make_raw([[3.0, 4.0, 0.0], [0.0, 0.0, 1.0]])
        result = preprocessor(min_range_m=1.0, max_range_m=5.0).run(frame)

        assert result.metrics.range_rejected_count == 0
        assert result.metrics.output_point_count == 2

    def test_range_is_3d_not_planar(self) -> None:
        """A point directly above the sensor has planar distance 0 but range 2."""
        frame = make_raw([[0.0, 0.0, 2.0]])
        kept = preprocessor(min_range_m=1.0, max_range_m=5.0).run(frame)
        dropped = preprocessor(min_range_m=1.0, max_range_m=1.5).run(frame)

        assert kept.metrics.output_point_count == 1
        assert dropped.metrics.range_rejected_count == 1

    def test_range_uses_absolute_distance_for_negative_coordinates(self) -> None:
        frame = make_raw([[-3.0, -4.0, 0.0]])
        result = preprocessor(min_range_m=1.0, max_range_m=5.0).run(frame)
        assert result.metrics.output_point_count == 1

    def test_very_large_coordinates_are_removed_without_error(self) -> None:
        """Squaring a large coordinate must not overflow or raise."""
        frame = make_raw([[1.0e6, 1.0e6, 1.0e6], [5.0, 0.0, 0.0]])
        result = preprocessor(
            max_range_m=100.0,
            roi_x_max_m=1.0e9,
            roi_y_max_m=1.0e9,
            roi_z_max_m=1.0e9,
        ).run(frame)

        assert result.metrics.range_rejected_count == 1
        assert result.frame.points.tolist() == [[5.0, 0.0, 0.0]]


class TestStageOrdering:
    def test_roi_is_applied_before_range(self) -> None:
        """A point failing both is attributed to the ROI, matching pipeline order."""
        frame = make_raw([[500.0, 0.0, 0.0]])
        result = preprocessor(roi_x_max_m=100.0, max_range_m=100.0).run(frame)

        assert result.metrics.roi_rejected_count == 1
        assert result.metrics.range_rejected_count == 0

    def test_counts_partition_the_input(self) -> None:
        frame = make_raw(
            [
                [1.0, 0.0, 0.0],  # kept
                [float("nan"), 0.0, 0.0],  # invalid
                [500.0, 0.0, 0.0],  # outside ROI
                [0.05, 0.0, 0.0],  # inside ROI, below min range
            ]
        )
        metrics = preprocessor(roi_x_max_m=100.0, min_range_m=0.5).run(frame).metrics

        assert metrics.input_point_count == 4
        assert metrics.invalid_point_count == 1
        assert metrics.roi_rejected_count == 1
        assert metrics.range_rejected_count == 1
        assert metrics.output_point_count == 1


class TestProcessingMetrics:
    def test_stages_are_reported_in_pipeline_order(self) -> None:
        result = preprocessor().run(make_raw([[1.0, 0.0, 0.0]]))
        assert [stage.stage for stage in result.metrics.stages] == [
            ProcessingStage.VALIDATION,
            ProcessingStage.INVALID_REMOVAL,
            ProcessingStage.ROI_FILTER,
            ProcessingStage.RANGE_FILTER,
        ]

    def test_each_stage_conserves_points(self) -> None:
        frame = make_raw(
            [[1.0, 0.0, 0.0], [float("nan"), 0.0, 0.0], [500.0, 0.0, 0.0], [0.05, 0.0, 0.0]]
        )
        result = preprocessor(roi_x_max_m=100.0, min_range_m=0.5).run(frame)

        for stage in result.metrics.stages:
            assert stage.input_points == stage.output_points + stage.rejected_points

    def test_stage_chain_is_continuous(self) -> None:
        frame = make_raw([[1.0, 0.0, 0.0], [float("nan"), 0.0, 0.0], [500.0, 0.0, 0.0]])
        stages = preprocessor(roi_x_max_m=100.0).run(frame).metrics.stages

        for earlier, later in itertools.pairwise(stages):
            assert earlier.output_points == later.input_points

    def test_duration_is_measured_and_positive(self) -> None:
        result = preprocessor().run(make_raw([[1.0, 0.0, 0.0]]))
        assert result.metrics.duration_ms > 0.0

    def test_retention_ratio(self) -> None:
        frame = make_raw([[1.0, 0.0, 0.0], [float("nan"), 0.0, 0.0]])
        assert preprocessor().run(frame).metrics.retention_ratio == pytest.approx(0.5)

    def test_processor_is_identified(self) -> None:
        result = preprocessor().run(make_raw([[1.0, 0.0, 0.0]]))
        assert result.metrics.processor == "preprocessing_v1"


class TestMetadataPreservation:
    def test_frame_metadata_survives_processing(self) -> None:
        frame = make_raw(
            [[1.0, 0.0, 0.0], [float("nan"), 0.0, 0.0]],
            frame_id=42,
            sensor_id="roof_lidar",
            source=DataSource.REPLAY,
            coordinate_frame=CoordinateFrame.EGO,
        )
        result = preprocessor().run(frame)

        assert result.frame.frame_id == 42
        assert result.frame.sensor_id == "roof_lidar"
        assert result.frame.source is DataSource.REPLAY
        assert result.frame.coordinate_frame is CoordinateFrame.EGO
        assert result.frame.timestamp == frame.timestamp
        assert result.frame.schema_version == frame.schema_version

    def test_input_summary_describes_the_frame_as_received(self) -> None:
        frame = make_raw([[1.0, 0.0, 0.0], [float("nan"), 0.0, 0.0]])
        result = preprocessor().run(frame)

        assert result.input_summary.point_count == 2
        assert result.output_summary.point_count == 1
        assert result.input_summary.frame_id == result.output_summary.frame_id

    def test_raw_bounds_ignore_non_finite_points(self) -> None:
        """An infinity must not leak into a bound and serialise as null."""
        frame = make_raw([[1.0, 0.0, 0.0], [float("inf"), 0.0, 0.0], [5.0, 2.0, 1.0]])
        bounds = frame.bounds()

        assert bounds is not None
        assert bounds.min_x == 1.0
        assert bounds.max_x == 5.0
        assert bounds.max_y == 2.0

    def test_bounds_are_none_when_no_point_is_finite(self) -> None:
        frame = make_raw([[float("nan")] * 3, [float("inf")] * 3])
        assert frame.bounds() is None

    def test_input_frame_is_not_mutated(self) -> None:
        original = np.array([[1.0, 0.0, 0.0], [500.0, 0.0, 0.0]])
        frame = RawPointCloudFrame(frame_id=0, sensor_id="s", points=original.copy())
        preprocessor(roi_x_max_m=100.0).run(frame)

        assert np.array_equal(frame.points, original)


class TestConfiguration:
    def test_configuration_changes_change_the_result(self) -> None:
        frame = make_raw([[50.0, 0.0, 0.0]])

        assert preprocessor(max_range_m=100.0).run(frame).metrics.output_point_count == 1
        assert preprocessor(max_range_m=10.0).run(frame).metrics.output_point_count == 0

    def test_range_bounds_must_be_ordered(self) -> None:
        from pydantic import ValidationError

        with pytest.raises(ValidationError, match="min_range_m must be <"):
            LiDARSettings(min_range_m=10.0, max_range_m=1.0)

    @pytest.mark.parametrize("axis", ["x", "y", "z"])
    def test_roi_bounds_must_be_ordered(self, axis: str) -> None:
        from pydantic import ValidationError

        with pytest.raises(ValidationError, match=f"roi_{axis}_min_m must be <"):
            LiDARSettings(**{f"roi_{axis}_min_m": 5.0, f"roi_{axis}_max_m": -5.0})

    def test_defaults_are_a_forward_biased_box(self) -> None:
        settings = LiDARSettings()
        assert settings.roi_x_max_m > abs(settings.roi_x_min_m)
        assert settings.roi_y_min_m == -settings.roi_y_max_m
        assert settings.min_range_m < settings.max_range_m


class TestDeterminism:
    def test_repeated_runs_produce_identical_output(self) -> None:
        frame = make_raw(
            [[1.0, 2.0, 0.0], [float("nan"), 0.0, 0.0], [500.0, 0.0, 0.0], [0.1, 0.0, 0.0]]
        )
        processor = preprocessor(roi_x_max_m=100.0, min_range_m=0.5)

        first, second = processor.run(frame), processor.run(frame)

        assert np.array_equal(first.frame.points, second.frame.points)
        assert first.metrics.output_point_count == second.metrics.output_point_count
        assert first.metrics.invalid_point_count == second.metrics.invalid_point_count

    def test_two_instances_with_equal_config_agree(self) -> None:
        frame = make_raw([[1.0, 0.0, 0.0], [500.0, 0.0, 0.0]])
        a = preprocessor(roi_x_max_m=100.0).run(frame)
        b = preprocessor(roi_x_max_m=100.0).run(frame)
        assert np.array_equal(a.frame.points, b.frame.points)


class TestLiDARProcessorContract:
    def test_process_satisfies_the_phase_1_interface(self) -> None:
        frame = make_raw([[1.0, 0.0, 0.0], [float("nan"), 0.0, 0.0]])
        processed = preprocessor().process(frame)  # type: ignore[arg-type]

        assert isinstance(processed, PointCloudFrame)
        assert processed.point_count == 1

    def test_process_accepts_an_already_validated_frame(self) -> None:
        frame = PointCloudFrame(
            frame_id=0, sensor_id="s", points=np.array([[1.0, 0.0, 0.0], [500.0, 0.0, 0.0]])
        )
        processed = preprocessor(roi_x_max_m=100.0).process(frame)
        assert processed.point_count == 1
