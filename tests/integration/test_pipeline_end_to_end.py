"""End-to-end LiDAR pipeline integration and sanity checks (Phase 2C).

Raw frame in, processed output out, across every synthetic scenario including
the deliberately degenerate ones.

These assert *properties* rather than exact point counts. The counts depend on
the interaction of seeded geometry with several thresholds, so pinning them
would produce tests that break whenever a default moves without saying anything
about correctness. What must always hold - conservation, finiteness, ordering,
determinism, metadata - is asserted exactly.
"""

from __future__ import annotations

import itertools

import numpy as np
import pytest

from adaptx.benchmark.baseline import (
    build_baseline_pipeline,
    filter_only_settings,
    fixed_resolution_settings,
)
from adaptx.benchmark.datasets import (
    EDGE_CASES,
    SIZE_LADDER,
    DatasetScenario,
    generate_dataset,
)
from adaptx.config.settings import LiDARSettings
from adaptx.models.point_cloud import PointCloudFrame
from adaptx.models.processing import PointCloudProcessingResult, ProcessingStage
from adaptx.perception.pipeline import LiDARProcessingPipeline

ALL_SCENARIOS = list(DatasetScenario)


def run(scenario: DatasetScenario, **overrides: object) -> PointCloudProcessingResult:
    """Process one synthetic scenario with the baseline profile."""
    settings = fixed_resolution_settings()
    if overrides:
        settings = settings.model_copy(update=dict(overrides))
    frame, _ = generate_dataset(scenario)
    return LiDARProcessingPipeline(settings).run(frame)


class TestEveryScenarioProcesses:
    @pytest.mark.parametrize("scenario", ALL_SCENARIOS)
    def test_pipeline_completes(self, scenario: DatasetScenario) -> None:
        result = run(scenario)
        assert isinstance(result.frame, PointCloudFrame)

    @pytest.mark.parametrize("scenario", ALL_SCENARIOS)
    def test_output_contains_no_invalid_numbers(self, scenario: DatasetScenario) -> None:
        """No stage may emit a NaN or an infinity, whatever the input held."""
        result = run(scenario)
        assert np.isfinite(result.frame.points).all()
        assert result.ground_frame is not None
        assert np.isfinite(result.ground_frame.points).all()

    @pytest.mark.parametrize("scenario", ALL_SCENARIOS)
    def test_counts_partition_the_input(self, scenario: DatasetScenario) -> None:
        metrics = run(scenario).metrics
        accounted = (
            metrics.invalid_point_count
            + metrics.roi_rejected_count
            + metrics.range_rejected_count
            + metrics.voxel_reduced_count
            + metrics.ground_point_count
            + metrics.noise_removed_count
            + metrics.output_point_count
        )
        assert accounted == metrics.input_point_count

    @pytest.mark.parametrize("scenario", ALL_SCENARIOS)
    def test_stage_chain_is_continuous(self, scenario: DatasetScenario) -> None:
        stages = run(scenario).metrics.stages
        for earlier, later in itertools.pairwise(stages):
            assert earlier.output_points == later.input_points

    @pytest.mark.parametrize("scenario", ALL_SCENARIOS)
    def test_timings_are_measured_and_consistent(self, scenario: DatasetScenario) -> None:
        metrics = run(scenario).metrics

        assert metrics.duration_ms > 0.0
        assert all(stage.duration_ms >= 0.0 for stage in metrics.stages)
        # Stages cannot account for more than the whole frame took.
        assert metrics.stage_duration_ms <= metrics.duration_ms
        assert metrics.overhead_ms >= 0.0

    @pytest.mark.parametrize("scenario", ALL_SCENARIOS)
    def test_metadata_survives(self, scenario: DatasetScenario) -> None:
        frame, _ = generate_dataset(scenario)
        result = build_baseline_pipeline().run(frame)

        assert result.frame.frame_id == frame.frame_id
        assert result.frame.sensor_id == frame.sensor_id
        assert result.frame.timestamp == frame.timestamp
        assert result.frame.source is frame.source
        assert result.frame.coordinate_frame is frame.coordinate_frame


class TestEdgeCaseBehaviour:
    def test_empty_input_produces_empty_output(self) -> None:
        result = run(DatasetScenario.EMPTY)
        assert result.metrics.input_point_count == 0
        assert result.frame.point_count == 0
        assert result.metrics.retention_ratio is None

    def test_dense_input_collapses_hard(self) -> None:
        """50k points inside a 1 m cube must reduce to very few voxels."""
        result = run(DatasetScenario.DENSE)
        assert result.metrics.input_point_count == 50_000
        assert result.metrics.voxel_reduced_count > 40_000

    def test_extreme_coordinates_are_filtered_not_crashed_on(self) -> None:
        result = run(DatasetScenario.EXTREME_COORDINATES)
        # The far-away points must be gone; the near-origin ones are below the
        # minimum range. What survives must at least be finite and in range.
        assert np.isfinite(result.frame.points).all()
        if result.frame.point_count:
            distance = np.linalg.norm(result.frame.points[:, :3], axis=1)
            assert (distance <= result.configuration.max_range_m + 1e-9).all()

    def test_all_ground_input_yields_almost_no_non_ground(self) -> None:
        result = run(DatasetScenario.ALL_GROUND)
        assert result.ground_frame is not None
        assert result.metrics.ground_point_count > result.metrics.output_point_count

    def test_no_ground_input_still_reports_a_ground_frame(self) -> None:
        """The documented weakness: with no road beneath it, a raised slab's
        lowest points are classified as ground. Asserted so it stays visible."""
        result = run(DatasetScenario.NO_GROUND)
        assert result.ground_frame is not None
        assert result.metrics.ground_point_count > 0

    def test_noisy_input_loses_points_to_the_noise_filter(self) -> None:
        result = run(DatasetScenario.NOISY, noise_min_neighbors=3, noise_cell_size_m=0.5)
        assert result.metrics.noise_removed_count > 0


class TestVoxelResolutionSweep:
    @pytest.mark.parametrize("voxel_size_m", [0.05, 0.10, 0.20, 0.50])
    def test_every_documented_resolution_works(self, voxel_size_m: float) -> None:
        result = run(DatasetScenario.SMALL, voxel_size_m=voxel_size_m)
        assert result.configuration.voxel_size_m == voxel_size_m
        assert np.isfinite(result.frame.points).all()

    def test_a_coarser_voxel_removes_more(self) -> None:
        fine = run(DatasetScenario.SMALL, voxel_size_m=0.05).metrics
        coarse = run(DatasetScenario.SMALL, voxel_size_m=0.50).metrics
        assert coarse.voxel_reduced_count > fine.voxel_reduced_count


class TestProfiles:
    def test_filter_only_is_faster_to_describe_and_keeps_more(self) -> None:
        """Skipping the 2B stages must leave strictly more points behind."""
        frame, _ = generate_dataset(DatasetScenario.SMALL)
        baseline = build_baseline_pipeline().run(frame)
        filtered = LiDARProcessingPipeline(filter_only_settings()).run(frame)

        assert filtered.metrics.output_point_count > baseline.metrics.output_point_count
        assert filtered.ground_frame is None
        assert len(filtered.metrics.stages) == 4
        assert len(baseline.metrics.stages) == 7

    def test_configuration_travels_with_the_result(self) -> None:
        result = run(DatasetScenario.SMALL)
        configuration = result.configuration

        assert configuration.voxel_enabled is True
        assert configuration.enabled_optional_stages == [
            ProcessingStage.VOXEL_DOWNSAMPLE,
            ProcessingStage.GROUND_SEGMENTATION,
            ProcessingStage.NOISE_FILTER,
        ]


class TestDeterminism:
    @pytest.mark.parametrize("scenario", SIZE_LADDER)
    def test_same_input_same_output(self, scenario: DatasetScenario) -> None:
        pipeline = build_baseline_pipeline()
        frame, _ = generate_dataset(scenario)
        first, second = pipeline.run(frame), pipeline.run(frame)

        assert np.array_equal(first.frame.points, second.frame.points)
        assert first.metrics.output_point_count == second.metrics.output_point_count

    @pytest.mark.parametrize("scenario", EDGE_CASES)
    def test_edge_cases_are_deterministic_too(self, scenario: DatasetScenario) -> None:
        pipeline = build_baseline_pipeline()
        frame, _ = generate_dataset(scenario)
        assert np.array_equal(pipeline.run(frame).frame.points, pipeline.run(frame).frame.points)

    def test_two_pipelines_with_equal_configuration_agree(self) -> None:
        frame, _ = generate_dataset(DatasetScenario.SMALL)
        a = build_baseline_pipeline().run(frame)
        b = build_baseline_pipeline().run(frame)
        assert np.array_equal(a.frame.points, b.frame.points)

    def test_the_input_frame_is_never_mutated(self) -> None:
        frame, _ = generate_dataset(DatasetScenario.SMALL)
        original = frame.points.copy()
        build_baseline_pipeline().run(frame)
        assert np.array_equal(frame.points, original, equal_nan=True)


class TestPointCountLimits:
    def test_a_frame_over_the_limit_is_rejected(self) -> None:
        from adaptx.core.exceptions import InvalidPointCloudError

        settings = LiDARSettings(max_points=100, voxel_enabled=True)
        frame, _ = generate_dataset(DatasetScenario.SMALL)

        with pytest.raises(InvalidPointCloudError, match="maximum"):
            LiDARProcessingPipeline(settings).run(frame)
