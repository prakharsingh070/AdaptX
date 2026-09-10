"""Benchmark system tests (Phase 2C).

Two things matter here and are tested accordingly: that generated datasets are
reproducible, and that measured figures are either real or explicitly absent.
Timing values themselves are asserted only for properties that must hold on any
machine - ordering, positivity, presence - never for magnitudes, which would be
brittle and would not mean anything portable.
"""

from __future__ import annotations

import numpy as np
import pytest

from adaptx.benchmark.baseline import (
    BASELINE_PROFILE,
    BASELINE_VOXEL_SIZE_M,
    build_baseline_pipeline,
    filter_only_settings,
    fixed_resolution_settings,
)
from adaptx.benchmark.datasets import (
    DEFAULT_SEED,
    EDGE_CASES,
    SIZE_LADDER,
    DatasetScenario,
    generate_dataset,
)
from adaptx.benchmark.models import BenchmarkResult, TimingSummary
from adaptx.benchmark.runner import BenchmarkRunner, describe_environment
from adaptx.config.settings import LiDARSettings
from adaptx.models.common import DataSource
from adaptx.models.processing import ProcessingStage


class TestDatasets:
    @pytest.mark.parametrize("scenario", list(DatasetScenario))
    def test_every_scenario_generates(self, scenario: DatasetScenario) -> None:
        frame, description = generate_dataset(scenario)
        assert description.scenario is scenario
        assert description.point_count == frame.point_count

    @pytest.mark.parametrize("scenario", list(DatasetScenario))
    def test_generation_is_reproducible(self, scenario: DatasetScenario) -> None:
        first, _ = generate_dataset(scenario)
        second, _ = generate_dataset(scenario)
        assert np.array_equal(first.points, second.points, equal_nan=True)

    def test_a_different_seed_gives_different_data(self) -> None:
        default, _ = generate_dataset(DatasetScenario.SMALL)
        other, _ = generate_dataset(DatasetScenario.SMALL, seed=DEFAULT_SEED + 1)
        assert not np.array_equal(default.points, other.points, equal_nan=True)

    @pytest.mark.parametrize("scenario", list(DatasetScenario))
    def test_every_frame_is_labelled_synthetic(self, scenario: DatasetScenario) -> None:
        """Benchmark data must never be able to pass as sensor data."""
        frame, description = generate_dataset(scenario)
        assert frame.source is DataSource.SYNTHETIC_TEST
        assert description.synthetic is True
        assert description.ground_truth_available is False

    def test_size_ladder_increases(self) -> None:
        counts = [generate_dataset(s)[0].point_count for s in SIZE_LADDER]
        assert counts == sorted(counts)
        assert counts[0] < counts[-1]

    def test_empty_scenario_has_no_points(self) -> None:
        frame, _ = generate_dataset(DatasetScenario.EMPTY)
        assert frame.point_count == 0

    def test_road_scenes_contain_non_returns(self) -> None:
        """The size ladder includes NaN, so the invalid-removal stage has work."""
        frame, _ = generate_dataset(DatasetScenario.SMALL)
        assert not np.isfinite(frame.points).all()

    def test_edge_cases_are_not_in_the_size_ladder(self) -> None:
        assert set(EDGE_CASES).isdisjoint(SIZE_LADDER)


class TestBaselineProfile:
    def test_baseline_enables_every_stage_at_a_fixed_voxel_size(self) -> None:
        settings = fixed_resolution_settings()
        assert settings.voxel_enabled is True
        assert settings.ground_enabled is True
        assert settings.noise_enabled is True
        assert settings.voxel_size_m == BASELINE_VOXEL_SIZE_M

    def test_voxel_size_is_configurable(self) -> None:
        assert fixed_resolution_settings(voxel_size_m=0.5).voxel_size_m == 0.5

    def test_baseline_preserves_the_sensing_volume_of_its_base(self) -> None:
        base = LiDARSettings(max_range_m=42.0, roi_x_max_m=33.0)
        settings = fixed_resolution_settings(base)
        assert settings.max_range_m == 42.0
        assert settings.roi_x_max_m == 33.0

    def test_filter_only_profile_disables_the_2b_stages(self) -> None:
        settings = filter_only_settings()
        assert settings.voxel_enabled is False
        assert settings.ground_enabled is False
        assert settings.noise_enabled is False

    def test_baseline_pipeline_reports_its_configuration(self) -> None:
        pipeline = build_baseline_pipeline()
        configuration = pipeline.configuration

        assert configuration.voxel_size_m == BASELINE_VOXEL_SIZE_M
        assert configuration.enabled_optional_stages == [
            ProcessingStage.VOXEL_DOWNSAMPLE,
            ProcessingStage.GROUND_SEGMENTATION,
            ProcessingStage.NOISE_FILTER,
        ]

    def test_the_baseline_uses_one_resolution_everywhere(self) -> None:
        """The defining property: no per-region resolution exists yet."""
        pipeline = build_baseline_pipeline()
        frame, _ = generate_dataset(DatasetScenario.SMALL)
        result = pipeline.run(frame)
        assert result.configuration.voxel_size_m == BASELINE_VOXEL_SIZE_M


class TestRunner:
    @staticmethod
    def _runner(**kwargs: object) -> BenchmarkRunner:
        defaults: dict[str, object] = {"repeats": 2, "warmup": 1, "measure_memory": False}
        defaults.update(kwargs)
        return BenchmarkRunner(**defaults)  # type: ignore[arg-type]

    def test_runs_a_scenario_and_records_measured_values(self) -> None:
        result = self._runner().run_scenario(DatasetScenario.SMALL)

        assert result.profile == BASELINE_PROFILE
        assert result.input_point_count > 0
        assert result.timing.repeats == 2
        assert result.timing.median_ms > 0.0

    def test_result_carries_everything_needed_to_reproduce_it(self) -> None:
        result = self._runner().run_scenario(DatasetScenario.SMALL)

        assert result.dataset.seed == DEFAULT_SEED
        assert result.dataset.scenario is DatasetScenario.SMALL
        assert result.configuration.voxel_size_m == BASELINE_VOXEL_SIZE_M
        assert result.timestamp is not None

    def test_per_stage_timings_are_reported(self) -> None:
        result = self._runner().run_scenario(DatasetScenario.SMALL)
        stages = {stage.stage: stage for stage in result.stages}

        assert ProcessingStage.VOXEL_DOWNSAMPLE in stages
        assert ProcessingStage.GROUND_SEGMENTATION in stages
        assert ProcessingStage.NOISE_FILTER in stages
        assert stages[ProcessingStage.VOXEL_DOWNSAMPLE].duration_ms > 0.0

    def test_memory_is_measured_when_requested(self) -> None:
        result = self._runner(measure_memory=True).run_scenario(DatasetScenario.SMALL)
        assert result.peak_memory_mb is not None
        assert result.peak_memory_mb > 0.0

    def test_memory_absence_is_declared_not_faked(self) -> None:
        result = self._runner(measure_memory=False).run_scenario(DatasetScenario.SMALL)
        assert result.peak_memory_mb is None
        assert any("peak_memory_mb" in note for note in result.unavailable)

    def test_empty_dataset_reports_null_rates_rather_than_zero(self) -> None:
        """A rate over no points is undefined, not zero and not infinite."""
        result = self._runner().run_scenario(DatasetScenario.EMPTY)

        assert result.input_point_count == 0
        assert result.points_per_second is None
        assert result.reduction_ratio is None
        assert any("no points" in note for note in result.unavailable)

    def test_run_all_produces_a_report_per_scenario(self) -> None:
        report = self._runner().run_all((DatasetScenario.SMALL, DatasetScenario.EMPTY))

        assert len(report.results) == 2
        assert report.warmup_runs == 1
        assert "synthetic" in report.notes.lower()

    def test_repeats_must_be_positive(self) -> None:
        with pytest.raises(ValueError, match="repeats"):
            BenchmarkRunner(repeats=0)

    def test_warmup_cannot_be_negative(self) -> None:
        with pytest.raises(ValueError, match="warmup"):
            BenchmarkRunner(warmup=-1)

    def test_a_filter_only_run_reports_only_the_2a_stages(self) -> None:
        runner = self._runner(settings=filter_only_settings(), profile="filter_only")
        result = runner.run_scenario(DatasetScenario.SMALL)

        assert [s.stage for s in result.stages] == [
            ProcessingStage.VALIDATION,
            ProcessingStage.INVALID_REMOVAL,
            ProcessingStage.ROI_FILTER,
            ProcessingStage.RANGE_FILTER,
        ]


class TestDerivedRates:
    @staticmethod
    def _result(points: int, median_ms: float, output: int = 0) -> BenchmarkResult:
        frame, description = generate_dataset(DatasetScenario.EMPTY)
        del frame
        return BenchmarkResult(
            dataset=description.model_copy(update={"point_count": points}),
            configuration=build_baseline_pipeline().configuration,
            profile="test",
            input_point_count=points,
            output_point_count=output,
            ground_point_count=0,
            invalid_point_count=0,
            timing=TimingSummary(
                repeats=1,
                median_ms=median_ms,
                mean_ms=median_ms,
                min_ms=median_ms,
                max_ms=median_ms,
            ),
        )

    def test_points_per_second(self) -> None:
        assert self._result(1000, 10.0).points_per_second == pytest.approx(100_000.0)

    def test_frames_per_second_is_the_inverse_of_the_median(self) -> None:
        assert self._result(1000, 20.0).frames_per_second == pytest.approx(50.0)

    def test_reduction_ratio(self) -> None:
        assert self._result(1000, 10.0, output=250).reduction_ratio == pytest.approx(0.75)

    def test_rates_are_none_rather_than_infinite_at_zero_duration(self) -> None:
        result = self._result(1000, 0.0)
        assert result.points_per_second is None
        assert result.frames_per_second is None

    def test_timing_summary_rejects_an_impossible_median(self) -> None:
        from pydantic import ValidationError

        with pytest.raises(ValidationError, match="median_ms"):
            TimingSummary(repeats=3, median_ms=99.0, mean_ms=5.0, min_ms=1.0, max_ms=10.0)


class TestEnvironment:
    def test_environment_is_captured(self) -> None:
        environment = describe_environment()
        assert environment.adaptx_version
        assert environment.python_version
        assert environment.numpy_version


class TestAdaptiveBenchmark:
    """The Phase 8 benchmark must measure both variants on identical input."""

    def _report(self) -> object:
        from adaptx.benchmark.adaptive import AdaptiveScenario, run_adaptive_benchmark

        return run_adaptive_benchmark(
            (AdaptiveScenario.OPEN_EMPTY, AdaptiveScenario.MIXED_COMPLEXITY),
            baseline_resolutions_m=(0.5,),
            repeats=1,
            warmup=0,
            measure_memory=False,
        )

    def test_every_scenario_is_defined(self) -> None:
        from adaptx.benchmark.adaptive import AdaptiveScenario, build_scene

        for scenario in AdaptiveScenario:
            scene = build_scene(scenario)
            assert scene.frame.points.shape[0] > 0

    def test_scenes_are_deterministic(self) -> None:
        from adaptx.benchmark.adaptive import AdaptiveScenario, build_scene

        first = build_scene(AdaptiveScenario.MIXED_COMPLEXITY)
        second = build_scene(AdaptiveScenario.MIXED_COMPLEXITY)
        assert np.array_equal(first.frame.points, second.frame.points)

    def test_both_variants_see_the_same_input(self) -> None:
        report = self._report()
        for case in report.cases:  # type: ignore[attr-defined]
            assert case.comparison.fixed.mapped_point_count == (
                case.comparison.adaptive.mapped_point_count
            )

    def test_planning_and_mapping_are_timed_separately(self) -> None:
        """They scale with different things, so one number would hide both."""
        report = self._report()
        for case in report.cases:  # type: ignore[attr-defined]
            assert case.controller_timing.median_ms >= 0.0
            assert case.mapping_timing.median_ms >= 0.0
            assert case.fixed_timing.median_ms >= 0.0

    def test_an_empty_scene_costs_fewer_cells_than_a_finer_uniform_grid(self) -> None:
        from adaptx.benchmark.adaptive import AdaptiveScenario

        report = self._report()
        empty = [
            case
            for case in report.cases  # type: ignore[attr-defined]
            if case.scenario is AdaptiveScenario.OPEN_EMPTY
        ]
        assert empty
        assert all(case.comparison.cell_ratio < 1.0 for case in empty)

    def test_a_busier_scene_costs_more_cells_than_an_empty_one(self) -> None:
        from adaptx.benchmark.adaptive import AdaptiveScenario

        report = self._report()
        by_scenario = {
            case.scenario: case
            for case in report.cases  # type: ignore[attr-defined]
        }
        empty = by_scenario[AdaptiveScenario.OPEN_EMPTY]
        mixed = by_scenario[AdaptiveScenario.MIXED_COMPLEXITY]

        assert (
            mixed.comparison.adaptive.total_cell_count > empty.comparison.adaptive.total_cell_count
        )

    def test_the_report_states_what_it_cannot_measure(self) -> None:
        report = self._report()

        assert "no labels" in report.notes  # type: ignore[attr-defined]
        for case in report.cases:  # type: ignore[attr-defined]
            assert any("no labelled reference map" in note for note in case.unavailable)
            assert any("no labelled risk data" in note for note in case.unavailable)

    def test_it_renders_without_claiming_a_probability(self) -> None:
        from adaptx.benchmark.adaptive import render

        text = render(self._report())  # type: ignore[arg-type]

        assert "SYNTHETIC SCENES" in text
        assert "not a probability of collision" in text
