"""2.5D mapping benchmarking (Phase 6).

Measures the mapper on the **processed** output of the LiDAR pipeline, at
several fixed resolutions, so the cost of spatial resolution is visible.

Unlike detection and tracking - whose cost scales with object and track counts -
mapping cost has two independent drivers, and the benchmark separates them:

* **points**, which are binned once each;
* **cells**, which are allocated and swept regardless of how many points land
  in them. Halving the cell size quadruples the grid.

That second term is the one that matters for the ADAPT-X claim: a uniform fine
map pays for detail everywhere, including where nothing is happening. Measuring
it now gives the adaptive mapper something concrete to be compared against
later (ADR-003).

Speed and workload only. Nothing here measures whether a map is *correct* or
whether any resolution is *appropriate* - the second question is exactly what
adaptive resolution will exist to answer, and it is not implemented.
"""

from __future__ import annotations

import statistics
import tracemalloc

from pydantic import Field

from adaptx.benchmark.baseline import filter_only_settings
from adaptx.benchmark.datasets import SIZE_LADDER, DatasetScenario, generate_dataset
from adaptx.benchmark.models import TimingSummary
from adaptx.benchmark.runner import DEFAULT_REPEATS, DEFAULT_WARMUP, describe_environment
from adaptx.config.settings import MapSettings
from adaptx.mapping.grid_mapper import FixedResolutionMapper
from adaptx.models.common import AdaptXModel, TimestampedModel
from adaptx.models.point_cloud import PointCloudFrame
from adaptx.models.resolution import ResolutionDecision
from adaptx.models.spatial_map import MapBounds
from adaptx.perception.pipeline import LiDARProcessingPipeline

#: Cell sizes swept by default, coarse to fine.
RESOLUTIONS_M: tuple[float, ...] = (1.0, 0.5, 0.25)

#: Map extent used for benchmarking, wide enough to hold the generated scenes.
BENCHMARK_BOUNDS = MapBounds(min_x=-100.0, max_x=100.0, min_y=-100.0, max_y=100.0)


class MappingBenchmarkCase(AdaptXModel):
    """One dataset mapped at one resolution."""

    scenario: DatasetScenario
    resolution_m: float = Field(gt=0.0)

    input_point_count: int = Field(ge=0)
    mapped_point_count: int = Field(ge=0)
    out_of_bounds_point_count: int = Field(ge=0)

    width: int = Field(ge=1)
    height: int = Field(ge=1)
    total_cell_count: int = Field(ge=1)
    occupied_cell_count: int = Field(ge=0)

    timing: TimingSummary = Field(description="Whole mapping pass, per call.")

    peak_memory_mb: float | None = Field(
        default=None,
        ge=0.0,
        description=(
            "Peak Python-tracked allocation during one dedicated run, via "
            "tracemalloc. Measured separately from the timed runs because "
            "tracemalloc itself slows execution. None when not measured."
        ),
    )
    grid_bytes: int = Field(
        ge=0,
        description=(
            "Exact size of the four dense arrays: one int64 and three float64 "
            "per cell. Arithmetic, not a measurement."
        ),
    )

    unavailable: list[str] = Field(default_factory=list)

    @property
    def occupancy_ratio(self) -> float:
        """Fraction of cells holding at least one point."""
        return self.occupied_cell_count / self.total_cell_count

    @property
    def points_per_second(self) -> float | None:
        """Input points binned per second, from the median duration."""
        if self.input_point_count == 0 or self.timing.median_ms <= 0.0:
            return None
        return self.input_point_count / (self.timing.median_ms / 1000.0)

    @property
    def cells_per_second(self) -> float | None:
        """Cells produced per second, from the median duration."""
        if self.timing.median_ms <= 0.0:
            return None
        return self.total_cell_count / (self.timing.median_ms / 1000.0)


class MappingBenchmarkReport(TimestampedModel):
    """A set of mapping measurements taken together."""

    cases: list[MappingBenchmarkCase] = Field(default_factory=list)
    warmup_runs: int = Field(ge=0)
    bounds: MapBounds = BENCHMARK_BOUNDS
    notes: str = Field(
        default=(
            "Synthetic datasets, no labels. These figures measure mapping speed "
            "and grid workload on generated geometry. They say nothing about "
            "map correctness, and nothing about whether a given resolution is "
            "appropriate for a scene - that question belongs to adaptive "
            "resolution, which is not implemented."
        )
    )


def _settings(resolution_m: float) -> MapSettings:
    """Map settings covering the benchmark bounds at ``resolution_m``."""
    return MapSettings(
        min_x_m=BENCHMARK_BOUNDS.min_x,
        max_x_m=BENCHMARK_BOUNDS.max_x,
        min_y_m=BENCHMARK_BOUNDS.min_y,
        max_y_m=BENCHMARK_BOUNDS.max_y,
        resolution_m=resolution_m,
        min_resolution_m=0.05,
        max_resolution_m=5.0,
        max_cells=64_000_000,
    )


def _processed_frame(scenario: DatasetScenario, *, seed: int | None = None) -> PointCloudFrame:
    """Run a generated dataset through the real pipeline, as the mapper expects.

    The mapper consumes Phase 2 output, so the benchmark feeds it Phase 2
    output. Pipeline time is excluded from the measurement; only the mapping
    call is timed.

    The **filter-only** profile is used deliberately. The full baseline profile
    voxelises at 0.2 m, which collapses a 400k-point dataset to a few thousand
    points - a real and useful reduction, but it would leave this benchmark
    measuring mapping at point counts nothing like the ones requested. Phase 2A
    validation, ROI and range filtering still run, so the mapper still receives
    a genuine processed frame.
    """
    frame, _ = (
        generate_dataset(scenario, seed=seed) if seed is not None else generate_dataset(scenario)
    )
    pipeline = LiDARProcessingPipeline(filter_only_settings())
    return pipeline.run(frame).frame


def _measure_peak_memory(
    mapper: FixedResolutionMapper, frame: PointCloudFrame, decision: ResolutionDecision
) -> float | None:
    """Peak Python-tracked allocation for one mapping pass, in megabytes."""
    if tracemalloc.is_tracing():  # pragma: no cover - nested profiling
        return None
    tracemalloc.start()
    try:
        mapper.build(frame, decision)
        _, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
    return peak / (1024.0 * 1024.0)


def run_mapping_benchmark(
    scenarios: tuple[DatasetScenario, ...] = SIZE_LADDER,
    *,
    resolutions_m: tuple[float, ...] = RESOLUTIONS_M,
    repeats: int = DEFAULT_REPEATS,
    warmup: int = DEFAULT_WARMUP,
    measure_memory: bool = True,
) -> MappingBenchmarkReport:
    """Measure mapping cost for each dataset at each resolution."""
    cases: list[MappingBenchmarkCase] = []

    for scenario in scenarios:
        frame = _processed_frame(scenario)
        for resolution_m in resolutions_m:
            mapper = FixedResolutionMapper(_settings(resolution_m))
            decision = ResolutionDecision.fixed(resolution_m)

            for _ in range(warmup):
                mapper.build(frame, decision)

            durations: list[float] = []
            result = mapper.build(frame, decision)
            for _ in range(repeats):
                result = mapper.build(frame, decision)
                durations.append(result.duration_ms)

            unavailable = ["map correctness: no labelled reference map exists"]
            peak = _measure_peak_memory(mapper, frame, decision) if measure_memory else None
            if peak is None:
                unavailable.append("peak_memory_mb: memory measurement was disabled")

            cells = result.accounting.total_cell_count
            cases.append(
                MappingBenchmarkCase(
                    scenario=scenario,
                    resolution_m=resolution_m,
                    input_point_count=result.accounting.input_point_count,
                    mapped_point_count=result.accounting.mapped_point_count,
                    out_of_bounds_point_count=result.accounting.out_of_bounds_point_count,
                    width=result.width,
                    height=result.height,
                    total_cell_count=cells,
                    occupied_cell_count=result.accounting.occupied_cell_count,
                    timing=_summarise(durations),
                    peak_memory_mb=peak,
                    # One int64 plus three float64 arrays, 8 bytes each.
                    grid_bytes=cells * 8 * 4,
                    unavailable=unavailable,
                )
            )

    return MappingBenchmarkReport(cases=cases, warmup_runs=warmup)


def _summarise(durations: list[float]) -> TimingSummary:
    return TimingSummary(
        repeats=len(durations),
        median_ms=statistics.median(durations),
        mean_ms=statistics.fmean(durations),
        min_ms=min(durations),
        max_ms=max(durations),
        stdev_ms=statistics.stdev(durations) if len(durations) > 1 else None,
    )


def render(report: MappingBenchmarkReport) -> str:
    """A readable summary of a mapping benchmark run."""
    environment = describe_environment()
    bounds = report.bounds
    lines = [
        "ADAPT-X 2.5D mapping benchmark",
        "=" * 108,
        f"adaptx {environment.adaptx_version} | python {environment.python_version} "
        f"| numpy {environment.numpy_version}",
        f"{environment.platform} | {environment.cpu_count} logical CPUs",
        f"warm-up runs discarded: {report.warmup_runs}",
        f"bounds: x [{bounds.min_x}, {bounds.max_x}] m, y [{bounds.min_y}, {bounds.max_y}] m",
        "",
        "SYNTHETIC DATASETS, NO LABELS. Speed and grid workload only. These",
        "figures say nothing about map correctness, and nothing about whether a",
        "resolution is appropriate - that is adaptive resolution, not implemented.",
        "",
        f"{'scenario':<12}{'res m':>7}{'points in':>11}{'mapped':>10}{'oob':>8}"
        f"{'grid':>12}{'cells':>10}{'occupied':>10}{'occ %':>8}"
        f"{'median ms':>11}{'pts/s':>12}{'grid MB':>9}",
        "-" * 108,
    ]
    for case in report.cases:
        rate = case.points_per_second
        lines.append(
            f"{case.scenario.value:<12}{case.resolution_m:>7.2f}"
            f"{case.input_point_count:>11}{case.mapped_point_count:>10}"
            f"{case.out_of_bounds_point_count:>8}"
            f"{f'{case.width}x{case.height}':>12}{case.total_cell_count:>10}"
            f"{case.occupied_cell_count:>10}{case.occupancy_ratio * 100:>8.2f}"
            f"{case.timing.median_ms:>11.3f}"
            f"{('n/a' if rate is None else f'{rate:,.0f}'):>12}"
            f"{case.grid_bytes / (1024 * 1024):>9.1f}"
        )

    lines += ["", "Peak tracked memory per case (separate run, tracemalloc):"]
    for case in report.cases:
        peak = case.peak_memory_mb
        lines.append(
            f"  {case.scenario.value:<12}{case.resolution_m:>6.2f} m  "
            f"{('n/a' if peak is None else f'{peak:.1f} MB')}"
        )
    if report.cases:
        lines += ["", "notes:"]
        for note in report.cases[0].unavailable:
            lines.append(f"  {note}")
    return "\n".join(lines)
