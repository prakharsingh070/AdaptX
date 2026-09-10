"""Benchmark execution.

Method
------
For each dataset the runner performs warm-up runs that are discarded, then a
fixed number of timed repeats, and reports the median. Warm-up matters because
the first pass through a fresh array pays page-fault and cache costs that the
steady state does not, and NumPy resolves some dispatch lazily.

Peak memory is measured in a **separate** dedicated run with ``tracemalloc``
active, never during the timed repeats: tracemalloc instruments every
allocation and materially slows execution, so timing it would report the cost
of measuring rather than the cost of processing.

Timing uses :func:`time.perf_counter` via the pipeline's own measurement, so
the recorded duration is the same number the pipeline reports to any other
caller - there is no separate benchmark-only clock that could drift from what
production code sees.

Honesty
-------
Every dataset is synthetic and labelled as such. Any quantity that could not be
measured is ``None`` with a reason in ``unavailable``; nothing is estimated or
defaulted to a plausible-looking value.
"""

from __future__ import annotations

import platform
import statistics
import tracemalloc

import numpy as np

from adaptx import __version__
from adaptx.benchmark.baseline import BASELINE_PROFILE, fixed_resolution_settings
from adaptx.benchmark.datasets import (
    SIZE_LADDER,
    DatasetScenario,
    generate_dataset,
)
from adaptx.benchmark.models import (
    BenchmarkReport,
    BenchmarkResult,
    EnvironmentInfo,
    TimingSummary,
)
from adaptx.config.settings import LiDARSettings
from adaptx.core.logging import get_logger
from adaptx.perception.pipeline import LiDARProcessingPipeline

logger = get_logger(__name__)

#: Timed repeats per dataset. Enough for a stable median without a long run.
DEFAULT_REPEATS = 7
#: Discarded runs before timing starts.
DEFAULT_WARMUP = 2


def describe_environment() -> EnvironmentInfo:
    """Capture where this benchmark is running."""
    import os

    return EnvironmentInfo(
        adaptx_version=__version__,
        python_version=platform.python_version(),
        numpy_version=np.__version__,
        platform=f"{platform.system()} {platform.release()}",
        processor=platform.processor(),
        cpu_count=os.cpu_count(),
    )


class BenchmarkRunner:
    """Runs the LiDAR pipeline against synthetic datasets and records results."""

    def __init__(
        self,
        settings: LiDARSettings | None = None,
        *,
        profile: str = BASELINE_PROFILE,
        repeats: int = DEFAULT_REPEATS,
        warmup: int = DEFAULT_WARMUP,
        measure_memory: bool = True,
    ) -> None:
        if repeats < 1:
            raise ValueError("repeats must be at least 1")
        if warmup < 0:
            raise ValueError("warmup cannot be negative")

        self._settings = settings if settings is not None else fixed_resolution_settings()
        self._profile = profile
        self._repeats = repeats
        self._warmup = warmup
        self._measure_memory = measure_memory
        self._pipeline = LiDARProcessingPipeline(self._settings)

    @property
    def pipeline(self) -> LiDARProcessingPipeline:
        """The pipeline under measurement."""
        return self._pipeline

    def run_scenario(
        self, scenario: DatasetScenario, *, seed: int | None = None
    ) -> BenchmarkResult:
        """Measure the pipeline against one dataset."""
        frame, description = (
            generate_dataset(scenario, seed=seed)
            if seed is not None
            else generate_dataset(scenario)
        )

        for _ in range(self._warmup):
            self._pipeline.run(frame)

        durations: list[float] = []
        result = self._pipeline.run(frame)
        for _ in range(self._repeats):
            result = self._pipeline.run(frame)
            durations.append(result.metrics.duration_ms)

        unavailable: list[str] = []
        peak_memory_mb = self._measure_peak_memory(frame) if self._measure_memory else None
        if peak_memory_mb is None:
            unavailable.append("peak_memory_mb: memory measurement was disabled")

        metrics = result.metrics
        if metrics.input_point_count == 0:
            unavailable.append("points_per_second, reduction_ratio: dataset contains no points")

        logger.info(
            "benchmark scenario complete",
            extra={
                "context": {
                    "scenario": scenario.value,
                    "profile": self._profile,
                    "points": metrics.input_point_count,
                    "median_ms": round(statistics.median(durations), 3),
                }
            },
        )

        return BenchmarkResult(
            dataset=description,
            configuration=self._pipeline.configuration,
            profile=self._profile,
            input_point_count=metrics.input_point_count,
            output_point_count=metrics.output_point_count,
            ground_point_count=metrics.ground_point_count,
            invalid_point_count=metrics.invalid_point_count,
            timing=_summarise(durations),
            stages=list(metrics.stages),
            peak_memory_mb=peak_memory_mb,
            unavailable=unavailable,
        )

    def run_all(self, scenarios: tuple[DatasetScenario, ...] = SIZE_LADDER) -> BenchmarkReport:
        """Measure every scenario and collect the results into one report."""
        return BenchmarkReport(
            environment=describe_environment(),
            warmup_runs=self._warmup,
            results=[self.run_scenario(scenario) for scenario in scenarios],
        )

    def _measure_peak_memory(self, frame: object) -> float | None:
        """Peak Python-tracked allocation for one run, in megabytes.

        Run on its own with tracemalloc active, so its overhead never lands in
        the reported timings.
        """
        if tracemalloc.is_tracing():  # pragma: no cover - nested profiling
            return None
        tracemalloc.start()
        try:
            self._pipeline.run(frame)  # type: ignore[arg-type]
            _, peak = tracemalloc.get_traced_memory()
        finally:
            tracemalloc.stop()
        return peak / (1024.0 * 1024.0)


def _summarise(durations: list[float]) -> TimingSummary:
    """Aggregate the timed repeats."""
    return TimingSummary(
        repeats=len(durations),
        median_ms=statistics.median(durations),
        mean_ms=statistics.fmean(durations),
        min_ms=min(durations),
        max_ms=max(durations),
        stdev_ms=statistics.stdev(durations) if len(durations) > 1 else None,
    )
