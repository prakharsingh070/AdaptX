"""Detection benchmarking (Phase 3).

Measures the cost of object detection on the **non-ground output** of the
processing pipeline, using the same deterministic synthetic datasets and the
same method as the pipeline benchmark: warm-up runs discarded, timed repeats,
median reported with the spread beside it.

Detection is timed separately from processing, because the two answer different
questions - how long does it take to clean a scan, and how long does it take to
find objects in what survives. Reporting a single combined figure would hide
which one dominates.

Every dataset is synthetic and carries no labels, so these figures describe
**speed only**. Nothing here measures whether the detections are correct, and
no such measurement is possible until a labelled dataset exists.
"""

from __future__ import annotations

import statistics
import time

from pydantic import Field

from adaptx.benchmark.baseline import DETECTION_PROFILE, fixed_resolution_settings
from adaptx.benchmark.datasets import (
    SIZE_LADDER,
    DatasetDescription,
    DatasetScenario,
    generate_dataset,
)
from adaptx.benchmark.models import TimingSummary
from adaptx.benchmark.runner import DEFAULT_REPEATS, DEFAULT_WARMUP, describe_environment
from adaptx.config.settings import DetectionSettings, LiDARSettings
from adaptx.models.common import AdaptXModel, TimestampedModel
from adaptx.models.detection import DetectionConfiguration
from adaptx.perception.detector import GeometricObjectDetector
from adaptx.perception.pipeline import LiDARProcessingPipeline


class DetectionBenchmarkResult(AdaptXModel):
    """One dataset processed and then run through detection."""

    dataset: DatasetDescription
    configuration: DetectionConfiguration
    profile: str = Field(min_length=1)

    input_point_count: int = Field(ge=0, description="Points in the raw frame.")
    processed_point_count: int = Field(
        ge=0, description="Non-ground points the detector actually received."
    )
    cluster_count: int = Field(ge=0)
    object_count: int = Field(ge=0)
    rejected_count: int = Field(ge=0)

    processing_timing: TimingSummary
    detection_timing: TimingSummary
    clustering_median_ms: float = Field(ge=0.0)

    counts_by_class: dict[str, int] = Field(default_factory=dict)
    unavailable: list[str] = Field(default_factory=list)

    @property
    def detection_points_per_second(self) -> float | None:
        """Non-ground points clustered per second, from the median duration."""
        if self.processed_point_count == 0 or self.detection_timing.median_ms <= 0.0:
            return None
        return self.processed_point_count / (self.detection_timing.median_ms / 1000.0)

    @property
    def total_median_ms(self) -> float:
        """Processing plus detection, both medians. Not a latency guarantee."""
        return self.processing_timing.median_ms + self.detection_timing.median_ms


class DetectionBenchmarkReport(TimestampedModel):
    """A set of detection results measured together."""

    results: list[DetectionBenchmarkResult] = Field(default_factory=list)
    warmup_runs: int = Field(ge=0)
    notes: str = Field(
        default=(
            "All datasets are synthetic and unlabelled. These figures measure "
            "detection speed only. They say nothing about detection accuracy, "
            "which cannot be measured without ground truth."
        )
    )


def run_detection_benchmark(
    scenarios: tuple[DatasetScenario, ...] = SIZE_LADDER,
    *,
    lidar_settings: LiDARSettings | None = None,
    detection_settings: DetectionSettings | None = None,
    repeats: int = DEFAULT_REPEATS,
    warmup: int = DEFAULT_WARMUP,
) -> DetectionBenchmarkReport:
    """Measure processing and detection across the given datasets."""
    pipeline = LiDARProcessingPipeline(lidar_settings or fixed_resolution_settings())
    detector = GeometricObjectDetector(detection_settings or DetectionSettings())

    results: list[DetectionBenchmarkResult] = []
    for scenario in scenarios:
        frame, description = generate_dataset(scenario)

        for _ in range(warmup):
            detector.detect(pipeline.run(frame).frame)

        processing_ms: list[float] = []
        detection_ms: list[float] = []
        clustering_ms: list[float] = []
        processed = pipeline.run(frame)
        detection = detector.detect(processed.frame)

        for _ in range(repeats):
            started = time.perf_counter()
            processed = pipeline.run(frame)
            processing_ms.append((time.perf_counter() - started) * 1000.0)

            detection = detector.detect(processed.frame)
            detection_ms.append(detection.duration_ms)
            clustering_ms.append(detection.clustering_duration_ms)

        unavailable = ["detection accuracy: no labelled ground truth exists"]
        if processed.frame.point_count == 0:
            unavailable.append("detection_points_per_second: no points reached the detector")

        results.append(
            DetectionBenchmarkResult(
                dataset=description,
                configuration=detector.configuration,
                profile=DETECTION_PROFILE,
                input_point_count=frame.point_count,
                processed_point_count=processed.frame.point_count,
                cluster_count=detection.cluster_count,
                object_count=detection.object_count,
                rejected_count=detection.rejected_count,
                processing_timing=_summarise(processing_ms),
                detection_timing=_summarise(detection_ms),
                clustering_median_ms=statistics.median(clustering_ms),
                counts_by_class=detection.counts_by_class(),
                unavailable=unavailable,
            )
        )

    return DetectionBenchmarkReport(results=results, warmup_runs=warmup)


def _summarise(durations: list[float]) -> TimingSummary:
    return TimingSummary(
        repeats=len(durations),
        median_ms=statistics.median(durations),
        mean_ms=statistics.fmean(durations),
        min_ms=min(durations),
        max_ms=max(durations),
        stdev_ms=statistics.stdev(durations) if len(durations) > 1 else None,
    )


def render(report: DetectionBenchmarkReport) -> str:
    """A readable summary of a detection benchmark run."""
    environment = describe_environment()
    lines = [
        "ADAPT-X detection benchmark",
        "=" * 104,
        f"adaptx {environment.adaptx_version} | python {environment.python_version} "
        f"| numpy {environment.numpy_version}",
        f"{environment.platform} | {environment.cpu_count} logical CPUs",
        f"warm-up runs discarded: {report.warmup_runs}",
        "",
        "ALL DATASETS ARE SYNTHETIC AND UNLABELLED. These figures measure detection",
        "SPEED only. Detection accuracy is not measured and cannot be, without",
        "ground truth.",
        "",
        f"{'scenario':<10}{'raw pts':>10}{'non-ground':>12}{'clusters':>10}{'objects':>9}"
        f"{'rejected':>10}{'process ms':>12}{'detect ms':>11}{'det pts/s':>12}",
        "-" * 104,
    ]
    for result in report.results:
        rate = result.detection_points_per_second
        lines.append(
            f"{result.dataset.scenario.value:<10}{result.input_point_count:>10}"
            f"{result.processed_point_count:>12}{result.cluster_count:>10}"
            f"{result.object_count:>9}{result.rejected_count:>10}"
            f"{result.processing_timing.median_ms:>12.2f}"
            f"{result.detection_timing.median_ms:>11.2f}"
            f"{('n/a' if rate is None else f'{rate:,.0f}'):>12}"
        )

    lines += ["", "Detected classes (geometric baseline, not verified against labels):"]
    for result in report.results:
        counts = result.counts_by_class or {"none": 0}
        rendered = ", ".join(f"{name}={count}" for name, count in sorted(counts.items()))
        lines.append(f"  {result.dataset.scenario.value:<10}{rendered}")
    return "\n".join(lines)
