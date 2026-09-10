"""Benchmark record contracts.

Every figure in these models is measured on the machine that ran the benchmark.
Nothing is estimated, and a quantity that could not be measured is ``None``
with the reason recorded, following the same rule as
:class:`~adaptx.models.system.SystemMetrics`.

A record is deliberately self-describing: it carries the dataset, the seed, the
effective pipeline configuration, the software version and the environment, so
a result can be reproduced from what it reports rather than from what someone
remembers about the run.
"""

from __future__ import annotations

from pydantic import Field, model_validator

from adaptx.benchmark.datasets import DatasetDescription
from adaptx.models.common import AdaptXModel, TimestampedModel
from adaptx.models.processing import PipelineConfiguration, StageMetrics


class EnvironmentInfo(AdaptXModel):
    """Where a benchmark ran. Results are only comparable within one of these."""

    adaptx_version: str
    python_version: str
    numpy_version: str
    platform: str
    processor: str = Field(default="", description="May be empty; not all platforms report it.")
    cpu_count: int | None = None


class TimingSummary(AdaptXModel):
    """Aggregated wall-clock timings over the measured repeats.

    The median is the headline figure: it is far less sensitive to a single
    scheduling hiccup than the mean, which matters when repeats are few.
    """

    repeats: int = Field(ge=1)
    median_ms: float = Field(ge=0.0)
    mean_ms: float = Field(ge=0.0)
    min_ms: float = Field(ge=0.0)
    max_ms: float = Field(ge=0.0)
    stdev_ms: float | None = Field(
        default=None, ge=0.0, description="None when only one repeat was measured."
    )

    @model_validator(mode="after")
    def _check_ordering(self) -> TimingSummary:
        if not self.min_ms <= self.median_ms <= self.max_ms:
            raise ValueError(
                f"median_ms ({self.median_ms}) must lie between min_ms ({self.min_ms}) "
                f"and max_ms ({self.max_ms})"
            )
        return self


class BenchmarkResult(TimestampedModel):
    """One dataset processed by one pipeline configuration."""

    dataset: DatasetDescription
    configuration: PipelineConfiguration
    profile: str = Field(
        min_length=1, description="Name of the configuration profile that was run."
    )

    input_point_count: int = Field(ge=0)
    output_point_count: int = Field(ge=0)
    ground_point_count: int = Field(ge=0)
    invalid_point_count: int = Field(ge=0)

    timing: TimingSummary
    #: Per-stage timings from a single representative run, not an average.
    stages: list[StageMetrics] = Field(default_factory=list)
    stage_timing_source: str = Field(
        default="single representative run, not averaged across repeats",
        description="How the per-stage numbers were obtained.",
    )

    peak_memory_mb: float | None = Field(
        default=None,
        ge=0.0,
        description=(
            "Peak Python-tracked allocation during one dedicated run, via "
            "tracemalloc. Measured separately from the timed runs because "
            "tracemalloc itself slows execution. None when not measured."
        ),
    )

    unavailable: list[str] = Field(
        default_factory=list, description="Metrics that could not be measured, and why."
    )

    @property
    def points_per_second(self) -> float | None:
        """Input points processed per second, from the median duration.

        ``None`` for an empty dataset or a zero-length measurement, rather than
        an infinity or a fabricated number.
        """
        if self.input_point_count == 0 or self.timing.median_ms <= 0.0:
            return None
        return self.input_point_count / (self.timing.median_ms / 1000.0)

    @property
    def frames_per_second(self) -> float | None:
        """Frames of *this dataset* one process could push through per second.

        Defined precisely as ``1000 / median_ms`` for sequential single-threaded
        processing on the benchmark machine. This is pipeline throughput only.
        It is **not** end-to-end system FPS, which would also include sensor
        I/O, detection, tracking and mapping - none of which exist yet.
        """
        if self.timing.median_ms <= 0.0:
            return None
        return 1000.0 / self.timing.median_ms

    @property
    def reduction_ratio(self) -> float | None:
        """Fraction of input points removed or separated out.

        ``None`` for an empty dataset. ``1.0`` means nothing reached the output;
        ground points count as removed from the output because they are
        returned separately.
        """
        if self.input_point_count == 0:
            return None
        return 1.0 - (self.output_point_count / self.input_point_count)


class BenchmarkReport(TimestampedModel):
    """A set of results measured together in one session."""

    environment: EnvironmentInfo
    results: list[BenchmarkResult] = Field(default_factory=list)
    warmup_runs: int = Field(ge=0)
    notes: str = Field(
        default=(
            "All datasets are synthetic. These figures measure pipeline speed on "
            "generated geometry and say nothing about perception accuracy or "
            "real-world autonomous-driving performance."
        )
    )
