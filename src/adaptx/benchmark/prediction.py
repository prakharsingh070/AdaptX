"""Trajectory prediction benchmarking (Phase 5).

Measures the predictor on its own, fed synthetic tracks rather than a real
point cloud. Deliberate, for the same reason the tracking benchmark is:
prediction cost scales with the **number of tracks and the points per
trajectory**, not with points in the scene, and running the LiDAR pipeline
first would bury a sub-millisecond prediction pass under hundreds of
milliseconds of clustering.

Every track is given a measured velocity, so every track is eligible and the
benchmark measures the full extrapolation path rather than a lucky run of
skips.

Speed only. Nothing here measures whether the predicted paths are *correct*:
that would need labelled trajectories, which do not exist. A constant-velocity
extrapolation of a vehicle that then brakes or turns is wrong, and no figure in
this module says otherwise.
"""

from __future__ import annotations

import math
import statistics
import tracemalloc
from datetime import UTC, datetime, timedelta

from pydantic import Field

from adaptx.benchmark.models import TimingSummary
from adaptx.benchmark.runner import DEFAULT_REPEATS, DEFAULT_WARMUP, describe_environment
from adaptx.config.settings import PredictionSettings
from adaptx.models.common import (
    AdaptXModel,
    BoundingBox3D,
    CoordinateFrame,
    DataSource,
    Dimensions,
    ObjectClass,
    TimestampedModel,
    Vector3,
)
from adaptx.models.prediction_result import PredictionConfiguration
from adaptx.models.tracking import TrackedObject, TrackStatus
from adaptx.prediction.constant_velocity import ConstantVelocityPredictor

#: Fixed epoch so every generated scene is reproducible.
EPOCH = datetime(2026, 1, 1, 12, 0, 0, tzinfo=UTC)

#: Track counts benchmarked by default.
TRACK_COUNTS: tuple[int, ...] = (5, 25, 100, 500)


class PredictionBenchmarkCase(AdaptXModel):
    """One track count measured at one prediction configuration."""

    track_count: int = Field(ge=0)
    predicted_track_count: int = Field(ge=0)
    skipped_track_count: int = Field(ge=0)
    points_per_trajectory: int = Field(ge=1)
    predicted_point_count: int = Field(ge=0)

    timing: TimingSummary = Field(description="Whole prediction pass, per call.")

    peak_memory_mb: float | None = Field(
        default=None,
        ge=0.0,
        description=(
            "Peak Python-tracked allocation during one dedicated run, via "
            "tracemalloc. Measured separately from the timed runs because "
            "tracemalloc itself slows execution. None when not measured."
        ),
    )

    configuration: PredictionConfiguration
    unavailable: list[str] = Field(default_factory=list)

    @property
    def tracks_per_second(self) -> float | None:
        """Tracks extrapolated per second, from the median duration."""
        if self.track_count == 0 or self.timing.median_ms <= 0.0:
            return None
        return self.track_count / (self.timing.median_ms / 1000.0)

    @property
    def points_per_second(self) -> float | None:
        """Trajectory points produced per second, from the median duration."""
        if self.predicted_point_count == 0 or self.timing.median_ms <= 0.0:
            return None
        return self.predicted_point_count / (self.timing.median_ms / 1000.0)


class PredictionBenchmarkReport(TimestampedModel):
    """A set of prediction measurements taken together."""

    cases: list[PredictionBenchmarkCase] = Field(default_factory=list)
    warmup_runs: int = Field(ge=0)
    notes: str = Field(
        default=(
            "Synthetic tracks, no labels. These figures measure predictor speed "
            "only and say nothing about prediction correctness, which cannot be "
            "measured without labelled trajectories. Constant velocity is a "
            "baseline motion model, not a claim about how objects move."
        )
    )


def _tracks(count: int) -> list[TrackedObject]:
    """Tracks on a grid, each with a measured constant velocity.

    Spread over distinct positions and speeds so no case degenerates into one
    repeated computation the CPU could cache unrealistically well.
    """
    per_row = max(1, math.ceil(math.sqrt(count)))
    tracks: list[TrackedObject] = []

    for index in range(count):
        row, column = divmod(index, per_row)
        centre = Vector3(x=10.0 + column * 8.0, y=-40.0 + row * 8.0, z=0.0)
        tracks.append(
            TrackedObject(
                timestamp=EPOCH,
                track_id=index,
                object_class=ObjectClass.VEHICLE,
                status=TrackStatus.CONFIRMED,
                position=centre,
                previous_position=centre,
                velocity=Vector3(x=5.0 + (index % 7), y=(index % 3) - 1.0, z=0.0),
                observed_velocity=Vector3(x=5.0 + (index % 7), y=(index % 3) - 1.0, z=0.0),
                bounding_box=BoundingBox3D(
                    center=centre,
                    dimensions=Dimensions(length=4.5, width=1.9, height=1.6),
                    yaw_rad=0.0,
                ),
                point_count=500,
                hits=5,
                first_seen=EPOCH - timedelta(seconds=0.5),
                confidence=0.8,
                age_frames=5,
                missed_frames=0,
                last_seen=EPOCH,
                coordinate_frame=CoordinateFrame.EGO,
                source=DataSource.SYNTHETIC_TEST,
            )
        )
    return tracks


def _measure_peak_memory(
    predictor: ConstantVelocityPredictor, tracks: list[TrackedObject]
) -> float | None:
    """Peak Python-tracked allocation for one pass, in megabytes."""
    if tracemalloc.is_tracing():  # pragma: no cover - nested profiling
        return None
    tracemalloc.start()
    try:
        predictor.predict(tracks, EPOCH)
        _, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
    return peak / (1024.0 * 1024.0)


def run_prediction_benchmark(
    track_counts: tuple[int, ...] = TRACK_COUNTS,
    *,
    settings: PredictionSettings | None = None,
    repeats: int = DEFAULT_REPEATS,
    warmup: int = DEFAULT_WARMUP,
    measure_memory: bool = True,
) -> PredictionBenchmarkReport:
    """Measure prediction cost at each track count."""
    resolved = settings or PredictionSettings(max_tracks=1_000_000)
    predictor = ConstantVelocityPredictor(resolved)
    cases: list[PredictionBenchmarkCase] = []

    for count in track_counts:
        tracks = _tracks(count)

        for _ in range(warmup):
            predictor.predict(tracks, EPOCH)

        durations: list[float] = []
        result = predictor.predict(tracks, EPOCH)
        for _ in range(repeats):
            result = predictor.predict(tracks, EPOCH)
            durations.append(result.duration_ms)

        unavailable = ["prediction correctness: no labelled trajectories exist"]
        peak_memory_mb = _measure_peak_memory(predictor, tracks) if measure_memory else None
        if peak_memory_mb is None:
            unavailable.append("peak_memory_mb: memory measurement was disabled")

        cases.append(
            PredictionBenchmarkCase(
                track_count=count,
                predicted_track_count=result.predicted_track_count,
                skipped_track_count=result.skipped_track_count,
                points_per_trajectory=result.configuration.points_per_trajectory,
                predicted_point_count=result.predicted_point_count,
                timing=_summarise(durations),
                peak_memory_mb=peak_memory_mb,
                configuration=result.configuration,
                unavailable=unavailable,
            )
        )

    return PredictionBenchmarkReport(cases=cases, warmup_runs=warmup)


def _summarise(durations: list[float]) -> TimingSummary:
    return TimingSummary(
        repeats=len(durations),
        median_ms=statistics.median(durations),
        mean_ms=statistics.fmean(durations),
        min_ms=min(durations),
        max_ms=max(durations),
        stdev_ms=statistics.stdev(durations) if len(durations) > 1 else None,
    )


def render(report: PredictionBenchmarkReport) -> str:
    """A readable summary of a prediction benchmark run."""
    environment = describe_environment()
    first = report.cases[0].configuration if report.cases else None
    lines = [
        "ADAPT-X trajectory prediction benchmark",
        "=" * 100,
        f"adaptx {environment.adaptx_version} | python {environment.python_version}",
        f"{environment.platform} | {environment.cpu_count} logical CPUs",
        f"warm-up passes discarded: {report.warmup_runs}",
        (
            f"model: constant_velocity | horizon {first.horizon_s}s "
            f"| interval {first.interval_s}s | {first.points_per_trajectory} points/track"
            if first is not None
            else "no cases measured"
        ),
        "",
        "SYNTHETIC TRACKS, NO LABELS. Speed only - prediction correctness is",
        "not measured and cannot be without labelled trajectories.",
        "",
        f"{'tracks':>8}{'predicted':>11}{'skipped':>9}{'points':>9}"
        f"{'median ms':>12}{'spread ms':>16}{'tracks/s':>12}{'points/s':>13}{'peak MB':>10}",
        "-" * 100,
    ]
    for case in report.cases:
        tracks_rate = case.tracks_per_second
        points_rate = case.points_per_second
        peak = case.peak_memory_mb
        lines.append(
            f"{case.track_count:>8}{case.predicted_track_count:>11}"
            f"{case.skipped_track_count:>9}{case.predicted_point_count:>9}"
            f"{case.timing.median_ms:>12.4f}"
            f"{f'{case.timing.min_ms:.3f}-{case.timing.max_ms:.3f}':>16}"
            f"{('n/a' if tracks_rate is None else f'{tracks_rate:,.0f}'):>12}"
            f"{('n/a' if points_rate is None else f'{points_rate:,.0f}'):>13}"
            f"{('n/a' if peak is None else f'{peak:.2f}'):>10}"
        )
    if report.cases:
        lines += ["", "notes:"]
        for note in report.cases[0].unavailable:
            lines.append(f"  {note}")
    return "\n".join(lines)
