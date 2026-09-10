"""Tracking benchmarking (Phase 4).

Measures the tracker on its own, fed synthetic detections rather than a real
point cloud. Deliberate: association cost scales with the **number of objects**,
not with points, and running the LiDAR pipeline first would bury a
sub-millisecond tracker update under hundreds of milliseconds of clustering.
The pipeline and detection benchmarks cover that side.

Scenes are objects on a grid, each moving at a constant velocity, so the number
of tracks is exactly the number requested and every frame produces a full set of
associations - the tracker's worst realistic case rather than a lucky one.

Speed only. Nothing here measures whether the tracks are *correct*: that would
need labelled sequences, which do not exist.
"""

from __future__ import annotations

import math
import statistics
from datetime import UTC, datetime, timedelta

from pydantic import Field

from adaptx.benchmark.models import TimingSummary
from adaptx.benchmark.runner import DEFAULT_REPEATS, DEFAULT_WARMUP, describe_environment
from adaptx.config.settings import TrackingSettings
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
from adaptx.models.objects import DetectedObject
from adaptx.models.tracking_result import TrackingConfiguration
from adaptx.tracking.tracker import GeometricObjectTracker

#: Fixed epoch so every generated sequence is reproducible.
EPOCH = datetime(2026, 1, 1, 12, 0, 0, tzinfo=UTC)

#: Object counts benchmarked by default.
OBJECT_COUNTS: tuple[int, ...] = (5, 25, 100, 500)

#: Seconds between generated frames.
FRAME_INTERVAL_S = 0.1


class TrackingBenchmarkCase(AdaptXModel):
    """One object count measured across a sequence of frames."""

    object_count: int = Field(ge=0)
    frames: int = Field(ge=1)
    detections_per_frame: int = Field(ge=0)

    active_tracks: int = Field(ge=0, description="Live tracks after the final frame.")
    matches_last_frame: int = Field(ge=0)
    unmatched_detections_last_frame: int = Field(ge=0)
    unmatched_tracks_last_frame: int = Field(ge=0)

    update_timing: TimingSummary = Field(description="Whole tracker update, per frame.")
    association_median_ms: float = Field(ge=0.0)

    configuration: TrackingConfiguration
    unavailable: list[str] = Field(default_factory=list)

    @property
    def objects_per_second(self) -> float | None:
        """Objects associated per second, from the median update time."""
        if self.object_count == 0 or self.update_timing.median_ms <= 0.0:
            return None
        return self.object_count / (self.update_timing.median_ms / 1000.0)


class TrackingBenchmarkReport(TimestampedModel):
    """A set of tracking measurements taken together."""

    cases: list[TrackingBenchmarkCase] = Field(default_factory=list)
    warmup_runs: int = Field(ge=0)
    notes: str = Field(
        default=(
            "Synthetic detections, no labels. These figures measure tracker "
            "speed only and say nothing about tracking correctness, which "
            "cannot be measured without labelled sequences."
        )
    )


def _detections(count: int, frame_index: int) -> list[DetectedObject]:
    """Objects on a grid, each drifting at a constant velocity.

    Spacing is wide enough that no two objects compete for one detection, so
    the measurement reflects association cost rather than gate contention.
    """
    timestamp = EPOCH + timedelta(seconds=frame_index * FRAME_INTERVAL_S)
    per_row = max(1, math.ceil(math.sqrt(count)))
    detections: list[DetectedObject] = []

    for index in range(count):
        row, column = divmod(index, per_row)
        drift = frame_index * FRAME_INTERVAL_S * 5.0
        centre = Vector3(
            x=10.0 + column * 8.0 + drift,
            y=-40.0 + row * 8.0,
            z=0.0,
        )
        detections.append(
            DetectedObject(
                timestamp=timestamp,
                object_id=index,
                frame_id=frame_index,
                object_class=ObjectClass.VEHICLE,
                position=centre,
                velocity=None,
                bounding_box=BoundingBox3D(
                    center=centre,
                    dimensions=Dimensions(length=4.5, width=1.9, height=1.6),
                    yaw_rad=0.0,
                ),
                confidence=0.8,
                point_count=500,
                distance_m=centre.magnitude,
                classifier="benchmark",
                coordinate_frame=CoordinateFrame.EGO,
                source=DataSource.SYNTHETIC_TEST,
            )
        )
    return detections


def run_tracking_benchmark(
    object_counts: tuple[int, ...] = OBJECT_COUNTS,
    *,
    settings: TrackingSettings | None = None,
    frames: int = 10,
    repeats: int = DEFAULT_REPEATS,
    warmup: int = DEFAULT_WARMUP,
) -> TrackingBenchmarkReport:
    """Measure tracker update cost at each object count."""
    resolved = settings or TrackingSettings(max_association_distance_m=3.0)
    cases: list[TrackingBenchmarkCase] = []

    for count in object_counts:
        sequence = [_detections(count, index) for index in range(frames)]
        timestamps = [
            EPOCH + timedelta(seconds=index * FRAME_INTERVAL_S) for index in range(frames)
        ]

        for _ in range(warmup):
            warm = GeometricObjectTracker(resolved)
            for detections, timestamp in zip(sequence, timestamps, strict=True):
                warm.update(detections, timestamp)

        durations: list[float] = []
        association: list[float] = []
        result = None
        for _ in range(repeats):
            tracker = GeometricObjectTracker(resolved)
            for detections, timestamp in zip(sequence, timestamps, strict=True):
                result = tracker.update(detections, timestamp)
                durations.append(result.duration_ms)
                association.append(result.association_duration_ms)

        assert result is not None
        cases.append(
            TrackingBenchmarkCase(
                object_count=count,
                frames=frames,
                detections_per_frame=count,
                active_tracks=result.active_track_count,
                matches_last_frame=result.association_count,
                unmatched_detections_last_frame=len(result.unmatched_detection_ids),
                unmatched_tracks_last_frame=len(result.unmatched_track_ids),
                update_timing=_summarise(durations),
                association_median_ms=statistics.median(association),
                configuration=result.configuration,
                unavailable=["tracking correctness: no labelled sequences exist"],
            )
        )

    return TrackingBenchmarkReport(cases=cases, warmup_runs=warmup)


def _summarise(durations: list[float]) -> TimingSummary:
    return TimingSummary(
        repeats=len(durations),
        median_ms=statistics.median(durations),
        mean_ms=statistics.fmean(durations),
        min_ms=min(durations),
        max_ms=max(durations),
        stdev_ms=statistics.stdev(durations) if len(durations) > 1 else None,
    )


def render(report: TrackingBenchmarkReport) -> str:
    """A readable summary of a tracking benchmark run."""
    environment = describe_environment()
    lines = [
        "ADAPT-X tracking benchmark",
        "=" * 100,
        f"adaptx {environment.adaptx_version} | python {environment.python_version}",
        f"{environment.platform} | {environment.cpu_count} logical CPUs",
        f"warm-up sequences discarded: {report.warmup_runs}",
        "",
        "SYNTHETIC DETECTIONS, NO LABELS. Speed only - tracking correctness is",
        "not measured and cannot be without labelled sequences.",
        "",
        f"{'objects':>8}{'tracks':>9}{'matched':>9}{'unm.det':>9}{'unm.trk':>9}"
        f"{'update ms':>12}{'assoc ms':>11}{'spread ms':>16}{'obj/s':>12}",
        "-" * 100,
    ]
    for case in report.cases:
        rate = case.objects_per_second
        lines.append(
            f"{case.object_count:>8}{case.active_tracks:>9}{case.matches_last_frame:>9}"
            f"{case.unmatched_detections_last_frame:>9}{case.unmatched_tracks_last_frame:>9}"
            f"{case.update_timing.median_ms:>12.3f}{case.association_median_ms:>11.3f}"
            f"{f'{case.update_timing.min_ms:.2f}-{case.update_timing.max_ms:.2f}':>16}"
            f"{('n/a' if rate is None else f'{rate:,.0f}'):>12}"
        )
    return "\n".join(lines)
