"""Risk assessment benchmarking (Phase 7).

Measures the risk engine on its own, fed synthetic tracks, trajectories and a
real map rather than a full perception run. Deliberate: risk cost scales with
the **number of objects** and the points per trajectory, not with points in the
scene, and running the LiDAR pipeline first would bury a sub-millisecond pass
under hundreds of milliseconds of clustering.

Every track is given a measured velocity and a trajectory, so every object
exercises all three factors - the engine's worst realistic case rather than a
lucky run of dropped factors.

Speed only. Nothing here measures whether an assessment is *correct*: that
would need labelled risk data, which does not exist. The distribution of levels
is reported because it describes the synthetic scene, not because it validates
anything.
"""

from __future__ import annotations

import math
import statistics
import tracemalloc
from datetime import UTC, datetime, timedelta

import numpy as np
from pydantic import Field

from adaptx.benchmark.models import TimingSummary
from adaptx.benchmark.runner import DEFAULT_REPEATS, DEFAULT_WARMUP, describe_environment
from adaptx.config.settings import MapSettings, PredictionSettings, RiskSettings
from adaptx.mapping.grid_mapper import FixedResolutionMapper
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
from adaptx.models.point_cloud import PointCloudFrame
from adaptx.models.prediction import PredictedTrajectory
from adaptx.models.risk import RiskLevel
from adaptx.models.spatial_map import SpatialMap
from adaptx.models.tracking import TrackedObject, TrackStatus
from adaptx.prediction.constant_velocity import ConstantVelocityPredictor
from adaptx.risk.heuristic import HeuristicRiskEngine

#: Fixed epoch so every generated scene is reproducible.
EPOCH = datetime(2026, 1, 1, 12, 0, 0, tzinfo=UTC)

#: Object counts benchmarked by default.
OBJECT_COUNTS: tuple[int, ...] = (5, 25, 100, 500, 1000)


class RiskBenchmarkCase(AdaptXModel):
    """One object count measured at one configuration."""

    object_count: int = Field(ge=0)
    assessed_count: int = Field(ge=0)
    with_trajectory_count: int = Field(ge=0)

    low_count: int = Field(ge=0)
    medium_count: int = Field(ge=0)
    high_count: int = Field(ge=0)
    critical_count: int = Field(ge=0)
    unknown_count: int = Field(ge=0)

    timing: TimingSummary = Field(description="Whole assessment pass, per call.")

    peak_memory_mb: float | None = Field(
        default=None,
        ge=0.0,
        description=(
            "Peak Python-tracked allocation during one dedicated run, via "
            "tracemalloc. Measured separately from the timed runs because "
            "tracemalloc itself slows execution."
        ),
    )
    unavailable: list[str] = Field(default_factory=list)

    @property
    def objects_per_second(self) -> float | None:
        """Objects assessed per second, from the median duration."""
        if self.object_count == 0 or self.timing.median_ms <= 0.0:
            return None
        return self.object_count / (self.timing.median_ms / 1000.0)

    @property
    def per_object_us(self) -> float | None:
        """Median microseconds spent per object."""
        if self.object_count == 0:
            return None
        return (self.timing.median_ms * 1000.0) / self.object_count


class RiskBenchmarkReport(TimestampedModel):
    """A set of risk measurements taken together."""

    cases: list[RiskBenchmarkCase] = Field(default_factory=list)
    warmup_runs: int = Field(ge=0)
    notes: str = Field(
        default=(
            "Synthetic benchmark; not a real-world autonomous-driving "
            "performance claim. These figures measure engine speed only and "
            "say nothing about whether an assessment is correct, which cannot "
            "be measured without labelled risk data. The level distribution "
            "describes the generated scene, not the quality of the engine."
        )
    )


def _scene(count: int) -> tuple[list[TrackedObject], list[PredictedTrajectory], SpatialMap]:
    """Tracks on a ring around the ego, each closing at a different rate.

    Spread over a range of distances so the level distribution spans the whole
    scale rather than collapsing into one band, and every track carries a
    measured velocity so all three factors are exercised.
    """
    tracks: list[TrackedObject] = []
    for index in range(count):
        angle = (index / max(count, 1)) * 2.0 * math.pi
        distance = 3.0 + (index % 40)
        position = Vector3(x=distance * math.cos(angle), y=distance * math.sin(angle), z=0.0)
        # Radially inward at a rate that varies by index, so closing speed
        # spans the configured saturation bound.
        rate = 1.0 + (index % 20)
        velocity = Vector3(x=-rate * math.cos(angle), y=-rate * math.sin(angle), z=0.0)
        tracks.append(
            TrackedObject(
                timestamp=EPOCH,
                track_id=index,
                object_class=ObjectClass.VEHICLE,
                status=TrackStatus.CONFIRMED,
                position=position,
                previous_position=position,
                velocity=velocity,
                observed_velocity=velocity,
                bounding_box=BoundingBox3D(
                    center=position,
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

    # Real Phase 5 output rather than hand-built paths, so the benchmark
    # measures what the engine actually receives in the pipeline.
    # The Phase 5 per-call limit is raised here so every track carries a
    # trajectory: the point of this benchmark is the engine's full three-factor
    # path, not Phase 5's response-bounding behaviour.
    predictor = ConstantVelocityPredictor(PredictionSettings(max_tracks=1_000_000))
    trajectories = predictor.predict(tracks, EPOCH).trajectories

    mapper = FixedResolutionMapper(
        MapSettings(min_x_m=-60.0, max_x_m=60.0, min_y_m=-60.0, max_y_m=60.0, resolution_m=0.5)
    )
    cloud = np.array(
        [[track.position.x, track.position.y, 0.5] for track in tracks], dtype=np.float64
    ).reshape(-1, 3)
    spatial_map = mapper.build(
        PointCloudFrame(
            timestamp=EPOCH,
            frame_id=0,
            sensor_id="synthetic_risk",
            points=cloud,
            coordinate_frame=CoordinateFrame.EGO,
            source=DataSource.SYNTHETIC_TEST,
        )
    )
    return tracks, trajectories, spatial_map


def _measure_peak_memory(
    engine: HeuristicRiskEngine,
    tracks: list[TrackedObject],
    trajectories: list[PredictedTrajectory],
    spatial_map: SpatialMap,
) -> float | None:
    """Peak Python-tracked allocation for one pass, in megabytes."""
    if tracemalloc.is_tracing():  # pragma: no cover - nested profiling
        return None
    tracemalloc.start()
    try:
        engine.assess_many(
            tracks, trajectories=trajectories, spatial_map=spatial_map, timestamp=EPOCH
        )
        _, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
    return peak / (1024.0 * 1024.0)


def run_risk_benchmark(
    object_counts: tuple[int, ...] = OBJECT_COUNTS,
    *,
    settings: RiskSettings | None = None,
    repeats: int = DEFAULT_REPEATS,
    warmup: int = DEFAULT_WARMUP,
    measure_memory: bool = True,
) -> RiskBenchmarkReport:
    """Measure assessment cost at each object count."""
    resolved = settings or RiskSettings(max_assessed_tracks=1_000_000)
    engine = HeuristicRiskEngine(resolved)
    cases: list[RiskBenchmarkCase] = []

    for count in object_counts:
        tracks, trajectories, spatial_map = _scene(count)

        for _ in range(warmup):
            engine.assess_many(
                tracks, trajectories=trajectories, spatial_map=spatial_map, timestamp=EPOCH
            )

        durations: list[float] = []
        result = engine.assess_many(
            tracks, trajectories=trajectories, spatial_map=spatial_map, timestamp=EPOCH
        )
        for _ in range(repeats):
            result = engine.assess_many(
                tracks, trajectories=trajectories, spatial_map=spatial_map, timestamp=EPOCH
            )
            durations.append(result.duration_ms)

        unavailable = ["assessment correctness: no labelled risk data exists"]
        peak = (
            _measure_peak_memory(engine, tracks, trajectories, spatial_map)
            if measure_memory
            else None
        )
        if peak is None:
            unavailable.append("peak_memory_mb: memory measurement was disabled")

        counts = result.counts_by_level()
        cases.append(
            RiskBenchmarkCase(
                object_count=count,
                assessed_count=len(result.scored_assessments),
                with_trajectory_count=sum(
                    1 for a in result.assessments if a.trajectory is not None
                ),
                low_count=counts.get(RiskLevel.LOW.value, 0),
                medium_count=counts.get(RiskLevel.MEDIUM.value, 0),
                high_count=counts.get(RiskLevel.HIGH.value, 0),
                critical_count=counts.get(RiskLevel.CRITICAL.value, 0),
                unknown_count=result.unknown_count,
                timing=_summarise(durations),
                peak_memory_mb=peak,
                unavailable=unavailable,
            )
        )

    return RiskBenchmarkReport(cases=cases, warmup_runs=warmup)


def _summarise(durations: list[float]) -> TimingSummary:
    return TimingSummary(
        repeats=len(durations),
        median_ms=statistics.median(durations),
        mean_ms=statistics.fmean(durations),
        min_ms=min(durations),
        max_ms=max(durations),
        stdev_ms=statistics.stdev(durations) if len(durations) > 1 else None,
    )


def render(report: RiskBenchmarkReport) -> str:
    """A readable summary of a risk benchmark run."""
    environment = describe_environment()
    lines = [
        "ADAPT-X risk assessment benchmark",
        "=" * 104,
        f"adaptx {environment.adaptx_version} | python {environment.python_version}",
        f"{environment.platform} | {environment.cpu_count} logical CPUs",
        f"warm-up passes discarded: {report.warmup_runs}",
        "",
        "SYNTHETIC BENCHMARK; NOT A REAL-WORLD AUTONOMOUS-DRIVING PERFORMANCE",
        "CLAIM. Speed only - assessment correctness is not measured and cannot",
        "be without labelled risk data. The level distribution describes the",
        "generated scene, not the quality of the engine.",
        "",
        f"{'objects':>8}{'scored':>8}{'traj':>7}{'low':>6}{'med':>6}{'high':>6}"
        f"{'crit':>6}{'unk':>6}{'median ms':>12}{'spread ms':>16}"
        f"{'obj/s':>12}{'us/obj':>9}{'peak MB':>9}",
        "-" * 104,
    ]
    for case in report.cases:
        rate = case.objects_per_second
        per_object = case.per_object_us
        peak = case.peak_memory_mb
        lines.append(
            f"{case.object_count:>8}{case.assessed_count:>8}{case.with_trajectory_count:>7}"
            f"{case.low_count:>6}{case.medium_count:>6}{case.high_count:>6}"
            f"{case.critical_count:>6}{case.unknown_count:>6}"
            f"{case.timing.median_ms:>12.3f}"
            f"{f'{case.timing.min_ms:.2f}-{case.timing.max_ms:.2f}':>16}"
            f"{('n/a' if rate is None else f'{rate:,.0f}'):>12}"
            f"{('n/a' if per_object is None else f'{per_object:.1f}'):>9}"
            f"{('n/a' if peak is None else f'{peak:.2f}'):>9}"
        )
    if report.cases:
        lines += ["", "notes:"]
        for note in report.cases[0].unavailable:
            lines.append(f"  {note}")
    return "\n".join(lines)
