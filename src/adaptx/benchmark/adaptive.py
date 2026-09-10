"""Adaptive resolution benchmarking (Phase 8).

Measures the controller and the region-adaptive mapper against the Phase 6
fixed-resolution baseline **on identical input**, across scenes built to differ
in exactly the way the policy is supposed to notice.

What this measures, and what it does not
----------------------------------------
Speed, cells and bytes. Nothing here measures whether either map is *correct*
or whether an allocation is *appropriate*: no labelled reference map exists, so
map correctness remains unmeasured and unmeasurable, and no labelled risk data
exists, so nothing can say the priority ordering is right.

What it can honestly show is **structure**: that detail concentrates where the
scene is complex and stays coarse where it is not, and what that costs against
a uniform grid.

The figure to beat
------------------
Experiment 005 measured occupancy falling to 1-16% at 0.25 m: a uniform fine
map spends most of its cells recording that nothing was observed. The adaptive
mapper is only interesting if it spends fewer cells overall *and* puts a larger
share of them where the priority is.

A scenario where the adaptive path costs more is reported exactly as measured.
Two costs are separated throughout, because they behave differently: planning
scales with regions and objects, mapping scales with cells and points.
"""

from __future__ import annotations

import math
import statistics
import tracemalloc
from enum import StrEnum

import numpy as np
from pydantic import Field

from adaptx.benchmark.models import TimingSummary
from adaptx.benchmark.runner import DEFAULT_REPEATS, DEFAULT_WARMUP, describe_environment
from adaptx.config.settings import AdaptiveResolutionSettings, MapSettings
from adaptx.mapping.adaptive_mapper import TiledAdaptiveMapper
from adaptx.mapping.comparison import compare, partition_by_priority
from adaptx.mapping.controller import HeuristicResolutionController
from adaptx.mapping.grid_mapper import FixedResolutionMapper
from adaptx.models.adaptive_map import MappingComparison
from adaptx.models.common import (
    AdaptXModel,
    CoordinateFrame,
    DataSource,
    ObjectClass,
    TimestampedModel,
    Vector3,
)
from adaptx.models.point_cloud import PointCloudFrame
from adaptx.models.prediction import PredictedTrajectory, PredictionStatus, TrajectoryPoint
from adaptx.models.resolution import ResolutionDecision
from adaptx.models.risk import RiskFactors, RiskLevel
from adaptx.models.risk_assessment import (
    AssessmentStatus,
    MapContext,
    MapObservation,
    RiskAssessment,
    UncertaintyBreakdown,
)
from adaptx.models.spatial_map import MapBounds
from adaptx.models.tracking import TrackedObject, TrackStatus

#: Map extent used for benchmarking.
BENCHMARK_BOUNDS = MapBounds(min_x=-60.0, max_x=60.0, min_y=-60.0, max_y=60.0)

#: Uniform cell sizes the adaptive map is compared against.
BASELINE_RESOLUTIONS_M: tuple[float, ...] = (1.0, 0.5, 0.25)

#: Seed for the scatter in generated scenes. Fixed so runs are identical.
SEED = 20260101

#: Ground height below the sensor, matching the other benchmark datasets.
GROUND_Z_M = -1.8


class AdaptiveScenario(StrEnum):
    """Scenes chosen to differ in what the resolution policy should notice."""

    OPEN_EMPTY = "open_empty"
    SINGLE_LOW_RISK_OBJECT = "single_low_risk_object"
    SINGLE_HIGH_RISK_OBJECT = "single_high_risk_object"
    MULTIPLE_OBJECTS = "multiple_objects"
    HIGH_UNCERTAINTY = "high_uncertainty"
    PREDICTED_TRAJECTORY = "predicted_trajectory"
    DENSE_SCENE = "dense_scene"
    MIXED_COMPLEXITY = "mixed_complexity"


class AdaptiveBenchmarkCase(AdaptXModel):
    """One scenario measured against one uniform baseline resolution."""

    scenario: AdaptiveScenario
    baseline_resolution_m: float = Field(gt=0.0)

    object_count: int = Field(ge=0)
    unknown_risk_object_count: int = Field(ge=0)
    trajectory_count: int = Field(ge=0)

    controller_timing: TimingSummary = Field(description="Resolution planning, per call.")
    mapping_timing: TimingSummary = Field(description="Adaptive grid construction, per call.")
    fixed_timing: TimingSummary = Field(description="Baseline grid construction, per call.")

    comparison: MappingComparison
    tiles_by_level: dict[str, int]
    cells_by_level: dict[str, int]
    changed_tile_count: int = Field(ge=0)
    demoted_tile_count: int = Field(ge=0)

    peak_memory_mb: float | None = Field(
        default=None,
        ge=0.0,
        description=(
            "Peak Python-tracked allocation during one dedicated adaptive pass, "
            "via tracemalloc. Measured separately from the timed runs because "
            "tracemalloc itself slows execution. None when not measured."
        ),
    )

    unavailable: list[str] = Field(default_factory=list)

    @property
    def total_adaptive_ms(self) -> float:
        """Planning plus mapping, from the medians."""
        return self.controller_timing.median_ms + self.mapping_timing.median_ms

    @property
    def latency_ratio(self) -> float | None:
        """Adaptive total time divided by the baseline mapping time."""
        if self.fixed_timing.median_ms <= 0.0:
            return None
        return self.total_adaptive_ms / self.fixed_timing.median_ms


class AdaptiveBenchmarkReport(TimestampedModel):
    """A set of adaptive-resolution measurements taken together."""

    cases: list[AdaptiveBenchmarkCase] = Field(default_factory=list)
    warmup_runs: int = Field(ge=0)
    bounds: MapBounds = BENCHMARK_BOUNDS
    notes: str = Field(
        default=(
            "Synthetic scenes, no labels. These figures measure planning and "
            "mapping speed, cell counts and bytes on generated geometry with "
            "hand-specified risk. They say nothing about map correctness, "
            "nothing about whether the allocation is appropriate, and nothing "
            "about real-world autonomous-driving performance. The detail "
            "priority driving the allocation is a heuristic, not a probability."
        )
    )


# -- scene construction ----------------------------------------------------
def _map_settings(resolution_m: float) -> MapSettings:
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


def _adaptive_settings() -> AdaptiveResolutionSettings:
    """Controller settings with budgets wide enough not to bind by default.

    A budget that bound in every scenario would measure the budget rather than
    the policy. The DENSE_SCENE case is where it is expected to matter, and the
    report records demotions when it does.
    """
    return AdaptiveResolutionSettings(
        tile_size_m=10.0,
        max_tiles=4_096,
        max_total_cells=4_000_000,
        max_fine_tiles=64,
    )


def _scene_points(
    rng: np.random.Generator, object_positions: list[tuple[float, float]]
) -> np.ndarray:
    """Ground returns plus a box of returns at each object position."""
    axis = np.arange(-50.0, 50.0, 0.5)
    grid_x, grid_y = np.meshgrid(axis, axis, indexing="ij")
    ground = np.column_stack([grid_x.ravel(), grid_y.ravel(), np.full(grid_x.size, GROUND_Z_M)])

    blocks = [ground]
    for x, y in object_positions:
        count = 400
        blocks.append(
            np.column_stack(
                [
                    rng.uniform(x - 2.0, x + 2.0, count),
                    rng.uniform(y - 1.0, y + 1.0, count),
                    rng.uniform(GROUND_Z_M, GROUND_Z_M + 1.6, count),
                ]
            )
        )
    return np.vstack(blocks)


def _frame(points: np.ndarray) -> PointCloudFrame:
    """Wrap points in a validated frame labelled synthetic."""
    return PointCloudFrame(
        frame_id=1,
        sensor_id="benchmark_lidar",
        points=np.ascontiguousarray(points, dtype=np.float64),
        coordinate_frame=CoordinateFrame.EGO,
        source=DataSource.SYNTHETIC_TEST,
    )


def _track(track_id: int, x: float, y: float, speed: float | None) -> TrackedObject:
    velocity = None if speed is None else Vector3(x=speed, y=0.0, z=0.0)
    return TrackedObject(
        track_id=track_id,
        object_class=ObjectClass.VEHICLE,
        status=TrackStatus.CONFIRMED,
        position=Vector3(x=x, y=y, z=0.0),
        velocity=velocity,
        confidence=0.8,
        hits=5,
        age_frames=5,
        missed_frames=0,
        coordinate_frame=CoordinateFrame.EGO,
        source=DataSource.SYNTHETIC_TEST,
    )


def _assessment(
    track_id: int,
    x: float,
    y: float,
    *,
    risk: float | None,
    uncertainty: float,
    speed: float | None,
) -> RiskAssessment:
    """A hand-specified assessment: the benchmark controls the inputs exactly."""
    unknown = risk is None
    return RiskAssessment(
        track_id=track_id,
        status=AssessmentStatus.INSUFFICIENT_DATA if unknown else AssessmentStatus.ASSESSED,
        risk_level=RiskLevel.UNKNOWN if unknown else RiskLevel.MEDIUM,
        risk_score=risk,
        distance_m=math.hypot(x, y),
        closing_speed_mps=None if speed is None else speed,
        speed_mps=speed,
        track_status=TrackStatus.CONFIRMED,
        object_class=ObjectClass.VEHICLE.value,
        trajectory=None,
        map_context=MapContext(observation=MapObservation.OBSERVED_OCCUPIED),
        uncertainty=UncertaintyBreakdown(
            score=uncertainty,
            reasons=[],
            observation_age_s=0.0,
            is_stale=False,
            velocity_known=speed is not None,
            prediction_available=False,
            track_confidence=0.8,
        ),
        factors=[],
        factor_scores=RiskFactors(),
        reason="benchmark assessment",
        source=DataSource.SYNTHETIC_TEST,
    )


def _trajectory(track_id: int, x: float, y: float, speed: float) -> PredictedTrajectory:
    points = [
        TrajectoryPoint(
            time_offset_s=index * 0.25,
            position=Vector3(x=x + speed * index * 0.25, y=y, z=0.0),
            velocity=Vector3(x=speed, y=0.0, z=0.0),
            confidence=0.8,
            position_uncertainty_m=0.5 + 0.5 * index * 0.25,
        )
        for index in range(13)
    ]
    return PredictedTrajectory(
        track_id=track_id,
        horizon_s=3.0,
        timestep_s=0.25,
        points=points,
        confidence=0.8,
        predictor_name="constant_velocity_v1",
        status=PredictionStatus.PREDICTED,
        observation_age_s=0.0,
        source=DataSource.SYNTHETIC_TEST,
    )


class _Scene:
    """One benchmark scene: geometry plus the perception state describing it."""

    def __init__(
        self,
        frame: PointCloudFrame,
        assessments: list[RiskAssessment],
        tracks: list[TrackedObject],
        trajectories: list[PredictedTrajectory],
    ) -> None:
        self.frame = frame
        self.assessments = assessments
        self.tracks = tracks
        self.trajectories = trajectories


def build_scene(scenario: AdaptiveScenario) -> _Scene:
    """Construct one deterministic scene.

    Risk, uncertainty and speed are stated rather than derived, so a scenario
    isolates the factor it is named for instead of depending on what the Phase 7
    heuristic happens to produce for a given geometry.
    """
    rng = np.random.default_rng(SEED)

    if scenario is AdaptiveScenario.OPEN_EMPTY:
        return _Scene(_frame(_scene_points(rng, [])), [], [], [])

    if scenario is AdaptiveScenario.SINGLE_LOW_RISK_OBJECT:
        positions = [(35.0, 25.0)]
        return _Scene(
            _frame(_scene_points(rng, positions)),
            [_assessment(0, 35.0, 25.0, risk=0.10, uncertainty=0.10, speed=1.0)],
            [_track(0, 35.0, 25.0, 1.0)],
            [],
        )

    if scenario is AdaptiveScenario.SINGLE_HIGH_RISK_OBJECT:
        positions = [(8.0, 2.0)]
        return _Scene(
            _frame(_scene_points(rng, positions)),
            [_assessment(0, 8.0, 2.0, risk=0.95, uncertainty=0.20, speed=12.0)],
            [_track(0, 8.0, 2.0, 12.0)],
            [],
        )

    if scenario is AdaptiveScenario.MULTIPLE_OBJECTS:
        positions = [(10.0, 5.0), (-20.0, 12.0), (25.0, -18.0), (-8.0, -30.0)]
        risks = [0.85, 0.40, 0.25, 0.60]
        return _Scene(
            _frame(_scene_points(rng, positions)),
            [
                _assessment(i, x, y, risk=risks[i], uncertainty=0.25, speed=6.0)
                for i, (x, y) in enumerate(positions)
            ],
            [_track(i, x, y, 6.0) for i, (x, y) in enumerate(positions)],
            [],
        )

    if scenario is AdaptiveScenario.HIGH_UNCERTAINTY:
        # Low risk, badly observed, and two objects that could not be scored at
        # all: the case where uncertainty alone must earn detail.
        positions = [(12.0, 8.0), (-15.0, -10.0), (30.0, 20.0)]
        return _Scene(
            _frame(_scene_points(rng, positions)),
            [
                _assessment(0, 12.0, 8.0, risk=0.15, uncertainty=0.95, speed=None),
                _assessment(1, -15.0, -10.0, risk=None, uncertainty=0.90, speed=None),
                _assessment(2, 30.0, 20.0, risk=None, uncertainty=0.85, speed=None),
            ],
            [_track(i, x, y, None) for i, (x, y) in enumerate(positions)],
            [],
        )

    if scenario is AdaptiveScenario.PREDICTED_TRAJECTORY:
        positions = [(-40.0, 0.0)]
        return _Scene(
            _frame(_scene_points(rng, positions)),
            [_assessment(0, -40.0, 0.0, risk=0.55, uncertainty=0.30, speed=14.0)],
            [_track(0, -40.0, 0.0, 14.0)],
            [_trajectory(0, -40.0, 0.0, 14.0)],
        )

    if scenario is AdaptiveScenario.DENSE_SCENE:
        positions = [(float(x), float(y)) for x in range(-40, 41, 10) for y in range(-40, 41, 20)]
        return _Scene(
            _frame(_scene_points(rng, positions)),
            [
                _assessment(i, x, y, risk=0.70, uncertainty=0.40, speed=8.0)
                for i, (x, y) in enumerate(positions)
            ],
            [_track(i, x, y, 8.0) for i, (x, y) in enumerate(positions)],
            [],
        )

    # MIXED_COMPLEXITY: one busy corner, one quiet object, open space elsewhere.
    positions = [(9.0, 4.0), (12.0, -2.0), (-35.0, 30.0)]
    return _Scene(
        _frame(_scene_points(rng, positions)),
        [
            _assessment(0, 9.0, 4.0, risk=0.92, uncertainty=0.35, speed=11.0),
            _assessment(1, 12.0, -2.0, risk=0.78, uncertainty=0.55, speed=9.0),
            _assessment(2, -35.0, 30.0, risk=0.12, uncertainty=0.10, speed=0.5),
        ],
        [
            _track(0, 9.0, 4.0, 11.0),
            _track(1, 12.0, -2.0, 9.0),
            _track(2, -35.0, 30.0, 0.5),
        ],
        [_trajectory(0, 9.0, 4.0, 11.0)],
    )


# -- measurement -----------------------------------------------------------
def _summarise(durations: list[float]) -> TimingSummary:
    return TimingSummary(
        repeats=len(durations),
        median_ms=statistics.median(durations),
        mean_ms=statistics.fmean(durations),
        min_ms=min(durations),
        max_ms=max(durations),
        stdev_ms=statistics.stdev(durations) if len(durations) > 1 else None,
    )


def _measure_peak_memory(
    controller: HeuristicResolutionController, mapper: TiledAdaptiveMapper, scene: _Scene
) -> float | None:
    """Peak Python-tracked allocation for one adaptive pass, in megabytes."""
    if tracemalloc.is_tracing():  # pragma: no cover - nested profiling
        return None
    tracemalloc.start()
    try:
        plan = controller.plan(
            scene.assessments, tracks=scene.tracks, trajectories=scene.trajectories
        )
        mapper.build(scene.frame, plan)
        _, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
    return peak / (1024.0 * 1024.0)


def run_adaptive_benchmark(
    scenarios: tuple[AdaptiveScenario, ...] = tuple(AdaptiveScenario),
    *,
    baseline_resolutions_m: tuple[float, ...] = BASELINE_RESOLUTIONS_M,
    repeats: int = DEFAULT_REPEATS,
    warmup: int = DEFAULT_WARMUP,
    measure_memory: bool = True,
) -> AdaptiveBenchmarkReport:
    """Measure planning and mapping for each scene against each uniform baseline."""
    cases: list[AdaptiveBenchmarkCase] = []
    adaptive_settings = _adaptive_settings()

    for scenario in scenarios:
        scene = build_scene(scenario)

        for baseline_m in baseline_resolutions_m:
            map_settings = _map_settings(baseline_m)
            controller = HeuristicResolutionController(map_settings, adaptive_settings)
            mapper = TiledAdaptiveMapper(map_settings, adaptive_settings)
            fixed_mapper = FixedResolutionMapper(map_settings)
            decision = ResolutionDecision.fixed(baseline_m)

            for _ in range(warmup):
                warm_plan = controller.plan(
                    scene.assessments, tracks=scene.tracks, trajectories=scene.trajectories
                )
                mapper.build(scene.frame, warm_plan)
                fixed_mapper.build(scene.frame, decision)

            # A fresh controller for the measured runs, so stabilisation state
            # from the warm-up cannot change what is measured. Each repeat then
            # sees the same starting state as the first.
            controller_durations: list[float] = []
            mapping_durations: list[float] = []
            fixed_durations: list[float] = []
            plan = None
            adaptive_map = None
            fixed_map = None

            for _ in range(repeats):
                controller = HeuristicResolutionController(map_settings, adaptive_settings)
                plan = controller.plan(
                    scene.assessments, tracks=scene.tracks, trajectories=scene.trajectories
                )
                adaptive_map = mapper.build(scene.frame, plan)
                fixed_map = fixed_mapper.build(scene.frame, decision)

                controller_durations.append(plan.duration_ms)
                mapping_durations.append(adaptive_map.duration_ms)
                fixed_durations.append(fixed_map.duration_ms)

            assert plan is not None and adaptive_map is not None and fixed_map is not None

            unavailable = [
                "map correctness: no labelled reference map exists",
                "allocation quality: no labelled risk data exists, so nothing "
                "here says the priority ordering is right",
            ]
            peak = (
                _measure_peak_memory(
                    HeuristicResolutionController(map_settings, adaptive_settings), mapper, scene
                )
                if measure_memory
                else None
            )
            if peak is None:
                unavailable.append("peak_memory_mb: memory measurement was disabled")

            cases.append(
                AdaptiveBenchmarkCase(
                    scenario=scenario,
                    baseline_resolution_m=baseline_m,
                    object_count=len(scene.assessments),
                    unknown_risk_object_count=sum(
                        1 for a in scene.assessments if a.risk_score is None
                    ),
                    trajectory_count=len(scene.trajectories),
                    controller_timing=_summarise(controller_durations),
                    mapping_timing=_summarise(mapping_durations),
                    fixed_timing=_summarise(fixed_durations),
                    comparison=compare(fixed_map, adaptive_map),
                    tiles_by_level=adaptive_map.tiles_by_level(),
                    cells_by_level=adaptive_map.cells_by_level(),
                    changed_tile_count=plan.changed_tile_count,
                    demoted_tile_count=plan.budget.demoted_tile_count,
                    peak_memory_mb=peak,
                    unavailable=unavailable,
                )
            )

    return AdaptiveBenchmarkReport(cases=cases, warmup_runs=warmup)


def render(report: AdaptiveBenchmarkReport) -> str:
    """A readable summary of an adaptive-resolution benchmark run."""
    environment = describe_environment()
    bounds = report.bounds
    lines = [
        "ADAPT-X adaptive resolution benchmark",
        "=" * 118,
        f"adaptx {environment.adaptx_version} | python {environment.python_version} "
        f"| numpy {environment.numpy_version}",
        f"{environment.platform} | {environment.cpu_count} logical CPUs",
        f"warm-up runs discarded: {report.warmup_runs}",
        f"bounds: x [{bounds.min_x}, {bounds.max_x}] m, y [{bounds.min_y}, {bounds.max_y}] m",
        "",
        "SYNTHETIC SCENES, NO LABELS. Speed, cells and bytes only. These figures",
        "say nothing about map correctness, and nothing about whether the",
        "allocation is appropriate - no labelled risk data exists. The detail",
        "priority is a heuristic, not a probability of collision.",
        "",
        f"{'scenario':<24}{'base m':>7}{'obj':>5}{'plan ms':>9}{'map ms':>8}"
        f"{'fixed ms':>9}{'adapt cells':>12}{'fixed cells':>12}{'ratio':>7}"
        f"{'MB adapt':>9}{'MB fixed':>9}{'hi cells %':>11}",
        "-" * 118,
    ]
    for case in report.cases:
        comparison = case.comparison
        share = comparison.high_priority_cell_share
        lines.append(
            f"{case.scenario.value:<24}{case.baseline_resolution_m:>7.2f}"
            f"{case.object_count:>5}"
            f"{case.controller_timing.median_ms:>9.2f}"
            f"{case.mapping_timing.median_ms:>8.2f}"
            f"{case.fixed_timing.median_ms:>9.2f}"
            f"{comparison.adaptive.total_cell_count:>12}"
            f"{comparison.fixed.total_cell_count:>12}"
            f"{comparison.cell_ratio:>7.3f}"
            f"{comparison.adaptive.grid_bytes / (1024 * 1024):>9.1f}"
            f"{comparison.fixed.grid_bytes / (1024 * 1024):>9.1f}"
            f"{('n/a' if share is None else f'{share * 100:.1f}'):>11}"
        )

    lines += ["", "Region level distribution per case:"]
    for case in report.cases:
        levels = ", ".join(f"{name}={count}" for name, count in case.tiles_by_level.items())
        demoted = f", demoted={case.demoted_tile_count}" if case.demoted_tile_count else ""
        lines.append(
            f"  {case.scenario.value:<24}{case.baseline_resolution_m:>5.2f} m  {levels}{demoted}"
        )

    lines += ["", "Peak tracked memory per case (separate run, tracemalloc):"]
    for case in report.cases:
        peak = case.peak_memory_mb
        lines.append(
            f"  {case.scenario.value:<24}{case.baseline_resolution_m:>5.2f} m  "
            f"{('n/a' if peak is None else f'{peak:.1f} MB')}"
        )

    if report.cases:
        lines += ["", "notes:"]
        for note in report.cases[0].unavailable:
            lines.append(f"  {note}")
    return "\n".join(lines)


__all__ = [
    "BASELINE_RESOLUTIONS_M",
    "BENCHMARK_BOUNDS",
    "AdaptiveBenchmarkCase",
    "AdaptiveBenchmarkReport",
    "AdaptiveScenario",
    "build_scene",
    "partition_by_priority",
    "render",
    "run_adaptive_benchmark",
]
