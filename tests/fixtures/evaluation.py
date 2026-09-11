"""Hand-built run records for evaluation tests (Phase 11).

A :class:`RunBuilder` assembles a :class:`~adaptx.scenarios.result.ScenarioRunResult`
with pipeline outputs from short per-frame specifications, so every metric
can be asserted on a case where the answer is known exactly - a track 0.3 m
from its actor, a trajectory that lands 1 m off, a tile that flips level and
back. Nothing here runs perception; the result contracts are built directly,
with the configuration snapshots taken from real services so they are the
shapes production produces.

Coordinates are the **sensor** frame for pipeline outputs and the **ego**
frame for ground truth, exactly as a real record has them; the builder puts
the configured mount between the two so the evaluator's conversion is
exercised, not bypassed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import UTC, datetime, timedelta

from adaptx.carla.ground_truth import GroundTruthActor, GroundTruthFrame
from adaptx.config.settings import Settings
from adaptx.core.lifecycle import build_context
from adaptx.models.adaptive_map import (
    AdaptiveSpatialMapSummary,
    MappingComparison,
    MappingVariantMetrics,
)
from adaptx.models.adaptive_resolution import (
    LEVEL_RANK,
    ResolutionBudget,
    ResolutionPlan,
    TileDecisionReason,
    TileResolutionDecision,
)
from adaptx.models.common import (
    BoundingBox3D,
    DataSource,
    Dimensions,
    ObjectClass,
    Vector3,
)
from adaptx.models.detection import DetectionResult
from adaptx.models.map import ResolutionLevel
from adaptx.models.objects import DetectedObject
from adaptx.models.prediction import PredictedTrajectory, PredictionStatus, TrajectoryPoint
from adaptx.models.prediction_result import PredictionResult, SkippedTrack
from adaptx.models.resolution import ResolutionDecision, ResolutionSource
from adaptx.models.risk import RiskLevel
from adaptx.models.risk_assessment import RiskAssessment, RiskAssessmentResult
from adaptx.models.spatial_map import MapAccounting, MapBounds, SpatialMapSummary
from adaptx.models.tracking import TrackedObject, TrackStatus
from adaptx.models.tracking_result import TrackingResult
from adaptx.scenarios.models import (
    EgoDefinition,
    Placement,
    ScenarioActor,
    ScenarioDefinition,
    ScenarioState,
)
from adaptx.scenarios.result import (
    PipelineFrameOutputs,
    ScenarioActorRecord,
    ScenarioFrameRecord,
    ScenarioRunResult,
    SensorConfiguration,
)
from adaptx.scenarios.runner import resolve
from tests.fixtures.assessments import assessed, unknown_risk

EPOCH = datetime(2000, 1, 1, tzinfo=UTC)
TIMESTEP_S = 0.05
SENSOR_ID = "sensor.lidar.ray_cast"
MOUNT = Vector3(x=0.0, y=0.0, z=1.8)
BOUNDS = MapBounds(min_x=-20.0, max_x=20.0, min_y=-20.0, max_y=20.0)
TILE_SIZE_M = 10.0  # 4 x 4 = 16 tiles over the bounds
LEVEL_SIZES: dict[ResolutionLevel, float] = {
    ResolutionLevel.LOW: 1.0,
    ResolutionLevel.MEDIUM: 0.5,
    ResolutionLevel.HIGH: 0.2,
    ResolutionLevel.CRITICAL: 0.1,
}
FIXED_RESOLUTION_M = 0.5
_CONTEXT = build_context(
    Settings(
        app={"environment": "development", "debug": True},
        api={"cors_origins": []},
        logging={"level": "WARNING"},
        map={
            "min_x_m": BOUNDS.min_x,
            "max_x_m": BOUNDS.max_x,
            "min_y_m": BOUNDS.min_y,
            "max_y_m": BOUNDS.max_y,
            "resolution_m": FIXED_RESOLUTION_M,
        },
        adaptive={"tile_size_m": TILE_SIZE_M, "min_dwell_frames": 3},
    )
)


@dataclass
class TrackSpec:
    """One live track on one frame, sensor frame."""

    track_id: int
    x: float
    y: float
    z: float = 0.0
    velocity: tuple[float, float] | None = (0.0, 0.0)
    object_class: ObjectClass = ObjectClass.VEHICLE
    status: TrackStatus = TrackStatus.CONFIRMED


@dataclass
class ActorSpec:
    """One ground-truth actor on one frame, ego frame."""

    actor_id: int
    x: float
    y: float
    z: float = 1.8  # ego-frame height of the sensor, so the default actor sits at z=0 there
    object_class: ObjectClass = ObjectClass.VEHICLE
    type_id: str = "vehicle.audi.tt"


@dataclass
class RiskSpec:
    """The assessment of one track. ``score=None`` means UNKNOWN."""

    track_id: int
    level: RiskLevel
    score: float | None


@dataclass
class FrameSpec:
    """Everything one frame carries."""

    tracks: list[TrackSpec] = field(default_factory=list)
    detections: list[tuple[float, float]] | None = None
    actors: list[ActorSpec] = field(default_factory=list)
    trajectories: dict[int, list[tuple[float, float, float]]] = field(default_factory=dict)
    skipped: dict[int, PredictionStatus] = field(default_factory=dict)
    risk: list[RiskSpec] = field(default_factory=list)
    levels: list[ResolutionLevel] | None = None
    fixed_duration_ms: float = 1.0
    adaptive_duration_ms: float = 2.0
    controller_duration_ms: float = 1.0
    within_budget: bool = True
    demoted: int = 0
    holds: dict[int, TileDecisionReason] = field(default_factory=dict)


def tile_bounds(index: int) -> MapBounds:
    """Bounds of tile ``index`` in the 4 x 4 grid, row-major from the min corner."""
    per_row = int(BOUNDS.size_x_m / TILE_SIZE_M)
    row, column = divmod(index, per_row)
    return MapBounds(
        min_x=BOUNDS.min_x + column * TILE_SIZE_M,
        max_x=BOUNDS.min_x + (column + 1) * TILE_SIZE_M,
        min_y=BOUNDS.min_y + row * TILE_SIZE_M,
        max_y=BOUNDS.min_y + (row + 1) * TILE_SIZE_M,
    )


def tile_index_for(x: float, y: float) -> int:
    """The 4 x 4 tile holding sensor-frame ``(x, y)``."""
    per_row = int(BOUNDS.size_x_m / TILE_SIZE_M)
    column = min(int((x - BOUNDS.min_x) // TILE_SIZE_M), per_row - 1)
    row = min(int((y - BOUNDS.min_y) // TILE_SIZE_M), per_row - 1)
    return row * per_row + column


def _tracked(spec: TrackSpec, timestamp: datetime) -> TrackedObject:
    centre = Vector3(x=spec.x, y=spec.y, z=spec.z)
    return TrackedObject(
        timestamp=timestamp,
        track_id=spec.track_id,
        object_class=spec.object_class,
        status=spec.status,
        position=centre,
        velocity=(
            None
            if spec.velocity is None
            else Vector3(x=spec.velocity[0], y=spec.velocity[1], z=0.0)
        ),
        bounding_box=BoundingBox3D(
            center=centre, dimensions=Dimensions(length=4.0, width=2.0, height=1.5), yaw_rad=0.0
        ),
        confidence=0.8,
        hits=3,
        source=DataSource.SIMULATION,
    )


def _detected(index: int, x: float, y: float, frame_id: int, timestamp: datetime) -> DetectedObject:
    centre = Vector3(x=x, y=y, z=0.0)
    return DetectedObject(
        timestamp=timestamp,
        object_id=index,
        frame_id=frame_id,
        object_class=ObjectClass.VEHICLE,
        position=centre,
        bounding_box=BoundingBox3D(
            center=centre, dimensions=Dimensions(length=4.0, width=2.0, height=1.5), yaw_rad=0.0
        ),
        confidence=0.8,
        source=DataSource.SIMULATION,
    )


def _trajectory(
    track_id: int, points: list[tuple[float, float, float]], timestamp: datetime
) -> PredictedTrajectory:
    return PredictedTrajectory(
        timestamp=timestamp,
        track_id=track_id,
        horizon_s=max(offset for offset, _, _ in points),
        timestep_s=0.25,
        points=[
            TrajectoryPoint(
                timestamp=timestamp,
                time_offset_s=offset,
                position=Vector3(x=x, y=y, z=0.0),
                confidence=0.8,
            )
            for offset, x, y in points
        ],
        confidence=0.8,
        predictor_name="synthetic",
        status=PredictionStatus.PREDICTED,
        source=DataSource.SIMULATION,
    )


def _assessment(spec: RiskSpec, track: TrackSpec, timestamp: datetime) -> RiskAssessment:
    position = (track.x, track.y, track.z)
    if spec.score is None:
        return unknown_risk(track_id=spec.track_id, position=position, timestamp=timestamp)
    base = assessed(track_id=spec.track_id, position=position, timestamp=timestamp)
    return base.model_copy(update={"risk_level": spec.level, "risk_score": spec.score})


def _plan(
    levels: list[ResolutionLevel],
    previous: list[ResolutionLevel] | None,
    *,
    frame_id: int,
    frame_index: int,
    timestamp: datetime,
    spec: FrameSpec,
) -> ResolutionPlan:
    decisions: list[TileResolutionDecision] = []
    per_row = int(BOUNDS.size_x_m / TILE_SIZE_M)
    for index, level in enumerate(levels):
        size = LEVEL_SIZES[level]
        cells = round(TILE_SIZE_M / size)
        before = None if previous is None else previous[index]
        decisions.append(
            TileResolutionDecision(
                tile_index=index,
                tile_row=index // per_row,
                tile_column=index % per_row,
                bounds=tile_bounds(index),
                level=level,
                resolution=ResolutionDecision(
                    resolution_m=size,
                    source=ResolutionSource.ADAPTIVE,
                    reason="synthetic",
                    requested_by="test",
                ),
                previous_level=before,
                changed=before is not None and before is not level,
                detail_priority=None,
                reasons=[TileDecisionReason.SCORED]
                + ([spec.holds[index]] if index in spec.holds else []),
                reason="synthetic",
                cell_width=cells,
                cell_height=cells,
            )
        )
    total_cells = sum(d.cell_count for d in decisions)
    fine = sum(1 for d in decisions if LEVEL_RANK[d.level] >= LEVEL_RANK[ResolutionLevel.HIGH])
    return ResolutionPlan(
        timestamp=timestamp,
        frame_id=frame_id,
        sensor_id=SENSOR_ID,
        controller="synthetic",
        decisions=decisions,
        excluded=[],
        considered_assessment_count=0,
        influencing_assessment_count=0,
        frame_index=frame_index,
        budget=ResolutionBudget(
            tile_count=len(decisions),
            max_tiles=1000,
            total_cell_count=total_cells,
            max_total_cells=1_000_000,
            fine_tile_count=fine,
            max_fine_tiles=1000,
            demoted_tile_count=spec.demoted,
            within_budget=spec.within_budget,
        ),
        duration_ms=spec.controller_duration_ms,
        configuration=_CONTEXT.adaptive_mapping.configuration,
    )


def _ground_truth(
    actors: list[ActorSpec], frame_id: int, timestamp: datetime, ego_id: int
) -> GroundTruthFrame:
    records = [
        GroundTruthActor(
            actor_id=ego_id,
            type_id="vehicle.tesla.model3",
            object_class=ObjectClass.VEHICLE,
            is_ego=True,
            position=Vector3(),
            velocity=Vector3(),
            heading_rad=0.0,
            dimensions=Dimensions(length=4.7, width=1.9, height=1.5),
            world_position=Vector3(x=100.0, y=50.0, z=0.0),
            distance_m=0.0,
        )
    ]
    for spec in actors:
        records.append(
            GroundTruthActor(
                actor_id=spec.actor_id,
                type_id=spec.type_id,
                object_class=spec.object_class,
                is_ego=False,
                position=Vector3(x=spec.x, y=spec.y, z=spec.z),
                # Deliberately meaningless, as the simulator reports for a
                # placed actor: the evaluator must not use it.
                velocity=Vector3(x=0.0, y=0.0, z=-5.0),
                heading_rad=0.0,
                dimensions=Dimensions(length=4.0, width=2.0, height=1.5),
                world_position=Vector3(x=100.0 + spec.x, y=50.0 + spec.y, z=spec.z),
                distance_m=(spec.x**2 + spec.y**2) ** 0.5,
            )
        )
    return GroundTruthFrame(
        timestamp=timestamp,
        frame_id=frame_id,
        map_name="SyntheticTown",
        actors=sorted(records, key=lambda a: a.actor_id),
        ego_actor_id=ego_id,
    )


class RunBuilder:
    """Assemble a run record frame by frame.

    ``scenario_actors`` maps scenario names to simulator actor ids; only
    those actors are scenario actors, any other id in a frame is a map actor.
    """

    def __init__(
        self,
        scenario_actors: dict[str, int],
        *,
        scenario_id: str = "synthetic",
        seed: int = 7,
        first_frame_id: int = 1000,
        ego_id: int = 1,
        mount: Vector3 = MOUNT,
    ) -> None:
        self._scenario_actors = scenario_actors
        self._scenario_id = scenario_id
        self._seed = seed
        self._first_frame_id = first_frame_id
        self._ego_id = ego_id
        self._mount = mount
        self._frames: list[FrameSpec] = []

    def frame(self, spec: FrameSpec) -> RunBuilder:
        """Append one frame."""
        self._frames.append(spec)
        return self

    def build(self, *, state: ScenarioState = ScenarioState.COMPLETED) -> ScenarioRunResult:
        """The record."""
        definition = ScenarioDefinition(
            scenario_id=self._scenario_id,
            name="Synthetic evaluation scenario",
            description="Hand-built for evaluation tests.",
            seed=self._seed,
            duration_s=max(len(self._frames), 2) * TIMESTEP_S,
            fixed_delta_seconds=TIMESTEP_S,
            ego=EgoDefinition(),
            actors=[
                ScenarioActor(
                    actor_id=name,
                    blueprint="vehicle.audi.tt",
                    object_class=ObjectClass.VEHICLE,
                    placement=Placement(forward_m=10.0, left_m=0.0),
                )
                for name in self._scenario_actors
            ],
        )
        records: list[ScenarioFrameRecord] = []
        truths: list[GroundTruthFrame] = []
        previous_levels: list[ResolutionLevel] | None = None
        for index, spec in enumerate(self._frames):
            frame_id = self._first_frame_id + index
            timestamp = EPOCH + timedelta(seconds=index * TIMESTEP_S)
            levels = spec.levels if spec.levels is not None else [ResolutionLevel.LOW] * 16
            outputs, previous_levels = self._outputs(
                spec, levels, previous_levels, frame_id, index, timestamp
            )
            records.append(
                ScenarioFrameRecord(
                    frame_index=index,
                    scenario_time_s=index * TIMESTEP_S,
                    simulator_frame_id=frame_id,
                    timestamp=timestamp,
                    point_count=1000,
                    expected_poses=[],
                    ground_truth_actor_count=len(spec.actors),
                    pipeline=outputs.counts(),
                    outputs=outputs,
                )
            )
            truths.append(_ground_truth(spec.actors, frame_id, timestamp, self._ego_id))
        return ScenarioRunResult(
            timestamp=EPOCH,
            scenario_id=self._scenario_id,
            name=definition.name,
            seed=self._seed,
            state=state,
            resolved=resolve(definition),
            actors=[
                ScenarioActorRecord(
                    actor_id=name, simulator_actor_id=actor_id, blueprint="vehicle.audi.tt"
                )
                for name, actor_id in self._scenario_actors.items()
            ],
            map_name="SyntheticTown",
            simulator_version="synthetic",
            fixed_delta_seconds=TIMESTEP_S,
            sensor=SensorConfiguration(
                channels=32,
                range_m=100.0,
                points_per_second=560_000,
                rotation_frequency_hz=20.0,
                upper_fov_deg=10.0,
                lower_fov_deg=-30.0,
                dropoff_general_rate=0.0,
                mount=self._mount,
                include_intensity=False,
            ),
            frames=records,
            ground_truth=truths,
        )

    def _outputs(
        self,
        spec: FrameSpec,
        levels: list[ResolutionLevel],
        previous: list[ResolutionLevel] | None,
        frame_id: int,
        index: int,
        timestamp: datetime,
    ) -> tuple[PipelineFrameOutputs, list[ResolutionLevel]]:
        tracks = [_tracked(t, timestamp) for t in spec.tracks]
        detection_points = (
            spec.detections if spec.detections is not None else [(t.x, t.y) for t in spec.tracks]
        )
        detection = DetectionResult(
            timestamp=timestamp,
            frame_id=frame_id,
            sensor_id=SENSOR_ID,
            detector="synthetic",
            objects=[
                _detected(i, x, y, frame_id, timestamp) for i, (x, y) in enumerate(detection_points)
            ],
            input_point_count=1000,
            non_ground_point_count=1000,
            cluster_count=len(detection_points),
            duration_ms=1.0,
            configuration=_CONTEXT.detector.configuration,
        )
        tracking = TrackingResult(
            timestamp=timestamp,
            frame_id=frame_id,
            sensor_id=SENSOR_ID,
            tracker="synthetic",
            tracks=tracks,
            detection_count=len(detection_points),
            association_count=len(detection_points),
            duration_ms=1.0,
            configuration=_CONTEXT.tracking.configuration,
        )
        trajectories = [
            _trajectory(track_id, points, timestamp)
            for track_id, points in spec.trajectories.items()
        ]
        skipped = [
            SkippedTrack(track_id=track_id, status=status, reason="synthetic skip")
            for track_id, status in spec.skipped.items()
        ]
        prediction = PredictionResult(
            timestamp=timestamp,
            frame_id=frame_id,
            sensor_id=SENSOR_ID,
            predictor="synthetic",
            model_name="constant_velocity",
            trajectories=trajectories,
            skipped=skipped,
            considered_track_count=len(trajectories) + len(skipped),
            duration_ms=1.0,
            configuration=_CONTEXT.prediction.configuration,
        )
        by_id = {t.track_id: t for t in spec.tracks}
        risk = RiskAssessmentResult(
            timestamp=timestamp,
            frame_id=frame_id,
            sensor_id=SENSOR_ID,
            engine="synthetic",
            assessments=[_assessment(r, by_id[r.track_id], timestamp) for r in spec.risk],
            considered_track_count=len(spec.risk),
            duration_ms=1.0,
            configuration=_CONTEXT.risk.configuration,
        )
        width = int(BOUNDS.size_x_m / FIXED_RESOLUTION_M)
        fixed_cells = width * width
        fixed_map = SpatialMapSummary(
            timestamp=timestamp,
            frame_id=frame_id,
            sensor_id=SENSOR_ID,
            mapper="fixed",
            is_adaptive=False,
            resolution=ResolutionDecision(resolution_m=FIXED_RESOLUTION_M, reason="fixed"),
            bounds=BOUNDS,
            width=width,
            height=width,
            accounting=MapAccounting(
                input_point_count=1000,
                mapped_point_count=900,
                out_of_bounds_point_count=100,
                occupied_cell_count=50,
                total_cell_count=fixed_cells,
            ),
            duration_ms=spec.fixed_duration_ms,
            configuration=_CONTEXT.mapping.configuration,
        )
        plan = _plan(
            levels, previous, frame_id=frame_id, frame_index=index, timestamp=timestamp, spec=spec
        )
        adaptive_cells = plan.total_cell_count
        sizes = [d.resolution_m for d in plan.decisions]
        area = sum(
            d.resolution_m * d.bounds.size_x_m * d.bounds.size_y_m for d in plan.decisions
        ) / sum(d.bounds.size_x_m * d.bounds.size_y_m for d in plan.decisions)
        adaptive_map = AdaptiveSpatialMapSummary(
            timestamp=timestamp,
            frame_id=frame_id,
            sensor_id=SENSOR_ID,
            mapper="adaptive",
            controller="synthetic",
            is_adaptive=True,
            bounds=BOUNDS,
            tile_size_m=TILE_SIZE_M,
            tile_count=len(levels),
            accounting=MapAccounting(
                input_point_count=1000,
                mapped_point_count=900,
                out_of_bounds_point_count=100,
                occupied_cell_count=40,
                total_cell_count=adaptive_cells,
            ),
            tiles_by_level=plan.counts_by_level(),
            cells_by_level=plan.cells_by_level(),
            finest_resolution_m=min(sizes),
            coarsest_resolution_m=max(sizes),
            area_weighted_resolution_m=area,
            changed_tile_count=plan.changed_tile_count,
            grid_bytes=adaptive_cells * 32,
            controller_duration_ms=spec.controller_duration_ms,
            mapping_duration_ms=spec.adaptive_duration_ms,
        )
        comparison = MappingComparison(
            fixed=MappingVariantMetrics(
                variant="fixed",
                mapper="fixed",
                is_adaptive=False,
                uniform_resolution_m=FIXED_RESOLUTION_M,
                finest_resolution_m=FIXED_RESOLUTION_M,
                coarsest_resolution_m=FIXED_RESOLUTION_M,
                total_cell_count=fixed_cells,
                occupied_cell_count=50,
                mapped_point_count=900,
                out_of_bounds_point_count=100,
                grid_bytes=fixed_cells * 32,
                duration_ms=spec.fixed_duration_ms,
            ),
            adaptive=MappingVariantMetrics(
                variant="adaptive",
                mapper="adaptive",
                is_adaptive=True,
                uniform_resolution_m=None,
                finest_resolution_m=min(sizes),
                coarsest_resolution_m=max(sizes),
                total_cell_count=adaptive_cells,
                occupied_cell_count=40,
                mapped_point_count=900,
                out_of_bounds_point_count=100,
                grid_bytes=adaptive_cells * 32,
                duration_ms=spec.adaptive_duration_ms,
            ),
            high_priority_tile_count=0,
            low_priority_tile_count=len(levels),
            cells_on_high_priority_tiles=0,
            cells_on_low_priority_tiles=adaptive_cells,
        )
        outputs = PipelineFrameOutputs(
            detection=detection,
            tracking=tracking,
            prediction=prediction,
            risk=risk,
            fixed_map=fixed_map,
            adaptive_map=adaptive_map,
            plan=plan,
            comparison=comparison,
            processing_ms=5.0,
        )
        return outputs, levels


def levels(*fine: tuple[int, ResolutionLevel]) -> list[ResolutionLevel]:
    """Sixteen LOW tiles with the given ``(index, level)`` overrides."""
    result = [ResolutionLevel.LOW] * 16
    for index, level in fine:
        result[index] = level
    return result


__all__ = [
    "BOUNDS",
    "EPOCH",
    "FIXED_RESOLUTION_M",
    "LEVEL_SIZES",
    "MOUNT",
    "TILE_SIZE_M",
    "TIMESTEP_S",
    "ActorSpec",
    "FrameSpec",
    "RiskSpec",
    "RunBuilder",
    "TrackSpec",
    "levels",
    "tile_bounds",
    "tile_index_for",
]
