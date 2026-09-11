"""Contracts of the live simulation session (post-Phase-12 extension).

What the long-lived CARLA loop reports about itself: its state, what it
measured about its own timing, the events it noticed in the pipeline's
outputs, and the scenario it is running. Nothing here is a perception
result and nothing here carries ground truth about other actors.
"""

from __future__ import annotations

from datetime import datetime
from enum import StrEnum

from pydantic import Field, model_validator

from adaptx.control.models import ControlCommand, ControlConfiguration, ControllerState
from adaptx.models.common import AdaptXModel, ObjectClass
from adaptx.models.system import SimulationSessionStatus
from adaptx.models.vehicle import VehicleState


class LiveState(StrEnum):
    """Lifecycle of the live session."""

    IDLE = "IDLE"
    STARTING = "STARTING"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    STOPPING = "STOPPING"
    STOPPED = "STOPPED"
    COLLIDED = "COLLIDED"
    ERROR = "ERROR"


class ActorKind(StrEnum):
    """What a live scenario spawns."""

    VEHICLE = "vehicle"
    PEDESTRIAN = "pedestrian"
    CYCLIST = "cyclist"
    OBSTACLE = "obstacle"


class LiveMotion(AdaptXModel):
    """A constant-velocity segment in the anchor frame (same semantics as Phase 10)."""

    start_s: float = Field(ge=0.0)
    stop_s: float = Field(gt=0.0)
    forward_mps: float = 0.0
    left_mps: float = 0.0

    @model_validator(mode="after")
    def _ordered(self) -> LiveMotion:
        if self.stop_s <= self.start_s:
            raise ValueError("a motion segment must stop after it starts")
        return self

    def displacement_at(self, elapsed_s: float) -> tuple[float, float]:
        """Displacement contributed by this segment after ``elapsed_s`` of the actor's life."""
        active = min(max(elapsed_s, self.start_s), self.stop_s) - self.start_s
        return (self.forward_mps * active, self.left_mps * active)


class SpawnEvent(AdaptXModel):
    """An actor that appears at a session time, placed relative to the ego *then*.

    The placement is anchored to the ego's pose at ``at_s`` and never moves
    with the ego afterwards; motion is scripted in that anchor frame. A
    ``lifetime_s`` removes the actor again, which is how a scenario makes an
    obstacle leave the path.
    """

    actor_id: str = Field(min_length=1)
    kind: ActorKind
    blueprint: str = Field(min_length=1)
    at_s: float = Field(ge=0.0, description="Session time at which the actor appears.")
    forward_m: float
    left_m: float = 0.0
    up_m: float = 0.0
    yaw_offset_deg: float = Field(
        default=0.0, description="Facing relative to the ego heading at spawn; 0 = same way."
    )
    jitter_m: float = Field(default=0.0, ge=0.0, description="Seeded placement jitter.")
    motion: list[LiveMotion] = Field(default_factory=list)
    lifetime_s: float | None = Field(
        default=None, gt=0.0, description="Seconds after spawn at which the actor is removed."
    )

    def expected_offset(self, elapsed_s: float, base: tuple[float, float]) -> tuple[float, float]:
        """Anchor-frame offset after ``elapsed_s`` of life, from a (jittered) base."""
        forward, left = base
        for segment in self.motion:
            d_forward, d_left = segment.displacement_at(elapsed_s)
            forward += d_forward
            left += d_left
        return (forward, left)


class LiveScenarioDefinition(AdaptXModel):
    """A running scene: background traffic plus timed, scripted appearances."""

    scenario_id: str = Field(min_length=1, pattern=r"^[a-z0-9_]+$")
    name: str = Field(min_length=1)
    description: str = Field(min_length=1)
    default_seed: int = Field(ge=0)
    traffic_vehicles: int = Field(default=0, ge=0, le=50)
    traffic_blueprints: list[str] = Field(default_factory=list)
    events: list[SpawnEvent] = Field(default_factory=list)

    @model_validator(mode="after")
    def _unique_actors(self) -> LiveScenarioDefinition:
        ids = [event.actor_id for event in self.events]
        if len(ids) != len(set(ids)):
            raise ValueError("live scenario actor ids must be unique")
        if self.traffic_vehicles and not self.traffic_blueprints:
            raise ValueError("traffic_vehicles > 0 needs traffic_blueprints")
        return self


class LiveScenarioSummary(AdaptXModel):
    """What the dashboard lists."""

    scenario_id: str
    name: str
    description: str
    default_seed: int
    traffic_vehicles: int
    event_count: int


class ActiveActor(AdaptXModel):
    """A scenario actor currently in the world."""

    actor_id: str
    kind: ActorKind
    blueprint: str
    simulator_actor_id: int
    spawned_at_s: float


class LiveTiming(AdaptXModel):
    """Measured wall-clock cost of the last loop iteration.

    Every value is a ``perf_counter`` difference. ``realtime_factor`` is the
    simulation timestep divided by the loop period: 1.0 means the loop keeps
    up with simulated time, below 1.0 it does not, and ``lagging`` says so
    plainly rather than letting a slow loop pass as real-time.
    """

    step_ms: float = Field(ge=0.0, description="tick + wait for the LiDAR frame")
    pipeline_ms: float = Field(ge=0.0, description="Phase 2-8 chain")
    control_ms: float = Field(ge=0.0)
    snapshot_ms: float = Field(
        ge=0.0, description="building and publishing the PREVIOUS frame's snapshot"
    )
    loop_ms: float = Field(ge=0.0, description="whole iteration, wall clock")
    fixed_delta_s: float = Field(gt=0.0)
    realtime_factor: float = Field(ge=0.0)
    lagging: bool
    frames_processed: int = Field(ge=0)
    loop_ms_median: float | None = Field(
        default=None, ge=0.0, description="Median over the last window, once enough frames"
    )
    loop_ms_max: float | None = Field(default=None, ge=0.0)


class LiveFrameInfo(AdaptXModel):
    """What a live snapshot carries about the session that produced it."""

    scenario_id: str
    seed: int
    simulation_frame: int
    simulation_time_s: float = Field(ge=0.0)
    session_time_s: float = Field(ge=0.0, description="Seconds since the session started.")
    controller_state: ControllerState
    collision_count: int = Field(ge=0)
    active_actor_count: int = Field(ge=0)
    traffic_count: int = Field(ge=0)
    timing: LiveTiming


class LiveEvent(AdaptXModel):
    """Something the loop noticed. Derived from outputs and session state only."""

    sequence: int = Field(ge=0)
    kind: str = Field(min_length=1)
    detail: str = Field(min_length=1)
    simulation_time_s: float | None = None
    simulation_frame: int | None = None
    wall_time: datetime
    track_id: int | None = None
    object_class: ObjectClass | None = None


class LiveStatus(AdaptXModel):
    """The live session as the dashboard and the status endpoint see it."""

    state: LiveState
    controls_enabled: bool
    scenario_id: str | None = None
    scenario_name: str | None = None
    seed: int | None = None
    started_at: datetime | None = None
    session_time_s: float | None = None
    frames_processed: int = Field(default=0, ge=0)
    snapshots_published: int = Field(default=0, ge=0)
    simulation: SimulationSessionStatus
    carla_connected: bool
    ego: VehicleState | None = None
    ego_speed_mps: float | None = None
    control: ControlCommand | None = None
    controller_state: ControllerState | None = None
    control_configuration: ControlConfiguration
    timing: LiveTiming | None = None
    collision_count: int = Field(default=0, ge=0)
    active_actors: list[ActiveActor] = Field(default_factory=list)
    traffic_count: int = Field(default=0, ge=0)
    camera_available: bool = False
    last_error: str | None = None
    event_sequence: int = Field(default=0, ge=0, description="Sequence of the newest event.")
    detail: str


__all__ = [
    "ActiveActor",
    "ActorKind",
    "LiveEvent",
    "LiveFrameInfo",
    "LiveMotion",
    "LiveScenarioDefinition",
    "LiveScenarioSummary",
    "LiveState",
    "LiveStatus",
    "LiveTiming",
    "SpawnEvent",
]
