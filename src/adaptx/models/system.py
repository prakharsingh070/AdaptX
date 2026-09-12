"""System status and performance-metric contracts.

Two orthogonal axes describe every subsystem:

``readiness``
    Whether the subsystem can serve requests right now (READY / NOT_READY).
``implementation``
    How much of it exists at all (IMPLEMENTED / PARTIAL / PLANNED / MOCK).

Keeping them separate prevents a planned module from ever looking like a
working one in an API response or on the dashboard.

All metric fields are optional. A value that was not measured is ``None`` and
is named in ``unavailable``; ADAPT-X never substitutes an estimate
(``docs/knowledge-base/20-constraints.md``).
"""

from __future__ import annotations

from enum import StrEnum

from pydantic import Field

from adaptx.models.common import AdaptXModel, DataSource, TimestampedModel


class SystemState(StrEnum):
    """Overall backend state."""

    RUNNING = "RUNNING"
    DEGRADED = "DEGRADED"
    ERROR = "ERROR"


class LiDARStatus(StrEnum):
    """State of the LiDAR input channel."""

    CONNECTED = "CONNECTED"
    DISCONNECTED = "DISCONNECTED"
    SIMULATED = "SIMULATED"


class CarlaStatus(StrEnum):
    """State of the CARLA connection."""

    CONNECTED = "CONNECTED"
    DISCONNECTED = "DISCONNECTED"
    ERROR = "ERROR"


class SimulationState(StrEnum):
    """Where a simulation session is in its lifecycle (Phase 9).

    Deliberately separate from :class:`~adaptx.models.system.CarlaStatus`,
    which answers a different question. Three facts about CARLA are
    independent and all three are worth reporting truthfully:

    * is the integration *available* (``client_available``);
    * is a server *connected* (``CarlaStatus``);
    * is a simulation *running* (this).

    Collapsing them into one enum would force a session that has been
    configured but not stepped to claim one of the other two states.

    ``IDLE``
        No session has been opened.
    ``CONFIGURING``
        Applying world settings and spawning actors. Not yet steppable.
    ``READY``
        Actors and sensor in place, waiting for the first tick.
    ``RUNNING``
        At least one frame has been stepped.
    ``STOPPED``
        Closed cleanly; actors destroyed and world settings restored.
    ``ERROR``
        Setup or stepping failed. Cleanup has run; the reason is in ``detail``.
    """

    IDLE = "IDLE"
    CONFIGURING = "CONFIGURING"
    READY = "READY"
    RUNNING = "RUNNING"
    STOPPED = "STOPPED"
    ERROR = "ERROR"


class SimulationSessionStatus(AdaptXModel):
    """Compact description of a simulation session, safe for status and telemetry.

    Counts and identifiers only. **Never point data**: a single LiDAR frame is
    tens of thousands of points, and a status channel is not where frame
    geometry belongs - the same rule every other ADAPT-X summary follows.
    """

    state: SimulationState = SimulationState.IDLE
    map_name: str | None = None
    synchronous_mode: bool = False
    fixed_delta_seconds: float | None = None
    frames_stepped: int = Field(default=0, ge=0)
    simulation_frame: int | None = Field(
        default=None, description="Simulator frame number of the most recent tick."
    )
    simulation_time_s: float | None = Field(
        default=None, ge=0.0, description="Simulator clock at the most recent tick."
    )
    ego_actor_id: int | None = None
    ego_spawn_index: int | None = Field(
        default=None,
        ge=0,
        description=(
            "Index into the map spawn points the ego took: the first that accepted "
            "it. Recorded because index 0 is refused on some maps."
        ),
    )
    server_version: str | None = Field(
        default=None,
        description="Version the CARLA server reported at connect; null when it did not.",
    )
    sensor_actor_id: int | None = None
    actor_count: int = Field(default=0, ge=0, description="Actors this session spawned.")
    last_point_count: int | None = Field(
        default=None, ge=0, description="Points in the most recent LiDAR frame."
    )
    detail: str = ""
    reclaimed_actors: int = Field(
        default=0,
        ge=0,
        description=(
            "Stale ADAPT-X actors (by role_name) an earlier process left on the server "
            "and this session destroyed on open. Zero on a clean server."
        ),
    )


class ComponentReadiness(StrEnum):
    """Whether a subsystem can serve requests."""

    READY = "READY"
    NOT_READY = "NOT_READY"


class ImplementationStatus(StrEnum):
    """How much of a subsystem actually exists."""

    IMPLEMENTED = "IMPLEMENTED"
    PARTIAL = "PARTIAL"
    PLANNED = "PLANNED"
    MOCK = "MOCK"


class ComponentStatus(AdaptXModel):
    """Status of one ADAPT-X subsystem."""

    name: str = Field(min_length=1)
    readiness: ComponentReadiness = ComponentReadiness.NOT_READY
    implementation: ImplementationStatus = ImplementationStatus.PLANNED
    detail: str = Field(default="", description="Plain-language explanation of the current state.")
    #: Phase in which this subsystem is scheduled (see docs/ROADMAP.md).
    phase: int | None = None
    required: bool = Field(
        default=False,
        description=(
            "True when this component must be READY for the backend to report "
            "RUNNING. Subsystems still scheduled for a later phase are not "
            "required: their absence is expected, not a fault."
        ),
    )


class LiDARSourceStatus(AdaptXModel):
    """State of the LiDAR input, derived from actually received frames."""

    status: LiDARStatus = LiDARStatus.DISCONNECTED
    source: DataSource = DataSource.UNAVAILABLE
    last_frame_id: int | None = None
    last_frame_age_s: float | None = None
    frames_received: int = 0
    detail: str = ""


class CarlaConnectionStatus(AdaptXModel):
    """State of the CARLA integration.

    Three independent facts, reported as three fields rather than collapsed
    into one, because any two of them can disagree:

    ``client_available``
        Is the optional ``carla`` package importable at all?
    ``status``
        Is a server actually connected?
    ``simulation``
        Is a deterministic simulation session configured and stepping?

    A machine with the package installed but no server running is
    ``client_available: true``, ``DISCONNECTED``, ``IDLE`` - and saying so is
    more useful than any single word could be.
    """

    status: CarlaStatus = CarlaStatus.DISCONNECTED
    enabled: bool = False
    client_available: bool = Field(
        default=False, description="True when the `carla` Python package is importable."
    )
    is_mock: bool = Field(
        default=False, description="True when an in-process fake simulator is in use."
    )
    host: str | None = None
    port: int | None = None
    world: str | None = None
    detail: str = ""
    simulation: SimulationSessionStatus = Field(
        default_factory=SimulationSessionStatus,
        description=(
            "Phase 9 simulation session: lifecycle state, simulator frame and "
            "clock, actor counts. Counts only - never point data."
        ),
    )


class SystemStatus(TimestampedModel):
    """Aggregated backend status for the API and dashboard."""

    state: SystemState
    version: str
    environment: str
    uptime_s: float = Field(ge=0.0)
    lidar: LiDARSourceStatus
    carla: CarlaConnectionStatus
    components: list[ComponentStatus] = Field(default_factory=list)


class SystemMetrics(TimestampedModel):
    """Measured runtime metrics.

    Every populated value is measured by this process. Anything not measured is
    ``None`` and listed in ``unavailable``.
    """

    fps: float | None = Field(default=None, ge=0.0, description="Measured ingest frame rate.")
    latency_ms: float | None = Field(
        default=None, ge=0.0, description="Mean end-to-end ingest latency over the window."
    )
    processing_time_ms: float | None = Field(
        default=None, ge=0.0, description="Processing time of the most recent frame."
    )
    cpu_percent: float | None = Field(default=None, ge=0.0)
    gpu_percent: float | None = Field(default=None, ge=0.0)
    memory_mb: float | None = Field(default=None, ge=0.0)
    point_count: int | None = Field(
        default=None, ge=0, description="Points in the most recent frame."
    )
    sample_count: int = Field(default=0, ge=0, description="Frames in the measurement window.")
    unavailable: list[str] = Field(
        default_factory=list, description="Metrics that could not be measured, and why not."
    )
