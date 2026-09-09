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
    """State of the CARLA integration."""

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
