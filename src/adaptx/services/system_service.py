"""System status aggregation.

Produces the single view of backend health consumed by the status API, the
telemetry WebSocket and, later, the dashboard.

The component table below is the authoritative statement of what exists. It is
deliberately explicit rather than inferred, so a subsystem cannot appear to
work because an interface for it happens to import.

State rules
-----------
``ERROR``
    A required component is NOT_READY - the backend cannot do its job.
``DEGRADED``
    An optional but enabled component is unavailable, e.g. CARLA is enabled and
    not connected, or the LiDAR feed has gone stale after delivering frames.
``RUNNING``
    Everything required is ready. Components still scheduled for a later phase
    are reported as PLANNED and do not degrade the state.
"""

from __future__ import annotations

import time

from adaptx import __version__
from adaptx.config.settings import Settings
from adaptx.models.system import (
    ComponentReadiness,
    ComponentStatus,
    ImplementationStatus,
    LiDARStatus,
    SystemState,
    SystemStatus,
)
from adaptx.services.carla_service import CarlaService
from adaptx.services.lidar_service import LiDARIngestService


def _declared_components() -> list[ComponentStatus]:
    """Declared implementation state of every ADAPT-X subsystem."""
    return [
        ComponentStatus(
            name="api",
            readiness=ComponentReadiness.READY,
            implementation=ImplementationStatus.IMPLEMENTED,
            detail="FastAPI application, health, status, metrics and ingest routes",
            phase=1,
            required=True,
        ),
        ComponentStatus(
            name="configuration",
            readiness=ComponentReadiness.READY,
            implementation=ImplementationStatus.IMPLEMENTED,
            detail="environment-driven typed settings",
            phase=1,
            required=True,
        ),
        ComponentStatus(
            name="telemetry",
            readiness=ComponentReadiness.READY,
            implementation=ImplementationStatus.IMPLEMENTED,
            detail="WebSocket channel carrying system status and measured metrics",
            phase=1,
            required=True,
        ),
        ComponentStatus(
            name="lidar_ingest",
            readiness=ComponentReadiness.READY,
            implementation=ImplementationStatus.PARTIAL,
            detail=(
                "frame acceptance, validation, bounds and metadata only; the "
                "processing stages are reported separately as lidar_preprocessing"
            ),
            phase=1,
            required=True,
        ),
        ComponentStatus(
            name="lidar_preprocessing",
            readiness=ComponentReadiness.READY,
            implementation=ImplementationStatus.PARTIAL,
            detail=(
                "input validation, NaN/Inf removal, ROI and range filtering, plus "
                "opt-in voxel downsampling, baseline ground segmentation and "
                "baseline noise filtering; no clustering, no coordinate transforms"
            ),
            phase=2,
        ),
        ComponentStatus(
            name="perception",
            readiness=ComponentReadiness.READY,
            implementation=ImplementationStatus.PARTIAL,
            detail=(
                "geometric object detection: grid clustering, size filtering and "
                "baseline classification by dimension bands. No trained model, no "
                "oriented boxes, no velocity, no camera fusion, no semantic "
                "segmentation. Classification is a heuristic and its confidence is "
                "a geometric fit score, not a calibrated probability"
            ),
            phase=3,
        ),
        ComponentStatus(
            name="tracking",
            readiness=ComponentReadiness.READY,
            implementation=ImplementationStatus.PARTIAL,
            detail=(
                "geometric baseline: gated nearest-neighbour association, "
                "velocity measured from frame timestamps, and a track lifecycle. "
                "No learned motion model, no appearance features, no "
                "re-identification - a track retired for missing too many frames "
                "does not come back. Tracking quality is bounded by detection "
                "quality. Trajectory prediction is a separate component"
            ),
            phase=4,
        ),
        ComponentStatus(
            name="mapping",
            readiness=ComponentReadiness.READY,
            implementation=ImplementationStatus.PARTIAL,
            detail=(
                "deterministic frame-local 2.5D fixed-resolution mapping "
                "baseline. One uniform cell size over configured bounds, with "
                "binary occupancy, point counts and min/max/mean height per "
                "cell; an unobserved cell reports null height, never zero. "
                "Every input point is accounted for as mapped or out of "
                "bounds. ADAPTIVE RESOLUTION IS NOT IMPLEMENTED: the mapper "
                "applies a resolution it is given and never chooses one, so no "
                "region receives more detail than another. Nothing accumulates "
                "between frames - this is not a persistent world map, not "
                "SLAM, and there is no localisation, loop closure, sensor "
                "fusion or semantic labelling. Occupancy is binary rather than "
                "probabilistic or temporally fused"
            ),
            phase=6,
        ),
        ComponentStatus(
            name="risk",
            readiness=ComponentReadiness.NOT_READY,
            implementation=ImplementationStatus.PARTIAL,
            detail=(
                "contract plus a proximity-only baseline used for testing; "
                "the ADAPT-X risk engine is not implemented"
            ),
            phase=7,
        ),
        ComponentStatus(
            name="prediction",
            readiness=ComponentReadiness.READY,
            implementation=ImplementationStatus.PARTIAL,
            detail=(
                "deterministic constant-velocity baseline with heuristic "
                "uncertainty. Measured velocity is extrapolated over a "
                "configurable horizon; a track without a measured velocity is "
                "reported as skipped rather than assumed stationary. No "
                "acceleration model, no Kalman filter, no learned model, no "
                "lane or map conditioning, and no interaction between objects. "
                "Uncertainty grows with extrapolation time by a documented "
                "formula and is not a calibrated sigma or probability. "
                "Prediction accuracy is unmeasured: no labelled trajectories "
                "exist. Collision and conflict reasoning belong to the risk "
                "engine, not here"
            ),
            phase=5,
        ),
    ]


class SystemService:
    """Aggregates subsystem state into a :class:`SystemStatus`."""

    def __init__(
        self,
        settings: Settings,
        lidar: LiDARIngestService,
        carla: CarlaService,
    ) -> None:
        self._settings = settings
        self._lidar = lidar
        self._carla = carla
        self._started_monotonic = time.monotonic()
        self._components = _declared_components()

    @property
    def uptime_s(self) -> float:
        """Seconds since the service was constructed."""
        return time.monotonic() - self._started_monotonic

    def components(self) -> list[ComponentStatus]:
        """Copy of the component table."""
        return [component.model_copy() for component in self._components]

    def status(self) -> SystemStatus:
        """Return the aggregated backend status."""
        lidar_status = self._lidar.status()
        carla_status = self._carla.status()
        components = self.components()

        required_failed = any(
            component.required and component.readiness is ComponentReadiness.NOT_READY
            for component in components
        )
        carla_degraded = carla_status.enabled and not self._carla.client.is_connected
        lidar_degraded = (
            lidar_status.status is LiDARStatus.DISCONNECTED and lidar_status.frames_received > 0
        )

        if required_failed:
            state = SystemState.ERROR
        elif carla_degraded or lidar_degraded:
            state = SystemState.DEGRADED
        else:
            state = SystemState.RUNNING

        return SystemStatus(
            state=state,
            version=__version__,
            environment=self._settings.app.environment.value,
            uptime_s=self.uptime_s,
            lidar=lidar_status,
            carla=carla_status,
            components=components,
        )
