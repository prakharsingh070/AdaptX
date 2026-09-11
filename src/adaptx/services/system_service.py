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
                "bounds. This mapper applies a resolution it is given and "
                "never chooses one, so every region receives the same detail; "
                "risk-aware allocation is a separate component, reported as "
                "adaptive_resolution, and this baseline is retained unchanged "
                "for comparison against it. Nothing accumulates "
                "between frames - this is not a persistent world map, not "
                "SLAM, and there is no localisation, loop closure, sensor "
                "fusion or semantic labelling. Occupancy is binary rather than "
                "probabilistic or temporally fused"
            ),
            phase=6,
        ),
        ComponentStatus(
            name="risk",
            readiness=ComponentReadiness.READY,
            implementation=ImplementationStatus.PARTIAL,
            detail=(
                "deterministic heuristic risk and uncertainty baseline. Scores "
                "proximity, rate of approach and predicted approach as a "
                "weighted mean over the factors actually available; a factor "
                "that cannot be computed is dropped, never treated as zero. "
                "THE SCORE IS NOT A PROBABILITY OF COLLISION: it is not "
                "calibrated and has never been validated against labelled risk "
                "data, because none exists. Thresholds are baseline "
                "engineering values, not safety-certified limits. Uncertainty "
                "is heuristic and is reported separately from risk, never "
                "folded into the score. A track that cannot be assessed is "
                "reported UNKNOWN with a null score rather than a fabricated "
                "number. No time-to-collision, no trajectory-map intersection, "
                "no ego planned path, no object interaction, and no spatial "
                "risk field. RISK DOES NOT DECIDE SPATIAL RESOLUTION - that is "
                "a separate decision belonging to the resolution controller "
                "reported as adaptive_resolution, which consumes these "
                "assessments and is never consulted by this engine. The "
                "proximity-only baseline is retained unchanged for comparison"
            ),
            phase=7,
        ),
        ComponentStatus(
            name="adaptive_resolution",
            readiness=ComponentReadiness.READY,
            implementation=ImplementationStatus.PARTIAL,
            detail=(
                "deterministic heuristic adaptive spatial resolution baseline. "
                "The map extent is partitioned into fixed-size regions and a "
                "controller gives each its own cell size, so one map genuinely "
                "holds several resolutions; regions partition the extent "
                "exactly, with no gap and no double coverage. The level comes "
                "from a detail priority combining risk, uncertainty, "
                "predicted-motion relevance, object density, proximity and "
                "measured motion as a weighted mean over the factors actually "
                "available - a factor that cannot be computed is dropped, "
                "never treated as zero. THE DETAIL PRIORITY IS NOT A "
                "PROBABILITY OF COLLISION and is not a safety margin: it is an "
                "engineering prioritisation score, never calibrated and never "
                "validated against labelled data, because none exists. "
                "Uncertainty is consumed as an independent input, so a quiet "
                "but poorly observed region can still earn detail. A region "
                "influenced by an object whose risk could not be scored takes a "
                "conservative floor rather than the coarsest level. Resolution "
                "is stabilised by an asymmetric hysteresis margin plus a "
                "minimum dwell time, so it does not oscillate between frames; "
                "that level memory is the only mapping state that survives a "
                "frame, and no occupancy accumulates. Region and cell budgets "
                "are enforced by coarsening the lowest-priority regions first, "
                "and the demotion is reported rather than hidden. No learned "
                "policy, no reinforcement learning, no ego planned path, no "
                "per-cell risk field, and no measurement of whether the "
                "allocation is correct - only of what it costs"
            ),
            phase=8,
        ),
        ComponentStatus(
            name="carla",
            readiness=ComponentReadiness.READY,
            implementation=ImplementationStatus.PARTIAL,
            detail=(
                "deterministic CARLA simulation boundary. CARLA is a DATA "
                "SOURCE upstream of the pipeline, never a second perception "
                "stack: a session applies synchronous mode and a fixed "
                "timestep, spawns the ego, attaches a LiDAR, ticks, and "
                "destroys every actor it created on close - including after "
                "a failed setup - restoring world settings. CARLA's "
                "left-handed frame is converted to the ADAPT-X frame exactly "
                "once at the boundary; frame timestamps are simulation time, "
                "never the wall clock; frames are labelled source=simulation "
                "and enter the existing ingest path unchanged. Ground truth "
                "is recorded on a separate path and never reaches detection, "
                "tracking, prediction, risk or adaptive resolution. The live "
                "connection and session state are reported under `carla`, "
                "not here. NO LIVE CARLA RUN HAS BEEN EXECUTED IN THIS "
                "REPOSITORY: the package is optional, is absent in the "
                "development environment, and the adapter has been exercised "
                "only against a stand-in. Nothing about CARLA performance or "
                "perception accuracy is claimed. One hard-coded smoke scenario; "
                "no scenario framework, no traffic, no camera, no pitch/roll "
                "conversion"
            ),
            phase=9,
        ),
        ComponentStatus(
            name="scenarios",
            readiness=ComponentReadiness.READY,
            implementation=ImplementationStatus.PARTIAL,
            detail=(
                "deterministic scenario framework. A scenario is a declarative "
                "definition - actors, ego-relative placement, timed "
                "constant-velocity motion segments, duration, timestep and an "
                "explicit seed - validated before any simulator is touched and "
                "reproducible from the definition alone. A runner drives the "
                "CARLA boundary through a narrow protocol, places every actor "
                "at its closed-form scripted pose each frame, records ground "
                "truth beside every sensor frame and feeds it to NO pipeline "
                "stage, and destroys every actor on completion or failure. "
                "The run result is raw evidence - frame identities, scripted "
                "poses, ground truth, stage counts - and carries NO accuracy "
                "or evaluation figure; comparing perception against ground "
                "truth is Phase 11 and has not been done. Four catalogue "
                "scenarios. No live CARLA run has been executed; the framework "
                "has been exercised only against a stand-in. Ego motion, event "
                "replay, traffic and weather are not implemented"
            ),
            phase=10,
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
