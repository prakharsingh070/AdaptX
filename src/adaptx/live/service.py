"""The long-lived live simulation loop (post-Phase-12 extension).

One thread owns one :class:`~adaptx.carla.session.CarlaSimulationSession`
and is the **only** caller of ``world.tick()`` while it runs. Per frame::

    scenario manager: spawn / move / remove actors due at this session time
    session.step()          -> RawPointCloudFrame           (tick + LiDAR)
    pipeline_processor()    -> Phase 2-8 outputs, unchanged (ADR-050)
    policy.decide()         -> ControlCommand from those outputs + ego odometry
    session.apply_ego_control()
    build_snapshot()        -> SceneSnapshot -> SceneService (latest only)
    events: new / lost tracks, risk level, controller state, tiles, collisions

Ground truth is **never** read in this loop. The controller sees the risk
engine's assessments, the tracker's tracks, the predictor's paths and the
ego's own odometry, and nothing else - the signatures make it so
(ADR-045). The collision sensor is a safety fallback that ends the session
and records the contact; it is not an input to anything.

Backpressure: the scene service keeps the latest snapshot only, and the
scene channel tells each client how many it skipped. Nothing queues.
"""

from __future__ import annotations

import statistics
import threading
import time
from collections import deque
from collections.abc import Callable
from datetime import UTC, datetime
from typing import TYPE_CHECKING, Any

from adaptx.carla.session import CarlaSimulationSession
from adaptx.config.settings import CarlaSettings, Settings
from adaptx.control.corridor import PathRelation
from adaptx.control.models import ControlCommand, ControllerState, EgoObservation
from adaptx.control.policy import RiskGovernedSpeedPolicy
from adaptx.core.exceptions import AdaptXError, SimulatorUnavailableError
from adaptx.core.logging import get_logger
from adaptx.live.camera import encode_png_from_bgra
from adaptx.live.models import (
    LiveEvent,
    LiveFrameInfo,
    LiveState,
    LiveStatus,
    LiveTiming,
)
from adaptx.live.scenarios import (
    LiveScenarioManager,
    ResolvedLiveScenario,
    load_live,
    resolve_live,
)
from adaptx.models.common import ObjectClass
from adaptx.models.map import ResolutionLevel
from adaptx.models.processing import PointCloudProcessingResult
from adaptx.models.risk import RiskLevel
from adaptx.models.scene import PointStage
from adaptx.models.system import SimulationSessionStatus
from adaptx.models.tracking import TrackStatus
from adaptx.models.vehicle import VehicleState
from adaptx.services.scene_service import build_snapshot, object_records

if TYPE_CHECKING:
    from adaptx.core.lifecycle import ApplicationContext
    from adaptx.scenarios.result import PipelineFrameOutputs

logger = get_logger(__name__)

SessionFactory = Callable[[CarlaSettings], CarlaSimulationSession]

_LEVEL_ORDER = {
    ResolutionLevel.LOW: 0,
    ResolutionLevel.MEDIUM: 1,
    ResolutionLevel.HIGH: 2,
    ResolutionLevel.CRITICAL: 3,
}

#: Frames over which the loop period statistics are reported.
TIMING_WINDOW = 40


class LiveSessionError(AdaptXError):
    """A live session control request could not be honoured."""

    code = "live_session_error"
    http_status = 409


class LiveSimulationService:
    """Owns the live loop: start, pause, resume, stop, reset, status, events.

    Args:
        settings: Process settings; the CARLA, control and live sections.
        session_factory: Builds the session; injectable so the loop runs
            against a stand-in in tests. Production builds a real session.
    """

    def __init__(
        self, settings: Settings, *, session_factory: SessionFactory | None = None
    ) -> None:
        self._settings = settings
        self._factory = session_factory or self._default_factory
        self._context: ApplicationContext | None = None
        self._lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._pause = threading.Event()
        self._state = LiveState.IDLE
        self._detail = "no live session"
        self._session: CarlaSimulationSession | None = None
        self._manager: LiveScenarioManager | None = None
        self._resolved: ResolvedLiveScenario | None = None
        self._policy = RiskGovernedSpeedPolicy(settings.control)
        self._started_at: datetime | None = None
        self._frames = 0
        self._published = 0
        self._session_time_s = 0.0
        self._last_ego: VehicleState | None = None
        self._last_command: ControlCommand | None = None
        self._last_timing: LiveTiming | None = None
        self._loop_ms: deque[float] = deque(maxlen=TIMING_WINDOW)
        self._events: deque[LiveEvent] = deque(maxlen=settings.live.event_history)
        self._event_sequence = 0
        self._last_error: str | None = None
        self._collision_count = 0
        # Per-frame bookkeeping for event derivation.
        self._known_tracks: dict[int, str] = {}
        self._known_relations: dict[int, PathRelation] = {}
        self._last_level: RiskLevel | None = None
        self._last_controller_state: ControllerState | None = None
        self._last_controller_event_frame = -100
        self._last_level_event_frame = -100
        self._last_resolution_event_frame = 0
        self._pending_refined = 0
        self._pending_coarsened = 0
        self._last_snapshot_ms = 0.0

    def _default_factory(self, carla_settings: CarlaSettings) -> CarlaSimulationSession:
        return CarlaSimulationSession(
            carla_settings,
            drive_ego=True,
            traffic_manager_port=self._settings.live.traffic_manager_port,
        )

    def attach(self, context: ApplicationContext) -> None:
        """Bind the pipeline context; the loop runs the context's own stages."""
        self._context = context

    # -- controls ----------------------------------------------------------
    @property
    def state(self) -> LiveState:
        """Current lifecycle state."""
        return self._state

    def start(self, scenario_id: str | None = None, seed: int | None = None) -> LiveStatus:
        """Open a session, spawn the scenario and start the loop.

        The session is opened synchronously so a caller learns at once that
        CARLA is disabled or unreachable; the loop itself runs on a thread.

        Raises:
            LiveSessionError: controls are disabled, or a session is active.
            SimulatorUnavailableError: CARLA is disabled or could not be opened.
        """
        self._require_controls()
        with self._lock:
            if self._state in (LiveState.STARTING, LiveState.RUNNING, LiveState.PAUSED):
                raise LiveSessionError(
                    f"a live session is already {self._state.value}; stop it first",
                    details={"state": self._state.value},
                )
            if self._context is None:
                raise LiveSessionError("the live service is not attached to a pipeline")
            definition = load_live(scenario_id or self._settings.live.default_scenario)
            chosen_seed = definition.default_seed if seed is None else int(seed)
            carla_settings = self._settings.carla.model_copy(
                update={"enabled": True, "seed": chosen_seed}
            )
            self._reset_bookkeeping()
            self._state = LiveState.STARTING
            self._detail = f"opening CARLA session for '{definition.scenario_id}'"
            session = self._factory(carla_settings)
            try:
                session.open()
                live = self._settings.live
                if live.collision_sensor_enabled:
                    session.attach_collision_sensor()
                if live.camera_enabled:
                    session.attach_camera(
                        width=live.camera_width,
                        height=live.camera_height,
                        fov_deg=live.camera_fov_deg,
                    )
                status = session.status()
                spawn_points = self._spawn_point_count(session)
                self._resolved = resolve_live(
                    definition, chosen_seed, spawn_point_count=spawn_points
                )
                self._manager = LiveScenarioManager(self._resolved, session)
                traffic = self._manager.spawn_traffic()
            except Exception as exc:
                session.close()
                self._state = LiveState.ERROR
                self._last_error = str(exc)
                self._detail = f"could not start: {exc}"
                if isinstance(exc, SimulatorUnavailableError):
                    raise
                raise SimulatorUnavailableError(f"could not start the live session: {exc}") from exc

            self._session = session
            self._context.carla.attach_session(session)
            self._context.tracking.reset()
            self._context.adaptive_mapping.reset()
            self._policy.reset()
            self._started_at = datetime.now(UTC)
            self._stop.clear()
            self._pause.clear()
            self._state = LiveState.RUNNING
            self._detail = (
                f"running '{definition.scenario_id}' seed {chosen_seed} on "
                f"{status.map_name} ({traffic} traffic vehicle(s))"
            )
            self._event(
                "session_started",
                f"live session started: {definition.name}, seed {chosen_seed}, "
                f"map {status.map_name}, CARLA {status.server_version}",
            )
            self._thread = threading.Thread(target=self._run, name="adaptx-live-loop", daemon=True)
            self._thread.start()
            return self.status()

    def pause(self) -> LiveStatus:
        """Stop ticking; the world freezes (synchronous mode) until resumed."""
        self._require_controls()
        with self._lock:
            if self._state is not LiveState.RUNNING:
                raise LiveSessionError(
                    f"cannot pause a session that is {self._state.value}",
                    details={"state": self._state.value},
                )
            self._pause.set()
            self._state = LiveState.PAUSED
            self._detail = "paused: the simulator is not being ticked"
            self._event("paused", "session paused")
            return self.status()

    def resume(self) -> LiveStatus:
        """Continue ticking after a pause."""
        self._require_controls()
        with self._lock:
            if self._state is not LiveState.PAUSED:
                raise LiveSessionError(
                    f"cannot resume a session that is {self._state.value}",
                    details={"state": self._state.value},
                )
            self._pause.clear()
            self._state = LiveState.RUNNING
            self._detail = "running"
            self._event("resumed", "session resumed")
            return self.status()

    def stop(self, *, reason: str = "stopped by request") -> LiveStatus:
        """End the loop and close the session: every actor destroyed, world restored."""
        self._require_controls()
        return self._stop_session(reason)

    def reset(self) -> LiveStatus:
        """Stop, then start the same scenario and seed again."""
        self._require_controls()
        resolved = self._resolved
        if resolved is None:
            raise LiveSessionError("nothing to reset: no scenario has been started")
        self._stop_session("reset")
        return self.start(resolved.definition.scenario_id, resolved.seed)

    def shutdown(self) -> None:
        """Process shutdown: stop without the controls check."""
        if self._thread is not None:
            self._stop_session("backend shutdown")

    # -- reporting -----------------------------------------------------------
    def status(self) -> LiveStatus:
        """The session as it is now."""
        with self._lock:
            session = self._session
            simulation = session.status() if session is not None else SimulationSessionStatus()
            camera = session is not None and session.camera_image() is not None
            resolved = self._resolved
            return LiveStatus(
                state=self._state,
                controls_enabled=self._settings.live.controls_enabled,
                scenario_id=None if resolved is None else resolved.definition.scenario_id,
                scenario_name=None if resolved is None else resolved.definition.name,
                seed=None if resolved is None else resolved.seed,
                started_at=self._started_at,
                session_time_s=self._session_time_s if self._started_at else None,
                frames_processed=self._frames,
                snapshots_published=self._published,
                simulation=simulation,
                carla_connected=session is not None
                and simulation.state.value in ("READY", "RUNNING"),
                ego=self._last_ego,
                ego_speed_mps=(
                    None
                    if self._last_ego is None
                    else (self._last_ego.velocity.x**2 + self._last_ego.velocity.y**2) ** 0.5
                ),
                control=self._last_command,
                controller_state=None if self._last_command is None else self._policy.state,
                control_configuration=self._policy.configuration(),
                timing=self._last_timing,
                collision_count=self._collision_count,
                active_actors=[] if self._manager is None else self._manager.active,
                traffic_count=0 if self._manager is None else self._manager.traffic_count,
                camera_available=camera,
                last_error=self._last_error,
                event_sequence=self._event_sequence,
                detail=self._detail,
            )

    def events(self, since: int = 0, limit: int = 50) -> list[LiveEvent]:
        """Events with a sequence greater than ``since``, oldest first."""
        with self._lock:
            newer = [event for event in self._events if event.sequence > since]
        return newer[-limit:]

    def camera_png(self) -> bytes | None:
        """The latest camera frame as PNG, or ``None`` when there is none."""
        session = self._session
        if session is None:
            return None
        image = session.camera_image()
        if image is None:
            return None
        return encode_png_from_bgra(image.bgra, image.width, image.height)

    # -- the loop ------------------------------------------------------------
    def _run(self) -> None:
        context = self._context
        session = self._session
        manager = self._manager
        assert context is not None and session is not None and manager is not None
        from adaptx.scenarios.result import PipelineFrameOutputs
        from adaptx.scenarios.runner import pipeline_processor

        latest_processed: list[PointCloudProcessingResult] = []
        processor = pipeline_processor(
            context, on_processed=lambda result: latest_processed.append(result)
        )
        dt = self._settings.carla.fixed_delta_seconds
        live = self._settings.live
        control = self._settings.control
        outcome = LiveState.STOPPED
        reason = "stopped by request"
        try:
            while not self._stop.is_set():
                if self._pause.is_set():
                    time.sleep(0.02)
                    continue
                loop_start = time.perf_counter()
                notes = manager.advance(self._session_time_s)
                for note in notes:
                    self._event("scenario", note)

                frame = session.step()
                after_step = time.perf_counter()

                produced = processor(frame)
                after_pipeline = time.perf_counter()
                if not isinstance(produced, PipelineFrameOutputs):  # pragma: no cover
                    raise LiveSessionError("the pipeline processor did not return outputs")

                # The controller reads the pipeline's outputs and the ego's
                # own odometry. Nothing else is passed; nothing else exists here.
                observation: EgoObservation = session.ego_observation(control.lane_lookahead_m)
                command = self._policy.decide(
                    risk=produced.risk,
                    tracking=produced.tracking,
                    prediction=produced.prediction,
                    ego=observation,
                    dt_s=dt,
                )
                session.apply_ego_control(
                    throttle=command.throttle, brake=command.brake, steer_left=command.steer
                )
                after_control = time.perf_counter()

                collisions = session.collisions()
                ego_state = session.ego_state()
                # The frame counts as an ingested LiDAR frame, as it does on the
                # endpoints: the status and the measured FPS/latency come from
                # here, not from anything the dashboard computes.
                if latest_processed:
                    context.lidar.ingest(
                        latest_processed[-1].frame,
                        pre_validated=True,
                        upstream_duration_s=after_pipeline - after_step,
                    )
                    latest_processed.clear()
                self._publish(
                    frame=frame,
                    produced=produced,
                    ego_state=ego_state,
                    command=command,
                    session=session,
                    manager=manager,
                    collisions=len(collisions),
                    timings=(loop_start, after_step, after_pipeline, after_control),
                    dt=dt,
                    lag_ratio=live.lag_ratio,
                )
                self._derive_events(produced, command, frame.frame_id)

                if len(collisions) > self._collision_count:
                    self._collision_count = len(collisions)
                    latest = collisions[-1]
                    self._event(
                        "collision",
                        f"COLLISION with {latest.other_type_id} at simulation frame "
                        f"{latest.simulation_frame} (impulse {latest.impulse:.0f}); "
                        "session stopped by the safety fallback",
                    )
                    outcome, reason = LiveState.COLLIDED, "collision detected"
                    break
                if live.max_frames and self._frames >= live.max_frames:
                    reason = f"reached max_frames={live.max_frames}"
                    break
        except Exception as exc:
            logger.warning("live loop failed", extra={"context": {"error": str(exc)}})
            outcome, reason = LiveState.ERROR, str(exc)
            self._last_error = str(exc)
        finally:
            self._finish(outcome, reason)

    def _publish(
        self,
        *,
        frame: Any,
        produced: PipelineFrameOutputs,
        ego_state: VehicleState,
        command: ControlCommand,
        session: CarlaSimulationSession,
        manager: LiveScenarioManager,
        collisions: int,
        timings: tuple[float, float, float, float],
        dt: float,
        lag_ratio: float,
    ) -> None:
        assert self._context is not None and self._resolved is not None
        loop_start, after_step, after_pipeline, after_control = timings
        snapshot_start = time.perf_counter()
        status = session.status()
        self._frames += 1
        self._session_time_s = self._frames * dt
        provisional_loop_ms = (snapshot_start - loop_start) * 1000.0
        loop_ms_list = list(self._loop_ms)
        timing = LiveTiming(
            step_ms=(after_step - loop_start) * 1000.0,
            pipeline_ms=(after_pipeline - after_step) * 1000.0,
            control_ms=(after_control - after_pipeline) * 1000.0,
            # The cost of building and publishing THIS snapshot cannot be inside
            # it; the previous frame's measured cost is carried instead.
            snapshot_ms=self._last_snapshot_ms,
            loop_ms=provisional_loop_ms,
            fixed_delta_s=dt,
            realtime_factor=min(99.0, (dt * 1000.0) / max(provisional_loop_ms, 1e-3)),
            lagging=provisional_loop_ms > dt * 1000.0 * lag_ratio,
            frames_processed=self._frames,
            loop_ms_median=statistics.median(loop_ms_list) if len(loop_ms_list) >= 5 else None,
            loop_ms_max=max(loop_ms_list) if loop_ms_list else None,
        )
        info = LiveFrameInfo(
            scenario_id=self._resolved.definition.scenario_id,
            seed=self._resolved.seed,
            simulation_frame=status.simulation_frame or frame.frame_id,
            simulation_time_s=status.simulation_time_s or 0.0,
            session_time_s=self._session_time_s,
            controller_state=self._policy.state,
            collision_count=collisions,
            active_actor_count=len(manager.active),
            traffic_count=manager.traffic_count,
            timing=timing,
        )
        snapshot = build_snapshot(
            frame_id=frame.frame_id,
            sensor_id=frame.sensor_id,
            source=frame.source,
            frame_timestamp=frame.timestamp,
            origin=f"live:{self._resolved.definition.scenario_id}",
            points=frame.points,
            point_stage=PointStage.RAW,
            detection=produced.detection,
            tracking=produced.tracking,
            prediction=produced.prediction,
            risk=produced.risk,
            plan=produced.plan,
            adaptive_map=produced.adaptive_map,
            fixed_map=produced.fixed_map,
            comparison=produced.comparison,
            processing_ms=produced.processing_ms,
            scenario_id=self._resolved.definition.scenario_id,
            frame_index=self._frames - 1,
            scenario_time_s=self._session_time_s,
            max_points=self._settings.dashboard.scene_max_points,
            ego=ego_state,
            control=command,
            live=info,
            path_half_width_m=self._settings.control.path_half_width_m,
        )
        self._context.scene.publish(snapshot)
        loop_ms = (time.perf_counter() - loop_start) * 1000.0
        self._last_snapshot_ms = (time.perf_counter() - snapshot_start) * 1000.0
        with self._lock:
            self._published += 1
            self._loop_ms.append(loop_ms)
            self._last_ego = ego_state
            self._last_command = command
            self._last_timing = timing.model_copy(
                update={
                    "snapshot_ms": (time.perf_counter() - snapshot_start) * 1000.0,
                    "loop_ms": loop_ms,
                    "realtime_factor": min(99.0, (dt * 1000.0) / max(loop_ms, 1e-3)),
                    "lagging": loop_ms > dt * 1000.0 * lag_ratio,
                }
            )

    def _derive_events(
        self, produced: PipelineFrameOutputs, command: ControlCommand, frame_id: int
    ) -> None:
        """Turn frame-to-frame differences in the outputs into events. Bookkeeping only."""
        distances = {a.track_id: a.distance_m for a in produced.risk.assessments}
        # Only CONFIRMED tracks make events. Measured live, the tracker opens
        # dozens of tentative tracks a second on street furniture and drops
        # most within two frames (Experiments 011/012: identity churn); the
        # tracker's own lifecycle status is the honest filter.
        current = {
            t.track_id: t.object_class.value
            for t in produced.tracking.tracks
            if t.status in (TrackStatus.CONFIRMED, TrackStatus.COASTING)
        }
        records = object_records(
            produced.tracking,
            produced.prediction,
            produced.risk,
            self._settings.control.path_half_width_m,
        )
        relation_of = {r.track_id: r.path_relation for r in records}
        for track_id, object_class in current.items():
            if track_id not in self._known_tracks:
                distance = distances.get(track_id)
                where = f" at {distance:.1f} m" if distance is not None else ""
                relation = relation_of.get(track_id)
                tail = f", {relation.value.replace('_', ' ')}" if relation is not None else ""
                self._event(
                    "object_detected",
                    f"{object_class} track #{track_id} confirmed{where}{tail}",
                    track_id=track_id,
                    object_class=object_class,
                )
        for track_id, object_class in self._known_tracks.items():
            if track_id not in current:
                self._event(
                    "track_lost",
                    f"{object_class} track #{track_id} lost",
                    track_id=track_id,
                    object_class=object_class,
                )
        self._known_tracks = current

        # Path-relation transitions of confirmed tracks: entering the corridor,
        # a predicted crossing, leaving. Derived from the same corridor rule the
        # controller and the snapshot use, so the event matches the label.
        relations: dict[int, PathRelation] = {}
        for record in records:
            if record.track_id not in current:
                continue
            relations[record.track_id] = record.path_relation
            before = self._known_relations.get(record.track_id)
            if before is None or before is record.path_relation:
                continue
            where = "" if record.distance_m is None else f" at {record.distance_m:.1f} m"
            label = f"{record.object_class.value} #{record.track_id}"
            if record.path_relation is PathRelation.IN_PATH:
                self._event(
                    "entered_path",
                    f"{label} entered the ego path{where}",
                    track_id=record.track_id,
                    object_class=record.object_class.value,
                )
            elif record.path_relation is PathRelation.CROSSING:
                self._event(
                    "crossing_path",
                    f"{label} predicted to cross the ego path{where}",
                    track_id=record.track_id,
                    object_class=record.object_class.value,
                )
            elif before in (PathRelation.IN_PATH, PathRelation.CROSSING):
                self._event(
                    "left_path",
                    f"{label} left the ego path",
                    track_id=record.track_id,
                    object_class=record.object_class.value,
                )
        self._known_relations = relations

        # The scene level is the maximum over every assessed object, roadside
        # clutter included, and flaps HIGH<->CRITICAL several times a second
        # as tracks come and go (measured live). One event per 20 frames.
        level = produced.risk.highest_risk_level
        if (
            self._last_level is not None
            and level is not self._last_level
            and self._frames - self._last_level_event_frame >= 20
        ):
            self._event(
                "risk_level_changed",
                f"scene risk level {self._last_level.value.upper()} → {level.value.upper()}",
            )
            self._last_level_event_frame = self._frames
        self._last_level = level

        # Hold-class transitions always make an event; CRUISING/SLOWING flips
        # are coalesced, because a level that flickers with track identity
        # (measured live) would otherwise fill the log several times a second.
        if command.state is not self._last_controller_state:
            hold_class = command.state in (
                ControllerState.HOLDING,
                ControllerState.EMERGENCY_BRAKING,
                ControllerState.STOPPED,
                ControllerState.RESUMING,
            )
            if hold_class or self._frames - self._last_controller_event_frame >= 20:
                self._event("controller", f"controller {command.state.value}: {command.reason}")
                self._last_controller_event_frame = self._frames
        self._last_controller_state = command.state

        for decision in produced.plan.decisions:
            if not decision.changed or decision.previous_level is None:
                continue
            if _LEVEL_ORDER[decision.level] > _LEVEL_ORDER[decision.previous_level]:
                self._pending_refined += 1
            else:
                self._pending_coarsened += 1
        # Tile changes are summed over a window and reported as one event:
        # measured live, the plan changes a few tiles on most frames.
        if (self._pending_refined or self._pending_coarsened) and (
            self._frames - self._last_resolution_event_frame >= 20
        ):
            self._event(
                "resolution_changed",
                f"adaptive map: {self._pending_refined} tile refinement(s), "
                f"{self._pending_coarsened} coarsening(s) over the last "
                f"{self._frames - max(0, self._last_resolution_event_frame)} frame(s)",
            )
            self._pending_refined = self._pending_coarsened = 0
            self._last_resolution_event_frame = self._frames

    def _finish(self, outcome: LiveState, reason: str) -> None:
        with self._lock:
            session = self._session
            if session is not None:
                self._state = LiveState.STOPPING
                session.close()
            if self._manager is not None:
                self._manager.clear()
            if self._context is not None:
                self._context.carla.attach_session(session)
            self._state = outcome
            self._detail = reason
            self._event("session_ended", f"session ended: {reason} ({outcome.value})")

    def _stop_session(self, reason: str) -> LiveStatus:
        with self._lock:
            thread = self._thread
            if thread is None or not thread.is_alive():
                if self._state in (LiveState.STARTING, LiveState.RUNNING, LiveState.PAUSED):
                    self._finish(LiveState.STOPPED, reason)
                return self.status()
            self._detail = reason
            self._stop.set()
            self._pause.clear()
        thread.join(timeout=30.0)
        with self._lock:
            self._thread = None
            if self._state is LiveState.STOPPED:
                self._detail = reason
            return self.status()

    # -- helpers -------------------------------------------------------------
    def _require_controls(self) -> None:
        if not self._settings.live.controls_enabled:
            raise LiveSessionError(
                "live session controls are disabled (ADAPTX_LIVE__CONTROLS_ENABLED=false)",
                details={"controls_enabled": False},
            )

    @staticmethod
    def _spawn_point_count(session: CarlaSimulationSession) -> int:
        world = getattr(session, "_world", None)
        try:
            return len(world.get_map().get_spawn_points()) if world is not None else 0
        except Exception:  # pragma: no cover - defensive
            return 0

    def _reset_bookkeeping(self) -> None:
        self._frames = 0
        self._published = 0
        self._session_time_s = 0.0
        self._last_ego = None
        self._last_command = None
        self._last_timing = None
        self._loop_ms.clear()
        self._last_error = None
        self._collision_count = 0
        self._known_tracks = {}
        self._known_relations = {}
        self._last_level = None
        self._last_controller_state = None
        self._last_controller_event_frame = -100
        self._last_level_event_frame = -100
        self._last_resolution_event_frame = 0
        self._pending_refined = 0
        self._pending_coarsened = 0
        self._last_snapshot_ms = 0.0
        self._started_at = None

    def _event(
        self,
        kind: str,
        detail: str,
        *,
        track_id: int | None = None,
        object_class: str | None = None,
    ) -> None:
        session = self._session
        status = session.status() if session is not None else None
        with self._lock:
            self._event_sequence += 1
            self._events.append(
                LiveEvent(
                    sequence=self._event_sequence,
                    kind=kind,
                    detail=detail,
                    simulation_time_s=None if status is None else status.simulation_time_s,
                    simulation_frame=None if status is None else status.simulation_frame,
                    wall_time=datetime.now(UTC),
                    track_id=track_id,
                    object_class=None if object_class is None else ObjectClass(object_class),
                )
            )


__all__ = ["LiveSessionError", "LiveSimulationService"]
