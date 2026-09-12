"""Deterministic CARLA simulation session (Phase 9).

The lifecycle boundary. Everything that *talks to* CARLA lives here; everything
that *converts* CARLA values lives in :mod:`adaptx.carla.conversion`, which
imports no simulator at all. That split is what makes the risky part - a sign
error in a coordinate flip - testable on a machine with no CARLA installed.

::

    connect -> configure world (synchronous, fixed timestep)
            -> spawn ego -> attach LiDAR
            -> tick -> RawPointCloudFrame (+ GroundTruthFrame, separately)
            -> close: stop sensor, destroy actors, restore settings

What this module must never do
------------------------------
No detection, tracking, prediction, mapping, risk or resolution logic. The
session produces a :class:`~adaptx.models.point_cloud.RawPointCloudFrame` and
hands it to the existing ingest path; from that point on nothing downstream can
tell the frame came from a simulator rather than a sensor, which is the whole
point of the boundary (ADR-042).

Determinism
-----------
Synchronous mode with a fixed timestep, driven by explicit ticks (ADR-044).
Never a sleep, never a wall-clock read. Tracking measures velocity from frame
intervals and adaptive resolution counts frames, so a free-running simulator
would make both non-reproducible.

Cleanup
-------
The session owns every actor it spawns and destroys all of them on
:meth:`close`, including when setup fails part-way through. World settings are
captured before they are changed and restored afterwards, so a crashed run does
not leave a server stuck in synchronous mode waiting for a client that has gone
away - which is the failure that makes a CARLA server *appear* hung.
"""

from __future__ import annotations

import importlib
import math
import queue
import threading
from collections import deque
from types import ModuleType
from typing import Any, NamedTuple

from adaptx.carla.conversion import (
    adaptx_offset_to_carla,
    build_raw_frame,
    carla_location_to_vector3,
    carla_vector_to_ego,
    carla_world_to_ego,
    carla_yaw_to_heading_rad,
    decode_lidar_buffer,
    ego_offset_to_carla_world,
    simulation_timestamp,
)
from adaptx.carla.ground_truth import (
    GroundTruthActor,
    GroundTruthFrame,
    build_ground_truth_frame,
    classify_blueprint,
)
from adaptx.config.settings import CarlaSettings
from adaptx.control.models import EgoObservation
from adaptx.core.exceptions import SimulatorUnavailableError
from adaptx.core.logging import get_logger
from adaptx.models.common import AdaptXModel, DataSource, Dimensions
from adaptx.models.point_cloud import RawPointCloudFrame
from adaptx.models.system import SimulationSessionStatus, SimulationState
from adaptx.models.vehicle import VehicleState

logger = get_logger(__name__)

#: Sensor blueprint used for the LiDAR. The ray-cast variant is the one that
#: reports per-point intensity.
LIDAR_BLUEPRINT = "sensor.lidar.ray_cast"

#: Height above the resting pose at which an actor is spawned before being
#: set down: the server refuses a spawn whose collision volume meets the road.
SPAWN_CLEARANCE_M = 0.5

#: Safety-fallback sensor (live extension): reports a contact the controller
#: failed to prevent. Never a perception input.
COLLISION_BLUEPRINT = "sensor.other.collision"

#: Visualisation-only front camera (live extension). No perception stage
#: reads it; LiDAR remains the only perception input.
CAMERA_BLUEPRINT = "sensor.camera.rgb"

#: Camera mount relative to the ego origin, in the ADAPT-X frame. A fixed
#: presentation choice (windscreen height, slightly forward), not a
#: calibrated value.
CAMERA_MOUNT_M = (1.2, 0.0, 1.5)

#: How many collision events are kept.
COLLISION_HISTORY = 50

#: ``role_name`` values ADAPT-X stamps on the actors it spawns. Reclaiming
#: stale actors on open touches these and their attached sensors only -
#: never another client's actors.
EGO_ROLE = "ego"
TRAFFIC_ROLE = "traffic"
SCENARIO_ROLE = "adaptx_scenario"
OWNED_ROLES = frozenset({EGO_ROLE, TRAFFIC_ROLE, SCENARIO_ROLE})


class WorldAnchor(NamedTuple):
    """An opaque world pose (CARLA frame) captured from the ego at one instant.

    A live scenario spawns an obstacle "30 m ahead of where the ego is
    *now*" and then keeps it there while the ego drives on. Placing relative
    to the *current* ego pose every frame would drag the obstacle along, so
    the pose is captured once and every later placement is relative to it.
    Callers never read the fields; they hand the anchor back.
    """

    x: float
    y: float
    z: float
    yaw_deg: float


class CollisionEvent(AdaptXModel):
    """One contact reported by the collision sensor - a safety record, not perception."""

    simulation_frame: int
    other_type_id: str
    impulse: float


class CameraImage(NamedTuple):
    """The latest RGB camera frame, raw BGRA bytes as CARLA delivers them."""

    frame: int
    width: int
    height: int
    bgra: bytes


def _stand_offset(actor: Any) -> float:
    """How far an actor's origin sits above the bottom of its bounding box.

    A CARLA vehicle's origin is at its wheels (offset about zero); a walker's
    is at the middle of its capsule (about 0.93 m). Zero when the server
    reports no bounding box, in which case ``up_m`` is measured to the origin.
    """
    box = getattr(actor, "bounding_box", None)
    extent = getattr(box, "extent", None)
    if extent is None:
        return 0.0
    centre = getattr(getattr(box, "location", None), "z", 0.0)
    return float(extent.z) - float(centre)


def load_carla_module() -> ModuleType:
    """Import the optional ``carla`` package, or fail with an actionable message."""
    try:
        return importlib.import_module("carla")
    except ImportError as exc:
        raise SimulatorUnavailableError(
            "the optional CARLA package is not installed "
            "(install with: pip install -e .[carla], and start a CARLA server)",
            details={"package_available": False},
        ) from exc


class CarlaSimulationSession:
    """Owns one deterministic CARLA simulation and the LiDAR stream from it.

    Args:
        settings: Connection, determinism and sensor configuration.
        carla_module: The CARLA package. Injectable so the lifecycle can be
            exercised against a stand-in; production passes nothing and the
            real package is imported lazily.
    """

    def __init__(
        self,
        settings: CarlaSettings,
        carla_module: ModuleType | Any | None = None,
        *,
        drive_ego: bool = False,
        traffic_manager_port: int = 8050,
    ) -> None:
        self._settings = settings
        self._carla = carla_module
        # Phase 10 scenarios keep the ego physics-less and grounded (ADR-054)
        # because the ego never drives. The live extension drives it, so its
        # physics stays on and it settles onto the road like any vehicle.
        self._drive_ego = drive_ego
        self._tm_port = traffic_manager_port
        self._traffic_manager: Any | None = None
        self._collision_sensor: Any | None = None
        self._collisions: deque[CollisionEvent] = deque(maxlen=COLLISION_HISTORY)
        self._camera: Any | None = None
        self._camera_latest: CameraImage | None = None
        self._sensor_lock = threading.Lock()
        self._last_control: tuple[float, float, float] | None = None
        self._client: Any | None = None
        self._world: Any | None = None
        self._original_settings: Any | None = None

        self._ego: Any | None = None
        self._ego_spawn_index: int | None = None
        self._ego_spawn_transform: Any | None = None
        # Per placed actor: how far its origin sits above the bottom of its
        # bounding box, so "up_m" can mean the same thing for a car (origin at
        # the wheels) and a walker (origin at the middle of the capsule).
        self._stand_offsets: dict[int, float] = {}
        self._sensor: Any | None = None
        self._actors: list[Any] = []
        self._queue: queue.Queue[Any] = queue.Queue()

        self._state = SimulationState.IDLE
        self._detail = ""
        self._frames_stepped = 0
        self._sim_frame: int | None = None
        self._sim_time: float | None = None
        self._last_point_count: int | None = None
        self._map_name: str | None = None
        self._server_version: str | None = None
        self._reclaimed_actors = 0

    # -- state -------------------------------------------------------------
    @property
    def state(self) -> SimulationState:
        """Current lifecycle state."""
        return self._state

    @property
    def is_running(self) -> bool:
        """True once the session has stepped at least one frame."""
        return self._state is SimulationState.RUNNING

    @property
    def ego_actor_id(self) -> int | None:
        """Actor id of the ego vehicle, or ``None`` before it is spawned."""
        return None if self._ego is None else int(self._ego.id)

    @property
    def ego_spawn_index(self) -> int | None:
        """Index into the map's spawn points the ego was placed at, once spawned."""
        return self._ego_spawn_index

    def status(self) -> SimulationSessionStatus:
        """Compact, bounded description of the session."""
        return SimulationSessionStatus(
            state=self._state,
            map_name=self._map_name,
            synchronous_mode=self._settings.synchronous_mode,
            fixed_delta_seconds=self._settings.fixed_delta_seconds,
            frames_stepped=self._frames_stepped,
            simulation_frame=self._sim_frame,
            simulation_time_s=self._sim_time,
            ego_actor_id=self.ego_actor_id,
            ego_spawn_index=self._ego_spawn_index,
            server_version=self._server_version,
            sensor_actor_id=None if self._sensor is None else int(self._sensor.id),
            actor_count=len(self._actors),
            last_point_count=self._last_point_count,
            detail=self._detail,
            reclaimed_actors=self._reclaimed_actors,
        )

    @property
    def reclaimed_actors(self) -> int:
        """Stale ADAPT-X actors destroyed when this session opened."""
        return self._reclaimed_actors

    # -- lifecycle ---------------------------------------------------------
    def open(self) -> None:
        """Connect, configure determinism, spawn the ego and attach the LiDAR.

        Any failure part-way through tears down whatever was already created
        before re-raising, so a failed setup never leaves actors behind.

        Raises:
            SimulatorUnavailableError: CARLA is disabled or unavailable, the
                server is unreachable, or an actor could not be spawned.
        """
        if self._state in (SimulationState.READY, SimulationState.RUNNING):
            raise SimulatorUnavailableError(
                "the simulation session is already open; close it before opening another",
                details={"state": self._state.value},
            )
        if not self._settings.enabled:
            raise SimulatorUnavailableError(
                "CARLA is disabled (set ADAPTX_CARLA__ENABLED=true to enable it)",
                details={"enabled": False},
            )

        self._state = SimulationState.CONFIGURING
        try:
            self._connect()
            self._reclaim_stale_actors()
            self._configure_world()
            self._spawn_ego()
            self._attach_lidar()
        except Exception as exc:
            self._detail = str(exc)
            self.close()
            self._state = SimulationState.ERROR
            if isinstance(exc, SimulatorUnavailableError):
                raise
            raise SimulatorUnavailableError(
                f"could not open the CARLA simulation session: {exc}",
                details={"host": self._settings.host, "port": self._settings.port},
            ) from exc

        self._state = SimulationState.READY
        self._detail = "session ready"
        logger.info(
            "CARLA session ready",
            extra={
                "context": {
                    "map": self._map_name,
                    "fixed_delta_seconds": self._settings.fixed_delta_seconds,
                    "ego_actor_id": self.ego_actor_id,
                }
            },
        )

    def close(self) -> None:
        """Stop the sensor, destroy every spawned actor and restore the world.

        Idempotent and defensive: it runs during a failed setup as well as a
        normal shutdown, so every step tolerates the thing it is cleaning up
        never having been created. A failure to destroy one actor does not
        prevent the rest from being destroyed.
        """
        if self._ego is not None and self._drive_ego:
            self._safely(self.release_ego, "release ego control")
        for name in ("_collision_sensor", "_camera"):
            extra = getattr(self, name)
            if extra is not None:
                self._safely(extra.stop, f"stop {name[1:]}")
                self._safely(extra.destroy, f"destroy {name[1:]}")
                setattr(self, name, None)
        if self._sensor is not None:
            self._safely(self._sensor.stop, "stop LiDAR sensor")
            self._safely(self._sensor.destroy, "destroy LiDAR sensor")
            self._sensor = None
        if self._traffic_manager is not None:
            # Measured on CARLA 0.9.16: destroying a Traffic-Manager vehicle
            # while the world is still synchronous and nobody ticks aborts the
            # client process (STATUS_STACK_BUFFER_OVERRUN) and leaks every
            # actor. The order the CARLA examples use is the one that works:
            # world back to asynchronous FIRST, then the TM, then autopilot
            # off, and only then destroy.
            if self._world is not None and self._original_settings is not None:
                self._safely(
                    lambda: self._world.apply_settings(self._original_settings),
                    "restore world settings before releasing traffic",
                )
            self._safely(
                lambda: self._traffic_manager.set_synchronous_mode(False),
                "release traffic manager",
            )
            for actor in self._actors:
                if getattr(actor, "set_autopilot", None) is not None and actor is not self._ego:
                    self._safely(lambda a=actor: a.set_autopilot(False), "autopilot off")
            # One asynchronous server frame so the mode change and the
            # autopilot release are applied before anything is destroyed.
            if self._world is not None:
                self._safely(self._world.wait_for_tick, "settle after releasing traffic")
            self._traffic_manager = None

        self._destroy_actors()
        self._actors.clear()
        self._ego = None
        self._ego_spawn_index = None
        self._ego_spawn_transform = None
        self._stand_offsets.clear()

        # Restoring settings matters more than it looks: a server left in
        # synchronous mode blocks on a client that no longer exists, and looks
        # to the next user like a hung simulator.
        if self._world is not None and self._original_settings is not None:
            self._safely(
                lambda: self._world.apply_settings(self._original_settings),
                "restore world settings",
            )
        self._original_settings = None

        _drain(self._queue)
        self._camera_latest = None
        self._last_control = None

        self._world = None
        self._client = None
        if self._state is not SimulationState.ERROR:
            self._state = SimulationState.STOPPED
            self._detail = "session closed"
        logger.info("CARLA session closed")

    def __enter__(self) -> CarlaSimulationSession:
        self.open()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    # -- stepping ----------------------------------------------------------
    def step(self) -> RawPointCloudFrame:
        """Advance the simulation one fixed timestep and return the LiDAR frame.

        The tick is explicit and the sensor frame is matched to it, so the
        returned frame belongs to the step just taken rather than to whatever
        happened to be queued.

        Raises:
            SimulatorUnavailableError: the session is not open, the tick
                failed, or no sensor frame arrived within
                ``sensor_timeout_s``.
        """
        if self._state not in (SimulationState.READY, SimulationState.RUNNING):
            raise SimulatorUnavailableError(
                "the simulation session is not open; call open() first",
                details={"state": self._state.value},
            )

        world = self._require_world()
        try:
            world.tick()
        except Exception as exc:
            self._state = SimulationState.ERROR
            self._detail = f"simulation tick failed: {exc}"
            raise SimulatorUnavailableError(
                f"simulation tick failed: {exc}", details={"frame": self._sim_frame}
            ) from exc

        snapshot = world.get_snapshot()
        self._sim_frame = int(snapshot.frame)
        self._sim_time = float(snapshot.timestamp.elapsed_seconds)

        measurement = self._await_measurement(self._sim_frame)
        points = decode_lidar_buffer(measurement.raw_data)
        frame = build_raw_frame(
            points_carla=points,
            frame_id=int(measurement.frame),
            sensor_id=LIDAR_BLUEPRINT,
            elapsed_seconds=float(measurement.timestamp),
            include_intensity=self._settings.include_intensity,
        )

        self._frames_stepped += 1
        self._last_point_count = frame.point_count
        self._state = SimulationState.RUNNING
        self._detail = "stepping"
        return frame

    def ground_truth(self) -> GroundTruthFrame:
        """What the simulator knew at the most recent tick.

        **Never fed to perception.** This exists for evaluation and debugging
        (ADR-045); calling it does not affect anything the pipeline computes,
        and nothing in the pipeline calls it.

        Raises:
            SimulatorUnavailableError: the session has not stepped a frame.
        """
        world = self._require_world()
        if self._sim_frame is None or self._sim_time is None:
            raise SimulatorUnavailableError(
                "no frame has been stepped, so there is no ground truth to report",
                details={"state": self._state.value},
            )

        ego_transform = None if self._ego is None else self._ego_transform()
        ego_xyz = (0.0, 0.0, 0.0)
        ego_yaw = 0.0
        if ego_transform is not None:
            ego_xyz = (
                float(ego_transform.location.x),
                float(ego_transform.location.y),
                float(ego_transform.location.z),
            )
            ego_yaw = float(ego_transform.rotation.yaw)

        actors: list[GroundTruthActor] = []
        for actor in world.get_actors():
            record = self._describe_actor(actor, ego_xyz, ego_yaw)
            if record is not None:
                actors.append(record)

        return build_ground_truth_frame(
            frame_id=self._sim_frame,
            timestamp=simulation_timestamp(self._sim_time),
            map_name=self._map_name or "unknown",
            actors=actors,
            ego_actor_id=self.ego_actor_id,
        )

    def ego_state(self) -> VehicleState:
        """Ego kinematics from the simulator, in the ADAPT-X world frame.

        Ground-truth ego state, so it is labelled ``SIMULATION`` and is not a
        measurement. Nothing in the pipeline consumes it.
        """
        if self._ego is None:
            raise SimulatorUnavailableError("no ego vehicle has been spawned")
        transform = self._ego_transform()
        velocity = self._ego.get_velocity()
        return VehicleState(
            timestamp=simulation_timestamp(self._sim_time or 0.0),
            position=carla_location_to_vector3(
                float(transform.location.x),
                float(transform.location.y),
                float(transform.location.z),
            ),
            velocity=carla_location_to_vector3(
                float(velocity.x), float(velocity.y), float(velocity.z)
            ),
            heading_rad=carla_yaw_to_heading_rad(float(transform.rotation.yaw)),
            source=DataSource.SIMULATION,
        )

    def spawn_ahead_of_ego(
        self, blueprint_id: str, *, forward_m: float, left_m: float = 0.0, up_m: float = 0.0
    ) -> Any:
        """Spawn an actor at an offset from the ego, in the ADAPT-X frame.

        A scenario describes positions the way a person does - "30 m ahead and
        3 m to the left" - which is a statement in the ego frame. The
        conversion to CARLA world coordinates happens here so the scenario
        never writes a CARLA coordinate.

        The actor is **placed, not simulated** (ADR-047, ADR-054): its physics
        is switched off the moment it exists, and ``up_m`` is the height of
        the bottom of its bounding box above the ego's ground plane, so a
        walker and a car asked for ``up_m=0`` both stand on the road. It is
        spawned with clearance first, because the server refuses a spawn that
        intersects the road, then moved to its resting pose.

        Raises:
            SimulatorUnavailableError: no ego exists, or the location is
                occupied or off-road.
        """
        carla = self._require_carla()
        world = self._require_world()
        blueprint = self._find_blueprint(blueprint_id)
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", SCENARIO_ROLE)
        cx, cy, cz = self._world_position_for(forward_m, left_m, up_m + SPAWN_CLEARANCE_M)

        transform = carla.Transform(
            carla.Location(x=cx, y=cy, z=cz), carla.Rotation(yaw=self._ego_yaw())
        )
        actor = world.try_spawn_actor(blueprint, transform)
        if actor is None:
            raise SimulatorUnavailableError(
                f"could not spawn '{blueprint_id}' {forward_m} m ahead of the ego; "
                "the location is probably occupied or off-road",
                details={"blueprint": blueprint_id, "forward_m": forward_m},
            )
        self._actors.append(actor)
        actor.set_simulate_physics(False)
        self._stand_offsets[int(actor.id)] = _stand_offset(actor)
        self.place_ahead_of_ego(actor, forward_m=forward_m, left_m=left_m, up_m=up_m)
        return actor

    def place_ahead_of_ego(
        self, actor: Any, *, forward_m: float, left_m: float = 0.0, up_m: float = 0.0
    ) -> None:
        """Move an actor to an ego-relative pose.

        Scripted rather than physical: setting the transform each tick makes
        the motion exactly reproducible, which physics and traffic autopilot
        are not (ADR-044). Phase 9 needs repeatable observations, not realistic
        dynamics. ``up_m`` is the height of the bottom of the actor's bounding
        box above the ego's ground plane (ADR-054).
        """
        carla = self._require_carla()
        stand = self._stand_offsets.get(int(actor.id), 0.0)
        cx, cy, cz = self._world_position_for(forward_m, left_m, up_m + stand)
        actor.set_transform(
            carla.Transform(carla.Location(x=cx, y=cy, z=cz), carla.Rotation(yaw=self._ego_yaw()))
        )

    def _ego_yaw(self) -> float:
        """Ego yaw in CARLA degrees, or 0.0 before an ego exists."""
        if self._ego is None:
            return 0.0
        return float(self._ego_transform().rotation.yaw)

    def _ego_transform(self) -> Any:
        """The ego's pose, in CARLA's frame, as the reference for placement.

        CARLA does not report a freshly spawned actor's pose until the server
        has ticked once: in synchronous mode ``get_transform()`` returns the
        world origin with zero yaw, not the spawn point (observed on 0.9.16,
        first live run). Offsetting from that put every scripted actor
        kilometres from the ego and off the road, so the spawn refused. Until
        the first tick the transform the ego was spawned with is the only
        truthful pose; after it the simulator's own answer is, and the two
        agree because the ego is never driven (ADR-047).
        """
        if self._ego is None:
            raise SimulatorUnavailableError("no ego vehicle has been spawned")
        if self._frames_stepped == 0 and self._ego_spawn_transform is not None:
            return self._ego_spawn_transform
        return self._ego.get_transform()

    def _world_position_for(
        self, forward_m: float, left_m: float, up_m: float
    ) -> tuple[float, float, float]:
        """CARLA world coordinates for an ADAPT-X ego-relative offset."""
        if self._ego is None:
            raise SimulatorUnavailableError(
                "no ego vehicle has been spawned, so there is no frame to offset from"
            )
        transform = self._ego_transform()
        ego_xyz = (
            float(transform.location.x),
            float(transform.location.y),
            float(transform.location.z),
        )
        return ego_offset_to_carla_world(
            (forward_m, left_m, up_m), ego_xyz, float(transform.rotation.yaw)
        )

    # -- live extension: anchored placement ---------------------------------
    def anchor(self) -> WorldAnchor:
        """The ego's pose now, to place actors relative to later."""
        transform = self._ego_transform()
        return WorldAnchor(
            float(transform.location.x),
            float(transform.location.y),
            float(transform.location.z),
            float(transform.rotation.yaw),
        )

    def spawn_relative_to(
        self,
        anchor: WorldAnchor,
        blueprint_id: str,
        *,
        forward_m: float,
        left_m: float = 0.0,
        up_m: float = 0.0,
        physics: bool = False,
    ) -> Any:
        """Spawn an actor at an ADAPT-X offset from a captured anchor pose.

        Like :meth:`spawn_ahead_of_ego`, but relative to where the ego *was*
        when the anchor was taken, so a driving ego does not drag the actor
        along. ``physics=False`` places the actor (ADR-047/054); ``True``
        leaves it to the simulator, for traffic that drives itself.
        """
        carla = self._require_carla()
        world = self._require_world()
        blueprint = self._find_blueprint(blueprint_id)
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", SCENARIO_ROLE)
        cx, cy, cz = ego_offset_to_carla_world(
            (forward_m, left_m, up_m + SPAWN_CLEARANCE_M),
            (anchor.x, anchor.y, anchor.z),
            anchor.yaw_deg,
        )
        transform = carla.Transform(
            carla.Location(x=cx, y=cy, z=cz), carla.Rotation(yaw=anchor.yaw_deg)
        )
        actor = world.try_spawn_actor(blueprint, transform)
        if actor is None:
            raise SimulatorUnavailableError(
                f"could not spawn '{blueprint_id}' {forward_m} m ahead of the anchor; "
                "the location is probably occupied or off-road",
                details={"blueprint": blueprint_id, "forward_m": forward_m},
            )
        self._actors.append(actor)
        self._stand_offsets[int(actor.id)] = _stand_offset(actor)
        if physics:
            actor.set_simulate_physics(True)
        else:
            actor.set_simulate_physics(False)
            self.place_relative_to(actor, anchor, forward_m=forward_m, left_m=left_m, up_m=up_m)
        return actor

    def place_relative_to(
        self,
        actor: Any,
        anchor: WorldAnchor,
        *,
        forward_m: float,
        left_m: float = 0.0,
        up_m: float = 0.0,
        yaw_offset_deg: float = 0.0,
    ) -> None:
        """Move a placed actor to an ADAPT-X offset from a captured anchor pose."""
        carla = self._require_carla()
        stand = self._stand_offsets.get(int(actor.id), 0.0)
        cx, cy, cz = ego_offset_to_carla_world(
            (forward_m, left_m, up_m + stand), (anchor.x, anchor.y, anchor.z), anchor.yaw_deg
        )
        actor.set_transform(
            carla.Transform(
                carla.Location(x=cx, y=cy, z=cz),
                carla.Rotation(yaw=anchor.yaw_deg + yaw_offset_deg),
            )
        )

    def destroy_actor(self, actor: Any) -> None:
        """Remove one scenario actor now rather than at close. Idempotent."""
        if actor in self._actors:
            self._actors.remove(actor)
            self._stand_offsets.pop(int(actor.id), None)
            self._safely(actor.destroy, f"destroy actor {getattr(actor, 'id', '?')}")

    # -- live extension: traffic --------------------------------------------
    def spawn_traffic_vehicle(self, blueprint_id: str, spawn_index: int) -> Any | None:
        """Spawn a vehicle at a map spawn point and hand it to the Traffic Manager.

        Traffic drives itself: CARLA's Traffic Manager, in lockstep with the
        session and seeded from the settings, so the same seed replays the
        same traffic on the same server build. Returns ``None`` when the
        point is occupied - traffic is best-effort, a scenario is not.
        Perception never learns that an actor is autopiloted.
        """
        world = self._require_world()
        spawn_points = world.get_map().get_spawn_points()
        if spawn_index < 0 or spawn_index >= len(spawn_points):
            return None
        if spawn_index == self._ego_spawn_index:
            return None
        blueprint = self._find_blueprint(blueprint_id)
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", TRAFFIC_ROLE)
        actor = world.try_spawn_actor(blueprint, spawn_points[spawn_index])
        if actor is None:
            return None
        self._actors.append(actor)
        actor.set_simulate_physics(True)
        actor.set_autopilot(True, self._traffic_manager_port())
        return actor

    def _traffic_manager_port(self) -> int:
        if self._traffic_manager is None:
            client = self._client
            if client is None:
                raise SimulatorUnavailableError("the simulation session is not connected")
            manager = client.get_trafficmanager(self._tm_port)
            manager.set_synchronous_mode(self._settings.synchronous_mode)
            manager.set_random_device_seed(self._settings.seed)
            self._traffic_manager = manager
        return int(self._traffic_manager.get_port())

    # -- live extension: ego control ----------------------------------------
    def apply_ego_control(self, *, throttle: float, brake: float, steer_left: float) -> None:
        """Actuate the ego. ``steer_left`` is ADAPT-X-positive-left; flipped here.

        The one place a control command meets ``carla.VehicleControl``. The
        sign flip is the same handedness conversion as every other value that
        crosses this boundary (ADR-043): CARLA's positive steer is to the
        right.
        """
        if not self._drive_ego:
            raise SimulatorUnavailableError(
                "this session was opened with drive_ego=False; the ego is placed, not driven"
            )
        if self._ego is None:
            raise SimulatorUnavailableError("no ego vehicle has been spawned")
        carla = self._require_carla()
        control = carla.VehicleControl(
            throttle=float(max(0.0, min(1.0, throttle))),
            brake=float(max(0.0, min(1.0, brake))),
            steer=float(max(-1.0, min(1.0, -steer_left))),
        )
        self._ego.apply_control(control)
        self._last_control = (control.throttle, control.brake, control.steer)

    def release_ego(self) -> None:
        """Neutral throttle, brake on. Safe to call on a closed session."""
        if self._ego is None or not self._drive_ego:
            return
        carla = self._require_carla()
        self._ego.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))
        self._last_control = (0.0, 1.0, 0.0)

    def ego_observation(self, lookahead_m: float) -> EgoObservation:
        """The ego's odometry and the lane centre ahead, for the controller.

        Speed and heading are the simulator's report of the ego itself - its
        odometry, not a perception of anything. The lane figures come from
        the map's driving-lane waypoint under the ego and one ``lookahead_m``
        further along: road geometry, which a real vehicle would take from
        its HD map. Nothing here describes another actor.
        """
        if self._ego is None:
            raise SimulatorUnavailableError("no ego vehicle has been spawned")
        transform = self._ego_transform()
        velocity = self._ego.get_velocity()
        speed = math.hypot(float(velocity.x), float(velocity.y))
        ego_yaw = float(transform.rotation.yaw)
        heading = carla_yaw_to_heading_rad(ego_yaw)

        error: float | None = None
        offset: float | None = None
        try:
            world = self._require_world()
            waypoint = world.get_map().get_waypoint(transform.location)
            ahead = waypoint.next(lookahead_m) if waypoint is not None else []
            if waypoint is not None and ahead:
                target = ahead[0].transform
                ego_xyz = (
                    float(transform.location.x),
                    float(transform.location.y),
                    float(transform.location.z),
                )
                target_xyz = (
                    float(target.location.x),
                    float(target.location.y),
                    float(target.location.z),
                )
                relative = carla_world_to_ego(target_xyz, ego_xyz, ego_yaw)
                # Bearing to the lane-centre point ahead, in the ego frame:
                # positive means the target is to the left.
                error = math.atan2(relative.y, max(relative.x, 1e-3))
                centre = waypoint.transform.location
                offset = carla_world_to_ego(
                    (float(centre.x), float(centre.y), float(centre.z)), ego_xyz, ego_yaw
                ).y
        except Exception as exc:  # pragma: no cover - map query failures are non-fatal
            logger.debug("lane query failed", extra={"context": {"error": str(exc)}})
        return EgoObservation(
            speed_mps=speed,
            heading_rad=heading,
            lane_heading_error_rad=error,
            lane_offset_m=offset,
        )

    @property
    def last_control(self) -> tuple[float, float, float] | None:
        """The last (throttle, brake, carla_steer) applied, for the record."""
        return self._last_control

    # -- live extension: safety and visualisation sensors -------------------
    def attach_collision_sensor(self) -> None:
        """Attach the collision sensor to the ego. A safety fallback, not perception."""
        carla = self._require_carla()
        world = self._require_world()
        blueprint = self._find_blueprint(COLLISION_BLUEPRINT)
        sensor = world.try_spawn_actor(blueprint, carla.Transform(), attach_to=self._ego)
        if sensor is None:
            raise SimulatorUnavailableError("could not attach the collision sensor")

        def on_collision(event: Any) -> None:
            impulse = getattr(event, "normal_impulse", None)
            magnitude = (
                math.sqrt(float(impulse.x) ** 2 + float(impulse.y) ** 2 + float(impulse.z) ** 2)
                if impulse is not None
                else 0.0
            )
            record = CollisionEvent(
                simulation_frame=int(getattr(event, "frame", self._sim_frame or 0)),
                other_type_id=str(getattr(getattr(event, "other_actor", None), "type_id", "?")),
                impulse=magnitude,
            )
            with self._sensor_lock:
                self._collisions.append(record)

        sensor.listen(on_collision)
        self._collision_sensor = sensor

    def collisions(self) -> list[CollisionEvent]:
        """Every contact recorded so far, oldest first."""
        with self._sensor_lock:
            return list(self._collisions)

    def attach_camera(self, *, width: int, height: int, fov_deg: float) -> None:
        """Attach a forward RGB camera to the ego, for display only."""
        carla = self._require_carla()
        world = self._require_world()
        blueprint = self._find_blueprint(CAMERA_BLUEPRINT)
        for key, value in {
            "image_size_x": str(width),
            "image_size_y": str(height),
            "fov": str(fov_deg),
        }.items():
            if blueprint.has_attribute(key):
                blueprint.set_attribute(key, value)
        cx, cy, cz = adaptx_offset_to_carla(*CAMERA_MOUNT_M)
        sensor = world.try_spawn_actor(
            blueprint, carla.Transform(carla.Location(x=cx, y=cy, z=cz)), attach_to=self._ego
        )
        if sensor is None:
            raise SimulatorUnavailableError("could not attach the RGB camera")

        def on_image(image: Any) -> None:
            # Latest only: a camera in synchronous mode delivers one image per
            # tick, and the dashboard asks for whichever is newest.
            latest = CameraImage(
                frame=int(image.frame),
                width=int(image.width),
                height=int(image.height),
                bgra=bytes(image.raw_data),
            )
            with self._sensor_lock:
                self._camera_latest = latest

        sensor.listen(on_image)
        self._camera = sensor

    def camera_image(self) -> CameraImage | None:
        """The newest camera frame, or ``None`` before one arrived."""
        with self._sensor_lock:
            return self._camera_latest

    @property
    def sensor_actor_ids(self) -> list[int]:
        """Ids of every sensor the session attached."""
        return [
            int(sensor.id)
            for sensor in (self._sensor, self._collision_sensor, self._camera)
            if sensor is not None
        ]

    # -- internals ---------------------------------------------------------
    def _connect(self) -> None:
        carla = self._require_carla()
        try:
            client = carla.Client(self._settings.host, self._settings.port)
            client.set_timeout(self._settings.timeout_s)
            world = (
                client.load_world(self._settings.town)
                if self._settings.town
                else client.get_world()
            )
        except Exception as exc:
            raise SimulatorUnavailableError(
                f"could not reach CARLA at {self._settings.host}:{self._settings.port}: {exc}. "
                "Is the server running?",
                details={"host": self._settings.host, "port": self._settings.port},
            ) from exc

        self._client = client
        self._world = world
        self._map_name = str(world.get_map().name)
        # The server's own answer. The ``carla`` package exposes no
        # ``__version__`` (0.9.16), so this is the only version a run can
        # truthfully record; kept None rather than guessed if the call fails.
        try:
            self._server_version = str(client.get_server_version())
        except Exception:  # pragma: no cover - older or unusual server builds
            logger.warning("CARLA server version unavailable", exc_info=True)
            self._server_version = None

    def _reclaim_stale_actors(self) -> None:
        """Destroy actors an earlier ADAPT-X process left behind, and nothing else.

        A backend killed without a clean stop (measured: the desktop app
        terminating the process between turns) leaves its ego, sensors,
        Traffic-Manager traffic and scripted actors on the server. Orphaned
        traffic has no client driving it and sits in the lane at 0 m/s; the
        next session then spawns into that road and, correctly, stops behind
        a "vehicle at 7.8 m, 0.0 m/s". Only actors carrying one of
        ``OWNED_ROLES`` in ``role_name`` are touched, plus the sensors
        attached to them. If such orphans exist and the world was left
        synchronous, the dead process also left it that way, so the settings
        this session will restore on close are asynchronous - otherwise the
        next client finds a server that appears hung (Experiment 016).
        """
        if not self._settings.reclaim_stale_actors:
            return
        world = self._require_world()
        # Measured on 0.9.16: a fresh client's actor list is EMPTY on a world
        # left synchronous until one frame is produced. One bootstrap tick (or
        # one asynchronous frame) refreshes it; this happens before the live
        # loop exists, so the loop remains the only ticker while it runs.
        try:
            if world.get_settings().synchronous_mode:
                world.tick()
            else:
                world.wait_for_tick()
        except Exception as exc:  # pragma: no cover - a hung server is reported, not hidden
            logger.warning(
                "could not refresh the actor list", extra={"context": {"error": str(exc)}}
            )
        stale: list[Any] = []
        stale_ids: set[int] = set()
        for actor in world.get_actors():
            attributes = getattr(actor, "attributes", None) or {}
            if attributes.get("role_name") in OWNED_ROLES:
                stale.append(actor)
                stale_ids.add(int(actor.id))
        if not stale:
            return
        for actor in world.get_actors():
            parent = getattr(actor, "parent", None)
            if (
                parent is not None
                and int(parent.id) in stale_ids
                and int(actor.id) not in stale_ids
            ):
                stale.append(actor)
                stale_ids.add(int(actor.id))

        settings = world.get_settings()
        left_synchronous = bool(settings.synchronous_mode)
        if left_synchronous:
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            self._safely(lambda: world.apply_settings(settings), "release stale synchronous mode")
            # The dead process's Traffic Manager may be gone; the port is the one
            # this session uses, so releasing it is harmless when it is live.
            client = self._client
            if client is not None:
                self._safely(
                    lambda: client.get_trafficmanager(self._tm_port).set_synchronous_mode(False),
                    "release stale traffic manager",
                )
        for actor in stale:
            if str(getattr(actor, "type_id", "")).startswith("sensor."):
                self._safely(actor.stop, f"stop stale sensor {actor.id}")
        for actor in stale:
            self._safely(actor.destroy, f"destroy stale actor {actor.id}")
        self._reclaimed_actors = len(stale)
        logger.warning(
            "reclaimed stale ADAPT-X actors left by an earlier process",
            extra={
                "context": {
                    "count": len(stale),
                    "left_synchronous": left_synchronous,
                    "types": sorted({str(getattr(a, "type_id", "?")) for a in stale}),
                }
            },
        )

    def _configure_world(self) -> None:
        world = self._require_world()
        self._original_settings = world.get_settings()
        if self._reclaimed_actors and self._original_settings.synchronous_mode:
            # Belt and braces: the reclaim step already switched it off.
            self._original_settings.synchronous_mode = False
            self._original_settings.fixed_delta_seconds = None

        settings = world.get_settings()
        settings.synchronous_mode = self._settings.synchronous_mode
        settings.fixed_delta_seconds = self._settings.fixed_delta_seconds
        world.apply_settings(settings)

        if not self._settings.sweep_matches_timestep:
            logger.warning(
                "LiDAR rotation frequency does not match the timestep; each tick will "
                "deliver a partial sweep",
                extra={
                    "context": {
                        "rotation_frequency_hz": self._settings.lidar_rotation_frequency_hz,
                        "implied_hz": 1.0 / self._settings.fixed_delta_seconds,
                    }
                },
            )

    def _spawn_ego(self) -> None:
        world = self._require_world()
        blueprint = self._find_blueprint(self._settings.ego_blueprint)
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", EGO_ROLE)

        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            raise SimulatorUnavailableError(
                f"map '{self._map_name}' offers no spawn points",
                details={"map": self._map_name},
            )
        # A pinned index is used and nothing else is tried: the caller asked for
        # that pose, and quietly taking another would run a different scene.
        # Otherwise walk the map's spawn points in order and take the first
        # that accepts - deterministic for a given map and world state, and
        # robust to a point being occupied, which the first live run met when
        # an actor left behind by a killed process sat on index 0. The index
        # used is recorded on the status so a run is reproducible from what it
        # reports (ADR-049).
        pinned = self._settings.ego_spawn_index
        if pinned is not None:
            if pinned >= len(spawn_points):
                raise SimulatorUnavailableError(
                    f"ego_spawn_index {pinned} is out of range: map '{self._map_name}' has "
                    f"{len(spawn_points)} spawn points",
                    details={"ego_spawn_index": pinned, "spawn_points": len(spawn_points)},
                )
            candidates = [(pinned, spawn_points[pinned])]
        else:
            candidates = list(enumerate(spawn_points))
        actor = None
        for index, transform in candidates:
            actor = world.try_spawn_actor(blueprint, transform)
            if actor is not None:
                self._ego_spawn_index = index
                self._ego_spawn_transform = transform
                break
        if actor is None:
            where = (
                f"spawn point {pinned}"
                if pinned is not None
                else f"any of the {len(spawn_points)} spawn points"
            )
            raise SimulatorUnavailableError(
                f"could not spawn the ego vehicle '{self._settings.ego_blueprint}' at {where} "
                f"of '{self._map_name}'; the location is occupied or colliding",
                details={
                    "blueprint": self._settings.ego_blueprint,
                    "spawn_points": len(spawn_points),
                    "ego_spawn_index": pinned,
                },
            )
        if self._ego_spawn_index:
            logger.info(
                "ego spawned at a later spawn point; earlier ones refused",
                extra={"context": {"spawn_index": self._ego_spawn_index}},
            )
        self._ego = actor
        self._actors.append(actor)
        self._ground_ego(actor, self._ego_spawn_transform)

    def _ground_ego(self, actor: Any, spawn_transform: Any) -> None:
        """Stop the ego simulating physics and stand it on the road surface.

        A map spawn point sits about 0.6 m above the road so a spawned vehicle
        has clearance; with physics on, the ego then falls for half a second
        and every ego-relative placement, and the LiDAR, falls with it
        (Experiment 011). The ego never drives (ADR-047), so physics buys
        nothing: it is switched off and the ego is set down at the road
        surface under the spawn point, which is where the measured settled
        pose was. If the map has no road under the point the spawn height is
        kept and the fact is logged.
        """
        carla = self._require_carla()
        world = self._require_world()
        if self._drive_ego:
            # A driven ego needs physics; it settles the 0.6 m spawn clearance
            # itself. The spawn transform stays the pre-tick reference.
            actor.set_simulate_physics(True)
            return
        actor.set_simulate_physics(False)
        waypoint = world.get_map().get_waypoint(spawn_transform.location)
        if waypoint is None:
            logger.warning(
                "no road under the ego spawn point; keeping the spawn height",
                extra={"context": {"spawn_index": self._ego_spawn_index}},
            )
            return
        grounded = carla.Transform(
            carla.Location(
                x=float(spawn_transform.location.x),
                y=float(spawn_transform.location.y),
                z=float(waypoint.transform.location.z),
            ),
            spawn_transform.rotation,
        )
        actor.set_transform(grounded)
        self._ego_spawn_transform = grounded

    def _attach_lidar(self) -> None:
        carla = self._require_carla()
        world = self._require_world()
        settings = self._settings
        blueprint = self._find_blueprint(LIDAR_BLUEPRINT)

        attributes = {
            "channels": str(settings.lidar_channels),
            "range": str(settings.lidar_range_m),
            "points_per_second": str(settings.lidar_points_per_second),
            "rotation_frequency": str(settings.lidar_rotation_frequency_hz),
            "upper_fov": str(settings.lidar_upper_fov_deg),
            "lower_fov": str(settings.lidar_lower_fov_deg),
            "dropoff_general_rate": str(settings.lidar_dropoff_general_rate),
            # The sensor's own random number generator - point drop-off and
            # atmospheric attenuation draw from it. Left unseeded, two runs
            # of one scenario return different point counts and everything
            # downstream diverges (Experiment 011); seeded, the sensor is
            # repeatable. ``settings.seed`` is the scenario seed when a
            # scenario is running (ADR-046).
            "noise_seed": str(settings.seed),
        }
        for key, value in attributes.items():
            if blueprint.has_attribute(key):
                blueprint.set_attribute(key, value)

        # The mount is configured in ADAPT-X coordinates and converted here -
        # the only place an ADAPT-X value travels back into CARLA's frame.
        cx, cy, cz = adaptx_offset_to_carla(
            settings.lidar_x_m, settings.lidar_y_m, settings.lidar_z_m
        )
        transform = carla.Transform(carla.Location(x=cx, y=cy, z=cz))
        sensor = world.try_spawn_actor(blueprint, transform, attach_to=self._ego)
        if sensor is None:
            raise SimulatorUnavailableError(
                "could not attach the LiDAR sensor to the ego vehicle",
                details={"blueprint": LIDAR_BLUEPRINT},
            )
        sensor.listen(self._queue.put)
        self._sensor = sensor

    def _await_measurement(self, expected_frame: int) -> Any:
        """Return the sensor measurement belonging to ``expected_frame``.

        Frames older than the tick are discarded rather than returned: in
        synchronous mode a stale measurement would silently pair this tick's
        ground truth with the previous tick's points.
        """
        deadline = self._settings.sensor_timeout_s
        while True:
            try:
                measurement = self._queue.get(timeout=deadline)
            except queue.Empty as exc:
                self._state = SimulationState.ERROR
                self._detail = "sensor timeout"
                raise SimulatorUnavailableError(
                    f"no LiDAR frame arrived within {deadline}s of tick {expected_frame}; "
                    "the sensor may have failed or the timestep may be too small",
                    details={"expected_frame": expected_frame, "timeout_s": deadline},
                ) from exc
            if int(measurement.frame) >= expected_frame:
                return measurement

    def _find_blueprint(self, blueprint_id: str) -> Any:
        world = self._require_world()
        library = world.get_blueprint_library()
        try:
            return library.find(blueprint_id)
        except Exception as exc:
            raise SimulatorUnavailableError(
                f"blueprint '{blueprint_id}' is not available on this CARLA server",
                details={"blueprint": blueprint_id},
            ) from exc

    def _describe_actor(
        self, actor: Any, ego_xyz: tuple[float, float, float], ego_yaw: float
    ) -> GroundTruthActor | None:
        """Convert one CARLA actor into a ground-truth record.

        Sensors and spectators are skipped: they are not objects in the scene.
        """
        type_id = str(actor.type_id)
        if type_id.startswith("sensor.") or type_id.startswith("spectator"):
            return None

        transform = actor.get_transform()
        location = (
            float(transform.location.x),
            float(transform.location.y),
            float(transform.location.z),
        )
        position = carla_world_to_ego(location, ego_xyz, ego_yaw)
        velocity_raw = actor.get_velocity()
        velocity = carla_vector_to_ego(
            (float(velocity_raw.x), float(velocity_raw.y), float(velocity_raw.z)), ego_yaw
        )

        extent = getattr(getattr(actor, "bounding_box", None), "extent", None)
        dimensions = (
            Dimensions(
                length=max(float(extent.x) * 2.0, 1e-3),
                width=max(float(extent.y) * 2.0, 1e-3),
                height=max(float(extent.z) * 2.0, 1e-3),
            )
            if extent is not None
            else Dimensions(length=1e-3, width=1e-3, height=1e-3)
        )

        heading = carla_yaw_to_heading_rad(
            float(transform.rotation.yaw)
        ) - carla_yaw_to_heading_rad(ego_yaw)
        world_position = carla_location_to_vector3(*location)

        return GroundTruthActor(
            actor_id=int(actor.id),
            type_id=type_id,
            object_class=classify_blueprint(type_id),
            is_ego=self._ego is not None and int(actor.id) == int(self._ego.id),
            position=position,
            velocity=velocity,
            heading_rad=heading,
            dimensions=dimensions,
            world_position=world_position,
            distance_m=float((position.x**2 + position.y**2) ** 0.5),
        )

    def _require_carla(self) -> Any:
        if self._carla is None:
            self._carla = load_carla_module()
        return self._carla

    def _require_world(self) -> Any:
        if self._world is None:
            raise SimulatorUnavailableError("the simulation session is not connected")
        return self._world

    def _destroy_actors(self) -> None:
        """Destroy every owned actor, as one server-side batch where the API allows.

        ``apply_batch_sync`` with ``DestroyActor`` commands is how the CARLA
        examples tear traffic down; a per-actor ``destroy()`` is the fallback
        for stand-ins and older builds. Either way one failure never stops
        the rest.
        """
        actors = list(reversed(self._actors))
        command = getattr(self._carla, "command", None)
        client = self._client
        if actors and command is not None and client is not None:
            try:
                client.apply_batch_sync([command.DestroyActor(actor) for actor in actors], True)
                return
            except Exception as exc:
                logger.warning(
                    "batch destroy failed; destroying one by one",
                    extra={"context": {"error": str(exc)}},
                )
        for actor in actors:
            self._safely(actor.destroy, f"destroy actor {getattr(actor, 'id', '?')}")

    @staticmethod
    def _safely(action: Any, description: str) -> None:
        """Run a cleanup step, logging rather than raising if it fails.

        Cleanup must not abort half-way: one actor that refuses to die should
        not strand the other twenty. The failure is logged, never swallowed
        silently.
        """
        try:
            action()
        except Exception as exc:
            logger.warning(
                "CARLA cleanup step failed",
                extra={"context": {"step": description, "error": str(exc)}},
            )


def _drain(target: queue.Queue[Any]) -> None:
    """Discard anything the sensor delivered after the last consumed tick."""
    while True:
        try:
            target.get_nowait()
        except queue.Empty:
            return
