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
import queue
from types import ModuleType
from typing import Any

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
from adaptx.core.exceptions import SimulatorUnavailableError
from adaptx.core.logging import get_logger
from adaptx.models.common import DataSource, Dimensions
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
        self, settings: CarlaSettings, carla_module: ModuleType | Any | None = None
    ) -> None:
        self._settings = settings
        self._carla = carla_module
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
        )

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
        if self._sensor is not None:
            self._safely(self._sensor.stop, "stop LiDAR sensor")
            self._safely(self._sensor.destroy, "destroy LiDAR sensor")
            self._sensor = None

        for actor in reversed(self._actors):
            self._safely(actor.destroy, f"destroy actor {getattr(actor, 'id', '?')}")
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

    def _configure_world(self) -> None:
        world = self._require_world()
        self._original_settings = world.get_settings()

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
            blueprint.set_attribute("role_name", "ego")

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
