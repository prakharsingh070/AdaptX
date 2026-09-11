"""A stand-in for the CARLA Python package.

CARLA is optional and is not installed in this environment, but the session
lifecycle - spawn order, sensor delivery, cleanup on failure, restoring world
settings - is exactly the code most likely to leak an actor or hang a server.
That logic needs exercising, so this module implements the small surface of
CARLA that :mod:`adaptx.carla.session` actually touches.

**This is a test double, not a simulator.** It does no physics, no rendering
and no ray casting worth the name: LiDAR points are generated geometrically
from actor positions. Nothing measured against it is a CARLA measurement, and
no figure produced with it may be reported as simulator performance.

It does model the one thing that matters for correctness: **points are emitted
in CARLA's own left-handed frame** (+y to the right). A test that places a
target to the ego's left and finds it on the left in ADAPT-X output has
therefore genuinely exercised the handedness flip - if the conversion were
dropped, the object would come out mirrored.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field
from typing import Any

import numpy as np

#: Ground height below the sensor origin, matching the default roof mount.
GROUND_DROP_M = 1.8


# ---------------------------------------------------------------------------
# geometry primitives (mirroring carla.Location / Rotation / Transform)
# ---------------------------------------------------------------------------
@dataclass
class Location:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Rotation:
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0


@dataclass
class Transform:
    location: Location = field(default_factory=Location)
    rotation: Rotation = field(default_factory=Rotation)


@dataclass
class Vector3D:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class BoundingBox:
    """A vehicle-shaped box whose bottom sits at the actor origin, as CARLA's do."""

    extent: Vector3D = field(default_factory=lambda: Vector3D(2.3, 1.0, 0.8))
    location: Vector3D = field(default_factory=lambda: Vector3D(0.0, 0.0, 0.8))


@dataclass
class FakeWaypoint:
    transform: Transform


@dataclass
class WorldSettings:
    synchronous_mode: bool = False
    fixed_delta_seconds: float | None = None


# ---------------------------------------------------------------------------
# actors
# ---------------------------------------------------------------------------
class FakeActor:
    """An actor in the fake world. Destroys itself out of the world registry."""

    def __init__(self, world: FakeWorld, actor_id: int, type_id: str, transform: Transform) -> None:
        self._world = world
        self.id = actor_id
        self.type_id = type_id
        self._transform = transform
        # A real server does not report a spawned actor's pose until it has
        # ticked once: ``get_transform()`` answers the world origin with zero
        # yaw in the meantime (CARLA 0.9.16, synchronous mode, first live
        # run). The fake reproduces that so code offsetting from a fresh
        # ego cannot pass here and fail on the simulator.
        self.settled = False
        self.blueprint_attributes: dict[str, str] = {}
        self.simulate_physics = True
        self.bounding_box = BoundingBox()
        self.destroyed = False
        self.velocity = Vector3D()

    def get_transform(self) -> Transform:
        if not self.settled:
            return Transform()
        return self._transform

    def set_transform(self, transform: Transform) -> None:
        self._transform = transform

    def get_velocity(self) -> Vector3D:
        return self.velocity

    def set_simulate_physics(self, enabled: bool) -> None:
        self.simulate_physics = enabled

    def destroy(self) -> bool:
        self.destroyed = True
        self._world.actors.pop(self.id, None)
        return True


class FakeSensor(FakeActor):
    """A LiDAR that emits a generated scan on every tick while listening."""

    def __init__(self, world: FakeWorld, actor_id: int, type_id: str, transform: Transform) -> None:
        super().__init__(world, actor_id, type_id, transform)
        self._callback: Any | None = None
        self.listening = False
        self.stopped = False

    def listen(self, callback: Any) -> None:
        self._callback = callback
        self.listening = True

    def stop(self) -> None:
        self.listening = False
        self.stopped = True

    def deliver(self, frame: int, timestamp: float) -> None:
        """Emit one scan for the given simulator frame."""
        if not self.listening or self._callback is None:
            return
        points = self._world.scan(self._transform)
        self._callback(FakeLidarMeasurement(frame=frame, timestamp=timestamp, points=points))


@dataclass
class FakeLidarMeasurement:
    """Mirrors ``carla.LidarMeasurement``: a flat float32 buffer of xyzi."""

    frame: int
    timestamp: float
    points: np.ndarray

    @property
    def raw_data(self) -> bytes:
        """Points packed exactly as CARLA packs them: little-endian float32."""
        flat = np.asarray(self.points, dtype="<f4").reshape(-1)
        return struct.pack(f"<{flat.size}f", *flat.tolist())


# ---------------------------------------------------------------------------
# blueprints
# ---------------------------------------------------------------------------
class FakeBlueprint:
    def __init__(self, type_id: str) -> None:
        self.id = type_id
        self.attributes: dict[str, str] = {
            "role_name": "",
            "channels": "32",
            "range": "100",
            "points_per_second": "56000",
            "rotation_frequency": "20",
            "upper_fov": "10",
            "lower_fov": "-30",
            "dropoff_general_rate": "0.0",
            "noise_seed": "0",
        }

    def has_attribute(self, key: str) -> bool:
        return key in self.attributes

    def set_attribute(self, key: str, value: str) -> None:
        self.attributes[key] = value


class FakeBlueprintLibrary:
    def __init__(self, known: set[str]) -> None:
        self._known = known

    def find(self, type_id: str) -> FakeBlueprint:
        if type_id not in self._known:
            raise IndexError(f"blueprint '{type_id}' not found")
        return FakeBlueprint(type_id)


# ---------------------------------------------------------------------------
# world
# ---------------------------------------------------------------------------
class FakeMap:
    def __init__(self, name: str) -> None:
        self.name = name
        # The first point is the world origin facing +x, which keeps the
        # fake's arithmetic inspectable: an ego-relative offset is then also a
        # world offset. A second point (also at the origin, so placement
        # arithmetic is unchanged) exists so a test can refuse the first, as
        # Town10HD_Opt does on a real CARLA 0.9.16 server. Held as one stable
        # list so a world can identify which point a spawn request used.
        self.spawn_points: list[Transform] = [
            Transform(Location(0.0, 0.0, 0.0), Rotation()),
            Transform(Location(0.0, 0.0, 0.0), Rotation()),
        ]

    def get_spawn_points(self) -> list[Transform]:
        return list(self.spawn_points)

    def get_waypoint(self, location: Location) -> FakeWaypoint:
        """The road under ``location``: a flat plane at the origin's height."""
        return FakeWaypoint(Transform(Location(location.x, location.y, 0.0), Rotation()))


@dataclass
class FakeTimestamp:
    elapsed_seconds: float


@dataclass
class FakeSnapshot:
    frame: int
    timestamp: FakeTimestamp


class FakeWorld:
    """A world that tracks actors and manufactures LiDAR scans from them."""

    def __init__(
        self,
        map_name: str = "FakeTown",
        *,
        fixed_delta_seconds: float = 0.05,
        spawn_points: bool = True,
        known_blueprints: set[str] | None = None,
    ) -> None:
        self.actors: dict[int, FakeActor] = {}
        self.settings = WorldSettings()
        self.applied_settings: list[WorldSettings] = []
        self._map = FakeMap(map_name)
        self._has_spawn_points = spawn_points
        self._next_id = 1
        self._frame = 100  # A server that has been up a while, like a real one.
        self._elapsed = 0.0
        self._dt = fixed_delta_seconds
        # The blueprints the scenario catalogue uses, plus the sensor. A
        # catalogue scenario asking for anything else fails the same way it
        # would against a real server missing that asset.
        self._known = known_blueprints or {
            "vehicle.tesla.model3",
            "vehicle.audi.tt",
            "vehicle.diamondback.century",
            "walker.pedestrian.0001",
            "sensor.lidar.ray_cast",
        }
        self.refuse_spawn: set[str] = set()
        self.refuse_spawn_point_indices: set[int] = set()
        self.tick_error: Exception | None = None
        self.deliver_frames = True

    # -- settings ----------------------------------------------------------
    def get_settings(self) -> WorldSettings:
        return WorldSettings(
            synchronous_mode=self.settings.synchronous_mode,
            fixed_delta_seconds=self.settings.fixed_delta_seconds,
        )

    def apply_settings(self, settings: WorldSettings) -> None:
        self.settings = settings
        self.applied_settings.append(
            WorldSettings(settings.synchronous_mode, settings.fixed_delta_seconds)
        )

    # -- structure ---------------------------------------------------------
    def get_map(self) -> FakeMap:
        if not self._has_spawn_points:
            self._map.get_spawn_points = lambda: []  # type: ignore[method-assign]
        return self._map

    def get_blueprint_library(self) -> FakeBlueprintLibrary:
        return FakeBlueprintLibrary(self._known)

    def get_actors(self) -> list[FakeActor]:
        return list(self.actors.values())

    def try_spawn_actor(
        self, blueprint: FakeBlueprint, transform: Transform, attach_to: FakeActor | None = None
    ) -> FakeActor | None:
        if blueprint.id in self.refuse_spawn:
            return None
        for index in self.refuse_spawn_point_indices:
            if index < len(self._map.spawn_points) and transform is self._map.spawn_points[index]:
                return None
        actor_id = self._next_id
        self._next_id += 1
        if blueprint.id.startswith("sensor."):
            actor: FakeActor = FakeSensor(self, actor_id, blueprint.id, transform)
        else:
            actor = FakeActor(self, actor_id, blueprint.id, transform)
        # What the blueprint carried at spawn time, so a test can check the
        # attributes the session set (a real actor exposes them too).
        actor.blueprint_attributes = dict(blueprint.attributes)
        self.actors[actor_id] = actor
        return actor

    # -- stepping ----------------------------------------------------------
    def tick(self) -> int:
        if self.tick_error is not None:
            raise self.tick_error
        self._frame += 1
        self._elapsed += self._dt
        for actor in self.actors.values():
            actor.settled = True
        if self.deliver_frames:
            for actor in list(self.actors.values()):
                if isinstance(actor, FakeSensor):
                    actor.deliver(self._frame, self._elapsed)
        return self._frame

    def get_snapshot(self) -> FakeSnapshot:
        return FakeSnapshot(frame=self._frame, timestamp=FakeTimestamp(self._elapsed))

    # -- scan generation ---------------------------------------------------
    def scan(self, sensor_transform: Transform) -> np.ndarray:
        """Generate a scan in the sensor's local **CARLA** frame.

        A ground plane plus a box of returns for every vehicle other than the
        one the sensor is attached to. Crude by design: it exists to give the
        pipeline something with real structure to cluster, not to model a
        beam pattern.
        """
        mount = sensor_transform.location
        blocks: list[np.ndarray] = [_ground_plane(mount.z)]

        for actor in self.actors.values():
            if isinstance(actor, FakeSensor) or actor.type_id.startswith("sensor."):
                continue
            location = actor.get_transform().location
            # Ego sits at the world origin in this fake, so a world position is
            # already sensor-relative once the mount offset is removed.
            dx = location.x - mount.x
            dy = location.y - mount.y
            if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                continue  # the ego itself, under the sensor
            blocks.append(_vehicle_box(dx, dy, -mount.z + 0.8))

        points = np.vstack(blocks)
        intensity = np.full((points.shape[0], 1), 0.75, dtype=np.float64)
        return np.hstack([points, intensity])


def _ground_plane(mount_z: float, extent_m: float = 30.0, spacing_m: float = 0.5) -> np.ndarray:
    """A flat ground return at the sensor's height below the mount."""
    axis = np.arange(-extent_m, extent_m, spacing_m)
    grid_x, grid_y = np.meshgrid(axis, axis, indexing="ij")
    return np.column_stack(
        [grid_x.ravel(), grid_y.ravel(), np.full(grid_x.size, -max(mount_z, GROUND_DROP_M))]
    )


def _vehicle_box(cx: float, cy: float, cz: float, spacing_m: float = 0.12) -> np.ndarray:
    """A dense shell of returns roughly the size of a car, centred at (cx, cy, cz)."""
    length, width, height = 4.5, 1.9, 1.6

    def samples(extent: float) -> np.ndarray:
        return np.linspace(-extent / 2, extent / 2, max(2, math.ceil(extent / spacing_m) + 1))

    xs, ys, zs = samples(length), samples(width), samples(height)
    faces: list[np.ndarray] = []
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    for sign in (-1, 1):
        faces.append(np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, sign * height / 2)]))
    gx, gz = np.meshgrid(xs, zs, indexing="ij")
    for sign in (-1, 1):
        faces.append(np.column_stack([gx.ravel(), np.full(gx.size, sign * width / 2), gz.ravel()]))
    gy, gz = np.meshgrid(ys, zs, indexing="ij")
    for sign in (-1, 1):
        faces.append(np.column_stack([np.full(gy.size, sign * length / 2), gy.ravel(), gz.ravel()]))
    return np.vstack(faces) + np.array([cx, cy, cz])


# ---------------------------------------------------------------------------
# client and module
# ---------------------------------------------------------------------------
class FakeClient:
    def __init__(self, host: str, port: int) -> None:
        self.host = host
        self.port = port
        self.timeout: float | None = None
        self.world = FakeWorld()
        self.loaded: list[str] = []
        self.connect_error: Exception | None = None

    def set_timeout(self, timeout: float) -> None:
        self.timeout = timeout

    def get_server_version(self) -> str:
        return "fake-0.0"

    def get_world(self) -> FakeWorld:
        if self.connect_error is not None:
            raise self.connect_error
        return self.world

    def load_world(self, map_name: str) -> FakeWorld:
        if self.connect_error is not None:
            raise self.connect_error
        self.loaded.append(map_name)
        self.world = FakeWorld(map_name)
        return self.world


class FakeCarlaModule:
    """The subset of the ``carla`` module namespace the session touches."""

    Location = Location
    Rotation = Rotation
    Transform = Transform
    WorldSettings = WorldSettings
    __version__ = "0.9.15-fake"

    def __init__(self, world: FakeWorld | None = None) -> None:
        self.world = world if world is not None else FakeWorld()
        self.clients: list[FakeClient] = []
        self.connect_error: Exception | None = None

    def Client(self, host: str, port: int) -> FakeClient:
        client = FakeClient(host, port)
        client.world = self.world
        client.connect_error = self.connect_error
        self.clients.append(client)
        return client


def fake_module(**world_kwargs: Any) -> FakeCarlaModule:
    """A fake CARLA module wrapping one configurable world."""
    return FakeCarlaModule(FakeWorld(**world_kwargs))
