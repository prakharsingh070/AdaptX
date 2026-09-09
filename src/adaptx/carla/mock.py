"""In-process fake CARLA simulator for development and tests.

Everything this client returns is deterministic and synthetic. Frames and
states are labelled :attr:`~adaptx.models.common.DataSource.SYNTHETIC_TEST` so
they can never be mistaken for sensor measurements, and the API reports
``is_mock: true`` whenever it is active.

It is enabled only by ``ADAPTX_CARLA__USE_MOCK=true``.
"""

from __future__ import annotations

import numpy as np

from adaptx.carla.interfaces import CarlaSimulatorClient
from adaptx.carla.models import CarlaActorRef, CarlaWorldInfo
from adaptx.core.exceptions import SimulatorUnavailableError
from adaptx.models.common import CoordinateFrame, DataSource, Vector3
from adaptx.models.point_cloud import PointCloudFrame
from adaptx.models.vehicle import VehicleState

#: Number of points in a generated synthetic frame.
_MOCK_POINT_COUNT = 512
#: Seed making every generated frame reproducible.
_MOCK_SEED = 20260101


class MockCarlaSimulatorClient(CarlaSimulatorClient):
    """Deterministic fake simulator. Never produces real sensor data."""

    name = "carla_mock"
    is_mock = True

    def __init__(self, map_name: str = "MockTown") -> None:
        self._map_name = map_name
        self._connected = False
        self._frame_id = 0
        self._next_actor_id = 1
        self._actors: dict[int, CarlaActorRef] = {}
        self._rng = np.random.default_rng(_MOCK_SEED)

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> CarlaWorldInfo:
        self._connected = True
        return self.get_world()

    def disconnect(self) -> None:
        self._connected = False
        self._actors.clear()

    def get_world(self) -> CarlaWorldInfo:
        self._require_connected()
        return CarlaWorldInfo(
            map_name=self._map_name,
            synchronous_mode=True,
            fixed_delta_seconds=0.05,
            actor_count=len(self._actors),
        )

    def load_world(self, map_name: str) -> CarlaWorldInfo:
        self._require_connected()
        self._map_name = map_name
        self._actors.clear()
        return self.get_world()

    def get_vehicle_state(self) -> VehicleState:
        """Return a fixed, clearly synthetic ego state."""
        self._require_connected()
        return VehicleState(
            position=Vector3(x=0.0, y=0.0, z=0.0),
            velocity=Vector3(x=0.0, y=0.0, z=0.0),
            acceleration=Vector3(),
            heading_rad=0.0,
            coordinate_frame=CoordinateFrame.WORLD,
            source=DataSource.SYNTHETIC_TEST,
        )

    def get_sensor_data(self) -> PointCloudFrame:
        """Return a reproducible synthetic point cloud.

        Points are drawn from a seeded uniform distribution inside a 40 m box.
        This is test scaffolding, not a sensor model.
        """
        self._require_connected()
        points = self._rng.uniform(-20.0, 20.0, size=(_MOCK_POINT_COUNT, 3))
        frame = PointCloudFrame(
            frame_id=self._frame_id,
            sensor_id="mock_lidar",
            points=points,
            coordinate_frame=CoordinateFrame.LIDAR,
            source=DataSource.SYNTHETIC_TEST,
        )
        self._frame_id += 1
        return frame

    def spawn_vehicle(self, blueprint_id: str, *, role: str = "") -> CarlaActorRef:
        return self._spawn(blueprint_id, role or "vehicle")

    def spawn_actor(self, blueprint_id: str, *, role: str = "") -> CarlaActorRef:
        return self._spawn(blueprint_id, role or "actor")

    def destroy_actor(self, actor_id: int) -> bool:
        self._require_connected()
        return self._actors.pop(actor_id, None) is not None

    def _spawn(self, blueprint_id: str, role: str) -> CarlaActorRef:
        self._require_connected()
        actor = CarlaActorRef(actor_id=self._next_actor_id, blueprint_id=blueprint_id, role=role)
        self._actors[actor.actor_id] = actor
        self._next_actor_id += 1
        return actor

    def _require_connected(self) -> None:
        if not self._connected:
            raise SimulatorUnavailableError("mock CARLA client is not connected")
