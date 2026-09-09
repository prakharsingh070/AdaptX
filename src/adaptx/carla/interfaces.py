"""CARLA client contract.

Two implementations exist: :class:`adaptx.carla.client.CarlaClient` (the real
boundary, requiring the optional ``carla`` package) and
:class:`adaptx.carla.mock.MockCarlaSimulatorClient` (an in-process fake used by
development and tests). Both honour this contract, so callers never branch on
which one is active.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from adaptx.carla.models import CarlaActorRef, CarlaWorldInfo
from adaptx.models.point_cloud import PointCloudFrame
from adaptx.models.vehicle import VehicleState


class CarlaSimulatorClient(ABC):
    """Lifecycle and data access for a CARLA simulation."""

    name: str = "carla_client"
    #: True for fakes. Data produced by a fake is labelled SYNTHETIC_TEST.
    is_mock: bool = False

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """True when a simulator session is established."""

    @abstractmethod
    def connect(self) -> CarlaWorldInfo:
        """Open a session and return the loaded world.

        Raises:
            adaptx.core.exceptions.SimulatorUnavailableError: CARLA is not
                installed, not enabled, or not reachable.
        """

    @abstractmethod
    def disconnect(self) -> None:
        """Close the session and release simulator resources. Idempotent."""

    @abstractmethod
    def get_world(self) -> CarlaWorldInfo:
        """Return information about the currently loaded world."""

    @abstractmethod
    def load_world(self, map_name: str) -> CarlaWorldInfo:
        """Load ``map_name`` and return the new world."""

    @abstractmethod
    def get_vehicle_state(self) -> VehicleState:
        """Return the current ego-vehicle state."""

    @abstractmethod
    def get_sensor_data(self) -> PointCloudFrame | None:
        """Return the latest LiDAR frame, or ``None`` when none is available."""

    @abstractmethod
    def spawn_vehicle(self, blueprint_id: str, *, role: str = "") -> CarlaActorRef:
        """Spawn a vehicle actor and return a handle to it."""

    @abstractmethod
    def spawn_actor(self, blueprint_id: str, *, role: str = "") -> CarlaActorRef:
        """Spawn a non-vehicle actor (pedestrian, prop, sensor) and return a handle."""

    @abstractmethod
    def destroy_actor(self, actor_id: int) -> bool:
        """Destroy an actor. Returns False when the actor is unknown."""
