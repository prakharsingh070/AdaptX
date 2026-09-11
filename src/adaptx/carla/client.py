"""Real CARLA client boundary.

The ``carla`` Python package is an optional dependency. It is imported lazily
inside :meth:`CarlaClient.connect` so that importing ADAPT-X, starting the API
and running the test suite never require CARLA.

Scope
-----
This class owns the **connection**: reaching a server, reporting the loaded
world, and answering the status endpoint. That is all it has ever done, and
Phase 9 did not widen it.

Running a simulation - deterministic settings, actors, the LiDAR sensor,
ticking and cleanup - belongs to
:class:`adaptx.carla.session.CarlaSimulationSession`, which is a session with a
lifecycle rather than a connection handle (ADR-042). The actor and sensor
methods on this contract therefore say where the operation actually lives
instead of returning invented data. Two objects rather than one, because
"am I connected?" and "is a simulation running?" are genuinely different
questions with different lifetimes.
"""

from __future__ import annotations

import importlib.util
from typing import Any

from adaptx.carla.interfaces import CarlaSimulatorClient
from adaptx.carla.models import CarlaActorRef, CarlaWorldInfo
from adaptx.config.settings import CarlaSettings
from adaptx.core.exceptions import SimulatorUnavailableError
from adaptx.core.logging import get_logger
from adaptx.models.point_cloud import PointCloudFrame
from adaptx.models.vehicle import VehicleState

logger = get_logger(__name__)

_SESSION_NOTE = (
    "not available on the connection client: simulation operations belong to "
    "adaptx.carla.session.CarlaSimulationSession, which owns actor and sensor "
    "lifetimes and cleans them up (Phase 9)"
)


def carla_package_available() -> bool:
    """True when the optional CARLA package can be imported."""
    try:
        return importlib.util.find_spec("carla") is not None
    except (ImportError, ValueError):  # pragma: no cover - defensive
        return False


class CarlaClient(CarlaSimulatorClient):
    """Connects ADAPT-X to a running CARLA server."""

    name = "carla_client"
    is_mock = False

    def __init__(self, settings: CarlaSettings) -> None:
        self._settings = settings
        self._client: Any | None = None
        self._world: Any | None = None

    @property
    def is_connected(self) -> bool:
        return self._client is not None and self._world is not None

    def connect(self) -> CarlaWorldInfo:
        if not self._settings.enabled:
            raise SimulatorUnavailableError(
                "CARLA is disabled (set ADAPTX_CARLA__ENABLED=true to enable it)",
                details={"enabled": False},
            )
        if not carla_package_available():
            raise SimulatorUnavailableError(
                "the optional CARLA package is not installed "
                "(install with: pip install -e .[carla])",
                details={"package_available": False},
            )

        import carla

        try:
            client = carla.Client(self._settings.host, self._settings.port)
            client.set_timeout(self._settings.timeout_s)
            world = client.get_world()
        except Exception as exc:  # CARLA surfaces connection faults as RuntimeError.
            raise SimulatorUnavailableError(
                f"could not reach CARLA at {self._settings.host}:{self._settings.port}: {exc}",
                details={"host": self._settings.host, "port": self._settings.port},
            ) from exc

        self._client = client
        self._world = world
        logger.info(
            "connected to CARLA",
            extra={"context": {"host": self._settings.host, "port": self._settings.port}},
        )
        return self.get_world()

    def disconnect(self) -> None:
        self._world = None
        self._client = None
        logger.info("disconnected from CARLA")

    def get_world(self) -> CarlaWorldInfo:
        world = self._require_world()
        settings = world.get_settings()
        return CarlaWorldInfo(
            map_name=world.get_map().name,
            synchronous_mode=bool(settings.synchronous_mode),
            fixed_delta_seconds=settings.fixed_delta_seconds,
            actor_count=len(world.get_actors()),
        )

    def load_world(self, map_name: str) -> CarlaWorldInfo:
        client = self._require_client()
        self._world = client.load_world(map_name)
        return self.get_world()

    def get_vehicle_state(self) -> VehicleState:
        self._require_world()
        raise SimulatorUnavailableError(f"get_vehicle_state {_SESSION_NOTE}")

    def get_sensor_data(self) -> PointCloudFrame | None:
        self._require_world()
        raise SimulatorUnavailableError(f"get_sensor_data {_SESSION_NOTE}")

    def spawn_vehicle(self, blueprint_id: str, *, role: str = "") -> CarlaActorRef:
        self._require_world()
        raise SimulatorUnavailableError(f"spawn_vehicle {_SESSION_NOTE}")

    def spawn_actor(self, blueprint_id: str, *, role: str = "") -> CarlaActorRef:
        self._require_world()
        raise SimulatorUnavailableError(f"spawn_actor {_SESSION_NOTE}")

    def destroy_actor(self, actor_id: int) -> bool:
        self._require_world()
        raise SimulatorUnavailableError(f"destroy_actor {_SESSION_NOTE}")

    def _require_client(self) -> Any:
        if self._client is None:
            raise SimulatorUnavailableError("CARLA client is not connected")
        return self._client

    def _require_world(self) -> Any:
        if self._world is None:
            raise SimulatorUnavailableError("CARLA client is not connected")
        return self._world
