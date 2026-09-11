"""CARLA connection management.

Owns the simulator client and reports its state. A missing, disabled or
unreachable CARLA is a normal condition: it is reported as
``CARLA: DISCONNECTED`` with an explanation and never crashes the backend.
"""

from __future__ import annotations

from contextlib import suppress

from adaptx.carla.client import CarlaClient, carla_package_available
from adaptx.carla.interfaces import CarlaSimulatorClient
from adaptx.carla.mock import MockCarlaSimulatorClient
from adaptx.carla.session import CarlaSimulationSession
from adaptx.config.settings import CarlaSettings
from adaptx.core.exceptions import SimulatorUnavailableError
from adaptx.core.logging import get_logger
from adaptx.models.system import (
    CarlaConnectionStatus,
    CarlaStatus,
    SimulationSessionStatus,
    SimulationState,
)

logger = get_logger(__name__)


def build_client(settings: CarlaSettings) -> CarlaSimulatorClient:
    """Return the client implied by configuration.

    The mock is selected only by explicit configuration; it is never a silent
    fallback for a failed real connection.
    """
    if settings.use_mock:
        logger.warning(
            "using the MOCK CARLA simulator - all simulator data is synthetic",
            extra={"context": {"is_mock": True}},
        )
        return MockCarlaSimulatorClient()
    return CarlaClient(settings)


class CarlaService:
    """Holds the CARLA client and exposes its connection status."""

    def __init__(
        self,
        settings: CarlaSettings,
        client: CarlaSimulatorClient | None = None,
        session: CarlaSimulationSession | None = None,
    ) -> None:
        self._settings = settings
        self._client = client if client is not None else build_client(settings)
        self._detail = "CARLA is disabled" if not settings.enabled else "not connected"
        self._world_name: str | None = None
        # A session is *observed*, not created: this service reports state and
        # never starts a simulation of its own. Spawning actors from an
        # unauthenticated HTTP surface is not something the API should offer
        # (ADR-042), so a session is attached by whatever drives it - the
        # smoke runner today.
        self._session = session

    @property
    def client(self) -> CarlaSimulatorClient:
        """The active simulator client."""
        return self._client

    @property
    def session(self) -> CarlaSimulationSession | None:
        """The attached simulation session, or ``None`` when none is running."""
        return self._session

    def attach_session(self, session: CarlaSimulationSession | None) -> None:
        """Record the session whose state should be reported.

        Attaching does not start anything and detaching does not stop
        anything; this only decides what the status endpoint describes.
        """
        self._session = session

    def connect(self) -> bool:
        """Attempt a connection. Returns False and records why if it fails.

        Never raises: the backend must start with or without CARLA.
        """
        if not self._settings.enabled:
            self._detail = "CARLA is disabled (ADAPTX_CARLA__ENABLED=false)"
            return False
        try:
            world = self._client.connect()
        except SimulatorUnavailableError as exc:
            self._detail = exc.message
            logger.warning("CARLA unavailable", extra={"context": {"reason": exc.message}})
            return False
        self._world_name = world.map_name
        self._detail = "connected"
        logger.info("CARLA connected", extra={"context": {"map": world.map_name}})
        return True

    def disconnect(self) -> None:
        """Close the simulator session if one is open."""
        with suppress(SimulatorUnavailableError):  # defensive: already closed
            self._client.disconnect()
        self._world_name = None
        self._detail = "disconnected"

    def status(self) -> CarlaConnectionStatus:
        """Return the current CARLA integration status.

        A session that is open (READY or RUNNING) is a connection, whether or
        not the status client ever connected; a session in ERROR is reported
        as ``ERROR`` with its own detail, so a server that died mid-run does
        not read as merely "disconnected".
        """
        simulation = (
            self._session.status() if self._session is not None else SimulationSessionStatus()
        )
        session_open = simulation.state in (SimulationState.READY, SimulationState.RUNNING)
        connected = self._client.is_connected or session_open
        if simulation.state is SimulationState.ERROR:
            status = CarlaStatus.ERROR
            detail = simulation.detail or "the simulation session failed"
        elif connected:
            status = CarlaStatus.CONNECTED
            detail = self._detail if self._client.is_connected else "live session open"
        else:
            status = CarlaStatus.DISCONNECTED
            detail = self._detail
        return CarlaConnectionStatus(
            status=status,
            enabled=self._settings.enabled,
            client_available=carla_package_available(),
            is_mock=self._client.is_mock,
            host=self._settings.host,
            port=self._settings.port,
            world=self._world_name or simulation.map_name,
            detail=detail,
            simulation=simulation,
        )
