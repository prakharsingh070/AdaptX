"""CARLA integration boundary.

CARLA is the controlled test environment, not the project
(``docs/knowledge-base/11_carla.md``, ADR-002). The dependency is optional:
ADAPT-X starts, serves every endpoint and passes its test suite without CARLA
installed, reporting ``CARLA: DISCONNECTED``.
"""

from adaptx.carla.client import CarlaClient, carla_package_available
from adaptx.carla.interfaces import CarlaSimulatorClient
from adaptx.carla.mock import MockCarlaSimulatorClient
from adaptx.carla.models import CarlaActorRef, CarlaWorldInfo

__all__ = [
    "CarlaActorRef",
    "CarlaClient",
    "CarlaSimulatorClient",
    "CarlaWorldInfo",
    "MockCarlaSimulatorClient",
    "carla_package_available",
]
