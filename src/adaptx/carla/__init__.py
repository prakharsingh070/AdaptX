"""CARLA integration boundary.

CARLA is the controlled test environment, not the project
(``docs/knowledge-base/11_carla.md``, ADR-002). The dependency is optional:
ADAPT-X starts, serves every endpoint and passes its test suite without CARLA
installed, reporting ``CARLA: DISCONNECTED``.

Everything CARLA-specific stops here (ADR-042). Downstream perception,
tracking, prediction, mapping, risk and adaptive resolution consume ADAPT-X
contracts and never a simulator object, so nothing below this package knows
whether a frame came from a sensor or a simulation.

Two objects, two questions
--------------------------
:class:`~adaptx.carla.client.CarlaClient` answers *am I connected?* and backs
the status endpoint. :class:`~adaptx.carla.session.CarlaSimulationSession`
answers *is a deterministic simulation running?* and owns actor and sensor
lifetimes.

:mod:`adaptx.carla.conversion` holds the coordinate and time conversions and
imports no simulator at all, which is what makes the part where a sign error
would silently mirror the world testable without CARLA installed.
"""

from adaptx.carla.client import CarlaClient, carla_package_available, carla_package_version
from adaptx.carla.conversion import (
    SIMULATION_EPOCH,
    build_raw_frame,
    carla_points_to_adaptx,
    carla_world_to_ego,
    carla_yaw_to_heading_rad,
    decode_lidar_buffer,
    ego_offset_to_carla_world,
    flip_y,
    simulation_timestamp,
)
from adaptx.carla.ground_truth import (
    GroundTruthActor,
    GroundTruthFrame,
    classify_blueprint,
)
from adaptx.carla.interfaces import CarlaSimulatorClient
from adaptx.carla.mock import MockCarlaSimulatorClient
from adaptx.carla.models import CarlaActorRef, CarlaWorldInfo
from adaptx.carla.session import CarlaSimulationSession, load_carla_module

__all__ = [
    "SIMULATION_EPOCH",
    "CarlaActorRef",
    "CarlaClient",
    "CarlaSimulationSession",
    "CarlaSimulatorClient",
    "CarlaWorldInfo",
    "GroundTruthActor",
    "GroundTruthFrame",
    "MockCarlaSimulatorClient",
    "build_raw_frame",
    "carla_package_available",
    "carla_package_version",
    "carla_points_to_adaptx",
    "carla_world_to_ego",
    "carla_yaw_to_heading_rad",
    "classify_blueprint",
    "decode_lidar_buffer",
    "ego_offset_to_carla_world",
    "flip_y",
    "load_carla_module",
    "simulation_timestamp",
]
