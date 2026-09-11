"""What a scenario runner needs from a simulator (Phase 10).

The runner drives a simulator through this protocol and nothing wider. It is
deliberately the exact surface :class:`~adaptx.carla.session.CarlaSimulationSession`
already exposes, so no adapter code changed to satisfy it - the protocol was
extracted from the boundary rather than imposed on it (ADR-048).

Structural rather than nominal: any object with these methods qualifies, which
is what lets the test suite drive the runner against a stand-in and what would
let a second simulator plug in without touching the scenario layer.
"""

from __future__ import annotations

from typing import Any, Protocol

from adaptx.carla.ground_truth import GroundTruthFrame
from adaptx.models.point_cloud import RawPointCloudFrame
from adaptx.models.system import SimulationSessionStatus


class ScenarioSimulator(Protocol):
    """A deterministic, steppable simulation with ego-relative placement."""

    def open(self) -> None:
        """Connect, configure determinism, spawn the ego and attach the sensor."""

    def close(self) -> None:
        """Destroy every spawned actor and restore the world. Idempotent."""

    def step(self) -> RawPointCloudFrame:
        """Advance one fixed timestep and return the sensor frame for it."""

    def ground_truth(self) -> GroundTruthFrame:
        """What the simulator knew at the most recent step."""

    def status(self) -> SimulationSessionStatus:
        """Bounded description of the session."""

    def spawn_ahead_of_ego(
        self, blueprint_id: str, *, forward_m: float, left_m: float = 0.0, up_m: float = 0.5
    ) -> Any:
        """Spawn an actor at an ego-relative ADAPT-X offset; returns a handle."""

    def place_ahead_of_ego(
        self, actor: Any, *, forward_m: float, left_m: float = 0.0, up_m: float = 0.5
    ) -> None:
        """Move a spawned actor to an ego-relative ADAPT-X offset."""
