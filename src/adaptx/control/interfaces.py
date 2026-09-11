"""What the control layer needs from a vehicle (live extension).

The policy decides; a :class:`VehicleController` applies. The protocol is
the whole coupling between the decision and the simulator, so the policy
can be exercised without a vehicle and a vehicle can be driven by a test
without a policy. The only implementation today wraps the CARLA session;
the CARLA call itself lives in :mod:`adaptx.carla.session`, never here.
"""

from __future__ import annotations

from typing import Protocol

from adaptx.control.models import ControlCommand, EgoObservation


class VehicleController(Protocol):
    """Applies a command to a vehicle and reports the vehicle's own state."""

    def apply(self, command: ControlCommand) -> None:
        """Actuate the vehicle for the next timestep."""

    def observe(self, lookahead_m: float) -> EgoObservation:
        """The ego's odometry and lane geometry for the coming decision."""

    def release(self) -> None:
        """Stop actuating: neutral throttle, brake applied. Idempotent."""


__all__ = ["VehicleController"]
