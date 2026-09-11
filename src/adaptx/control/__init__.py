"""Baseline ego vehicle control for the live simulation (post-Phase-12 extension).

Reads the perception stack's outputs and the ego's own odometry; never
ground truth about other actors. A controlled-simulation baseline, not an
autonomous-driving controller (ADR-056).
"""

from adaptx.control.interfaces import VehicleController
from adaptx.control.models import (
    ControlCommand,
    ControlConfiguration,
    ControllerState,
    EgoObservation,
)
from adaptx.control.policy import POLICY_NAME, RiskGovernedSpeedPolicy

__all__ = [
    "POLICY_NAME",
    "ControlCommand",
    "ControlConfiguration",
    "ControllerState",
    "EgoObservation",
    "RiskGovernedSpeedPolicy",
    "VehicleController",
]
