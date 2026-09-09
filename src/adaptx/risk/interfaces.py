"""Risk-layer contract.

See ``docs/knowledge-base/06_risk-engine.md``. The conceptual model is

``Risk(x, y) = f(proximity, relative motion, TTC, trajectory overlap,
uncertainty, object importance)``

which is a research formulation, not a mandated equation. Any implementation
must document its assumptions, normalisation, bounds, thresholds and
validation results.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from adaptx.models.prediction import PredictedTrajectory
from adaptx.models.risk import RiskField, RiskLevel
from adaptx.models.tracking import TrackedObject
from adaptx.models.vehicle import VehicleState


class RiskEngine(ABC):
    """Estimates the relative importance of spatial regions and objects."""

    #: Stable identifier recorded on every produced RiskField.
    name: str = "risk_engine"
    #: True for comparison baselines, false for the ADAPT-X formulation.
    is_baseline: bool = False

    @abstractmethod
    def evaluate(
        self,
        tracks: list[TrackedObject],
        *,
        ego_state: VehicleState | None = None,
        trajectories: list[PredictedTrajectory] | None = None,
    ) -> RiskField:
        """Return the risk field implied by the current world state."""

    @abstractmethod
    def classify(self, risk_score: float) -> RiskLevel:
        """Map a normalised risk score in ``[0, 1]`` to a discrete level."""
