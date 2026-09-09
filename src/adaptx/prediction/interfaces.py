"""Prediction-layer contract.

See ``docs/knowledge-base/09_prediction.md``. No implementation exists in
Phase 1.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from adaptx.models.prediction import PredictedTrajectory
from adaptx.models.tracking import TrackedObject


class TrajectoryPredictor(ABC):
    """Estimates future trajectories for tracked objects.

    An implementation must document its horizon, timestep, model assumptions,
    confidence semantics and how uncertainty grows with horizon. Predicted
    positions must never be reported as measurements.
    """

    name: str = "trajectory_predictor"

    @abstractmethod
    def predict(
        self, tracks: list[TrackedObject], *, horizon_s: float, timestep_s: float
    ) -> list[PredictedTrajectory]:
        """Return one trajectory per track that has enough history to predict."""
