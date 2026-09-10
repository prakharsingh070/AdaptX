"""Prediction-layer contract.

See ``docs/knowledge-base/09_prediction.md``. Phase 5 implements this contract
as a deterministic constant-velocity baseline
(:class:`~adaptx.prediction.constant_velocity.ConstantVelocityPredictor`).
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from datetime import datetime

from adaptx.models.prediction_result import PredictionResult
from adaptx.models.tracking import TrackedObject


class TrajectoryPredictor(ABC):
    """Estimates future trajectories for tracked objects.

    An implementation must document its horizon, timestep, model assumptions,
    confidence semantics and how uncertainty grows with horizon. Predicted
    positions must never be reported as measurements.

    Like the detector (ADR-022) and tracker (ADR-024) contracts, ``predict``
    returns a full :class:`~adaptx.models.prediction_result.PredictionResult`
    rather than a bare list. Only the predictor knows which tracks it declined
    and why, and a caller given only the surviving trajectories cannot tell a
    scene with no tracks from one where every track lacked a measured velocity.
    """

    name: str = "trajectory_predictor"
    #: True for deterministic geometric baselines, false for a learned model.
    is_baseline: bool = True

    @abstractmethod
    def predict(
        self,
        tracks: list[TrackedObject],
        timestamp: datetime,
        *,
        horizon_s: float | None = None,
        timestep_s: float | None = None,
        frame_id: int = 0,
        sensor_id: str = "unknown",
    ) -> PredictionResult:
        """Predict future trajectories for ``tracks`` as of ``timestamp``.

        Args:
            tracks: Live tracks from a tracking update, in any order.
            timestamp: The source time predictions are made from. Every
                trajectory point is offset from it, so future times are
                arithmetic and never taken from a wall clock.
            horizon_s: Overrides the configured horizon for this call.
            timestep_s: Overrides the configured interval for this call.
            frame_id: Frame identifier recorded on the result.
            sensor_id: Sensor identifier recorded on the result.

        Returns:
            A result carrying the trajectories produced, the tracks skipped
            with a reason each, measured timings and a configuration snapshot.
        """
