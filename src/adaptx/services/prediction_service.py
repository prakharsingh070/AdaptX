"""Trajectory prediction service (Phase 5).

Unlike :mod:`adaptx.services.tracking_service`, this service is **stateless**
with respect to perception: a prediction is a pure function of one tracking
result, so nothing needs to be carried between frames. Tracking state is not
duplicated here - the service reads a :class:`TrackingResult` it is handed and
keeps no tracks of its own.

The only state it holds is observational: a counter and the last result, used
for status and telemetry summaries. Both are guarded by a lock so a concurrent
request cannot read a half-updated pair, and neither influences a prediction.
That distinction matters: resetting this service changes what the status
endpoint reports and nothing about what the predictor produces.
"""

from __future__ import annotations

from datetime import datetime
from threading import Lock

from adaptx.config.settings import PredictionSettings
from adaptx.core.logging import get_logger
from adaptx.models.prediction_result import PredictionConfiguration, PredictionResult
from adaptx.models.tracking import TrackedObject
from adaptx.models.tracking_result import TrackingResult
from adaptx.prediction.constant_velocity import (
    UNCERTAINTY_MODEL,
    ConstantVelocityPredictor,
)

logger = get_logger(__name__)


class PredictionService:
    """Runs the configured predictor over a tracking result."""

    def __init__(
        self,
        settings: PredictionSettings,
        predictor: ConstantVelocityPredictor | None = None,
    ) -> None:
        self._settings = settings
        self._predictor = (
            predictor if predictor is not None else ConstantVelocityPredictor(settings)
        )
        self._lock = Lock()
        self._frames_predicted = 0
        self._last_result: PredictionResult | None = None

    @property
    def predictor(self) -> ConstantVelocityPredictor:
        """The predictor under management."""
        return self._predictor

    @property
    def configuration(self) -> PredictionConfiguration:
        """Effective prediction configuration."""
        return self._predictor.configuration

    @property
    def frames_predicted(self) -> int:
        """Frames predicted since the last reset."""
        with self._lock:
            return self._frames_predicted

    @property
    def last_result(self) -> PredictionResult | None:
        """The most recent prediction, or ``None`` before the first frame."""
        with self._lock:
            return self._last_result

    def predict_from_tracking(self, tracking: TrackingResult) -> PredictionResult:
        """Predict trajectories for the tracks in ``tracking``.

        The tracking result supplies the source timestamp, frame id and sensor
        id, so a prediction is anchored to the frame that produced the tracks
        rather than to a wall clock.
        """
        return self.predict(
            tracking.tracks,
            tracking.timestamp,
            frame_id=tracking.frame_id,
            sensor_id=tracking.sensor_id,
        )

    def predict(
        self,
        tracks: list[TrackedObject],
        timestamp: datetime,
        *,
        frame_id: int = 0,
        sensor_id: str = "unknown",
    ) -> PredictionResult:
        """Predict trajectories for ``tracks`` as of ``timestamp``.

        The predictor call itself is outside the lock: it holds no mutable
        state, so two concurrent predictions cannot interfere. Only the
        bookkeeping is guarded.
        """
        result = self._predictor.predict(tracks, timestamp, frame_id=frame_id, sensor_id=sensor_id)
        with self._lock:
            self._frames_predicted += 1
            self._last_result = result
        return result

    def reset(self) -> None:
        """Clear the observational counters.

        Affects reporting only. There is no perception state to drop, because
        prediction carries none between frames.
        """
        with self._lock:
            self._frames_predicted = 0
            self._last_result = None
        logger.info("prediction counters reset")

    def summary(self) -> dict[str, object]:
        """Compact state summary for status and telemetry.

        Counts, identifiers and configuration only. Full trajectories are
        returned by the prediction endpoint; putting them on every telemetry
        tick would push frame geometry down a status channel - the same rule
        detection and tracking already follow.
        """
        with self._lock:
            result = self._last_result
            frames = self._frames_predicted

        summary: dict[str, object] = {
            "predictor": self._predictor.name,
            "model": "constant_velocity",
            "is_baseline": self._predictor.is_baseline,
            "uncertainty_model": UNCERTAINTY_MODEL,
            "uncertainty_is_heuristic": True,
            "frames_predicted": frames,
            "horizon_s": self._settings.horizon_s,
            "interval_s": self._settings.interval_s,
            "configuration": self.configuration.model_dump(mode="json"),
        }
        if result is None:
            summary["note"] = "no frame has been predicted since the last reset"
            return summary

        summary.update(
            {
                "considered_tracks": result.considered_track_count,
                "predicted_tracks": result.predicted_track_count,
                "skipped_tracks": result.skipped_track_count,
                "predicted_points": result.predicted_point_count,
                "predicted_track_ids": result.predicted_track_ids,
                "counts_by_status": result.counts_by_status(),
                "duration_ms": result.duration_ms,
            }
        )
        return summary
