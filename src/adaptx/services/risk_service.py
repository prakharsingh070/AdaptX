"""Risk and uncertainty service (Phase 7).

Like :mod:`adaptx.services.prediction_service` and
:mod:`adaptx.services.mapping_service`, and unlike
:mod:`adaptx.services.tracking_service`, this service is **stateless with
respect to perception**. An assessment is a pure function of one tracking
result, its trajectories and the current map, so nothing is carried between
frames.

The only state held is observational - a counter and the last result's
aggregate - used for status and telemetry. Neither influences an assessment.

The service orchestrates; it contains no scoring logic::

    TrackingResult + PredictionResult + SpatialMap
        -> HeuristicRiskEngine -> RiskAssessmentResult

It does **not** decide spatial resolution. That is Phase 8 (ADR-036).
"""

from __future__ import annotations

from datetime import datetime
from threading import Lock

from adaptx.config.settings import RiskSettings
from adaptx.core.logging import get_logger
from adaptx.models.prediction import PredictedTrajectory
from adaptx.models.prediction_result import PredictionResult
from adaptx.models.risk import RiskLevel
from adaptx.models.risk_assessment import RiskAssessmentResult, RiskConfiguration
from adaptx.models.spatial_map import SpatialMap
from adaptx.models.tracking import TrackedObject
from adaptx.models.tracking_result import TrackingResult
from adaptx.models.vehicle import VehicleState
from adaptx.risk.heuristic import SCORING_MODEL, HeuristicRiskEngine

logger = get_logger(__name__)

#: Per-object entries included in a telemetry summary, most concerning first.
TELEMETRY_TOP_N = 5


class RiskService:
    """Runs the configured risk engine over a tracking result."""

    def __init__(self, settings: RiskSettings, engine: HeuristicRiskEngine | None = None) -> None:
        self._settings = settings
        self._engine = engine if engine is not None else HeuristicRiskEngine(settings)
        self._lock = Lock()
        self._frames_assessed = 0
        self._last_result: RiskAssessmentResult | None = None

    @property
    def engine(self) -> HeuristicRiskEngine:
        """The engine under management."""
        return self._engine

    @property
    def configuration(self) -> RiskConfiguration:
        """Effective risk configuration."""
        return self._engine.configuration

    @property
    def frames_assessed(self) -> int:
        """Frames assessed since the last reset."""
        with self._lock:
            return self._frames_assessed

    @property
    def last_result(self) -> RiskAssessmentResult | None:
        """The most recent assessment, or ``None`` before the first frame."""
        with self._lock:
            return self._last_result

    def assess_from_pipeline(
        self,
        tracking: TrackingResult,
        *,
        prediction: PredictionResult | None = None,
        spatial_map: SpatialMap | None = None,
        ego_state: VehicleState | None = None,
    ) -> RiskAssessmentResult:
        """Assess the tracks in ``tracking``, with whatever context is available.

        The tracking result supplies the timestamp, frame id and sensor id, so
        an assessment is anchored to the frame that produced the tracks rather
        than to a wall clock. Prediction and map are optional: absent context
        raises uncertainty rather than blocking the assessment.
        """
        trajectories: list[PredictedTrajectory] | None = (
            None if prediction is None else prediction.trajectories
        )
        return self.assess(
            tracking.tracks,
            trajectories=trajectories,
            spatial_map=spatial_map,
            ego_state=ego_state,
            timestamp=tracking.timestamp,
            frame_id=tracking.frame_id,
            sensor_id=tracking.sensor_id,
        )

    def assess(
        self,
        tracks: list[TrackedObject],
        *,
        trajectories: list[PredictedTrajectory] | None = None,
        spatial_map: SpatialMap | None = None,
        ego_state: VehicleState | None = None,
        timestamp: datetime | None = None,
        frame_id: int = 0,
        sensor_id: str = "unknown",
    ) -> RiskAssessmentResult:
        """Assess ``tracks``.

        The engine call is outside the lock: it holds no mutable state, so two
        concurrent frames cannot interfere. Only the bookkeeping is guarded.
        """
        result = self._engine.assess_many(
            tracks,
            trajectories=trajectories,
            spatial_map=spatial_map,
            ego_state=ego_state,
            timestamp=timestamp,
            frame_id=frame_id,
            sensor_id=sensor_id,
        )
        with self._lock:
            self._frames_assessed += 1
            self._last_result = result
        return result

    def reset(self) -> None:
        """Clear the observational counters.

        Affects reporting only. There is no accumulated risk state to drop,
        because assessment carries none between frames.
        """
        with self._lock:
            self._frames_assessed = 0
            self._last_result = None
        logger.info("risk counters reset")

    def summary(self) -> dict[str, object]:
        """Compact state summary for status and telemetry.

        Counts, the scene aggregate and a handful of the most concerning
        objects. **Never** full trajectories, risk cells or map data - the same
        rule detection, tracking, prediction and mapping already follow.
        """
        with self._lock:
            result = self._last_result
            frames = self._frames_assessed

        state: dict[str, object] = {
            "risk_engine": self._engine.name,
            "is_baseline": self._engine.is_baseline,
            "scoring_model": SCORING_MODEL,
            "score_is_heuristic": True,
            "is_collision_probability": False,
            "frames_assessed": frames,
            "configuration": self.configuration.model_dump(mode="json"),
        }
        if result is None:
            state["note"] = "no frame has been assessed since the last reset"
            return state

        counts = result.counts_by_level()
        state.update(
            {
                "total_objects": result.considered_track_count,
                "low_count": counts.get(RiskLevel.LOW.value, 0),
                "medium_count": counts.get(RiskLevel.MEDIUM.value, 0),
                "high_count": counts.get(RiskLevel.HIGH.value, 0),
                "critical_count": counts.get(RiskLevel.CRITICAL.value, 0),
                "unknown_count": result.unknown_count,
                "highest_risk_level": result.highest_risk_level.value,
                "highest_risk_score": result.highest_risk_score,
                "max_uncertainty": result.max_uncertainty,
                "timestamp": result.timestamp.isoformat(),
                "processing_time_ms": result.duration_ms,
                "objects": _top_objects(result),
            }
        )
        return state


def _top_objects(result: RiskAssessmentResult) -> list[dict[str, object]]:
    """The most concerning objects, compactly, for telemetry.

    Ordered by score descending with ``track_id`` breaking ties, so the same
    result always yields the same list. Unscored tracks sort last: they are
    reported in ``unknown_count`` and are not ranked against real scores.
    """
    ranked = sorted(
        result.assessments,
        key=lambda a: (-(a.risk_score if a.risk_score is not None else -1.0), a.track_id),
    )
    return [
        {
            "track_id": a.track_id,
            "risk_level": a.risk_level.value,
            "risk_score": a.risk_score,
            "distance_m": round(a.distance_m, 2),
            "uncertainty": a.uncertainty.score,
        }
        for a in ranked[:TELEMETRY_TOP_N]
    ]
