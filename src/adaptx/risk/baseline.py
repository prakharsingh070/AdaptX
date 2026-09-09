"""Deterministic baseline risk calculator.

**This is a baseline, not the ADAPT-X risk engine.** It exists so the risk
contract can be exercised by tests and so future work has a documented
reference point. It models proximity only and deliberately ignores relative
velocity, time-to-collision, trajectory overlap, object class importance and
predicted conflicts. Any published comparison must state that these inputs are
absent.

Formulation
-----------
For a track at planar distance ``d`` from the ego vehicle::

    risk = clamp(1 - d / max_range_m, 0, 1)

Risk is normalised to ``[0, 1]``. Uncertainty is passed through from the track
and does not influence the score; the two quantities stay independent.
"""

from __future__ import annotations

import math

from adaptx.config.settings import RiskSettings
from adaptx.models.common import DataSource, Vector3
from adaptx.models.prediction import PredictedTrajectory
from adaptx.models.risk import ObjectRisk, RiskFactors, RiskField, RiskLevel
from adaptx.models.tracking import TrackedObject
from adaptx.models.vehicle import VehicleState
from adaptx.risk.interfaces import RiskEngine


class BaselineProximityRiskEngine(RiskEngine):
    """Proximity-only risk baseline. Not the ADAPT-X risk engine."""

    name = "baseline_proximity"
    is_baseline = True

    def __init__(self, settings: RiskSettings) -> None:
        self._settings = settings

    def evaluate(
        self,
        tracks: list[TrackedObject],
        *,
        ego_state: VehicleState | None = None,
        trajectories: list[PredictedTrajectory] | None = None,
    ) -> RiskField:
        """Score each track by planar distance from the ego vehicle.

        ``trajectories`` is accepted for contract compatibility and ignored:
        this baseline does not use predicted motion.
        """
        origin = ego_state.position if ego_state is not None else Vector3()
        object_risks = [self._score(track, origin) for track in tracks]
        source = ego_state.source if ego_state is not None else DataSource.UNAVAILABLE
        return RiskField(
            engine=self.name,
            is_baseline=True,
            cells=[],  # This baseline produces no spatial field.
            object_risks=object_risks,
            source=source,
        )

    def classify(self, risk_score: float) -> RiskLevel:
        """Partition ``[0, 1]`` using the configured thresholds."""
        if not 0.0 <= risk_score <= 1.0:
            raise ValueError(f"risk_score must be in [0, 1], got {risk_score}")
        if risk_score >= self._settings.threshold_critical:
            return RiskLevel.CRITICAL
        if risk_score >= self._settings.threshold_high:
            return RiskLevel.HIGH
        if risk_score >= self._settings.threshold_medium:
            return RiskLevel.MEDIUM
        return RiskLevel.LOW

    def _score(self, track: TrackedObject, origin: Vector3) -> ObjectRisk:
        distance = math.sqrt(
            (track.position.x - origin.x) ** 2 + (track.position.y - origin.y) ** 2
        )
        proximity = max(0.0, 1.0 - distance / self._settings.max_range_m)
        return ObjectRisk(
            timestamp=track.timestamp,
            track_id=track.track_id,
            risk_score=proximity,
            risk_level=self.classify(proximity),
            uncertainty=track.uncertainty,
            factors=RiskFactors(proximity=proximity),
            source=track.source,
        )
