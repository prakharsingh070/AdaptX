"""Risk engine status endpoint."""

from __future__ import annotations

from fastapi import APIRouter, status

from adaptx.api.dependencies import ContextDep, SettingsDep, SystemServiceDep
from adaptx.api.schemas import RiskStatusResponse
from adaptx.models.risk import SCORED_RISK_LEVELS, RiskLevel
from adaptx.models.system import ComponentStatus
from adaptx.risk.heuristic import SCORING_MODEL

router = APIRouter(prefix="/risk", tags=["risk"])


def _component(service: SystemServiceDep) -> ComponentStatus:
    for component in service.components():
        if component.name == "risk":
            return component
    raise RuntimeError("risk component is missing from the system component table")


@router.get(
    "/status",
    response_model=RiskStatusResponse,
    status_code=status.HTTP_200_OK,
    summary="Risk engine status",
)
def risk_status(
    service: SystemServiceDep, settings: SettingsDep, context: ContextDep
) -> RiskStatusResponse:
    """Report the configured risk engine and what its numbers mean.

    The configured engine is a **deterministic heuristic baseline**, flagged by
    ``is_baseline``. It scores proximity, rate of approach and predicted
    approach, combined as a weighted mean over the factors actually available.

    ``score_is_heuristic`` is always true and ``is_collision_probability`` is
    always false: **no collision-probability model exists in this project**, and
    the score has never been calibrated or validated against labelled risk data,
    because none exists. Thresholds are baseline engineering values, not
    safety-certified limits, and no accuracy figure is reported.

    ``uncertainty_is_heuristic`` is likewise always true. Uncertainty is
    reported *separately* from risk: an object can be low-risk and poorly
    observed.

    ``decides_resolution`` is always false. The risk engine never chooses
    spatial resolution; that is a separate decision belonging to a resolution
    controller, which is not implemented.

    Thresholds cover the four **scored** levels only. ``UNKNOWN`` appears in
    ``risk_levels`` but has no threshold, because it means nothing was scored.
    """
    risk = context.risk
    engine = risk.engine
    last = risk.last_result

    return RiskStatusResponse(
        component=_component(service),
        configuration={
            **risk.configuration.model_dump(mode="json"),
            "scale": "normalised [0, 1]; dashboards may render 0-100",
            "modelled_factors": ["proximity", "closing_speed", "predicted_proximity"],
            "unmodelled_factors": [
                "time_to_collision",
                "trajectory_map_intersection",
                "object_interaction",
                "road_and_lane_geometry",
                "ego_planned_path",
                "calibrated_collision_probability",
            ],
            "uncertainty_sources": [
                "unknown_velocity",
                "stale_observation",
                "low_track_confidence",
                "no_prediction",
                "wide_prediction_uncertainty",
                "tentative_track",
                "coasting_track",
                "unobserved_map_context",
            ],
            "thresholds_are": "baseline engineering values, not safety-certified limits",
            "unobserved_map_cells": "treated as unobserved, never as free space",
        },
        engine=engine.name,
        is_baseline=engine.is_baseline,
        risk_levels=list(RiskLevel),
        thresholds={
            RiskLevel.LOW.value: 0.0,
            RiskLevel.MEDIUM.value: settings.risk.threshold_medium,
            RiskLevel.HIGH.value: settings.risk.threshold_high,
            RiskLevel.CRITICAL.value: settings.risk.threshold_critical,
        },
        scoring_model=SCORING_MODEL,
        score_is_heuristic=True,
        is_collision_probability=False,
        uncertainty_is_heuristic=True,
        decides_resolution=False,
        baseline_engine=context.risk_engine.name,
        frames_assessed=risk.frames_assessed,
        last_assessment_timestamp=(None if last is None else last.timestamp),
        summary=risk.summary(),
    )


#: Scored levels, re-exported for callers that need the ordered bands without
#: the UNKNOWN sentinel.
__all__ = ["SCORED_RISK_LEVELS", "risk_status", "router"]
