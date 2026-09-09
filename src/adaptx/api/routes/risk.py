"""Risk engine status endpoint."""

from __future__ import annotations

from fastapi import APIRouter, status

from adaptx.api.dependencies import ContextDep, SettingsDep, SystemServiceDep
from adaptx.api.schemas import RiskStatusResponse
from adaptx.models.risk import RiskLevel
from adaptx.models.system import ComponentStatus

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
    """Report the configured risk engine and its normalisation bounds.

    The configured engine in Phase 1 is a proximity-only baseline, flagged by
    ``is_baseline``. It is not the ADAPT-X risk engine and models neither
    relative velocity, time-to-collision, trajectory overlap nor predicted
    conflicts.

    Thresholds are lower bounds on the normalised ``[0, 1]`` risk scale.
    """
    engine = context.risk_engine
    return RiskStatusResponse(
        component=_component(service),
        configuration={
            "max_range_m": settings.risk.max_range_m,
            "scale": "normalised [0, 1]; dashboards may render 0-100",
            "modelled_factors": ["proximity"],
            "unmodelled_factors": [
                "relative_velocity",
                "time_to_collision",
                "trajectory_overlap",
                "object_importance",
                "uncertainty",
            ],
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
    )
