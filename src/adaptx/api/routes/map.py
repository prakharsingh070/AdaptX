"""Adaptive 2.5D map status endpoint."""

from __future__ import annotations

from fastapi import APIRouter, status

from adaptx.api.dependencies import SettingsDep, SystemServiceDep
from adaptx.api.schemas import MapStatusResponse
from adaptx.models.map import ResolutionLevel
from adaptx.models.system import ComponentStatus, ImplementationStatus

router = APIRouter(prefix="/map", tags=["map"])


def _component(service: SystemServiceDep) -> ComponentStatus:
    for component in service.components():
        if component.name == "mapping":
            return component
    # Unreachable while the component table declares "mapping"; kept explicit
    # rather than returning an invented status.
    raise RuntimeError("mapping component is missing from the system component table")


@router.get(
    "/status",
    response_model=MapStatusResponse,
    status_code=status.HTTP_200_OK,
    summary="Adaptive 2.5D map status",
)
def map_status(service: SystemServiceDep, settings: SettingsDep) -> MapStatusResponse:
    """Report mapping readiness and the configured resolution vocabulary.

    No mapper exists in Phase 1, so ``active_cells`` is 0 and the component is
    reported as PLANNED. The resolution levels returned here are configuration:
    they define what each level would mean, not a decision that has been made.
    """
    component = _component(service)
    return MapStatusResponse(
        component=component,
        configuration={
            "is_adaptive_algorithm_implemented": component.implementation
            is ImplementationStatus.IMPLEMENTED,
            "fixed_resolution_baseline_available": False,
        },
        resolution_levels={
            ResolutionLevel.LOW: settings.map.resolution_low_m,
            ResolutionLevel.MEDIUM: settings.map.resolution_medium_m,
            ResolutionLevel.HIGH: settings.map.resolution_high_m,
            ResolutionLevel.CRITICAL: settings.map.resolution_critical_m,
        },
        range_m=settings.map.range_m,
        active_cells=0,
    )
