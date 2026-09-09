"""System status and metrics endpoints."""

from __future__ import annotations

from fastapi import APIRouter, status

from adaptx.api.dependencies import MetricsServiceDep, SystemServiceDep
from adaptx.models.system import SystemMetrics, SystemStatus

router = APIRouter(prefix="/system", tags=["system"])


@router.get(
    "/status",
    response_model=SystemStatus,
    status_code=status.HTTP_200_OK,
    summary="Aggregated backend status",
)
def system_status(service: SystemServiceDep) -> SystemStatus:
    """Report backend state, the LiDAR and CARLA channels, and every subsystem.

    Each component carries both a readiness (READY / NOT_READY) and an
    implementation status (IMPLEMENTED / PARTIAL / PLANNED / MOCK), so a
    subsystem that does not exist yet is visibly reported as planned.
    """
    return service.status()


@router.get(
    "/metrics",
    response_model=SystemMetrics,
    status_code=status.HTTP_200_OK,
    summary="Measured runtime metrics",
)
def system_metrics(service: MetricsServiceDep) -> SystemMetrics:
    """Return metrics measured by this process.

    A metric that could not be measured is ``null`` and is named in
    ``unavailable``. No value is ever estimated or defaulted.
    """
    return service.snapshot()
