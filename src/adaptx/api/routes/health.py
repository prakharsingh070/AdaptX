"""Liveness endpoint."""

from __future__ import annotations

from fastapi import APIRouter, status

from adaptx import __version__
from adaptx.api.dependencies import SettingsDep
from adaptx.api.schemas import HealthResponse

router = APIRouter(tags=["health"])


@router.get(
    "/health",
    response_model=HealthResponse,
    status_code=status.HTTP_200_OK,
    summary="Liveness probe",
)
def health(settings: SettingsDep) -> HealthResponse:
    """Report that the process is alive and serving requests.

    This is a liveness check only. It says nothing about subsystem readiness;
    use ``/api/v1/system/status`` for that.
    """
    return HealthResponse(status="ok", version=__version__, name=settings.app.name)
