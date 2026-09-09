"""CARLA status endpoint."""

from __future__ import annotations

from fastapi import APIRouter, status

from adaptx.api.dependencies import CarlaServiceDep
from adaptx.models.system import CarlaConnectionStatus

router = APIRouter(prefix="/carla", tags=["carla"])


@router.get(
    "/status",
    response_model=CarlaConnectionStatus,
    status_code=status.HTTP_200_OK,
    summary="CARLA connection status",
)
def carla_status(service: CarlaServiceDep) -> CarlaConnectionStatus:
    """Report the CARLA integration state.

    CARLA is optional. A disabled, uninstalled or unreachable simulator is
    reported as ``DISCONNECTED`` with an explanation in ``detail``; it is not
    an error. ``is_mock`` is true when the in-process fake simulator is active,
    in which case no data from it is sensor data.
    """
    return service.status()
