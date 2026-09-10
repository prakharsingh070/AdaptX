"""2.5D map status endpoint."""

from __future__ import annotations

from fastapi import APIRouter, status

from adaptx.api.dependencies import ContextDep, SettingsDep, SystemServiceDep
from adaptx.api.schemas import MapStatusResponse
from adaptx.models.map import ResolutionLevel
from adaptx.models.system import ComponentStatus

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
    summary="2.5D map status",
)
def map_status(
    service: SystemServiceDep, settings: SettingsDep, context: ContextDep
) -> MapStatusResponse:
    """Report mapping readiness, the applied resolution and the mapped extent.

    The configured mapper is a **fixed-resolution baseline**, flagged by
    ``is_adaptive: false``. One cell size applies everywhere.
    ``adaptive_resolution_implemented`` is false: no resolution controller
    exists, so no region receives more detail than another.

    ``lifecycle`` is ``frame_local`` - each map covers exactly one frame and
    nothing accumulates. This is not a persistent world map.

    The ``resolution_levels`` returned here remain configuration: they define
    what each level *would* mean to a controller, not a decision that has been
    made.
    """
    mapping = context.mapping
    summary = mapping.summary()
    last = mapping.last_summary
    decision = mapping.default_resolution()
    bounds = mapping.mapper.bounds
    height, width = mapping.mapper.grid_shape(settings.map.resolution_m)

    return MapStatusResponse(
        component=_component(service),
        configuration={
            # Retained from the Phase 1 response shape: still accurate, and
            # removing a field a consumer may read would be a breaking change.
            "is_adaptive_algorithm_implemented": False,
            "fixed_resolution_baseline_available": True,
            "adaptive_resolution_implemented": False,
            "occupancy": "binary: a cell is occupied iff it holds at least one point",
            "unobserved_height": "null, never zero",
            "modelled": ["occupancy", "point_count", "min/max/mean height"],
            "not_modelled": [
                "probabilistic_occupancy",
                "temporal_fusion",
                "adaptive_resolution",
                "risk",
                "uncertainty",
                "semantic_labels",
            ],
        },
        resolution_levels={
            ResolutionLevel.LOW: settings.map.resolution_low_m,
            ResolutionLevel.MEDIUM: settings.map.resolution_medium_m,
            ResolutionLevel.HIGH: settings.map.resolution_high_m,
            ResolutionLevel.CRITICAL: settings.map.resolution_critical_m,
        },
        range_m=settings.map.range_m,
        active_cells=(0 if last is None else last.accounting.occupied_cell_count),
        mapper=mapping.mapper.name,
        is_adaptive=mapping.mapper.is_adaptive,
        adaptive_resolution_implemented=False,
        lifecycle="frame_local",
        resolution_m=decision.resolution_m,
        resolution_source=decision.source,
        bounds=bounds,
        width=width,
        height=height,
        total_cells=width * height,
        last_map_timestamp=(None if last is None else last.timestamp),
        summary=summary,
    )
