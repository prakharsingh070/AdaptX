"""2.5D map status endpoint."""

from __future__ import annotations

from fastapi import APIRouter, status

from adaptx.api.dependencies import ContextDep, SettingsDep, SystemServiceDep
from adaptx.api.schemas import AdaptiveResolutionResetResponse, MapStatusResponse
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

    Two mappers are reported, and both remain available so a fixed-resolution
    measurement is never presented as an adaptive one (ADR-003).

    ``mapper`` is the **fixed-resolution baseline** used by
    ``POST /api/v1/lidar/map``, flagged ``is_adaptive: false``. One cell size
    applies everywhere and it never chooses that size itself.

    ``adaptive_mapper`` and ``adaptive_controller`` are the Phase 8 pair used by
    ``POST /api/v1/lidar/adaptive-map``. ``adaptive_resolution_implemented`` is
    now true: a controller exists, and regions genuinely receive different cell
    sizes. It remains a **deterministic heuristic baseline**, not a learned
    policy and not validated against labelled data.

    ``lifecycle`` is ``frame_local`` - each map covers exactly one frame and no
    occupancy accumulates. This is not a persistent world map. The adaptive
    path additionally remembers each region resolution *level* between frames,
    which is what stops resolution oscillating; that is policy state, not
    measurement, and it is cleared by ``POST /api/v1/map/adaptive/reset``.

    The ``resolution_levels`` returned here are the vocabulary both halves
    share: what each level means in metres.
    """
    mapping = context.mapping
    adaptive = context.adaptive_mapping
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
            "is_adaptive_algorithm_implemented": True,
            "fixed_resolution_baseline_available": True,
            "adaptive_resolution_implemented": True,
            "adaptive_resolution_is_heuristic": True,
            "adaptive_priority_is_collision_probability": False,
            "adaptive_representation": (
                "region-partitioned tiles, each with its own uniform sub-grid"
            ),
            "adaptive_stabilisation": (
                "asymmetric hysteresis margin plus a minimum dwell time before coarsening"
            ),
            "occupancy": "binary: a cell is occupied iff it holds at least one point",
            "unobserved_height": "null, never zero",
            "modelled": ["occupancy", "point_count", "min/max/mean height"],
            "not_modelled": [
                "probabilistic_occupancy",
                "temporal_fusion",
                "occlusion",
                "per_cell_risk",
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
        adaptive_resolution_implemented=True,
        lifecycle="frame_local",
        resolution_m=decision.resolution_m,
        resolution_source=decision.source,
        bounds=bounds,
        width=width,
        height=height,
        total_cells=width * height,
        last_map_timestamp=(None if last is None else last.timestamp),
        summary=summary,
        adaptive_mapper=adaptive.mapper.name,
        adaptive_controller=adaptive.controller.name,
        adaptive_enabled=adaptive.enabled,
        adaptive_is_baseline=adaptive.controller.is_baseline,
        tile_size_m=adaptive.configuration.tile_size_m,
        tile_count=adaptive.controller.tile_grid().tile_count,
        adaptive_summary=adaptive.summary(),
    )


@router.post(
    "/adaptive/reset",
    response_model=AdaptiveResolutionResetResponse,
    status_code=status.HTTP_200_OK,
    summary="Clear adaptive resolution stabilisation state",
)
def reset_adaptive_resolution(context: ContextDep) -> AdaptiveResolutionResetResponse:
    """Make every region forget the resolution level it was holding.

    The adaptive path is the only mapping state that survives a frame. Regions
    remember their level and how long a coarser one has been proposed, so that
    resolution does not oscillate; that memory must not carry across unrelated
    sequences, or a new scene starts refined where the old one happened to be
    busy.

    This clears **policy** state only. No occupancy is retained anywhere, so
    there is nothing else to drop: mapping is frame-local either way.
    """
    controller = context.adaptive_mapping.controller
    grid = controller.tile_grid()
    cleared = sum(
        1 for index in range(grid.tile_count) if controller.current_level(index) is not None
    )
    context.adaptive_mapping.reset()
    return AdaptiveResolutionResetResponse(cleared_region_count=cleared)
