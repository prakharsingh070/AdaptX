"""Trajectory prediction status endpoint (Phase 5).

Frame-level prediction lives on the LiDAR router as
``POST /api/v1/lidar/predict``, because it consumes a point cloud exactly as
the other LiDAR endpoints do. This module carries the module-status view, in
the same shape as the map and risk status endpoints.
"""

from __future__ import annotations

from fastapi import APIRouter, status

from adaptx.api.dependencies import ContextDep, SettingsDep, SystemServiceDep
from adaptx.api.schemas import PredictionStatusResponse
from adaptx.models.prediction import PredictionStatus
from adaptx.models.system import ComponentStatus
from adaptx.prediction.constant_velocity import MODEL_NAME, UNCERTAINTY_MODEL

router = APIRouter(prefix="/prediction", tags=["prediction"])


def _component(service: SystemServiceDep) -> ComponentStatus:
    for component in service.components():
        if component.name == "prediction":
            return component
    raise RuntimeError("prediction component is missing from the system component table")


@router.get(
    "/status",
    response_model=PredictionStatusResponse,
    status_code=status.HTTP_200_OK,
    summary="Trajectory prediction status",
)
def prediction_status(
    service: SystemServiceDep, settings: SettingsDep, context: ContextDep
) -> PredictionStatusResponse:
    """Report the configured predictor, its horizon and what its numbers mean.

    The configured predictor is a **deterministic constant-velocity baseline**,
    flagged by ``is_baseline``. It models no acceleration, no turning, no road
    or lane geometry and no interaction between objects.

    ``uncertainty_is_heuristic`` is always true: the reported
    ``position_uncertainty_m`` is a documented formula that grows with
    extrapolation time, not a calibrated sigma, probability or confidence
    interval. No labelled trajectories exist to calibrate one against, so
    prediction accuracy is unmeasured and no accuracy figure is reported here.
    """
    predictor = context.prediction.predictor
    configuration = predictor.configuration
    return PredictionStatusResponse(
        component=_component(service),
        configuration={
            **configuration.model_dump(mode="json"),
            "modelled_factors": ["measured_velocity"],
            "unmodelled_factors": [
                "acceleration",
                "turning",
                "road_and_lane_geometry",
                "object_interaction",
                "object_class_specific_motion",
            ],
        },
        predictor=predictor.name,
        model_name=MODEL_NAME,
        is_baseline=predictor.is_baseline,
        horizon_s=settings.prediction.horizon_s,
        interval_s=settings.prediction.interval_s,
        points_per_trajectory=configuration.points_per_trajectory,
        uncertainty_model=UNCERTAINTY_MODEL,
        uncertainty_is_heuristic=True,
        prediction_statuses=list(PredictionStatus),
        summary=context.prediction.summary(),
    )
