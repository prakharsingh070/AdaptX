"""Adaptive spatial resolution service (Phase 8).

Orchestrates the two halves of adaptive mapping and contains neither
algorithm::

    assessments + tracks + trajectories -> HeuristicResolutionController -> ResolutionPlan
    frame + plan                        -> TiledAdaptiveMapper           -> AdaptiveSpatialMap

State, and why this one has some
--------------------------------
:mod:`adaptx.services.mapping_service`, :mod:`adaptx.services.prediction_service`
and :mod:`adaptx.services.risk_service` are all stateless with respect to
perception. **This one is not**, and the difference is deliberate.

Stabilising resolution is temporal by definition: deciding whether a region has
been quiet long enough to give up detail requires remembering what it held and
for how long (ADR-039). That history lives in the controller, so this service
is stateful in exactly the way :mod:`adaptx.services.tracking_service` is, and
follows the same rule - the state sits on the application context, is guarded
by a lock, and is cleared on shutdown (ADR-025).

The **map** remains frame-local (ADR-030). Nothing about frame ``N`` occupancy
survives into frame ``N+1``; only the resolution levels do, and those are a
policy decision rather than a measurement.

Consequence for callers: frames must arrive in temporal order, and unrelated
sequences must be separated by a reset - exactly the contract ``/track`` and
``/predict`` already carry.
"""

from __future__ import annotations

from datetime import datetime
from threading import Lock

from adaptx.config.settings import AdaptiveResolutionSettings, MapSettings
from adaptx.core.logging import get_logger
from adaptx.mapping.adaptive_mapper import TiledAdaptiveMapper
from adaptx.mapping.comparison import compare
from adaptx.mapping.controller import HeuristicResolutionController
from adaptx.models.adaptive_map import (
    AdaptiveSpatialMap,
    AdaptiveSpatialMapSummary,
    MappingComparison,
)
from adaptx.models.adaptive_resolution import (
    DETAIL_POLICY_MODEL,
    AdaptiveResolutionConfiguration,
    ResolutionPlan,
)
from adaptx.models.point_cloud import PointCloudFrame
from adaptx.models.prediction import PredictedTrajectory
from adaptx.models.risk_assessment import RiskAssessment, RiskAssessmentResult
from adaptx.models.spatial_map import SpatialMap
from adaptx.models.tracking import TrackedObject
from adaptx.models.tracking_result import TrackingResult

logger = get_logger(__name__)


class AdaptiveMappingService:
    """Runs the resolution controller and the region-adaptive mapper."""

    def __init__(
        self,
        map_settings: MapSettings,
        settings: AdaptiveResolutionSettings,
        controller: HeuristicResolutionController | None = None,
        mapper: TiledAdaptiveMapper | None = None,
    ) -> None:
        self._map_settings = map_settings
        self._settings = settings
        self._controller = (
            controller
            if controller is not None
            else HeuristicResolutionController(map_settings, settings)
        )
        self._mapper = mapper if mapper is not None else TiledAdaptiveMapper(map_settings, settings)
        self._lock = Lock()
        self._frames_mapped = 0
        self._last_summary: AdaptiveSpatialMapSummary | None = None
        self._last_plan_duration_ms: float | None = None

    # -- components --------------------------------------------------------
    @property
    def controller(self) -> HeuristicResolutionController:
        """The resolution controller under management."""
        return self._controller

    @property
    def mapper(self) -> TiledAdaptiveMapper:
        """The region-adaptive mapper under management."""
        return self._mapper

    @property
    def enabled(self) -> bool:
        """Whether adaptation is switched on in configuration."""
        return self._settings.enabled

    @property
    def configuration(self) -> AdaptiveResolutionConfiguration:
        """Effective controller configuration."""
        return self._controller.configuration

    @property
    def frames_mapped(self) -> int:
        """Frames mapped adaptively since the last reset."""
        with self._lock:
            return self._frames_mapped

    @property
    def last_summary(self) -> AdaptiveSpatialMapSummary | None:
        """Summary of the most recent adaptive map, or ``None`` before the first.

        Only the summary is kept. Holding the last tiled map would pin every
        tile arrays for the lifetime of the process to serve a status endpoint
        that reports counts.
        """
        with self._lock:
            return self._last_summary

    # -- planning and mapping ---------------------------------------------
    def plan(
        self,
        assessments: list[RiskAssessment],
        *,
        tracks: list[TrackedObject] | None = None,
        trajectories: list[PredictedTrajectory] | None = None,
        timestamp: datetime | None = None,
        frame_id: int = 0,
        sensor_id: str = "unknown",
    ) -> ResolutionPlan:
        """Decide a resolution for every region.

        Guarded by the lock: the controller carries stabilisation state, so two
        concurrent frames would otherwise interleave dwell counters and produce
        a plan neither of them asked for.
        """
        with self._lock:
            plan = self._controller.plan(
                assessments,
                tracks=tracks,
                trajectories=trajectories,
                timestamp=timestamp,
                frame_id=frame_id,
                sensor_id=sensor_id,
            )
            self._last_plan_duration_ms = plan.duration_ms
        return plan

    def build(self, frame: PointCloudFrame, plan: ResolutionPlan) -> AdaptiveSpatialMap:
        """Map one processed frame against a plan.

        The mapper call is outside the lock: it holds no mutable state, so two
        concurrent frames cannot interfere. Only the bookkeeping is guarded.
        """
        adaptive_map = self._mapper.build(frame, plan)
        summary = adaptive_map.summary(controller_duration_ms=plan.duration_ms)
        with self._lock:
            self._frames_mapped += 1
            self._last_summary = summary
        return adaptive_map

    def run(
        self,
        frame: PointCloudFrame,
        assessments: list[RiskAssessment],
        *,
        tracks: list[TrackedObject] | None = None,
        trajectories: list[PredictedTrajectory] | None = None,
    ) -> AdaptiveSpatialMap:
        """Plan and map one frame in the order the pipeline runs them."""
        plan = self.plan(
            assessments,
            tracks=tracks,
            trajectories=trajectories,
            timestamp=frame.timestamp,
            frame_id=frame.frame_id,
            sensor_id=frame.sensor_id,
        )
        return self.build(frame, plan)

    def run_from_pipeline(
        self,
        frame: PointCloudFrame,
        risk: RiskAssessmentResult,
        tracking: TrackingResult,
        *,
        trajectories: list[PredictedTrajectory] | None = None,
    ) -> AdaptiveSpatialMap:
        """Plan and map one frame from the upstream results directly.

        Tracks come from the tracking result because a ``RiskAssessment``
        carries a distance rather than a position: Phase 7 is deliberately not
        given a spatial vocabulary (ADR-036), so the controller reads positions
        from the tracks the assessments describe.
        """
        return self.run(
            frame,
            risk.assessments,
            tracks=tracking.tracks,
            trajectories=trajectories,
        )

    def compare_with_fixed(
        self, adaptive_map: AdaptiveSpatialMap, spatial_map: SpatialMap
    ) -> MappingComparison:
        """Compare an adaptive map against a fixed map of the same frame."""
        return compare(spatial_map, adaptive_map)

    # -- lifecycle ---------------------------------------------------------
    def reset(self) -> None:
        """Clear the stabilisation state and the observational counters.

        Unlike the other mapping services, this genuinely discards perception
        policy state: every region forgets the level it held and how long it
        has been quiet. Call it between unrelated sequences.
        """
        with self._lock:
            self._frames_mapped = 0
            self._last_summary = None
            self._last_plan_duration_ms = None
            self._controller.reset()
        self._mapper.reset()
        logger.info("adaptive mapping state cleared")

    def summary(self) -> dict[str, object]:
        """Compact state summary for status and telemetry.

        Counts, the level distribution and configuration only. **Never the
        tiles**: a single CRITICAL region at 0.1 m holds 10,000 cells, and the
        whole map can hold hundreds of thousands. The full map is returned by
        ``POST /api/v1/lidar/adaptive-map`` instead - the same rule detection,
        tracking, prediction, mapping and risk already follow.
        """
        with self._lock:
            summary = self._last_summary
            frames = self._frames_mapped

        grid = self._controller.tile_grid()
        state: dict[str, object] = {
            "controller": self._controller.name,
            "mapper": self._mapper.name,
            "is_adaptive": self._mapper.is_adaptive,
            "is_baseline": self._controller.is_baseline,
            "policy_model": DETAIL_POLICY_MODEL,
            "enabled": self._settings.enabled,
            "priority_is_heuristic": True,
            "is_collision_probability": False,
            "lifecycle": "frame_local_map_with_stabilised_resolution",
            "frames_mapped": frames,
            "frame_index": self._controller.frame_index,
            "tile_size_m": self._settings.tile_size_m,
            "tile_count": grid.tile_count,
            "resolution_levels": {
                level.value: size for level, size in self._controller.levels().items()
            },
            "configuration": self.configuration.model_dump(mode="json"),
        }
        if summary is None:
            state["note"] = "no frame has been mapped adaptively since the last reset"
            return state

        accounting = summary.accounting
        state.update(
            {
                "last_frame_id": summary.frame_id,
                "last_map_timestamp": summary.timestamp.isoformat(),
                "tiles_by_level": summary.tiles_by_level,
                "cells_by_level": summary.cells_by_level,
                "total_cells": accounting.total_cell_count,
                "occupied_cells": accounting.occupied_cell_count,
                "occupancy_ratio": accounting.occupancy_ratio,
                "input_points": accounting.input_point_count,
                "mapped_points": accounting.mapped_point_count,
                "out_of_bounds_points": accounting.out_of_bounds_point_count,
                "finest_resolution_m": summary.finest_resolution_m,
                "coarsest_resolution_m": summary.coarsest_resolution_m,
                "area_weighted_resolution_m": summary.area_weighted_resolution_m,
                "resolution_changes": summary.changed_tile_count,
                "grid_bytes": summary.grid_bytes,
                "controller_duration_ms": summary.controller_duration_ms,
                "mapping_duration_ms": summary.mapping_duration_ms,
            }
        )
        return state
