"""2.5D mapping service (Phase 6).

Like :mod:`adaptx.services.prediction_service` and unlike
:mod:`adaptx.services.tracking_service`, this service is **stateless with
respect to perception**. A map is a pure function of one frame and one
resolution decision, so nothing needs to be carried between frames, and
crucially nothing *can* be: map ``N`` cannot contaminate map ``N+1`` because
map ``N`` is not retained as input to anything (ADR-030).

The only state held is observational - a counter and the last map's summary,
used for status and telemetry. Neither influences a mapping result. Resetting
this service changes what the status endpoint reports and nothing about what
the mapper produces.

The service orchestrates; it contains no mapping algorithm::

    PointCloudFrame + ResolutionDecision -> FixedResolutionMapper -> SpatialMap
"""

from __future__ import annotations

from threading import Lock

from adaptx.config.settings import MapSettings
from adaptx.core.logging import get_logger
from adaptx.mapping.grid_mapper import FixedResolutionMapper
from adaptx.models.point_cloud import PointCloudFrame
from adaptx.models.resolution import ResolutionDecision
from adaptx.models.spatial_map import MappingConfiguration, SpatialMap, SpatialMapSummary

logger = get_logger(__name__)


class MappingService:
    """Runs the configured mapper over a processed frame."""

    def __init__(self, settings: MapSettings, mapper: FixedResolutionMapper | None = None) -> None:
        self._settings = settings
        self._mapper = mapper if mapper is not None else FixedResolutionMapper(settings)
        self._lock = Lock()
        self._frames_mapped = 0
        self._last_summary: SpatialMapSummary | None = None

    @property
    def mapper(self) -> FixedResolutionMapper:
        """The mapper under management."""
        return self._mapper

    @property
    def configuration(self) -> MappingConfiguration:
        """Effective mapping configuration at the default resolution."""
        return self._mapper.configuration(self._settings.resolution_m)

    @property
    def frames_mapped(self) -> int:
        """Frames mapped since the last reset."""
        with self._lock:
            return self._frames_mapped

    @property
    def last_summary(self) -> SpatialMapSummary | None:
        """Summary of the most recent map, or ``None`` before the first frame.

        Only the summary is kept. Holding the last grid would pin megabytes of
        arrays for the lifetime of the process to serve a status endpoint that
        reports counts.
        """
        with self._lock:
            return self._last_summary

    def default_resolution(self) -> ResolutionDecision:
        """The configured fixed-resolution decision."""
        return self._mapper.default_resolution()

    def build(
        self, frame: PointCloudFrame, resolution: ResolutionDecision | None = None
    ) -> SpatialMap:
        """Map one processed frame.

        The mapper call is outside the lock: it holds no mutable state, so two
        concurrent frames cannot interfere. Only the bookkeeping is guarded.

        Args:
            frame: A processed frame from the Phase 2 pipeline.
            resolution: Resolution to apply; the configured default when
                ``None``.
        """
        spatial_map = self._mapper.build(frame, resolution)
        with self._lock:
            self._frames_mapped += 1
            self._last_summary = spatial_map.summary()
        return spatial_map

    def reset(self) -> None:
        """Clear the observational counters.

        Affects reporting only. There is no accumulated map state to drop,
        because mapping is frame-local.
        """
        with self._lock:
            self._frames_mapped = 0
            self._last_summary = None
        self._mapper.reset()
        logger.info("mapping counters reset")

    def summary(self) -> dict[str, object]:
        """Compact state summary for status and telemetry.

        Counts, dimensions and configuration only. **Never the grid itself**:
        a 0.5 m map over the default bounds is 57,600 cells, and a 0.25 m map
        is 230,400. Pushing that down a status channel every tick would be
        frame data in the wrong place - the same rule detection, tracking and
        prediction already follow.
        """
        with self._lock:
            summary = self._last_summary
            frames = self._frames_mapped

        bounds = self._mapper.bounds
        height, width = self._mapper.grid_shape(self._settings.resolution_m)
        state: dict[str, object] = {
            "mapper": self._mapper.name,
            "is_adaptive": self._mapper.is_adaptive,
            "adaptive_resolution_implemented": False,
            "lifecycle": "frame_local",
            "frames_mapped": frames,
            "resolution_m": self._settings.resolution_m,
            "resolution_source": self.default_resolution().source.value,
            "width": width,
            "height": height,
            "total_cells": width * height,
            "bounds": bounds.model_dump(mode="json"),
            "configuration": self.configuration.model_dump(mode="json"),
        }
        if summary is None:
            state["note"] = "no frame has been mapped since the last reset"
            return state

        accounting = summary.accounting
        state.update(
            {
                "last_frame_id": summary.frame_id,
                "last_map_timestamp": summary.timestamp.isoformat(),
                "input_points": accounting.input_point_count,
                "mapped_points": accounting.mapped_point_count,
                "out_of_bounds_points": accounting.out_of_bounds_point_count,
                "occupied_cells": accounting.occupied_cell_count,
                "occupancy_ratio": accounting.occupancy_ratio,
                "duration_ms": summary.duration_ms,
            }
        )
        return state
