"""Mapping-layer contracts.

See ``docs/knowledge-base/05_2.5d-mapping.md`` and
``docs/knowledge-base/10_adaptive-resolution.md``.

The two contracts here sit either side of the resolution decision, and the
split is deliberate (ADR-029)::

    ResolutionContext -> [ResolutionController] -> ResolutionDecision -> AdaptiveMapper

A controller decides *how much detail a region deserves* from risk and
uncertainty. A mapper *applies* a decision it is given, and the separation is
what keeps a risk-aware choice from leaking into the mapper by accident.

Phase 6 implements :class:`AdaptiveMapper` against a fixed decision. Phase 8
implements :class:`ResolutionController` and :class:`RegionAdaptiveMapper`, the
variant that applies one decision per region. Both mappers remain available so
a fixed-resolution run and an adaptive one can be measured against each other
(ADR-003).
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from adaptx.models.adaptive_map import AdaptiveSpatialMap
from adaptx.models.adaptive_resolution import ResolutionPlan
from adaptx.models.map import ResolutionContext, ResolutionLevel
from adaptx.models.point_cloud import PointCloudFrame
from adaptx.models.resolution import ResolutionDecision
from adaptx.models.spatial_map import SpatialMap


class ResolutionController(ABC):
    """Decides how much spatial detail a region receives.

    This is the central ADAPT-X decision point: *given the environment and risk
    around this region, what resolution should it use?*

    An implementation must also honour the stabilisation requirement in
    ``docs/knowledge-base/10_adaptive-resolution.md``: resolution must not
    oscillate between frames, so ``ResolutionContext.current_level`` is
    provided for hysteresis, smoothing or a minimum dwell time.

    Implemented in Phase 8 by
    :class:`~adaptx.mapping.controller.HeuristicResolutionController`, which
    combines an asymmetric hysteresis margin with a minimum dwell time
    (ADR-039).
    """

    name: str = "resolution_controller"

    @abstractmethod
    def select_resolution(self, context: ResolutionContext) -> ResolutionLevel:
        """Return the resolution level for the region described by ``context``."""

    @abstractmethod
    def cell_size_m(self, level: ResolutionLevel) -> float:
        """Return the cell edge length in metres for ``level``."""


class AdaptiveMapper(ABC):
    """Builds a **uniform-resolution** 2.5D map from a processed point cloud.

    One :class:`~adaptx.models.resolution.ResolutionDecision` covers the whole
    map, so an implementation of this contract has exactly one cell size. The
    Phase 6 baseline implements it.

    The region-adaptive variant does **not**: a map whose resolution varies by
    region needs a decision per region, which this signature cannot express.
    That variant implements :class:`RegionAdaptiveMapper` instead, and both
    record ``is_adaptive`` so a measurement of one can never be presented as
    the other (ADR-003, ADR-037).

    A mapper is handed a resolution; it never chooses one. It is deliberately
    not given tracks, predicted trajectories, risk or uncertainty, so a
    risk-aware choice is structurally impossible here rather than merely
    discouraged (ADR-029).

    Mapping is **frame-local** (ADR-030): a call builds a whole map from one
    frame and nothing accumulates between calls. ``reset`` exists for a future
    accumulating implementation and is a documented no-op for one that holds no
    state.
    """

    name: str = "adaptive_mapper"
    #: True for a risk-aware mapper, false for the fixed-resolution baseline.
    is_adaptive: bool = False

    @abstractmethod
    def build(
        self, frame: PointCloudFrame, resolution: ResolutionDecision | None = None
    ) -> SpatialMap:
        """Build a complete map from ``frame`` at ``resolution``.

        Args:
            frame: A processed frame from the Phase 2 pipeline. A mapper never
                repeats preprocessing.
            resolution: The resolution to apply. Implementations fall back to
                their configured default when this is ``None``.

        Returns:
            A map of the configured bounds, with per-cell occupancy, point
            counts and height statistics, plus accounting for every input point.
        """

    @abstractmethod
    def reset(self) -> None:
        """Discard any accumulated map state."""


class RegionAdaptiveMapper(ABC):
    """Builds a 2.5D map whose resolution varies by region (Phase 8).

    The sibling of :class:`AdaptiveMapper`, and the reason it has a sibling: a
    map at several resolutions cannot be described by a single
    ``ResolutionDecision``, so it is handed a
    :class:`~adaptx.models.adaptive_resolution.ResolutionPlan` carrying one
    decision per region (ADR-037).

    Everything else is unchanged. A mapper still never chooses its own
    resolution, is still never given tracks, trajectories, risk or uncertainty,
    and still cannot make a risk-aware choice even by accident: the plan
    already contains every choice it needs (ADR-029).

    Mapping remains **frame-local** (ADR-030). Nothing accumulates between
    calls. The stabilisation history that makes adaptive resolution stable
    lives in the controller, never in a map.
    """

    name: str = "region_adaptive_mapper"
    #: True: resolution varies by region.
    is_adaptive: bool = True

    @abstractmethod
    def build(self, frame: PointCloudFrame, plan: ResolutionPlan) -> AdaptiveSpatialMap:
        """Build a complete map from ``frame``, applying ``plan``.

        Args:
            frame: A processed frame from the Phase 2 pipeline. A mapper never
                repeats preprocessing.
            plan: One resolution decision per region, covering the whole map.

        Returns:
            A tiled map with per-cell occupancy, point counts and height
            statistics, plus accounting for every input point.
        """

    @abstractmethod
    def reset(self) -> None:
        """Discard any accumulated map state."""
