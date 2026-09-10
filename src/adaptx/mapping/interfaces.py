"""Mapping-layer contracts.

See ``docs/knowledge-base/05_2.5d-mapping.md`` and
``docs/knowledge-base/10_adaptive-resolution.md``.

The two contracts here sit either side of the resolution decision, and the
split is deliberate (ADR-029)::

    ResolutionContext -> [ResolutionController] -> ResolutionDecision -> AdaptiveMapper

A controller decides *how much detail a region deserves* from risk and
uncertainty. A mapper *applies* a decision it is given. Phase 6 implements the
mapper against a fixed decision; no controller exists.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

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

    **Not implemented.** Phase 6 supplies mappers with a fixed
    :class:`~adaptx.models.resolution.ResolutionDecision` instead.
    """

    name: str = "resolution_controller"

    @abstractmethod
    def select_resolution(self, context: ResolutionContext) -> ResolutionLevel:
        """Return the resolution level for the region described by ``context``."""

    @abstractmethod
    def cell_size_m(self, level: ResolutionLevel) -> float:
        """Return the cell edge length in metres for ``level``."""


class AdaptiveMapper(ABC):
    """Builds a 2.5D map from a processed point cloud.

    Implementations exist in two comparable variants (ADR-003): a
    fixed-resolution baseline and, later, the ADAPT-X adaptive mapper. The
    variant is recorded on every map as ``is_adaptive`` so a measurement of one
    can never be presented as the other.

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
