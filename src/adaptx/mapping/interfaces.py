"""Mapping-layer contracts.

See ``docs/knowledge-base/05_2.5d-mapping.md`` and
``docs/knowledge-base/10_adaptive-resolution.md``. No implementation exists in
Phase 1.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from adaptx.models.map import AdaptiveMap, ResolutionContext, ResolutionLevel
from adaptx.models.point_cloud import PointCloudFrame
from adaptx.models.risk import RiskField
from adaptx.models.tracking import TrackedObject
from adaptx.models.vehicle import VehicleState


class ResolutionController(ABC):
    """Decides how much spatial detail a region receives.

    This is the central ADAPT-X decision point: *given the environment and risk
    around this region, what resolution should it use?*

    An implementation must also honour the stabilisation requirement in
    ``docs/knowledge-base/10_adaptive-resolution.md``: resolution must not
    oscillate between frames, so ``ResolutionContext.current_level`` is
    provided for hysteresis, smoothing or a minimum dwell time.
    """

    name: str = "resolution_controller"

    @abstractmethod
    def select_resolution(self, context: ResolutionContext) -> ResolutionLevel:
        """Return the resolution level for the region described by ``context``."""

    @abstractmethod
    def cell_size_m(self, level: ResolutionLevel) -> float:
        """Return the cell edge length in metres for ``level``."""


class AdaptiveMapper(ABC):
    """Builds a 2.5D occupancy map from perception output.

    Implementations exist in two comparable variants (ADR-003): a
    fixed-resolution baseline and the ADAPT-X adaptive mapper. The variant is
    recorded on the produced map as ``AdaptiveMap.is_adaptive``.
    """

    name: str = "adaptive_mapper"

    @abstractmethod
    def update(
        self,
        frame: PointCloudFrame,
        *,
        ego_state: VehicleState | None = None,
        tracks: list[TrackedObject] | None = None,
        risk_field: RiskField | None = None,
    ) -> AdaptiveMap:
        """Fold ``frame`` into the map and return the resulting snapshot."""

    @abstractmethod
    def reset(self) -> None:
        """Discard all accumulated map state."""
