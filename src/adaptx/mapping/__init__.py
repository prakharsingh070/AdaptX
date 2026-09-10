"""2.5D mapping layer.

Two comparable variants exist, and both stay available so a measurement of one
is never presented as the other (ADR-003):

**Fixed resolution** (Phase 6) - :class:`FixedResolutionMapper` applies one
uniform cell size over the configured bounds. It is the baseline.

**Region adaptive** (Phase 8) - :class:`HeuristicResolutionController` decides
a resolution level per region from risk, uncertainty, predicted motion,
density and proximity, and :class:`TiledAdaptiveMapper` builds a tiled map
applying those decisions (ADR-037).

A mapper never chooses its own resolution in either variant. Deciding belongs
to the controller; applying belongs to the mapper (ADR-029).
"""

from adaptx.mapping.adaptive_mapper import TiledAdaptiveMapper, build_adaptive_mapper
from adaptx.mapping.comparison import compare, describe_adaptive, describe_fixed
from adaptx.mapping.controller import HeuristicResolutionController, build_controller
from adaptx.mapping.grid_mapper import FixedResolutionMapper, build_mapper
from adaptx.mapping.interfaces import (
    AdaptiveMapper,
    RegionAdaptiveMapper,
    ResolutionController,
)
from adaptx.mapping.tiles import TileGrid

__all__ = [
    "AdaptiveMapper",
    "FixedResolutionMapper",
    "HeuristicResolutionController",
    "RegionAdaptiveMapper",
    "ResolutionController",
    "TileGrid",
    "TiledAdaptiveMapper",
    "build_adaptive_mapper",
    "build_controller",
    "build_mapper",
    "compare",
    "describe_adaptive",
    "describe_fixed",
]
