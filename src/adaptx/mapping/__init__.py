"""2.5D mapping layer.

Phase 6 implements a deterministic, frame-local, fixed-resolution baseline:
one uniform cell size applied over configured bounds, producing occupancy and
height statistics per cell.

**Adaptive resolution is not implemented.** A mapper applies a
:class:`~adaptx.models.resolution.ResolutionDecision` it is handed; deciding
what that resolution should be belongs to a
:class:`~adaptx.mapping.interfaces.ResolutionController`, which does not exist
(ADR-029).
"""

from adaptx.mapping.grid_mapper import FixedResolutionMapper, build_mapper
from adaptx.mapping.interfaces import AdaptiveMapper, ResolutionController

__all__ = [
    "AdaptiveMapper",
    "FixedResolutionMapper",
    "ResolutionController",
    "build_mapper",
]
