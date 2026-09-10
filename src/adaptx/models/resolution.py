"""Resolution decision contract (Phase 6).

The single value a mapper needs in order to build a map: **how large is a
cell, who chose that, and why**.

Relationship to :class:`~adaptx.models.map.ResolutionContext`
------------------------------------------------------------
These two contracts sit on opposite sides of the resolution decision and must
not be confused (ADR-029):

``ResolutionContext``
    The *inputs* to a decision - risk, uncertainty, object density, object
    speed, whether a region lies in the ego path. Consumed by a
    :class:`~adaptx.mapping.interfaces.ResolutionController`, which is not
    implemented.

``ResolutionDecision`` (this module)
    The *output* of that decision, and the only resolution information the
    mapper ever sees.

::

    ResolutionContext -> [ResolutionController] -> ResolutionDecision -> mapper

That ordering is the reason the mapper cannot make a risk-aware choice by
accident: it is never handed the material such a choice would need. Phase 6
produces decisions with ``source = FIXED`` only; an adaptive controller is a
later phase and will produce ``source = ADAPTIVE`` through the same contract,
with no change to the mapper.
"""

from __future__ import annotations

import math
from enum import StrEnum

from pydantic import Field, field_validator

from adaptx.models.common import AdaptXModel

#: Reason recorded by the Phase 6 fixed-resolution baseline.
FIXED_BASELINE_REASON = "phase6_fixed_baseline"


class ResolutionSource(StrEnum):
    """Where a resolution value came from.

    Recorded on every map so a benchmark result can never confuse a
    fixed-resolution run with an adaptive one (ADR-003).

    ``FIXED``
        A configured constant. The only source Phase 6 produces.
    ``ADAPTIVE``
        Chosen per region by a resolution controller from risk and
        uncertainty. **Not implemented** - reserved so the contract does not
        change when it is.
    ``OVERRIDE``
        Supplied explicitly by a caller for one operation, e.g. a benchmark
        sweeping cell sizes. Distinguished from ``FIXED`` because it did not
        come from configuration.
    """

    FIXED = "fixed"
    ADAPTIVE = "adaptive"
    OVERRIDE = "override"


class ResolutionDecision(AdaptXModel):
    """The resolution in force for one mapping operation, and its provenance.

    Deliberately minimal. It carries a cell size and an explanation, and
    nothing a mapper could use to second-guess the choice.
    """

    resolution_m: float = Field(
        gt=0.0, description="Cell edge length in metres. Uniform across the map in Phase 6."
    )
    source: ResolutionSource = Field(
        default=ResolutionSource.FIXED, description="Where this resolution came from."
    )
    reason: str = Field(
        min_length=1,
        description=(
            "Human-readable justification, recorded on the map for traceability. "
            "For the Phase 6 baseline this is always the same constant, because "
            "the choice never varies."
        ),
    )
    requested_by: str = Field(
        default="configuration",
        min_length=1,
        description="Component that produced this decision, for attribution.",
    )

    @field_validator("resolution_m")
    @classmethod
    def _require_finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("resolution_m must be finite")
        return value

    @property
    def is_adaptive(self) -> bool:
        """Whether this decision came from an adaptive controller.

        False for the Phase 6 baseline. Carried onto every map so a
        fixed-resolution measurement is never presented as an adaptive one.
        """
        return self.source is ResolutionSource.ADAPTIVE

    @classmethod
    def fixed(cls, resolution_m: float) -> ResolutionDecision:
        """The Phase 6 baseline decision: a configured constant, applied everywhere."""
        return cls(
            resolution_m=resolution_m,
            source=ResolutionSource.FIXED,
            reason=FIXED_BASELINE_REASON,
            requested_by="configuration",
        )

    @classmethod
    def override(cls, resolution_m: float, *, reason: str, requested_by: str) -> ResolutionDecision:
        """A resolution supplied for one operation rather than read from configuration."""
        return cls(
            resolution_m=resolution_m,
            source=ResolutionSource.OVERRIDE,
            reason=reason,
            requested_by=requested_by,
        )
