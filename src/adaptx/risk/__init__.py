"""Risk and uncertainty layer.

Phase 7 implements a **deterministic heuristic** risk and uncertainty baseline:
proximity, rate of approach and predicted approach, combined as a weighted mean
over the factors actually available, with uncertainty reported separately
rather than folded into the score.

The score is not a probability of collision, not calibrated, and not validated
against labelled risk data - none exists (ADR-032).

:class:`~adaptx.risk.baseline.BaselineProximityRiskEngine` is retained
unchanged as the proximity-only comparison reference.

**Risk does not decide resolution.** How much spatial detail a region receives
is a separate decision, made by a
:class:`~adaptx.mapping.interfaces.ResolutionController`, which consumes these
assessments and is never consulted by this engine (ADR-036).
"""

from adaptx.risk.baseline import BaselineProximityRiskEngine
from adaptx.risk.heuristic import SCORING_MODEL, HeuristicRiskEngine, build_risk_engine
from adaptx.risk.interfaces import RiskEngine

__all__ = [
    "SCORING_MODEL",
    "BaselineProximityRiskEngine",
    "HeuristicRiskEngine",
    "RiskEngine",
    "build_risk_engine",
]
