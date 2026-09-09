"""Risk layer.

Phase 1 provides the :class:`~adaptx.risk.interfaces.RiskEngine` contract and a
deterministic baseline calculator used for testing. The ADAPT-X risk engine
itself is not implemented.
"""

from adaptx.risk.baseline import BaselineProximityRiskEngine
from adaptx.risk.interfaces import RiskEngine

__all__ = ["BaselineProximityRiskEngine", "RiskEngine"]
