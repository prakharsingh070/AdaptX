"""Trajectory-prediction layer.

Phase 5 implements a deterministic constant-velocity baseline: measured
velocity extrapolated over a configurable horizon, with heuristic uncertainty
that grows with extrapolation time. No learned model, no Kalman filter, no
map or lane conditioning, and no collision reasoning - that belongs to the risk
engine, which consumes this output.
"""

from adaptx.prediction.constant_velocity import (
    MODEL_NAME,
    UNCERTAINTY_MODEL,
    ConstantVelocityPredictor,
    build_predictor,
)
from adaptx.prediction.interfaces import TrajectoryPredictor

__all__ = [
    "MODEL_NAME",
    "UNCERTAINTY_MODEL",
    "ConstantVelocityPredictor",
    "TrajectoryPredictor",
    "build_predictor",
]
