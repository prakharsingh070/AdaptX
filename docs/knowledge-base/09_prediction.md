# Prediction

## Flow

Past observations -> current state -> future trajectory.

For each tracked object, prediction may use current position, velocity, acceleration, heading, and object class to estimate positions at future horizons such as `t+1`, `t+2`, and `t+3`.

## Conflict Analysis

Compare predicted object trajectories with the ego trajectory. Their spatial and temporal overlap contributes to potential conflict and future risk.

## Requirements

Document prediction horizon, timestep, model assumptions, confidence, uncertainty growth, coordinate frame, handling of missing history, and validation metrics. Prediction outputs must be distinguishable from measured positions.
