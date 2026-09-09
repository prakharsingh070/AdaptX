# Testing Strategy

## Unit Tests

Cover risk calculation, grid generation, coordinate transformation, resolution selection, tracking, prediction, schema validation, and event generation.

## Integration Tests

Exercise these paths:

- LiDAR -> detection
- detection -> tracking
- tracking -> prediction
- prediction -> risk
- risk -> adaptive map
- adaptive map -> API response

## System Test

Run CARLA -> LiDAR -> ADAPT-X -> dashboard with a controlled scenario and verify timestamps, object state, map state, event replay, and error handling.

## Performance Test

Run equivalent scenarios with fixed-resolution and adaptive-resolution processing. Record workload, latency, resource use, and perception metrics using measured values only.

## Test Requirements

Tests should be deterministic where possible, use seeded scenarios, validate units and coordinate frames, and distinguish expected unavailable-data behavior from failures.
