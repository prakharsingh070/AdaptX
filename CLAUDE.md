# ADAPT-X - Claude Project Instructions

## Project Name

ADAPT-X (Adaptive Dynamic Perception and Tracking)

## Project Goal

ADAPT-X is a LiDAR-based perception system for autonomous and smart vehicles operating in dynamic environments. It converts raw LiDAR point clouds into a 2.5D spatial representation and dynamically changes spatial resolution based on environmental complexity, object movement, uncertainty, ego-vehicle trajectory, and collision risk.

The central innovation is risk-aware adaptive perception: low-risk regions use coarse representation while regions containing moving objects, uncertainty, or potential collision zones receive finer detail. The system should also predict future object movement so perception resolution can increase before a region becomes critical.

## Core Pipeline

LiDAR / CARLA -> point-cloud processing -> ground and noise filtering -> object detection -> object tracking -> 2.5D occupancy mapping -> risk and uncertainty estimation -> trajectory prediction -> predictive risk -> adaptive resolution controller -> adaptive 2.5D map -> benchmarking -> dashboard.

## Core Modules

1. LiDAR processing
2. Object detection
3. Temporal tracking
4. 2.5D mapping
5. Risk engine
6. Uncertainty engine
7. Trajectory prediction
8. Adaptive resolution controller
9. Predictive risk heatmap
10. CARLA integration
11. Scenario generator
12. Benchmark engine
13. Event replay
14. FastAPI backend
15. Dashboard

## Development Principles

Work phase by phase. Before a major module:

1. Inspect the existing repository and interfaces.
2. Propose the implementation plan and affected files.
3. Implement only the requested feature.
4. Write or update tests.
5. Run focused tests and measure performance where applicable.
6. Report changes and assumptions.

Do not silently redesign unrelated modules.

## Non-Negotiable Rules

- Never fabricate sensor measurements, accuracy numbers, FPS, latency, CPU usage, GPU usage, memory values, or benchmark results.
- Clearly label simulation and demo values.
- Do not claim production readiness for autonomous-vehicle deployment without real-world validation.
- Keep modules modular, typed, configurable, testable, and observable.
- Keep dashboard code separate from perception business logic.
- Maintain a fixed-resolution baseline for quantitative comparison.
- Record important architecture decisions and experiments in `docs/decisions/` and `docs/experiments/`.

## Baseline Requirement

Run comparable scenarios using both fixed-resolution perception and ADAPT-X adaptive-resolution perception. Compare FPS, latency, CPU, GPU, memory, processed point count, active cells, map resolution, detection, tracking, prediction, and safety-related behavior. Report only measured results.

## UI Requirement

The dashboard should expose real backend data where possible and include: 3D LiDAR view, system status, recent events, detected objects, adaptive 2.5D map, risk map, tracking and prediction, performance cards, LiDAR processing, perception, prediction, routing, decision, planning, CARLA simulator, scenario manager, settings, and logs.

## CARLA and Scenarios

CARLA is the controlled test environment, not the project itself. It should provide repeatable roads, intersections, vehicles, pedestrians, cyclists, obstacles, traffic, weather, lighting, and sensor configurations. Scenario generation must support random seeds and reproducible configurations.

## Event Replay

Record events such as object detection, track updates, risk changes, prediction updates, predicted conflict creation, resolution increases, and resolution decreases. Each event should include a timestamp, event type, and relevant metadata.

## Current Development Status

- Phase 1: Project foundation - DONE (backend foundation, data contracts, module
  interfaces, FastAPI + WebSocket, LiDAR ingest validation, CARLA boundary, tests,
  Docker. No perception algorithm implemented. See `docs/ARCHITECTURE.md`.)
- Phase 2: LiDAR processing - TODO
- Phase 3: Object detection - TODO
- Phase 4: Tracking - TODO
- Phase 5: 2.5D mapping - TODO
- Phase 6: Risk and uncertainty - TODO
- Phase 7: Adaptive resolution - TODO
- Phase 8: Prediction - TODO
- Phase 9: CARLA - TODO
- Phase 10: Scenario generation and replay - TODO
- Phase 11: Benchmarking - TODO
- Phase 12: Dashboard and final integration - TODO

## Knowledge Base

Read the relevant documents in `docs/knowledge-base/` before changing a module. Read `docs/knowledge-base/20-constraints.md` before benchmarking or demo work. Record accepted architectural choices in `docs/decisions/architecture-decisions.md` and measured work in `docs/experiments/experiment-log.md`.
