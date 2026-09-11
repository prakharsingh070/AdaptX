# ADAPT-X - Claude Project Instructions

## Project Name

ADAPT-X (Adaptive Dynamic Perception and Tracking)

## Project Goal

ADAPT-X is a LiDAR-based perception system for autonomous and smart vehicles operating in dynamic environments. It converts raw LiDAR point clouds into a 2.5D spatial representation and dynamically changes spatial resolution based on environmental complexity, object movement, uncertainty, ego-vehicle trajectory, and collision risk.

The central innovation is risk-aware adaptive perception: low-risk regions use coarse representation while regions containing moving objects, uncertainty, or potential collision zones receive finer detail. The system should also predict future object movement so perception resolution can increase before a region becomes critical.

## Core Pipeline

LiDAR / CARLA -> point-cloud processing -> ground and noise filtering -> object detection -> object tracking -> trajectory prediction -> 2.5D occupancy mapping -> risk and uncertainty estimation -> predictive risk -> adaptive resolution controller -> adaptive 2.5D map -> benchmarking -> dashboard.

Stages up to and including the adaptive resolution controller are implemented as
deterministic, explainable baselines, and CARLA can now feed them simulated LiDAR.
Nothing after that is implemented.

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

Phases 5 to 8 were renumbered when Phase 5 was implemented: trajectory prediction moved
from 8 to 5, because it is what the risk engine needs next, and mapping, risk and
adaptive resolution each shifted one later. `docs/ROADMAP.md` and the `phase` field on
every component in `services/system_service.py` agree with the list below.

- Phase 1: Project foundation - DONE (backend foundation, data contracts, module
  interfaces, FastAPI + WebSocket, LiDAR ingest validation, CARLA boundary, tests,
  Docker. No perception algorithm implemented. See `docs/ARCHITECTURE.md`.)
- Phase 2: LiDAR processing - 2A DONE (input validation, NaN/Inf removal, ROI and
  range filtering, measured per-stage metrics; coordinate convention documented in
  ADR-009). 2B DONE (voxel downsampling, baseline ground segmentation, baseline
  noise filtering; all opt-in per ADR-012. No coordinate transform: shown to be
  unnecessary for these stages, ADR-013). 2C DONE (LiDARProcessingPipeline
  orchestration, per-stage measured timing, configuration snapshot, deterministic
  synthetic benchmark datasets and the fixed-resolution processing baseline of
  ADR-018). Deferred: clustering, coordinate transforms when a tilted or
  multi-sensor mount requires them, and a recorded dataset.
- Phase 3: Object detection - DONE as a geometric baseline (grid connected-component
  clustering, size filtering, dimension-band classification with UNKNOWN on ambiguity,
  POST /api/v1/lidar/detect; ADR-020/021/022). Confidence is a geometric fit score, not
  a probability. No ML detector, no oriented boxes, no velocity, no labelled data - so
  detection accuracy is unmeasured and currently unmeasurable.
- Phase 4: Tracking - DONE as a geometric baseline (gated nearest-neighbour
  association, velocity measured from frame timestamps and null until two
  observations, TENTATIVE/CONFIRMED/COASTING/LOST lifecycle, stateful
  TrackingService on the application context, POST /api/v1/lidar/track and
  /api/v1/tracking/reset; ADR-023/024/025). No learned tracker, no
  re-identification, no trajectory prediction. Tracking correctness is
  unmeasured and unmeasurable without labelled sequences.
- Phase 5: Trajectory prediction - DONE as a deterministic constant-velocity
  baseline (position + velocity * (age_s + t) over a configurable horizon, default
  3.0 s at 0.25 s giving 13 points per track; heuristic uncertainty growing linearly
  with extrapolation time; velocity=None yields no trajectory and a recorded skip
  reason, while a measured standstill yields a stationary one; over-speed velocities
  rejected, never clipped; POST /api/v1/lidar/predict and
  GET /api/v1/prediction/status; ADR-026/027). Uncertainty is a documented heuristic,
  not a calibrated sigma, probability or confidence interval. No acceleration model,
  no Kalman filter, no learned model, no map or lane conditioning, no interaction
  between objects. Prediction accuracy is unmeasured and unmeasurable without
  labelled trajectories.
- Phase 6: 2.5D mapping - DONE as a deterministic frame-local fixed-resolution
  baseline (bounded dense XY grid, uniform cell size, binary occupancy, per-cell
  point count and min/max/mean height, null height for unobserved cells, full
  point accounting, POST /api/v1/lidar/map; ADR-028/029/030/031). ADAPTIVE
  RESOLUTION IS NOT IMPLEMENTED: the mapper applies a resolution it is given and
  never chooses one. Nothing accumulates between frames - not SLAM, no
  localisation, no loop closure, no sensor fusion, no semantic labelling.
  Occupancy is binary, not probabilistic or temporally fused. Map correctness is
  unmeasured: no labelled reference map exists.
- Phase 7: Risk and uncertainty - DONE as a deterministic heuristic baseline
  (object-level risk from proximity, rate of approach and predicted approach,
  combined as a weighted mean over the factors actually available; a factor that
  cannot be computed is dropped, never scored zero; UNKNOWN with a null score
  when nothing can be computed; uncertainty reported separately from risk with
  its contributing reasons visible; map context never lowers risk; scene
  aggregate is a maximum, never a mean; POST /api/v1/lidar/risk;
  ADR-032/033/034/035/036). THE SCORE IS NOT A PROBABILITY OF COLLISION: it is
  not calibrated and has never been validated against labelled risk data,
  because none exists. Thresholds are baseline engineering values, not
  safety-certified limits. No time-to-collision, no trajectory-map
  intersection, no spatial risk field, no ego planned path. RISK DOES NOT
  DECIDE SPATIAL RESOLUTION - that is Phase 8. The proximity-only baseline is
  retained unchanged for comparison.
- Phase 8: Adaptive resolution - DONE as a deterministic heuristic baseline (the map
  extent is partitioned into fixed-size regions and a controller gives each its own cell
  size, so one map genuinely holds several resolutions; regions partition the extent
  exactly, with no gap and no double coverage. The level comes from a detail priority
  combining risk, uncertainty, predicted-motion relevance, object density, proximity and
  measured motion as a weighted mean over the factors actually available, a missing factor
  dropped and never scored zero; `risk_score=None` drops the risk factor AND floors the
  level, because unknown is not low; resolution is stabilised by an asymmetric hysteresis
  margin plus a minimum dwell time, so it does not oscillate; region and cell budgets
  coarsen the lowest-priority regions first and report the demotion; POST
  /api/v1/lidar/adaptive-map and POST /api/v1/map/adaptive/reset;
  ADR-037/038/039/040/041). THE DETAIL PRIORITY IS NOT A PROBABILITY OF COLLISION and is
  not a safety margin: it is an engineering prioritisation score, never calibrated and
  never validated, because no labelled data exists. Whether the allocation is *appropriate*
  is unmeasured and unmeasurable. Measured in Experiment 007: adaptive costs 0.06-0.30x the
  cells of a 0.25 m uniform map but MORE cells than a 1.0 m one, and adaptive mapping is
  slower in wall-clock time than the fixed mapper in every scene - cost scales with region
  count, not cells. The fixed-resolution mapper is retained unchanged as the baseline. No
  learned policy, no ego planned path, no per-cell risk field, no occlusion model.
- Phase 9: CARLA - DONE as a deterministic simulation boundary (CARLA is a DATA
  SOURCE upstream of the pipeline, not a second perception stack; `CarlaSimulationSession`
  owns connect -> configure -> spawn -> attach LiDAR -> tick -> close with actor cleanup on
  failure and world settings restored; CARLA's left-handed frame converts to ADAPT-X's
  right-handed frame EXACTLY ONCE in `carla/conversion.py`, which imports no simulator and
  is therefore fully testable without one; simulation time is authoritative - never the wall
  clock - so frame intervals are exactly `fixed_delta_seconds`; frames are labelled
  `source=simulation` and enter the existing Phase 2 ingest path unchanged; ground truth is
  published on a SEPARATE path and never reaches detection, tracking, prediction, risk or
  adaptive resolution; one hard-coded smoke scenario via `python -m adaptx.carla.smoke`;
  ADR-042/043/044/045). NO LIVE CARLA RUN HAS BEEN EXECUTED: the `carla` package is not
  installed here, so the live smoke test skips and Experiment 008 records only the
  conversion cost of ADAPT-X's own code. Phases 1-8 were not modified to accommodate CARLA.
  Ground truth now exists, which makes accuracy measurable for the first time - but nothing
  measures it yet; that is Phase 11.
- Phase 10: Scenario generation and replay - TODO (next)
- Phase 11: Benchmarking - TODO
- Phase 12: Dashboard and final integration - TODO

## Start Here

New session? Read `docs/PROJECT_STATE.md` first — it is the current snapshot of what is
implemented, verified and off-limits. Then `docs/PHASE_HISTORY.md` for how the project got
here, and `docs/NEXT_PHASE.md` for the agreed next work item.

Note: the earlier phase-numbering discrepancy is resolved. Trajectory prediction is
**Phase 5**; 2.5D mapping, risk and adaptive resolution are Phases 6, 7 and 8. This file,
`docs/ROADMAP.md` and `services/system_service.py` all use those numbers. The phase
headings in `docs/PHASE_HISTORY.md` are a historical record and were left as written.

## Knowledge Base

Read the relevant documents in `docs/knowledge-base/` before changing a module. Read `docs/knowledge-base/20-constraints.md` before benchmarking or demo work. Record accepted architectural choices in `docs/decisions/architecture-decisions.md` and measured work in `docs/experiments/experiment-log.md`.
