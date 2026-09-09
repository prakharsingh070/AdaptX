# Roadmap

Phase status is tracked here and mirrored in `CLAUDE.md`. A phase is **Done** only when its
work is implemented, tested and documented — not when its interfaces exist.

The running backend is the authority on what exists: `GET /api/v1/system/status` reports
every subsystem's implementation status.

---

## Phase 1 — Project foundation · **Done**

Repository structure, packaging, configuration, structured logging, data contracts, module
interfaces, the FastAPI backend, LiDAR ingest with validation, the CARLA boundary,
WebSocket telemetry, the test suite and the Docker/development environment.

Delivered:

- `src/adaptx/` package with `config`, `core`, `models`, `perception`, `mapping`, `risk`,
  `tracking`, `prediction`, `carla`, `services`, `api`
- 7 HTTP endpoints + `/ws/telemetry`, documented in [`API.md`](API.md)
- 150 tests (unit + integration), Ruff, Ruff format and mypy all clean
- Docker image and compose file for the backend
- Honesty constraints enforced by code and covered by tests (see [`TESTING.md`](TESTING.md))

Not delivered, by design: any perception algorithm.

---

## Phase 2 — LiDAR processing · **2A done, 2B to do**

### Phase 2A — input and preprocessing · **Done**

Input validation, NaN/Inf removal, ROI filtering and range filtering, with per-stage
counts and a measured duration, behind `PointCloudPreprocessor`.

- Coordinate convention fixed and documented (ADR-009); range convention documented
  (ADR-011); raw vs validated frame types separated (ADR-010).
- Reachable through `POST /api/v1/lidar/frame` with `preprocess: true`; the default path
  is unchanged.
- Open3D was **not** added: every operation is a NumPy boolean mask, so the dependency
  would have bought nothing.

### Phase 2B — downsampling and segmentation · **To do**

- Voxel grid downsampling, ground segmentation, statistical outlier removal.
- Coordinate transforms between the `lidar`, `ego` and `world` frames.
- Reconsider Open3D here: voxelisation and normal estimation are where it would earn its
  place. Record the decision as an ADR either way.
- Reference: [`knowledge-base/04_lidar-knowledge.md`](knowledge-base/04_lidar-knowledge.md).
- Done when: the stages are configurable, measured on representative data, and
  `lidar_preprocessing` moves from `PARTIAL` toward `IMPLEMENTED`.

## Phase 3 — Object detection · **To do**

- Implement `perception.interfaces.ObjectDetector` producing `DetectedObject`.
- Prefer a geometric/clustering approach before considering an ML framework; adding one is
  an ADR-level decision.
- Done when: the `perception` component reports `READY` and detections reach the API.

## Phase 4 — Tracking · **To do**

- Implement `tracking.interfaces.ObjectTracker` producing `TrackedObject`.
- Document association, initialisation, occlusion handling and termination rules.
- Reference: [`knowledge-base/08_tracking.md`](knowledge-base/08_tracking.md).

## Phase 5 — 2.5D mapping · **To do**

- Implement `mapping.interfaces.AdaptiveMapper` in **two** variants: the fixed-resolution
  baseline (ADR-003) and the adaptive mapper, distinguished by `AdaptiveMap.is_adaptive`.
- Reference: [`knowledge-base/05_2.5d-mapping.md`](knowledge-base/05_2.5d-mapping.md).

## Phase 6 — Risk and uncertainty · **To do**

- Implement `risk.interfaces.RiskEngine` as the real ADAPT-X engine; keep
  `BaselineProximityRiskEngine` for comparison.
- Populate `RiskFactors` so resolution changes are attributable.
- Add the uncertainty engine: define representation, propagation and its effect on risk and
  resolution.
- References: [`knowledge-base/06_risk-engine.md`](knowledge-base/06_risk-engine.md),
  [`knowledge-base/07_uncertainty.md`](knowledge-base/07_uncertainty.md).

## Phase 7 — Adaptive resolution · **To do**

- Implement `mapping.interfaces.ResolutionController` consuming `ResolutionContext`.
- Include a documented stabilisation mechanism (hysteresis, smoothing or minimum dwell
  time) so resolution does not oscillate between frames.
- Reference: [`knowledge-base/10_adaptive-resolution.md`](knowledge-base/10_adaptive-resolution.md).

## Phase 8 — Prediction · **To do**

- Implement `prediction.interfaces.TrajectoryPredictor`; add conflict analysis against the
  ego trajectory and predictive refinement of resolution.
- Reference: [`knowledge-base/09_prediction.md`](knowledge-base/09_prediction.md).

## Phase 9 — CARLA · **To do**

- Complete `CarlaClient`: sensor attachment, ego-state extraction, actor spawning and
  cleanup (each currently raises an explicit "not implemented in Phase 1" error).
- Document CARLA version, map, synchronous mode, fixed timestep, sensor transforms and
  seeds.
- Reference: [`knowledge-base/11_carla.md`](knowledge-base/11_carla.md).

## Phase 10 — Scenario generation and event replay · **To do**

- Seeded, reproducible scenario configurations; the event record and replay path.
- References: [`knowledge-base/12_scenario-generation.md`](knowledge-base/12_scenario-generation.md),
  [`knowledge-base/14_event-replay.md`](knowledge-base/14_event-replay.md).

## Phase 11 — Benchmarking · **To do**

- Run identical scenarios through fixed-resolution and adaptive perception; record
  measured performance, workload and perception metrics into
  [`experiments/experiment-log.md`](experiments/experiment-log.md).
- Read [`knowledge-base/20-constraints.md`](knowledge-base/20-constraints.md) before any
  benchmarking or demo work.

## Phase 12 — Dashboard and final integration · **To do**

- Build the dashboard against the existing backend contracts. Perception logic must not
  live in the UI.
- Reference: [`knowledge-base/15_dashboard-ui.md`](knowledge-base/15_dashboard-ui.md).

---

## Standing rules across every phase

- Never fabricate a measurement, benchmark result or performance figure.
- Label simulation, replay and synthetic data everywhere it appears — API, logs, dashboard.
- Keep the fixed-resolution baseline comparable and alive.
- Record accepted architecture decisions in
  [`decisions/architecture-decisions.md`](decisions/architecture-decisions.md) and measured
  work in [`experiments/experiment-log.md`](experiments/experiment-log.md).
- When a module becomes real, update its row in `_declared_components()`
  (`src/adaptx/services/system_service.py`), remove its stream from `not_yet_available` in
  the telemetry payload, and update [`ARCHITECTURE.md`](ARCHITECTURE.md).
