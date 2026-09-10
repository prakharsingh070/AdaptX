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

## Phase 2 — LiDAR processing · **Done (2A, 2B, 2C)**

### Phase 2A — input and preprocessing · **Done**

Input validation, NaN/Inf removal, ROI filtering and range filtering, with per-stage
counts and a measured duration, behind `LiDARProcessingPipeline`.

- Coordinate convention fixed and documented (ADR-009); range convention documented
  (ADR-011); raw vs validated frame types separated (ADR-010).
- Reachable through `POST /api/v1/lidar/frame` with `preprocess: true`; the default path
  is unchanged.
- Open3D was **not** added: every operation is a NumPy boolean mask, so the dependency
  would have bought nothing.

### Phase 2B — downsampling and segmentation · **Done**

Voxel downsampling, baseline ground segmentation and baseline noise filtering, each in its
own module behind the `LiDARProcessingPipeline` orchestrator, each opt-in (ADR-012).

- Voxel keeps the real point nearest each voxel centroid rather than synthesising a
  centroid (ADR-015).
- Ground uses a per-cell lowest point plus tolerance, which handles slope and needs no
  sensor mount height (ADR-016).
- Noise counts neighbours over a 3×3×3 cell block — an honest approximation of radius
  outlier removal that avoids a SciPy dependency (ADR-014).
- **No coordinate transform was implemented**: it was shown per stage to be unnecessary
  (ADR-013).
- Open3D was reconsidered and again **not** added: voxelisation is `np.unique` over integer
  cell keys, and the other two stages are boolean masks.

### Phase 2C — integration and benchmarking · **Done**

- `LiDARProcessingPipeline` orchestrates the seven stages; each algorithm stays in its own
  independently testable module (ADR-017).
- Per-stage measured timing, with unattributed time reported as `overhead_ms` rather than
  inflating a stage.
- Every result carries a `PipelineConfiguration` snapshot, so records are self-describing.
- `adaptx.benchmark`: deterministic synthetic datasets, a fixed-resolution baseline profile
  distinct from ADR-003 (ADR-018), and a runner recording timing, throughput and memory
  (ADR-019).
- Measured results in [`experiments/experiment-log.md`](experiments/experiment-log.md);
  method in [`BENCHMARKING.md`](BENCHMARKING.md).

### Phase 2D — deferred LiDAR work · **To do**

Not required by Phase 2 and deliberately left undone:

- Clustering of non-ground points, the natural input to Phase 3 detection.
- Exact radius or statistical outlier removal, if the grid approximation proves insufficient
  against real data — that is the point to weigh SciPy on evidence.
- A coordinate transform stage once a tilted or multi-sensor mount, sensor fusion, or CARLA
  ingestion requires one (ADR-013 names those triggers).
- A recorded dataset to replace the synthetic generator, which would make accuracy
  measurable for the first time.
- Reference: [`knowledge-base/04_lidar-knowledge.md`](knowledge-base/04_lidar-knowledge.md).

## Phase 3 — Object detection · **Done (geometric baseline)**

A deterministic clustering detector on the pipeline's non-ground output, built to be
replaced by an ML detector without touching the API or service layer.

- `GridConnectedComponentClusterer` — grid connectivity rather than DBSCAN or a KD-tree,
  so no new dependency (ADR-020).
- `GeometricClassifier` — dimension bands, `UNKNOWN` on ambiguity, confidence documented as
  a geometric fit score rather than a probability (ADR-021).
- `GeometricObjectDetector` — geometry, size filtering, and a `DetectionResult` carrying
  rejected candidates and measured timings (ADR-022).
- `POST /api/v1/lidar/detect`; detection benchmark via `--detect`.
- No ML framework added; no new dependencies at all.

### Phase 3B — deferred detection work · **To do**

- Oriented bounding boxes. Axis-aligned boxes make a diagonal vehicle measure larger than
  it is, which is the main source of `UNKNOWN` classifications in a real scene.
- Exact Euclidean clustering, if grid connectivity proves too coarse — that is where SciPy
  would be weighed on evidence.
- An ML detector, once a labelled dataset exists. Until then the geometric baseline is the
  reference an ML detector would have to beat.
- A labelled dataset, which is the precondition for measuring detection accuracy at all.

## Phase 4 — Tracking · **Done (geometric baseline)**

Deterministic multi-object tracking on the Phase 3 detections, built to be replaced.

- `GeometricObjectTracker` — gated greedy nearest-neighbour association with optional class
  and size compatibility (ADR-024).
- Velocity measured from frame timestamps, null until two observations (ADR-023); raw and
  smoothed values both reported.
- Lifecycle over the existing four-state enum: TENTATIVE → CONFIRMED → COASTING → LOST.
- State owned by `TrackingService` on the application context, resettable through the API
  (ADR-025).
- `POST /api/v1/lidar/track`, `POST /api/v1/tracking/reset`, `GET /api/v1/tracking/status`;
  tracking benchmark via `--track`.
- No ML framework, no SciPy, no new dependencies.

### Phase 4B — deferred tracking work · **To do**

- Re-identification, so an object returning after occlusion regains its old id. Needs
  appearance features the geometric detector does not produce.
- A motion model (Kalman or similar) for smoother state and better gating during crossings.
- Spatial bucketing for association, if object counts ever reach the thousands where the
  `O(T x D)` term dominates.
- Labelled sequences, the precondition for measuring tracking correctness at all.
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
