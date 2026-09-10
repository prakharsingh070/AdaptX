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

## Phase 5 — Trajectory prediction · **Done (constant-velocity baseline)**

> **Renumbered.** Prediction was Phase 8 and 2.5D mapping was Phase 5 until Phase 5 was
> implemented. Prediction moved to 5 because it is what the risk engine needs next;
> mapping, risk and adaptive resolution each shifted one later. `CLAUDE.md` and the
> `phase` field on every component in `services/system_service.py` agree with the numbers
> used here.

Deterministic extrapolation of the Phase 4 measured velocity, built to be replaced.

- `ConstantVelocityPredictor` — `position + velocity * (age_s + t)` over a configurable
  horizon, sampled `t+0` to the horizon inclusive (ADR-026). Defaults: 3.0 s at 0.25 s,
  so 13 points per track.
- `age_s` is the measured staleness of a coasting track's last observation, so one
  formula covers fresh and coasting tracks without special-casing.
- Heuristic uncertainty growing linearly with extrapolation time. **Not** a calibrated
  sigma, probability or confidence interval, and labelled as such everywhere it appears.
- `velocity is None` produces **no trajectory** and a recorded skip reason (ADR-027); a
  *measured* standstill produces a stationary one. Over-speed velocities are rejected,
  never clipped.
- `PredictionResult` accounts for every track: `considered == predicted + skipped`.
- State-free with respect to perception: `PredictionService` holds counters only.
- `POST /api/v1/lidar/predict`, `GET /api/v1/prediction/status`; prediction summary on
  `/ws/telemetry`; prediction benchmark via `--predict`.
- No ML framework, no SciPy, no new dependencies.

### Phase 5B — deferred prediction work · **To do**

- Constant-acceleration, Kalman or IMM motion models, once there is data to fit their
  noise parameters against rather than guess them.
- Class-conditioned motion, if measurement ever justifies the split points.
- Map- and lane-conditioned prediction, which needs the 2.5D map (Phase 6).
- Interaction-aware prediction between objects.
- Labelled trajectories, the precondition for measuring prediction accuracy at all, and
  for calibrating the uncertainty model into a real one.
- Reference: [`knowledge-base/09_prediction.md`](knowledge-base/09_prediction.md).

## Phase 6 — 2.5D mapping · **Done (fixed-resolution baseline)**

A deterministic, frame-local spatial representation, built to be replaced by an adaptive
one.

- `FixedResolutionMapper` — bins a processed frame into a bounded, uniform-resolution XY
  grid with per-cell point counts and min/max/mean height (ADR-028).
- Half-open cells anchored at the map's lower corner; a point on a max edge is out of
  bounds. Quantisation reuses the Phase 2B overflow-guarded quantiser.
- **Binary** occupancy: a cell is occupied iff it holds a point. An unobserved cell reports
  **null** height, never zero (ADR-031).
- Every input point accounted for: `input == mapped + out_of_bounds`, model-enforced.
- **Frame-local** (ADR-030): a call builds a whole map from one frame and nothing
  accumulates. Not SLAM, not a persistent world map.
- `ResolutionDecision` separates resolution *policy* from the mapper (ADR-029). The mapper
  is never handed tracks, trajectories, risk or uncertainty, so a risk-aware choice is
  structurally impossible here.
- Dense NumPy arrays, not one object per cell; `to_adaptive_map()` projects occupied cells
  into the pre-existing `AdaptiveMap` contract.
- `POST /api/v1/lidar/map`, extended `GET /api/v1/map/status`; mapping summary on
  `/ws/telemetry`; mapping benchmark via `--map` with a resolution sweep.
- No ML framework, no SciPy, no new dependencies.

### Phase 6B — deferred mapping work · **To do**

- **The adaptive mapper itself.** Phase 6 shipped only the baseline half of ADR-003; the
  variant that allocates resolution by risk needs Phase 7 and Phase 8 first.
- Temporal occupancy fusion, which needs ego-motion compensation and a decay policy — see
  ADR-030 for why neither exists yet.
- Distinguishing *unobserved* from *free*: occlusion is not tracked, so a cell hidden behind
  a vehicle is reported the same as empty space. This matters for safety.
- Probabilistic occupancy, which needs a sensor model (ADR-031).
- Reference: [`knowledge-base/05_2.5d-mapping.md`](knowledge-base/05_2.5d-mapping.md).

## Phase 7 — Risk and uncertainty · **Done (heuristic baseline)**

A deterministic, explainable object-level risk and uncertainty layer, built to be replaced.

- `HeuristicRiskEngine` — three normalised factors (proximity, rate of approach, predicted
  approach) combined as a weighted mean **over the factors actually available** (ADR-032).
- A factor that cannot be computed is **dropped and the weights renormalise**, never scored
  zero. Unknown velocity is not a standstill.
- `UNKNOWN` with a **null score** when nothing can be computed or the track is lost, rather
  than a fabricated number. `RiskLevel.UNKNOWN` added additively; `CRITICAL` retained.
- **Uncertainty is reported beside risk, never folded into it** (ADR-033), with the
  contributing reasons kept visible.
- Map context never lowers risk: an empty cell is *unobserved*, not free (ADR-034).
- Scene aggregate is a **maximum, never a mean** — one critical object cannot vanish behind
  ten quiet ones (ADR-035).
- Explanations are generated from the computed factors, never free-form.
- `BaselineProximityRiskEngine` retained **unchanged** as the comparison reference.
- `POST /api/v1/lidar/risk`, extended `GET /api/v1/risk/status`; risk summary on
  `/ws/telemetry`; risk benchmark via `--risk`.
- No ML framework, no SciPy, no new dependencies.

**The score is not a probability of collision.** It is not calibrated, has never been
validated against labelled risk data — none exists — and its thresholds are baseline
engineering values, not safety-certified limits.

### Phase 7B — deferred risk work · **To do**

- Time-to-collision, deliberately excluded: over a constant-velocity extrapolation with
  heuristic uncertainty it would be a precise-looking number resting on two approximations.
- Trajectory-map intersection and occlusion modelling, which need a visibility model the
  project does not have (ADR-034).
- A spatial risk **field** (`RiskCell`): Phase 7 is object-level only, so `risk_field` stays
  listed as unavailable in telemetry.
- Populating `AdaptiveMapCell.risk_score` / `uncertainty`, which needs that per-cell
  formulation.
- Ego planned path and object interaction.
- Labelled risk data, the precondition for measuring whether any of this is *right*.
- References: [`knowledge-base/06_risk-engine.md`](knowledge-base/06_risk-engine.md),
  [`knowledge-base/07_uncertainty.md`](knowledge-base/07_uncertainty.md).

## Phase 8 — Adaptive resolution · **To do**

- **This is where the ADAPT-X claim gets tested.** Phase 6 built the fixed-resolution
  mapper; Phase 7 produced risk and uncertainty. Phase 8 connects them and must show that
  risk-aware allocation beats uniform allocation on measured workload.
- Implement `mapping.interfaces.ResolutionController` consuming `ResolutionContext`, fed
  from Phase 7 `RiskAssessment` values, and producing a `ResolutionDecision` the existing
  mapper already applies unchanged (ADR-029, ADR-036).
- Consume **uncertainty as well as risk**: a poorly observed region may deserve finer
  perception even when its computed risk is low. That is the whole reason the two are kept
  separate (ADR-033).
- Include a documented stabilisation mechanism (hysteresis, smoothing or minimum dwell
  time) so resolution does not oscillate between frames.
- Reference: [`knowledge-base/10_adaptive-resolution.md`](knowledge-base/10_adaptive-resolution.md).

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
