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

## Phase 8 — Adaptive resolution · **Done (heuristic baseline)**

**This is where the ADAPT-X claim finally got tested**, and the answer is more interesting
than a yes.

- `HeuristicResolutionController` — the map extent is partitioned into fixed-size regions
  and each is given its own cell size, so one map genuinely holds several resolutions
  (ADR-037). Resolution is decided per region, never per point.
- `TiledAdaptiveMapper` — builds one dense sub-grid per region. Regions partition the
  extent exactly: half-open on their upper edges, clipped at the map bounds, so a point
  lands in exactly one cell and `input == mapped + out_of_bounds` still holds.
- A **detail priority** combines risk, uncertainty, predicted-motion relevance, object
  density, proximity and measured motion as a weighted mean **over the factors actually
  available** (ADR-038). A factor that cannot be computed is dropped and the weights
  renormalise — the ADR-032 rule, one phase later.
- **Unknown is not low.** `risk_score is None` drops the risk factor *and* floors the
  region level. Coercing it to zero would hand the coarsest representation to the objects
  the system understands least.
- **Uncertainty is an independent input**, never summed into risk: a quiet, badly observed
  region can earn detail on uncertainty alone. That is the ADR-033 payoff.
- Resolution is stabilised by an **asymmetric hysteresis margin plus a minimum dwell time**
  (ADR-039): refinement is immediate, coarsening must be earned twice over. The controller
  is the only stateful component in the adaptive path.
- Region and cell budgets coarsen the **lowest-priority** regions first, and every demotion
  is reported rather than silently absorbed (ADR-040).
- `POST /api/v1/lidar/adaptive-map`, `POST /api/v1/map/adaptive/reset`, extended
  `GET /api/v1/map/status`; adaptive summary on `/ws/telemetry`; benchmark via `--adaptive`.
- The Phase 6 fixed-resolution mapper is **retained unchanged** as the baseline (ADR-003).
- No ML framework, no SciPy, no new dependencies.

**The detail priority is not a probability of collision** and is not a safety margin. It is
an engineering prioritisation score: never calibrated, never validated against labelled
data, because none exists. It orders regions; it measures nothing physical.

**Measured (Experiment 007), including the unflattering results:** adaptive uses 0.06–0.30x
the cells of a uniform 0.25 m map and 0.25–0.50x of a 0.5 m one, but **more** cells than a
1.0 m map in every scene containing an object — the base level *is* 1.0 m, so against that
baseline the policy can only add cells. Adaptive mapping is also **slower in wall-clock
time than the fixed mapper in every scene**, even where it allocates a quarter of the
cells. Cost scales with **region count, not cell count** (~120 µs per region); the lever is
`tile_size_m`, not the resolution vocabulary.

### Phase 8B — deferred adaptive-resolution work · **To do**

- A hierarchical or quadtree representation, which is where the per-region overhead measured
  in Experiment 007 would be attacked. ADR-037 records why it was not the first choice.
- A per-cell risk field, so `RiskCell` and `AdaptiveMapCell.risk_score` can finally be
  populated. Phase 7 is object-level only.
- An ego planned path: `ResolutionContext.in_ego_path` exists and is never set true,
  because no planner exists to set it.
- Predicted **risk** as a distinct input. `ResolutionContext.predicted_risk_score` stays
  null: Phase 7 folds predicted approach into its score rather than publishing a second one.
- Tuning the weights and thresholds against outcomes, which needs outcomes to be recorded.
- Reference: [`knowledge-base/10_adaptive-resolution.md`](knowledge-base/10_adaptive-resolution.md).

## Phase 9 — CARLA · **Done (simulation boundary)**

CARLA became an upstream **data source**, not a second perception stack.

- `CarlaSimulationSession` — the lifecycle: connect, apply deterministic world settings,
  spawn the ego, attach the LiDAR, tick, and close. Every actor it spawns is destroyed on
  close **including after a failed setup**, and world settings are restored, so a crashed
  run cannot leave a server wedged in synchronous mode (ADR-042).
- `carla/conversion.py` — CARLA's left-handed frame (+y right) becomes ADAPT-X's
  right-handed frame (+y left) **exactly once**. The module imports no simulator, which is
  what makes the one silent failure mode in this phase — a dropped sign flip mirrors the
  world without raising — testable on a machine with no CARLA (ADR-043).
- **Simulation time is authoritative** (ADR-044). Synchronous mode, fixed timestep, explicit
  ticks; never a wall clock, never a sleep. Phase 4 velocity, Phase 5 intervals and Phase 8
  dwell counting all depend on it, and all three degrade silently without it.
- Frames are labelled `source=simulation` and enter the **existing** Phase 2 ingest path.
  Phases 1–8 were not modified to accommodate CARLA, which is the evidence the boundary is
  in the right place.
- **Ground truth is a separate path** (ADR-045). `GroundTruthFrame` shares the LiDAR frame's
  id and timestamp so the two join later, and reaches no perception stage. A test runs the
  chain with and without reading it and asserts identical output.
- One hard-coded smoke scenario, `carla/smoke.py` — **replaced in Phase 10** by the
  `vehicle_approach` catalogue scenario and deleted.
- CARLA remains **optional**: the backend starts, all 17 endpoints respond and the suite
  passes with the package absent.

**Live-validated on 2026-09-11** against CARLA 0.9.16 on Town10HD_Opt: `pytest -m carla`
7/7 pass (Experiment 010). Three real-server behaviours the stand-in could not show were
found and fixed with fake-backed regressions: a spawned actor reports the origin until the
first tick (ADR-049), an occupied spawn point is walked past (the point 0 refusal turned
out in Experiment 011 to be a stale actor from a killed run; `ego_spawn_index` can pin a
point), and the package exposes no `__version__` (the server's own is recorded). Experiment
011 also found `carla.seed` was applied to nothing and now seeds the LiDAR. Requires a Python 3.12
environment, because no `carla` wheel exists for 3.13. Nothing in this phase is a CARLA
accuracy claim.

**Phase 9 is not an accuracy phase.** What it delivers is the *precondition* for one:
repeatable simulation, deterministic time, and labelled ground truth. Measuring accuracy
against that ground truth is Phase 11.

### Phase 9B — deferred CARLA work · **To do**

- Pitch and roll conversion, once a tilted sensor mount needs them (ADR-043).
- Traffic manager and populated scenes, which need a seeded, reproducible configuration —
  that is Phase 10, not this one.
- Camera or other sensors; only LiDAR is attached.
- A recorded dataset, so perception can be exercised without a live server.
- Reference: [`knowledge-base/11_carla.md`](knowledge-base/11_carla.md).

## Phase 10 — Scenario generation · **Done (scenario framework)** · replay **deferred**

From "CARLA can provide one hard-coded scripted simulation" to "ADAPT-X can describe, seed,
run and reproduce controlled scenarios."

- `ScenarioDefinition` — **data, not code** (ADR-046). Actors, ego-relative placement,
  timed constant-velocity motion segments, duration, timestep, an explicit seed. Validated at
  construction; survives a JSON round-trip; the definition models import nothing from the
  CARLA boundary.
- `resolve(seed)` — every randomised value drawn once from `random.Random(seed)` into a
  `ResolvedScenario` recorded on the result. The only randomised element is placement
  jitter; the catalogue uses none. The seed is reported regardless.
- Scripted motion (ADR-047) — timed segments summed in closed form. The runner **places**
  each actor at its expected pose every frame; nothing integrates physics. So every frame
  carries the *commanded* pose beside the simulator's *reported* one.
- `ScenarioRunner` (ADR-048) — drives the Phase 9 session through a protocol extracted from
  it; single-use, so runs share nothing; `CREATED → VALIDATING → READY → RUNNING →
  COMPLETED | FAILED`; cleanup in a `finally` on every path. A malformed definition raises
  before any simulator contact; a run-time failure returns `FAILED` with the frames stepped.
- Ground truth recorded beside every frame, joined by simulator frame id, fed to **no**
  pipeline stage (ADR-045). The processor callback receives the sensor frame and nothing
  else, by signature.
- Four catalogue scenarios: `stationary_vehicle`, `vehicle_approach` (the Phase 9 smoke
  scene as a definition), `pedestrian_crossing` (timed start and stop), `cyclist_crossing`
  (two classes, diagonal motion, a close pass).
- `python -m adaptx.scenarios list | run <id> [--seed] [--json]`. **No HTTP endpoint** starts
  a scenario (ADR-042).
- `carla/smoke.py` and its two orphaned settings **deleted**; its seven tests retargeted onto
  the framework with intent preserved.
- No new dependencies.

**The run result is raw evidence, not a conclusion.** Frame identities, scripted poses,
ground truth and stage counts - and no accuracy, precision, error or match figure anywhere.
Computing one is Phase 11, and a test asserts the result contracts carry no such field.

**Live-validated on 2026-09-11:** all four catalogue scenarios COMPLETED against a real
CARLA 0.9.16 server, contiguous frame ids, dt exactly 0.05 s, ~27,000 points per frame,
~92 ms of ADAPT-X pipeline per frame on this machine, zero actors left behind
(Experiment 010). All five catalogue blueprints exist on that server. Experiment 009
measures orchestration cost alone (~35 µs to resolve, ~7 µs per actor per frame).

### Deferred from Phase 10

- **Event replay.** The `ScenarioRunResult` is the recording a replay would need, but no
  playback path exists and `DataSource.REPLAY` is still produced by nothing. The open design
  question from the handoff - re-run the simulation or re-play a recording - is still open.
- **Ego motion.** `EgoDefinition.stationary` is validated `True`; every catalogue scenario
  moves the targets instead.
- Traffic, weather, time of day, the Traffic Manager, and any scene with more than a handful
  of actors.
- Moving the ground-truth contracts out of `adaptx.carla`, which a second simulator would
  trigger.
- References: [`knowledge-base/12_scenario-generation.md`](knowledge-base/12_scenario-generation.md),
  [`knowledge-base/14_event-replay.md`](knowledge-base/14_event-replay.md).

## Phase 11 — Evaluation and benchmarking · **Done**

- `adaptx.evaluation`: an offline evaluation layer that reads a recorded `ScenarioRunResult`
  and produces an `EvaluationReport` — detection and tracking against ground truth at
  several gates, ADE/FDE, risk against proximity events, map workload, adaptive resolution
  paired against the fixed map within one run, resource; `python -m adaptx.evaluation
  evaluate|compare|run`. Architecture and metric definitions in
  [`EVALUATION.md`](EVALUATION.md); ADR-050/051/052/053.
- The Phase 10 record gained the pipeline's result contracts per frame and the sensor
  configuration, additively; it still carries no metric.
- Ground truth reaches `adaptx.evaluation` and nothing else — asserted in a subprocess and
  by source inspection. No production package imports the evaluation layer.
- **Measured (Experiment 011, simulation evidence only):** the baselines lost, as the
  handoff predicted. Vehicle detection recall 0.23–0.61 at a 2 m gate with a consistent
  1.5–1.7 m planar offset and no vehicle ever classified as one; 1–3 identity switches per
  moving actor; ADE 1.5–3.4 m mean; risk concordance with proximity 0.80–0.86 for moving
  actors; the adaptive map at 0.48–0.56 of the fixed map's cells and 5× its build time,
  with finer cells under the perceived actor than elsewhere. The placement defect that
  confounded the pedestrian scenario (a placed walker falling through the road) was fixed
  in ADR-054 and everything re-measured (Experiment 012): a static scene is now
  bit-repeatable; under the process defaults (ground segmentation off) a parked car and a
  pedestrian are never detected, and with ground segmentation on both are seen on every
  frame — the default is an open Phase 2/3 decision.
- **Not evaluated:** mapping occupancy accuracy (no honest reference), precision over
  unlabelled tracks, peak memory, anything real-world. Nothing was tuned.

## Phase 12 — Dashboard and final integration · **To do (next)**

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
