# ADAPT-X Architecture

This document describes the **actual state of the code**. Domain knowledge, requirements
and the conceptual design of each layer live in [`knowledge-base/`](knowledge-base), which
this document does not restate or override.

Anything marked *Planned* below does not exist in the repository beyond a data contract or
an abstract interface.

---

## 1. Implementation status

| Layer | Status | What exists today |
|---|---|---|
| Configuration | **Implemented** | Typed, environment-driven settings with validation (`config/settings.py`) |
| Logging | **Implemented** | Structured text/JSON logging with contextual fields (`core/logging.py`) |
| Data contracts | **Implemented** | Point cloud, vehicle, object, track, trajectory, map, risk, system models (`models/`) |
| API | **Implemented** | 17 HTTP endpoints + OpenAPI (`api/`) |
| Telemetry (WebSocket) | **Implemented** | `/ws/telemetry`, carrying system status, measured metrics and detection/tracking/prediction/mapping/risk/adaptive **summaries**. No per-frame geometry, no trajectory points, no map cells, no tiles |
| Metrics | **Implemented** | Measured ingest FPS/latency and process CPU/memory; unmeasured values are `null` |
| LiDAR ingest | **Partial** | Structural validation, point count, bounds, provenance |
| LiDAR pipeline | **Partial** | Phase 2A: input validation, NaN/Inf removal, ROI and range filtering. Phase 2B (opt-in): voxel downsampling, baseline ground segmentation, baseline noise filtering. Phase 2C: `LiDARProcessingPipeline` orchestration, per-stage timing, configuration snapshot. **No** clustering, no coordinate transforms, no exact radius/statistical outlier removal |
| Benchmarking (pipeline, detection, tracking, prediction, mapping, risk, adaptive) | **Implemented** | Deterministic synthetic datasets, fixed-resolution baseline profile, measured timing/throughput/memory (`adaptx.benchmark`, flags `--detect` / `--track` / `--predict` / `--map` / `--risk` / `--adaptive`). Scope is **speed and workload only** - not the Phase 11 ADAPT-X evaluation, and never perception accuracy |
| Risk | **Partial** | Phase 7: deterministic heuristic object-level risk and uncertainty (`risk/heuristic.py`) - proximity, rate of approach, predicted approach, with uncertainty reported separately. **Not** a probability of collision, not calibrated, never validated - no labelled risk data exists. **No** time-to-collision, no trajectory-map intersection, no spatial risk field. The proximity-only baseline is retained for comparison |
| CARLA | **Partial** | Phase 9: deterministic simulation boundary - synchronous mode with a fixed timestep, ego spawn, LiDAR attach, ticking, and cleanup that survives a failed setup. One coordinate conversion at the boundary (ADR-043); simulation-authoritative time (ADR-044); ground truth on a separate path that never reaches perception (ADR-045). Optional: the backend and test suite run without it. **No live run has been executed here** - the package is not installed, so the live smoke test skips |
| Object detection | **Partial** | Phase 3: grid clustering, size filtering and baseline classification by dimension bands (`perception/{clustering,classification,detector}.py`). **No** trained model, no oriented boxes, no velocity, no camera fusion, no semantic segmentation |
| Tracking | **Partial** | Phase 4: gated nearest-neighbour association, measured velocity, track lifecycle, stateful service (`tracking/`). **No** learned motion model, no appearance features, no re-identification |
| 2.5D mapping | **Partial** | Phase 6: deterministic frame-local fixed-resolution mapper - bounded dense grid, binary occupancy, per-cell height statistics (`mapping/grid_mapper.py`). Retained unchanged as the baseline (ADR-003). **No** temporal fusion, no probabilistic occupancy, no occlusion, no SLAM, no localisation. Correctness unmeasured - no labelled reference map exists |
| Adaptive resolution | **Partial** | Phase 8: deterministic heuristic controller allocating a cell size per **region**, and a tiled mapper applying it (`mapping/{controller,adaptive_mapper,tiles}.py`). Detail priority combines risk, uncertainty, predicted-motion relevance, density, proximity and motion over the factors available. Stabilised by asymmetric hysteresis plus a minimum dwell time. **Not** a probability, not calibrated, never validated. **No** learned policy, no ego planned path, no per-cell risk field. Whether the allocation is *appropriate* is unmeasured |
| Trajectory prediction | **Partial** | Phase 5: deterministic constant-velocity baseline with heuristic uncertainty (`prediction/constant_velocity.py`). **No** acceleration model, no Kalman filter, no learned model, no map or lane conditioning, no interaction between objects. Accuracy unmeasured - no labelled trajectories exist |
| Uncertainty engine | **Partial** | Phase 7: a heuristic per-object uncertainty scalar with its contributing reasons, reported beside risk rather than folded into it (ADR-033). Not a variance, not calibrated. `AdaptiveMapCell.uncertainty` is still unpopulated - that needs a per-cell formulation |
| Fixed-resolution baseline | **Implemented** | Both halves of ADR-003 now exist: `FixedResolutionMapper` (`is_adaptive: false`) and `TiledAdaptiveMapper` (`is_adaptive: true`). Experiment 007 is the first comparison over identical input, and it is **not a clean win** - see the entry before quoting it |
| Scenario framework | **Partial** | Phase 10: `ScenarioDefinition` as data (ADR-046), timed constant-velocity motion placed rather than simulated (ADR-047), a single-use runner driving the Phase 9 boundary through an extracted protocol (ADR-048). Ground truth recorded per frame, never fed to perception. Four catalogue scenarios. Result is raw evidence with **no accuracy figure**. **No live run executed here.** Event replay deferred; `DataSource.REPLAY` still unproduced |
| Event replay, scenario benchmarking | *Planned* | Not started. Replay was deferred from Phase 10 with the design question open |
| Dashboard | *Planned* | Not started (`dashboard/README.md`) |

The running backend reports this itself at `GET /api/v1/system/status`. Each component
carries a **readiness** (`READY` / `NOT_READY`) and an **implementation status**
(`IMPLEMENTED` / `PARTIAL` / `PLANNED` / `MOCK`), so a module that does not exist can never
look like one that does.

---

## 2. Layering

The layer model from [`knowledge-base/03_system-architecture.md`](knowledge-base/03_system-architecture.md)
is unchanged. Phase 1 implements the edges of that pipeline (input and output) and the
contracts between the layers in the middle:

```
                        IMPLEMENTED                    PLANNED
                   ┌───────────────────┐   ┌──────────────────────────────────┐
  HTTP / WS  ──▶   │  api/             │   │                                  │
                   │   routes          │   │  perception  → tracking          │
                   │   websocket       │   │       ↓            ↓             │
                   ├───────────────────┤   │  mapping  ←   prediction         │
                   │  services/        │   │       ↑            ↓             │
                   │   lidar_ingest    │   │       └──────  risk              │
                   │   metrics         │   │              (baseline only)     │
                   │   carla           │   └──────────────────────────────────┘
                   │   system status   │
                   ├───────────────────┤
                   │  models/  (contracts shared by every layer)              │
                   └──────────────────────────────────────────────────────────┘
```

**Dependency rule.** `api` → `services` → `perception` / `risk` / `carla` → `models`.
Nothing in `models` imports a service or a route. Perception modules never import
dashboard or API code (knowledge-base boundary rule).

---

## 3. Package map

| Package | Responsibility |
|---|---|
| `adaptx.config` | Typed settings loaded from the environment; validation of ranges and orderings |
| `adaptx.core` | `logging`, `exceptions`, `lifecycle` (the service graph and startup/shutdown) |
| `adaptx.models` | All data contracts. State and validation only — no business logic |
| `adaptx.perception` | `LiDARProcessor` / `ObjectDetector` contracts; `FrameValidationProcessor` (Phase 1); `LiDARProcessingPipeline` orchestrator plus `VoxelDownsampler`, `GroundSegmenter`, `NoiseFilter` and the shared `grid` quantiser |
| `adaptx.benchmark` | Synthetic datasets, the fixed-resolution baseline profile, pipeline, detection, tracking and prediction runners, and their record contracts |
| `adaptx.mapping` | `AdaptiveMapper` / `ResolutionController` contracts; `FixedResolutionMapper` |
| `adaptx.risk` | `RiskEngine` contract; `HeuristicRiskEngine`; `BaselineProximityRiskEngine` |
| `adaptx.tracking` | `ObjectTracker` contract; `GeometricObjectTracker` and the association algorithm |
| `adaptx.prediction` | `TrajectoryPredictor` contract; `ConstantVelocityPredictor` |
| `adaptx.carla` | `CarlaSimulatorClient` contract, real client, mock, simulator-local models |
| `adaptx.services` | Ingest, metrics, CARLA, system-status, tracking, prediction, mapping and risk services |
| `adaptx.api` | Routes, HTTP schemas, WebSocket telemetry, dependency wiring |

---

## 4. Data contracts

Defined in `adaptx/models/`, re-exported from `adaptx.models`.

| Model | Purpose |
|---|---|
| `BasePointCloudFrame` | Shared frame metadata and structural validation (shape, dtype, column layout) |
| `RawPointCloudFrame` | A frame as received; **may** contain NaN/Inf (a sensor reports a non-return that way) |
| `PointCloudFrame` | A frame whose coordinates are all finite - the contract every downstream module consumes |
| `ProcessingMetrics` / `StageMetrics` / `PointCloudProcessingResult` | Per-stage counts and measured durations for one processed frame, plus the separated ground frame |
| `PipelineConfiguration` | The effective settings that produced a result, so a record is self-describing |
| `BenchmarkResult` / `BenchmarkReport` / `TimingSummary` | Measured benchmark records with dataset, seed, configuration and environment |
| `PointCloudSummary` / `PointCloudBounds` | JSON-safe metadata and axis-aligned bounds |
| `VehicleState` | Ego position, velocity, acceleration, heading, dimensions |
| `DetectedObject` | Per-frame detection: class, geometry, distance, point count, fit score. Identity is frame-local |
| `DetectionResult` / `RejectedCluster` / `DetectionConfiguration` | What one detection pass found, what it rejected and why, with measured timings |
| `TrackedObject` | Persistent track: status, measured kinematics (null until observed), geometry, age, hits |
| `TrackingResult` / `TrackingConfiguration` | What one update matched, created and retired, with measured timings |
| `PredictedTrajectory` / `TrajectoryPoint` | Predicted future path, ordered and horizon-bounded. Positions are extrapolations, never measurements |
| `PredictionResult` / `SkippedTrack` / `PredictionConfiguration` | What one prediction pass produced, which tracks it declined and why, with measured timings |
| `AdaptiveMapCell` / `AdaptiveMap` | 2.5D cell (occupancy, height, resolution, risk, uncertainty) and snapshot. Populated by projecting occupied cells out of a `SpatialMap` |
| `SpatialMap` / `MapBounds` / `MapAccounting` / `MappingConfiguration` | The Phase 6 grid itself: NumPy arrays of point count and min/max/mean height, plus bounds and full point accounting |
| `ResolutionDecision` / `ResolutionSource` | The resolution in force and where it came from. The **output** of a resolution decision; `ResolutionContext` holds its inputs |
| `ResolutionContext` | Inputs to the future resolution decision |
| `RiskCell` / `ObjectRisk` / `RiskField` / `RiskFactors` | Normalised risk, attribution and spatial field. `RiskCell` is still unpopulated - Phase 7 is object-level only |
| `RiskAssessment` / `RiskAssessmentResult` | Phase 7 per-object risk: level, score (**null when UNKNOWN**), distance, closing speed, trajectory relevance, map context, uncertainty breakdown, computed factors and a generated explanation |
| `UncertaintyBreakdown` / `TrajectoryRelevance` / `MapContext` | Heuristic uncertainty with visible reasons; closest predicted approach; what the map recorded at the object's cell |
| `SystemStatus` / `ComponentStatus` / `SystemMetrics` | Backend state and measured metrics |

Shared rules enforced by the base classes in `models/common.py`:

- `schema_version` on every public model.
- Timestamps are timezone-aware and normalised to UTC; naive datetimes are rejected.
- `coordinate_frame` on every spatial model (`lidar` / `ego` / `world` / `map`).
- `source` (`live_sensor` / `simulation` / `replay` / `synthetic_test` / `unavailable`) on
  everything that could originate anywhere but a sensor.
- Units are metres, m/s, m/s², radians and seconds unless the field name says otherwise.
- Unknown fields are rejected (`extra="forbid"`), so a schema drift fails loudly.

---

## 5. Provenance and honesty rules in the code

These are constraints from [`knowledge-base/20-constraints.md`](knowledge-base/20-constraints.md)
enforced by the implementation, not just by convention:

1. **`source` is mandatory on frame submission.** `POST /api/v1/lidar/frame` has no default
   for `source`; a caller must declare provenance.
2. **LiDAR status derives from provenance.** Frames labelled simulation, replay or
   synthetic make the channel report `SIMULATED`, never `CONNECTED`.
3. **Unmeasured metrics are `null`.** `SystemMetrics.unavailable` names each missing metric
   and why. GPU utilisation is always `null` in Phase 1.
4. **The mock is never a fallback.** `MockCarlaSimulatorClient` is selected only by
   `ADAPTX_CARLA__USE_MOCK=true`, logs a warning, and is flagged as `is_mock` in the API.
   A failed real connection never silently falls back to it.
5. **Telemetry declares what it does not have.** `/ws/telemetry` lists absent streams in
   `not_yet_available` rather than sending empty or invented ones.
6. **The risk baseline is labelled.** `RiskField.is_baseline` and `/api/v1/risk/status`
   report the modelled and unmodelled factors explicitly.

---

## 6. Request lifecycle

```
uvicorn → create_app()
            ├─ build_context()      construct settings + services (no I/O)
            └─ lifespan startup()   configure logging, optionally connect CARLA
                                    attach context + ConnectionManager to app.state

HTTP request → route → Depends(get_*_service) → service → model → response
                                    │
                       AdaptXError ─┴─▶ exception handler → {"error": {code, message, details}}
```

`ApplicationContext` (`core/lifecycle.py`) owns every long-lived service. Routes reach it
through FastAPI dependencies rather than module globals, so tests build an isolated context
per test.

---

## 7. LiDAR path today

Two paths, selected by the request's `preprocess` flag.

**`preprocess: false`** (default, unchanged from Phase 1)

```
POST /api/v1/lidar/frame
   → LiDARFrameRequest             JSON body, source required
   → PointCloudFrame.from_sequence structural validation + finiteness
   → FrameValidationProcessor      configured min/max point-count limits
   → LiDARIngestService            record summary, count, measured processing time
   → PointCloudSummary
```

**`preprocess: true`** (Phase 2A + 2B)

```
POST /api/v1/lidar/frame
   → RawPointCloudFrame            structural validation only; NaN/Inf permitted
   → LiDARProcessingPipeline.run
        ├ validation        configured min/max point-count limits (on the RAW input)
        ├ invalid removal   drop rows with any non-finite value, count them
        ├ ROI filter        inclusive axis-aligned box, count rejects
        ├ range filter      inclusive 3D Euclidean band, count rejects
        │
        │   ---- Phase 2B, each opt-in (ADR-012) ----
        ├ voxel downsample  VoxelDownsampler: one real point per occupied voxel
        ├ ground segment    GroundSegmenter:  split ground from non-ground
        └ noise filter      NoiseFilter:      drop sparse-neighbourhood points
   → PointCloudProcessingResult    non-ground frame + ground frame + metrics
   → LiDARIngestService            pre_validated, upstream duration folded into latency
   → PointCloudSummary + ProcessingMetrics (+ ground summary)
```

Each Phase 2B algorithm lives in its own module with its own tests; the
orchestrator only sequences them and accounts for what each one did. All three
share `perception/grid.py`, which quantises coordinates onto an integer grid and
guards the int64 overflow that would otherwise corrupt cell membership silently.

Points are only ever **removed**; nothing is repaired, clamped or invented. The counts
partition the input exactly, and each stage's output is the next stage's input.

The frame is still **not** stored or interpreted: there is no detector to hand it to.

### Coordinate convention (ADR-009)

Right-handed, origin at the sensor, metres: **+x forward, +y left, +z up**. This is what
Phase 1 already implied, since `BoundingBox3D.yaw_rad` and `VehicleState.heading_rad` are
counter-clockwise about +z from +x.

CARLA uses a **left-handed** frame (+y right). Converting is the CARLA boundary's job in
Phase 9 and does not exist yet; no module currently transforms coordinates.

### Range convention (ADR-011)

`sqrt(x² + y² + z²)` from the sensor origin, **not** the ground-plane distance, because
the minimum range models a physically 3D blind zone. Bounds are inclusive at both ends.
Squared distances are compared against squared bounds to avoid a square root over the
array.

### Point-count limits apply to the input

`min_points` / `max_points` bound how large a **raw** scan may be. A frame that filtering
legitimately reduces to zero points is a valid observation ("everything was out of
range"), not malformed input, so it is accepted and reported with its metrics.

### What exists and what does not

**Implemented:** LiDAR input validation and filtering, voxel downsampling, baseline ground
segmentation, baseline noise filtering, pipeline orchestration with per-stage measurement,
the fixed-resolution benchmark, geometric object detection with baseline classification,
baseline temporal tracking with persistent ids and measured velocity, deterministic
constant-velocity trajectory prediction with heuristic uncertainty, deterministic
frame-local fixed-resolution 2.5D mapping, heuristic object-level risk and uncertainty, and
deterministic region-adaptive spatial resolution.

**Not implemented:** ML object detection, learned tracking, appearance-based
re-identification, camera fusion, semantic segmentation, production-grade classification,
learned or map-aware trajectory prediction, collision prediction, a learned resolution
policy, a per-cell spatial risk field, an ego planned path, temporal occupancy fusion,
occlusion modelling, SLAM or localisation, CARLA sensor and actor operations, scenario
generation, event replay, and the dashboard. Their contracts exist where relevant; nothing
computes them.

**Both mapping variants now exist.** Phase 6 applies one uniform cell size everywhere and is
retained as the baseline; Phase 8 partitions the extent into regions and gives each its own
cell size. Neither mapper chooses its own resolution — deciding belongs to the controller,
applying belongs to the mapper (ADR-029, ADR-037).

**Measured for speed and workload only.** Detection, tracking, prediction, map correctness
and whether the resolution allocation is *appropriate* are all unmeasured and currently
unmeasurable: no labelled data exists. Every benchmark figure in
[`experiments/experiment-log.md`](experiments/experiment-log.md) is a timing, a cell count or
a byte count.

### Object detection (Phase 3)

```
processed non-ground frame
   -> GridConnectedComponentClusterer   grid connectivity, cluster tolerance (ADR-020)
   -> cluster geometry                  centroid, min/max, extents, distance
   -> size filtering                    point count, height, footprint bounds
   -> GeometricClassifier               dimension bands, UNKNOWN on ambiguity (ADR-021)
   -> DetectionResult                   objects + rejected candidates + measured timings
```

The detector consumes the pipeline's **non-ground** output and repeats none of its work.
Clustering, classification and the detector are separate modules, each independently
constructible and testable.

| Property | Behaviour |
|---|---|
| Clustering | Grid connected components, not DBSCAN. Merges objects in touching cells; cannot split points sharing a cell (ADR-020) |
| Bounding box | **Axis-aligned**; `yaw_rad` is always 0. No orientation is estimated |
| Classification | Dimension bands; matches none or several → `UNKNOWN` |
| Confidence | A **geometric fit score**, not a probability. `UNKNOWN` scores 0.0 (ADR-021) |
| Velocity | Always `null`. One frame cannot show motion |
| Accounting | `cluster_count == len(objects) + len(rejected)`, checked by the model |
| Rejections | Carry the reason, the measured value and the threshold it missed |

### Temporal tracking (Phase 4)

```
DetectionResult.objects
   -> expected positions        extrapolated from measured velocity, for gating only
   -> associate                 gated greedy nearest neighbour (ADR-024)
   -> update matched            position, geometry, class, measured velocity
   -> create unmatched          new tentative tracks
   -> age unmatched tracks      CONFIRMED -> COASTING, no detection fabricated
   -> retire stale tracks       ids retired, never reused
   -> TrackingResult
```

| Property | Behaviour |
|---|---|
| Identity | `track_id` is stable for a track's lifetime and never reused once retired |
| Association | Distance gate, plus optional class and size compatibility. Greedy, deterministic ties (ADR-024) |
| Velocity | `(position - previous_position) / dt` from frame timestamps. **Null** until two observations, and for non-positive or over-long intervals (ADR-023) |
| Smoothing | Exponential moving average; the raw value stays visible in `observed_velocity`, so smoothing cannot hide a jump |
| Heading | `atan2(vy, vx)` only above the configured speed floor; null below it |
| Acceleration | Null until two consecutive velocities exist |
| Lifecycle | TENTATIVE → CONFIRMED → COASTING → LOST. The existing four-state enum covers it; no new states were added |
| Class | An unknown track adopts a class immediately; a known one needs repeated agreement to change |
| `predicted_position` | Tracker state for gating. **Not** an observation and **not** a trajectory prediction |

**State.** Tracking is the first stateful stage. The tracker is owned by
`TrackingService` on the `ApplicationContext`, cleared at shutdown and resettable through
the API (ADR-025). One tracker serves the whole process, so concurrent clients share one
track set.

### Trajectory prediction (Phase 5)

Consumes `TrackingResult.tracks` and extrapolates each track's **measured** velocity:

```
tracks -> eligibility -> extrapolation -> heuristic uncertainty -> PredictedTrajectory[]
```

One formula covers every eligible track (ADR-026):

```
position(t)    = position + velocity * (age_s + t)
uncertainty(t) = base_uncertainty_m + uncertainty_growth_mps * (age_s + t)
```

`age_s` is the **measured** interval between a track's last observation and the prediction
time. It is `0.0` for a track matched in the current frame, collapsing the formula to
`p + v*t`; it is positive for a coasting track, whose stored position is already stale by
exactly that much. Ignoring it would silently pretend a missed frame never happened, so the
trajectory is marked `EXTRAPOLATED` and carries `observation_age_s`.

Points run from `t+0` to the horizon **inclusive**, so the defaults (3.0 s, 0.25 s) give 13
points. Each point's absolute timestamp is `source timestamp + time_offset_s`, computed
arithmetically - never read from a wall clock, so a trajectory is reproducible.

**What is deliberately not modelled:** acceleration (a second difference of noisy centroids,
amplified over a 3-second horizon), turning, road and lane geometry, interaction between
objects, and class-conditioned motion. `heading_rad` is **not** used to steer the
extrapolation: it is derived from velocity and null below the tracker's speed floor, so the
velocity vector remains the authoritative motion vector.

**Eligibility (ADR-027).** Every track handed to the predictor appears either in
`trajectories` or in `skipped` with a reason; `considered == predicted + skipped` is enforced
by the contract. `velocity is None` yields **no trajectory** - null is not zero (ADR-023) -
while a *measured* standstill yields a stationary one. An over-speed velocity is rejected,
never clipped.

**Uncertainty is heuristic and labelled as such.** It is not a calibrated sigma, not a
probability and not a confidence interval, because no labelled trajectories exist to
calibrate one against. Point `confidence` decays exactly as fast as uncertainty grows, and
measures *evidence* (track hits and missed frames), not correctness.

**Statefulness.** `PredictionService` holds no perception state: a prediction is a pure
function of one tracking result. Its counters exist only for status and telemetry, and
resetting it changes what the status endpoint reports, never what the predictor produces.

**Limitations.** Constant velocity is wrong through turns and braking, and a wrong
trajectory looks exactly as confident as a right one apart from its uncertainty radius.
Prediction quality is bounded by tracking quality, which is bounded by detection quality.
Cost is linear in trajectory points and dominated by contract validation rather than
arithmetic (Experiment 004).

### Risk and uncertainty (Phase 7)

Consumes tracks, Phase 5 trajectories and the Phase 6 map, and answers one question per
object: *how concerning is this object right now, and how sure are we?*

```
tracks + trajectories + map -> factors -> weighted mean -> level
                            -> uncertainty breakdown -> RiskAssessment
```

Three factors, each normalised to `[0, 1]`, combined as a weighted mean **over the factors
actually available** (ADR-032):

```
risk_score = sum(w_i * f_i) / sum(w_i)     over available i only
```

| Factor | Source | Unavailable when |
|---|---|---|
| `proximity` | planar distance; 1.0 at/below `proximity_near_m`, 0.0 at/above `proximity_far_m` | never |
| `closing_speed` | radial rate of approach `-(p·v)/\|p\|`; receding contributes 0.0 | `velocity is None`, or the object sits exactly at the reference point |
| `predicted_proximity` | closest approach of the Phase 5 trajectory | no trajectory for that track |

**A missing factor is dropped and the remaining weights renormalise — never scored zero.**
Zero is what an unmeasured value would look like, so scoring it that way would make the
object we know least about appear least concerning. When no factor can be computed, the
assessment is `UNKNOWN` with `risk_score = None`.

**Uncertainty is reported beside risk, never folded into it** (ADR-033). Two tracks identical
except for observability get the *same* risk and different uncertainty. That separation is
what Phase 8 needs: a poorly observed region may deserve finer perception precisely because
it is poorly observed.

**Map context never lowers risk** (ADR-034). An empty cell means no returns landed there,
which may be because nothing is present or because something occluded it — Phase 6 cannot
tell the difference. It raises uncertainty instead.

**The scene aggregate is a maximum, never a mean** (ADR-035), and an unassessed scene reports
`UNKNOWN` rather than `LOW`.

**What is deliberately not modelled:** time-to-collision (a precise-looking number over two
approximations), trajectory-map intersection, occlusion, ego planned path, object
interaction, and a per-cell spatial risk field. Object class is *reported* and appears in the
explanation but is not a score multiplier — that would encode an unmeasured judgement.

**Risk does not decide resolution** (ADR-036). The engine imports no resolution type,
produces no `ResolutionDecision`, and `RiskAssessment` carries no cell size. Phase 8 consumes
these assessments and decides spatial detail — and did so without this engine changing, which
is the outcome the boundary was drawn for.

**Limitations.** The score orders objects by concern; it measures nothing physical. It is not
a probability of collision, is not calibrated, and has never been validated — no labelled
risk data exists. Weights and thresholds are baseline engineering values that have never been
tuned against outcomes. Quality is bounded by tracking and prediction, which are themselves
baselines.

### Adaptive spatial resolution (Phase 8)

*How much spatial detail should this region receive?* — the question the project is named
for, and the only one that spans risk, uncertainty, motion and the compute budget at once.

```
RiskAssessment[] + TrackedObject[] + PredictedTrajectory[]
    -> spatial influence -> per-region ResolutionContext
    -> detail priority   -> thresholds -> unknown-risk floor
    -> hysteresis + dwell -> budget
    -> ResolutionPlan -> TiledAdaptiveMapper -> AdaptiveSpatialMap
```

**Regions, not points (ADR-037).** The map extent is partitioned into fixed-size square
regions, each holding its own dense sub-grid at its own cell size — which is how one map holds
several resolutions when a NumPy array can only hold one. Regions are half-open on their upper
edges and clipped at the map bounds, so they partition the extent **exactly**: a point lands
in one region and one cell of it, and `input == mapped + out_of_bounds` is unchanged from
Phase 6.

**The detail priority (ADR-038).** Six normalised factors — risk, uncertainty,
predicted-motion relevance, proximity, object density, measured motion — combined as a
weighted mean **over the factors actually available**. A factor that cannot be computed is
dropped and the remaining weights renormalise; it is never scored zero.

**Unknown is not low.** `risk_score is None` drops the risk factor *and* floors the region
level. Coercing it to zero would hand the coarsest representation to the objects the system
understands least — the ADR-032 inversion, one phase later and with worse consequences.

**Uncertainty is an independent input**, never summed into risk (ADR-033). A quiet, badly
observed region can earn detail on uncertainty alone. That is why Phase 7 kept them apart.

**Stabilisation (ADR-039).** Refinement applies on the frame it is asked for. Coarsening must
clear a hysteresis margin *and* be proposed on `min_dwell_frames` consecutive frames. The
asymmetry is deliberate: wasted cells are cheaper than missing structure. This makes the
controller the only stateful component in the adaptive path — but the **map** stays
frame-local, because what persists is a policy decision, not occupancy (ADR-030 intact).

**Budgets (ADR-040).** Region, cell and fine-region ceilings are enforced by coarsening the
**lowest-priority** regions first, deterministically, with every demotion recorded on the
decision and counted in the plan.

**Positions come from tracks, not assessments (ADR-041).** A `RiskAssessment` carries a
distance, not a location; adding one would have pushed a spatial concept into the layer
ADR-036 keeps free of it.

**Limitations.** The priority orders regions; it measures nothing physical. It is not a
probability of collision, not a safety margin, not calibrated, never validated — no labelled
data exists. Whether the allocation is appropriate is unmeasured and unmeasurable. Allocation
is quantised to the region. Cost is dominated by **region count, not cell count**
(Experiment 007), and adaptive mapping measured slower than the fixed mapper in every scene.

### Phase 2B stage semantics

| Stage | Behaviour |
|---|---|
| Voxel downsample | One point per occupied voxel: the **real measured point nearest that voxel's centroid**, ties broken by lower input index. The centroid is never emitted, so no coordinate is manufactured (ADR-015). Grid anchored at the frame origin, so voxel boundaries are the same for every frame. |
| Ground segmentation | Per xy cell, the lowest point defines the local ground level; points within a tolerance above it are ground (ADR-016). Ground is **separated, not discarded** — it is returned as `ground_frame`. Needs no sensor mount height, which is why Phase 2B needs no coordinate transform (ADR-013). |
| Noise filter | Drops points with too few neighbours in the 3×3×3 block of cells around them (ADR-014). Applied to non-ground points only, since that is what feeds detection. |

**Accounting.** `input = invalid + roi_rejected + range_rejected + voxel_reduced +
ground + noise_removed + output`, checked by the model itself. Ground points count as
"did not continue downstream" rather than "discarded".

**Timing.** Every stage carries its own measured `duration_ms`. The stage durations sum to
*less* than the frame total; the difference is reported as `overhead_ms` — the array
compaction shared by the filtering stages, frame construction and metric assembly. It is
reported rather than folded into a stage, so no stage duration is inflated.

**Configuration.** Every result carries a `PipelineConfiguration` snapshot of the settings
that produced it, so a benchmark record or a replayed frame can be reproduced from what it
reports rather than from the environment it happened to run in.

**Known stage-order consequence.** The knowledge-base order is voxelise → ground → noise,
and ground bypasses the noise filter. An isolated stray point is therefore the lowest
point of its own cell, is classified as ground, and is never seen by the noise filter. A
test asserts this so it stays visible; changing it would mean deviating from the
documented order, which is not done silently.

---

## 8. CARLA boundary

`CarlaSimulatorClient` (ABC) has two implementations:

- `CarlaClient` — imports the optional `carla` package lazily inside `connect()`. Without
  it, `connect()` raises `SimulatorUnavailableError` with the reason. `get_sensor_data`,
  `get_vehicle_state`, `spawn_*` and `destroy_actor` raise an explicit "not implemented in
  Phase 1" error rather than returning placeholder data.
- `MockCarlaSimulatorClient` — deterministic (seeded) fake for development and tests. All
  output is labelled `synthetic_test`.

`CarlaService.connect()` never raises: a disabled, missing or unreachable simulator is
recorded as `DISCONNECTED` with a human-readable reason, and the backend keeps running.

---

## 9. System state semantics

| State | Meaning |
|---|---|
| `RUNNING` | Every **required** component is `READY` |
| `DEGRADED` | An optional but *enabled* component is unavailable — CARLA enabled and not connected, or the LiDAR feed went stale after previously delivering frames |
| `ERROR` | A required component is `NOT_READY` |

Required in Phase 1: `api`, `configuration`, `telemetry`, `lidar_ingest`. Components
scheduled for a later phase are `PLANNED` and do not degrade the state — their absence is
expected, not a fault.

---

## 10. Extension points

Each future module plugs in behind an existing contract, with no change to the API,
services or models:

| To add | Implement | Then wire in |
|---|---|---|
| Further point-cloud processing (Phase 2C+) | `perception.interfaces.LiDARProcessor` | add a stage module beside `voxel` / `ground` / `noise` and sequence it in `LiDARProcessingPipeline.run` |
| ML object detection | `perception.interfaces.ObjectDetector` | replace `GeometricObjectDetector` in `build_context()`; the API and services are unchanged (ADR-022) |
| A learned tracker | `tracking.interfaces.ObjectTracker` | replace `GeometricObjectTracker` in `build_context()`; the API and services are unchanged |
| A better predictor | `prediction.interfaces.TrajectoryPredictor` | replace `ConstantVelocityPredictor` in `build_context()`; the API and services are unchanged (ADR-027) |
| An adaptive mapper | `mapping.interfaces.AdaptiveMapper` | replace `FixedResolutionMapper` in `build_context()`; the API and services are unchanged. Set `is_adaptive` so the two variants stay distinguishable (ADR-003) |
| Risk (Phase 7) | `risk.interfaces.RiskEngine` | replace `BaselineProximityRiskEngine` in `ApplicationContext`; keep the baseline for comparison. Consumes Phase 5 trajectories, including their uncertainty and confidence |
| Adaptive resolution (Phase 8) | `mapping.interfaces.ResolutionController` | consumed by the adaptive mapper |

When a module becomes real, update its row in `_declared_components()`
(`services/system_service.py`) and add its stream to the telemetry payload, removing it
from `not_yet_available`.

---

## 11. Deliberate non-decisions

Phase 1 does **not** introduce a database, a message broker, a task queue, an ML framework
or a cloud service. None is needed to ingest, validate and report on a frame, and adding
one now would fix an architecture before the requirement is understood
(`knowledge-base/20-constraints.md`: no unnecessary frameworks).

Open3D is declared as an optional extra (`pip install -e ".[pointcloud]"`) and is **not
installed**: Phase 1 performs no geometric processing. Phase 2 should install it and record
the decision.
