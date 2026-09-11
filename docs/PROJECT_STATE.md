# ADAPT-X — Project State

**Primary context file for a fresh session.** Read this first, then
[`PHASE_HISTORY.md`](PHASE_HISTORY.md) for how it got here and
[`NEXT_PHASE.md`](NEXT_PHASE.md) for what to build next.

Snapshot taken 2026-09-11 (Phase 10). The repository is the source of truth; if this file
disagrees with the code, the code wins and this file needs fixing.

---

## 1. Identity

ADAPT-X — Adaptive Dynamic Perception and Tracking. A LiDAR perception system that
allocates spatial map resolution by **risk** instead of uniformly. Research prototype,
not validated for real-world autonomous driving.

Rules that govern all work are in [`../CLAUDE.md`](../CLAUDE.md) and
[`knowledge-base/20-constraints.md`](knowledge-base/20-constraints.md). The central one:
**never fabricate a measurement, metric, accuracy figure or benchmark result.**

## 2. Current phase

**Phase 10 COMPLETE** (scenario framework; event replay deferred). Phases 1 through 10 are
implemented and verified.
Adaptive resolution — the thing the project is named for — now exists as a deterministic
heuristic baseline, and has been measured against the fixed baseline (Experiment 007).

**Phase numbering was resolved and changed in Phase 5.** Trajectory prediction moved from
Phase 8 to **Phase 5**, because it is what the risk engine needs next; 2.5D mapping, risk
and adaptive resolution each shifted one later (now 6, 7, 8). `CLAUDE.md`,
`docs/ROADMAP.md` and the `phase` field on every component in `services/system_service.py`
agree. Phase headings in [`PHASE_HISTORY.md`](PHASE_HISTORY.md) are a historical record and
were deliberately left as originally written.

## 3. Branch and status

- Branch: `phase-10-scenario-framework`, branched from `origin/main`
- Phases 1 through 9 are committed **and merged into `origin/main`** (PR #1 through PR #6).
  Phase 9 is commit `efc935c`, merged as PR #6 in `9092c69`
- Phase 10 is committed on this branch as `2a4ce2d`, **not pushed**. The live-validation
  fixes of 2026-09-11 (ADR-049, Experiment 010) are in the working tree on top of it,
  **not committed**

> Local `main` was stale at `5f68a5e` (the Phase 7 merge) when Phase 9 began, two commits
> behind `origin/main`. Branching from it would have silently dropped Phase 8. **Check
> `git log origin/main` rather than local `main` before branching.**

> A previous version of this file claimed Phase 5 and Phase 6 were unpushed and Phase 7 was
> "in the working tree". That was stale by the time it was read: all three are merged. The
> lesson keeps repeating — **check `git log` rather than trusting this section**, and update
> it when it is wrong.

## 4. Architecture implemented

```
ScenarioDefinition (data: actors, placement, motion, seed)   (Phase 10)
  → resolve(seed) → ScenarioRunner: place actors at scripted pose per frame
        ↓
CARLA simulation                                        (Phase 9, optional)
  → session: sync mode, fixed timestep, ego + LiDAR
  → one coordinate conversion (left-handed → right-handed)
  → RawPointCloudFrame(source=simulation) ─┐
                                           │  (and, on a separate path,
                                           │   GroundTruthFrame → evaluation only)
                                           ▼
RawPointCloudFrame
  → validation → invalid removal → ROI → range          (Phase 2A, always on)
  → voxelisation → ground segmentation → noise filter   (Phase 2B, opt-in)
  → PointCloudProcessingResult  { frame, ground_frame, metrics, configuration }
        ↓ non-ground points
  → grid clustering → geometry → size filter → classification   (Phase 3)
  → DetectionResult  { objects, rejected, counts, timings, configuration }
        ↓ DetectedObject[]
  → association → update/create/age/retire              (Phase 4)
  → TrackingResult  { tracks, new/deleted ids, counts, timings, configuration }
        ↓ TrackedObject[]
  → eligibility → constant-velocity extrapolation → heuristic uncertainty   (Phase 5)
  → PredictionResult { trajectories, skipped+reasons, counts, timing, configuration }

PointCloudFrame (Phase 2 output) + ResolutionDecision
  → bounds check → cell indices → accumulate counts and height stats        (Phase 6)
  → SpatialMap { occupancy, point_count, min/max/mean height, accounting, configuration }

TrackedObject[] + PredictedTrajectory[] + SpatialMap
  → eligibility → factors → weighted mean over available → level            (Phase 7)
  → uncertainty breakdown (reported separately, never summed into the score)
  → RiskAssessmentResult { assessments, aggregate, timing, configuration }
        ↓ RiskAssessment[] + TrackedObject[] (positions) + PredictedTrajectory[]
  → spatial influence → per-region ResolutionContext → detail priority      (Phase 8)
  → thresholds → unknown-risk floor → hysteresis → dwell → budget
  → ResolutionPlan { decisions, excluded, budget, timing, configuration }
        ↓ + PointCloudFrame
  → tile assignment → per-region binning
  → AdaptiveSpatialMap { tiles, plan, accounting, timing }
```

Phase 8 reads **positions from tracks**, not from assessments: a `RiskAssessment` carries a
distance, not a location, because Phase 7 was deliberately given no spatial vocabulary
(ADR-036, ADR-041). Neither the mapper nor the risk engine changed to accommodate Phase 8 —
the boundaries drawn in ADR-029 and ADR-036 held.

Mapping is a **parallel consumer** of the Phase 2 frame, not a stage after prediction. It
does not read detections, tracks or trajectories, and stays independently usable.

Every stage measures its own duration with `time.perf_counter`. Every result carries a
configuration snapshot so a record is self-describing.

## 5. Module structure

| Package | Contents |
|---|---|
| `adaptx.config` | `settings.py` — all typed settings |
| `adaptx.core` | `logging`, `exceptions`, `lifecycle` (service graph) |
| `adaptx.models` | All data contracts (see §6) |
| `adaptx.perception` | `pipeline` (orchestrator), `voxel`, `ground`, `noise`, `grid`, `clustering`, `classification`, `detector`, `lidar`, `interfaces` |
| `adaptx.tracking` | `tracker`, `association`, `interfaces` |
| `adaptx.prediction` | `constant_velocity`, `interfaces` |
| `adaptx.mapping` | `grid_mapper` (fixed baseline), `controller` (Phase 8 policy), `adaptive_mapper` (tiled), `tiles` (shared geometry), `comparison`, `interfaces` |
| `adaptx.risk` | `heuristic` (Phase 7 engine), `baseline` (proximity-only comparison reference), `interfaces` |
| `adaptx.scenarios` | `models` (definition, motion, resolution - imports nothing from `adaptx.carla`), `result` (raw evidence, no metrics), `interfaces` (`ScenarioSimulator` protocol), `runner`, `catalogue` (4 scenarios), `__main__` (CLI) |
| `adaptx.carla` | Boundary: `client` (connection), `session` (deterministic simulation lifecycle), `conversion` (the single coordinate/time boundary, imports no simulator), `ground_truth`, `interfaces`, `mock`. Optional dependency. `smoke.py` was deleted in Phase 10 |
| `adaptx.services` | `lidar_service`, `metrics_service`, `carla_service`, `system_service`, `tracking_service`, `prediction_service`, `mapping_service`, `risk_service`, `adaptive_mapping_service` |
| `adaptx.api` | `app`, `schemas`, `dependencies`, `routes/`, `websocket/` |
| `adaptx.benchmark` | `datasets`, `baseline`, `runner`, `detection`, `tracking`, `prediction`, `mapping`, `risk`, `adaptive`, `models` |

## 6. Key contracts (`adaptx.models`)

| Model | Notes |
|---|---|
| `BasePointCloudFrame` → `RawPointCloudFrame` / `PointCloudFrame` | Raw **may** hold NaN/Inf; validated may not (ADR-010) |
| `PointCloudProcessingResult`, `ProcessingMetrics`, `StageMetrics`, `PipelineConfiguration` | Per-stage counts + durations; `overhead_ms` reported separately |
| `DetectedObject` | class, position, AABB, confidence (**geometric fit score, not a probability**), `distance_m`, `velocity` always `None` |
| `DetectionResult`, `RejectedCluster`, `DetectionConfiguration` | `cluster_count == len(objects) + len(rejected)`, model-enforced |
| `TrackedObject` | `velocity` / `acceleration` / `heading_rad` are **`None` until measured** (ADR-023); `speed_mps` → `float \| None` |
| `TrackingResult`, `TrackingConfiguration` | matched/created/retired ids, counts, timings |
| `PredictedTrajectory`, `TrajectoryPoint` | Extrapolated positions, **never measurements**. `timestamp` is the *source* time; `horizon_s`/`predictor_name` are what other projects call `prediction_horizon_s`/`model_name`. Carries `status` and measured `observation_age_s` |
| `PredictionResult`, `SkippedTrack`, `PredictionConfiguration` | `considered_track_count == len(trajectories) + len(skipped)`, model-enforced |
| `PredictionStatus` | `predicted`, `extrapolated`, `insufficient_velocity`, `invalid_velocity`, `stale_observation`, `track_lost`, `limit_exceeded` |
| `SpatialMap` | The Phase 6 grid: NumPy arrays `[row, column]` of `point_count` and min/max/mean height. **NaN where unobserved** (ADR-031). Occupancy is derived, `point_count > 0` |
| `MapBounds`, `MapAccounting`, `MappingConfiguration`, `SpatialMapSummary` | Half-open extent; `input == mapped + out_of_bounds` model-enforced; summary carries no grid |
| `ResolutionDecision`, `ResolutionSource` | The **output** of a resolution decision (`resolution_m`, `source`, `reason`). `FIXED` from Phase 6, `ADAPTIVE` from Phase 8 |
| `AdaptiveMap`, `AdaptiveMapCell` | Pre-existing contract, populated by projecting **occupied cells only**. Since Phase 8 its per-cell `resolution_m` and `resolution_level` finally carry genuinely *varying* values |
| `ResolutionContext` | The **inputs** to one region's decision, now consumed by the Phase 8 controller. Every quantity a region may lack is **nullable**: `risk_score is None` means unscored, never 0.0 (ADR-038) |
| `TileResolutionDecision`, `ResolutionPlan` | Phase 8 output: one decision per region with its priority, factors, influencing tracks, previous level and generated reason. Accounts for every assessment (`considered == influencing + excluded`) |
| `DetailFactors`, `DetailFactorName` | Normalised value of each factor; **null when not computed**, never zero |
| `ResolutionBudget` | What the plan allocated against the region, cell and fine-region ceilings, plus how many regions were coarsened to fit |
| `AdaptiveSpatialMap`, `MapTile` | The Phase 8 map: one dense sub-grid per region, each at its own cell size. Same per-cell quantities as `SpatialMap`, NaN where unobserved |
| `MappingComparison`, `MappingVariantMetrics` | Fixed and adaptive over identical input. A ratio here is arithmetic over two measurements, never a claim |
| `RiskAssessment`, `RiskAssessmentResult` | Phase 7 per-object risk. `risk_score` is **`None` exactly when `risk_level` is `UNKNOWN`** (ADR-032) - nothing is invented |
| `UncertaintyBreakdown` | Heuristic scalar **plus its reasons**. Reported beside risk, never folded into it (ADR-033) |
| `TrajectoryRelevance`, `MapContext` | Closest predicted approach; what the map recorded at the object's cell. Map context **never lowers** risk (ADR-034) |
| `RiskLevel` | `LOW/MEDIUM/HIGH/CRITICAL` are scored bands; **`UNKNOWN` is not a point on the scale** and has no threshold |
| `RiskField`, `RiskCell`, `ObjectRisk` | Phase 1 contracts. `ObjectRisk` is now populated via `evaluate()`; `RiskCell` is still unused - Phase 7 is object-level only |
| `SystemStatus`, `ComponentStatus`, `SystemMetrics` | Readiness **and** implementation status (ADR-005) |
| `SimulationState`, `SimulationSessionStatus` | Phase 9 session lifecycle, reported **beside** `CarlaStatus` because "package installed", "server connected" and "simulation running" are three independent facts |
| `ScenarioDefinition`, `ScenarioActor`, `MotionSegment`, `Placement`, `EgoDefinition` | Phase 10 scenario as **data**: ego-relative placement, timed constant-velocity segments, explicit seed. Validated at construction; JSON round-trips |
| `ResolvedScenario`, `ResolvedActor` | The definition with every seeded value drawn, recorded on the result so a run reproduces without the catalogue |
| `ScenarioRunResult`, `ScenarioFrameRecord`, `ExpectedPose`, `StageCounts` | Raw evidence: frame identities, commanded poses, ground truth, stage counts. **No accuracy or evaluation field**, asserted by test |
| `GroundTruthActor`, `GroundTruthFrame` | What the simulator *knows*. Deliberately **not** a perception contract and carries no confidence - the simulator is not estimating (ADR-045) |

Shared rules: `schema_version`, tz-aware UTC timestamps, `coordinate_frame`, `source`
(`live_sensor`/`simulation`/`replay`/`synthetic_test`/`unavailable`), `extra="forbid"`.

## 7. API endpoints (17, unchanged in Phases 9 and 10)

| Method | Path |
|---|---|
| GET | `/health` |
| GET | `/api/v1/system/status`, `/api/v1/system/metrics` |
| GET | `/api/v1/carla/status`, `/api/v1/map/status`, `/api/v1/risk/status` |
| GET | `/api/v1/prediction/status` |
| POST | `/api/v1/lidar/frame` — validate/ingest, optional `preprocess` flag |
| POST | `/api/v1/lidar/detect` — process + detect |
| POST | `/api/v1/lidar/track` — process + detect + track (**stateful**) |
| POST | `/api/v1/lidar/predict` — process + detect + track + predict (**stateful**) |
| POST | `/api/v1/lidar/map` — process + map (**stateless, frame-local**) |
| POST | `/api/v1/lidar/risk` — whole chain + risk assessment (**stateful**) |
| POST | `/api/v1/lidar/adaptive-map` — whole chain + adaptive map (**stateful twice**: tracks *and* resolution stabilisation) |
| POST | `/api/v1/map/adaptive/reset` — clear the resolution stabilisation state |
| POST | `/api/v1/tracking/reset` |
| GET | `/api/v1/tracking/status` |
| WS | `/ws/telemetry` |

## 8. Configuration sections

`ADAPTX_` prefix, `__` nesting. Sections: `APP`, `API`, `LOGGING`, `CARLA`, `LIDAR`
(2A bounds + opt-in 2B stages), `DETECTION`, `TRACKING`, `PREDICTION`, `MAP` (levels
vocabulary **and** Phase 6 mapping geometry), `ADAPTIVE` (Phase 8 resolution policy),
`RISK` (thresholds **and** Phase 7 heuristics), `WEBSOCKET`.

`LIDAR`, `PREDICTION`, `MAP`, `ADAPTIVE`, `RISK` and `WEBSOCKET` are documented in `.env.example`.
**`DETECTION` and `TRACKING` are not** — a gap left by Phases 3 and 4, still open.

Defaults worth knowing: detection `cluster_tolerance_m=0.5`, `min_cluster_points=10`;
tracking `max_association_distance_m=2.5`, `min_hits_to_confirm=3`, `max_missed_frames=3`,
`velocity_smoothing=0.5`, `max_timestep_s=2.0`, `min_speed_for_heading_mps=0.3`;
prediction `horizon_s=3.0`, `interval_s=0.25` (13 points, t+0 inclusive), `max_tracks=256`,
`max_speed_mps=80.0`, `base_uncertainty_m=0.5`, `uncertainty_growth_mps=0.5`,
`confidence_hits_full=3`; mapping `resolution_m=0.5`, bounds ±60 m (240x240 = 57,600 cells), `min/max_resolution_m=0.05/5.0`, `max_cells=4,000,000`; risk proximity `5.0`-`40.0 m`, `closing_speed_high_mps=15.0`, weights `0.50/0.25/0.25`, thresholds `0.35/0.60/0.85`, `stale_observation_s=0.5`;
adaptive `tile_size_m=10.0` (144 regions over the default bounds), `base_level=LOW`,
`unknown_risk_min_level=MEDIUM`, thresholds `0.35/0.60/0.85`, weights
`0.35/0.20/0.15/0.15/0.10/0.05`, `influence_radius_m=8.0`, `hysteresis_margin=0.08`,
`min_dwell_frames=3`, `max_tiles=4096`, `max_total_cells=1,000,000`, `max_fine_tiles=64`.

## 9. Telemetry (`/ws/telemetry`)

`hello` then periodic `telemetry`. Payload provides `system`, `metrics`, `detection`,
`tracking`, `prediction`, `mapping`, `risk`, `adaptive_mapping` — the last six are
**summaries** (configuration and counts), never per-frame geometry, trajectory points, map
cells, tiles, region decisions, full assessments or point arrays. Risk carries scene counts
plus at most five ranked objects; adaptive mapping carries the region count, the level
distribution and how many regions changed.

`not_yet_available` now lists **only** `risk_field`: object-level risk exists but a per-cell
risk **field** does not. `adaptive_map` left the list in Phase 8 because something genuinely
produces it, which is the rule that list has always followed.

## 10. Lifecycle

`ApplicationContext` (`core/lifecycle.py`) holds: `settings`, `metrics`, `lidar`, `carla`,
`system`, `risk_engine` (Phase 1 proximity baseline), `preprocessor`, `detector`, `tracking`, `prediction`, `mapping`, `risk` (Phase 7), `adaptive_mapping` (Phase 8). Built by
`build_context()`, attached to `app.state`, reached through FastAPI dependencies.

**Tracking and adaptive resolution are the two stateful components** (ADR-025). One tracker
per process, lock-guarded, cleared on shutdown; concurrent clients share one track set. The
resolution controller likewise remembers each region's level and dwell counter, because
stabilising resolution is temporal by definition (ADR-039) — but the **map** stays
frame-local: what persists is a policy decision, never occupancy.

`PredictionService`, `MappingService` and `RiskService` are **stateless with respect to
perception**: each holds counters for status and telemetry only, and resetting one changes
what the status endpoint reports, never what it produces. Mapping is frame-local by
construction (ADR-030) — map `N` cannot contaminate map `N+1` because nothing is retained.

## 11. Dependencies

Runtime: `fastapi`, `uvicorn[standard]`, `pydantic>=2`, `pydantic-settings`, `numpy`, `psutil`.
Dev: `pytest`, `pytest-asyncio`, `httpx`, `ruff`, `mypy`.
Optional extras declared but **not installed**: `open3d` (`[pointcloud]`), `carla` (`[carla]`).

**No SciPy, no ML framework, no database.** Declined on record in ADR-014, 020, 024 and
026. Phases 5, 6, 7 and 8 each added none — eight phases, zero beyond the Phase 1 set.

## 12–14. Verification status

- **1452 tests pass**, 7 deselected (`pytest`, Python 3.13; 1451 + 1 skipped on 3.12). The 7 are the live CARLA tests
- `ruff check .` — All checks passed
- `ruff format --check .` — 192 files formatted
- `mypy src` — no issues in 109 source files
- Backend starts; all 17 endpoints respond; no tracebacks
- **Live CARLA run executed on 2026-09-11** (Experiment 010): CARLA 0.9.16 server on
  `127.0.0.1:2000`, Town10HD_Opt. `pytest -m carla` **7 passed** from a Python 3.12
  environment (`.venv312`) holding the 0.9.16 wheel; every catalogue scenario COMPLETED via
  the CLI; zero actors left on the server. In the primary Python 3.13 environment the
  package cannot be installed (no wheel), so `pytest -m carla` there still reports 7 skipped
  and the default suite is unchanged. Nothing in this repository reports a CARLA accuracy
  figure; the only live performance figures are the per-frame stage timings in
  Experiment 010, measured on this machine
- Live temporal check: a vehicle advancing 1 m per 0.5 s measured 2.000 m/s, and its
  trajectory advanced +1 m at t+0.5, +2 m at t+1, +4 m at t+2 and +6 m at t+3; uncertainty
  rose 0.5 → 2.0 m; the track's first frame produced **no trajectory** and an explicit
  `insufficient_velocity` skip

Benchmarks (`python -m adaptx.benchmark [--detect|--track|--predict|--map|--risk|--adaptive]`)
— all synthetic, **workload and speed only**: pipeline ~213 ms/100k points; detection ~3.4 ms
at that size; tracking ~10.6 ms at 100 objects; prediction ~27 ms at 100 tracks (13 points
each); mapping ~17 ms at 98k points and 1.0 m cells, rising to ~52 ms at 0.25 m; risk ~7.4 ms
at 100 objects (~74 µs each); adaptive resolution planning 7–13 ms and adaptive mapping
25–32 ms over 144 regions. Measured results in
[`experiments/experiment-log.md`](experiments/experiment-log.md).

**Experiment 007 is the fixed-versus-adaptive comparison, and it is not a clean win.**
Adaptive uses 0.06–0.30x the cells of a uniform 0.25 m map, but **more** cells than a 1.0 m
one in every scene containing an object — the base level *is* 1.0 m. Adaptive mapping is also
**slower in wall-clock time than the fixed mapper in every scene**, even where it allocates a
quarter of the cells: cost scales with **region count, not cell count** (~120 µs per region),
so `tile_size_m` is the lever. Read the entry before quoting any of it.

## 15. Known limitations — do not hide these

- **No labelled data exists.** Detection accuracy, tracking correctness and prediction
  accuracy are all unmeasured and currently **unmeasurable**. Every benchmark measures
  speed only.
- Detection is a geometric baseline: no trained model, no oriented boxes (AABB only,
  `yaw_rad` always 0), no velocity from a single frame.
- Clustering **merges** objects in touching grid cells and cannot split points sharing a cell.
- Classification returns `UNKNOWN` on ambiguity — common in real scenes. Confidence is a
  geometric fit score, **not** a probability.
- Ground segmentation misclassifies an object-only cell's lowest points as ground.
- Noise filtering tests a **cube**, not a sphere.
- Tracking: centroid association can swap identities during close crossings; **no
  re-identification** — a retired track returning gets a new id; quality bounded by detection.
- Association is `O(T×D)`: ~10 ms at 100 objects, ~219 ms at 500.
- Stage-order consequence: ground bypasses the noise filter, so an isolated stray point
  becomes its own cell's ground and is never removed.
- **Prediction is constant velocity and nothing else.** It is wrong through turns and
  braking, and a wrong trajectory looks exactly as confident as a right one apart from its
  uncertainty radius. Quality is bounded by tracking, which is bounded by detection.
- **Prediction uncertainty is a documented heuristic**, not a calibrated sigma, probability
  or confidence interval. It cannot be calibrated without labelled trajectories.
- Prediction cost is linear in trajectory points and dominated by **contract validation**,
  not arithmetic: ~27 ms at 100 tracks. The lever is `interval_s`, not the motion model.
- The Phase 6 mapper is still one uniform cell size everywhere — deliberately, because it
  is the baseline (ADR-003). Measured occupancy falls to 1-16% at 0.25 m (Experiment 005): a
  fine uniform map spends most of its cells recording that nothing was observed. Phase 8 is
  the answer to that, and it is reported separately below.
- **Mapping cannot distinguish unobserved from free.** A cell occluded behind a vehicle
  reads exactly like empty space. This matters for safety and is the first thing a future
  occupancy model should fix.
- Mapping is frame-local, so occlusion is never remembered; there is no temporal fusion, no
  ego-motion compensation, no SLAM and no localisation.
- Map correctness is **unmeasured and unmeasurable**: no labelled reference map exists.
- **The risk score is a heuristic, not a probability of collision.** It is not
  calibrated, has never been validated against labelled risk data — none exists — and its
  weights and thresholds are baseline engineering values never tuned against outcomes. It
  orders objects by concern; it measures nothing physical.
- Risk is **object-level only**. There is no spatial risk field, so `RiskCell` and
  `AdaptiveMapCell.risk_score` remain unpopulated and `risk_field` stays unavailable.
- Risk models **no** time-to-collision, no trajectory-map intersection, no occlusion, no ego
  planned path and no interaction between objects. Object class is reported but is not a
  score multiplier.
- Map context contributes nothing to the score by design (ADR-034): an empty cell is
  unobserved, not free, so it can only raise uncertainty.
- **The scenario framework records evidence and computes nothing about it.** Every run
  carries commanded poses, reported ground truth and pipeline counts side by side, and
  no field compares them. Doing so is Phase 11.
- **Scripted motion has no physics.** Actors are placed, not driven: no acceleration, no
  turning radius, no collision response. An actor scripted into a wall sits in the wall.
- **The ego is stationary in every scenario.** `EgoDefinition.stationary` is validated
  `True`; only targets move.
- **The catalogue's blueprints have not been confirmed on any real server.** A missing one
  fails the run honestly, but it has not been tried.
- **Event replay is deferred.** The run result is the recording a replay would need;
  `DataSource.REPLAY` is still produced by nothing.
- **CARLA is integrated but unexercised against a real server here.** The adapter, the
  conversion and the lifecycle are tested against a stand-in; nothing has confirmed API
  compatibility with a running CARLA, and no simulator figure is claimed.
- CARLA pitch and roll are not converted - only yaw. A tilted sensor mount would need them.
- The smoke scenario is one hard-coded scene with scripted motion, not physics. It exists to
  prove the integration and is not a scenario framework.
- **Ground truth now exists, and nothing measures against it yet.** That is the point of
  Phase 11; treating Phase 9 as an accuracy result would be wrong.
- Risk quality is bounded by tracking and prediction, which are themselves baselines.
- **The detail priority is a heuristic, not a probability and not a safety margin.** It is
  uncalibrated, never validated, and its weights and thresholds have never been tuned against
  outcomes because no outcomes exist. It orders regions; it measures nothing physical.
- **Whether the allocation is *appropriate* is unmeasured and unmeasurable.** Experiment 007
  measures what it costs, not whether it is right — that needs labelled risk data and a
  reference map, and neither exists.
- **Adaptive mapping is slower than the fixed mapper in every measured scene**, even where it
  allocates far fewer cells. Cost is dominated by per-region overhead (~120 µs/region), not
  by cells or points. A hierarchical representation is where that would be attacked (ADR-037).
- **Adaptive costs more cells than a coarse uniform map.** It is a way of affording a fine
  map, not a way of beating a coarse one.
- **Allocation is quantised to the region.** A small object refines a whole 10 m region around
  it, so detail is spent on ground that did not ask for it.
- Resolution levels persist between frames, so a genuinely quiet region keeps its detail for
  up to `min_dwell_frames` after it stops needing it — the deliberate cost of not oscillating.
- **What the adaptive map loses is not quantified.** Where a region sits at LOW it is coarser
  than a fine uniform baseline, and no measurement says what structure that missed.
- `ResolutionContext.in_ego_path` is never set true (no planner exists) and
  `predicted_risk_score` is always null (Phase 7 publishes no such score). Both are contract
  fields waiting on work that has not happened.
- Adaptive mapping is frame-local like Phase 6: no temporal fusion, no occlusion, no SLAM.
- CARLA is a boundary only; sensor/actor operations raise explicitly.

## 16. Architecture decisions

ADR-001 … ADR-048 in [`decisions/architecture-decisions.md`](decisions/architecture-decisions.md).
Most load-bearing for future work:

- **ADR-009** — coordinate convention: **+x forward, +y left, +z up**, metres, right-handed
- **ADR-005** — readiness *and* implementation status reported separately
- **ADR-023** — motion is `None` until measured; zero would claim a standstill
- **ADR-022 / ADR-024 / ADR-027** — module contracts return full result objects, not bare
  lists, and account for what they declined
- **ADR-025** — stateful services live on the `ApplicationContext`, never module globals
- **ADR-019** — benchmark data is synthetic and labelled as such
- **ADR-026** — constant-velocity prediction with heuristic, explicitly uncalibrated
  uncertainty
- **ADR-029** — resolution *policy* is separated from the mapper: `ResolutionContext`
  (inputs) → controller → `ResolutionDecision` (output) → mapper
- **ADR-030** — mapping is frame-local; nothing accumulates
- **ADR-031** — binary occupancy, and null height for unobserved cells
- **ADR-032** — a missing risk factor is dropped and the weights renormalise, never scored
  zero; `UNKNOWN` carries a null score
- **ADR-033** — uncertainty is reported beside risk, never folded into it
- **ADR-034** — unobserved map cells never reduce risk
- **ADR-036** — risk does not decide resolution
- **ADR-037** — an adaptive map is region-partitioned tiles, each with its own sub-grid
- **ADR-038** — detail priority is an engineering score, and unknown risk is not low risk
- **ADR-039** — asymmetric hysteresis plus a minimum dwell time stop resolution oscillating
- **ADR-040** — region and cell budgets coarsen the lowest-priority regions, and say so
- **ADR-041** — the controller reads positions from tracks, never from assessments
- **ADR-042** — CARLA is a data source behind an adapter boundary, never a second stack
- **ADR-043** — one coordinate conversion, at the boundary, in a module that imports no CARLA
- **ADR-044** — synchronous simulation and simulation-authoritative time
- **ADR-045** — ground truth is a separate path and never enters perception
- **ADR-046** — scenarios are generic declarative contracts, resolved once by an explicit seed
- **ADR-047** — scripted motion is timed constant-velocity segments, placed not simulated
- **ADR-048** — the runner drives a protocol extracted from the boundary, and runs once

## 17. What MUST NOT change

1. The coordinate convention (ADR-009).
2. Phase 1–7 algorithms, unless a measured defect justifies it.
3. Existing endpoint behaviour and response shapes — extend additively.
4. The honesty rules: no fabricated metrics; unmeasured values are `null` with a reason;
   `source` provenance is mandatory; baselines are labelled `is_baseline`; prediction
   uncertainty is never described as calibrated, probabilistic or validated.
5. Existing tests — never weaken or delete them. If a premise genuinely changes, retarget
   the test to guard the same property and say so.
6. `IMPLEMENTED` status is reserved for mature functionality; baselines are `PARTIAL`.
7. No new dependencies without an ADR.
8. Predicted positions live only in `PredictedTrajectory` and are never written back onto
   `TrackedObject.position`.
9. The mapper never chooses its own resolution, and is never handed tracks, trajectories,
   risk or uncertainty (ADR-029). Keep that interface narrow.
10. An unobserved map cell reports null height, never zero (ADR-031).
11. A missing risk factor is dropped, never scored zero; `UNKNOWN` carries a **null** score
    (ADR-032).
12. Uncertainty is never summed into the risk score (ADR-033), and map context never lowers
    it (ADR-034).
13. The risk engine never chooses spatial resolution and is never handed a cell size
    (ADR-036).
14. Nothing may describe the risk score as a probability, calibrated or validated.
15. The Phase 6 `FixedResolutionMapper` stays available and unchanged. It is the baseline the
    adaptive mapper is measured against, and a comparison needs both (ADR-003).
16. A region no object influences carries `detail_priority = None`, never 0.0, and an
    assessment with `risk_score is None` never has that coerced to zero (ADR-038).
17. Uncertainty stays an independent factor in the priority, never summed into risk.
18. The adaptive mapper never chooses a resolution: every cell size comes from the plan
    (ADR-029, ADR-037).
19. Regions partition the map exactly. No gap, no overlap, and `input == mapped +
    out_of_bounds` stays model-enforced.
20. Resolution must not oscillate: refinement immediate, coarsening earned (ADR-039).
21. Nothing may describe the detail priority as a probability, a safety margin, calibrated or
    validated.
22. CARLA stays optional: the backend starts, all endpoints respond and the suite passes with
    the package absent. No module outside `adaptx.carla` imports `carla` (ADR-042).
23. The coordinate conversion happens **once**, at the boundary, in a module that imports no
    simulator (ADR-043). Never scatter it downstream.
24. Frame timestamps come from simulation time, never the wall clock (ADR-044).
25. Ground truth never reaches detection, tracking, prediction, risk or adaptive resolution
    (ADR-045). It is for evaluation, and it stops being useful the moment it is an input.
26. A simulated frame is labelled `source=simulation` and is never presentable as
    `live_sensor`.
27. A scenario is **data**: `adaptx.scenarios.models` and `catalogue` import nothing from
    `adaptx.carla`, and a definition round-trips through JSON (ADR-046).
28. Every randomised scenario value comes from `random.Random(definition.seed)`, drawn once.
    Never the module-level generator, never a wall clock, never a UUID.
29. Actors are **placed** at their closed-form scripted pose each frame. Nothing integrates
    physics (ADR-047).
30. A `ScenarioRunResult` carries **no accuracy, precision, recall, error or match field**.
    It is evidence for Phase 11, not a Phase 11 result.
31. `carla/smoke.py` stays deleted. The scenario framework is the way to run a scene.

## 18–19. Next step — benchmarking and evaluation (Phase 11)

Everything Phase 11 needs now exists and has been kept apart on purpose. A scenario run
records, per frame, three things that have never been compared: what the scenario
**commanded** (`ExpectedPose`), what the simulator **reported** (`GroundTruthFrame`), and
what the pipeline **counted** (`StageCounts`). Phase 11 is the first phase allowed to put a
number between them.

That also makes it the first phase where an honest negative is likely. Every "unmeasured
and unmeasurable" limitation in §15 becomes measurable, and the baselines were built to be
replaced.

Objective and full handoff: [`NEXT_PHASE.md`](NEXT_PHASE.md).

## 20. Not yet

Do **not** start the dashboard (Phase 12), and do not start Phase 11 without a live CARLA
run first - see [`NEXT_PHASE.md`](NEXT_PHASE.md). Evaluating perception against ground truth
that has only ever come from a stand-in would measure the stand-in.

Event replay was **deferred** from Phase 10, not done. If it is picked up, the design
question from the Phase 9 handoff is still open: re-run the simulation, or re-play a
recording. They have different costs and different guarantees.

The dashboard design target is captured in [`UI_UX.md`](UI_UX.md) with a panel-by-panel
audit of what can actually be fed today. Phase 8 added the region decisions, level
distribution and fixed-versus-adaptive comparison; Phase 9 adds live simulation state and
ground truth.
