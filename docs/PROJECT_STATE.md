# ADAPT-X — Project State

**Primary context file for a fresh session.** Read this first, then
[`PHASE_HISTORY.md`](PHASE_HISTORY.md) for how it got here and
[`NEXT_PHASE.md`](NEXT_PHASE.md) for what to build next.

Snapshot taken 2026-09-12 (Phase 12 + live simulation extension + live perception upgrade). The repository is the source of truth; if this file
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

**Phase 12 COMPLETE** (dashboard). Phases 1 through 12 are implemented and verified.
The dashboard is a static browser console served by the backend at `/dashboard`: a live
3-D / top-down scene of what the pipeline last produced, an object inspector, the adaptive
map, and a viewer for stored Phase 11 evaluation reports and recorded runs with frame
playback. **It computes nothing** - every number is a backend field - and it has no control
over the simulator (ADR-055, [`DASHBOARD.md`](DASHBOARD.md)).

**Post-Phase-12 live simulation extension (not a Phase 13; ADR-056, Experiment 014).** A
long-lived CARLA session behind the dashboard: a driven ego with a real LiDAR, every frame
through the unchanged Phase 2-8 chain, a **baseline speed governor** (`adaptx.control`)
that reads the risk, tracking and prediction outputs and the ego's own odometry - never
ground truth - and drives the ego, seeded Traffic Manager traffic plus timed scripted
actors, a collision sensor as a safety fallback, an ego camera for display, five
high-level session controls (start / pause / resume / stop / reset) and nothing that
touches an actor over HTTP. **Measured live:** the ego slows for a parked car the pipeline
detects, holds ~7.7 m short of it, resumes when it leaves, 0 collisions in every run; the
loop runs at ~0.4x wall-clock speed and the header says LAGGING. Routing, planning,
ego-motion compensation and a spatial risk field remain NOT IMPLEMENTED.

**Live perception upgrade (ADR-057, Experiment 015).** Every live object now carries a
backend-built record - class, tracking state, planar distance, longitudinal and lateral
distance, ego-relative speed, closing speed, risk level and score, IN_PATH / CROSSING /
BEHIND / OUTSIDE against the controller's own corridor, fit-score confidence - and the
dashboard draws it as cards, scene labels and the inspector. Three measured perception
fixes: the detector rejects clusters whose bottom is more than 0.8 m above the road the
ground stage found (94 % of the false "pedestrians" were overhead signs and foliage), a
track's label lapses after three UNKNOWN observations, and the vehicle band accepts a
car's rear face (the parked car is now VEHICLE from ~12 m; it had never been one).
Tentative tracks survive two misses (the parked car's id switched 5 times, now 2). Tile
geometry is memoised: loop 160 -> 120 ms per frame. **Not** supported: a riderless
CARLA bicycle is mostly UNKNOWN/OBSTACLE and switches identity while crossing.
Adaptive resolution — the thing the project is named for — exists as a deterministic
heuristic baseline and has now been measured on live CARLA scenes against the fixed
baseline **and** against ground truth (Experiment 011): 0.48–0.56 of the fixed map's cells,
5× its build time, finer cells under the perceived actor than elsewhere. The perception
baselines were measured for the first time and lost, as expected: see §15.

**Phase numbering was resolved and changed in Phase 5.** Trajectory prediction moved from
Phase 8 to **Phase 5**, because it is what the risk engine needs next; 2.5D mapping, risk
and adaptive resolution each shifted one later (now 6, 7, 8). `CLAUDE.md`,
`docs/ROADMAP.md` and the `phase` field on every component in `services/system_service.py`
agree. Phase headings in [`PHASE_HISTORY.md`](PHASE_HISTORY.md) are a historical record and
were deliberately left as originally written.

## 3. Branch and status

- Branch: `live-perception-upgrade`, branched from `phase-12-dashboard` at `24cb73e`
- Phases 1 through 11 are committed **and merged into `origin/main`** (PR #1 through PR #8)
- Phase 12 and the live simulation extension are commit `24cb73e` on
  `phase-12-dashboard`, pushed, **not yet merged**
- The live perception upgrade is in the working tree on this branch, **not committed and
  not pushed**

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

**Phase 12 sits after all of that as a consumer:**

```
full-chain endpoint  ──┐
scenario CLI --publish ┘→ SceneSnapshot (outputs + point sample, NO ground truth)
                          → SceneService (latest + sequence) → /ws/scene → browser canvas
reports/*.json (EvaluationReport)  ─┐
runs/*.json (ScenarioRunResult)    ─┘→ EvidenceService (read-only, validated, per-frame)
                                     → /api/v1/reports, /api/v1/runs → browser
```

The dashboard never asks the backend to process, evaluate or simulate anything, except
through the five high-level live-session controls (start / pause / resume / stop / reset),
which start a catalogue scenario inside the process and never touch an actor directly.

**The live loop (post-Phase-12 extension) closes the circle:**

```
CARLA ──► CarlaSimulationSession(drive_ego) ──► RawPointCloudFrame ──► pipeline_processor (unchanged)
   ▲                                                                          │ risk, tracking, prediction
   └── apply_ego_control ◄── RiskGovernedSpeedPolicy.decide(outputs, ego odometry) ◄──┘
                                          └──► SceneSnapshot(+ego, control, live timing) ──► /ws/scene
```

`adaptx.live` owns the loop and is the only caller of `world.tick()` while it runs; the
loop never calls `session.ground_truth()`.

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
| `adaptx.scenarios` | `models` (definition, motion, resolution - imports nothing from `adaptx.carla`), `result` (raw evidence, no metrics; since Phase 11 also the pipeline's result contracts per frame and the sensor configuration), `interfaces` (`ScenarioSimulator` protocol), `runner` (since Phase 12 accepts a per-frame observer), `publish` (Phase 12: posts each frame's outputs to a backend, never ground truth), `catalogue` (4 scenarios), `__main__` (CLI, `--publish URL`) |
| `adaptx.evaluation` | Phase 11, **the only package that reads ground truth**: `dataset` (record → evaluation view, sensor-frame conversion, finite-difference reference velocity), `matching` (greedy gated), `tracking` (detection + tracking), `prediction` (ADE/FDE), `risk` (proximity events), `mapping` (workload; accuracy explicitly unavailable), `adaptive` (paired fixed vs adaptive, churn, refinement lead), `resource`, `evaluator`, `report` (text), `compare` (repeatability), `models` (report contracts), `__main__` (CLI). Offline; imports no simulator |
| `adaptx.carla` | Boundary: `client` (connection), `session` (deterministic simulation lifecycle), `conversion` (the single coordinate/time boundary, imports no simulator), `ground_truth`, `interfaces`, `mock`. Optional dependency. `smoke.py` was deleted in Phase 10 |
| `adaptx.services` | `lidar_service`, `metrics_service`, `carla_service`, `system_service`, `tracking_service`, `prediction_service`, `mapping_service`, `risk_service`, `adaptive_mapping_service`, `scene_service` (Phase 12: builds and keeps the latest `SceneSnapshot`) |
| `adaptx.control` | Live extension: `models` (`ControlCommand`, `EgoObservation`, `ControlConfiguration`, `ControllerState`), `interfaces` (`VehicleController`), `policy` (`RiskGovernedSpeedPolicy`, pure, reads no ground truth), `corridor` (the shared IN_PATH / CROSSING / BEHIND / OUTSIDE rule, ADR-057) |
| `adaptx.live` | Live extension: `models` (`LiveState`, `LiveStatus`, `LiveEvent`, `LiveTiming`, `LiveFrameInfo`, scenario contracts), `scenarios` (six-entry catalogue, `resolve_live`, anchored `LiveScenarioManager`), `service` (`LiveSimulationService`: the loop, controls, events, camera PNG), `camera` (PNG encoder) |
| `adaptx.evidence` | Phase 12, **downstream of evaluation and outside every pipeline package**: `service` reads stored reports and runs, validates them, serves them read-only (runs frame by frame, small LRU) and compares reports through the Phase 11 contract |
| `adaptx.api` | `app` (also mounts `dashboard/` at `/dashboard`), `schemas`, `dependencies`, `routes/` (+ `scene`, `evidence` in Phase 12), `websocket/` (+ `scene`) |
| `dashboard/` (repo root, not a Python package) | Static ES-module console: `app.js`, `ui.js`, `data/` (api, channel, evidence stores, format, normalise), `render/` (projection, scene, charts), `views/` (8 views + viewport), `tests/` (`node --test`) |
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
| `SceneSnapshot`, `PointSample`, `PointStage` | Phase 12: one frame's outputs bundled for display - tracks, trajectories, assessments, tiles, budget, map summaries, comparison, stage timings and a deterministic stride sample of points (`is_downsampled`, `stride`, `stage`). **Has no ground-truth field and refuses one**; a trajectory or assessment without its track is refused |
| `RunSummary`, `RunFrame`, `StoredFile` (`adaptx.evidence`) | A recorded run without its frames; one frame with its ground truth beside it (the only path that serves ground truth); a listed file |

Shared rules: `schema_version`, tz-aware UTC timestamps, `coordinate_frame`, `source`
(`live_sensor`/`simulation`/`replay`/`synthetic_test`/`unavailable`), `extra="forbid"`.

## 7. API endpoints (33 HTTP + `/` + 2 WebSockets; Phase 12 added 7 + 1, the live extension 9, all additive)

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
| GET | `/api/v1/scene/latest` — the last `SceneSnapshot`, or `has_scene=false` (Phase 12) |
| POST | `/api/v1/scene/frame` — a producer hands over a snapshot it already produced; nothing is computed (Phase 12) |
| GET | `/api/v1/reports`, `/api/v1/reports/{name}`, `/api/v1/reports/compare?first=&second=` — stored `EvaluationReport`s, read-only (Phase 12) |
| GET | `/api/v1/runs`, `/api/v1/runs/{name}`, `/api/v1/runs/{name}/frames/{index}` — stored runs: summary and one frame at a time (Phase 12) |
| WS | `/ws/telemetry` |
| WS | `/ws/scene` — pushes the latest snapshot when its sequence changes (Phase 12) |
| GET | `/api/v1/live/status`, `/api/v1/live/scenarios`, `/api/v1/live/events?since=`, `/api/v1/live/camera` — live session state, catalogue, events, ego camera PNG (live extension) |
| POST | `/api/v1/live/start` `{scenario_id?, seed?}`, `/api/v1/live/pause`, `/resume`, `/stop`, `/reset` — the five high-level session controls, allowlisted by full path; `ADAPTX_LIVE__CONTROLS_ENABLED=false` refuses them (live extension) |
| — | `/dashboard/` static console; `/` redirects to it (Phase 12) |

**No endpoint spawns, destroys, moves, ticks or drives an actor**; a test audits every
route path for those words and allowlists exactly the five session controls.

## 8. Configuration sections

`ADAPTX_` prefix, `__` nesting. Sections: `APP`, `API`, `LOGGING`, `CARLA`, `LIDAR`
(2A bounds + opt-in 2B stages), `DETECTION`, `TRACKING`, `PREDICTION`, `MAP` (levels
vocabulary **and** Phase 6 mapping geometry), `ADAPTIVE` (Phase 8 resolution policy),
`RISK` (thresholds **and** Phase 7 heuristics), `WEBSOCKET`, `DASHBOARD` (Phase 12:
`report_dir`, `run_dir`, `scene_max_points`, `run_cache_size`), `CONTROL` (live extension:
the baseline governor's speeds, distances, limits, dwell, hold throttle) and `LIVE`
(controls switch, default scenario, camera, collision sensor, Traffic Manager port, lag
ratio). All documented in `.env.example`.

`LIDAR`, `PREDICTION`, `MAP`, `ADAPTIVE`, `RISK`, `WEBSOCKET` and `DASHBOARD` are documented in `.env.example`.
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
`system`, `risk_engine` (Phase 1 proximity baseline), `preprocessor`, `detector`, `tracking`, `prediction`, `mapping`, `risk` (Phase 7), `adaptive_mapping` (Phase 8), `scene` and `evidence` (Phase 12), `live` (the live session service, bound to the context after construction). Built by
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
026. Phases 5 through 12 each added none — twelve phases, zero beyond the Phase 1 set.
The dashboard has **no** JavaScript dependency and no build step; its unit tests use Node's
built-in runner (Node 22 was present; it is a dev convenience, not a runtime need).

## 12–14. Verification status

- **1684 tests pass**, 12 deselected (`pytest`, Python 3.13). The 12 are the live CARLA tests
- `pytest -m carla` — **12 passed** live against CARLA 0.9.16 from `.venv312` (2026-09-12),
  including the perception acceptance cases: the parked car is a VEHICLE with at most two
  ids and a real distance, the crossing walker is a PEDESTRIAN whose lateral distance and
  path relation change,
  including the live loop: the ego drives under the controller, the obstacle-stop demo
  holds short of the parked car with no contact, and the session leaves no actor behind
  and restores the world
- `ruff check .` — All checks passed
- `ruff format --check .` — 223 files formatted
- `mypy src` — no issues in 142 source files
- `node --test` in `dashboard/` — 20 pass (format incl. labels, normalise incl. records, projection)
- **Phase 12 live validation (2026-09-11):** `python -m adaptx.scenarios run cyclist_crossing
  --publish http://127.0.0.1:8000` against CARLA 0.9.16 published 80 frames; the browser
  drew the streaming point sample, tiles, risk-coloured boxes and predicted paths, selection
  filled the inspector, a stored report and a 42 MB run played back at the recorded timestep,
  the backend was stopped and restarted with the page open and every channel reconnected on
  its own. Browser costs are in Experiment 013
- **Live loop validation (2026-09-12, Experiment 014):** from the dashboard's Start button
  and from a probe script, `static_obstacle`, `mixed_obstacles` (6 Traffic Manager
  vehicles) and `pedestrian_crossing` ran against CARLA 0.9.16 on Town10HD_Opt; the ego
  reached 5.1 m/s, slowed, held 6.3-7.7 m short of the in-path object, resumed after it
  left; 0 collisions; the header showed CARLA CONNECTED, the session pill, and PIPELINE
  LAGGING at 0.31x; the Front View showed the ego camera; Stop from the UI left no actor
  on the server and restored the world settings. Loop median 150-164 ms per 50 ms step
- **Phase 11 live experiments** (Experiment 011): all four catalogue scenarios recorded
  twice on CARLA 0.9.16 from spawn point 1 and evaluated offline; figures in §15 and the
  experiment log. Two stale actors from an earlier killed run were found on the server and
  destroyed first; the "spawn point 0 refuses" finding of Experiment 010 was a consequence
  of them and is corrected in place
- Backend starts; all 33 API endpoints and both WebSockets respond; no tracebacks
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

- **Measured against simulator ground truth for the first time (Experiment 011), and the
  baselines lost.** Simulation evidence only, one map, one pose, one seed:
  - Vehicle detection recall 0.00–0.06 at a 1 m gate, 0.23–0.61 at 2 m, with a consistent
    ~1.5–1.7 m planar offset between detection centroid and actor origin. The classifier
    never labelled the Audi a vehicle (class agreement 0.00 on 54 matched frames).
  - Tracking coverage 0.25–0.48 for moving actors with 1–3 identity switches each; the
    stationary waiting vehicle reached 0.98 coverage and still switched once. Velocity
    error median 0.1–0.8 m/s with outlying frames that measured a standstill for an
    object closing at 8 m/s.
  - ADE 1.5–3.4 m mean over the trajectories that could be aligned, growing with horizon
    to 5 m (cyclist, 3 s) and 12 m (approach, 1.5 s).
  - The risk score orders proximity for moving actors (concordance 0.80–0.86) and not for
    a stationary one; at the pre-registered 20 m band no scenario actor was inside long
    enough to measure alert recall (a post-hoc 25 m band gave 0.13–0.63).
  - **Re-measured after the placement fix (ADR-054, Experiment 012).** With physics off on
    placed actors and the ego grounded, the walker stands on the road, nothing settles, and
    a static scene is bit-repeatable; scenes with moving placed actors still differ by ≤ 8
    of 27,000 points on some frames (unexplained). Under the **process defaults** (ground
    segmentation OFF) the parked car (20 m) and the pedestrian (15 m) are detected on **0**
    frames — their returns are clustered with the road — so Experiment 011's parked-car
    recall was an artefact of the car falling. With ground segmentation **ON** (a disclosed
    post-hoc variation, `ADAPTX_LIDAR__GROUND_ENABLED=true`) both are detected on every
    frame (pedestrian 0.10 m; parked car at a constant 1.70 m centroid-to-origin offset),
    no scenario shows an identity switch, pedestrian ADE is 0.66 m and the pipeline runs at
    80 ms/frame instead of 140. **Every correctness figure depends on that one default**,
    which this phase does not decide.
  - **Still unmeasured:** mapping occupancy accuracy (no honest reference), precision or
    any false-positive figure (static geometry is unlabelled), peak memory, anything with a
    moving ego, traffic or weather, and anything real-world.
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
- **The dashboard shows the last frame only** - no history, no replay of a live session.
  The live point cloud is a stride sample (default at most 6000 points) of what the pipeline
  processed; playback of a recorded run has no point cloud at all, because records carry
  none. Per-cell occupancy is not streamed. A random seek during playback waits behind the
  frames being prefetched (~200 ms, Experiment 013) because the backend handlers are
  synchronous. Routing, decision, planning and a spatial risk field are shown as *Not
  implemented*; GPU as *Not measured*. The frontend "computes nothing" rule is enforced by a
  source scan, which is a guard, not a proof.
- **The live loop is slow and says so.** ~120 ms per 50 ms frame on the validation
  machine (Experiment 015; 160 ms before the tile memoisation), ~0.42x wall-clock speed,
  reported as `PIPELINE LAGGING`; with the dashboard open in the same process slower
  still. The simulation is consistent (synchronous mode), just slow-motion. The next
  hotspot is the adaptive mapper's per-tile model construction.
- **Classification is geometric and says so.** Bands on cluster extents plus an
  elevated-cluster filter and label decay (ADR-057): a walker is PEDESTRIAN in ~70 % of its
  tracked frames, a parked car VEHICLE from ~12 m and OBSTACLE beyond, a riderless bicycle
  mostly UNKNOWN/OBSTACLE with identity churn while crossing, a bollard can read
  PEDESTRIAN, a bus shelter's side VEHICLE. Confidence is a fit score. Phase 11's
  evaluation reports were measured under the old detector and tracker defaults and have
  not been re-run under the new ones.
- **The controller is a baseline, and the perception it obeys is the baseline it is.**
  Velocities are ego-relative (no ego-motion compensation), so a moving ego sees every
  static object "approach" and the risk engine's closing-speed factor rises everywhere;
  the governor keys speed on objects in a straight +X corridor, so on a bend roadside
  geometry stops the ego. Track identities on the parked car switch every few frames
  (Experiments 011/012), so the level flickers; the setpoint ramp hides the lurch, not the
  churn. Poles are labelled "pedestrian", cars "obstacle". Not tuned, not validated, not
  collision-free by claim - the safety sensor counts what it fails to prevent (0 in the
  measured runs).
- **Actor hygiene depends on a clean stop.** A killed backend leaves actors on the server;
  a server left synchronous shows a fresh client a stale actor list. `Stop` (or process
  shutdown) cleans up; a crash does not.

## 16. Architecture decisions

ADR-001 … ADR-057 in [`decisions/architecture-decisions.md`](decisions/architecture-decisions.md).
Most load-bearing for future work:

- **ADR-050** — evaluation is offline from the run record, which carries the pipeline's
  outputs; never re-run perception to evaluate
- **ADR-051** — a metric that could not be computed is null with a reason, never zero
- **ADR-052** — greedy gated matching at several gates; no precision over unlabelled tracks;
  reference velocity by finite difference, never the simulator's
- **ADR-053** — fixed vs adaptive is paired within one run; every report is simulation
  evidence with fixed limitations
- **ADR-054** — placed actors do not simulate physics and stand on the ego's ground plane
- **ADR-055** — the dashboard is a consumer: scene snapshots without ground truth, stored
  evidence served read-only from a package outside the pipeline, Canvas rendering, no
  control endpoints, no new dependency
- **ADR-056** — a long-lived live session drives the ego from the pipeline's own outputs:
  the loop owns the tick, runs the unchanged chain, a baseline speed governor keyed on
  in-path risk reads no ground truth, five high-level session controls and no actor
  endpoint, shutdown order that survives the Traffic Manager
- **ADR-057** — objects are labelled once, in the backend: floor-referenced elevated
  filter in the detector, class decay in the tracker, a partial-view vehicle band, one
  corridor rule shared by the governor and the snapshot, and a per-object record the
  dashboard only formats

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
11. Ground truth reaches `adaptx.evaluation` and nothing else; no production package imports
    the evaluation layer (ADR-045, ADR-050). Two tests assert it.
12. The run record carries no metric; evaluation produces its own report (ADR-050).
13. Nothing in Phases 2–8 is tuned in response to an evaluation result without its own
    experiment entry stating the before and after.
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
32. **The dashboard computes nothing** (ADR-055): no risk, trajectory, resolution, match,
    distance, rate or metric in the browser; `data/format.js` is the only place null becomes
    "Not available". A source scan asserts it - keep the scan passing, do not widen its
    allowlist.
33. A `SceneSnapshot` has no ground-truth field. Ground truth is served only on
    `/api/v1/runs/{name}/frames/{i}` and shown only in the Run / Scenario view, labelled.
34. No endpoint spawns, destroys, moves, starts or stops anything; the scenario CLI is the
    only way to run a scene. A route audit asserts it.
35. `adaptx.evidence` is the one place outside `adaptx.evaluation` that imports it, and
    `api/routes/evidence.py` the one API module; the Phase 11 boundary test pins both.
36. The dashboard never displays "collision probability", "safe", "production ready" or
    "real-time"; UNKNOWN stays UNKNOWN; null is "Not available"; unobserved is never free.
37. **The live loop never reads ground truth** (ADR-056): no call to `session.ground_truth()`
    in `adaptx.live`, no ground-truth import in `adaptx.control` or `adaptx.live` (the
    Phase 11 boundary tests cover both packages), and the controller's signature takes the
    pipeline's outputs and the ego's own odometry only.
38. Exactly one component ticks the world: the live loop's thread while a session runs,
    the Phase 10 runner while a scenario runs, never both.
39. The five session controls are the only non-read live endpoints; the route audit
    allowlists them by full path. No endpoint spawns, destroys, moves, ticks or drives an
    actor.
40. The live session is stopped, not killed: `close()` restores the world (asynchronous
    first when a Traffic Manager exists), releases the TM, then destroys - the order
    measured to not abort the client.
41. Nothing calls the controller "autonomous driving", "safe" or "collision-free"; it is a
    baseline speed governor and every threshold carries the word.
42. The object record is built by `object_records()` in the backend and nowhere else;
    the corridor rule lives in `control.corridor` and nowhere else. The frontend formats
    (ADR-057). A new per-object figure is a backend field first.
43. The detector's floor comes from the ground stage's own output (`floor_estimate_m`),
    never from the sensor mount setting and never from the simulator.

## 18–19. Next step

**The roadmap defines no Phase 13.** Phase 12 was "dashboard and final integration" and
every numbered phase is done as a baseline; the live simulation extension is an extension
of Phase 12, not a new phase. What remains is on record: the ground-segmentation default
decision (Experiment 012, a Phase 2/3 experiment before any more evaluation runs are
read; the live demo already has to run with it on), the deferred "B" items under each
phase in [`ROADMAP.md`](ROADMAP.md), event replay deferred from Phase 10, and - new from
Experiment 014 - ego-motion compensation (velocities are ego-relative, which a moving ego
now exposes) and a corridor that follows the road rather than +X. Nothing here should be
started as a side effect of another change.

Full handoff: [`NEXT_PHASE.md`](NEXT_PHASE.md).

## 20. Not yet

Do **not** tune Phases 2–8 against Experiment 011's figures without a new experiment entry
stating the before and after. Do **not** build an occupancy reference by hand; if one is
built from simulator ray casts it lives in `adaptx.evaluation`, is labelled a simulation
reference, and never reaches perception. Do **not** report a precision figure while static
geometry is unlabelled.

Event replay was **deferred** from Phase 10, not done. If it is picked up, the design
question from the Phase 9 handoff is still open: re-run the simulation, or re-play a
recording. They have different costs and different guarantees.

Do **not** add a dashboard endpoint that runs perception, evaluation or a benchmark on
request, and do **not** add actor-level simulator control (the five session controls are
the whole surface). Do **not** move a computation into the browser to make a panel richer;
if a figure is wanted, the backend or the evaluation layer produces it under a contract
and the dashboard shows it. Do **not** let the controller read ground truth "for the demo",
and do **not** tune it until an experiment entry says what changed and why.
