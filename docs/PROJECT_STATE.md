# ADAPT-X — Project State

**Primary context file for a fresh session.** Read this first, then
[`PHASE_HISTORY.md`](PHASE_HISTORY.md) for how it got here and
[`NEXT_PHASE.md`](NEXT_PHASE.md) for what to build next.

Snapshot taken 2026-09-10. The repository is the source of truth; if this file
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

**Phase 7 COMPLETE.** Phases 1, 2 (A/B/C), 3, 4, 5, 6 and 7 are implemented and verified.

**Phase numbering was resolved and changed in Phase 5.** Trajectory prediction moved from
Phase 8 to **Phase 5**, because it is what the risk engine needs next; 2.5D mapping, risk
and adaptive resolution each shifted one later (now 6, 7, 8). `CLAUDE.md`,
`docs/ROADMAP.md` and the `phase` field on every component in `services/system_service.py`
agree. Phase headings in [`PHASE_HISTORY.md`](PHASE_HISTORY.md) are a historical record and
were deliberately left as originally written.

## 3. Git checkpoint

Verified against `git` on 2026-09-10. **Everything through Phase 7 is committed, pushed and
merged.** There is no uncommitted work.

| | |
|---|---|
| Branch | `phase-7-risk-uncertainty` |
| HEAD | `a652622f00282755898c4bc9fa00880b9a5eb646` (`a652622`) |
| HEAD message | `feat: add deterministic risk and uncertainty engine` |
| Parent | `73f08e3` |
| `origin/main` | `5f68a5e` — *Merge pull request #4 from prakharsingh070/phase-7-risk-uncertainty* |
| Working tree | clean |
| HEAD vs `origin/main` | 0 ahead — HEAD is **contained in** `origin/main` |

Merged history, newest first:

```
5f68a5e  Merge PR #4  <- phase-7-risk-uncertainty
a652622  feat: add deterministic risk and uncertainty engine      (Phase 7)
43b6fc2  Merge PR #3  <- phase-6-spatial-mapping
73f08e3  Add dashboard design reference
dae152b  feat: add deterministic 2.5D spatial mapping baseline    (Phase 6)
8fb844d  feat: add trajectory prediction baseline                 (Phase 5)
0c3806d  Merge PR #2  <- phase-2a-lidar-preprocessing             (Phases 2-4)
14f5e3f  Merge PR #1  <- phase-1-foundation                       (Phase 1)
```

> **Local `main` is stale — 6 commits behind `origin/main`.** It still points at `0c3806d`.
> Before starting Phase 8, fast-forward it or branch from `origin/main`:
>
> ```bash
> git checkout main && git pull --ff-only origin main
> ```
>
> Branching Phase 8 off local `main` as it stands would silently lose Phases 5, 6 and 7.
> This has caught a session out before.

Phase branches `phase-5-trajectory-prediction` and `phase-6-spatial-mapping` still exist
locally and are fully merged; they can be deleted safely.

## 4. Architecture implemented

```
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
```

Mapping is a **parallel consumer** of the Phase 2 frame, not a stage after prediction. It
does not read detections, tracks or trajectories, and stays independently usable.

Every stage measures its own duration with `time.perf_counter`. Every result carries a
configuration snapshot so a record is self-describing.

### What each phase delivered

Full narrative per phase — objective, decisions, defects found, measurements — is in
[`PHASE_HISTORY.md`](PHASE_HISTORY.md). This table is the index.

| Phase | Status | Delivered | Key ADRs |
|---|---|---|---|
| **1** Foundation | Done | Package, typed settings, structured logging, exception→HTTP mapping, all data contracts, every module interface, FastAPI + `/ws/telemetry`, CARLA boundary (optional dep, real client + mock), Docker | 004, 005, 006, 007, 008 |
| **2A** LiDAR input | Done | Structural validation, NaN/Inf removal, ROI box, 3D range filter. NumPy masks over the original array, so counts partition the input exactly | 009, 010, 011 |
| **2B** Downsampling | Done | Voxelisation keeping a **real measured point**; per-cell-lowest ground segmentation; grid-approximated noise filter. All three **opt-in**. Guarded int64 quantiser after an overflow defect | 012, 013, 014, 015, 016 |
| **2C** Pipeline | Done | `LiDARProcessingPipeline` orchestrating 7 stages, per-stage measured timing, `overhead_ms` reported separately, synthetic benchmark datasets, fixed-resolution *processing* baseline | 017, 018, 019 |
| **3** Detection | Done | Grid connected-component clustering (not DBSCAN), dimension-band classification, `UNKNOWN` on ambiguity, rejected clusters reported with reasons | 020, 021, 022 |
| **4** Tracking | Done | Gated greedy nearest-neighbour association, velocity measured from frame timestamps, TENTATIVE/CONFIRMED/COASTING/LOST lifecycle, stateful service on the context | 023, 024, 025 |
| **5** Prediction | Done | Constant-velocity extrapolation `p + v·(age_s + t)`, 13 points over 3.0 s, heuristic uncertainty growing with time, `velocity is None` → **no trajectory** + recorded skip | 026, 027 |
| **6** Mapping | Done | Bounded dense XY grid, uniform cell size, binary occupancy, per-cell point count and min/max/mean height, **null** height where unobserved, full point accounting, frame-local | 028, 029, 030, 031 |
| **7** Risk | Done | Object-level risk from 3 factors as a weighted mean **over available factors**, `UNKNOWN` + null score, uncertainty reported **separately** with visible reasons, map context never lowers risk, aggregate by maximum | 032, 033, 034, 035, 036 |
| **8** Adaptive resolution | **NEXT** | Nothing. `ResolutionController` is an ABC with zero implementations | — |
| 9–12 | Future | CARLA, scenarios/replay, fixed-vs-adaptive evaluation, dashboard | — |

**Every implemented phase is a deterministic, explainable baseline behind an interface,
labelled `is_baseline`, with its failure modes asserted by tests.** None is a finished
subsystem, and no phase has added a dependency beyond the Phase 1 set.

## 5. Module structure

| Package | Contents |
|---|---|
| `adaptx.config` | `settings.py` — all typed settings |
| `adaptx.core` | `logging`, `exceptions`, `lifecycle` (service graph) |
| `adaptx.models` | All data contracts (see §6) |
| `adaptx.perception` | `pipeline` (orchestrator), `voxel`, `ground`, `noise`, `grid`, `clustering`, `classification`, `detector`, `lidar`, `interfaces` |
| `adaptx.tracking` | `tracker`, `association`, `interfaces` |
| `adaptx.prediction` | `constant_velocity`, `interfaces` |
| `adaptx.mapping` | `grid_mapper` (fixed-resolution baseline), `interfaces` |
| `adaptx.risk` | `heuristic` (Phase 7 engine), `baseline` (proximity-only comparison reference), `interfaces` |
| `adaptx.carla` | Boundary: interface, real client, mock. Optional dependency |
| `adaptx.services` | `lidar_service`, `metrics_service`, `carla_service`, `system_service`, `tracking_service`, `prediction_service`, `mapping_service`, `risk_service` |
| `adaptx.api` | `app`, `schemas`, `dependencies`, `routes/`, `websocket/` |
| `adaptx.benchmark` | `datasets`, `baseline`, `runner`, `detection`, `tracking`, `prediction`, `mapping`, `risk`, `models` |

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
| `ResolutionDecision`, `ResolutionSource` | The **output** of a resolution decision (`resolution_m`, `source`, `reason`). `FIXED` only in Phase 6 |
| `AdaptiveMap`, `AdaptiveMapCell` | Pre-existing contract, now populated by projecting **occupied cells only** out of a `SpatialMap` |
| `ResolutionContext` | The **inputs** to a resolution decision. Still contract-only — consumed by a controller that does not exist (ADR-029) |
| `RiskAssessment`, `RiskAssessmentResult` | Phase 7 per-object risk. `risk_score` is **`None` exactly when `risk_level` is `UNKNOWN`** (ADR-032) - nothing is invented |
| `UncertaintyBreakdown` | Heuristic scalar **plus its reasons**. Reported beside risk, never folded into it (ADR-033) |
| `TrajectoryRelevance`, `MapContext` | Closest predicted approach; what the map recorded at the object's cell. Map context **never lowers** risk (ADR-034) |
| `RiskLevel` | `LOW/MEDIUM/HIGH/CRITICAL` are scored bands; **`UNKNOWN` is not a point on the scale** and has no threshold |
| `RiskField`, `RiskCell`, `ObjectRisk` | Phase 1 contracts. `ObjectRisk` is now populated via `evaluate()`; `RiskCell` is still unused - Phase 7 is object-level only |
| `SystemStatus`, `ComponentStatus`, `SystemMetrics` | Readiness **and** implementation status (ADR-005) |

Shared rules: `schema_version`, tz-aware UTC timestamps, `coordinate_frame`, `source`
(`live_sensor`/`simulation`/`replay`/`synthetic_test`/`unavailable`), `extra="forbid"`.

## 7. API endpoints (15)

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
| POST | `/api/v1/tracking/reset` |
| GET | `/api/v1/tracking/status` |
| WS | `/ws/telemetry` |

## 8. Configuration sections

`ADAPTX_` prefix, `__` nesting. Sections: `APP`, `API`, `LOGGING`, `CARLA`, `LIDAR`
(2A bounds + opt-in 2B stages), `DETECTION`, `TRACKING`, `PREDICTION`, `MAP` (levels
vocabulary **and** Phase 6 mapping geometry), `RISK` (thresholds **and** Phase 7 heuristics), `WEBSOCKET`.

`LIDAR`, `PREDICTION`, `MAP`, `RISK` and `WEBSOCKET` are documented in `.env.example`.
**`DETECTION` and `TRACKING` are not** — a gap left by Phases 3 and 4, still open.

Defaults worth knowing: detection `cluster_tolerance_m=0.5`, `min_cluster_points=10`;
tracking `max_association_distance_m=2.5`, `min_hits_to_confirm=3`, `max_missed_frames=3`,
`velocity_smoothing=0.5`, `max_timestep_s=2.0`, `min_speed_for_heading_mps=0.3`;
prediction `horizon_s=3.0`, `interval_s=0.25` (13 points, t+0 inclusive), `max_tracks=256`,
`max_speed_mps=80.0`, `base_uncertainty_m=0.5`, `uncertainty_growth_mps=0.5`,
`confidence_hits_full=3`; mapping `resolution_m=0.5`, bounds ±60 m (240x240 = 57,600 cells), `min/max_resolution_m=0.05/5.0`, `max_cells=4,000,000`; risk proximity `5.0`-`40.0 m`, `closing_speed_high_mps=15.0`, weights `0.50/0.25/0.25`, thresholds `0.35/0.60/0.85`, `stale_observation_s=0.5`.

## 9. Telemetry (`/ws/telemetry`)

`hello` then periodic `telemetry`. Payload provides `system`, `metrics`, `detection`,
`tracking`, `prediction`, `mapping`, `risk` — the last five are **summaries**
(configuration and counts), never per-frame geometry, trajectory points, map cells, full
assessments or point arrays. Risk carries scene counts plus at most five ranked objects.
`not_yet_available` currently lists `risk_field` and `adaptive_map`, both deliberately:
object-level risk exists but a per-cell risk **field** does not, and a fixed-resolution map
exists but an **adaptive** one does not.

## 10. Lifecycle

`ApplicationContext` (`core/lifecycle.py`) holds: `settings`, `metrics`, `lidar`, `carla`,
`system`, `risk_engine` (Phase 1 proximity baseline), `preprocessor`, `detector`, `tracking`, `prediction`, `mapping`, `risk` (Phase 7 service). Built by
`build_context()`, attached to `app.state`, reached through FastAPI dependencies.

**Tracking is the only stateful perception component** (ADR-025). One tracker per process,
lock-guarded, cleared on shutdown. Concurrent clients share one track set.

`PredictionService`, `MappingService` and `RiskService` are **stateless with respect to
perception**: each holds counters for status and telemetry only, and resetting one changes
what the status endpoint reports, never what it produces. Mapping is frame-local by
construction (ADR-030) — map `N` cannot contaminate map `N+1` because nothing is retained.

## 11. Dependencies

Runtime: `fastapi`, `uvicorn[standard]`, `pydantic>=2`, `pydantic-settings`, `numpy`, `psutil`.
Dev: `pytest`, `pytest-asyncio`, `httpx`, `ruff`, `mypy`.
Optional extras declared but **not installed**: `open3d` (`[pointcloud]`), `carla` (`[carla]`).

**No SciPy, no ML framework, no database.** Declined on record in ADR-014, 020, 024 and
026. Phases 5, 6 and 7 each added none — seven phases, zero beyond the Phase 1 set.

## 12–14. Verification status

- **1021 tests pass** (`pytest`)
- `ruff check .` — All checks passed
- `ruff format --check .` — 161 files formatted
- `mypy src` — no issues in 91 source files
- Backend starts; all 15 endpoints respond; no tracebacks
- Live temporal check: a vehicle advancing 1 m per 0.5 s measured 2.000 m/s, and its
  trajectory advanced +1 m at t+0.5, +2 m at t+1, +4 m at t+2 and +6 m at t+3; uncertainty
  rose 0.5 → 2.0 m; the track's first frame produced **no trajectory** and an explicit
  `insufficient_velocity` skip

Benchmarks (`python -m adaptx.benchmark [--detect|--track|--predict|--map|--risk]`) — all
synthetic, **speed only**: pipeline ~213 ms/100k points; detection ~3.4 ms at that size;
tracking ~10.6 ms at 100 objects; prediction ~27 ms at 100 tracks (13 points each); mapping
~17 ms at 98k points and 1.0 m cells, rising to ~52 ms at 0.25 m; risk ~7.4 ms at 100
objects (~74 µs each). Measured results in
[`experiments/experiment-log.md`](experiments/experiment-log.md).

### Architectural integrity checks

Re-run these before and after any Phase 8 work. They encode the boundaries the project's
central claim depends on, and all six passed at commit `a652622`:

| Check | How to verify | Status at `a652622` |
|---|---|---|
| No `ResolutionController` leakage into risk | AST scan of `risk/`, `models/risk_assessment.py`, `services/risk_service.py`, `api/routes/risk.py` for `ResolutionController`/`ResolutionDecision`/`ResolutionLevel`/`MapSettings`/`AdaptiveMapper` | **clean** |
| Risk does not decide resolution | No resolution/cell field on `RiskAssessment`, `RiskAssessmentResult`, `RiskConfiguration`; `GET /api/v1/risk/status` reports `decides_resolution: false` | **clean** |
| `risk_score is None` for `UNKNOWN` | Assess a `LOST` track → `risk_level == UNKNOWN`, `risk_score is None` | **holds** |
| Uncertainty separate from risk | Two tracks identical but for observability → **same** `risk_score`, different `uncertainty.score` | **holds** (0.5714 vs 0.5714; 0.25 vs 0.45) |
| Unobserved cells never reduce risk | Same track with no map vs an empty-cell map → score never lower | **holds** (0.5714 → 0.5714) |
| `FixedResolutionMapper` intact | `git diff origin/main -- src/adaptx/mapping/` | **identical** |

The one legitimate exception: `risk/heuristic.py` reads `SpatialMap.resolution_m` to find
which cell an object occupies. That is cell **lookup**, not resolution **selection**, and it
lives in a `staticmethod` receiving only `(track, spatial_map)` — structurally unable to see
a risk value.

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
- **Mapping is one uniform cell size everywhere.** Adaptive resolution is not
  implemented, so no region receives more detail than another. Measured occupancy falls to
  1-16% at 0.25 m (Experiment 005): a fine uniform map spends most of its cells recording
  that nothing was observed.
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
- Risk quality is bounded by tracking and prediction, which are themselves baselines.
- CARLA is a boundary only; sensor/actor operations raise explicitly.

## 16. Architecture decisions

ADR-001 … ADR-036 in [`decisions/architecture-decisions.md`](decisions/architecture-decisions.md).
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

## 18–19. Next step — adaptive resolution (Phase 8)

The agreed next work item is the **resolution controller**, and it is where the project's
central claim finally gets tested. Phase 6 built the fixed-resolution mapper; Phase 7
produced risk and uncertainty. Phase 8 implements
`mapping.interfaces.ResolutionController`, feeding `ResolutionContext` from Phase 7
assessments and producing a `ResolutionDecision` the existing mapper already applies
unchanged. Objective and full handoff: [`NEXT_PHASE.md`](NEXT_PHASE.md).

## 20. Not yet

Do **not** start CARLA scenarios (Phase 9), scenario generation and replay (Phase 10) or
the dashboard (Phase 12). Phase 11 benchmarking depends on Phase 8 existing first: the
fixed-versus-adaptive comparison cannot be run until an adaptive mapper exists.
The dashboard design target is captured in [`UI_UX.md`](UI_UX.md) with a panel-by-panel
audit of what can actually be fed today.
