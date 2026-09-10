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

**Phase 5 COMPLETE.** Phases 1, 2 (A/B/C), 3, 4 and 5 are implemented and verified.

**Phase numbering was resolved and changed in Phase 5.** Trajectory prediction moved from
Phase 8 to **Phase 5**, because it is what the risk engine needs next; 2.5D mapping, risk
and adaptive resolution each shifted one later (now 6, 7, 8). `CLAUDE.md`,
`docs/ROADMAP.md` and the `phase` field on every component in `services/system_service.py`
agree. Phase headings in [`PHASE_HISTORY.md`](PHASE_HISTORY.md) are a historical record and
were deliberately left as originally written.

## 3. Branch and status

- Branch: `main`, in sync with `origin/main`
- Phases 1 through 4 are committed and merged (PR #1 and PR #2)
- Phase 5 is in the working tree

> A previous version of this file claimed Phases 2B–4 were uncommitted and unpushed. That
> was stale: they are commit `e3677dd`, merged as PR #2.

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
```

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
| `adaptx.mapping` | **Interfaces only — nothing implemented** |
| `adaptx.risk` | Contract + `baseline.py` (proximity-only, not the ADAPT-X engine) |
| `adaptx.carla` | Boundary: interface, real client, mock. Optional dependency |
| `adaptx.services` | `lidar_service`, `metrics_service`, `carla_service`, `system_service`, `tracking_service`, `prediction_service` |
| `adaptx.api` | `app`, `schemas`, `dependencies`, `routes/`, `websocket/` |
| `adaptx.benchmark` | `datasets`, `baseline`, `runner`, `detection`, `tracking`, `prediction`, `models` |

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
| `AdaptiveMap`, `AdaptiveMapCell`, `ResolutionContext` | **Contracts only** |
| `RiskField`, `RiskCell`, `ObjectRisk` | Contract + baseline only |
| `SystemStatus`, `ComponentStatus`, `SystemMetrics` | Readiness **and** implementation status (ADR-005) |

Shared rules: `schema_version`, tz-aware UTC timestamps, `coordinate_frame`, `source`
(`live_sensor`/`simulation`/`replay`/`synthetic_test`/`unavailable`), `extra="forbid"`.

## 7. API endpoints (13)

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
| POST | `/api/v1/tracking/reset` |
| GET | `/api/v1/tracking/status` |
| WS | `/ws/telemetry` |

## 8. Configuration sections

`ADAPTX_` prefix, `__` nesting. Sections: `APP`, `API`, `LOGGING`, `CARLA`, `LIDAR`
(2A bounds + opt-in 2B stages), `DETECTION`, `TRACKING`, `PREDICTION`, `MAP`, `RISK`,
`WEBSOCKET`.

`LIDAR`, `PREDICTION`, `MAP`, `RISK` and `WEBSOCKET` are documented in `.env.example`.
**`DETECTION` and `TRACKING` are not** — a gap left by Phases 3 and 4, still open.

Defaults worth knowing: detection `cluster_tolerance_m=0.5`, `min_cluster_points=10`;
tracking `max_association_distance_m=2.5`, `min_hits_to_confirm=3`, `max_missed_frames=3`,
`velocity_smoothing=0.5`, `max_timestep_s=2.0`, `min_speed_for_heading_mps=0.3`;
prediction `horizon_s=3.0`, `interval_s=0.25` (13 points, t+0 inclusive), `max_tracks=256`,
`max_speed_mps=80.0`, `base_uncertainty_m=0.5`, `uncertainty_growth_mps=0.5`,
`confidence_hits_full=3`.

## 9. Telemetry (`/ws/telemetry`)

`hello` then periodic `telemetry`. Payload provides `system`, `metrics`, `detection`,
`tracking`, `prediction` — the last three are **summaries** (configuration and counts),
never per-frame geometry, trajectory points or point arrays. `not_yet_available` currently
lists `risk_field` and `adaptive_map`. A stream leaves that list only once something
genuinely produces it.

## 10. Lifecycle

`ApplicationContext` (`core/lifecycle.py`) holds: `settings`, `metrics`, `lidar`, `carla`,
`system`, `risk_engine`, `preprocessor`, `detector`, `tracking`, `prediction`. Built by
`build_context()`, attached to `app.state`, reached through FastAPI dependencies.

**Tracking is the only stateful perception component** (ADR-025). One tracker per process,
lock-guarded, cleared on shutdown. Concurrent clients share one track set.

`PredictionService` is **stateless with respect to perception**: it holds counters for
status and telemetry only, and resetting it changes what the status endpoint reports, never
what the predictor produces.

## 11. Dependencies

Runtime: `fastapi`, `uvicorn[standard]`, `pydantic>=2`, `pydantic-settings`, `numpy`, `psutil`.
Dev: `pytest`, `pytest-asyncio`, `httpx`, `ruff`, `mypy`.
Optional extras declared but **not installed**: `open3d` (`[pointcloud]`), `carla` (`[carla]`).

**No SciPy, no ML framework, no database.** Declined four times on record (ADR-014, 020,
024, 026). Phase 5 added no dependency.

## 12–14. Verification status

- **733 tests pass** (`pytest`)
- `ruff check .` — All checks passed
- `ruff format --check .` — 148 files formatted
- `mypy src` — no issues in 82 source files
- Backend starts; all 13 endpoints respond; no tracebacks
- Live temporal check: a vehicle advancing 1 m per 0.5 s measured 2.000 m/s, and its
  trajectory advanced +1 m at t+0.5, +2 m at t+1, +4 m at t+2 and +6 m at t+3; uncertainty
  rose 0.5 → 2.0 m; the track's first frame produced **no trajectory** and an explicit
  `insufficient_velocity` skip

Benchmarks (`python -m adaptx.benchmark [--detect|--track|--predict]`) — all synthetic,
**speed only**: pipeline ~213 ms/100k points; detection ~3.4 ms at that size; tracking
~10.6 ms at 100 objects; prediction ~27 ms at 100 tracks (13 points each). Measured results
in [`experiments/experiment-log.md`](experiments/experiment-log.md).

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
- CARLA is a boundary only; sensor/actor operations raise explicitly.

## 16. Architecture decisions

ADR-001 … ADR-027 in [`decisions/architecture-decisions.md`](decisions/architecture-decisions.md).
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

## 17. What MUST NOT change

1. The coordinate convention (ADR-009).
2. Phase 1–5 algorithms, unless a measured defect justifies it.
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

## 18–19. Next step — 2.5D mapping (Phase 6)

The agreed next work item is the **2.5D map**: implement
`mapping.interfaces.AdaptiveMapper` in two variants — the fixed-resolution baseline
(ADR-003) and the adaptive mapper — distinguished by `AdaptiveMap.is_adaptive`. Objective
and full handoff: [`NEXT_PHASE.md`](NEXT_PHASE.md).

## 20. Not yet

Do **not** start the risk engine (Phase 7), adaptive resolution (Phase 8), CARLA scenarios
or the dashboard. In particular, **collision reasoning, time-to-collision and trajectory
overlap belong to Phase 7**, not to prediction — Phase 5 deliberately stops at trajectories.
The dashboard design target is captured in [`UI_UX.md`](UI_UX.md) with a panel-by-panel
audit of what can actually be fed today.
