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

**Phase 4 COMPLETE.** Phases 1, 2 (A/B/C), 3 and 4 are implemented and verified.

## 3. Branch and status

- Branch: `phase-2a-lidar-preprocessing` (name is stale — it now carries Phases 2A–4)
- Last commits: 3 Phase 2A commits on top of `14f5e3f` (merged Phase 1 PR)
- **Phases 2B, 2C, 3 and 4 are uncommitted** in the working tree (~32 modified, ~30 new files)
- Nothing has been pushed since the Phase 1 merge

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
| `adaptx.mapping` / `adaptx.prediction` | **Interfaces only — nothing implemented** |
| `adaptx.risk` | Contract + `baseline.py` (proximity-only, not the ADAPT-X engine) |
| `adaptx.carla` | Boundary: interface, real client, mock. Optional dependency |
| `adaptx.services` | `lidar_service`, `metrics_service`, `carla_service`, `system_service`, `tracking_service` |
| `adaptx.api` | `app`, `schemas`, `dependencies`, `routes/`, `websocket/` |
| `adaptx.benchmark` | `datasets`, `baseline`, `runner`, `detection`, `tracking`, `models` |

## 6. Key contracts (`adaptx.models`)

| Model | Notes |
|---|---|
| `BasePointCloudFrame` → `RawPointCloudFrame` / `PointCloudFrame` | Raw **may** hold NaN/Inf; validated may not (ADR-010) |
| `PointCloudProcessingResult`, `ProcessingMetrics`, `StageMetrics`, `PipelineConfiguration` | Per-stage counts + durations; `overhead_ms` reported separately |
| `DetectedObject` | class, position, AABB, confidence (**geometric fit score, not a probability**), `distance_m`, `velocity` always `None` |
| `DetectionResult`, `RejectedCluster`, `DetectionConfiguration` | `cluster_count == len(objects) + len(rejected)`, model-enforced |
| `TrackedObject` | `velocity` / `acceleration` / `heading_rad` are **`None` until measured** (ADR-023); `speed_mps` → `float \| None` |
| `TrackingResult`, `TrackingConfiguration` | matched/created/retired ids, counts, timings |
| `PredictedTrajectory`, `TrajectoryPoint` | **Contracts exist; nothing produces them** |
| `AdaptiveMap`, `AdaptiveMapCell`, `ResolutionContext` | **Contracts only** |
| `RiskField`, `RiskCell`, `ObjectRisk` | Contract + baseline only |
| `SystemStatus`, `ComponentStatus`, `SystemMetrics` | Readiness **and** implementation status (ADR-005) |

Shared rules: `schema_version`, tz-aware UTC timestamps, `coordinate_frame`, `source`
(`live_sensor`/`simulation`/`replay`/`synthetic_test`/`unavailable`), `extra="forbid"`.

## 7. API endpoints (11)

| Method | Path |
|---|---|
| GET | `/health` |
| GET | `/api/v1/system/status`, `/api/v1/system/metrics` |
| GET | `/api/v1/carla/status`, `/api/v1/map/status`, `/api/v1/risk/status` |
| POST | `/api/v1/lidar/frame` — validate/ingest, optional `preprocess` flag |
| POST | `/api/v1/lidar/detect` — process + detect |
| POST | `/api/v1/lidar/track` — process + detect + track (**stateful**) |
| POST | `/api/v1/tracking/reset` |
| GET | `/api/v1/tracking/status` |
| WS | `/ws/telemetry` |

## 8. Configuration sections

`ADAPTX_` prefix, `__` nesting. Sections: `APP`, `API`, `LOGGING`, `CARLA`, `LIDAR`
(2A bounds + opt-in 2B stages), `DETECTION`, `TRACKING`, `MAP`, `RISK`, `WEBSOCKET`.
Documented in `.env.example`.

Defaults worth knowing: detection `cluster_tolerance_m=0.5`, `min_cluster_points=10`;
tracking `max_association_distance_m=2.5`, `min_hits_to_confirm=3`, `max_missed_frames=3`,
`velocity_smoothing=0.5`, `max_timestep_s=2.0`, `min_speed_for_heading_mps=0.3`.

## 9. Telemetry (`/ws/telemetry`)

`hello` then periodic `telemetry`. Payload provides `system`, `metrics`, `detection`,
`tracking` — the last two are **summaries** (configuration and counts), never per-frame
geometry or point arrays. `not_yet_available` currently lists
`predicted_trajectories`, `risk_field`, `adaptive_map`.

## 10. Lifecycle

`ApplicationContext` (`core/lifecycle.py`) holds: `settings`, `metrics`, `lidar`, `carla`,
`system`, `risk_engine`, `preprocessor`, `detector`, `tracking`. Built by `build_context()`,
attached to `app.state`, reached through FastAPI dependencies.

**Tracking is the only stateful component** (ADR-025). One tracker per process, lock-guarded,
cleared on shutdown. Concurrent clients share one track set.

## 11. Dependencies

Runtime: `fastapi`, `uvicorn[standard]`, `pydantic>=2`, `pydantic-settings`, `numpy`, `psutil`.
Dev: `pytest`, `pytest-asyncio`, `httpx`, `ruff`, `mypy`.
Optional extras declared but **not installed**: `open3d` (`[pointcloud]`), `carla` (`[carla]`).

**No SciPy, no ML framework, no database.** Declined three times on record (ADR-014, 020, 024).

## 12–14. Verification status

- **623 tests pass** (`pytest`)
- `ruff check .` — All checks passed
- `ruff format --check .` — 138 files formatted
- `mypy src` — no issues in 77 source files
- Backend starts; all 11 endpoints respond; no tracebacks

Benchmarks (`python -m adaptx.benchmark [--detect|--track]`) — all synthetic, **speed only**:
pipeline ~213 ms/100k points; detection ~3.4 ms at that size; tracking ~10.6 ms at 100 objects.
Measured results in [`experiments/experiment-log.md`](experiments/experiment-log.md).

## 15. Known limitations — do not hide these

- **No labelled data exists.** Detection accuracy and tracking correctness are unmeasured
  and currently **unmeasurable**. Every benchmark measures speed only.
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
- CARLA is a boundary only; sensor/actor operations raise explicitly.

## 16. Architecture decisions

ADR-001 … ADR-025 in [`decisions/architecture-decisions.md`](decisions/architecture-decisions.md).
Most load-bearing for future work:

- **ADR-009** — coordinate convention: **+x forward, +y left, +z up**, metres, right-handed
- **ADR-005** — readiness *and* implementation status reported separately
- **ADR-023** — motion is `None` until measured; zero would claim a standstill
- **ADR-022 / ADR-024** — module contracts return full result objects, not bare lists
- **ADR-025** — stateful services live on the `ApplicationContext`, never module globals
- **ADR-019** — benchmark data is synthetic and labelled as such

## 17. What MUST NOT change

1. The coordinate convention (ADR-009).
2. Phase 1–4 algorithms, unless a measured defect justifies it.
3. Existing endpoint behaviour and response shapes — extend additively.
4. The honesty rules: no fabricated metrics; unmeasured values are `null` with a reason;
   `source` provenance is mandatory; baselines are labelled `is_baseline`.
5. Existing tests — never weaken or delete them. If a premise genuinely changes, retarget
   the test to guard the same property and say so.
6. `IMPLEMENTED` status is reserved for mature functionality; baselines are `PARTIAL`.
7. No new dependencies without an ADR.

## 18–19. Next step — trajectory prediction

**⚠ Phase-numbering discrepancy, unresolved.** The repository (`docs/ROADMAP.md`,
`CLAUDE.md`, and the `prediction` status component with `phase=8`) numbers trajectory
prediction as **Phase 8**, with **Phase 5 = 2.5D mapping**. The most recent instruction
called trajectory prediction **"Phase 5"**. This file does not renumber anything — a fresh
session should **ask the user which numbering to use** before writing status text.

Whatever it is called, **the agreed next work item is trajectory prediction**. Objective and
full handoff: [`NEXT_PHASE.md`](NEXT_PHASE.md). In short — implement
`prediction.interfaces.TrajectoryPredictor` as a constant-velocity baseline over
`TrackedObject`, producing the existing `PredictedTrajectory` contract.

## 20. Not yet

Do **not** start the risk engine, adaptive resolution, 2.5D mapping, CARLA scenarios or the
dashboard. The dashboard design target is captured in [`UI_UX.md`](UI_UX.md) with a
panel-by-panel audit of what can actually be fed today.
