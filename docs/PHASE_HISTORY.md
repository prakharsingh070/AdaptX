# ADAPT-X — Phase History

What was built in each phase and why, compressed to what future work needs. Full detail
lives in [`ARCHITECTURE.md`](ARCHITECTURE.md),
[`decisions/architecture-decisions.md`](decisions/architecture-decisions.md) and
[`experiments/experiment-log.md`](experiments/experiment-log.md).

---

## Phase 1 — Engineering foundation · verified

**Purpose:** make the project buildable, testable and honest before any perception exists.

**Built:** `src/adaptx/` package; typed environment-driven settings; structured text/JSON
logging; exception hierarchy mapped to HTTP statuses; all data contracts; module interfaces
for every future subsystem; FastAPI app with 7 endpoints; `/ws/telemetry`; CARLA boundary
(optional dependency, real client + mock); Docker; 150 tests.

**Decisions that still bind:** ADR-004 (`/api/v1` prefix), ADR-005 (readiness *and*
implementation status, so a planned module can never look implemented), ADR-006 (risk
normalised to [0,1]), ADR-007 (CARLA optional; mock never a silent fallback), ADR-008 (no
database, broker or ML framework).

**Status:** Committed and merged to `main` (PR #1). No perception algorithm — by design.

---

## Phase 2A — LiDAR input and preprocessing · verified

**Purpose:** first real processing stage — turn a raw scan into a validated, cleaned,
spatially restricted cloud.

**Algorithms:** structural validation; non-finite (NaN/Inf) removal; ROI box filter; range
filter. All three filters are NumPy boolean masks evaluated against the original array and
combined in pipeline order, so counts partition the input exactly while only one output
array is allocated.

**Decisions:** ADR-009 (coordinate convention **+x forward, +y left, +z up** — written down,
not invented: Phase 1's yaw definitions already implied it); ADR-010 (`RawPointCloudFrame`
may hold NaN/Inf, `PointCloudFrame` may not); ADR-011 (range is 3D Euclidean, not planar,
because the blind zone is a 3D phenomenon).

**Two defects found and fixed:** `min_points` was being re-applied to the pipeline's
*output*, rejecting a frame legitimately filtered to zero; and `bounds()` let an infinity
serialise as `null`, reading as missing data.

**Status:** Committed (3 commits). 221 tests at the time.

---

## Phase 2B — voxelisation, ground, noise · verified

**Voxelisation:** one point per occupied voxel, keeping the **real measured point nearest
the voxel centroid** — never the centroid itself, which is a coordinate no sensor returned
(ADR-015). Origin-anchored grid so voxel boundaries are stable across frames.

**Ground segmentation:** per-xy-cell lowest point plus a height tolerance (ADR-016). Chosen
over a global height threshold because that would need a known sensor mount height; working
per cell discovers ground locally and handles slope.

**Noise filtering:** neighbour count over the 3×3×3 cell block — an honest approximation of
radius outlier removal, named for what it does. Avoids a SciPy dependency (ADR-014).

**All three are opt-in** (ADR-012): they are lossy or baseline, so enabling them silently
would change what every downstream consumer sees.

**Key finding:** no LiDAR→ego coordinate transform is required (ADR-013) — checked per
stage rather than assumed. Voxelisation is frame-agnostic, noise filtering is
rigid-transform invariant, and ground segmentation needs only an "up" axis plus a locally
discovered level.

**Defect found:** `astype(int64)` silently overflows on extreme coordinates, corrupting cell
membership. All stages now go through a guarded quantiser (`perception/grid.py`).

---

## Phase 2C — pipeline integration and benchmarking · verified

**Built:** `LiDARProcessingPipeline` orchestrator (renamed from `PointCloudPreprocessor`,
ADR-017) sequencing seven stages and doing accounting/timing only; per-stage measured
durations with unattributed time reported as `overhead_ms` rather than inflating a stage;
`PipelineConfiguration` snapshot on every result; `adaptx.benchmark` with deterministic
synthetic datasets (ADR-019) and a fixed-resolution **processing** baseline explicitly
distinct from ADR-003's future map baseline (ADR-018).

**Measured (Experiment 001):** ~21 ms / 213 ms / 900 ms for 10k / 100k / 400k points.
Phase 2B stages are ~85% of frame cost; voxelisation is about half. Replacing a 3-key
`lexsort` with a stable integer sort gave a controlled **1.30×** on that step.

**Status:** 459 tests.

---

## Phase 3 — Geometric object detection · verified

**Detection:** consumes the pipeline's **non-ground** output; repeats none of its work.

**Clustering:** grid connected components, **not DBSCAN** (ADR-020). Touching occupied cells
join; connected groups become clusters. Vectorised label propagation with pointer jumping.

**Classification:** dimension bands for pedestrian / cyclist / vehicle / obstacle. A cluster
matching **none or more than one** band is `UNKNOWN` — ambiguity is reported, not resolved
by picking a favourite (ADR-021). Reuses the existing `ObjectClass.CYCLIST` rather than
adding a `BICYCLE` duplicate.

**Confidence is a geometric fit score, not a probability.** No labelled data exists to
calibrate one. `UNKNOWN` scores 0.0.

**Contract:** `ObjectDetector.detect` returns a full `DetectionResult` including *rejected*
clusters with reasons, so "saw nothing" is distinguishable from "saw candidates and turned
them all down" (ADR-022).

**Limitations:** merges objects in touching cells; AABB only, so a diagonal vehicle measures
larger than it is (the main source of `UNKNOWN`); accuracy unmeasurable.

**Measured (Experiment 002):** detection ~3.4 ms on a 100k-point frame — ~2% of frame cost.
Scales with **cluster count**, not point count.

**Status:** 537 tests.

---

## Phase 4 — Temporal tracking · verified

**Association:** gated greedy nearest neighbour (ADR-024) with optional class and size
compatibility; `UNKNOWN` matches anything. Ties break on `(distance, track_id, detection
index)`, so detection ordering cannot change the outcome. Hungarian rejected — needs SciPy,
and greedy is explainable when a crossing goes wrong.

**Lifecycle:** reused the existing four-state enum — `TENTATIVE → CONFIRMED → COASTING →
LOST`. No new states were needed. Tentative tracks die faster than confirmed ones so a
spurious detection cannot linger as a ghost.

**Velocity:** `(position − previous_position) / dt` from **frame timestamps**, so variable
intervals are handled. `None` until two observations, and for non-positive or over-long
intervals. Both raw (`observed_velocity`) and EMA-smoothed values are reported, so smoothing
cannot hide a jump. Heading only above a speed floor; acceleration only with two consecutive
velocities.

**Contract change (ADR-023) — the only Phase 1–3 contract Phase 4 altered:**
`TrackedObject.velocity`/`acceleration` became `Vector3 | None` and `heading_rad` became
`float | None`. The previous zero-vector defaults were indistinguishable from a *measured*
standstill, which the "no invented velocity" requirement forbids. `speed_mps` now returns
`float | None`; `is_moving` added.

**State:** `TrackingService` on the `ApplicationContext` (ADR-025), lock-guarded, reset on
shutdown and through the API. One tracker per process — concurrent clients share one track
set, documented rather than disguised.

**API:** `POST /api/v1/lidar/track` (stateful, needs temporal ordering and explicit
timestamps), `POST /api/v1/tracking/reset`, `GET /api/v1/tracking/status`.

**Telemetry:** tracking **summary** — counts, ids, configuration. No per-track history, no
trajectories.

**Defect introduced and fixed:** a route-registration helper mutated the module-level LiDAR
router on every `create_app()`, so routes accumulated (caught via a duplicate-operation-ID
warning). Moved into `routes/lidar.py`.

**Limitations:** identity swaps possible during close crossings; **no re-identification**;
`O(T×D)` association (~10 ms at 100 objects, ~219 ms at 500); correctness unmeasurable.

**Measured (Experiment 003):** hoisting per-detection work out of the inner loop gave a
measured **2.6×** at 500 objects with all tracker tests unchanged.

**Status:** 623 tests, ruff and mypy clean, live multi-frame verification passed (id stable
across frames, velocity `null → 2.000 → 2.000 m/s` matching supplied timestamps).

---

## Cross-phase pattern

Each phase ships a **deterministic, explainable baseline** behind an interface, labelled
`is_baseline`, with its failure modes documented **and asserted by tests** so they stay
visible. No phase has added a dependency beyond the Phase 1 set.
