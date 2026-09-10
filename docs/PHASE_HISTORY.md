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

## Phase 5 — Trajectory prediction · verified

**Purpose:** turn the Phase 4 measured velocity into a forward-looking trajectory the risk
engine can reason over, without inventing anything the sensors never showed.

**Renumbering.** Prediction was Phase 8 and 2.5D mapping was Phase 5. The discrepancy left
open at the end of Phase 4 was resolved in favour of prediction being **Phase 5** — it is
what the risk engine needs next — and mapping, risk and adaptive resolution each shifted one
later. `CLAUDE.md`, `ROADMAP.md` and `system_service.py` were made to agree. The phase
headings in this file were deliberately left as originally written: they are a record of
what happened, and renaming them would falsify it.

**Algorithm:** constant velocity, and nothing else (ADR-026):

```
position(t)    = position + velocity * (age_s + t)
uncertainty(t) = base_uncertainty_m + uncertainty_growth_mps * (age_s + t)
```

Points run from `t+0` to the horizon inclusive — 13 at the defaults (3.0 s, 0.25 s). Each
point's absolute timestamp is derived arithmetically from the source time, never from a wall
clock, so a trajectory is reproducible.

**The `age_s` insight.** A coasting track's stored position is stale by a *measured*
interval: the gap between its `last_seen` and the prediction time. Folding that into the
same formula makes one expression correct for both fresh and coasting tracks — `age_s` is
zero for a track matched this frame, collapsing it to `p + v*t` — and avoids a second code
path. Ignoring it would have silently pretended a missed frame never happened.

**Eligibility is reported, not silent (ADR-027).** Every track appears either in
`trajectories` or in `skipped` with a status and reason, enforced by
`considered == predicted + skipped`. `velocity is None` yields **no trajectory** — null is
not zero (ADR-023), and a flat "stays where it is" path would fabricate a measurement. A
*measured* standstill is different and legitimately yields a stationary trajectory. An
over-speed velocity is **rejected, never clipped**: a clipped value is a number no sensor
produced. A `STALE_OBSERVATION` bound stops a coasting track being extrapolated across a gap
longer than the horizon itself.

Tentative tracks are predicted rather than skipped when they have a velocity — a velocity
measured from two observations is real however new the track is — with the lower evidence
expressed as lower confidence rather than as exclusion.

**Uncertainty and confidence are evidence, not probability.** Uncertainty is a documented
heuristic growing linearly with extrapolation time; point confidence is the track's evidence
score decayed by exactly the ratio the uncertainty grew, so the two can never disagree.
Neither is calibrated, and the status endpoint says so with `uncertainty_is_heuristic`.
`base_uncertainty_m` is constrained strictly positive: a zero floor would claim a perfectly
known position.

**Contract reuse.** `PredictedTrajectory` and `TrajectoryPoint` had existed unused since
Phase 1 and were reused rather than duplicated; `status` and `observation_age_s` were added
additively. `TrajectoryPredictor.predict` was widened to return a `PredictionResult` — safe,
because it had no implementations.

**Statefulness.** `PredictionService` holds no perception state, unlike `TrackingService`.
A prediction is a pure function of one tracking result, so its counters exist only for
status and telemetry.

**Four Phase 1–4 tests were retargeted, none weakened.** All four asserted that prediction
was `PLANNED` / unimplemented — a premise Phase 5 genuinely changed. Each now guards the
same underlying property: that a baseline never reports `IMPLEMENTED`, that a telemetry
stream leaves `not_yet_available` only when something produces it, and that the channel
still carries no raw trajectory geometry.

**Measured (Experiment 004):** prediction is **linear** in trajectory points — 45,000 to
55,000 points/s across a 100× range of track counts — with no quadratic term, unlike Phase 4
association. Cost is dominated by **contract validation, not arithmetic**: cProfile
attributed 0.95 s of a 1.51 s five-pass run at 500 tracks to Pydantic construction across
67,510 calls. Hoisting per-point timestamp construction out of the per-track loop removed a
measured 13.4 ms per 500-track pass (~11%), though the end-to-end difference sits inside this
machine's run-to-run spread, so no end-to-end speedup was claimed.

**Limitations:** constant velocity is wrong through turns and braking, and a wrong trajectory
looks as confident as a right one apart from its uncertainty radius; accuracy is unmeasured
and unmeasurable without labelled trajectories; quality is bounded by tracking, which is
bounded by detection.

**Status:** 733 tests, ruff and mypy clean, live multi-frame verification passed — a vehicle
advancing 1 m per 0.5 s measured 2.000 m/s and predicted +1 m at t+0.5, +2 m at t+1, +4 m at
t+2 and +6 m at t+3, with uncertainty rising 0.5 → 2.0 m and the first frame producing an
explicit `insufficient_velocity` skip rather than a trajectory.

---

## Phase 6 — 2.5D spatial mapping · verified

**Purpose:** give ADAPT-X its first spatial representation - a structured grid the risk
engine and, later, the adaptive resolution controller can reason over - without prejudging
what "adaptive" will mean.

**Algorithm:** bin a processed Phase 2 frame into a bounded, uniform XY grid (ADR-028)::

    column = floor((x - min_x) / resolution)
    row    = floor((y - min_y) / resolution)

Cells are half-open, so a point on `min_x` is the first cell and a point on `max_x` is out
of bounds rather than clamped into a cell it does not belong to. Per cell: point count, and
min / max / mean height. Quantisation reuses `perception.grid.cell_indices`, so the int64
overflow guard found necessary in Phase 2B is not reimplemented and cannot drift.

**The decision that shapes everything else (ADR-029).** A mapper never chooses its own
resolution. It is handed a `ResolutionDecision` and applies it. The naming needed care: a
`ResolutionContext` already existed carrying risk, uncertainty, object density and speed -
the *inputs* to a decision. Reusing that name for the decision itself would have destroyed
the contract Phase 8 needs and put risk fields inside the mapper's argument. Two contracts
now sit either side of the boundary::

    ResolutionContext -> [ResolutionController] -> ResolutionDecision -> AdaptiveMapper
       (inputs)              (not implemented)         (output)            (Phase 6)

The mapper is never *given* tracks, trajectories, risk or uncertainty, so a risk-aware
choice is structurally impossible rather than merely discouraged.

**Frame-local, deliberately (ADR-030).** Every call builds a whole map from one frame.
Accumulation was declined because it needs ego-motion compensation, a decay policy and a way
to tell a moving object's trail from a wall - none of which exist, and without them an
accumulating map would smear every moving vehicle into a barrier while looking plausible.
`reset()` is a documented no-op, asserted by a test: being unable to contaminate the next
frame is the guarantee.

**Binary occupancy and null height (ADR-031).** A cell is occupied iff `point_count > 0` -
derived, not stored, so the two can never disagree. A probability would need a sensor model
that would be invented rather than measured. An unobserved cell reports **NaN / null**
height, never zero: `z = 0` is a real height about 1.8 m above the road here, so zero would
be indistinguishable from a measured flat surface at sensor height. Same reasoning as
ADR-023 applied to a different quantity.

**Dense arrays, not cell objects.** The pre-existing `AdaptiveMap` holds
`list[AdaptiveMapCell]`; a 0.25 m map is 230,400 cells, and one validated Pydantic object
each would cost more than the mapping. `SpatialMap` holds NumPy arrays instead - the
precedent `BasePointCloudFrame.points` already set - and `to_adaptive_map()` projects
*occupied cells only* into the old contract, so nothing was lost and nothing duplicated.

**Interface change:** `AdaptiveMapper.update(frame, *, ego_state, tracks, risk_field)` became
`build(frame, resolution)`. The old signature handed the mapper exactly the material ADR-029
forbids it to use. Zero implementations and zero importers existed outside the module - the
same situation, and the same precedent, as widening `TrajectoryPredictor.predict` in Phase 5.

**Three Phase 1–5 tests were retargeted, none weakened.** All asserted mapping was `PLANNED`,
which Phase 6 makes false. Each now guards the property that mattered underneath: that a
subsystem shipped as a baseline never reports `IMPLEMENTED`, and that a fixed-resolution map
never makes ADAPT-X look like it allocates resolution by risk.

**Defect found by profiling.** The first implementation allocated four full-grid arrays up
front and seven more inside the accumulation branch - eleven where six suffice, which at
640,000 cells is real waste. The rewrite builds flat arrays once and reshapes, using
`np.fmin`/`np.fmax`, which ignore NaN so untouched cells keep their unobserved state without
a second pass. All 114 Phase 6 tests were unchanged by it, so it is semantics-preserving.

**Measured (Experiment 005):** mapping cost has **two independent drivers**, and separating
them is the point. At 9,800 points, going 1.00 m → 0.25 m multiplies cells by 16 and time by
7.0x while the point count never changes; at 1,000,000 points the same change costs only
1.3x. Occupancy falls from 11.30% to 1.17% on the small dataset as resolution rises: a fine
uniform map spends a growing majority of its cells recording that nothing was observed.
**That gap is the first quantitative statement of the problem ADAPT-X exists to solve** - and
it is not evidence that an adaptive mapper would do better, because none exists to measure.
The remaining `.at` calls are 14% of the pass and were left alone rather than optimised
speculatively.

**Limitations:** one uniform cell size everywhere; unobserved and free space are
indistinguishable, so an occluded cell reads like empty space; no temporal fusion, no
localisation, no SLAM; correctness unmeasured and unmeasurable without a labelled reference
map.

**Status:** 848 tests, ruff and mypy clean, live verification passed - 14 endpoints
responding, a real scene mapped to a 160x160 grid, accounting balancing exactly, and
consecutive requests confirmed independent.

---

## Cross-phase pattern

Each phase ships a **deterministic, explainable baseline** behind an interface, labelled
`is_baseline`, with its failure modes documented **and asserted by tests** so they stay
visible. No phase has added a dependency beyond the Phase 1 set - six phases, zero new dependencies.
