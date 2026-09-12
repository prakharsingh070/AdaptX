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

## Phase 7 — Risk and uncertainty · verified

**Purpose:** turn tracks, trajectories and the map into a single per-object answer to *how
concerning is this, and how sure are we* - the signal Phase 8 needs to allocate perception
effort.

**Three conflicts with the brief, resolved against the repository.** The instruction asked
for risk levels `LOW/MEDIUM/HIGH/UNKNOWN`, but `RiskLevel` already had `CRITICAL`, exercised
by tests and present in the API thresholds. `UNKNOWN` was **added**; `CRITICAL` kept. The
instruction sketched `RiskEngine.assess_many` on the interface, but `RiskEngine` already had
one implementation, so widening it would have forced changes to Phase 1 code; instead the new
engine implements the **existing** abstract methods - keeping it comparable to the baseline
through `RiskField` - and adds `assess_many` as its own richer API, matching how the mapping
and prediction services already type against concrete engines. The instruction's example
formula summed uncertainty into the score, but `models/risk.py` states the opposite; the
repository won (ADR-033).

**Algorithm (ADR-032):** three factors, each normalised to `[0, 1]` - proximity, radial rate
of approach, and how close the Phase 5 predicted path passes - combined as a weighted mean
**over the factors actually available**::

    risk_score = sum(w_i * f_i) / sum(w_i)   over available i only

**The renormalisation rule is the whole decision.** Treating a missing factor as zero is
quietly catastrophic, because *low* is exactly what an unmeasured value would look like: a
track whose velocity was never measured would score as though standing still, and the object
we know least about would look least concerning. That inversion is the reason ADR-023 exists,
applied one phase later. When no factor can be computed at all, the assessment is `UNKNOWN`
with `risk_score = None` - never a fabricated number.

**Uncertainty is reported beside risk, never folded into it (ADR-033).** Two tracks identical
except for observability score the *same* risk and different uncertainty - asserted by test.
That separation is not fastidiousness: it is what Phase 8 needs. A poorly observed region may
deserve finer perception precisely *because* it is poorly observed, and summing the two would
collapse the signal a resolution controller most needs, leaving a low score ambiguous between
"we looked and it is quiet" and "we could barely see it". Nine contributing reasons stay
visible beside the scalar so a consumer can act on the cause.

**Map context never lowers risk (ADR-034).** This is ADR-031 carried into the phase most
tempted to violate it. The seductive move - lower risk where a predicted path crosses cells
with no points - would treat *unobserved* as *clear*, reducing risk exactly where the sensor
saw least: behind the vehicle occluding the pedestrian. So map context is a context signal
and an uncertainty source, contributing nothing to the score.

**Aggregation is a maximum, never a mean (ADR-035).** Ten quiet objects and one critical one
average to something reassuring, and the object that matters disappears into the arithmetic.
An unassessed scene reports `UNKNOWN`, not `LOW`.

**Deliberately excluded:** time-to-collision - over a constant-velocity extrapolation with
heuristic uncertainty it would be a precise-looking number resting on two approximations;
trajectory-map intersection and occlusion, which need a visibility model that does not exist;
object class as a score multiplier, which would encode an unmeasured judgement that a
pedestrian is inherently N times more concerning than a vehicle. Class is reported and
appears in the explanation instead.

**Explanations are generated, never written.** Every clause comes from a computed number or
flag, and a parametrised test asserts the text never contains "probability", "guaranteed",
"safe", "validated" or "calibrated".

**Phase 8 boundary (ADR-036).** The engine imports no resolution type, produces no
`ResolutionDecision`, and `RiskAssessment` carries no cell size or resolution level. Asserted
at the model, the wire format and the module surface, and reported as
`decides_resolution: false`.

**Two Phase 1–6 tests were retargeted, none weakened.** Both asserted the *only* risk engine
was the proximity baseline. Each now guards what mattered underneath: that the status endpoint
names its engine and its limits, that the proximity baseline is still retained for comparison,
and that risk never reports `IMPLEMENTED`.

**Measured (Experiment 006):** linear in object count at 74-82 µs per object across a 200x
range, with no quadratic term - each object is assessed independently. Cost is dominated by
**contract construction, not arithmetic**: roughly five Pydantic models per assessment against
a handful of multiplications and a `min` over 13 trajectory points. The same finding as
Experiment 004. At the ~96 objects the pipeline actually produces, assessment costs about
7 ms.

**Limitations:** the score orders objects by concern but measures nothing physical; it is not
a probability, not calibrated, never validated - no labelled risk data exists; weights and
thresholds have never been tuned against outcomes because no outcomes have been recorded;
risk is object-level only, so `RiskCell` and `AdaptiveMapCell.risk_score` stay unpopulated
and `risk_field` stays unavailable; quality is bounded by tracking and prediction, themselves
baselines.

**Status:** 1021 tests, ruff and mypy clean, live verification passed - 15 endpoints
responding, a real scene assessed end to end, unknown velocity reported as unknown rather
than zero, and the Phase 8 boundary confirmed absent from every response.

---

## Phase 8 — Adaptive spatial resolution · verified

**The phase the project is named for.** Phase 6 built a mapper that applies a resolution;
Phase 7 built an engine that scores concern. Phase 8 is the policy between them, and it is
the first time ADAPT-X does the thing it claims: spend spatial detail unevenly, on purpose,
for a stated reason.

**One map, several resolutions (ADR-037).** A dense NumPy grid has exactly one cell size, so
an adaptive map cannot be one. The extent is partitioned into fixed-size square **regions**,
each holding its own sub-grid at its own cell size. Regions are half-open on their upper
edges and clipped at the map bounds, so they partition the extent exactly — a point lands in
one region and one cell of it, and `input == mapped + out_of_bounds` survives untouched. A
quadtree would allocate fewer cells and was deferred, not rejected; the reason is in the ADR,
and Experiment 007 is the evidence for when to revisit it.

**A detail priority, not a second risk score (ADR-038).** Six normalised factors — risk,
uncertainty, predicted-motion relevance, proximity, object density, measured motion —
combined as a weighted mean **over the factors actually available**. A factor that cannot be
computed is dropped and the weights renormalise. That is ADR-032 one phase later, and the
failure it prevents is worse here: coercing an unknown risk to zero would hand the *coarsest*
representation to exactly the objects the system understands least. Instead `risk_score is
None` drops the risk factor **and** floors the region level.

**Uncertainty finally pays for itself.** Phase 7 kept uncertainty separate from risk on the
argument that a later phase would need both. This is that phase: a quiet, badly observed
region earns detail on uncertainty alone. The `high_uncertainty` benchmark scene refines 27
regions with two of its three objects carrying no risk score at all.

**Refinement is immediate; coarsening is earned twice (ADR-039).** A hysteresis margin must
be cleared before a coarser level is even proposed, and it must then be proposed on three
consecutive frames before it applies. Both are needed — the margin alone still flips on wide
oscillation, the dwell alone still flips on a boundary. The asymmetry is deliberate: the cost
of refining a region that did not need it is some wasted cells; the cost of coarsening one
that did is missing structure exactly where the system was most concerned.

**The controller is stateful, and it is the only mapping state that survives a frame.** It
remembers each region's level and dwell counter, because stabilisation is temporal by
definition. Occupancy still accumulates nowhere: what persists is a policy decision, not a
measurement (ADR-030 intact).

**Budgets coarsen the least important regions, and say so (ADR-040).** Region, cell and
fine-region ceilings are enforced by demoting lowest-priority regions first, deterministically,
with every demotion recorded on the decision and counted in the plan. A budget that silently
coarsened the map would make a benchmark measure the budget instead of the policy.

**Neither Phase 6 nor Phase 7 changed.** `NEXT_PHASE.md` asked for that to be reported either
way, and the boundaries held. The controller reads positions from **tracks**, not from
assessments, because a `RiskAssessment` deliberately carries a distance rather than a
location (ADR-036, ADR-041) — adding a position to it would have eroded the separation for
one consumer's convenience.

**Six Phase 6/7-era tests were retargeted, none weakened.** Their premise was that adaptive
resolution did not exist. Each now guards what mattered underneath: that the *fixed* mapper
keeps its own identity and never claims to be adaptive, that a stream leaves
`not_yet_available` only when something genuinely produces it and then appears as a summary
rather than raw geometry, and that neither component ever reports IMPLEMENTED. One
contract was widened: `ResolutionContext.risk_score` became nullable, because a Phase 1
default of `0.0` could not express the very case Phase 7 made central.

**Measured (Experiment 007) — and it is not a clean win.** Against a uniform 0.25 m map the
adaptive map uses 6–30% of the cells; against 0.5 m, 25–50%. Against a **1.0 m** map it uses
*more* cells in every scene containing an object, which is arithmetic rather than a defect:
the base level is 1.0 m, so the policy can only add to it. Adaptive resolution is a way of
affording a fine map, not of beating a coarse one.

More awkwardly: **adaptive mapping is slower in wall-clock time than the fixed mapper in
every scene**, 25–32 ms against 5–8 ms, even where it allocates a quarter of the cells. The
cause was measured rather than guessed — holding cells constant at 14,400 and varying only
the region size moved mapping from 8.9 ms at 1 region to 77.2 ms at 576. **Cost scales with
region count, not cell count**, at roughly 120 µs per region. The lever is `tile_size_m`, and
that is now a number instead of an intuition. One small optimisation was tried, measured, and
kept only for the validation it added, because it changed nothing outside noise.

**Limitations:** the priority orders regions but measures nothing physical — not a
probability, not a safety margin, never calibrated or validated, because no labelled data
exists; whether the allocation is *appropriate* is unmeasured and unmeasurable; what the
coarse regions lose is not quantified, which would need a reference map; allocation is
quantised to the region, so a small object refines the ground around it; a quiet region keeps
its detail for up to the dwell time; `in_ego_path` is never set true and
`predicted_risk_score` is always null, because no planner and no predicted-risk formulation
exist.

**Status:** 1181 tests, ruff and mypy clean, 17 endpoints responding, a real scene mapped end
to end at four resolutions in one grid, and threshold jitter proven not to flip a region.

---

## Phase 9 — CARLA simulation boundary · verified (no live run)

The first phase whose deliverable sits *upstream* of the pipeline rather than inside it.
CARLA became a data source; nothing below the boundary changed.

**Phases 1-8 were not modified.** That is the result worth leading with. A simulated frame
enters the same `RawPointCloudFrame` contract, the same Phase 2A validation and the same
chain through to the adaptive map, and no stage branches on where the points came from. The
alternative - a CARLA-specific perception path - would have doubled every downstream
behaviour and made the Phase 11 comparison measure the plumbing instead of the perception
(ADR-042).

**Two objects, because there are two questions.** `CarlaClient` answers *am I connected?*
and backs the status endpoint. `CarlaSimulationSession` answers *is a simulation running?*
and owns actor and sensor lifetimes. A connected server with no session is a real and
ordinary state, and one object owning both would have had to lie about it. The status
endpoint now reports three independent facts - package importable, server connected,
simulation stepping - rather than collapsing them into one word.

**One coordinate conversion, in a module that imports no simulator (ADR-043).** ADR-009
predicted this flip back in Phase 2A and named Phase 9 as its trigger. What decided the
*placement* was the failure mode: a dropped sign flip mirrors the world **silently**. Every
object appears on the wrong side, nothing raises, no contract is violated, and the pipeline
produces confident output about a scene that never existed. Putting the arithmetic in a
CARLA-free module made it exhaustively testable on a machine with no simulator - which is
every machine this project has run on. Yaw converts by the same handedness change expressed
as an angle, so positions and orientations cannot disagree.

That test suite earned its keep immediately: it caught **my own** inverted expectation about
which way "left" points for a yaw-90 ego. The code was right; the test's expected value was
not, and re-deriving it from ADR-009 rather than from the code's output is the only reason
that was visible.

**Simulation time is authoritative (ADR-044).** Synchronous mode, fixed timestep, explicit
ticks - never a wall clock, never a sleep. Three phases already depended on this without
knowing it: Phase 4 measures velocity from frame intervals, Phase 5 extrapolates over them,
Phase 8 counts frames for its dwell time. Wall-clock timestamps would have made velocity a
function of machine load. Motion is scripted rather than physical for the same reason.

**Ground truth is a separate path (ADR-045)**, sharing the LiDAR frame's id and timestamp so
the two join later, and reaching no perception stage. The risk was never that someone would
wire it in deliberately - it is that doing so is *convenient*. Correcting a track id from
ground truth looks like an improvement and silently invalidates every accuracy figure taken
afterwards. Two tests guard it: the chain runs with and without reading ground truth and
must produce identical output, and a subprocess check confirms no perception module imports
the CARLA package at all.

**Cleanup was treated as a correctness property, not housekeeping.** The session destroys
every actor it spawned including after a *failed* setup, tolerates one actor refusing to
die without stranding the rest, and restores world settings on close - because a server left
in synchronous mode blocks on a client that has gone away and looks to the next user like a
hung simulator.

**No live CARLA run was executed.** The package is not installed here, so the live smoke
test reports 6 skipped with the reason, and Experiment 008 measures only ADAPT-X's own
conversion code (~0.2 ms for a 28,000-point frame; the cost is the float32-to-float64 copy,
not the sign flip). The lifecycle tests run against a hand-written stand-in that models
CARLA's left-handed frame but does no physics and no real ray casting. **Nothing in this
phase is a CARLA performance or accuracy claim**, and the adapter's API compatibility with a
real server remains unconfirmed.

**One test-configuration bug was fixed rather than worked around.** An integration test moved
the target 4 m per 0.05 s tick - 80 m/s - and tracking correctly refused to associate across
a 2.5 m gate, creating a new track every frame. The tracker was right; the scenario was
absurd. Slowing it to the scenario's real 8 m/s fixed it.

**Limitations:** no live validation; CARLA pitch and roll are not converted, only yaw; the
smoke scenario is one hard-coded scene with scripted motion and must be replaced by Phase 10
rather than grown; ground truth now exists and nothing measures against it, which is Phase
11's job and emphatically not a Phase 9 result.

**Status:** 1326 tests (1189 before), 6 deselected live, ruff and mypy clean, 17 endpoints
unchanged, zero regressions.

---

## Phase 10 — Scenario framework · verified (no live run) · replay deferred

From one hard-coded scene to a way of describing scenes. The framework sits above the
Phase 9 boundary and did not change it.

**A scenario is data (ADR-046).** `ScenarioDefinition` holds actors, ego-relative placement,
timed constant-velocity motion segments, duration, timestep and an explicit seed. It is
validated at construction, survives a JSON round-trip, and its models import nothing from
the CARLA boundary - a source-level test guards that, because an import-based one is vacuous
when a package `__init__` pulls in the runner. The handoff's trap was named precisely: the
moment scenarios are functions, they are reproducible from a commit, not a description.

**The seed is explicit, consumed, and reported (ADR-046).** Every randomised value is drawn
from `random.Random(seed)` once, before any simulator is opened, into a `ResolvedScenario`
recorded on the result. A test asserts the draw never touches the global generator. The
only randomised element is placement jitter; the catalogue uses none, so every catalogue
scenario is exact - and still reports its seed. `CarlaSettings.seed`, declared in Phase 9
and consumed by nothing, is now set from the scenario.

**Motion is placed, not simulated (ADR-047).** An actor's position at any scenario time is
a closed-form sum over its segments, and the runner places it there every frame. Frame *n*
is therefore a function of the definition, the seed and *n*. The consequence that matters
most: the scenario can state where every actor *should* be without a simulator running,
so every frame records the **commanded** pose beside the **reported** one. That is the
third leg of a comparison Phase 11 will make and Phase 10 does not.

**The runner drives a protocol extracted from the boundary (ADR-048).** `ScenarioSimulator`
is exactly the surface `CarlaSimulationSession` already had; no Phase 9 code changed to
satisfy it. A runner is single-use, which is the isolation guarantee - a test runs two
scenarios against one world and asserts the second inherits nothing. Cleanup runs in a
`finally` on every path; a test breaks the second of two spawns and asserts the first was
destroyed.

**Two failure modes, kept apart.** The first version of the runner returned a `FAILED`
result for a malformed definition and crashed inside its own failure path trying to embed
the invalid definition in that result. The distinction was drawn from the bug: a malformed
definition is a misuse and **raises** before any simulator contact; a run-time failure
**returns** `FAILED` with the frames stepped so far, so a batch survives one bad run.

**Ground truth stays out, again.** The processor callback receives the sensor frame and
nothing else, by signature. An integration test runs the chain by hand without ever calling
`ground_truth()` and asserts identical stage counts to the runner's. The result contracts
carry no accuracy, precision, recall, error or match field, and a test asserts that too.

**`carla/smoke.py` was deleted, not grown.** Its three constants became the
`vehicle_approach` definition; its two orphaned settings were removed; its seven tests were
retargeted onto the framework with intent preserved.

**Three things the boundary tests caught.** A lazy `import carla` for a version string in
generic runner code (moved into the boundary as `carla_package_version()`); the cyclist
scenario failing against the stand-in because the fake did not know the bicycle blueprint -
which exercised the partial-spawn cleanup path exactly as designed, with zero leaked actors;
and my own smoke script producing an invalid definition through `model_copy`, which is what
surfaced the failure-path crash above.

**No live CARLA run was executed.** The package is still absent. Every catalogue scenario has
run only against the stand-in, the catalogue's blueprints have not been confirmed on any
real server, and a real server settles spawned vehicles onto the road in a way the fake does
not. Experiment 009 measures orchestration cost alone: ~35 µs to resolve, ~7 µs per actor per
frame - negligible, and the only figure this phase can honestly report.

**Event replay is deferred, not done.** The `ScenarioRunResult` is the recording a replay
would need, but no playback path exists and `DataSource.REPLAY` is still produced by nothing.
The open question - re-run the simulation or re-play a recording - is still open.

**Limitations:** no live validation; placed motion has no physics; the ego is stationary in
every scenario; four scenarios, no traffic, no weather; ground-truth contracts still live in
`adaptx.carla` though they are simulator-generic.

**Status:** 1442 tests (1327 before), 7 deselected live, ruff and mypy clean, 17 endpoints
unchanged, zero regressions, no new dependencies.

---

## Live CARLA validation of Phases 9 and 10 · 2026-09-11 · 7/7 live tests pass

The "no live run" caveats on the two sections above were retired by running everything
against a real CARLA 0.9.16 server on Town10HD_Opt (Experiment 010). The 3.13 environment
cannot hold the client - the 0.9.16 wheel is built for CPython 3.12 only and PyPI's `carla`
is 0.9.5 - so a second, Python 3.12 environment was created for it; the primary environment
and the default suite are unchanged.

**What the stand-in could not show, and the server did.** Three real behaviours, each fixed
in the boundary with a fake-backed regression so it cannot recur silently:

- A freshly spawned actor's `get_transform()` returns the world origin until the server has
  ticked. Every ego-relative placement was computed from that origin, so the first live
  scenario put its target 70 m from the ego, off-road, and the spawn was refused. The session
  now uses the spawn transform as the reference until the first tick (ADR-049), and the fake
  reports the origin until ticked so this cannot pass against the stand-in again.
- Spawn point 0 of Town10HD_Opt refuses the ego every time; 1-11 accept. The session walks
  the spawn points in order and records the index it used.
- The `carla` module has no `__version__`; the server's `get_server_version()` is recorded.

**One Phase 1 defect surfaced only because of the interpreter change.** `MetricsService`
measured `fps` with `time.monotonic()`, which on Windows Python 3.12 ticks every 15.6 ms;
frames within one tick shared a timestamp and `fps` was never computed. `perf_counter` now,
with a mocked-clock test. On 3.13 it had been fine by accident.

**Two tests assumed the absence of CARLA rather than testing for it.** A "no simulator"
CLI test ran against the default port and, with a server there, *succeeded* and failed; it
now targets a closed port. An import-isolation test ran in-process after other tests had
legitimately imported `carla`; it now checks in a subprocess.

**Result.** `pytest -m carla`: 7 passed. `python -m adaptx.scenarios run` for all four
catalogue scenarios: COMPLETED, contiguous simulator frame ids, dt exactly 0.05 s,
26,982-27,038 points per frame, ~92 ms of ADAPT-X pipeline per frame on this machine, and
zero vehicle, walker or sensor actors left on the server, checked with a fresh client.
Detections ran 8-16 per frame - most of them the map's static geometry. **No accuracy figure
was computed**; the ground truth to compute one is in every run record, and that is Phase 11.

**Status:** 1452 tests on 3.13 (1442 before), 1451 + 1 skipped on 3.12, 7 live tests
passing against the server, ruff and mypy clean, 17 endpoints unchanged, no new
dependencies. Committed as `ec93951` and merged with Phase 10 as PR #7 (`fe64d1c`).

---

## Phase 11 — Evaluation against simulator ground truth · verified · first measured correctness figures

**What was built.** `adaptx.evaluation`, an offline layer that reads a recorded
`ScenarioRunResult` and produces an `EvaluationReport`. To make that possible the Phase 10
record gained, additively, the pipeline's own result contracts per frame and the sensor
configuration (ADR-050); it still carries no metric and the Phase 10 test that says so still
passes. Matching is greedy nearest-neighbour within a gate, reported at 1, 2 and 4 m, with
deterministic tie-breaking and no precision figure, because the map's static geometry is
not a CARLA actor and a track on it is unlabelled, not false (ADR-052). Detection recall,
tracking match rate, planar and 3-D position error, velocity error against a
finite-difference reference (the simulator's velocity of a placed actor is meaningless),
continuity with identity switches and fragments, ADE/FDE over exactly aligned future frames
with no interpolation and the t+0 point excluded, risk against proximity events with
`UNKNOWN` preserved and never scored, map workload with occupancy accuracy explicitly not
evaluated, adaptive resolution paired against the fixed map within the same run - cells,
bytes, build time, detail at the actor's tile versus elsewhere, detail by risk level,
refinement lead, churn, reversals, holds - and resource. Every missing metric is null with a
reason (ADR-051). `python -m adaptx.evaluation evaluate | compare | run`. No endpoint, no
telemetry, no dependency. Architecture written before the code in `docs/EVALUATION.md`.

**The boundary held and is now proven in both directions.** Ground truth reaches
`adaptx.evaluation` and nothing else: a subprocess check with a clean module table and a
source inspection over `perception`, `tracking`, `prediction`, `mapping`, `risk` and
`services` assert it, and the evaluation package is shown never to require the `carla`
package. Evaluating a record does not modify it; a run evaluated and a fresh identical run
produce the same stage counts.

**Three things the live runs found before a figure was read.** Two stale actors - an ADAPT-X
ego and its LiDAR from the run killed during the first live attempt - were still parked at
spawn point 0 and about 4 m from every Experiment 010 ego; destroying them made point 0
accept, so Experiment 010's "the map refuses index 0" was wrong and is corrected in place
(ADR-049). From point 0 the approach scenario's target is off-road, so `ego_spawn_index`
now pins the ego and every experiment ran from point 1. And `carla.seed`, documented as the
seed for simulator randomness, seeded nothing: the LiDAR is now seeded from it, after
which most frames repeat to the point and the rest differ by at most 17 points in 27,000.

**The measurements (Experiment 011, simulation evidence only).** The baselines lost, as the
handoff said they would. Vehicle detection recall 0.00-0.06 at a 1 m gate and 0.23-0.61 at
2 m, with a consistent 1.5-1.7 m planar offset; the classifier never called the Audi a
vehicle. Tracking coverage 0.25-0.48 for moving actors with 1-3 identity switches each.
ADE 1.5-3.4 m mean, 5 m at a 3 s horizon for the cyclist and 12 m at 1.5 s for the
approaching car, where some frames measured a standstill for an object closing at 8 m/s.
The risk score orders proximity for moving actors (concordance 0.80-0.86) and not for a
stationary one; at the pre-registered 20 m band no actor was inside long enough to measure
alert recall, and a post-hoc 25 m band is reported as post-hoc. The adaptive map used
0.48-0.56 of the fixed map's cells, took 5x longer to build (8x with the controller), and
put finer cells under the perceived actor (0.41-0.58 m) than elsewhere (0.92-0.93 m),
refining ahead of arrival in all but two entries. **The pedestrian scenario is confounded**:
a placed walker keeps its physics velocity between placements and fell through the road on
45 of 120 frames, so one detection in 120 says nothing about the detector. Recorded, not
fixed - Phase 11 measures.

**Two evaluation definitions were corrected after the first live read, and both are
disclosed.** An actor sitting on a tile edge flickered between two tiles and was counted as
arriving dozens of times; an (actor, tile) pair now counts once. And the comparison of two
runs treated simulator-assigned actor ids as content; they are session identity and are
now excluded. Neither changes a measured value; both change how many there are.

**The test environment was fixed, then everything re-measured (ADR-054, Experiment 012).**
Placed actors now have physics off and stand on the road with `up_m` measured to the bottom
of the bounding box; the ego is grounded at the road surface. The walker stands at 0.93 m
on every frame, nothing settles, and a static scene is bit-repeatable across runs - the first
live repeatability in the project. Under the process defaults the parked car and the
pedestrian are detected on no frame at all: their returns cluster with the road because
ground segmentation is off by default (ADR-012). A disclosed variation with it on sees both
on every frame, with no identity switch in any scenario and the pipeline at 80 ms rather
than 140. The default is left as it was and the decision is handed on with the evidence.

**Retargeted, not weakened.** Three status assertions that a live run had not happened and
that prediction was unmeasured were retargeted to assert what is now true - live-validated,
simulation only, measured in simulation and still unmeasured for real data - each with a
stronger negative beside it.

**Status:** 1565 tests (1452 before), 7 deselected live, ruff and mypy clean, 17 endpoints
unchanged, no new dependencies. Uncommitted on `phase-11-evaluation`, branched from
`origin/main` at `fe64d1c`.

---

## Phase 12 — Dashboard · verified · live-streamed from CARLA

**Objective.** Make ADAPT-X observable without giving it a second brain: a console that
shows what the pipeline produced, where the adaptive map spends its detail, and what the
Phase 11 evaluation measured — including everything it measured badly — while computing
nothing of its own.

**Design first.** `docs/DASHBOARD.md` was written before any code: purpose, the choice of
static vanilla ES modules over a framework (zero dependencies, no build step), the two
labelled modes, where a live scene comes from, the point sample, the contracts consumed,
the eight views, the screen transform (+X up, +Y left), playback, security assumptions, and
the list of things the dashboard deliberately never computes.

**Backend, additive.** `SceneSnapshot` bundles one frame's outputs with a deterministic
stride sample of the point cloud and **has no field for ground truth** — a payload carrying
one is refused. `SceneService` keeps the latest and a sequence; `/ws/scene` polls the
sequence every 40 ms and pushes. The full-chain endpoint publishes what it produced; the
scenario runner gained a per-frame observer and the CLI a `--publish URL` flag, so a live
CARLA run streams into the browser through `POST /api/v1/scene/frame` — a hand-over, not a
computation. `adaptx.evidence` reads stored reports and runs from configured directories,
validates them, and serves them read-only: reports whole, runs as a summary plus one frame
at a time with ground truth beside the frame, so a 42 MB record is never parsed by a
browser. Comparison goes through the Phase 11 `compare_reports` unchanged. A `dashboard`
component (PARTIAL, phase 12, "CONSUMER ONLY") joined system status; the evaluation
component's text now says "computed only offline; stored reports served read-only".

**Frontend.** `dashboard/`: `app.js` (state, channels, navigation, a per-tab memory of
which report, run and overlays were chosen), `data/` (API wrappers, a reconnecting channel
with a stale indicator, deep-frozen report and run stores with frame prefetch, `format.js`
as the single place null becomes "Not available" and UNKNOWN stays "UNKNOWN", `normalise.js`
joining tracks with assessments and paths by track id), `render/` (a top-down view with +X
up and +Y left, a perspective camera behind and above the ego, height-coloured points,
boxes coloured by risk level with UNKNOWN dashed, velocity vectors, predicted paths, tiles,
small bar/line charts that draw the report's numbers untransformed), and eight views laid
out after the mockup — with every panel the backend cannot feed reading *Not implemented*
or *Not measured* in the same card footprint.

**What the frontend was refused.** During validation the views had grown `Math.hypot`
distances and an occupancy ratio of their own; all were removed and the assessment's own
`distance_m` shown instead. A boundary test now scans the source for `Math.hypot`/`sqrt`/
`atan2`, `.reduce(`, `predict(` and assignments to metric or threshold names outside
`render/`, for a second `fetch` site or a non-GET request, for `ground_truth` outside the
playback view model and the Run view, for "collision probability" / "production ready" /
"real-time", and for "Not available" written anywhere but `format.js`. Every route path is
audited for spawn / destroy / teleport / start / stop / control / tick.

**Boundary finding.** The evidence reader was first placed in `services/` and the Phase 11
test that no production package imports the evaluation layer caught it. It moved to its
own package `adaptx.evidence`, downstream of evaluation; the test was retargeted to pin
`api/routes/evidence.py` as the single API module that reaches evaluation, and the
pipeline stages, runner and application context remain clean.

**Live.** `python -m adaptx.scenarios run cyclist_crossing --publish http://127.0.0.1:8000`
against CARLA 0.9.16 published 80 frames; the browser drew the streaming point sample
(5,404 of ~27,000 points, stride 5), tiles, risk-coloured boxes and predicted paths; a click
selected track #2 and the inspector showed MEDIUM 0.594 with its null factors as "Not
available" and uncertainty beside, not inside, the score. A stored report loaded from the
rail; a 42 MB run played back at the recorded timestep; the two-report comparison came from
the backend; the backend was stopped and restarted with the page open and both channels
went stale → disconnected → live on their own; a refresh kept the view, report, run and
toggles. Browser costs are Experiment 013: ~10 ms to draw 6,000 points, ~30 ms per run
frame, ~3 s for the backend to parse a 42 MB run once, ~200 ms for a random seek that waits
behind prefetches.

**Fixed along the way.** The perspective projection's pitch sign (everything drew below the
canvas); a top-down fit against a hidden 0×0 canvas that collapsed the map; a paired chart
that put cells and milliseconds on one axis (now per-category scaling with a note); browser
module caching that hid edits (`Cache-Control: no-cache` on `/dashboard`); a circular import
between the context and the evaluation layer (lazy imports in the evidence service).

**Status:** 1621 tests (1565 before), 7 live CARLA tests pass from `.venv312`, 17 Node
tests, ruff / format / mypy clean (131 source files), 24 endpoints + 2 WebSockets, no new
dependency, ADR-055, Experiment 013. Uncommitted on `phase-12-dashboard`, branched from
`origin/main` at `c0a89dd`.

---

## Post-Phase-12 extension — Live simulation loop · verified live · the pipeline drives the car

**Objective.** Turn the console from a viewer of stored evidence into a window on a
running system: CARLA → ego LiDAR → the unchanged Phase 2-8 chain → a decision → the
CARLA vehicle → the dashboard, with nothing pre-recorded and nothing invented.
Explicitly not a Phase 13.

**Backend.** `adaptx.live` (`LiveSimulationService`: one thread, one session, the only
caller of `world.tick()`, latest-only snapshots with skipped counts, events derived from
output differences, measured timing every frame) and `adaptx.control`
(`RiskGovernedSpeedPolicy`: target speed per in-path risk level, safe-distance hold,
emergency brake, resume dwell, rate-limited setpoint, lane-centre steering; a
`VehicleController` protocol). `CarlaSimulationSession` extended additively: `drive_ego`,
`apply_ego_control` (the one sign flip), anchored placement, Traffic Manager traffic,
collision sensor, RGB camera, batch destroy. A six-entry live scenario catalogue.
`SceneSnapshot` gained `ego`, `control`, `live`. Nine `/api/v1/live/*` endpoints: reads
plus five high-level controls. Components `live_simulation` and `vehicle_control`.

**Frontend.** LIVE SIMULATION / STORED EVALUATION mode switch (live by default; CARLA
DISCONNECTED shown plainly, never a recording), session controls in the rail, a Front
View from the ego camera, Recent Events from the loop, System Status with measured FPS
and sim/wall speed, Decision / Control and Performance cards, a Risk Map from the
per-tile risk factor the resolution controller recorded, a corrected CARLA pill.

**What the live server taught, in order.** Scene-maximum risk pinned the ego at the
kerb (poles 5.5 m beside the road score HIGH/CRITICAL on proximity) → the governor keys
on in-path objects. A proportional throttle stalled at 1.6 m/s under a 2 m/s target →
hold throttle. Track identity churn on the parked car flickered the level → setpoint
ramp, coalesced events. Destroying Traffic Manager vehicles in a synchronous world
aborted the client process and leaked every actor → the CARLA examples' shutdown order.
A fresh client sees a stale actor list on a server left synchronous → cleanup switches
async and waits a frame. Velocities are ego-relative → recorded as a limitation, not
patched.

**Live.** Experiment 014: `static_obstacle` ×2 seeds, `mixed_obstacles` with six TM
vehicles, `pedestrian_crossing`. The ego reached 5.1 m/s, slowed from ~16 m, held
6.3-7.7 m short of the in-path object, resumed after it left; 0 collisions; loop median
150-164 ms per 50 ms frame (0.30-0.33x wall-clock, LAGGING shown); from the dashboard's
own Start and Stop buttons, with no actor left on the server and the world restored.

**Tests.** Fake simulator extended (kinematics under `apply_control`, attached sensors,
Traffic Manager, collision and camera sensors). New: `tests/unit/test_control_policy.py`
(18), `tests/unit/test_live_scenarios.py` (10), `tests/integration/test_live_simulation.py`
(12, including the obstacle-stop demo against the fake), three live cases in
`test_carla_live.py`, frontend scans for random numbers, videos, a single camera URL site
and mode separation, the route audit retargeted to allowlist the five controls, the Phase
11 ground-truth boundary extended to `control` and `live`.

**Status:** 1663 tests (1621 before), 10 live CARLA tests pass, 18 Node tests, ruff /
format / mypy clean (141 source files), 33 API endpoints + 2 WebSockets, no new
dependency, ADR-056, Experiment 014. Uncommitted on `phase-12-dashboard`.

---

## Live perception upgrade — verified live · objects labelled once, in the backend

**Objective.** Make every visible object carry a meaningful class, distance, speed, path
relation and risk from the pipeline's own outputs - and find out why they did not.

**Audit first, on the server.** A probe recorded every track per frame and, offline only,
read ground truth to say which cluster was the scripted actor. Findings: 671 "pedestrian"
track-frames a minute with no walker present, nearly all fragments 0.9-3.5 m above the
road (signs, foliage) or labels that stuck after the cluster grew out of every band; a
parked car never once a VEHICLE (seen from behind it is 1.7-1.9 m across and 0.5-1.1 m
tall, which no band called a car); its id changed five times, all between 26 and 20 m
where a 16-21-point detection came on alternate frames and a one-hit tentative track died
on each miss; and the pedestrian scenario's walker had never spawned at spawn point 1.

**Changes, each measured before and after (ADR-057, Experiment 015).** Detector: reject
clusters whose bottom is more than 0.8 m above the ground stage's floor (`floor_estimate_m`,
the median z of the removed ground points; walker worst case 0.63 m, fragments >= 0.96).
Tracker: a label lapses after three UNKNOWN observations; tentative tracks survive two
misses. Classifier: the vehicle band accepts a 1.5-6.5 m long face (`geometric_bands_v2`).
Result: false "pedestrians" 671 -> 37, the car VEHICLE on 269 of 357 frames with two ids
instead of six, the walker PEDESTRIAN on 101 of 144. A riderless bicycle stays mostly
UNKNOWN/OBSTACLE - reported, not fixed.

**The object record.** `adaptx.control.corridor` holds the one IN_PATH / CROSSING /
BEHIND / OUTSIDE rule; the policy and `object_records()` both call it. Each snapshot
carries a `TrackedObjectSnapshot` per track with distance (risk engine), longitudinal and
lateral distance (track x, y), ego-relative and closing speed, risk, path relation,
fit-score confidence (null for UNKNOWN), hits, age, prediction horizon. The dashboard
gained object cards, `#id CLASS 14.8m HIGH` scene labels with `IN PATH` beneath, a path
column, an inspector block, and `entered_path` / `crossing_path` / `left_path` events.

**Performance.** cProfile put 78 of 135 ms in the adaptive resolution stage, led by
`TileGrid.tile_bounds` called ~1,230 times a frame. Memoising tile bounds and cell shapes
per grid (values unchanged) and skipping re-validation of the internally built point
sample: loop 160 -> 120 ms, pipeline 126 -> 84 ms, sim/wall 0.31 -> 0.42.

**Live.** Every scenario: 0 collisions, holds 7.45-7.98 m short of the in-path object,
resumes when it leaves; the roadside walker enters the corridor, stops the ego, leaves,
and the ego resumes with the matching events. `pytest -m carla` 12/12.

**Status:** 1684 tests (1663 before), 12 live, 20 Node, ruff / format / mypy clean (142
source files), no new dependency, ADR-057, Experiment 015. Uncommitted on
`live-perception-upgrade`.

---

## Debug: "the front vehicle stopped moving" · orphaned actors · Experiment 016

**Report.** Ego STOPPED behind `VEHICLE #17 at 7.8 m, 0.0 m/s` in `vehicle_cut_in`.

**Traced before touching code.** Scripted cut-in car: physics off, placed per frame,
moved 55.7 m in 17 s (CARLA velocity 0 by design; LiDAR-tracked 0.3-4.9 m/s). TM cars:
physics on, autopilot on TM 8050 synchronous seeded, 20-37 m in 6 s. Only the live loop
ticks. No actor reset to its initial transform. The stationary vehicle was one of ~25
Traffic-Manager cars orphaned by backend processes the desktop app terminated without a
Stop; the world was also left synchronous, which makes a fresh client's actor list empty
until one frame is produced.

**Fix.** `open()` reclaims ADAPT-X-tagged orphans (and attached sensors) after one
bootstrap frame and releases the orphaned synchronous mode; scripted actors are tagged
`adaptx_scenario`; `CarlaSettings.reclaim_stale_actors`. No controller, threshold or
physics change. Verified by recreating the damage on the real server (5 reclaimed, car
moved 28 m in 10 s, 0 collisions, 0 actors after stop) and by the five required scenarios.

**Status:** 1689 tests, 12 live, ruff / format / mypy clean. Uncommitted on
`live-perception-upgrade`.

---

## Cross-phase pattern

Each phase ships a **deterministic, explainable baseline** behind an interface, labelled
`is_baseline`, with its failure modes documented **and asserted by tests** so they stay
visible. Phase 9 declared `carla` as an **optional extra** rather than a dependency: the
backend, the endpoints and the whole test suite still run without it, so the required set is
unchanged after ten phases.

Phase 11 made the honest negative the whole phase: the first correctness figures in the
project's history are mostly unflattering, they are recorded with the method, the seed, the
gate and the confound beside them, and nothing was tuned to improve them.

Phase 8 added a second pattern worth naming: **the honest negative**. The phase the project
is named for produced a result that is partly unflattering — slower than the baseline, and
more expensive than a coarse uniform map — and the measurement was recorded as taken, with
the cause identified rather than explained away. A benchmark that could only ever confirm the
premise would not have been worth running.
