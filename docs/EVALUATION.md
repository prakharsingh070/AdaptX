# Evaluation (Phase 11)

How ADAPT-X measures its own behaviour against controlled simulator ground truth, without
the system under evaluation ever seeing that ground truth.

> **Simulation evidence, not real-world validation.** Every number this layer produces is
> computed from a CARLA run. It says nothing about physical LiDAR, real traffic, or safety.
> The heuristic risk score is not a probability of collision and no figure here treats it as
> one. No scenario in the catalogue contains a collision, so nothing here measures collision
> prediction.

---

## 1. What is evaluated, and from what

```
CARLA ──► ScenarioRunner ──► ScenarioRunResult (.json)  ──► adaptx.evaluation ──► EvaluationReport
                │                    │                              (offline, no simulator)
                │                    ├─ frames[i].outputs   what the pipeline PRODUCED
                │                    ├─ frames[i].expected  what the scenario COMMANDED
                │                    └─ ground_truth[i]     what the simulator REPORTED
                └─ Phase 2-8 pipeline (never sees ground_truth)
```

The run record is the only input. Once a run has been written with `--json`, evaluation runs
with CARLA closed, on any machine, and produces the same deterministic content every time.

### 1.1 The evidence extension (Phase 10 contract, additive)

Phase 10 recorded per-frame **counts** (`StageCounts`). Position error, ADE/FDE, per-object
risk and per-tile resolution cannot be computed from counts, so Phase 11 adds one optional
field to `ScenarioFrameRecord`:

| Field | Type | Contents |
|---|---|---|
| `outputs` | `PipelineFrameOutputs \| None` | The existing Phase 3–8 result contracts for that frame: `DetectionResult`, `TrackingResult`, `PredictionResult`, `RiskAssessmentResult`, `SpatialMapSummary`, `AdaptiveSpatialMapSummary`, `ResolutionPlan`, `MappingComparison` |

and one to `ScenarioRunResult`:

| Field | Type | Contents |
|---|---|---|
| `sensor` | `SensorConfiguration \| None` | The LiDAR configuration the run used: channels, rate, range, field of view, drop-off and the **mount offset** — needed to relate the sensor frame to the ego frame |

Both are raw evidence in the Phase 10 sense: no accuracy, error or match figure appears on
the run record, and the existing test asserting that stays. `pipeline_processor` records
outputs by default; `record_outputs=False` restores the counts-only behaviour, and every
earlier consumer of the result is unaffected because the fields default to `None`.

Recording costs nothing the pipeline was not already computing — the result objects exist;
they are now kept rather than reduced to counts.

### 1.2 What the run record does NOT contain, and therefore cannot be evaluated

- **The point cloud.** Only its count. Nothing here re-runs perception.
- **The map grids.** Summaries only — cells, bytes, occupancy, timing. There is no way to
  compare the map's occupancy against a reference, and none is manufactured (§6).
- **Process memory.** Not sampled during the run. Reported as unavailable.
- **Static scene geometry.** Buildings, poles and parked meshes produce LiDAR returns and
  detections but are not CARLA *actors*, so ground truth does not list them. A track with no
  ground-truth match is therefore **unlabelled**, never a false positive (§3.2).

---

## 2. Ground-truth boundary

Ground truth reaches exactly one package: `adaptx.evaluation`. It reads `ScenarioRunResult`
and `GroundTruthFrame`; nothing in `adaptx.perception`, `tracking`, `prediction`, `mapping`,
`risk` or `services` imports `adaptx.evaluation` or `adaptx.carla.ground_truth`, and tests
assert both directions in a subprocess with a clean module table.

Two facts about the recorded ground truth matter for every metric:

**Frames.** Perception output is in the **sensor** frame (origin at the LiDAR, 1.8 m above
the ego origin by default; ADR-013 applies no transform). Ground truth is relative to the
**ego actor origin**. Evaluation moves ground truth into the sensor frame by subtracting the
recorded mount offset — the one conversion it performs, done once in `dataset.py` and
tested. Planar (x, y) error is the primary position metric because the two frames share
their x and y axes when the mount has no lateral offset; the 3-D error is reported beside it
and labelled as including the actor-origin-to-detection-centroid offset, which is inherent
to comparing a mesh origin against a point-cloud centroid.

**Velocity.** Scenario actors are *placed* every frame (ADR-047), not driven, so the
simulator's `get_velocity()` reports a value unrelated to the scripted motion: ~0 in the
plane while the actor moves at 8 m/s (and, before ADR-054 switched their physics off, a
vertical component from falling between placements). The recorded
`GroundTruthActor.velocity` is therefore **not used**. The reference velocity is the finite
difference of consecutive ground-truth positions, `(p[i] − p[i−1]) / Δt`, which is exact for
scripted constant-velocity segments and is the same quantity the tracker estimates.

---

## 3. Matching

Tracks and ground-truth actors carry different identifiers (`track_id` is tracker-local;
`actor_id` is the simulator's). Matching is done **per frame** and is the basis of every
tracking, prediction and risk metric.

**Ground truth considered:** by default only the *scenario actors* — the actors the scenario
spawned, linked by `ScenarioRunResult.actors[].simulator_actor_id`. The map's traffic signs
are real actors but are not what the scenarios exercise; including them is a configuration
option (`include_map_actors`) and is reported when used.

**Eligibility:** an actor-frame is *eligible* when the actor is within the recorded sensor
range and inside the fixed map's bounds. Every actor takes part in matching, eligible or
not — a track on an actor outside the map bounds is still a track on something real and
must not be counted as unlabelled — but only eligible actor-frames enter a *rate*.

**Algorithm — greedy nearest neighbour, planar, gated:**

1. Compute the planar distance between every (track, actor) pair.
2. Sort pairs by (distance, track_id, actor_id) — ties resolved by id, so the result is
   deterministic.
3. Walk the list; accept a pair if neither side is already matched and the distance is
   within `match_gate_m`.

Optimal (Hungarian) assignment was considered and not used: the catalogue never has more
than two scenario actors per frame, greedy and optimal coincide unless two actors are
within one gate of one track, and no dependency is available or justified for it. If a
future scenario has dense actors, the decision should be revisited (see the ADR).

**Thresholds** are configuration, labelled baseline, recorded on every report, and results
are computed at every gate in `match_gates_m` (default `[1.0, 2.0, 4.0]`) so no single
headline depends on one choice. Class agreement is **reported**, never required for a
match: the Phase 3 classifier is itself under evaluation.

### 3.2 What a match, a miss and an unmatched track mean

| Situation | Counted as |
|---|---|
| Actor has a track within the gate | matched |
| Actor has no track within the gate | missed (actor within sensor range) |
| Track has no actor within the gate | **unlabelled** — could be static geometry, a sign, or a spurious cluster; the record cannot say |

"Precision" over unlabelled tracks would be a fabricated number and is not reported.

---

## 4. Metrics

Every section of the report carries a `status`:

| Status | Meaning |
|---|---|
| `measured` | Computed from the evidence as defined |
| `partial` | Computed, but some inputs were missing; the reason and the counts are given |
| `unavailable` | Could not be computed from this record; the reason is given |
| `not_applicable` | Does not apply to this scenario (e.g. no moving actor) |

A metric that could not be computed is `null` with a reason. **Never zero.**

Distributions are summarised as `count, mean, median, rmse, min, max, p95`. `p95` is
computed by NumPy's linear interpolation and reported only when `count >= 20`; below that
it is `null` with the rule stated, because the 95th percentile of twelve samples is the
maximum wearing a costume.

### 4.1 Detection (per frame, per scenario actor)

- **recall at gate** — frames where a detection lies within the gate of the actor, over
  frames where the actor is within the sensor range and the map bounds.
- **position error** — planar distance from the nearest detection to the actor origin.

### 4.2 Tracking (per frame, matched pairs)

- **match rate** — matched (actor, frame) pairs / eligible (actor, frame) pairs, per gate.
- **position error** — planar, and 3-D in the sensor frame, over matched pairs.
- **velocity error** — `|v_track − v_ref|` over matched pairs where the track has a measured
  velocity **and** a reference velocity exists (frame ≥ 1). Frames where the track's
  velocity is `null` are counted and reported separately: null is not zero (ADR-023).
- **continuity**, per actor: frames present, frames matched, coverage, distinct track ids,
  **id switches** (distinct ids − 1), **fragments** (maximal runs of consecutive matched
  frames), longest fragment, and the lifecycle status distribution of the matched track.
  No MOTA/MOTP: the unlabelled-track problem (§3.2) makes the false-positive term
  undefined, and a MOTA without it would not be MOTA.

### 4.3 Trajectory prediction (ADE / FDE)

For every trajectory emitted on frame *i* for a track matched to actor *a* on frame *i*
(eligibility is not required here — a future position is a future position):

- A trajectory point at offset `τ` is compared with the actor's ground-truth position on
  frame `i + τ/Δt` **only if `τ/Δt` is an integer** (with the catalogue's 0.05 s step and
  0.25 s interval it always is) **and that frame exists in the run**. No interpolation.
- **ADE** = mean planar displacement over the aligned points of one trajectory;
  **FDE** = displacement at the last aligned point. Both then summarised across trajectories.
- Trajectories with no aligned future frame (predicted too close to the end of the run) and
  points beyond the run are counted as skipped, with the reason. Tracks the predictor
  itself skipped (`insufficient_velocity`, …) are reported by reason from the record.
- **Horizon coverage** — aligned points / points in the trajectory.
- Per-offset error is also reported (error at t+0.25, t+0.5, …) so the growth of error with
  horizon is visible.

The ego is stationary in every catalogue scenario, so ego-relative ground truth on a later
frame is directly comparable. If a scenario ever moves the ego, this comparison must be
re-expressed in the world frame first; the evaluator checks the ego's ground-truth position
is constant and marks the section `partial` otherwise.

### 4.4 Risk (against proximity events)

No collision occurs in any catalogue scenario, so there is no collision label and no
collision metric. What ground truth *does* support objectively is **proximity**:

- **event** — an (actor, frame) where the actor's planar ground-truth distance is
  ≤ `proximity_event_m` (baseline 20 m, configurable).
- **alert** — the matched track's `risk_level` ≥ `alert_level` (baseline `high`).

Reported: event frames; **alert recall** on event frames; **lead time** — first alert time
minus first event time for the actor (negative means the alert came after the actor was
already inside the band); alerts while the actor was outside the band (reported as *early
alerts*, not false alerts); alerts on unlabelled tracks (count only); **risk ordering** —
over all frame pairs for one actor where both frames carry a score, the fraction where the
higher score coincides with the smaller distance (1.0 = perfectly ordered, 0.5 = no
relation); and the number of matched frames whose risk was `UNKNOWN` (`risk_score = null`),
which is reported and **never** converted to a level or a zero.

### 4.5 Mapping

**Occupancy accuracy is not evaluated in Phase 11.** The record holds no map grid, actor
ground truth is not an occupancy reference (static geometry is unlabelled), and building
one from simulator ray casts would be a Phase 9 boundary change made after the fact.
What is reported: dimensions, total and occupied cells, occupancy ratio, mapped and
out-of-bounds points, grid bytes and construction time, for both the fixed and the adaptive
map, per frame.

### 4.6 Adaptive resolution (the central question)

Fixed and adaptive maps are built from the **same processed frame** of the **same run**, so
the comparison is paired by construction; only the resolution policy differs. Per frame,
from the `ResolutionPlan`, the two map summaries and the `MappingComparison`:

- **resolution distribution** over tiles: min, max, mean, median, p95 cell size, unique
  values, and the area-weighted mean.
- **cells and bytes** — fixed vs adaptive totals and the ratio.
- **construction time** — fixed vs adaptive (mapping only, and controller + mapping).
- **detail at the actor** — the level of the tile containing each scenario actor's
  ground-truth position, versus the level distribution of every other tile. This is the
  policy's intent stated as a measurement: does the tile holding the object get finer
  detail than the rest?
- **detail by risk level** — the actor tile's cell size grouped by the matched track's risk
  level; `UNKNOWN` kept as its own group.
- **refinement lead time** — for every (actor, tile) pair, counted once at the actor's first
  frame in that tile: the frame the actor entered minus the frame the tile first reached
  ≥ `medium`. Positive = the region was refined before the object arrived, which is the
  project's stated aim; negative = after; null = never during the run. (An actor sitting on
  a tile edge flickers between two tiles with simulator jitter; counting once per pair keeps
  that from multiplying one arrival into many.)
- **churn** — tiles changed per frame, share of frames with any change, per-tile transition
  counts, and **reversals** (a tile returning to a level it held within `min_dwell_frames`
  frames).
- **holds** — decisions carrying `hysteresis_hold` or `dwell_hold`: the transitions the
  stabiliser reports having prevented. Reported as counts; whether preventing them was
  beneficial is not claimed.
- **budget** — frames where the plan was over budget or demoted tiles.

### 4.7 Resource

Per-stage durations from the record (median, p95, count) — measured on the machine that
ran the scenario, recorded in the report's environment. **Not** real-time claims: no budget
is being evaluated. Peak process memory: unavailable (not sampled during the run).

---

## 5. Reproducibility

An `EvaluationReport` records: scenario id, seed, simulator version, map, timestep, frame
count, the run's `sensor` configuration, every stage configuration (already carried on the
first frame's outputs), the evaluation configuration, `adaptx` version, the git commit of
the evaluating checkout when one can be read (else `null`), and the timestamp.

Two runs of the same scenario, seed and configuration should produce identical
**deterministic content** — matches, errors, cells, levels, churn — and different
**timings**. `python -m adaptx.evaluation compare a.json b.json` checks exactly that split
(timings, evaluation identity and simulator-assigned actor ids excluded) and exits non-zero
if deterministic content differs.

**What was found (Experiments 011, 012):** the pipeline is deterministic — two
fake-simulator runs evaluate identically, and a test asserts it. Live CARLA runs were not
repeatable at all unseeded; seeding the LiDAR from `carla.seed` (which previously seeded
nothing) and taking physics off placed actors (ADR-054) made a static scene bit-repeatable.
Scenes with moving placed actors still differ by at most 8 points in 27,000 on some frames,
which the geometric detector can amplify into a different cluster, track or tile level;
`compare` reports those as not identical, and that is the correct report.

---

## 6. Results

The measured figures, on CARLA 0.9.16 / Town10HD_Opt from spawn point 1, are in
`docs/experiments/experiment-log.md`: Experiment 011 (first run; found a placement defect
in the test environment) and Experiment 012 (re-measured after ADR-054 fixed it, under the
process defaults and under a disclosed ground-segmentation-on variation). They are
simulation evidence and nothing else.

## 7. What Phase 11 deliberately does not do

- Build an occupancy reference (§4.5).
- Compute MOTA/MOTP or any precision over unlabelled tracks (§3.2).
- Use the recorded ground-truth velocity (§2).
- Interpolate ground truth between frames (§4.3).
- Tune any Phase 2–8 parameter in response to a result. Measurement only.
- Replay: evaluation reads the record; nothing re-drives the pipeline.
- Expose evaluation over HTTP or telemetry. It is an offline research instrument.
