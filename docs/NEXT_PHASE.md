# Next Phase — Trajectory Prediction

Handoff for the next work item. Read [`PROJECT_STATE.md`](PROJECT_STATE.md) first.

> **Numbering:** the repository currently numbers trajectory prediction as **Phase 8**
> (`ROADMAP.md`, `CLAUDE.md`, and the `prediction` status component with `phase=8`), with
> Phase 5 = 2.5D mapping. The most recent instruction called it **"Phase 5"**. Nothing has
> been renumbered. **Ask the user which numbering to use before writing status text**, then
> make `ROADMAP.md`, `CLAUDE.md` and `system_service.py` agree.

---

## Objective

Implement `prediction.interfaces.TrajectoryPredictor` as a **constant-velocity baseline**
that extrapolates each tracked object's future path, producing the `PredictedTrajectory`
contract that already exists and is currently unused.

This is the last piece the risk engine needs. It is also the phase most at risk of
overclaiming: a predicted position looks exactly like a measured one in JSON.

## Inputs from Phase 4

`TrackedObject` instances from `TrackingResult.tracks`, each carrying:

| Field | Notes for prediction |
|---|---|
| `position` | Measured, always present |
| `velocity` | **`Vector3 \| None`** — null until two observations, and for bad intervals |
| `observed_velocity` | Raw, unsmoothed. Available if the smoothed value is unsuitable |
| `acceleration` | `None` until two consecutive velocities. **Do not** use it in the baseline |
| `heading_rad` | `None` below the speed floor |
| `status` | `tentative` / `confirmed` / `coasting` |
| `hits`, `age_frames`, `missed_frames` | Evidence quantity, useful for confidence |
| `predicted_position` | **Association gating state, not a forecast.** Do not confuse the two |

## Expected outputs

The **existing** contracts in `models/prediction.py` — do not create parallel ones:

- `PredictedTrajectory`: `track_id`, `horizon_s`, `timestep_s`, `points` (min 1),
  `confidence`, `predictor_name`, `coordinate_frame`, `source`
- `TrajectoryPoint`: `time_offset_s`, `position`, `velocity`, `confidence`,
  `position_uncertainty_m`

Validators already enforce that points are time-ordered and within the horizon.

Following ADR-022/ADR-024, add a **`PredictionResult`** wrapper carrying the trajectories,
the tracks skipped and why, measured durations, and a configuration snapshot. Change the
abstract `predict()` signature to return it — it has no implementations, so this is safe.

## The baseline

Constant velocity: `position(t) = position + velocity × t`. Nothing more.

Do **not** use acceleration, road geometry, class-conditioned motion models, or interaction
between objects. Those are later work, and each would need validation the project cannot
currently perform.

- **Horizon:** configurable, default around 3.0 s (`knowledge-base/09_prediction.md`
  suggests t+1, t+2, t+3)
- **Interval:** configurable, default around 0.5 s
- Both belong in a new `PredictionSettings` section (`ADAPTX_PREDICTION__`), consistent with
  `DetectionSettings` and `TrackingSettings`

## Uncertainty

`position_uncertainty_m` must **grow with horizon** — a 3-second extrapolation is far less
trustworthy than a 0.5-second one. A defensible baseline is uncertainty proportional to
`speed × time_offset`, plus a floor reflecting measurement noise. Whatever the formula:

- Document it exactly, as ADR-021 did for the classification fit score.
- Do **not** call it a probability or a confidence interval unless it genuinely is one.
  It is not calibrated, because no labelled data exists to calibrate it against.

`confidence` should likewise reflect *evidence*, not correctness — track age, hits, whether
the track is coasting.

## Required behaviours

**`velocity is None` → emit no trajectory.** A track with no measured motion cannot be
extrapolated. Record it as skipped with a reason. Emitting a flat "stays where it is"
trajectory would fabricate a prediction from nothing (ADR-023 exists precisely to stop this).

**Zero velocity is different.** A *measured* standstill is real information. Emitting a
stationary trajectory is legitimate — `is_moving is False`, not `None`. Handle the two cases
separately and test both.

**Track lifecycle:**
- `confirmed` — predict normally
- `tentative` — either skip, or predict with visibly lower confidence. Decide and document
- `coasting` — the position is already stale by `missed_frames`. Either skip, or widen
  uncertainty to account for the gap. Do not treat it as a fresh observation

**Predictions must never be mistakable for observations.** They live only in
`PredictedTrajectory`, never written back onto `TrackedObject.position`.

## API

Extend the existing architecture; do not break anything.

- Suggested: `POST /api/v1/lidar/predict` (process → detect → track → predict), matching the
  `/detect` and `/track` shape. Stateful, for the same reason `/track` is.
- Possibly `GET /api/v1/prediction/status`.
- Response carries trajectories and measured timings; **no raw point arrays**.

## Telemetry

Add a prediction **summary** — counts, configuration, horizon. Remove
`predicted_trajectories` from `_NOT_YET_AVAILABLE` in `websocket/telemetry.py` **only once
it genuinely exists**. Do not stream full trajectories on every tick; that is frame data on
a status channel.

Update the `prediction` component in `services/system_service.py` to `READY` / `PARTIAL`
with a detail naming what it cannot do. **Never `IMPLEMENTED`** — it is a baseline.

## Testing

Deterministic sequences with known motion, following `tests/fixtures/sequences.py`:

- constant velocity forward / lateral / diagonal — predicted positions must match arithmetic
- stationary track (measured zero) → stationary trajectory
- `velocity is None` → **no trajectory**, recorded as skipped
- horizon and interval boundaries; point count equals `horizon / timestep`
- uncertainty strictly increases with `time_offset_s`
- coasting and tentative track handling
- empty track list
- determinism: same input, same output
- no NaN/Inf anywhere in outputs
- integration: raw frame → process → detect → track → predict across several frames
- API contract and OpenAPI documentation

Assert *properties*, not brittle values, where an algorithm allows numerical variation.

Add a prediction benchmark (`--predict`) mirroring `--detect` / `--track`, and record
measured results in `experiments/experiment-log.md` as Experiment 004.

## MUST NOT implement

- Collision prediction or conflict detection (that is the risk engine)
- The final risk engine, adaptive resolution, or the 2.5D map
- Learned or interaction-aware prediction models
- Map- or lane-conditioned prediction
- CARLA scenarios or the dashboard
- Any new dependency without an ADR

## Backward-compatibility rules

1. Do not change the coordinate convention (ADR-009).
2. Do not modify Phase 1–4 algorithms unless a measured defect justifies it.
3. Do not change existing endpoint behaviour — extend additively.
4. Do not weaken or delete tests. If a premise genuinely changes (as happened in Phases 3
   and 4 when a module stopped being "planned"), retarget the test to guard the same
   property and report it.
5. If an existing file must change: explain why, make the smallest change, preserve
   compatibility, add a regression test, and list it in the final report.
6. Reuse `PredictedTrajectory` / `TrajectoryPoint`; do not create duplicate contracts.
7. Keep the honesty rules: unmeasured values are `null` with a reason, `source` provenance is
   mandatory, baselines are labelled `is_baseline`, and **no fabricated accuracy or
   performance figures**.
