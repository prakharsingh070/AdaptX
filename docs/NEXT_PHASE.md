# Next Phase — Risk and Uncertainty (Phase 7)

Handoff for the next work item. Read [`PROJECT_STATE.md`](PROJECT_STATE.md) first.

> **Numbering is settled.** Prediction is Phase 5 (done), 2.5D mapping Phase 6 (done), risk
> and uncertainty **Phase 7**, adaptive resolution **Phase 8**. `CLAUDE.md`, `ROADMAP.md`
> and `services/system_service.py` all agree. Do not renumber.

---

## Objective

Implement `risk.interfaces.RiskEngine` as the real ADAPT-X engine, producing the `RiskField`
/ `RiskCell` / `ObjectRisk` / `RiskFactors` contracts that already exist and are currently
unused. Keep `BaselineProximityRiskEngine` alive and comparable — it is the reference the
real engine must beat (ADR-003's reasoning, applied to risk).

Phase 7 is the phase where ADAPT-X's inputs finally meet. It is the first module entitled to
read tracks, trajectories **and** the map together.

## Inputs available — all of them, for the first time

| Source | What it gives you |
|---|---|
| `TrackingResult.tracks` | `TrackedObject` with measured velocity (or `None`), class, age, hits |
| `PredictionResult.trajectories` | `PredictedTrajectory` with extrapolated positions, **heuristic** `position_uncertainty_m` and evidence-based `confidence` |
| `PredictionResult.skipped` | Tracks that could **not** be predicted, with reasons. A track with no measured velocity is not a safe track — it is an unknown one |
| `SpatialMap` | `point_count`, `min/max/mean height` arrays, binary occupancy, bounds |
| `VehicleState` | Contract exists; **nothing populates it**. Ego motion is your first real gap |

Read `PROJECT_STATE.md` §15 before using any of it. Every input is a baseline with
documented failure modes.

## The three traps in this phase

**1. Uncertainty is not calibrated.** `position_uncertainty_m` is a documented heuristic
(ADR-026), not a sigma. If risk is computed as though it were a real distribution, the
output inherits a precision that was never measured. Treat it as an ordering signal, and say
so wherever risk is reported.

**2. Unobserved is not free (ADR-031).** A map cell with no points may be empty *or*
occluded, and Phase 6 cannot tell the difference. Treating unobserved cells as free space is
the single most dangerous thing this phase could do. Either model occlusion explicitly or
treat unobserved as unknown-and-therefore-not-safe, and document which.

**3. `velocity is None` is not `velocity == 0`.** A track on its first frame has no measured
motion. It must not be scored as stationary-and-therefore-safe.

## Expected outputs

The **existing** contracts in `models/risk.py` — do not create parallel ones. Following
ADR-022 / ADR-024 / ADR-027, return a result object carrying the field, what was excluded and
why, measured durations, and a configuration snapshot.

`RiskFactors` exists specifically so a score is **attributable**: a risk value that cannot be
decomposed into what caused it is not usable evidence for a later resolution decision.
Populate it.

Also populate `AdaptiveMapCell.risk_score` and `uncertainty`, which Phase 6 deliberately
leaves at their defaults.

## Scope boundary — what Phase 7 owns and what it must not touch

Phase 7 owns: collision risk, time-to-collision, trajectory overlap, risk scoring, risk
zones, the uncertainty engine.

Phase 7 must **not** implement `ResolutionController` or change any cell size. Deciding how
much detail a region deserves is **Phase 8**, and it consumes the risk this phase produces.
Keeping them apart is what makes the central claim measurable (ADR-029).

## Configuration

`RiskSettings` already exists (`ADAPTX_RISK__`) with `max_range_m` and level thresholds.
Extend it rather than adding a section, and document any new field in `.env.example`.

> Note: `.env.example` still has no `DETECTION` or `TRACKING` section — a gap from Phases 3
> and 4. A small, welcome side task; keep it in its own commit.

## API, telemetry, status

- `GET /api/v1/risk/status` exists — extend it; do not replace it.
- A frame-level endpoint (`POST /api/v1/lidar/risk`) matching `/detect`, `/track`,
  `/predict`, `/map` is the obvious addition. Decide statefulness explicitly.
- Telemetry gains a risk **summary**; remove `"risk_field"` from `_NOT_YET_AVAILABLE` only
  once it genuinely exists. Do not stream cells.
- Update the `risk` component to `READY` / `PARTIAL`. **Never `IMPLEMENTED`.** Note it is
  currently the only component still `NOT_READY`.

## Testing

Deterministic scenes with known geometry and known motion:

- a stationary object far away scores low; the same object closing fast scores higher
- a track with `velocity is None` is **not** scored as safe
- an unobserved cell is not scored as free
- risk stays within `[0, 1]` (ADR-006) and the level thresholds partition it
- `RiskFactors` decomposes to the reported score
- the baseline engine and the real engine both run over the same input and are distinguishable
- empty scene, single object, objects at bounds
- determinism: same input, same field
- no NaN/Inf anywhere in outputs
- integration: raw frame → process → detect → track → predict → map → risk
- API contract and OpenAPI documentation

Add a risk benchmark (`--risk`) and record it as Experiment 006. **Measure the real engine
against the proximity baseline on identical input** — that comparison is the first evidence
for the risk half of the project's claim.

## MUST NOT implement

- The resolution controller or any cell-size change (Phase 8)
- Learned risk models
- CARLA scenarios or the dashboard
- Any new dependency without an ADR

## Backward-compatibility rules

1. Do not change the coordinate convention (ADR-009).
2. Do not modify Phase 1–6 algorithms unless a measured defect justifies it.
3. Do not change existing endpoint behaviour — extend additively.
4. Do not weaken or delete tests. If a premise genuinely changes (as in Phases 3–6 when a
   module stopped being "planned"), retarget the test to guard the same property and report
   it.
5. If an existing file must change: explain why, make the smallest change, preserve
   compatibility, add a regression test, and list it in the final report.
6. Reuse `RiskField` / `RiskCell` / `ObjectRisk` / `RiskFactors`; do not duplicate contracts.
7. Keep the honesty rules: unmeasured values are `null` with a reason, `source` provenance is
   mandatory, baselines are labelled `is_baseline`, risk stays normalised to `[0, 1]`
   (ADR-006), and **no fabricated accuracy or performance figures**.
