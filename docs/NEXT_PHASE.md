# Next Phase — CARLA Integration (Phase 9)

Handoff for the next work item. Read [`PROJECT_STATE.md`](PROJECT_STATE.md) first.

> **Numbering is settled.** Prediction is Phase 5 (done), 2.5D mapping Phase 6 (done), risk
> and uncertainty Phase 7 (done), adaptive resolution Phase 8 (done), CARLA **Phase 9**.
> `CLAUDE.md`, `ROADMAP.md` and `services/system_service.py` all agree. Do not renumber.

---

## The perception chain is complete. Nothing has ever seen real data.

Phases 1–8 built the whole pipeline: processing, detection, tracking, prediction, mapping,
risk, and the adaptive resolution controller that ties them together. Every stage works, is
tested, and is measured.

Every stage has also only ever been fed geometry this repository generated for itself.

That is the single root of almost every limitation in `PROJECT_STATE.md` §15. Detection
accuracy, tracking correctness, prediction accuracy, map correctness, whether the risk
ordering is sensible, whether the resolution allocation is *appropriate* — all of them are
recorded as "unmeasured and unmeasurable", and all for the same reason: **no labelled data
exists**.

Phase 9 is the first thing in the roadmap that can change that. CARLA knows where every
object actually is, where it actually goes, and what is actually occupied. It is the
precondition for measuring correctness rather than cost.

## Objective

Complete `adaptx.carla.client.CarlaClient`. Every method below currently raises an explicit
"not implemented in Phase 1" error, which is the honest placeholder it was built as:

- sensor attachment and LiDAR frame retrieval
- ego-state extraction
- actor spawning and cleanup
- world/settings configuration

The boundary already exists and is already exercised by `CarlaMockClient`. Phase 9 fills it
in; it does not redesign it.

## What already works, and must keep working

- `CarlaService` and the `/api/v1/carla/status` endpoint.
- `CarlaMockClient` — the in-process fake. It must stay, and must stay clearly labelled: data
  produced through it is `SYNTHETIC_TEST` and must never be presented as sensor output.
- The whole perception chain, which consumes `RawPointCloudFrame` and does not care where it
  came from. **A CARLA frame should enter through the existing ingest path**, not a parallel
  one.

## The trap in this phase

**CARLA is an optional dependency and must stay optional.** The `carla` package is not
installed, is large, is version-locked to a simulator binary, and is unavailable on many
machines — including CI.

- Nothing outside `adaptx.carla` may import `carla`.
- The backend must start, all 17 endpoints must respond, and the full test suite must pass
  with the package absent. That is the current state and it is not negotiable.
- Tests for the real client belong behind a marker that skips cleanly when the import fails.
  The mock stays the default everywhere else.

## Provenance is the whole point

`DataSource` already distinguishes `live_sensor`, `simulation`, `replay`, `synthetic_test`
and `unavailable`. A CARLA frame is `simulation` — never `live_sensor`. This is the rule that
stops a demo screenshot becoming an accidental claim about real hardware, and it is asserted
by existing tests.

## Reproducibility

`knowledge-base/11_carla.md` requires the CARLA version, map, synchronous mode, fixed
timestep, sensor transforms and seeds to be documented. Record them where a result can find
them — a run that cannot be reproduced cannot be a measurement.

Synchronous mode with a fixed timestep matters more than it looks: **tracking and adaptive
resolution both depend on frame ordering and timestamps**. Velocity is measured from the
interval between frames, and the resolution dwell time counts frames. Free-running
asynchronous mode would make both non-deterministic.

## Ground truth is the prize — take it if it is cheap

CARLA can report actual actor positions, extents and velocities. If that is straightforward
to capture alongside the sensor frame, capture it: it is the raw material for the first real
accuracy measurement this project could make.

But keep it **strictly separate from the perception path**. Ground truth is for evaluation,
never an input. A detector that can see the answer measures nothing.

If it turns out not to be cheap, leave it for Phase 11 and say so.

## MUST NOT implement

- Scenario generation and replay (Phase 10) — CARLA is the environment, not the scenarios.
- Benchmarking against scenarios (Phase 11).
- The dashboard (Phase 12).
- Vehicle control, autopilot behaviour, planning or actuation of any kind.
- Any change to the perception algorithms of Phases 1–8. If real data exposes a defect,
  **report it** — a measured defect is a finding, and Phase 9 is the first chance to have one.
- New dependencies beyond `carla` itself, which is already declared as an optional extra.

## Backward-compatibility rules

1. Do not change the coordinate convention (ADR-009). CARLA's axes differ; convert at the
   boundary and document it. This is exactly the trigger ADR-013 named for a transform stage.
2. Do not modify Phase 1–8 algorithms unless a measured defect justifies it.
3. Do not change existing endpoint behaviour — extend additively.
4. Do not weaken or delete tests. If a premise genuinely changes, retarget it narrowly and
   report it, as Phases 3–8 each did.
5. Keep the honesty rules: no fabricated metrics; unmeasured values are `null` with a reason;
   `source` provenance is mandatory; simulation is labelled as simulation everywhere it
   appears; and nothing describes the risk score or the detail priority as a probability,
   calibrated or validated.
6. `IMPLEMENTED` stays reserved for mature functionality. A working CARLA client is still
   `PARTIAL`.

## Testing

- The client with `carla` absent: every method fails explicitly, and the backend still starts.
- The mock, unchanged, still satisfies the interface.
- Axis conversion, against hand-computed values.
- Frames ingested from CARLA are labelled `simulation` and reach the pipeline through the
  existing path.
- Determinism under synchronous mode with a fixed timestep and a fixed seed.
- The full existing suite, unchanged.

Record the setup and any measurement in `docs/experiments/experiment-log.md`.
