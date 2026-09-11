# Next Phase — Dashboard and Final Integration (Phase 12)

Handoff for the next work item. Read [`PROJECT_STATE.md`](PROJECT_STATE.md) first.

> **Numbering is settled.** Prediction is Phase 5, mapping 6, risk 7, adaptive resolution 8,
> CARLA 9, scenarios 10, evaluation 11 — all done. The dashboard is **Phase 12**.
> `CLAUDE.md`, `ROADMAP.md` and `services/system_service.py` agree. Do not renumber.

---

## Where things stand

Eleven phases in, the backend is complete as a set of deterministic baselines behind
typed contracts, CARLA feeds it, a scenario framework drives controlled scenes, and — as
of Phase 11 — there is a measured account of what the pipeline actually does with them
(Experiment 011). That account is mostly unflattering, and it is the most valuable thing the
dashboard can show, because a dashboard that only shows the pipeline working would be
showing the one thing the evaluation says it does not reliably do.

## Objective

Build the dashboard against the existing backend contracts, exactly as
[`UI_UX.md`](UI_UX.md) specifies, with perception logic kept out of the UI. Every panel
must be fed by a real endpoint or telemetry message, and every number it shows must be one
the backend measured. The visual target is the mockup; the numbers are never the mockup's.

## What already exists and must be reused

- **Seventeen endpoints and one WebSocket**, unchanged since Phase 8 — see
  [`API.md`](API.md). Every stage returns a full result object with provenance,
  `is_baseline`, configuration snapshot and measured duration.
- `GET /api/v1/system/status` — readiness *and* implementation status per component, so a
  panel for something that does not exist can say so instead of looking empty.
- `MappingComparison` — the fixed-versus-adaptive contract, per frame.
- `EvaluationReport` (Phase 11) — JSON written by `python -m adaptx.evaluation evaluate`.
  **Evaluation is offline by decision (ADR-050).** If the dashboard shows evaluation
  figures, it loads a recorded report; it never recomputes one, and it never asks the
  backend to. Adding an endpoint that serves a *stored* report is additive and acceptable;
  adding one that runs an evaluation is not.
- The `SIMULATION` / `SYNTHETIC_TEST` / `LIVE_SENSOR` provenance on every contract, which
  the UI must surface, not flatten.

## One decision owed before more evaluation runs are read

**Ground segmentation is off by default** (`ADAPTX_LIDAR__GROUND_ENABLED=false`, ADR-012).
Experiment 012 measured the consequence on the live server: with it off, a car parked 20 m
ahead and a pedestrian 15 m ahead are detected on zero frames because their returns are
clustered with the road; with it on, both are detected on every frame, no scenario shows an
identity switch, and the pipeline runs at 80 ms per frame instead of 140. Changing a process
default is a Phase 2/3 decision that affects every endpoint; make it with an experiment
entry, not as a side effect of the dashboard. Until it is made, every evaluation report
must state the configuration it ran under - it does.

Also known: scenes with moving placed actors are near- but not bit-repeatable on CARLA
0.9.16 (≤ 8 of 27,000 points differ on some frames; a static scene is exact). The cause is
not established and `python -m adaptx.evaluation compare` reports it correctly.

## The trap in this phase

**A dashboard is a claim.** A risk gauge reads as a probability. A "detection accuracy"
card reads as a validated number. A green status reads as production readiness. None of
those are true, and the contracts already carry the words that say so (`is_baseline`,
`source`, `MetricStatus`, `limitations`). Show them. The Phase 11 report renders its
limitations after its conclusion for exactly this reason; the dashboard should do the same.

## MUST NOT implement

- Perception, tracking, prediction, mapping, risk or resolution logic in the UI, or any
  recomputation of a backend figure in the browser.
- An endpoint that runs an evaluation, a scenario or a benchmark on request from the UI.
- Any tuning of Phases 2–8 in response to Experiment 011 without its own experiment entry
  stating the before and after.
- Event replay, unless a panel genuinely needs it — the design question from Phase 9 is
  still open.
- New dependencies on the backend without an ADR.

## Backward-compatibility rules

1. Existing endpoint behaviour and response shapes — extend additively.
2. Ground truth stays out of perception (ADR-045); evaluation stays offline (ADR-050); a
   metric that could not be computed stays null with a reason (ADR-051).
3. `ScenarioRunResult` stays free of metrics; `EvaluationReport` stays simulation evidence
   with its limitations.
4. Do not weaken or delete tests. Retarget narrowly and report it, as Phases 3–11 each did.
5. Report only measured results, with the method, the seed, the scenario and the
   environment beside every number.

## Testing

- The UI is tested against recorded responses and a recorded evaluation report, never
  against a live simulator.
- A panel for a `PLANNED` or `UNAVAILABLE` component says so; a test asserts it.
- Provenance is rendered; a test asserts a `SIMULATION` frame is never shown as live.
- The backend suite (1565 tests) continues to pass without the dashboard present.
