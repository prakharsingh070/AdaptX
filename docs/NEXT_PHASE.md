# Next Work — After Phase 12

Handoff for the next work item. Read [`PROJECT_STATE.md`](PROJECT_STATE.md) first.

> **Numbering is settled.** Prediction is Phase 5, mapping 6, risk 7, adaptive resolution 8,
> CARLA 9, scenarios 10, evaluation 11, dashboard 12 — all done as baselines.
> `CLAUDE.md`, `ROADMAP.md` and `services/system_service.py` agree. Do not renumber.

---

## Where things stand

Twelve phases in, every numbered item on the roadmap exists: a deterministic pipeline
behind typed contracts, CARLA feeding it, a scenario framework driving controlled scenes,
an offline evaluation layer that measured the pipeline against ground truth and found the
baselines wanting (Experiments 011–012), a dashboard that shows all of it without
computing any of it (Phase 12, ADR-055, [`DASHBOARD.md`](DASHBOARD.md)), and - as an
extension of Phase 12, not a new phase - a live loop in which a baseline governor drives
the ego from the pipeline's own outputs on a running CARLA server (ADR-056, Experiment
014: it stops for the parked car, resumes when it leaves, 0 collisions, 0.3x wall-clock).

**The roadmap defines no Phase 13.** This file therefore does not invent one. What follows
is the work already on record, in the order the evidence suggests, and the rules that
apply to any of it.

## Open items, on record

1. **The ground-segmentation default (owed since Experiment 012).** `ADAPTX_LIDAR__GROUND_ENABLED`
   is `false` by default (ADR-012). With it off, a car parked 20 m ahead and a pedestrian
   15 m ahead are detected on zero frames; with it on, both are seen every frame, no
   identity switch occurs, and the pipeline runs at 80 ms per frame instead of 140. This
   is a Phase 2/3 decision that changes every endpoint's behaviour. Make it with its own
   experiment entry stating before and after, then re-run the evaluation and load the new
   reports into the dashboard — which needs no change for that.
2. **The deferred "B" items** under each phase in [`ROADMAP.md`](ROADMAP.md): clustering
   and coordinate transforms (2D), oriented boxes and a labelled dataset (3B), a motion
   model and re-identification (4B), and so on. Each is independent and each is a phase
   of its own with an ADR and an experiment. The evaluation layer now exists to say
   whether any of them helped, which was not true before Phase 11.
3. **Event replay** (deferred from Phase 10, deferred again in Phase 12 because no panel
   needed it). The design question is still open: re-run the simulation, or re-play a
   recording. Phase 12's run playback is *not* event replay — it steps through a stored
   record's outputs and produces nothing marked `DataSource.REPLAY`.
4. **What the dashboard shows as missing, honestly**: a spatial risk field (`risk_field`
   in `not_yet_available`), routing, planning, GPU measurement, per-cell occupancy
   streaming, live scene history. None is required by anything now; each is a feature
   with a backend contract to design first and a panel to fill second.
5. **What the live loop exposed (Experiment 014).** Velocities are ego-relative: with the
   ego moving, every static object "approaches" at the ego's speed and the risk engine's
   closing-speed factor rises everywhere. Ego-motion compensation belongs in Phase 4B
   (tracking) with the ego odometry as an input - and it changes every evaluation figure,
   so it needs its own experiment. The governor's corridor is straight along +X; on a
   bend, roadside geometry stops the ego. A road-following corridor (map waypoints ahead)
   is a controller change, not a perception change, and stays a baseline.
6. **The live loop's speed.** Now ~85 ms of a 120 ms frame is the pipeline (Experiment
   015); the next measured hotspot is the adaptive mapper building a `MapTile` model per
   tile per frame. Profiling before optimising; no figure changes until an experiment
   says so.
7. **Re-run the Phase 11 evaluation under the ADR-057 defaults.** The elevated filter,
   class decay, vehicle band and tentative-miss change alter detection and tracking, so
   every report in Experiments 011/012 is now describing an older pipeline. A new
   experiment entry with before/after per scenario is owed before any of those figures
   is quoted again.
8. **Cyclist classification.** A riderless CARLA bicycle clusters as 1.0-1.9 x 1.2 x 1.1 m
   and fits no band; attach a walker to the bicycle (a rider) in the scenario, or accept
   that the geometric baseline cannot name it. Either way, measure first.

## The trap after this phase

**The console now makes the baselines look finished.** A moving box with a predicted path
and a risk colour reads as a working perception stack, and Experiments 011–012 say it is
not: recall 0.00–0.06 at a 1 m gate, a constant 1.7 m centroid offset, no vehicle ever
classified as one. The Evaluation view puts those figures and the report's `limitations`
on screen for exactly this reason. Any demo that hides that view is misrepresenting the
project.

## MUST NOT implement

- Anything computed in the browser: risk, trajectory, resolution, match, distance, rate,
  ratio or metric. If a panel wants a number, the backend or the evaluation layer produces
  it under a contract (ADR-055). The source scan in `tests/integration/test_dashboard_api.py`
  enforces it; do not widen its allowlist.
- A dashboard endpoint that runs perception, an evaluation or a benchmark on request, or
  any endpoint that spawns, destroys, moves, ticks or drives a simulator actor. The five
  session controls (`/api/v1/live/start|pause|resume|stop|reset`) are the whole control
  surface and are allowlisted by full path; the route audit enforces it.
- A controller that reads CARLA ground truth about other actors, for any reason. The
  loop never calls `session.ground_truth()`; keep it that way.
- Calling the governor "autonomous driving", "safe" or "collision-free". It is a baseline
  and the safety sensor counts what it fails to prevent.
- Ground truth on the live path. `SceneSnapshot` has no field for it and refuses one; keep
  it that way.
- Tuning of Phases 2–8 against Experiment 011/012 figures without an experiment entry
  stating before and after.
- New dependencies — backend or frontend — without an ADR. The dashboard has zero and no
  build step; keep both.

## Backward-compatibility rules

1. Existing endpoint behaviour and response shapes — extend additively.
2. Ground truth stays out of perception (ADR-045); evaluation stays offline (ADR-050);
   stored reports are served, never recomputed (ADR-055); a metric that could not be
   computed stays null with a reason (ADR-051).
3. `ScenarioRunResult` stays free of metrics; `EvaluationReport` stays simulation evidence
   with its limitations; both keep loading in the dashboard unchanged.
4. Do not weaken or delete tests. Retarget narrowly and report it, as Phases 3–12 each did.
5. Report only measured results, with the method, the seed, the scenario and the
   environment beside every number.
6. The live and stored modes stay labelled and never mixed; UNKNOWN stays UNKNOWN; null is
   "Not available"; unobserved is never shown as free; "collision probability", "safe",
   "production ready" and "real-time" never appear in the UI.

## Testing

- The backend suite (1684 tests) and the Node suite (20) must keep passing; `pytest -m
  carla` (12, including the live loop, the obstacle-stop demo and the perception
  acceptance cases) when a server is up.
- The live loop is tested against the fake simulator (kinematics, attached sensors,
  Traffic Manager, collision, camera) in `tests/integration/test_live_simulation.py`;
  a change to the loop needs a fake-based test first and a live run second.
- Any new panel is fed by a real endpoint or a stored report and has a test that says so.
- Any new endpoint is additive, documented in `API.md`, and passes the route audit.
