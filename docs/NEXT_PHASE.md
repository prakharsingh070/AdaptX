# Next Phase — Benchmarking and Evaluation (Phase 11)

Handoff for the next work item. Read [`PROJECT_STATE.md`](PROJECT_STATE.md) first.

> **Numbering is settled.** Prediction is Phase 5, mapping 6, risk 7, adaptive resolution 8,
> CARLA 9, scenarios 10 — all done. Benchmarking and evaluation is **Phase 11**. `CLAUDE.md`,
> `ROADMAP.md` and `services/system_service.py` agree. Do not renumber.

---

## Everything needed to measure correctness now exists, and has never been used for it

Ten phases in, no accuracy figure has ever been computed. That was deliberate: each phase
recorded that its correctness was "unmeasured and unmeasurable" because **no labelled data
existed**. Phase 9 produced ground truth. Phase 10 produced controlled, reproducible scenes
and recorded, per frame, three things side by side:

| What | Contract | Source |
|---|---|---|
| What the scenario **commanded** | `ExpectedPose` on `ScenarioFrameRecord` | exact, closed form |
| What the simulator **reported** | `GroundTruthFrame` | exact, from CARLA |
| What the pipeline **counted** | `StageCounts` | the Phase 2–8 output |

Phase 11 is the first phase allowed to put a number between them. Nothing before it did,
and a test in the scenario framework asserts that its result contracts carry no accuracy,
precision, recall, error or match field. That test should stay: evaluation belongs in an
evaluation module, not on a run record.

## Objective

Two things `CLAUDE.md` has asked for since Phase 1, now possible:

**The baseline comparison.** Run identical scenarios through fixed-resolution and adaptive
perception and compare the measured workload — cells, memory, latency — *and* whatever
correctness can now be measured. Experiment 007 did the workload half over synthetic scenes
with hand-specified risk; Phase 11 does it over scenario runs with real ground truth.

**Correctness against ground truth.** Detection against `GroundTruthFrame.others()`,
tracking continuity against stable actor ids, prediction against where the actor actually
went on later frames. Each of these has been called unmeasurable for nine phases; each is now
measurable, and each is likely to produce a humbling number, because every stage is a
baseline that was built to be replaced.

## Do this first, before writing any evaluation code

**Run the live CARLA tests.** Not one line of the scenario framework, the CARLA adapter or the
ground-truth path has executed against a real server. Everything has run against
`tests/fixtures/fake_carla.py`, a stand-in that places actors exactly where told and does no
physics. A real server settles a spawned vehicle onto the road, may reject a placement, and
may not have every blueprint the catalogue asks for.

```bash
pip install -e .[carla]
./CarlaUE4.sh -RenderOffScreen
pytest -m carla
python -m adaptx.scenarios run vehicle_approach --json runs/approach.json
```

Evaluating perception against ground truth that has only ever come from the stand-in would
measure the stand-in. If the real API disagrees with the adapter, or a catalogue scenario
fails on a real map, those are Phase 9 and Phase 10 defects and should be fixed as such
before Phase 11 builds on them.

## What already exists and must be reused

- `ScenarioRunResult` — the raw evidence. Serialisable; a run written with `--json` can be
  evaluated offline without a simulator. Evaluate from results, not from live runs.
- `run_scenario(definition, process=False)` — records frames and ground truth without the
  pipeline, for when the evaluation needs the sensor frames themselves rather than counts.
- `adaptx.benchmark` — the measurement conventions: warm-ups, repeats, medians, `tracemalloc`
  in a separate run, environment recorded, synthetic labelled as synthetic.
- `MappingComparison` — the fixed-versus-adaptive workload contract from Phase 8.
- Every stage's `duration_ms` and configuration snapshot, already on every result.

## The trap in this phase

**A flattering metric is easy to build and impossible to un-publish.** Detection "accuracy"
depends entirely on the association rule — how close a detection must be to a ground-truth
actor to count — and that rule is a choice. State it, make it configurable, report the
result at more than one threshold, and never report a single headline number without the
rule beside it.

Second: **the baselines will lose.** Constant-velocity prediction is wrong through turns;
centroid association swaps identities in close passes; the detail priority has never been
tuned. Phase 11's job is to measure that, not to explain it away. A negative result,
measured properly, is the point of having built the baselines.

## MUST NOT implement

- The dashboard (Phase 12).
- Changes to Phase 1–10 algorithms in order to improve a metric. Measure first; tuning is a
  separate, later decision with its own experiment entry.
- ML, RL, GPU, collision avoidance or vehicle control.
- Event replay, unless the evaluation genuinely needs it — it was deferred from Phase 10
  with its design question open.
- New dependencies without an ADR. `pandas` and `scipy` will look tempting; NumPy has done
  everything so far.

## Backward-compatibility rules

1. Ground truth stays out of perception (ADR-045). Evaluation *reads* it; nothing upstream
   ever does.
2. `ScenarioRunResult` stays free of metrics. Evaluation produces its own result contract.
3. Simulation time stays authoritative (ADR-044); synthetic and simulated data stay labelled.
4. The fixed-resolution mapper stays available and unchanged (ADR-003).
5. Do not weaken or delete tests. Retarget narrowly and report it, as Phases 3–10 each did.
6. Report only measured results, with the method, the seed, the scenario and the environment
   beside every number.

## Testing

- Every metric is asserted on a hand-built case where the answer is known exactly.
- A perfect match scores perfectly; an empty pipeline output scores zero; both are tested.
- The association threshold is a parameter, and results at two thresholds are shown to
  differ.
- Evaluation runs from a saved `ScenarioRunResult` with no simulator present.
- The whole suite still runs without CARLA installed.

Record every measurement in `docs/experiments/experiment-log.md` with its limitations.
