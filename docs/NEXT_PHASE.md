# Next Phase — Scenario Generation and Replay (Phase 10)

Handoff for the next work item. Read [`PROJECT_STATE.md`](PROJECT_STATE.md) first.

> **Numbering is settled.** Prediction is Phase 5, mapping 6, risk 7, adaptive resolution 8,
> CARLA 9 — all done. Scenario generation and replay is **Phase 10**. `CLAUDE.md`,
> `ROADMAP.md` and `services/system_service.py` agree. Do not renumber.

---

## There is a simulator, and exactly one hard-coded scene

Phase 9 made CARLA a usable data source: deterministic stepping, a converted coordinate
frame, simulation-authoritative time, and ground truth on a separate path. What it
deliberately did **not** build is any way to *describe* a scene.

The whole scenario today is three constants and a loop in `carla/smoke.py`:

```python
APPROACH_START_M = 45.0
APPROACH_SPEED_MPS = 8.0
APPROACH_LEFT_M = 3.5
```

One ego, one target, a straight line. It exists to prove the integration works and it says
so in its own docstring.

**Do not grow it into the framework.** Replace it. A scenario system that started as a
smoke test carries the smoke test's assumptions - one target, no traffic, scripted
transforms - into everything built on top.

## Objective

Two capabilities that share a contract:

**Scenario generation.** A seeded, declarative description of a scene that produces the same
simulation every time it is run: actors and their types, initial poses, motion, duration,
timestep, weather, and the map. Reproducible from the description alone, which is what
`knowledge-base/12_scenario-generation.md` means by repeatable.

**Event replay.** The record of what happened, and the ability to play it back.
`knowledge-base/14_event-replay.md` lists the events: object detected, track updated, risk
changed, prediction updated, predicted conflict created, resolution increased, resolution
decreased. Each with a timestamp, a type and its metadata.

Those two are related but not the same, and it is worth deciding early whether replay
re-runs the *simulation* or re-plays a *recording*. They have very different costs and very
different guarantees.

## What already exists and must be reused

- `CarlaSimulationSession` — the lifecycle. A scenario drives it; it does not need changing.
  `spawn_ahead_of_ego` and `place_ahead_of_ego` already take ADAPT-X coordinates.
- `GroundTruthFrame` — recorded per frame, joinable to the LiDAR frame by `frame_id`. This is
  most of what a replay record needs about the world.
- `CarlaSettings.seed` — declared and currently consumed by nothing. Phase 10 is what makes
  it real.
- `DataSource.REPLAY` — a provenance value that has existed since Phase 1 and has never been
  produced. Replayed frames are `replay`, not `simulation` and never `live_sensor`.

## The trap in this phase

**A scenario is configuration, not code.** The moment scenarios are Python functions, they
stop being reproducible from a description and start being reproducible from a git commit.
Seeds, actor lists and motion belong in data the runner reads.

Second: **determinism is a property to test, not to assume.** Phase 9 asserts that two runs
of the same scenario produce identical frames. Phase 10 must keep that true with more actors,
and physics or the traffic manager will break it if either is introduced without a seed. If
you enable the traffic manager, seed it and set it synchronous, and assert reproducibility.

## MUST NOT implement

- Benchmarking or evaluation suites (Phase 11) — including any comparison of perception
  against ground truth, however tempting once scenarios exist.
- The dashboard (Phase 12).
- Changes to Phase 1–9 algorithms unless a measured defect justifies it.
- ML, RL, GPU, collision avoidance or vehicle control.
- New dependencies without an ADR.

## Backward-compatibility rules

1. Do not change the coordinate convention (ADR-009) or the single-conversion rule (ADR-043).
2. Simulation time stays authoritative (ADR-044). A replay's timestamps come from the
   recording, never the wall clock.
3. Ground truth stays out of perception (ADR-045). Scenarios make it more tempting, not less.
4. CARLA stays optional (ADR-042): the backend and the default test run must work without it.
5. Do not weaken or delete tests. Retarget narrowly and report it, as Phases 3–9 each did.
6. Keep the honesty rules: no fabricated metrics, provenance mandatory, replayed data
   labelled `replay`, and nothing described as validated that has not been.

## Testing

- A scenario produces identical frames across two runs from the same seed.
- A scenario is reconstructible from its description alone.
- Replayed frames are labelled `replay` and reach the pipeline through the existing path.
- Events carry timestamp, type and metadata, and are ordered.
- The whole suite still runs without CARLA installed.

Record the scenario format and any measurement in `docs/experiments/experiment-log.md`.

## One piece of unfinished business from Phase 9

The live CARLA smoke test has **never been executed** — the package is not installed in this
environment, so `pytest -m carla` reports 6 skipped. Running it is cheap and worth doing
before building on the adapter:

```bash
pip install -e .[carla]
./CarlaUE4.sh -RenderOffScreen
pytest -m carla
python -m adaptx.carla.smoke --frames 20 --json runs/live.json
```

If the real API disagrees with the adapter anywhere, that is a Phase 9 defect and should be
fixed as one rather than worked around in Phase 10.
