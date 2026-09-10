# Next Phase — 2.5D Mapping (Phase 6)

Handoff for the next work item. Read [`PROJECT_STATE.md`](PROJECT_STATE.md) first.

> **Numbering is settled.** Trajectory prediction is **Phase 5** (done). 2.5D mapping is
> **Phase 6**, risk and uncertainty **Phase 7**, adaptive resolution **Phase 8**.
> `CLAUDE.md`, `ROADMAP.md` and `services/system_service.py` all agree. Do not renumber
> again.

---

## Objective

Implement `mapping.interfaces.AdaptiveMapper` in **two** variants, producing the
`AdaptiveMap` / `AdaptiveMapCell` contracts that already exist and are currently unused:

1. a **fixed-resolution baseline** — uniform cell size everywhere (ADR-003, ADR-018);
2. the **adaptive mapper** — variable cell size, distinguished by `AdaptiveMap.is_adaptive`.

Both must exist from the start. The whole ADAPT-X claim is a comparison, and a baseline
added afterwards is a baseline shaped to flatter the result.

**Phase 6 builds the map. It does not decide resolution.** The resolution *policy* is
Phase 8 (`ResolutionController`), and the risk signal that policy will consume is Phase 7.
Phase 6 must therefore accept a resolution decision from outside rather than invent one —
see "The awkward ordering" below.

## Inputs available

| Source | What it gives you |
|---|---|
| `PointCloudProcessingResult.frame` | Non-ground points, filtered and validated |
| `PointCloudProcessingResult.ground_frame` | Separated ground points, when 2B ground is enabled |
| `DetectionResult.objects` | `DetectedObject` with AABB, class, fit score |
| `TrackingResult.tracks` | `TrackedObject` with measured velocity (or `None`) |
| `PredictionResult.trajectories` | `PredictedTrajectory` with extrapolated positions, heuristic `position_uncertainty_m` and evidence-based `confidence` |

Read the prediction caveats in [`PROJECT_STATE.md`](PROJECT_STATE.md) §15 before using
trajectories to drive anything. They are constant-velocity extrapolations.

## Expected outputs

The **existing** contracts in `models/map.py` — do not create parallel ones:

- `AdaptiveMapCell`: occupancy, height, resolution level, risk, uncertainty
- `AdaptiveMap`: the snapshot, with `is_adaptive` distinguishing the two variants
- `ResolutionLevel` / `OccupancyState` enums, and `ResolutionContext`

Following ADR-022 / ADR-024 / ADR-027, return a **result object** carrying the map, what
was excluded and why, measured durations, and a configuration snapshot. Check whether
`AdaptiveMap` already suffices as that wrapper before adding one.

## The awkward ordering — read this before designing

Phase 6 comes *before* risk (7) and the resolution controller (8), but "adaptive" means
"resolution driven by risk". Resolve this explicitly rather than by accident:

- The adaptive mapper must take its per-region resolution as an **input**, from something
  implementing the `ResolutionContext` shape, not compute it internally. That keeps
  Phase 8's controller a drop-in and stops resolution policy leaking into the mapper.
- For Phase 6, supply that input from a **deliberately trivial, clearly labelled** stand-in
  (for example, distance-banded resolution). Name it for what it is. Do **not** call it
  risk-aware, and do **not** let it grow into the risk engine.
- If a trivial stand-in cannot be built without prejudging Phase 7, say so and propose
  swapping the phase order rather than quietly inventing a risk model here.

## Honesty requirements specific to this phase

- **A cell must distinguish "empty" from "unobserved".** LiDAR produces occlusion shadows;
  a cell behind a vehicle is not known to be free. If `OccupancyState` cannot express that
  distinction, extend it — reporting unobserved space as free is the most dangerous
  fabrication available in this phase.
- **Cell height is a measurement, not an estimate.** A cell with no points has no height:
  `None`, not `0.0`, following ADR-023's reasoning exactly.
- `risk` and `uncertainty` fields on a cell must stay `None`/unset until Phase 7 computes
  them. Do not fill them with placeholders.
- The fixed baseline must be genuinely comparable: same input, same accounting, same
  measurement method. ADR-018 exists because a baseline that is not comparable is worse
  than none.

## Configuration

`MapSettings` already exists (`ADAPTX_MAP__`) with `range_m` and a cell size per resolution
level. Extend it rather than adding a new section, and document any new field in
`.env.example`.

> Note: `.env.example` is missing `DETECTION` and `TRACKING` sections entirely — a gap from
> Phases 3 and 4. Filling those in is a small, welcome side task, but keep it in its own
> commit.

## API

Extend the existing architecture; do not break anything.

- `GET /api/v1/map/status` already exists and returns `MapStatusResponse` with
  `resolution_levels`, `range_m` and `active_cells`. Update it; do not replace it.
- A frame-level endpoint (`POST /api/v1/lidar/map`, matching `/detect`, `/track`,
  `/predict`) is the obvious addition. Decide whether it is stateful — a map that
  accumulates across frames is, and that decision needs an ADR either way.
- **Never return raw point arrays.** A full map may be large; consider whether the response
  needs the cells themselves or a summary plus a separate cell query.

## Telemetry

Add a map **summary** — cell counts by resolution level and occupancy state, configuration,
measured duration. Remove `"adaptive_map"` from `_NOT_YET_AVAILABLE` in
`websocket/telemetry.py` **only once it genuinely exists**. Do not stream cells on every
tick; that is frame data on a status channel.

Update the `mapping` component in `services/system_service.py` to `READY` / `PARTIAL` with a
detail naming what it cannot do. **Never `IMPLEMENTED`.**

## Testing

Deterministic scenes with known geometry, following `tests/fixtures/scenes.py`:

- a known flat plane produces the expected cell count and heights — exact arithmetic
- a cell containing no points reports **no** height, not zero
- occluded space is reported unobserved, **not** free
- fixed and adaptive variants over the same input, with `is_adaptive` set correctly
- cell size actually changes with the supplied resolution input, and the total cell count
  moves in the expected direction
- map bounds and `range_m` edges; a point exactly on a boundary lands in exactly one cell
- empty frame, single point, all-ground frame
- determinism: same input, same map
- no NaN/Inf anywhere in outputs
- integration: raw frame → process → detect → track → predict → map
- API contract and OpenAPI documentation

Assert *properties*, not brittle values, where an algorithm allows numerical variation.

Add a mapping benchmark (`--map`) mirroring `--detect` / `--track` / `--predict`, measuring
**fixed against adaptive** on identical input — that comparison is the first real evidence
for the project's central claim. Record it as Experiment 005. Measure cell counts and
memory as well as time; the adaptive claim is about *workload*, not only speed.

## MUST NOT implement

- The risk engine, time-to-collision, trajectory overlap or risk scoring (Phase 7)
- The resolution controller or its stabilisation policy (Phase 8)
- Learned mapping of any kind
- CARLA scenarios or the dashboard
- Any new dependency without an ADR

## Backward-compatibility rules

1. Do not change the coordinate convention (ADR-009).
2. Do not modify Phase 1–5 algorithms unless a measured defect justifies it.
3. Do not change existing endpoint behaviour — extend additively.
4. Do not weaken or delete tests. If a premise genuinely changes (as happened in Phases 3,
   4 and 5 when a module stopped being "planned"), retarget the test to guard the same
   property and report it.
5. If an existing file must change: explain why, make the smallest change, preserve
   compatibility, add a regression test, and list it in the final report.
6. Reuse `AdaptiveMap` / `AdaptiveMapCell` / `ResolutionContext`; do not create duplicate
   contracts.
7. Keep the honesty rules: unmeasured values are `null` with a reason, `source` provenance
   is mandatory, baselines are labelled `is_baseline`, and **no fabricated accuracy or
   performance figures**.
