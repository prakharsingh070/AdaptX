# Next Phase — Adaptive Resolution (Phase 8)

Handoff for the next work item. Read [`PROJECT_STATE.md`](PROJECT_STATE.md) first.

> **Numbering is settled.** Prediction is Phase 5 (done), 2.5D mapping Phase 6 (done), risk
> and uncertainty Phase 7 (done), adaptive resolution **Phase 8**. `CLAUDE.md`,
> `ROADMAP.md` and `services/system_service.py` all agree. Do not renumber.

---

## Three questions, three phases

The separation matters more than any individual implementation choice. Each phase answers
exactly one question and must not answer another's:

| Phase | Question | Status |
|---|---|---|
| **6** Mapping | *How do we represent space at a **fixed** resolution?* | Done |
| **7** Risk | *How **concerning** is this object?* | Done |
| **8** Adaptive resolution | *How much **spatial detail** should this region receive?* | **This phase** |

Phase 7 deliberately cannot answer Phase 8's question: it is never handed a cell size, and
`RiskAssessment` carries no resolution field (ADR-036). Phase 6 deliberately cannot answer
it either: the mapper applies a resolution it is given and never chooses one (ADR-029).

Phase 8 is the missing middle, and it is the phase this project exists for.

## This is the phase the project exists for

Every phase so far has built a piece. Phase 8 connects the last two and **tests the central
claim**: that allocating spatial resolution by risk and uncertainty beats allocating it
uniformly.

Phase 6 built the fixed-resolution mapper and the `ResolutionDecision` contract it applies.
Phase 7 produced per-object risk and uncertainty. Neither knows about the other. Phase 8 is
the controller between them:

```
RiskAssessment[]  ->  ResolutionContext  ->  [ResolutionController]  ->  ResolutionDecision
                                                                              |
                                                                              v
                                                                    FixedResolutionMapper
                                                                    (unchanged, ADR-029)
```

**Neither the mapper nor the risk engine should need to change.** If either does, the
boundary drawn in ADR-029 and ADR-036 was wrong and that is worth knowing early — say so
rather than quietly editing them.

## Objective

Implement `mapping.interfaces.ResolutionController`:

- `select_resolution(context: ResolutionContext) -> ResolutionLevel`
- `cell_size_m(level: ResolutionLevel) -> float`

Both already exist as abstract methods with **no implementation**. `ResolutionContext` also
already exists (`models/map.py`) carrying exactly the inputs a controller needs:
`distance_from_ego_m`, `risk_score`, `predicted_risk_score`, `uncertainty`, `object_density`,
`max_object_speed_mps`, `in_ego_path`, `current_level`.

Populating that contract from Phase 7 `RiskAssessment` values is most of the work.

## What Phase 8 must eventually provide

Not all of it has to land in one commit, but the design should leave room for all of it:

- a `ResolutionController` implementation and the `ResolutionDecision`s it produces
- an adaptive map representation genuinely carrying **more than one** cell size
- region- or tile-based adaptation, at a documented granularity
- influence from **risk**, **uncertainty**, predicted **trajectories**, and object presence
- object density where it is genuinely informative rather than decorative
- deterministic decisions with **explicit tie-breaking**
- hysteresis or an equivalent anti-oscillation mechanism
- bounded memory and a bounded region/cell count, checked **before** allocation
- a fixed-versus-adaptive comparison over identical input
- API, telemetry, benchmark and tests, following the shapes Phases 6 and 7 already use

## Inputs available

| From | Field | Note |
|---|---|---|
| `RiskAssessment` | `risk_score` | **`None` when `risk_level` is `UNKNOWN`** — handle it, do not coerce to 0.0 |
| `RiskAssessment` | `uncertainty.score` + `.reasons` | Heuristic, with contributors visible |
| `RiskAssessment` | `distance_m`, `closing_speed_mps` | Closing speed is `None` when velocity was never measured |
| `RiskAssessment` | `trajectory.min_distance_m`, `.time_to_min_distance_s` | `None` without a prediction |
| `RiskAssessment` | `map_context.observation` | `OBSERVED_EMPTY` means **unobserved**, not free |
| `SpatialMap` | bounds, resolution, occupancy | The map being refined |
| `MapSettings` | `resolution_low/medium/high/critical_m` | The existing level vocabulary |

## The three traps in this phase

**1. `uncertainty` is not a second risk score.** Phase 7 deliberately kept them separate
(ADR-033) so this phase can use both. A region can be low-risk and badly observed, and that
is a strong argument for *more* detail, not less. If the controller only reads `risk_score`,
the separation was pointless.

**2. `risk_score` is `None` for `UNKNOWN` assessments.** A lost track, or one where nothing
could be computed. Coercing it to `0.0` would allocate the coarsest resolution to the objects
the system understands least — the same inversion ADR-032 exists to prevent, one phase later.

**3. Resolution must not oscillate.** `knowledge-base/10_adaptive-resolution.md` requires a
documented stabilisation mechanism, and `ResolutionContext.current_level` exists for exactly
that. Hysteresis, smoothing or a minimum dwell time — pick one, document it, and **test that
a region on a threshold boundary does not flip every frame**. An oscillating map is worse
than a uniform one: it costs more and produces unstable output.

## The twelve rules

Numbered so a review can cite them. The first six are correctness; the rest are honesty and
scope. Rules 1, 2 and 5 each undo a specific, deliberate decision from an earlier phase if
broken — they are not style preferences.

1. **`risk_score is None` must not become `0.0`.** `UNKNOWN` is not `LOW`. Coercing it
   allocates the coarsest detail to the objects the system understands least — the exact
   inversion ADR-032 exists to prevent.
2. **Uncertainty is not another risk score.** Do not simply add it to risk. Phase 7 kept them
   separate (ADR-033) *so this phase can use both*. A low-risk, badly observed region is a
   strong argument for more detail.
3. **Phase 7 must not decide resolution.** If the controller needs something Phase 7 does not
   expose, add it to `ResolutionContext` — do not push a resolution decision back into the
   risk engine (ADR-036).
4. **`FixedResolutionMapper` must remain intact.** It is the baseline the comparison rests
   on. If it must change, the boundary in ADR-029 was wrong; say so explicitly rather than
   editing quietly.
5. **Unobserved space is not free space.** An empty cell may be empty or occluded, and the
   map cannot tell (ADR-031, ADR-034). Never coarsen a region because nothing was observed
   there — that reduces detail exactly where the sensor saw least.
6. **Resolution must not oscillate** around a threshold. Document the stabilisation mechanism
   and test a region sitting exactly on a boundary across several frames.
7. **Adaptive mapping must be bounded.** Compute region and cell counts *before* allocating,
   and reject a configuration that would exceed the limit — as `grid_shape` already does
   (ADR-028).
8. **Decisions must be deterministic**, including tie-breaking. Same assessments, same map,
   same configuration → byte-identical decisions.
9. **No collision-probability claims.** No calibrated probability model exists anywhere in
   this project.
10. **No safety-certification claims.** Thresholds are baseline engineering values.
11. **No real-time claims** without measurement, and none from a synthetic benchmark at all.
12. **No ML, GPU/CUDA, CARLA or dashboard work** in Phase 8.

## Expected outputs

Reuse the existing contracts. Do **not** create parallel ones:

- `ResolutionLevel` (LOW/MEDIUM/HIGH/CRITICAL) and `MapSettings.resolution_*_m`
- `ResolutionDecision` with `source = ResolutionSource.ADAPTIVE` — **reserved in Phase 6
  specifically for this** and never yet produced
- `AdaptiveMap.is_adaptive = True` on maps built from an adaptive decision

Following ADR-022/024/027/029/032, return a result object carrying the decisions, what was
excluded and why, measured durations, and a configuration snapshot.

## The hard part: one map, many resolutions

Phase 6's mapper applies **one uniform cell size** per call. A genuinely adaptive map needs
different cell sizes in different regions, and that is a real design decision this handoff
deliberately does not make for you. Options, none free:

- **Multiple passes.** Run the mapper at several resolutions and compose. Simple, reuses
  Phase 6 unchanged, but costs several full grids.
- **A region-partitioned map.** Tile the extent and assign each tile a level. Needs a new map
  representation and a decision about tile granularity.
- **A hierarchical / quadtree-like grid.** Coarse base with refined sub-blocks. Most
  efficient, most complex, and hardest to serialise for the dashboard.
- **Multi-layer grids.** One full grid per resolution level, with a rule for which layer owns
  a region. Conceptually simple and easy to compare against the baseline, but memory scales
  with the number of levels.

**This decision is deliberately left open.** It is the one genuine architectural question
Phase 8 must answer, and answering it here without reading the current code would be
guessing. Inspect `models/spatial_map.py`, `mapping/grid_mapper.py` and how `AdaptiveMap`
is consumed, then pick one.

Whichever you choose, **write it up as ADR-037 before implementing**, with context,
decision, alternatives, consequences and risks — the shape every ADR in this project uses.
Keep `AdaptiveMap.is_adaptive` meaningful so fixed and adaptive results can never be
confused (ADR-003).

## Measurement — this is the deliverable

Experiment 007 must run **`FixedResolutionMapper` versus the adaptive mapper over identical
deterministic input** and report, at minimum:

| Metric | Why |
|---|---|
| mapping latency | the cost of building the map |
| controller latency | the cost of *deciding*, separated from building |
| total latency | what a caller actually pays |
| total cells | the workload headline |
| occupied cells | how much of that workload held data |
| resolution distribution | cells at each level |
| finest / coarsest / average resolution | whether adaptation genuinely varied |
| peak memory | the other half of the workload claim |
| number of resolution changes between frames | stability, and evidence rule 6 holds |
| **cells spent on high-risk vs low-risk regions** | the number the whole claim rests on |
| what the adaptive map loses | where it is coarser than the baseline, and what detail that costs |

Label it exactly as every other experiment is labelled:

> Synthetic benchmark. Not a real-world autonomous-driving performance claim.

Phase 6 measured the problem: occupancy falls to 1–16% at 0.25 m, so a uniform fine map
spends most of its cells recording that nothing was observed (Experiment 005). **That is the
figure to beat.** Report it honestly if the adaptive mapper does not beat it — a negative
result, measured properly, is a real finding and far more valuable than a flattering one.

## API, telemetry, status

- Extend `GET /api/v1/map/status` — `adaptive_resolution_implemented` is currently hardcoded
  `false` in `routes/map.py`; it becomes real.
- A frame-level endpoint (`POST /api/v1/lidar/adaptive-map`) matching the existing shape.
- Remove `"adaptive_map"` from `_NOT_YET_AVAILABLE` **only once it genuinely exists**.
- Update the `mapping` component detail: it currently says
  `ADAPTIVE RESOLUTION IS NOT IMPLEMENTED` in capitals. That sentence is the thing you are
  deleting — make sure it is actually true first.
- **Never `IMPLEMENTED`.** This is still a baseline.

## Testing

- a high-risk region receives finer cells than a low-risk one, on identical geometry
- a **high-uncertainty, low-risk** region also receives finer cells — the ADR-033 payoff
- an `UNKNOWN` assessment (`risk_score is None`) does **not** receive the coarsest level
- a region sitting exactly on a threshold does not oscillate across frames
- levels stay within the configured vocabulary and cell sizes
- fixed and adaptive over the same input, with `is_adaptive` set correctly
- empty scene, single object, objects at map bounds
- determinism: same assessments, same decisions
- integration: raw frame → … → risk → resolution → adaptive map
- API contract and OpenAPI documentation

Add an adaptive-mapping benchmark and record Experiment 007.

## MUST NOT implement

- Learned or ML resolution policy
- CARLA scenarios (Phase 9), scenario generation (Phase 10) or the dashboard (Phase 12)
- Collision avoidance, vehicle control, or anything actuating
- Any new dependency without an ADR

## Backward-compatibility rules

1. Do not change the coordinate convention (ADR-009).
2. Do not modify Phase 1–7 algorithms unless a measured defect justifies it.
3. Do not change existing endpoint behaviour — extend additively.
4. Do not weaken or delete tests. If a premise genuinely changes (as in Phases 3–7 when a
   module stopped being "planned"), retarget the test to guard the same property and report
   it.
5. If an existing file must change: explain why, make the smallest change, preserve
   compatibility, add a regression test, and list it in the final report.
6. Reuse `ResolutionContext` / `ResolutionDecision` / `ResolutionLevel` / `AdaptiveMap`; do
   not duplicate contracts.
7. Keep the honesty rules: unmeasured values are `null` with a reason, `source` provenance is
   mandatory, baselines are labelled `is_baseline`, risk stays normalised to `[0, 1]`
   (ADR-006), **no fabricated accuracy or performance figures**, and nothing describes the
   risk score as a probability, calibrated or validated.
