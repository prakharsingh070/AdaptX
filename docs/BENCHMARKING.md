# LiDAR Pipeline Benchmarking

How the Phase 2C benchmark measures the LiDAR processing pipeline, and what its
numbers do and do not mean.

The wider evaluation strategy — fixed-resolution against adaptive perception, on detection,
tracking and safety behaviour — is Phase 11 and is described in
[`knowledge-base/13_benchmarking.md`](knowledge-base/13_benchmarking.md). **This is not
that.** Adaptive resolution does not exist yet, so no such comparison is possible or
attempted.

---

## What is measured

Point-cloud **processing speed only**. Nothing here measures perception accuracy, because
there is nothing to measure it against: the datasets are synthetic and carry no labels
(`ground_truth_available: false` on every one).

| Metric | Definition |
|---|---|
| `input_point_count` / `output_point_count` | Points entering and leaving the pipeline |
| `timing.median_ms` | Median wall-clock time across the timed repeats |
| `timing.min_ms` / `max_ms` / `stdev_ms` | The spread, so variance is visible rather than hidden |
| per-stage `duration_ms` | Time inside each stage, from one representative run |
| `overhead_ms` | Frame total minus the sum of stages — array compaction, frame construction, metric assembly |
| `peak_memory_mb` | Peak Python-tracked allocation, via `tracemalloc` |
| `points_per_second` | `input_point_count / median_seconds` |
| `frames_per_second` | `1000 / median_ms` |
| `reduction_ratio` | `1 − output/input` |

### On `frames_per_second`

Defined precisely as **one process, single-threaded, processing frames of this dataset back
to back**. It is pipeline throughput and nothing more.

It is **not** end-to-end system FPS. That figure would have to include sensor I/O, object
detection, tracking, mapping and the adaptive controller — none of which exist. Any claim
of system FPS would be fabricated.

### Values that are absent rather than invented

A rate over an empty dataset is undefined, so `points_per_second` and `reduction_ratio`
return `null` for the `empty` scenario, with the reason recorded in `unavailable`. They are
never reported as `0` or as an infinity.

---

## Method

1. **Warm-up.** A few runs are executed and discarded. The first pass over a fresh array
   pays page-fault and cache costs the steady state does not, and NumPy resolves some
   dispatch lazily.
2. **Timed repeats.** A fixed number of runs are timed and the **median** is reported. The
   median rather than the mean, because with few repeats a single scheduling hiccup moves a
   mean and barely moves a median.
3. **Memory separately.** Peak memory is measured in its own dedicated run with
   `tracemalloc` active — never during the timed repeats. `tracemalloc` instruments every
   allocation and materially slows execution, so timing it would measure the cost of
   measuring.
4. **One clock.** Timing comes from the pipeline's own `time.perf_counter` measurement, the
   same number it reports to any other caller. There is no benchmark-only clock that could
   drift from what production code sees.

`tracemalloc` was chosen over process RSS after checking both: RSS delta under-reports
badly here (0.05 MB observed for a 48 MB array, because untouched pages are not resident),
while `tracemalloc` tracked the full 48 MB.

---

## Datasets

All synthetic, all deterministic. Every frame is labelled
`source: synthetic_test` and can never be mistaken for sensor data.

| Scenario | Contents | Purpose |
|---|---|---|
| `small` | Road plane, three boxes, clutter, some non-returns (~10k points) | Size ladder |
| `medium` | Same composition (~100k) | Size ladder |
| `large` | Same composition (~400k) | Size ladder |
| `empty` | No points | Degenerate input |
| `dense` | 50k points inside a 1 m cube | Voxel collapse |
| `extreme_coordinates` | Points on and far beyond range/ROI bounds | Boundary and overflow guard |
| `all_ground` | A single flat plane | Segmentation upper bound |
| `no_ground` | A raised slab with nothing beneath it | Documented segmentation weakness |
| `noisy` | Road plus 20% isolated returns | Outlier filtering |

Reproducibility: `(scenario, seed)` always produces the identical array. The default seed is
`20260101`.

**The geometry is not a sensor model.** No beam divergence, no incidence-angle falloff, no
occlusion, no ring structure, no intensity physics. It exists so the pipeline has structure
to work on — a plane for segmentation, dense surfaces for voxelisation, isolated returns for
the noise filter. Swap in a recorded dataset when one is available; the generator is the
only thing that needs replacing.

---

## Profiles

| Profile | Configuration |
|---|---|
| `fixed_resolution_baseline` | Every stage on, one fixed voxel size everywhere (default 0.20 m) |
| `filter_only` | Phase 2A stages only — validation, non-finite removal, ROI, range |

Running both attributes cost: the difference is what downsampling, segmentation and noise
filtering actually add, rather than a guess.

Both profiles set `min_points: 0`, because the harness deliberately measures degenerate
input including a zero-point scan. That floor is a policy about acceptable sensor input; it
is not part of what the baseline tests.

---

## The fixed-resolution baseline

ADAPT-X claims that allocating resolution by risk beats spending it uniformly. Testing that
needs something to beat, and this is it: the pipeline at **one voxel size everywhere**,
which is what conventional preprocessing does.

It is a pinned, named *configuration*, not a new algorithm.

**It is not the fixed-resolution map baseline of ADR-003.** That one concerns cell size in
the 2.5D occupancy map in Phase 5, and does not exist. Keeping them separate matters:
conflating them would make it look as though ADAPT-X already had a mapping baseline.

| | Phase 2C (this) | ADR-003 (Phase 5, not built) |
|---|---|---|
| Fixes | Voxel size during point-cloud processing | Cell size in the 2.5D map |

Assumptions and limitations are recorded in `src/adaptx/benchmark/baseline.py` and ADR-018.
The default 0.20 m is a plausible starting point, **not** a tuned or validated value — no
measurement has established it as optimal for anything.

---

## Running it

```bash
.venv\Scripts\python.exe -m adaptx.benchmark
```

```bash
.venv\Scripts\python.exe -m adaptx.benchmark --profile filter_only --repeats 9 --warmup 3
```

```bash
.venv\Scripts\python.exe -m adaptx.benchmark --scenarios empty dense all_ground no_ground
```

```bash
.venv\Scripts\python.exe -m adaptx.benchmark --json data/processed/benchmark.json
```

The JSON report is the reproducible artefact. It carries the dataset, seed, effective
configuration, software version and environment alongside every measurement, so a result can
be reproduced from what it reports rather than from what anyone remembers about the run.

---

## Reading the results

Results are only comparable **within one machine, dataset and seed**. A median from one
laptop says nothing about another. Run-to-run variance on an ordinary desktop is easily 10%,
which is why the spread is printed next to every median rather than a single number that
looks more precise than it is.

Measured results are recorded in
[`experiments/experiment-log.md`](experiments/experiment-log.md). Nothing is written there
that was not measured.
