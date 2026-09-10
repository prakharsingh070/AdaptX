# Experiment Log

Record measured experiments here. Do not replace missing values with estimates.

## Experiment 001 - Phase 2C LiDAR pipeline throughput

**Date:** 2026-09-09

**Scenario:** Synthetic `small` / `medium` / `large` road scenes from
`adaptx.benchmark.datasets`. Flat road plane, three boxes, sparse clutter and 2%
non-returns.

**Random seed:** 20260101

**Vehicles / Pedestrians / Cyclists:** None. The geometry is boxes on a plane,
not actors - no simulator was involved.

**Weather and time:** Not modelled.

**CARLA map and version:** Not used. CARLA integration is a boundary only.

**Hardware and software:** Windows 11, 16 logical CPUs, Python 3.13.7,
NumPy 2.5.3, adaptx 0.1.0. Single-threaded.

**Baseline:** `fixed_resolution_baseline` - every stage enabled at a fixed
0.20 m voxel size (ADR-018). Compared against `filter_only`, which runs the
Phase 2A stages alone.

**ADAPT-X configuration:** Not applicable. Adaptive resolution does not exist;
this experiment measures the fixed-resolution pipeline only.

**Measurement window:** 3 warm-up runs discarded, 9 timed repeats, median
reported. Peak memory measured in a separate run with `tracemalloc`, so its
overhead does not enter the timings.

**Results:**

| Scenario | Points in | Points out | Median ms | Spread ms | Points/s | Frames/s | Peak MB |
|---|---|---|---|---|---|---|---|
| small | 10,000 | 1,340 | 21.17 | 19.7-24.3 | 472,271 | 47.2 | 2.0 |
| medium | 100,000 | 2,497 | 212.97 | 187.3-245.7 | 469,541 | 4.7 | 18.5 |
| large | 400,000 | 7,328 | 899.72 | 871.0-933.9 | 444,583 | 1.1 | 67.4 |

`filter_only` profile, same data and method:

| Scenario | Points in | Points out | Median ms | Points/s | Frames/s |
|---|---|---|---|---|---|
| small | 10,000 | 9,800 | 3.42 | 2,928,000 | 292.8 |
| medium | 100,000 | 97,999 | 39.14 | 2,555,231 | 25.6 |
| large | 400,000 | 391,999 | 138.59 | 2,886,161 | 7.2 |

Per-stage timing, `large`, single representative run:

| Stage | ms | Points in -> out |
|---|---|---|
| validation | 0.00 | 400,000 -> 400,000 |
| invalid_removal | 14.62 | 400,000 -> 392,000 |
| roi_filter | 8.80 | 392,000 -> 392,000 |
| range_filter | 6.93 | 392,000 -> 391,999 |
| voxel_downsample | 443.60 | 391,999 -> 214,667 |
| ground_segmentation | 233.60 | 214,667 -> 25,443 |
| noise_filter | 60.02 | 25,443 -> 7,328 |

- FPS: see the table. Defined as `1000 / median_ms` for single-threaded
  sequential processing of one dataset. **Not** end-to-end system FPS.
- Latency: median frame duration, as above.
- CPU: not measured per-run.
- GPU: not measured. No GPU code path exists.
- Memory: peak Python-tracked allocation, in the table.
- Processed points: in the table.
- Active cells / high-resolution area / detection / tracking / prediction
  metrics / missed detections / conflict detection: **not applicable.** None of
  those subsystems exist.

**Observations:**

1. The Phase 2B stages dominate: at 400k points they account for roughly 85% of
   the frame (899.7 ms total against 138.6 ms for filtering alone).
2. Voxel downsampling is the single largest cost, about half the frame. Most of
   that is the grouped nearest-to-centroid selection, which is the price of
   ADR-015 - keeping a real measured point instead of emitting a synthesised
   centroid.
3. Replacing a 3-key `lexsort` with a stable integer sort plus a grouped minimum
   gave a measured **1.30x** on that step (212.2 ms -> 163.7 ms, controlled
   interleaved A/B on identical data, both strategies verified to select
   identical indices). End to end that is about 5%, which is within run-to-run
   noise at whole-pipeline level.
4. Run-to-run spread on this machine reaches ~13% at medium size, so single
   medians should not be compared across sessions without the spread beside
   them.
5. Throughput is roughly flat at 440k-470k points/s across the size ladder,
   suggesting the pipeline is bandwidth- rather than overhead-bound at these
   sizes.

**Conclusion:** The fixed-resolution pipeline processes a 100k-point frame in
about 210 ms single-threaded on this machine. **No real-time claim is made**,
and none is supported: this is synthetic geometry, one machine, one thread, and
no perception work beyond filtering and segmentation. The value of this
measurement is as the reference point that later adaptive work is compared
against.

**Artifacts:** Reproduce with
`python -m adaptx.benchmark --repeats 9 --warmup 3 --json <path>`. The JSON
report carries the dataset, seed, configuration, version and environment.

## Experiment 002 - Phase 3 geometric detection throughput

**Date:** 2026-09-10

**Scenario:** The same synthetic `small` / `medium` / `large` road scenes as
Experiment 001, processed by the fixed-resolution pipeline and then passed to the
geometric detector.

**Random seed:** 20260101

**Hardware and software:** Windows 11, 16 logical CPUs, Python 3.13.7,
NumPy 2.5.3, adaptx 0.1.0. Single-threaded.

**Configuration:** Pipeline as the fixed-resolution baseline (0.20 m voxel, ground
segmentation and noise filtering on). Detector at defaults: 0.5 m cluster
tolerance, 10-50000 points, height 0.15-4.5 m, footprint 0.10-15.0 m.

**Measurement window:** 3 warm-up runs discarded, 9 timed repeats, median reported.
Processing and detection timed separately.

**Results:**

| Scenario | Raw pts | Non-ground | Clusters | Objects | Rejected | Process ms | Detect ms | Detect pts/s |
|---|---|---|---|---|---|---|---|---|
| small | 10,000 | 1,340 | 3 | 3 | 0 | 18.95 | 2.74 | 489,748 |
| medium | 100,000 | 2,497 | 24 | 3 | 21 | 178.74 | 3.42 | 729,733 |
| large | 400,000 | 7,328 | 1,202 | 96 | 1,106 | 683.79 | 40.35 | 181,612 |

Classes assigned (geometric baseline, **not** verified against labels):

| Scenario | Classes |
|---|---|
| small | unknown=3 |
| medium | unknown=3 |
| large | vehicle=20, unknown=65, obstacle=7, cyclist=4 |

- Detection accuracy: **not measured, and not measurable.** No dataset here carries
  labels. The class counts above record what the heuristic decided, not what is
  correct.
- FPS: not reported for detection alone; the pipeline figures are in Experiment 001.
- CPU / GPU / memory: not measured for this experiment.

**Observations:**

1. Detection is cheap relative to processing - 3.4 ms against 178.7 ms at the
   medium size, roughly 2%. The cost sits in cleaning the scan, not in finding
   objects in what survives.
2. Detection cost scales with **non-ground points and cluster count**, not raw
   points. `large` has 3x the non-ground points of `medium` but 50x the clusters,
   and takes 12x longer: the per-cluster Python loop that assembles geometry and
   classifies dominates once clusters number in the thousands.
3. The `large` scene produces 1,202 clusters of which 1,106 are rejected. The
   synthetic ground plane is rough enough that voxelisation leaves fragments which
   survive segmentation and cluster into specks. That is a property of the
   generator, not evidence about real scans.
4. Most `large` detections are `UNKNOWN` (65 of 96). Expected: the generator's
   "boxes" are point clouds sampled from a distribution rather than clean
   surfaces, so measured extents rarely land inside a single dimension band.
   Reporting `UNKNOWN` rather than guessing is the intended behaviour (ADR-021).

**Conclusion:** Detection adds a small fraction to frame cost at realistic cluster
counts and becomes the dominant term only when the scene fragments into
thousands of candidates. **No real-time claim is made.** No accuracy claim is made
or possible.

**Artifacts:** Reproduce with
`python -m adaptx.benchmark --detect --repeats 9 --warmup 3 --json <path>`.

## Experiment 003 - Phase 4 tracking throughput

**Date:** 2026-09-10

**Scenario:** Synthetic detections on a grid, each drifting at constant velocity,
10 frames per sequence at 0.1 s intervals. Spacing is wide enough that no two
objects compete for one detection, so the measurement reflects association cost
rather than gate contention. The tracker is measured **alone**, not behind the
LiDAR pipeline: association cost scales with object count, not point count, and
running the pipeline first would bury it under clustering.

**Random seed:** Not applicable - the scene is deterministic by construction, with
no random element.

**Hardware and software:** Windows 11, 16 logical CPUs, Python 3.13.7,
NumPy 2.5.3, adaptx 0.1.0. Single-threaded.

**Configuration:** 3.0 m association gate, defaults elsewhere.

**Measurement window:** 2 warm-up sequences discarded, 5 timed repeats of 10
frames each, median of all frame updates reported.

**Results:**

| Objects | Tracks | Matched | Update ms | Association ms | Spread ms | Objects/s |
|---|---|---|---|---|---|---|
| 5 | 5 | 5 | 0.168 | 0.053 | 0.03-0.42 | 29,674 |
| 25 | 25 | 25 | 1.162 | 0.621 | 0.12-1.42 | 21,512 |
| 100 | 100 | 100 | 10.624 | 8.332 | 0.43-14.60 | 9,413 |
| 500 | 500 | 500 | 218.954 | 205.855 | 2.02-260.90 | 2,284 |

- Tracking correctness: **not measured, and not measurable.** No labelled
  sequences exist. Every object was matched in every frame here because the scene
  was built so that it could be; that is a property of the generator, not
  evidence that association is correct on real data.
- CPU / GPU / memory: not measured for this experiment.

**Observations:**

1. Association dominates the update - 206 ms of 219 ms at 500 objects - and is
   `O(T x D)`. Cost grows roughly with the square of object count: 20x from 100
   to 500 objects for a 5x increase in count.
2. Hoisting per-detection work out of the inner loop and gating on squared
   distance gave a measured **2.6x** at 500 objects (569 ms -> 219 ms) and 2.4x
   at 100 (25.3 ms -> 10.6 ms), with all 59 tracker tests unchanged, so the
   change is semantics-preserving.
3. At the object counts this pipeline actually produces - the large detection
   scene yields 96 - tracking costs about 10 ms, roughly 1.5% of the ~700 ms that
   frame spends in processing. Tracking is not the bottleneck at realistic
   scales.
4. The spread is wide at every size because the first frame of each sequence has
   no tracks to associate against and completes almost instantly, which drags the
   minimum down.

**Conclusion:** The tracker is comfortably cheap at realistic object counts and
becomes the dominant cost only in the hundreds, where the quadratic term takes
over. **No real-time claim is made.** No correctness claim is made or possible.

**Artifacts:** Reproduce with
`python -m adaptx.benchmark --track --repeats 5 --warmup 2 --json <path>`.

## Experiment 004 - Phase 5 trajectory prediction throughput

**Date:** 2026-09-10

**Scenario:** Synthetic tracks on a grid, each carrying a measured constant
velocity, predicted over a 3.0 s horizon at 0.25 s intervals - 13 points per
track, `t+0` to `t+3.00` inclusive. Every track is eligible, so the measurement
reflects the full extrapolation path rather than a run of cheap skips. The
predictor is measured **alone**, not behind the LiDAR pipeline: prediction cost
scales with track count and points per trajectory, not with point count, and
running the pipeline first would bury it under clustering.

**Random seed:** Not applicable - the scene is deterministic by construction,
with no random element. Velocities vary by index (`5 + index % 7` m/s forward,
`index % 3 - 1` m/s lateral) so no case degenerates into one repeated
computation.

**Hardware and software:** Windows 11, 16 logical CPUs, Python 3.13.7,
NumPy 2.5.3, adaptx 0.1.0. Single-threaded.

**Configuration:** `horizon_s=3.0`, `interval_s=0.25`, `base_uncertainty_m=0.5`,
`uncertainty_growth_mps=0.5`, `confidence_hits_full=3`, `max_speed_mps=80.0`,
`max_tracks` raised so no track is skipped by the limit.

**Measurement window:** 4 warm-up passes discarded, 15 timed repeats, median
reported. Peak memory measured in a separate dedicated `tracemalloc` run, never
during the timed repeats.

**Results:**

| Tracks | Points | Median ms | Spread ms | Tracks/s | Points/s | Peak MB |
|---|---|---|---|---|---|---|
| 5 | 65 | 1.184 | 0.80-4.03 | 4,224 | 54,917 | 0.10 |
| 25 | 325 | 5.936 | 5.13-7.09 | 4,211 | 54,749 | 0.54 |
| 100 | 1,300 | 27.045 | 22.41-31.55 | 3,698 | 48,068 | 2.18 |
| 500 | 6,500 | 143.753 | 121.03-233.41 | 3,478 | 45,216 | 10.98 |

- Prediction correctness: **not measured, and not measurable.** No labelled
  trajectories exist. A constant-velocity extrapolation of a vehicle that then
  brakes or turns is wrong, and no figure above says otherwise.
- CPU / GPU: not measured for this experiment.

**Observations:**

1. Cost is **linear** in trajectory points: throughput sits between 45,000 and
   55,000 points/s across a 100x range of track counts. Unlike Phase 4
   association (`O(T x D)`, Experiment 003), prediction has no quadratic term -
   each track is extrapolated independently of every other.
2. The dominant cost is **contract validation, not arithmetic**. A cProfile run
   at 500 tracks attributed 0.95 s of a 1.51 s five-pass total to
   `pydantic.main.__init__` across 67,510 calls - 27 model constructions per
   track (13 `TrajectoryPoint`, 13 `Vector3`, 1 `PredictedTrajectory`). The
   extrapolation itself is a handful of multiplications per point.
3. Building each point's absolute timestamp inside the per-track loop was
   redundant work: the absolute times are identical for every track. Hoisting
   them to once per call removes a measured **13.4 ms per 500-track pass**
   (measured in isolation: 6,500 `datetime + timedelta` operations), about 11%
   of that pass. End-to-end the difference sits inside this machine's run-to-run
   spread (stdev 13.4 ms at 500 tracks), so **no end-to-end speedup is claimed** -
   only that strictly less work is now done, with all 109 prediction tests
   unchanged.
4. At the track counts this pipeline actually produces - the large detection
   scene yields 96 objects - prediction costs roughly 27 ms, comparable to
   Phase 4 tracking at the same scale (10.6 ms) and small against the ~700 ms
   that frame spends in processing.
5. Memory scales linearly and stays modest: 11 MB of Python-tracked allocation
   for 6,500 trajectory points, which is the cost of the point objects
   themselves.

**Conclusion:** Prediction is linear in output size and cheap at realistic track
counts, with validation rather than arithmetic setting the floor. If the cost
ever matters, the lever is the number of points emitted (`interval_s`), not the
motion model. **No real-time claim is made.** No correctness claim is made or
possible.

**Artifacts:** Reproduce with
`python -m adaptx.benchmark --predict --repeats 15 --warmup 4 --json <path>`.

## Experiment 005 - Phase 6 2.5D mapping throughput and the cost of resolution

**Date:** 2026-09-10

**Scenario:** The three size-ladder datasets, run through the **filter-only** pipeline
profile and then mapped at three fixed resolutions. The filter-only profile is used
deliberately: the full baseline profile voxelises at 0.2 m, which collapses the 400k dataset
to about 7k points - a real and useful reduction, but it would leave this benchmark
measuring mapping at point counts nothing like the ones under study. Phase 2A validation,
ROI and range filtering still run, so the mapper receives a genuine processed frame.

Only the mapping call is timed; pipeline time is excluded.

**Random seed:** The dataset generator's default seed. The same (scenario, seed) always
yields the same array, and mapping is deterministic, so a repeat run reproduces the cell
counts exactly.

**Hardware and software:** Windows 11, 16 logical CPUs, Python 3.13.7, NumPy 2.5.3,
adaptx 0.1.0. Single-threaded.

**Configuration:** Bounds x [-100, 100] m, y [-100, 100] m. Resolutions 1.00 / 0.50 /
0.25 m, giving 200x200, 400x400 and 800x800 grids.

**Measurement window:** 3 warm-up runs discarded, 11 timed repeats, median reported. Peak
memory measured in a separate dedicated `tracemalloc` run, never during the timed repeats.

**Results:**

| Scenario | Res (m) | Points | Grid | Cells | Occupied | Occ % | Median ms | Points/s | Grid MB |
|---|---|---|---|---|---|---|---|---|---|
| small | 1.00 | 9,800 | 200x200 | 40,000 | 4,519 | 11.30 | 3.303 | 2,967,359 | 1.2 |
| small | 0.50 | 9,800 | 400x400 | 160,000 | 6,650 | 4.16 | 7.474 | 1,311,300 | 4.9 |
| small | 0.25 | 9,800 | 800x800 | 640,000 | 7,518 | 1.17 | 23.147 | 423,381 | 19.5 |
| medium | 1.00 | 97,999 | 200x200 | 40,000 | 6,600 | 16.50 | 17.412 | 5,628,277 | 1.2 |
| medium | 0.50 | 97,999 | 400x400 | 160,000 | 24,937 | 15.59 | 25.101 | 3,904,203 | 4.9 |
| medium | 0.25 | 97,999 | 800x800 | 640,000 | 54,182 | 8.47 | 52.184 | 1,877,944 | 19.5 |
| large | 1.00 | 391,999 | 200x200 | 40,000 | 6,600 | 16.50 | 70.813 | 5,535,677 | 1.2 |

Peak tracked memory ranged from 2.3 MB (small, 1.00 m) to 51.7 MB (large, 0.25 m).

**Supplementary 1M-point measurement**, run separately from the standard benchmark because
no 1M dataset exists and adding one would mean editing a Phase 2C module. Points are drawn
from a synthetic road-like distribution (70% ground plane, 30% raised structure) over the
same bounds, seed 20260910, 2 warm-up runs and 7 timed repeats:

| Res (m) | Points | Grid | Cells | Occupied | Median ms | Points/s |
|---|---|---|---|---|---|---|
| 1.00 | 1,000,000 | 200x200 | 40,000 | 36,100 | 214.43 | 4,663,607 |
| 0.50 | 1,000,000 | 400x400 | 160,000 | 144,236 | 246.92 | 4,049,882 |
| 0.25 | 1,000,000 | 800x800 | 640,000 | 475,332 | 283.39 | 3,528,738 |

- Map correctness: **not measured, and not measurable.** No labelled reference map exists.
  Nothing here says a map is right, and nothing says a resolution is *appropriate* - that
  second question is what adaptive resolution will exist to answer.
- Out-of-bounds points were zero throughout: the generated scenes fit inside the benchmark
  bounds. The out-of-bounds path is covered by unit tests instead.
- CPU / GPU: not measured for this experiment.

**Observations:**

1. **Cost has two independent drivers, and the benchmark separates them.** At 9,800 points
   the time is almost entirely the grid: going from 1.00 m to 0.25 m multiplies cells by 16
   and time by 7.0x, while the point count never changes. At 1,000,000 points the same
   resolution change costs only 1.3x, because points now dominate. This is the central
   measurement of Phase 6: **a uniform fine map pays for detail everywhere, including where
   nothing is happening.**
2. **Occupancy falls as resolution rises** - 11.30% to 1.17% on the small dataset. A finer
   uniform map spends a rapidly growing majority of its cells recording that nothing was
   observed. That gap is the headroom an adaptive mapper would be trying to reclaim, and it
   is now a measured number rather than an assumption.
3. **Grid memory is fixed by geometry, not by data**: 1.2 / 4.9 / 19.5 MB for the three
   resolutions regardless of point count, being `width * height * 32` bytes across one
   int64 and three float64 arrays. Peak tracked allocation adds the per-point temporaries on
   top.
4. **A defect was found and fixed by profiling.** The first implementation allocated four
   full-grid arrays up front and then seven more inside the accumulation branch - eleven
   full-grid allocations where six suffice, which at 640,000 cells is substantial waste. The
   rewrite builds the flat arrays once and reshapes them, and uses `np.fmin`/`np.fmax`,
   which ignore NaN, so untouched cells keep their unobserved state without a second masking
   pass. All 114 Phase 6 tests were unchanged by the rewrite, so it is semantics-preserving.
5. **Remaining profile at 392k points, 1.0 m** (cProfile, 5 passes): 37 ms in the `build`
   body (boolean masking and `bincount`), 19 ms in `cell_indices` quantisation, 11 ms in the
   `fmin`/`fmax` `.at` calls, 5 ms in `clip`, 4 ms in `astype`. The `.at` calls are the
   classic NumPy slow path and could be replaced with a sort-and-`reduceat` grouping, but at
   14% of the pass that was judged **not worth the added complexity**, and was left alone
   rather than optimised speculatively.
6. In the real pipeline the mapper sees the *voxelised* frame, not the raw one. An earlier
   run through the full baseline profile reduced the 400k dataset to 7,328 points, where
   mapping cost 5.9 / 12.5 / 35.8 ms at the three resolutions - grid-dominated, as
   observation 1 predicts.

**Conclusion:** Mapping is linear in points and linear in cells, with the cell term
dominating at realistic post-voxelisation point counts. The measured occupancy collapse at
fine resolution is the first quantitative evidence for the problem ADAPT-X exists to solve;
it is **not** evidence that an adaptive mapper would do better, because no adaptive mapper
exists to measure. **No real-time claim is made.** No correctness claim is made or possible.

**Artifacts:** Reproduce with
`python -m adaptx.benchmark --map --repeats 11 --warmup 3 --json <path>`.

## Experiment 006 - Phase 7 risk assessment throughput

**Date:** 2026-09-10

**Scenario:** Synthetic tracks on a ring around the ego reference, each at a distance
cycling through 3-42 m and closing radially at a rate cycling through 1-20 m/s, so the score
spans the scale rather than collapsing into one band. Every track carries a measured
velocity, a real Phase 5 trajectory and a real Phase 6 map, so every object exercises all
three factors - the engine's worst realistic case rather than a lucky run of dropped
factors.

Trajectories come from the actual `ConstantVelocityPredictor` and the map from the actual
`FixedResolutionMapper`, so the benchmark measures what the engine receives in the pipeline.
The Phase 5 per-call track limit is raised for generation only; it bounds API responses and
would otherwise leave the larger cases without trajectories.

Only the assessment call is timed; generation is excluded.

**Random seed:** Not applicable - the scene is deterministic by construction, with no random
element. Risk assessment is deterministic, so a repeat run reproduces the level distribution
exactly.

**Hardware and software:** Windows 11, 16 logical CPUs, Python 3.13.7, NumPy 2.5.3,
adaptx 0.1.0. Single-threaded.

**Configuration:** Defaults - proximity 5-40 m, closing speed saturating at 15 m/s, weights
0.50 / 0.25 / 0.25, thresholds 0.35 / 0.60 / 0.85. Map 120 m square at 0.5 m.

**Measurement window:** 3 warm-up passes discarded, 11 timed repeats, median reported. Peak
memory measured in a separate dedicated `tracemalloc` run, never during the timed repeats.

**Results:**

| Objects | Scored | With trajectory | Low | Medium | High | Critical | Unknown | Median ms | Spread ms | Objects/s | µs/object | Peak MB |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 5 | 5 | 5 | 0 | 0 | 5 | 0 | 0 | 0.370 | 0.33-0.60 | 13,510 | 74.0 | 0.02 |
| 25 | 25 | 25 | 0 | 5 | 20 | 0 | 0 | 1.900 | 1.74-2.22 | 13,160 | 76.0 | 0.11 |
| 100 | 100 | 100 | 0 | 40 | 60 | 0 | 0 | 7.369 | 6.77-11.84 | 13,571 | 73.7 | 0.45 |
| 500 | 500 | 500 | 0 | 240 | 260 | 0 | 0 | 39.939 | 37.06-45.16 | 12,519 | 79.9 | 2.32 |
| 1000 | 1000 | 1000 | 0 | 500 | 500 | 0 | 0 | 81.544 | 73.57-139.96 | 12,263 | 81.5 | 4.66 |

**Synthetic benchmark; not a real-world autonomous-driving performance claim.**

- Assessment correctness: **not measured, and not measurable.** No labelled risk data exists.
  Nothing here says an assessment is right, and nothing says the thresholds are appropriate.
- The level distribution describes the **generated scene**, not the quality of the engine. It
  is reported so a reader can see the cases were not all trivially low, not as a result.
- No `CRITICAL` objects appear because reaching 0.85 needs proximity, closing speed **and**
  predicted proximity all near maximum simultaneously; the ring generator never places an
  object that close while also closing that fast. The band is exercised in the unit tests.
- CPU / GPU: not measured for this experiment.

**Observations:**

1. **Cost is linear in object count**, at 74-82 µs per object across a 200x range. There is
   no quadratic term: each object is assessed independently, unlike Phase 4 association
   (`O(T x D)`, Experiment 003), and unlike Phase 6 mapping there is no grid term that scales
   independently of the data.
2. **Per-object cost is dominated by contract construction, not arithmetic.** Each assessment
   builds four Pydantic models - `RiskAssessment`, `UncertaintyBreakdown`, `MapContext` and
   usually `TrajectoryRelevance` - plus a `RiskFactors`. The scoring itself is a handful of
   multiplications and a `min` over 13 trajectory points. This mirrors the finding in
   Experiment 004, where prediction cost was likewise validation-bound.
3. **The trajectory scan is the only per-object loop over a collection**, and it is bounded by
   the Phase 5 horizon and interval - 13 points at the defaults. Halving `interval_s` would
   double it; the assessment cost would follow.
4. At the object counts this pipeline actually produces - the large detection scene yields 96
   - risk assessment costs roughly 7 ms, comparable to Phase 4 tracking (10.6 ms) and Phase 5
   prediction (27 ms) at the same scale, and small against the ~700 ms that frame spends in
   processing.
5. Memory is modest and linear: 4.7 MB of Python-tracked allocation for 1000 assessments,
   which is the cost of the model objects themselves.
6. The spread widens at 1000 objects (73.6-140.0 ms). That is allocation pressure and garbage
   collection during a pass that constructs ~5000 models, not variance in the algorithm - the
   median is stable across repeats and the output is bit-identical.

**Conclusion:** Risk assessment is linear in object count and cheap at realistic scales, with
model construction rather than scoring setting the floor. If the cost ever matters, the levers
are the number of objects assessed (`max_assessed_tracks`) and the number of trajectory points
scanned (Phase 5's `interval_s`), not the scoring formulation. **No real-time claim is made.**
No correctness claim is made or possible.

**Artifacts:** Reproduce with
`python -m adaptx.benchmark --risk --repeats 11 --warmup 3 --json <path>`.

## Experiment Template

### Experiment XXX

**Date:**

**Scenario:**

**Random seed:**

**Vehicles:**

**Pedestrians:**

**Cyclists:**

**Weather and time:**

**CARLA map and version:**

**Hardware and software:**

**Baseline:** Fixed resolution, value and units.

**ADAPT-X configuration:** Resolution range, thresholds, smoothing, prediction horizon, and relevant settings.

**Measurement window:** Warm-up, duration, sample count, and aggregation method.

**Results:**

- FPS:
- Latency:
- CPU:
- GPU:
- Memory:
- Processed points:
- Active cells:
- High-resolution area:
- Detection metrics:
- Tracking metrics:
- Prediction metrics:
- Missed detections:
- Conflict detection:

**Observations:**

**Conclusion:**

**Artifacts:** Links or paths to configuration, raw measurements, logs, and plots.

---

## Experiment 007 - Phase 8 adaptive resolution versus the fixed baseline

**Date:** 2026-09-11

**Scenario:** Eight synthetic scenes over a 120 m square map, each built to differ in exactly
one thing the resolution policy is supposed to notice: `open_empty`,
`single_low_risk_object`, `single_high_risk_object`, `multiple_objects`, `high_uncertainty`,
`predicted_trajectory`, `dense_scene` and `mixed_complexity`. Every scene is ground returns
plus a block of returns per object.

Risk, uncertainty and speed are **stated per object rather than derived**, so a scenario
isolates the factor it is named for instead of depending on what the Phase 7 heuristic happens
to produce for a given geometry. `high_uncertainty` includes two objects whose risk could not
be scored at all (`risk_score = None`), which is the case the allocation must not read as
quiet.

Each scene is mapped twice from identical input: once by the Phase 6 `FixedResolutionMapper`
at a uniform cell size, once by the Phase 8 controller and `TiledAdaptiveMapper`. Planning and
mapping are timed separately, because they scale with different things.

**Random seed:** 20260101, fixed. The scenes and the allocation are both deterministic; a
repeat run reproduces the level distribution and the cell counts exactly.

**Hardware and software:** Windows 11, 16 logical CPUs, Python 3.13.7, NumPy 2.5.3,
adaptx 0.1.0. Single-threaded.

**Configuration:** Defaults - 10 m regions, levels LOW/MEDIUM/HIGH/CRITICAL at 1.0 / 0.5 /
0.2 / 0.1 m, thresholds 0.35 / 0.60 / 0.85, weights 0.35 risk / 0.20 uncertainty / 0.15
proximity / 0.15 trajectory / 0.10 density / 0.05 motion, influence radius 8 m widened by up
to 4 m with uncertainty, hysteresis margin 0.08, minimum dwell 3 frames. Budgets were set wide
enough not to bind (4,096 regions, 4,000,000 cells, 64 fine regions); `demoted` is 0 in every
case below, so these figures measure the policy and not the budget.

**Measurement window:** 3 warm-up passes discarded, 9 timed repeats, median reported. A fresh
controller is constructed for each repeat so stabilisation state cannot change what is
measured. Peak memory measured in a separate dedicated `tracemalloc` run.

### Cells: adaptive against three uniform baselines

The map is 120 m square, so the uniform baselines are 14,400 cells at 1.0 m, 57,600 at 0.5 m
and 230,400 at 0.25 m. Adaptive cell counts do not depend on the baseline - the same plan is
compared against each.

| Scene | Objects | Regions LOW/MED/HIGH/CRIT | Adaptive cells | vs 1.0 m | vs 0.5 m | vs 0.25 m | Cells on high-priority regions |
|---|---|---|---|---|---|---|---|
| open_empty | 0 | 144 / 0 / 0 / 0 | 14,400 | 1.000 | 0.250 | 0.062 | n/a |
| single_low_risk_object | 1 | 144 / 0 / 0 / 0 | 14,400 | 1.000 | 0.250 | 0.062 | n/a |
| single_high_risk_object | 1 | 140 / 3 / 1 / 0 | 17,700 | 1.229 | 0.307 | 0.077 | 14.1% |
| multiple_objects | 4 | 132 / 10 / 2 / 0 | 22,200 | 1.542 | 0.385 | 0.096 | 22.5% |
| high_uncertainty | 3 | 117 / 24 / 3 / 0 | 28,800 | 2.000 | 0.500 | 0.125 | 26.0% |
| predicted_trajectory | 1 | 134 / 10 / 0 / 0 | 17,400 | 1.208 | 0.302 | 0.076 | n/a |
| dense_scene | 45 | 44 / 88 / 12 / 0 | 69,600 | 4.833 | 1.208 | 0.302 | 43.1% |
| mixed_complexity | 3 | 135 / 5 / 4 / 0 | 25,500 | 1.771 | 0.443 | 0.111 | 39.2% |

### Latency, and the result that matters most

| Scene | Planning ms | Adaptive mapping ms | Fixed mapping ms (0.5 m) | Adaptive total ms |
|---|---|---|---|---|
| open_empty | 6.80 | 26.01 | 6.32 | 32.81 |
| single_high_risk_object | 7.93 | 26.92 | 6.52 | 34.85 |
| multiple_objects | 9.47 | 30.05 | 6.59 | 39.52 |
| high_uncertainty | 8.99 | 26.82 | 6.12 | 35.81 |
| predicted_trajectory | 8.32 | 25.00 | 5.35 | 33.32 |
| dense_scene | 13.03 | 32.21 | 8.20 | 45.24 |
| mixed_complexity | 9.07 | 25.47 | 5.30 | 34.54 |

### What drives adaptive mapping cost

The latency above is dominated by neither cells nor points. Holding the cell count **exactly
constant at 14,400** on the empty scene and varying only the tile size:

| Tile size | Regions | Cells | Adaptive mapping ms | µs per region |
|---|---|---|---|---|
| 5 m | 576 | 14,400 | 77.20 | 134 |
| 10 m | 144 | 14,400 | 26.69 | 185 |
| 20 m | 36 | 14,400 | 13.50 | 375 |
| 30 m | 16 | 14,400 | 10.16 | 635 |
| 60 m | 4 | 14,400 | 8.94 | 2,234 |
| 120 m | 1 | 14,400 | 10.43 | 10,426 |

Same cells, same points, 8.6x the time. **Cost scales with region count, not cell count**, at
roughly 120 µs of fixed overhead per region - three NaN array allocations, a reshape and a
validated `MapTile` per region. A profile attributes about 78% of a pass to `_build_tile` and
its model construction, the same "contract construction dominates the arithmetic" finding as
Experiments 004 and 006.

**Synthetic benchmark; not a real-world autonomous-driving performance claim.**

**Findings, including the unflattering ones:**

- **Adaptive costs more cells than a 1.0 m uniform map in every scene containing an object**
  (1.21x to 4.83x). This is not a defect, it is arithmetic: the base level *is* 1.0 m, so
  against that baseline the policy can only ever add cells. Adaptive resolution is a way of
  affording a fine map, not a way of beating a coarse one.
- **Against the finer baselines it wins clearly**: 0.30x at 0.5 m and 0.076x at 0.25 m on
  `mixed_complexity`, and 0.062x at 0.25 m on an empty scene. Experiment 005 measured
  occupancy falling to 1-16% at 0.25 m - a uniform fine map spending most of its cells
  recording that nothing was observed. The adaptive map reaches comparable detail *where the
  priority is* for a twelfth of the cells.
- **Adaptive mapping is slower in wall-clock time than the fixed mapper in every scene**, even
  where it allocates a quarter of the cells - 25-32 ms against 5-8 ms at 0.5 m. Detail is
  bought with cells and paid for in per-region overhead, and at 144 regions the overhead wins.
  A negative result, measured and reported as such.
- **The lever is `tile_size_m`, not the resolution vocabulary.** The table above is the
  evidence. Larger regions cost less and allocate detail more coarsely; that trade is now a
  measured number rather than a guess.
- **One small optimisation was applied and did not help.** The mapper was changed to reuse the
  region extent the plan already carries instead of rebuilding it per region, which also let
  it validate that the plan geometry matches the tiling. Measured before and after: no change
  outside run-to-run noise (~27 ms either way). It is retained for the added validation, not
  for speed. The remaining cost is the per-region array allocation and model construction, and
  removing that would mean changing the representation - which is where a hierarchical grid
  would earn its complexity (ADR-037).
- **The policy differentiates as designed.** `open_empty` and `single_low_risk_object` stay
  entirely at the base level; `high_uncertainty` refines 27 regions on uncertainty alone, with
  two of its three objects carrying no risk score at all; `predicted_trajectory` refines 10
  regions ahead of an object that has not arrived; `mixed_complexity` puts 39.2% of its cells
  on the 4 regions that reached the HIGH threshold while 135 regions stay coarse.
- **Peak tracked memory is ~2.0 MB per adaptive pass** across every scene, dominated by the
  point arrays rather than the grid.

**Not measured, and not measurable:**

- **Map correctness.** No labelled reference map exists. Nothing here says either map is right.
- **Whether the allocation is appropriate.** No labelled risk data exists, so nothing says the
  priority ordering is correct - only that it is deterministic, explainable and produces the
  spatial differentiation it was designed to produce.
- **What the adaptive map loses.** Where a region sits at LOW the map is coarser than a 0.5 m
  or 0.25 m uniform baseline would have been, and the structure inside those cells is not
  recorded. The cell counts above quantify the saving; no measurement quantifies the cost in
  represented detail, because that would need a reference map.
- **Any real-world figure.** These are generated scenes with hand-specified risk on one
  machine.

