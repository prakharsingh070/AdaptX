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

---

## Experiment 008 - Phase 9 CARLA ingest conversion cost

**Date:** 2026-09-11

### What was NOT executed

**No live CARLA run took place.** The `carla` package is not installed in this environment
and no CARLA server was reachable, so the live smoke test
(`tests/integration/test_carla_live.py`, `pytest -m carla`) reported **6 skipped**.

Nothing below is a CARLA measurement. In particular this entry contains **no** figure for
simulator throughput, sensor delivery latency, frame rate, actor capacity or ground-truth
accuracy, because none was measured. When a server is available, a live run is recorded by
`python -m adaptx.scenarios run vehicle_approach --json <path>` (the Phase 10 replacement
for the `carla.smoke` command this entry originally named) and its output is labelled as such.

The lifecycle tests that did run exercise the adapter against
`tests/fixtures/fake_carla.py`, a hand-written stand-in. It does no physics, no rendering and
no real ray casting. It proves the *integration logic*; it says nothing about CARLA.

### What was measured

The conversion at the ingest boundary - decoding a CARLA-format LiDAR buffer and turning it
into a `RawPointCloudFrame` in the ADAPT-X frame. This is ADAPT-X code operating on synthetic
byte buffers of the shape CARLA produces (flat little-endian float32 xyzi), with no simulator
in the loop at any point.

**Scenario:** Buffers of increasing size, packed exactly as a CARLA `LidarMeasurement`
packs them. 28,000 points is one full sweep at the configured defaults - 560,000 points per
second at 20 Hz - so it is the size a real frame would be under this configuration.

**Random seed:** 20260101, fixed.

**Hardware and software:** Windows 11, 16 logical CPUs, Python 3.13.7, NumPy 2.5.3,
adaptx 0.1.0. Single-threaded.

**CARLA version:** not installed.

**Measurement window:** 3 warm-up passes discarded, 15 timed repeats, median reported.

**Results:**

| Points | Decode ms | Convert ms | Frame build ms | Total ms | Points/s |
|---|---|---|---|---|---|
| 7,000 | 0.003 | 0.041 | 0.057 | 0.060 | 117 M |
| 28,000 | 0.005 | 0.165 | 0.203 | 0.208 | 135 M |
| 56,000 | 0.011 | 1.134 | 1.212 | 1.223 | 46 M |
| 112,000 | 0.013 | 2.276 | 2.394 | 2.407 | 47 M |

*Frame build* includes the conversion, so *Total* is decode plus frame build; the separate
*Convert* column is shown to attribute the cost.

**Synthetic buffers, no simulator. Not a CARLA performance claim.**

**Findings:**

- **Decoding is effectively free** - 5 to 13 microseconds regardless of size. `np.frombuffer`
  reinterprets the buffer rather than parsing it, so no per-point work happens at all.
- **The cost is the copy, not the arithmetic.** The sign flip is one vectorised column
  operation; what it costs is materialising a float64 array from a float32 buffer. That is a
  deliberate choice - every other frame in the project is float64, and matching it avoids a
  dtype seam at the boundary.
- **At the configured frame size the conversion costs about 0.2 ms.** For scale, the Phase 2
  pipeline takes roughly 200 ms per 100k points (Experiment 002), so the boundary is not
  where time goes.
- The throughput drop between 28k and 56k points is consistent with the working set leaving
  cache. It is recorded as measured; no attempt was made to tune it, because at 0.2 ms per
  frame there is nothing worth tuning.

**Not measured, and not measurable here:**

- **Anything about CARLA.** No server ran.
- **Sensor delivery behaviour** - whether a tick reliably yields one full sweep, and what the
  real point count per frame is. Both depend on the server.
- **End-to-end latency with a real simulator in the loop.** Synchronous mode means ADAPT-X
  paces the simulator, so wall-clock throughput would measure the pipeline, not CARLA
  (ADR-044).
- **Detection, tracking or prediction accuracy.** Ground truth is now recorded beside every
  frame, which makes accuracy measurable **for the first time** - but measuring it is
  Phase 11, and nothing here attempts it.

---

## Experiment 009 - Phase 10 scenario orchestration cost

**Date:** 2026-09-11

### What was NOT executed

**No live CARLA run took place.** The `carla` package remains uninstalled in this environment,
so every catalogue scenario has been executed only against `tests/fixtures/fake_carla.py`, a
stand-in with no physics and no real ray casting. `pytest -m carla` reports **7 skipped**.

Nothing below is a CARLA measurement, a pipeline measurement, or an accuracy figure.

### What was measured

The cost of the scenario framework's **own** work, isolated from both the simulator and the
pipeline: resolving a definition from its seed, computing every actor's closed-form scripted
pose for every frame, and building the per-frame record. This is the orchestration overhead
Phase 10 adds on top of whatever the simulator and the pipeline cost.

**Scenario:** The four catalogue definitions. `resolve()` timed over 200 repeats;
`expected_pose()` for every actor on every frame over 20 repeats; a full `ScenarioRunner.run()`
with no processor over 3 repeats against the fake simulator.

**Random seed:** 20260101 (the catalogue seed). No catalogue scenario has a randomised element.

**Hardware and software:** Windows 11, 16 logical CPUs, Python 3.13.7, NumPy 2.5.3,
adaptx 0.1.0. Single-threaded.

**Results (medians):**

| Scenario | Frames | Actors | resolve µs | pose µs / frame | run ms (fake) | ms / frame |
|---|---|---|---|---|---|---|
| stationary_vehicle | 40 | 1 | 34 | 6.7 | 317 | 7.9 |
| vehicle_approach | 60 | 1 | 35 | 7.7 | 462 | 7.7 |
| pedestrian_crossing | 120 | 1 | 35 | 7.4 | 908 | 7.6 |
| cyclist_crossing | 80 | 2 | 44 | 14.4 | 731 | 9.1 |

**Synthetic; fake simulator; not a CARLA or pipeline performance claim.**

**Findings:**

- **The framework's own cost is negligible.** Resolving a scenario is ~35 µs once; scripted
  poses are ~7 µs per actor per frame. Both scale linearly with actor count, as the
  closed-form arithmetic predicts, and neither is worth optimising.
- **The run column measures the fake, not the runner.** ~7.6 ms/frame is almost entirely the
  stand-in generating a ground plane and vehicle shells on every tick; the runner contributes
  the two figures to its left plus record construction. Against a real CARLA server this
  column would be dominated by the simulator instead, and would say nothing about ADAPT-X.
- No pipeline stage was run, so no pipeline figure is reported; Experiments 002-007 cover those.

**Not measured, and not measurable here:**

- **Anything about CARLA.** No server ran. The catalogue's blueprints have not been confirmed
  to exist on any real server.
- **Whether the scripted poses match what a real simulator reports.** Against the fake they
  match to rounding, because the fake places actors exactly where it is told. A real server
  settles a spawned vehicle onto the road and may reject a placement; that gap is what the
  live tests exist to find.
- **Anything about perception accuracy.** Ground truth and scripted poses are now recorded
  beside every frame, which is the precondition for measuring it. Measuring it is Phase 11.


## Experiment 010 - First live CARLA run (Phases 9 and 10 against a real server)

**Date:** 2026-09-11

### What was executed

The first run of ADAPT-X against a **real CARLA server**. Everything in Experiments 008 and
009 that was marked "not executed - no server" was executed here, and several of the
assumptions the stand-in could not check turned out to be wrong. Fixes are listed below;
each one has a fake-backed regression test so the suite fails without a server if it is
ever undone.

**Setup:** CARLA server 0.9.16 on `127.0.0.1:2000`, map `Carla/Maps/Town10HD_Opt`
(155 spawn points), already running before the session started. Client: the `carla`
0.9.16 wheel, which is built for CPython 3.12 only - there is no wheel for the
project's primary Python 3.13.7, and PyPI resolves `carla` to 0.9.5, which does not
install either. A second virtual environment on **Python 3.12.10** was created for the
client; the primary 3.13 environment stays without CARLA and the default suite is
unchanged. Windows 11, 16 logical CPUs, single-threaded, no GPU used by ADAPT-X.

### Results

**`pytest -m carla` against the live server: 7 passed, 0 failed, 0 skipped** (88.6 s;
repeated after every fix, last run 89.1 s). Session opens and cleans up; frames are
labelled `simulation`; successive frames advance by exactly the timestep; ground truth is
recorded beside the frame; the `vehicle_approach` catalogue scenario completes; every
catalogue scenario completes; no vehicle, walker or sensor actor is left on the server
afterwards (checked independently with a fresh client: 0).

**`python -m adaptx.scenarios run <id>` for every catalogue scenario**, full Phase 2-8
chain per frame, ground truth recorded beside every frame and fed to no stage:

| Scenario | Frames | State | Sim frame ids | dt observed | Points / frame | Wall clock |
|---|---|---|---|---|---|---|
| stationary_vehicle | 40 | COMPLETED | contiguous | 0.0500 s every frame | 26,982-27,035 | 8.4 s |
| vehicle_approach | 60 | COMPLETED | contiguous | 0.0500 s every frame | 26,991-27,037 | 11.6 s |
| pedestrian_crossing | 120 | COMPLETED | contiguous | 0.0500 s every frame | 26,989-27,035 | 21.4 s |
| cyclist_crossing | 80 | COMPLETED | contiguous | 0.0500 s every frame | 26,988-27,038 | 15.1 s |

Per-frame pipeline time on this machine (sum of the eight measured stage durations,
processing through adaptive mapping), medians: 94.0 / 93.3 / 91.6 / 93.9 ms, minimum
86.4 ms, occasional frames to 185 ms. Wall clock per frame (0.17-0.21 s including session open and close) is therefore
roughly half simulator tick plus sensor transfer and half ADAPT-X. Detections per frame ran 8-16 and
tracks 8-18 in every scenario - **most of those are the map's static geometry**, not the
one or two scripted actors; whether the scripted actor is among them is exactly the
question Phase 11 will answer against the recorded ground truth, and nothing here answers
it. LiDAR: 32 channels, 560,000 points/s at 20 Hz, giving ~27,000 returns per 0.05 s
frame, which is 96-97% of the theoretical 28,000 after CARLA's own drop-off.

**Also confirmed live:** server and client versions match (0.9.16); all five blueprints
the catalogue uses exist on this server; the sensor measurement's frame id equals the
tick's frame id, so the queue-matching logic in `step()` is correct on a real server;
`world.tick()` in synchronous mode returns a frame id that increases by exactly one.

### What the live run found, and what was changed

1. **A freshly spawned actor reports the world origin until the server has ticked.**
   `actor.get_transform()` immediately after `try_spawn_actor()` returns `(0, 0, 0)` with
   zero yaw; after one `world.tick()` it returns the spawn transform (verified directly:
   requested `(-67.25, 27.96, 0.60) yaw 0.16`, read back before tick `(0, 0, 0) yaw 0`,
   after tick `(-67.25, 27.96, 0.59) yaw 0.16`). The session computed every ego-relative
   placement from that origin, so "45 m ahead" landed 70 m from the ego, off the road, and
   the target spawn was refused - the one live failure of the first attempt. The stand-in
   never caught it because its ego *is* at the origin. Fix: the session keeps the ego's
   spawn transform and uses it as the placement reference until the first tick (ADR-049);
   the fake now reports the origin until ticked, like the server, and a regression test
   spawns the ego off-origin.
2. **Spawn point 0 refused the ego every time** (3 of 3 attempts, while indices 1-11 all
   accepted). The session now walks the spawn points in order and takes the first that
   accepts - deterministic for a given map and world state - and records the index it used
   as `ego_spawn_index` on the status. Every live run here used index 1.
   **Corrected in Experiment 011:** the map was not refusing; an ADAPT-X ego and LiDAR left
   at spawn point 0 by this session's earlier killed run were occupying it, and they were
   also parked about 4 m from the ego of every run in this experiment. Destroying them
   made point 0 accept.
3. **`fps` was never measured on Python 3.12 on Windows.** `time.monotonic()` there ticks
   every 15.6 ms, so two frames recorded within one tick shared a timestamp and the
   elapsed time was zero. Replaced with `time.perf_counter()` in `MetricsService`; a test
   with a mocked coarse clock guards it. Invisible on 3.13, whose `monotonic()` is fine.
4. **The `carla` package has no `__version__`** in 0.9.16, so the run result recorded
   `"unknown"`. The session now asks the server (`get_server_version()`) at connect, exposes
   it as `server_version` on the status, and the result records that instead.
5. **A "no simulator" CLI test passed a real run and failed.** It ran the scenario against
   the default host and port; with a server there it completed successfully, which is the
   opposite of what the test asserts. It now targets a port nothing listens on.
6. **An import-isolation test failed once `carla` was importable**, because earlier tests
   in the same process had legitimately imported it. It now checks in a subprocess.
7. **Editable installs point at one checkout.** `pip install -e` from the main checkout
   left a worktree's tests importing the main checkout's (older) source. Not a code
   defect; recorded so nobody loses an afternoon to it again.

Not changed, noted: CARLA's client prints `INFO: streaming client: connection failed: An
operation was attempted on something that is not a socket` when a sensor is stopped and at
process exit - harmless, from inside the CARLA library, not ADAPT-X. `carla.CityObjectLabel`
has no `Vehicles` member in 0.9.16 (only met in an ad-hoc probe; nothing in the repository
uses it). The `[carla]` extra in `pyproject.toml` (`carla>=0.9.15`) cannot be satisfied from
PyPI and the wheel must be installed by hand; still a documented gap.

### Not measured, and not claimed

- **Accuracy of anything.** Ground truth and scripted poses are recorded beside every
  frame of every run above; no comparison was made. Phase 11.
- **CARLA's own performance.** Wall clock includes the server rendering and ray casting on
  this machine's CPU; it is not a property of ADAPT-X and would differ on any other host.
- **Behaviour with a moving ego, traffic, weather or any map other than Town10HD_Opt.**
- **Whether the scripted placement matches the pose the server reports** after physics
  settles the actor. Both are in the run record; the comparison is Phase 11.

## Experiment 011 - Phase 11 evaluation against CARLA ground truth (first measured correctness figures)

**Date:** 2026-09-11

> **Superseded in part by Experiment 012.** The placement defect found below (a placed
> walker falling; every actor and the ego settling under physics) was fixed the same day
> (ADR-054) and every scenario re-measured. The pedestrian figures here are void and the
> parked-car recall was an artefact of the car falling; the method, the environment
> findings and the repeatability analysis stand.

### Question

How well does the existing Phase 2-8 pipeline behave when measured against controlled
CARLA ground truth, and what does the paired fixed-versus-adaptive comparison show?
This is the first experiment in the project to put a number between what the pipeline
inferred and what the simulator knew. Every earlier entry said "unmeasured".

### Hypothesis (written before the runs)

The baselines will lose. Constant-velocity prediction will be poor beyond a second;
centroid association will swap identities in the cyclist's close pass; the heuristic
risk score will order proximity but not much else; the adaptive map will use fewer cells
than the 0.5 m fixed map and take longer to build, as Experiment 007 found on synthetic
scenes. Whether the adaptive map puts detail where the objects are was unknown.

### Setup

CARLA 0.9.16 server on `127.0.0.1:2000`, `Carla/Maps/Town10HD_Opt`, ego pinned to
**spawn point 1** (`ADAPTX_CARLA__EGO_SPAWN_INDEX=1`, see "environment findings"), Python
3.12.10 client environment, Windows 11, single-threaded. Every catalogue scenario was run
**twice** with `python -m adaptx.evaluation run <id>`, the run record written to disk, and
evaluated offline with `python -m adaptx.evaluation evaluate` from the primary Python 3.13
environment with no CARLA present. Defaults: gates 1 / 2 / 4 m (primary 2 m), proximity
band 20 m, alert level HIGH, refined level MEDIUM. All thresholds were fixed before the
first run. Seed 20260101 (the catalogue seed) for scenario and sensor. Pipeline settings:
process defaults (map +/-60 m, 0.5 m fixed, 10 m tiles, LOW/MEDIUM/HIGH/CRITICAL = 1.0 /
0.5 / 0.2 / 0.1 m).

Figures below are from repeat 1; repeat 2 differs where "repeatability" says.

### Environment findings that changed the setup (before any figure was read)

1. **Spawn point 0 was never refused by the map.** Experiment 010 attributed the refusal
   to Town10HD_Opt. Before this experiment the server held two stale actors - an ADAPT-X
   ego and its LiDAR, ids 56 and 57 - left at spawn point 0 by the run that hung and was
   killed during the first live attempt. Destroying them made spawn point 0 accept. ADR-049
   and Experiment 010 are corrected in place; the spawn-point walk stays, because that is
   exactly the situation it handles.
2. **From spawn point 0 the approach scenario is off-road.** With point 0 free, the ego
   spawned there and `vehicle_approach`'s target, 45 m ahead and 3.5 m left, lies 3.7 m off
   the road (waypoint probe); from point 1 it is on a Driving lane 1.1 m from the lane
   centre. Scenario placements are ego-relative, so the catalogue is only valid from one
   pose on this map. `CarlaSettings.ego_spawn_index` was added to pin it; all four
   scenarios ran from point 1, which is also where Experiment 010 ran.
3. **The LiDAR was unseeded.** `CarlaSettings.seed` existed ("seed for any simulator
   randomness") and was applied to nothing. Two unseeded runs differed in point count on
   every frame. The session now sets the sensor's `noise_seed` from it. Seeded, most frames
   are identical; see repeatability.

### Results - correctness (simulation evidence; not real-world figures)

Per scenario, primary gate 2 m unless stated. "Actor" is the scenario actor.

**Detection recall** (a detection within the gate of the actor, over eligible frames):

| Scenario / actor | 1 m | 2 m | 4 m | class agreement (2 m) | planar error at 2 m, mean |
|---|---|---|---|---|---|
| stationary_vehicle / parked (20 m) | 0/40 | 9/40 = 0.23 | 9/40 | 0.00 | 1.58 m |
| vehicle_approach / approaching (45 to 21 m) | 1/60 | 25/60 = 0.42 | 33/60 | 0.00 | 1.63 m |
| pedestrian_crossing / pedestrian (15 m) | 1/120 | 1/120 = **0.01** | 1/120 | 0.00 | 0.09 m |
| cyclist_crossing / both (25 m; 12 to 21 m) | 10/160 | 84/160 = 0.53 | 84/160 | 0.07 | 1.53 m |

**Tracking** (match rate, planar position error, velocity error against the finite-difference
reference, continuity):

| Scenario / actor | match 2 m | planar err mean / median | velocity err mean / median (n) | null velocity | coverage | ids | switches | fragments |
|---|---|---|---|---|---|---|---|---|
| stationary / parked | 16/40 = 0.40 | 1.57 / 1.58 m | 0.19 / 0.08 m/s (14) | 2 | 0.40 | [9, 33] | 1 | 2 |
| approach / approaching | 29/60 = 0.48 | 1.50 / 1.69 m | 2.81 / 0.82 m/s (25) | 4 | 0.48 | [1, 33, 34, 35] | 3 | 3 |
| pedestrian / pedestrian | 2/120 = 0.02 | 0.09 / 0.09 m | n/a (0) | 2 | 0.02 | [13] | 0 | 1 |
| cyclist / waiting_vehicle | - | - | - | - | 0.98 | [10, 35] | 1 | 2 |
| cyclist / cyclist | - | - | - | - | 0.25 | [9, 38, 39] | 2 | 3 |
| cyclist / both pooled | 98/160 = 0.61 | 1.42 / 1.66 m | 0.45 / 0.29 m/s (91) | 7 | | | | |

Unlabelled track-frames (tracks with no scenario-actor match, mostly the map's static
geometry): 362, 499, 926, 684. Not false positives; the record cannot label them.

**Trajectory prediction** (ADE/FDE over trajectories whose track was matched on the source
frame and had at least one future ground-truth frame; the `t+0` point excluded):

| Scenario | trajectories in record | evaluated | skipped (unmatched track / past end) | predictor skips | ADE mean / median | FDE mean / median | coverage |
|---|---|---|---|---|---|---|---|
| stationary | 333 | 14 | 319 / 0 | 45 insufficient_velocity | 1.50 / 1.55 m | 1.48 / 1.53 m | 0.40 |
| approach | 479 | 22 | 454 / 3 | 49 | 3.37 / 1.94 m | 4.81 / 1.95 m | 0.31 |
| pedestrian | 879 | **0** | 879 / 0 | 49 | n/a | n/a | n/a |
| cyclist | 726 | 86 | 635 / 5 | 56 | 2.11 / 1.66 m | 2.83 / 1.66 m | 0.63 |

Error by horizon, cyclist scenario (n falls with horizon because the run ends): t+0.25 s
1.48 m, t+0.5 1.56, t+1.0 1.89, t+1.5 2.35, t+2.0 3.10, t+2.5 3.44, t+3.0 5.10 m (n=23).
Approach scenario: t+0.25 s 1.97 m, t+0.5 2.59, t+1.0 5.49 (n=12), t+1.5 11.73 m (n=5).

**Risk against proximity** (band 20 m, alert at or above HIGH):

| Scenario / actor | event frames (matched) | alert recall | lead time | early alerts | UNKNOWN | ordering concordance (pairs) | levels on matched frames |
|---|---|---|---|---|---|---|---|
| stationary / parked | 0 (0) | n/a | n/a | 2 | 0 | 0.855 (69) | medium 14, high 2 |
| approach / approaching | 0 (0) | n/a | n/a | 5 | 0 | 0.802 (398) | low 8, medium 16, high 5 |
| pedestrian / pedestrian | 120 (2) | 1.00 (of 2) | -0.05 s | 0 | 0 | n/a (0) | high 2 |
| cyclist / waiting_vehicle | 0 (0) | n/a | n/a | 0 | 0 | 0.396 (2439) | medium 78 |
| cyclist / cyclist | 63 (20) | 0.25 | 0.00 s | 0 | 0 | 0.808 (167) | medium 15, high 5 |

The 20 m band, chosen before the runs, sits just inside the closest approach of three
scenarios (the parked vehicle is at 20.0 m, the approaching one ends at 21.4 m, the waiting
vehicle is at 25 m), so three actors produced no event. **A second evaluation at 25 m was
run afterwards and is reported as what it is - a post-hoc choice:** approaching, 8 event
frames, alert recall 0.625, lead time -0.05 s; parked, 40 event frames (16 matched), alert
recall 0.125; waiting_vehicle, 32 event frames, alert recall 0.00. Alerting unlabelled
track-frames: 41, 63, 121, 81. No collision occurs in any scenario; no collision figure exists.

### Results - adaptive resolution (paired against the fixed 0.5 m map, same frames)

| Scenario | fixed cells | adaptive cells mean (min-max) | cell and byte ratio mean | build time ratio, mapping only / with controller (median) | area-weighted res. | actor-tile res. mean | other-tile res. mean |
|---|---|---|---|---|---|---|---|
| stationary | 57,600 | 31,785 (26,700-48,600) | 0.55 | 5.0 / 8.5 | 0.92 m | 0.58 m | 0.92 m |
| approach | 57,600 | 30,245 (26,700-44,400) | 0.53 | 5.1 / 8.2 | 0.93 m | 0.49 m | 0.93 m |
| pedestrian | 57,600 | 27,605 (26,700-39,600) | 0.48 | 4.9 / 7.9 | 0.94 m | **0.97 m** | 0.94 m |
| cyclist | 57,600 | 32,400 (27,300-62,400) | 0.56 | 5.1 / 8.4 | 0.92 m | 0.41 m | 0.93 m |

Actor-tile level distribution: stationary low 13 / medium 16 / high 11; approach low 3 /
medium 49 / high 8; pedestrian **low 116** / high 4; cyclist low 2 / medium 105 / high 53.
Actor-tile cell size by matched risk level: HIGH 0.20 m in every scenario (n = 2, 5, 2, 5);
MEDIUM 0.35-0.44 m; unmatched frames 0.49-0.99 m.

Refinement lead (frames; positive = the tile reached MEDIUM before the actor arrived):
stationary 2 entries, 0 and +25; approach 3 entries, -1, +14, +38; pedestrian 2 entries,
-1 and +77; cyclist 4 distinct (actor, tile) entries, 0, 0, +26, +50. No entry was never
refined.

Churn: changed tiles per frame mean 1.75 / 1.30 / 0.55 / 1.09; frames with any change
18/40, 21/60, 16/120, 24/80; transitions 70 / 78 / 66 / 87 over 33-37 of 144 tiles;
reversals (back to the level just left within 3 frames) 1 / 4 / 1 / 3; hysteresis holds
304 / 371 / 924 / 573 and dwell holds 107 / 97 / 88 / 115 decisions. No frame over budget.

Map workload: fixed occupancy ratio 0.052 on every scenario, adaptive 0.15-0.16; fixed grid
1,843,200 bytes, adaptive 0.88-1.04 MB. **Occupancy accuracy not evaluated** (no reference).

### Results - resource (this machine, not a real-time claim)

Pipeline per frame, median (p95): 162.6 (215.5), 150.9 (195.0), 147.5 (189.1), 150.8
(204.3) ms. Detection dominates at 96-105 ms median (clustering 85 ms of it), then adaptive
mapping 22 ms, controller 13-15 ms, processing 7-8 ms, fixed mapping 4.5 ms, prediction
1.5-2 ms, risk and tracking about 1 ms. Experiment 010 measured detection at 42 ms median
on the same scene; the cause of the difference was not identified and is not claimed.
Peak process memory: not sampled.

### Repeatability

Two seeded runs per scenario, compared with `python -m adaptx.evaluation compare`
(timings and session ids excluded):

| Scenario | frames | point count differs | max difference | detections differ | tracks differ | adaptive cells differ |
|---|---|---|---|---|---|---|
| stationary | 40 | 12 frames | 11 pts of ~27,000 | 7 frames | 9 | 12 |
| approach | 60 | 42 | 16 | 7 | 13 | 33 |
| pedestrian | 120 | 40 | 9 | 0 | 0 | 0 |
| cyclist | 80 | 55 | 17 | 6 | 2 | 16 |

**No live scenario is bit-repeatable.** With the sensor seeded, 30-70 % of frames return an
identical point count and the rest differ by at most 17 points in 27,000; unseeded, every
frame differed. The pipeline itself is deterministic - two fake-simulator runs evaluate
identically, and that is a test - so the divergence is the sensor's, and the geometric
detector amplifies a dozen points into a different cluster boundary, a different track and
a different tile level. The deterministic-content comparison therefore fails on every live
scenario, by the amounts above. The tracking, prediction and risk sections agreed to the
digit on the pedestrian scenario and differed by one to three matched frames on the others.

### Interpretation (written after the runs)

- **The consistent 1.5-1.7 m planar offset** on vehicles at every gate, with recall at the
  1 m gate near zero, is compatible with the detection centroid lying on the visible face of
  the vehicle while the ground-truth position is the mesh origin - but that is a hypothesis
  this record cannot test, and the number stands as the position error of the system as
  built. The 0.09 m on the one matched pedestrian frame is consistent with it.
- **The pedestrian is not detected.** One frame in 120. The record shows why the scenario
  cannot say whether that is the detector's fault: the walker's ground-truth height reached
  -1.06 m and was below the road on 45 of 120 frames, with a vertical speed of -31 m/s -
  a placed walker keeps its physics velocity between placements and falls. The vehicles
  settle (z at or above -0.09 m); the walker does not. **The pedestrian figures are
  confounded by a Phase 10 placement defect** and should not be read as a detector result
  until it is fixed. Not fixed here: Phase 11 measures.
- **The detector never labels the Audi a vehicle** (class agreement 0.00 on 54 matched
  frames across three scenarios; the cyclist scenario's 0.07 is the cyclist). The classes
  it produced over the approach run were unknown 319, obstacle 151, vehicle 1.
- **Identity does not survive**: 1-3 switches per moving actor, coverage 0.25-0.48, and the
  waiting vehicle - the easiest case - still switched once.
- **Prediction error grows with horizon as a constant-velocity model predicts it would**,
  and in the approach scenario grows faster (11.7 m at 1.5 s) because the velocity estimate
  itself was wrong on some frames (velocity error mean 2.8 m/s against a median of 0.8:
  frames that measured a standstill for an object closing at 8 m/s).
- **The risk score orders proximity** for the two moving actors (0.80, 0.81) and does not
  for the stationary one (0.40, where the only distance variation is jitter). At the
  pre-registered 20 m band nothing was inside it long enough to say more.
- **The adaptive map uses 0.48-0.56 of the fixed map's cells and 5x its build time** (8x
  with the controller), as Experiment 007 found. **It does put finer cells under the actor
  than elsewhere** - 0.41-0.58 m against 0.92-0.93 m - in the three scenarios where the
  actor was perceived, and refined ahead of arrival in every entry but two (one frame late
  each). In the pedestrian scenario it did not, because nothing perceived the pedestrian:
  the controller can only follow what upstream gives it.
- Churn is low (a change on a quarter to a half of frames, 1-4 reversals) with the
  stabiliser reporting hundreds of holds; whether the holds were right is not measured.

### Limitations

Simulation only; four scenarios; one map, one pose, one seed; the ego never moves; no
occupancy reference; ground truth excludes static geometry, so no precision; peak memory
not sampled; timings from one machine; runs are near- but not bit-repeatable. Nothing here
is a safety, collision-probability or real-world claim.

## Experiment 012 - Placement without physics: re-measurement of Experiment 011

**Date:** 2026-09-11

### Question

Experiment 011 found two defects in the test environment, not the system under test: a
placed walker fell through the road, and live runs were not repeatable. Does making placed
actors *actually* placed (ADR-054) remove them, and what do the Phase 11 figures look like
on a scene that stands still?

### Hypothesis (written before the runs)

Disabling physics on placed actors and grounding the ego will hold every actor where the
script puts it and remove the falling walker and the ego's half-second settling transient.
Repeatability should improve; whether it becomes exact was unknown. The pedestrian figures
of Experiment 011 will change; the vehicle figures were expected to change little.

### What changed (ADR-054)

- `spawn_ahead_of_ego` switches physics off on every scenario actor and places it so the
  **bottom of its bounding box** sits `up_m` above the ego's ground plane; `Placement.up_m`
  now defaults to 0.0 (was 0.5, "spawn clearance"). A car's origin lands at road level, a
  walker's 0.93 m up - both with their feet on the road.
- The ego's physics is switched off and it is set down at the road surface under its spawn
  point (a spawn point sits 0.6 m above the road; with physics on the ego fell that far
  over the first ten frames of every run in Experiment 011, and the LiDAR fell with it).

Same server, map, spawn point 1, seeds, pipeline defaults and evaluation thresholds as
Experiment 011. Two repeats per scenario.

### Results - the defects

| | Experiment 011 | Experiment 012 |
|---|---|---|
| walker height rel. ego, over the run | -1.06 to +0.92 m, 45/120 frames below the road | **0.93 m on every frame** |
| vehicle height rel. ego | -0.09 to +0.50 m (falling, caught by the road each tick) | **0.00 m on every frame** |
| ego world z over the run | settling 0.6 m in the first 10 frames | **0.000 on every frame** |
| stationary_vehicle repeat | 12/40 frames differ in point count | **0/40 - bit-repeatable; `compare` says REPEATABLE** |
| vehicle_approach repeat | 42/60 frames, max 16 pts | 23/60, max 5 |
| pedestrian_crossing repeat | 40/120, max 9 | 69/120, max 8 |
| cyclist_crossing repeat | 55/80, max 17 | 43/80, max 8 |

A scene in which nothing moves is now exactly repeatable. Scenes with moving placed actors
still differ by at most 8 points in 27,000 on a third to half of frames, with detections
identical in three of four scenarios. The residual is not explained; it is consistent with
the server's own handling of a transform set between ticks, and is not the placement code.

### Results - pipeline as configured by the process defaults (ground segmentation OFF)

Pre-registered configuration, same as Experiment 011.

| Scenario / actor | det. recall 2 m | trk. match 2 m | planar err. mean | coverage | id switches | ADE mean | notes |
|---|---|---|---|---|---|---|---|
| stationary / parked (20 m) | **0/40** | 0/40 | n/a | 0.00 | - | n/a | Experiment 011's 9/40 were the falling car's transient |
| approach / approaching | 26/60 | 30/60 | 1.59 m | 0.50 | 2 | 3.23 m (11.7 m at 1.5 s) | |
| pedestrian / pedestrian | **0/120** | 0/120 | n/a | 0.00 | - | n/a | standing on the road now, still unseen |
| cyclist / waiting_vehicle | 87/160 pooled | 93/160 pooled | 1.49 m | **1.00** | **0** | 1.65 m | |
| cyclist / cyclist | | | | 0.16 | 0 | | |

The parked car returns about 26 LiDAR points per sweep at 20 m (measured directly with a
sensor probe: 48 points within 3 m of its position against 22 without it). The detector
saw, at the car's position, a rejected two-point cluster: the rest of its points were
clustered with the ground, because **the process defaults disable ground segmentation**
(`ADAPTX_LIDAR__GROUND_ENABLED=false`, ADR-012 made every Phase 2B stage opt-in) and the
grid clusterer joins a low car to the road it stands on.

Adaptive: cells 0.46-0.49 of fixed, build time 5x (8x with controller), actor tile
0.47-0.56 m against 0.94 m elsewhere where the actor was perceived, 1.00 m (never refined)
where it was not. Churn fell to 0-13 transitions per run with 0-3 reversals: without the
settling transient the allocation is nearly static. Pipeline 140-142 ms per frame median,
detection 89-92 ms of it.

### Results - disclosed configuration variation: ground segmentation ON

**Chosen after reading the result above**, to test the explanation; it is not the
pre-registered configuration and is reported as a variation. One environment variable,
`ADAPTX_LIDAR__GROUND_ENABLED=true`, nothing else changed; two repeats.

| Scenario / actor | det. recall 1 m / 2 m | trk. match 2 m | planar err. mean / median | vel. err. median | coverage | id switches | ADE mean / med | ADE at 3 s |
|---|---|---|---|---|---|---|---|---|
| stationary / parked | 0/40 / **40/40** | 40/40 | 1.70 / 1.70 m | 0.00 m/s | 1.00 | 0 | 1.70 / 1.70 m | - |
| approach / approaching | 0/60 / **9/60** | 13/60 | 1.46 / 1.62 m | 1.33 m/s | 0.22 | 0 | 1.07 / 1.11 m | - |
| pedestrian / pedestrian | **120/120** / 120/120 | 120/120 | **0.10 / 0.10 m** | 0.10 m/s | 1.00 | 0 | 0.66 / 0.52 m | 1.87 m |
| cyclist / both pooled | 45/160 / 124/160 | 128/160 | 1.03 / 1.50 m | 0.00 m/s | veh 1.00, cyc 0.60 | 0, 0 | 2.06 / 1.50 m | 5.21 m |

- The pedestrian is seen on every frame at 0.10 m, tracked without a switch, and predicted
  to 1.9 m at 3 s. The parked car is seen on every frame - at a **constant 1.70 m** planar
  offset, so never within the 1 m gate: the centroid of the returns is not where the mesh
  origin is, and this configuration makes that offset exact rather than noisy.
- The approaching car is seen **less** with ground segmentation on (9/60 against 26/60): a
  low car at 25-45 m apparently loses its returns to the ground stage. Not investigated.
- No identity switch in any scenario. The classifier still never labels a vehicle a vehicle
  (agreement 0.00-0.05; the cyclist scenario's 0.13 at 1 m is the cyclist).
- Risk: ordering concordance 0.95-0.98 for the moving actors, 1.00 for the parked car (39
  pairs), 0.69 for the pedestrian; the parked and waiting vehicles never entered the
  pre-registered 20 m band; alert recall where an event existed was 0.008-0.021.
- Adaptive: cells **0.65-0.69** of fixed (more than with ground on the map, because the
  perceived objects now drive refinement), build time 7-8x (15-17x with controller), actor
  tile 0.31-0.48 m against 0.90 m elsewhere, refined before or on arrival in every entry
  but one (-9 frames), 9-11 transitions per run, no reversal. Pipeline **80-84 ms** per
  frame median: ground segmentation costs 12 ms and saves 70 ms of clustering.

### Interpretation

- The Experiment 011 defects are closed by ADR-054 and the pedestrian scenario is now a
  valid measurement. Its Experiment 011 figures are void.
- **Every correctness figure in this project now depends on one configuration switch that
  the process defaults leave off.** With it off, a car parked 20 m ahead and a pedestrian
  15 m ahead are never detected, and the earlier "recall" on the parked car was an artefact
  of the car falling. With it on, both are seen on every frame. Which default is right is a
  Phase 2/3 decision with its own experiment, not something this phase settles; the
  evaluation reports the configuration it ran under, and both are recorded here.
- The 1.70 m offset between detection centroid and actor origin on a vehicle is systematic,
  not noise. Any future accuracy claim at a 1 m gate would need either a reference point on
  the visible face or an oriented box, and the record cannot say which.
- Live runs are now bit-repeatable for a static scene and near-repeatable otherwise.

### Limitations

As Experiment 011, plus: the ground-on figures are a post-hoc variation; nothing was tuned
inside any stage; the residual non-repeatability of moving actors is unexplained; one map,
one pose, one seed; simulation only.

## Experiment 013 - Browser cost of the dashboard: load, report parse, frame fetch, draw

**Date:** 2026-09-11

### Question

Phase 12 renders on Canvas 2D with no framework and serves recorded runs one frame at a
time. Is that fast enough to look at a 6000-point scene and to step through a 42 MB run
without the tab freezing, and where does the time go? This measures the **display** cost
only; it says nothing about perception, and it is one machine, one browser.

### Setup

- Backend: `uvicorn` in-process, Python 3.13.7, FastAPI 0.141.1, on `127.0.0.1:8000`, this
  worktree, `reports/` holding the six Experiment 012 reports (24-32 KB each) and `runs/`
  holding `cyclist_crossing_ground_on.json` (41.7 MB, 80 frames) and
  `stationary_vehicle_ground_on.json` (20.4 MB, 40 frames).
- Browser: the Claude desktop app's embedded Chromium 152 on Windows 11, viewport emulated
  at 1536x1024, `devicePixelRatio` 1.25, 16 logical cores. Timings from `performance.now()`
  and the Navigation/Resource Timing APIs, taken in the page's own console; 30 draws per
  sample, min / median / max reported.
- Scene for the draw benchmark: a synthetic road plane with two vehicles and a pedestrian
  (16,132 points, `tests/fixtures/scenes`) pushed through `POST /api/v1/lidar/adaptive-map`
  three times; the published snapshot carried a stride-3 sample of **5,378 points**, 1
  object and 144 tiles. Playback scene: frames of the cyclist run (10-11 tracks, 144 tiles,
  no point cloud - records carry none).

### Results

| Measure | Value |
|---|---|
| page `DOMContentLoaded` / `load` | 569 / 575 ms (20 modules, 31 KB transferred, `no-cache`) |
| report fetch + text, 21 KB / 16 KB | 18.4 / 17.0 ms; `JSON.parse` 0.7 / 0.3 ms |
| compare endpoint (two reports, 18 KB) | 26.5 ms |
| run summary, **cold** (backend parses the record once) | **3,013 ms** for 41.7 MB; 1,709 ms for 20.4 MB |
| run summary, warm (cached in the backend) | 17.4 ms |
| one run frame (208-256 KB, 10-11 tracks) | fetch 27-52 ms (median 32.7), parse 1.8-3.0 ms |
| `seek()` to a frame already prefetched | 2.2-4.1 ms |
| `seek()` to an uncached frame, nothing in flight | 38.5-42.7 ms |
| `seek()` jumping 10 frames while 8 prefetches are in flight | 185-947 ms (median 208) |
| `selectRun` including first frame (warm summary) | 68.9 ms |
| perspective draw, live scene, 5,378 points, 950x841 px | 6.2 / **9.6** / 21.6 ms |
| top-down draw, same scene, 950x826 px | 5.4 / **8.3** / 9.5 ms |
| perspective draw, playback scene, 10 tracks, no points | 0.8 / 1.5 / 5.0 ms |
| top-down draw, playback scene | 0.5 / 0.9 / 1.8 ms |

### Interpretation

- A 6000-point sample draws in under 10 ms median on Canvas 2D, so a 20 Hz scenario stream
  (50 ms per frame) has headroom on this machine; the point cap, not the renderer, is what
  bounds the cost, and the cap is a setting. WebGL is not earned by these numbers.
- Sequential playback is bounded by the prefetch: a cached seek is ~3 ms and the 8-frame
  lookahead keeps up with the 50 ms timestep. The cost that matters is a **random seek while
  prefetches are queued**: the backend's handlers are synchronous, so a jump waits behind
  up to eight 30 ms frame requests - about 200 ms, and up to 0.9 s when the summary was
  still being built. That is the price of serving from one process and is recorded, not
  hidden.
- The one multi-second cost is the backend parsing a 42 MB record the first time it is
  chosen. It happens once per process per run (LRU of two) and the browser is not blocked
  meanwhile - the rail shows the run as loading.
- The page loads in ~0.6 s with no bundler; the 20 separate module requests are the cost of
  having no build step and are acceptable on localhost, which is the only place this is
  served.

### Limitations

One machine, one browser build, one viewport, localhost only; no GPU measurement (the
dashboard reports GPU as NOT MEASURED because nothing measures it); the draw benchmark
re-draws a static scene and excludes the DOM tables the views also rebuild on each frame;
the embedded Chromium may differ from a user's browser; nothing here is a frame-rate claim
for the perception pipeline, whose measured per-frame cost is in Experiment 012.

## Experiment 014 - The live loop on CARLA 0.9.16: does the pipeline stop the car?

**Date:** 2026-09-12

### Question

With a driven ego, real LiDAR, the unchanged Phase 2-8 chain and the baseline speed
governor (ADR-056), does the ego slow and hold for an obstacle the pipeline detects, resume
when it leaves, and what does the loop cost per frame? Nothing here is an accuracy figure;
Experiments 011/012 hold those.

### Setup

CARLA 0.9.16, Town10HD_Opt, spawn point 1, synchronous 0.05 s, LiDAR as configured
(32 channels, 100 m, 560k pts/s, 20 Hz, seeded), `ADAPTX_LIDAR__GROUND_ENABLED=true`
(the ground-on variation of Experiment 012; the process default is off), control
defaults (`max 8 / medium 5 / high 2 m/s`, safe 8 m, emergency 5 m, dwell 10 frames,
hold throttle 0.22), collision sensor and 320x180 camera attached. Python 3.12
(`.venv312`), no browser attached unless stated; the numbers were read from the
published snapshots by a probe script.

### Results

| Scenario / seed | frames · sim s · wall s | loop ms med / p90 / max | pipeline ms med / p90 / max | step ms med | sim/wall | max speed | first hold (sim s, nearest) | min nearest while holding | collisions |
|---|---|---|---|---|---|---|---|---|---|
| static_obstacle / 42 | 757 · 37.9 · 130 | 160 / 187 / 266 | 126 / 153 / 228 | 32 | 0.31 | 5.05 m/s | 11.55 s, 7.96 m | 6.78 m | 0 |
| static_obstacle / 43 | 375 · 18.8 · 60 | 150 / 174 / 211 | 115 / 140 / 173 | 32 | 0.33 | - | - | - | 0 |
| mixed_obstacles / 42 (6 TM vehicles) | 554 · 27.7 · 104 | 162 / 191 / 225 | 121 / 150 / 184 | 38 | 0.31 | 5.08 m/s | 11.8 s, 7.9 m | 7.63 m | 0 |
| pedestrian_crossing / 42 | 404 · 20.2 · 70 | 164 / 191 / 279 | 131 / 157 / 241 | 32 | 0.30 | 5.14 m/s | 9.5 s, 14.3 m (predicted crossing) | 6.27 m | 0 |

Per-stage medians inside the pipeline (static_obstacle, 40 frames): processing 20 ms
(ground segmentation on), detection 17, tracking 0.5, prediction 2, fixed mapping 2.3,
risk 1.1, resolution controller 15.5, adaptive mapping 15.2 - about 74 ms of stage time
inside a 90-125 ms `pipeline_ms`; the remainder is result-contract construction. Control
1 ms; snapshot build + publish 6.5 ms; tick + LiDAR wait 32 ms (38 ms with traffic).
Voxel downsampling did not help (processing 32 ms, detection unchanged).

**The obstacle-stop demo, as it happened (static_obstacle / 42, session time):** parked
car appears 45 m ahead at 1.0 s; ego accelerates to 5.0 m/s; the car is a `vehicle` or
`obstacle` track from ~35 m with identity switches every few frames; SLOWING at HIGH from
~16 m (target 2 m/s); HOLDING at 7.96 m (11.55 s); STOPPED at 7.65 m; obstacle removed at
23.36 s; RESUMING at 23.65 s; cruising again by 26 s. The ego never contacted the car.
After 27 s the ego reached road geometry ahead that the straight corridor rule treats as
in-path and held/resumed repeatedly - the honest behaviour of a straight-corridor
governor on a bending road, not a scenario event.

**With the browser attached** (the dashboard on the same machine) the loop median rose to
~190-210 ms (0.24-0.27x): snapshot serialisation for the WebSocket runs in the same
process and competes for the GIL.

### Findings that changed the design (recorded in ADR-056)

- Scene-maximum risk pinned the ego: lamp posts 5.5 m and 9 m beside the road were HIGH
  / CRITICAL on proximity, so the governor now keys the speed on in-path objects and
  shows the scene level beside it.
- A pure proportional throttle stalled at 1.62 m/s under a 2.0 m/s target; a 0.22 hold
  throttle fixed it.
- Destroying Traffic-Manager vehicles while the world was still synchronous aborted the
  client process (`0xC0000409`) and leaked every actor; the CARLA examples' shutdown order
  (world asynchronous, TM asynchronous, autopilot off, one asynchronous frame, batch
  destroy) is now the session's order, verified twice in a row and by the live suite.
- Fresh clients see a stale actor list on a server left in synchronous mode; cleanup
  scripts must switch it asynchronous and wait a frame first.
- Velocities are ego-relative: with the ego at 5 m/s every static object reports
  ~-5 m/s along x, so closing-speed rises everywhere (no ego-motion compensation exists).

### Limitations

One machine, one map, one spawn point, three seeds; simulation only; nothing tuned; no
claim of real-time (0.3x measured), no claim of collision-free behaviour (0 in these runs,
counted by the sensor); the classifier still labels poles "pedestrian" and cars
"obstacle"; the dashboard's own cost is in Experiment 013.
