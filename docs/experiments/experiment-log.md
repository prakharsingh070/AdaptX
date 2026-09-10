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
