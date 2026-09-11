# Benchmarking

## Objective

Determine whether adaptive perception provides a measurable computational advantage while preserving perception quality. ADAPT-X must not be judged only by how intelligent it appears.

## Comparison

Run the same scenario and sensor configuration through:

1. Fixed-resolution perception.
2. ADAPT-X adaptive-resolution perception.

Keep seeds, timing, inputs, warm-up behavior, and measurement windows comparable.

## Performance Metrics

- FPS
- processing latency
- CPU utilization
- GPU utilization
- memory usage

## Workload Metrics

- processed point count
- active cell count
- average resolution
- percentage of high-resolution area
- resolution-change frequency

## Perception Metrics

- detection accuracy
- tracking metrics
- prediction metrics
- critical-object detection
- missed detections
- reaction time
- predicted-conflict detection

## Reporting

Record hardware, software versions, scenario configuration, sample count, warm-up period, aggregation method, and raw or reproducible result files. Never manufacture improvements or hide failed runs.

## Status (Phase 11)

`adaptx.benchmark` measures **speed and workload** on synthetic data. `adaptx.evaluation`
(Phase 11) measures **perception against simulator ground truth** on recorded scenario
runs, and pairs the fixed and adaptive maps within one run. Metric definitions, the
ground-truth boundary and what is deliberately not measured are in
[`../EVALUATION.md`](../EVALUATION.md); the first figures are Experiment 011. Of the
metrics listed above, detection accuracy is reported as gated recall with the gate beside
it, tracking as match rate, error and continuity, prediction as ADE/FDE, and "missed
detections" as unmatched eligible actor-frames; "critical-object detection", "reaction
time" and "predicted-conflict detection" are not yet defined against a label the scenarios
can supply, and no precision or false-positive figure exists while static scene geometry is
unlabelled.
