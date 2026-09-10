# ADAPT-X API

Base URL (development): `http://localhost:8000`
Versioned prefix: `/api/v1`
Interactive documentation: `/docs` (Swagger UI), `/redoc`, `/openapi.json`

**This document describes only what is implemented.** Detection, tracking, trajectory
prediction, 2.5D mapping and object-level risk exist as deterministic baselines. Endpoints
for the **adaptive** map and the spatial **risk map** — a per-cell risk field, which Phase 7
does not produce — plus events, scenarios and benchmarking are sketched in
[`knowledge-base/16_api-contract.md`](knowledge-base/16_api-contract.md) and do not exist
yet.

---

## Conventions

| Topic | Rule |
|---|---|
| Timestamps | ISO 8601, timezone-aware, UTC (`2026-09-09T18:04:34.325074Z`) |
| Units | metres, m/s, m/s², radians, seconds — unless the field name says otherwise (`_ms`, `_mb`, `_percent`) |
| Coordinate frame | Every spatial payload names its frame: `lidar`, `ego`, `world`, `map` |
| Provenance | Every payload that could be non-sensor data carries `source`: `live_sensor`, `simulation`, `replay`, `synthetic_test`, `unavailable` |
| Risk scale | Normalised `[0, 1]` (ADR-006). Dashboards may render 0–100 |
| Confidence | `[0, 1]` |
| Schema version | `schema_version` on every versioned model (currently `"1.0"`) |
| Unknown fields | Rejected — requests with extra fields return 422 |
| Unmeasured values | `null`, never an estimate, with the reason listed in `unavailable` |

### Error format

Domain errors return a single envelope:

```json
{
  "error": {
    "code": "invalid_point_cloud",
    "message": "frame has 1001 points, maximum is 1000",
    "details": { "point_count": 1001, "max_points": 1000 }
  }
}
```

| Code | HTTP | Meaning |
|---|---|---|
| `invalid_point_cloud` | 422 | Malformed or out-of-limits point cloud |
| `validation_error` | 422 | Input violates a data contract |
| `simulator_unavailable` | 503 | CARLA not installed, disabled or unreachable |
| `module_not_ready` | 501 | The subsystem exists as an interface only |
| `configuration_error` | 500 | Invalid configuration |

Request-shape errors caught by FastAPI before reaching the handler (missing field, wrong
type) return FastAPI's standard 422 `{"detail": [...]}` body.

---

## `GET /health`

Liveness probe. Says nothing about subsystem readiness — use `/api/v1/system/status` for
that.

**200 OK**

```json
{
  "status": "ok",
  "version": "0.1.0",
  "name": "ADAPT-X",
  "timestamp": "2026-09-09T18:04:34.325074Z"
}
```

---

## `GET /api/v1/system/status`

Aggregated backend status: overall state, the LiDAR and CARLA channels, and every
subsystem's readiness **and** implementation status.

**200 OK** (abridged — the `components` array lists all nine subsystems)

```json
{
  "schema_version": "1.0",
  "timestamp": "2026-09-09T18:04:34.374661Z",
  "state": "RUNNING",
  "version": "0.1.0",
  "environment": "development",
  "uptime_s": 13.45,
  "lidar": {
    "status": "DISCONNECTED",
    "source": "unavailable",
    "last_frame_id": null,
    "last_frame_age_s": null,
    "frames_received": 0,
    "detail": "no point-cloud frame has been received"
  },
  "carla": {
    "status": "DISCONNECTED",
    "enabled": false,
    "client_available": false,
    "is_mock": false,
    "host": "localhost",
    "port": 2000,
    "world": null,
    "detail": "CARLA is disabled"
  },
  "components": [
    {
      "name": "lidar_ingest",
      "readiness": "READY",
      "implementation": "PARTIAL",
      "detail": "frame acceptance, validation, bounds and metadata only; the processing stages are reported separately as lidar_preprocessing",
      "phase": 1,
      "required": true
    },
    {
      "name": "perception",
      "readiness": "NOT_READY",
      "implementation": "PLANNED",
      "detail": "object detection contract only; no detector implemented",
      "phase": 3,
      "required": false
    }
  ]
}
```

**Enumerations**

| Field | Values |
|---|---|
| `state` | `RUNNING`, `DEGRADED`, `ERROR` |
| `lidar.status` | `CONNECTED`, `DISCONNECTED`, `SIMULATED` |
| `carla.status` | `CONNECTED`, `DISCONNECTED` |
| `components[].readiness` | `READY`, `NOT_READY` |
| `components[].implementation` | `IMPLEMENTED`, `PARTIAL`, `PLANNED`, `MOCK` |

`SIMULATED` is reported whenever the most recent frame was labelled `simulation`, `replay`
or `synthetic_test`. `state` is `DEGRADED` when CARLA is enabled but not connected, or when
the LiDAR feed goes stale after having delivered frames.

---

## `GET /api/v1/system/metrics`

Metrics measured by the running process.

**200 OK**

```json
{
  "schema_version": "1.0",
  "timestamp": "2026-09-09T18:04:53.122205Z",
  "fps": null,
  "latency_ms": 0.1915,
  "processing_time_ms": 0.1915,
  "cpu_percent": 0.5,
  "gpu_percent": null,
  "memory_mb": 61.41,
  "point_count": 3,
  "sample_count": 1,
  "unavailable": [
    "gpu_percent: not measured (no GPU monitoring dependency in Phase 1)",
    "fps: needs at least two ingested frames"
  ]
}
```

| Field | Definition |
|---|---|
| `fps` | Frames ingested per second over the rolling window. `null` until two frames exist |
| `latency_ms` | Mean **ingest processing** time over the window. Not sensor-to-output latency — there is no pipeline to measure yet |
| `processing_time_ms` | Ingest processing time of the most recent frame |
| `cpu_percent` / `memory_mb` | This process, measured with `psutil` |
| `gpu_percent` | Always `null`: no GPU monitoring dependency is installed |
| `point_count` | Points in the most recent frame |
| `sample_count` | Frames currently in the measurement window |
| `unavailable` | Every metric that could not be measured, and why |

No value here is estimated, extrapolated or defaulted.

---

## `GET /api/v1/carla/status`

**200 OK**

```json
{
  "status": "DISCONNECTED",
  "enabled": false,
  "client_available": false,
  "is_mock": false,
  "host": "localhost",
  "port": 2000,
  "world": null,
  "detail": "CARLA is disabled"
}
```

`client_available` reports whether the optional `carla` Python package is importable.
`is_mock` is `true` only when `ADAPTX_CARLA__USE_MOCK=true`, in which case nothing the
simulator returns is sensor data. A disabled or unreachable simulator is a 200 response
describing `DISCONNECTED`, not an error.

---

## `POST /api/v1/lidar/frame`

Submit a point-cloud frame. By default the frame is validated, counted and summarised and
nothing else. With `preprocess: true` it also runs the preprocessing pipeline described
below. It is **never** detected on, tracked or mapped — those modules do not exist.

**Request**

```json
{
  "frame_id": 1,
  "sensor_id": "verify_lidar",
  "source": "synthetic_test",
  "points": [[0.0, 0.0, 0.0], [1.5, -2.0, 0.3], [10.0, 4.0, -1.0]],
  "timestamp": "2026-09-09T18:04:44.000000Z",
  "coordinate_frame": "lidar"
}
```

| Field | Required | Notes |
|---|---|---|
| `frame_id` | yes | ≥ 0, monotonic counter from the source |
| `sensor_id` | yes | non-empty |
| `source` | **yes** | No default — the caller must declare provenance so synthetic data can never be recorded as sensor data |
| `points` | yes | Rows of `[x, y, z]` or `[x, y, z, intensity]`, in metres. Hard request cap 1,000,000 rows; the effective limit is `ADAPTX_LIDAR__MAX_POINTS` |
| `timestamp` | no | Timezone-aware UTC; defaults to now. Naive datetimes are rejected |
| `coordinate_frame` | no | Defaults to `lidar` |
| `preprocess` | no | Defaults to `false`. When `true`, runs the preprocessing pipeline before ingest |

Coordinates are in the ADAPT-X convention (ADR-009): right-handed, origin at the sensor,
**+x forward, +y left, +z up**, metres.

**202 Accepted**

```json
{
  "accepted": true,
  "summary": {
    "schema_version": "1.0",
    "timestamp": "2026-09-09T18:04:44.781302Z",
    "frame_id": 1,
    "sensor_id": "verify_lidar",
    "point_count": 3,
    "fields": ["x", "y", "z"],
    "coordinate_frame": "lidar",
    "source": "synthetic_test",
    "bounds": {
      "min_x": 0.0, "max_x": 10.0,
      "min_y": -2.0, "max_y": 4.0,
      "min_z": -1.0, "max_z": 0.3
    }
  },
  "detail": "Frame validated and accounted for. Phase 1 performs no filtering, detection, mapping or tracking."
}
```

`bounds` is `null` for an empty frame, and is computed over points with finite
coordinates. `processing` and `input_summary` are `null` unless preprocessing ran.

**422** — ragged rows, non-numeric values, wrong column count (not 3 or 4), NaN or infinite
values (when `preprocess` is false), a naive timestamp, an unknown `source`, or a point
count outside `[ADAPTX_LIDAR__MIN_POINTS, ADAPTX_LIDAR__MAX_POINTS]`.

### With `preprocess: true` (Phase 2A)

The frame is accepted as *raw* — NaN and infinite coordinates are permitted — and run
through validation, non-finite removal, ROI filtering and range filtering before ingest.
Points are only ever **removed**; nothing is repaired, clamped or invented.

`summary` then describes the **processed** frame, `input_summary` the frame as submitted,
and `processing` carries the measured counts and duration:

```json
{
  "accepted": true,
  "summary": { "frame_id": 7, "point_count": 2, "bounds": { } },
  "input_summary": { "frame_id": 7, "point_count": 5, "bounds": { } },
  "processing": {
    "schema_version": "1.0",
    "timestamp": "2026-09-09T18:58:33.745999Z",
    "processor": "preprocessing_v1",
    "input_point_count": 5,
    "invalid_point_count": 1,
    "roi_rejected_count": 1,
    "range_rejected_count": 1,
    "output_point_count": 2,
    "duration_ms": 0.6086,
    "stages": [
      { "stage": "validation",      "input_points": 5, "output_points": 5, "rejected_points": 0 },
      { "stage": "invalid_removal", "input_points": 5, "output_points": 4, "rejected_points": 1 },
      { "stage": "roi_filter",      "input_points": 4, "output_points": 3, "rejected_points": 1 },
      { "stage": "range_filter",    "input_points": 3, "output_points": 2, "rejected_points": 1 }
    ]
  }
}
```

Semantics:

| Rule | Behaviour |
|---|---|
| Stage order | validation → non-finite removal → ROI → range |
| Attribution | Each stage counts only points that reached it, so the counts partition the input. A point failing both ROI and range is attributed to the ROI |
| Boundaries | ROI and range bounds are **inclusive**; a point exactly on a face or at exactly `min_range_m` / `max_range_m` is kept |
| Range | 3D Euclidean `sqrt(x²+y²+z²)` from the sensor origin, not planar (ADR-011) |
| Intensity | A non-finite intensity invalidates the whole point |
| Point-count limits | Applied to the **raw input**. A frame filtered down to zero points is accepted (`202`) with `output_point_count: 0`, because that is a valid observation, not malformed input |
| `duration_ms` | Measured with `time.perf_counter` around the pipeline for this frame on this machine. Not a performance claim |

`duration_ms` is also folded into the `latency_ms` reported by
`GET /api/v1/system/metrics`, so that figure covers the real work done per frame.

#### Phase 2B stages

Voxel downsampling, ground segmentation and noise filtering are **opt-in** (ADR-012) and
configured on the server, not per request:

```
ADAPTX_LIDAR__VOXEL_ENABLED=true
ADAPTX_LIDAR__GROUND_ENABLED=true
ADAPTX_LIDAR__NOISE_ENABLED=true
```

With none enabled the response is exactly as above: four stages, and the 2B counters are
`0`. With all three enabled, `stages` reports seven entries in pipeline order and three
further counters are populated:

| Field | Meaning |
|---|---|
| `voxel_reduced_count` | Points merged away by downsampling |
| `ground_point_count` | Points classified as ground — **separated, not discarded** |
| `noise_removed_count` | Points dropped as outliers |
| `ground_summary` | Metadata of the ground points, or `null` if segmentation did not run |

When segmentation runs, `summary` describes the **non-ground** points and `ground_summary`
the ground ones. A stage that did not run is absent from `stages` rather than reported with
zero counts, so the list always says what actually happened.

The counters partition the input exactly:

```
input_point_count = invalid + roi_rejected + range_rejected
                  + voxel_reduced + ground + noise_removed + output
```

**Compatibility:** `preprocess` is optional and defaults to `false`; `processing`,
`input_summary` and `ground_summary` are optional response fields that are `null` on the
default path, and the 2B counters default to `0`. A Phase 1 or Phase 2A client is
unaffected.

Example:

```bash
curl -X POST http://localhost:8000/api/v1/lidar/frame \
  -H "Content-Type: application/json" \
  -d '{"frame_id":1,"sensor_id":"lidar_0","source":"synthetic_test","points":[[0,0,0],[1,2,3]]}'
```

---

## `POST /api/v1/lidar/detect`

Run the processing pipeline on a frame, then detect objects in the non-ground
points that survive.

The frame is **always** preprocessed here, whatever the request's `preprocess` flag says,
because detection consumes the pipeline's non-ground output. Ground segmentation must
therefore be enabled in configuration for this endpoint to be useful; with it off, the road
surface reaches the detector as one enormous cluster and is rejected by the footprint filter.

**Request:** identical to `POST /api/v1/lidar/frame`.

**200 OK** (abridged)

```json
{
  "accepted": true,
  "detection": {
    "frame_id": 3,
    "sensor_id": "roof_lidar",
    "detector": "geometric_detector_v1",
    "is_baseline": true,
    "objects": [
      {
        "object_id": 0,
        "frame_id": 3,
        "object_class": "vehicle",
        "position": { "x": 12.0, "y": -3.0, "z": -1.0 },
        "velocity": null,
        "bounding_box": {
          "center": { "x": 12.0, "y": -3.0, "z": -1.0 },
          "dimensions": { "length": 4.5, "width": 1.9, "height": 1.6 },
          "yaw_rad": 0.0
        },
        "confidence": 0.71,
        "point_count": 1948,
        "distance_m": 12.4,
        "classifier": "geometric_bands_v1",
        "is_baseline_classification": true
      }
    ],
    "rejected": [
      {
        "cluster_id": 4,
        "reason": "too_few_points",
        "point_count": 3,
        "measured_value": 3.0,
        "threshold": 10.0
      }
    ],
    "input_point_count": 2497,
    "non_ground_point_count": 2497,
    "cluster_count": 24,
    "duration_ms": 3.42,
    "clustering_duration_ms": 2.61,
    "classification_duration_ms": 0.68,
    "configuration": { }
  },
  "processing": { },
  "summary": { },
  "ground_summary": { }
}
```

### What the fields mean

| Field | Meaning |
|---|---|
| `object_class` | `vehicle`, `pedestrian`, `cyclist`, `obstacle` or `unknown` |
| `confidence` | A **geometric fit score** in [0, 1] — how centrally the cluster sits in its dimension band. **Not** a probability that the class is right; no labelled data exists to calibrate one. `0.0` when the class is `unknown` |
| `velocity` | Always `null`. A single frame cannot show motion |
| `bounding_box.yaw_rad` | Always `0.0`. Boxes are axis-aligned; no orientation is estimated |
| `distance_m` | Euclidean distance from the sensor origin to the centroid |
| `rejected` | Candidate clusters that were filtered out, each with the reason, the measured value and the threshold it missed |
| `cluster_count` | Candidates before filtering. Always equals `len(objects) + len(rejected)` |

Rejection reasons: `too_few_points`, `too_many_points`, `too_short`, `too_tall`,
`footprint_too_small`, `footprint_too_large`.

**Detections are geometric clusters classified by size.** There is no trained model, no
tracking and no semantic recognition. Classification is a documented heuristic (ADR-021),
and a cluster matching no dimension band — or more than one — is reported `unknown` rather
than guessed at.

The response carries summaries and object geometry only; raw point arrays are never echoed.

**422** — same conditions as `POST /api/v1/lidar/frame`.

---

## `POST /api/v1/lidar/track`

Process a frame, detect objects in it, and track them across frames.

**This endpoint is stateful**, unlike every other one here. Each call advances the
tracker by one frame.

| Rule | Why |
|---|---|
| Post frames in **temporal order** | Association compares each frame against the state left by the previous one |
| Send an explicit `timestamp` | Velocity is measured from the interval between successive frames. Omit it and every frame is "now", so the measured interval becomes the wall-clock gap between HTTP requests rather than between scans |
| Call `/api/v1/tracking/reset` between unrelated sequences | Otherwise the first frame of a new scene is associated against tracks from the old one |
| One track set per backend | The tracker is process-wide (ADR-025). Concurrent clients share it |

**Request:** identical to `POST /api/v1/lidar/frame`.

**200 OK** (abridged)

```json
{
  "accepted": true,
  "tracking": {
    "frame_id": 2,
    "tracker": "geometric_tracker_v1",
    "is_baseline": true,
    "tracks": [
      {
        "track_id": 0,
        "object_class": "vehicle",
        "status": "confirmed",
        "position": { "x": 12.0, "y": -3.0, "z": -1.0 },
        "previous_position": { "x": 11.0, "y": -3.0, "z": -1.0 },
        "velocity": { "x": 2.0, "y": 0.0, "z": 0.0 },
        "observed_velocity": { "x": 2.0, "y": 0.0, "z": 0.0 },
        "acceleration": { "x": 0.0, "y": 0.0, "z": 0.0 },
        "heading_rad": 0.0,
        "predicted_position": { "x": 12.0, "y": -3.0, "z": -1.0 },
        "hits": 3,
        "age_frames": 3,
        "missed_frames": 0,
        "first_seen": "2026-01-01T12:00:00Z",
        "last_seen": "2026-01-01T12:00:01Z",
        "confidence": 0.6
      }
    ],
    "new_track_ids": [],
    "deleted_track_ids": [],
    "unmatched_detection_ids": [],
    "unmatched_track_ids": [],
    "detection_count": 1,
    "association_count": 1,
    "duration_ms": 0.21,
    "association_duration_ms": 0.06,
    "configuration": { }
  },
  "detection": { },
  "processing": { },
  "summary": { },
  "ground_summary": { }
}
```

### What the track fields mean

| Field | Meaning |
|---|---|
| `track_id` | Stable for the track's lifetime, **never reused** once retired. Not a global identity |
| `status` | `tentative` (not yet trusted), `confirmed` (matched this frame), `coasting` (alive but unmatched), `lost` (retired — appears only in `deleted_track_ids`) |
| `velocity` | **Null until two observations.** One frame cannot show motion, and zero would claim a measured standstill (ADR-023). Also null for a non-positive or over-long frame interval |
| `observed_velocity` | The raw frame-to-frame value before smoothing, so smoothing can never hide a jump |
| `acceleration` | Null until two consecutive velocities exist |
| `heading_rad` | Null below the configured speed floor, where direction would describe noise rather than travel |
| `predicted_position` | Where the tracker expected this track, used for **association gating only**. Not an observation, and **not a trajectory prediction** — that is a later phase |
| `confidence` | The detector's geometric fit score, carried through. Still not a probability |

**Velocity is measured, never assumed.** Nothing here is extrapolated into the future.

**422** — same conditions as the other LiDAR endpoints.

---

## `POST /api/v1/lidar/predict`

Process a frame, detect objects, track them, and predict where the tracked objects will
be over the next few seconds.

**This endpoint is stateful**, for the same reason `/track` is: prediction consumes
tracks, and a track exists only because of the frames before it. Every rule in the
`/track` table above applies here unchanged — temporal order, explicit timestamps, reset
between sequences, one track set per backend.

Prediction *itself* carries no state between frames. The same tracks and timestamp always
produce the same trajectories.

**The predictor is a deterministic constant-velocity baseline** (ADR-026). It extrapolates
each track's measured velocity and models nothing else: no acceleration, no turning, no
road or lane geometry, no interaction between objects. **Its accuracy is unmeasured**,
because no labelled trajectories exist.

**Request:** identical to `POST /api/v1/lidar/frame`.

**200 OK** (abridged)

```json
{
  "accepted": true,
  "mapping": {
    "mapper": "fixed_resolution_mapper_v1",
    "is_adaptive": false,
    "adaptive_resolution_implemented": false,
    "lifecycle": "frame_local",
    "frames_mapped": 3,
    "resolution_m": 0.5,
    "width": 240,
    "height": 240,
    "total_cells": 57600,
    "occupied_cells": 6650,
    "occupancy_ratio": 0.1155,
    "input_points": 9800,
    "mapped_points": 9720,
    "out_of_bounds_points": 80,
    "duration_ms": 7.47,
    "configuration": { }
  },
  "prediction": {
    "timestamp": "2026-01-01T12:00:01Z",
    "frame_id": 2,
    "sensor_id": "roof_lidar",
    "predictor": "constant_velocity_v1",
    "model_name": "constant_velocity",
    "is_baseline": true,
    "uncertainty_model": "heuristic_linear_growth",
    "trajectories": [
      {
        "track_id": 0,
        "timestamp": "2026-01-01T12:00:01Z",
        "horizon_s": 3.0,
        "timestep_s": 0.25,
        "status": "predicted",
        "observation_age_s": 0.0,
        "confidence": 1.0,
        "predictor_name": "constant_velocity_v1",
        "coordinate_frame": "ego",
        "source": "live_sensor",
        "points": [
          {
            "time_offset_s": 0.0,
            "timestamp": "2026-01-01T12:00:01Z",
            "position": { "x": 12.0, "y": -3.0, "z": -1.0 },
            "velocity": { "x": 2.0, "y": 0.0, "z": 0.0 },
            "confidence": 1.0,
            "position_uncertainty_m": 0.5
          },
          {
            "time_offset_s": 1.0,
            "timestamp": "2026-01-01T12:00:02Z",
            "position": { "x": 14.0, "y": -3.0, "z": -1.0 },
            "velocity": { "x": 2.0, "y": 0.0, "z": 0.0 },
            "confidence": 0.5,
            "position_uncertainty_m": 1.0
          }
        ]
      }
    ],
    "skipped": [],
    "considered_track_count": 1,
    "duration_ms": 0.27,
    "configuration": { }
  },
  "tracking": { },
  "detection": { },
  "processing": { },
  "summary": { },
  "ground_summary": { }
}
```

Points run from `t+0` to the horizon **inclusive**, so the default 3.0 s horizon at a
0.25 s interval yields **13 points** per track. The example above is abridged to two.

### What the prediction fields mean

| Field | Meaning |
|---|---|
| `timestamp` (on the result and each trajectory) | The **source** time predictions were made from — the tracking frame's time. Every `time_offset_s` is measured forward from it |
| `timestamp` (on a point) | `source timestamp + time_offset_s`, computed arithmetically. **Never a wall clock**, so a trajectory is reproducible |
| `status` | `predicted` (extrapolated from a velocity measured this frame) or `extrapolated` (the track is coasting, so the starting position is itself already stale) |
| `observation_age_s` | **Measured** seconds between the track's last observation and the prediction time. `0.0` for a track seen this frame; positive for a coasting track, whose whole trajectory is shifted by it |
| `position` | An **extrapolation, not a measurement.** Predicted positions appear only here, and are never written back onto `tracking.tracks[].position` |
| `velocity` | The constant velocity the trajectory was built from, repeated on each point |
| `position_uncertainty_m` | A **heuristic** radius: `base_uncertainty_m + uncertainty_growth_mps * (observation_age_s + time_offset_s)`. Not a calibrated sigma, **not a probability**, not a confidence interval — no labelled trajectories exist to calibrate one (ADR-026) |
| `confidence` (trajectory) | An **evidence** score from the track's `hits` and `missed_frames`. Not a probability that the prediction is correct |
| `confidence` (point) | The trajectory confidence decayed exactly as fast as uncertainty grows: `track_confidence * uncertainty(t+0) / uncertainty(t)` |
| `skipped` | Tracks considered and deliberately **not** predicted, each with a status and reason. `considered_track_count == len(trajectories) + len(skipped)`, enforced by the contract |

### Why a track may be skipped

| `status` | Meaning |
|---|---|
| `insufficient_velocity` | The track has **no measured velocity** — usually its first frame. Null is not zero (ADR-023): emitting a "stays where it is" path would fabricate a measurement. A *measured* standstill is different and does produce a stationary trajectory |
| `invalid_velocity` | Measured speed exceeds `ADAPTX_PREDICTION__MAX_SPEED_MPS`. The value is **rejected, not clipped** — a clipped velocity is a number no sensor produced |
| `stale_observation` | The last observation is older than the horizon, so the output would be more gap-filling than prediction |
| `track_lost` | The track is terminated; no active prediction is published for it |
| `limit_exceeded` | `ADAPTX_PREDICTION__MAX_TRACKS` was reached. Tracks are **not** prioritised — Phase 5 has no risk signal to prioritise by |

**422** — same conditions as the other LiDAR endpoints.

---

## `POST /api/v1/lidar/map`

Process a frame and bin the resulting points into a 2.5D spatial grid.

**Stateless and frame-local**, unlike `/track` and `/predict`. Each call builds a complete
map from the frame it is given; nothing accumulates and nothing carries over from an earlier
request. There is no reset endpoint because there is no state to reset.

**The mapper is a deterministic fixed-resolution baseline** (ADR-028). One cell size applies
across the whole map. **Adaptive, risk-aware resolution is not implemented** — the mapper is
handed a resolution and never chooses one (ADR-029).

| Rule | Why |
|---|---|
| Cells are half-open, `[lo, hi)` | A point at exactly `max_x` would index one cell past the last column |
| A point on `min_x`/`min_y` is the first cell | The lower edge is inclusive |
| A point on `max_x`/`max_y` is **out of bounds** | Counted, never clamped inward into a cell it does not belong to |
| Occupancy is binary | A cell holds at least one point, or it does not. No probability, no Bayesian update, no temporal fusion (ADR-031) |
| An unobserved cell reports `null` height | `z = 0` is a real height here; zero would be indistinguishable from a measured flat surface at sensor height |

**Request:** as `POST /api/v1/lidar/frame`, plus three optional fields.

| Field | Default | Meaning |
|---|---|---|
| `resolution_m` | configured value | Cell edge length for this request only. Recorded with `source: "override"` so a one-off is never mistaken for the configured baseline |
| `include_cells` | `false` | Return occupied cells. Off by default: a 0.5 m map over the default bounds is 57,600 cells |
| `max_cells` | 20000 | Upper bound on returned cells. Truncation is reported, never silent |

**200 OK** (abridged)

```json
{
  "accepted": true,
  "map": {
    "timestamp": "2026-01-01T12:00:00Z",
    "frame_id": 0,
    "sensor_id": "roof_lidar",
    "mapper": "fixed_resolution_mapper_v1",
    "is_adaptive": false,
    "resolution": {
      "resolution_m": 0.5,
      "source": "fixed",
      "reason": "phase6_fixed_baseline",
      "requested_by": "configuration"
    },
    "bounds": { "min_x": -60.0, "max_x": 60.0, "min_y": -60.0, "max_y": 60.0 },
    "width": 240,
    "height": 240,
    "accounting": {
      "input_point_count": 9800,
      "mapped_point_count": 9720,
      "out_of_bounds_point_count": 80,
      "occupied_cell_count": 6650,
      "total_cell_count": 57600
    },
    "duration_ms": 7.47,
    "configuration": { }
  },
  "processing": { },
  "summary": { },
  "ground_summary": { },
  "cells": null,
  "cells_truncated": false
}
```

The **dense grid is never returned.** `map` carries dimensions, bounds, resolution and
accounting. With `include_cells: true`, `cells` holds an `AdaptiveMap` containing **occupied
cells only** — empty cells are omitted rather than emitted as unknown, because a fine map is
overwhelmingly empty and serialising it would say nothing at great length.

### What the map fields mean

| Field | Meaning |
|---|---|
| `is_adaptive` | Always `false` in Phase 6. Distinguishes a fixed-resolution measurement from an adaptive one so the two can never be confused in a benchmark (ADR-003) |
| `resolution.source` | `fixed` (configured), `override` (this request only), or `adaptive` — **reserved, never produced today** |
| `accounting` | `input_point_count == mapped_point_count + out_of_bounds_point_count`, enforced by the contract. A point outside the bounds is counted, not silently dropped |
| `occupied_cell_count` | Cells holding at least one point. `occupied / total` is the occupancy ratio |
| `duration_ms` | Measured with `perf_counter`, not estimated |
| `cells[].height_m` / `height_min_m` / `height_max_m` | Mean / min / max measured height in that cell. `null` never appears here because only occupied cells are projected |
| `cells[].risk_score` / `uncertainty` | Left at `0.0`. **Nothing computes them yet**; filling them would be an invention |

**422** — malformed points, as the other LiDAR endpoints; also when `resolution_m` is
outside the configured `[min_resolution_m, max_resolution_m]` range, or when the resulting
grid would exceed the configured cell budget.

---

## `POST /api/v1/lidar/risk`

Run the whole perception chain and assess the risk of each tracked object.

**Stateful**, because it consumes tracks. Every rule on `POST /api/v1/lidar/track` applies
unchanged — temporal order, explicit timestamps, reset between sequences. Risk assessment
itself carries no state between frames.

> **The score is a deterministic engineering heuristic. It is not a probability of
> collision.** It is not calibrated, has never been validated against labelled risk data —
> none exists — and its thresholds are baseline engineering values, not safety-certified
> limits. No accuracy figure is reported anywhere, because there is nothing to measure one
> against.

### How the score is built

Three factors, each normalised to `[0, 1]`, combined as a weighted mean **over the factors
actually available** (ADR-032):

```
risk_score = sum(w_i * f_i) / sum(w_i)     over available i only
```

| Factor | Meaning | Unavailable when |
|---|---|---|
| `proximity` | 1.0 at/below `proximity_near_m`, 0.0 at/above `proximity_far_m` | never |
| `closing_speed` | radial rate of approach; receding contributes 0.0 | `velocity` was never measured, or the object sits exactly at the reference point |
| `predicted_proximity` | closest approach of the Phase 5 trajectory | the track has no predicted trajectory |

**A missing factor is dropped and the remaining weights renormalise — never scored zero.**
Zero is what an unmeasured value would look like, so scoring it that way would make the
object the system knows least about appear least concerning. When nothing can be computed,
the assessment is `UNKNOWN` with `risk_score: null`.

**Request:** as `POST /api/v1/lidar/frame`, plus `include_map_context` (default `true`).
Turning it off raises reported uncertainty rather than hiding the absence.

**200 OK** (abridged, one assessment shown)

```json
{
  "accepted": true,
  "risk": {
    "timestamp": "2026-01-01T12:00:01Z",
    "frame_id": 2,
    "sensor_id": "roof_lidar",
    "engine": "heuristic_risk_v1",
    "is_baseline": true,
    "scoring_model": "heuristic_weighted_factors",
    "considered_track_count": 1,
    "duration_ms": 0.41,
    "assessments": [
      {
        "track_id": 0,
        "status": "assessed",
        "risk_level": "high",
        "risk_score": 0.72,
        "distance_m": 8.4,
        "closing_speed_mps": 6.0,
        "speed_mps": 6.0,
        "track_status": "confirmed",
        "object_class": "vehicle",
        "trajectory": {
          "min_distance_m": 1.2,
          "time_to_min_distance_s": 1.25,
          "uncertainty_at_min_m": 1.13,
          "horizon_s": 3.0,
          "is_approaching": true
        },
        "map_context": {
          "observation": "observed_occupied",
          "point_count": 47,
          "max_height_m": -0.82
        },
        "uncertainty": {
          "score": 0.0,
          "reasons": [],
          "observation_age_s": 0.0,
          "is_stale": false,
          "velocity_known": true,
          "prediction_available": true,
          "track_confidence": 0.8
        },
        "factors": ["proximity", "closing_speed", "predicted_proximity"],
        "factor_scores": {
          "proximity": 0.90,
          "relative_velocity": 0.40,
          "trajectory_overlap": 1.0,
          "uncertainty": 0.0
        },
        "reason": "High risk: vehicle at 8.4 m; closing at 6.0 m/s; predicted to pass within 1.2 m at t+1.25s (heuristic uncertainty 1.13 m)."
      }
    ],
    "configuration": { }
  },
  "tracking": { },
  "detection": { },
  "processing": { },
  "prediction_summary": { },
  "map_summary": { },
  "summary": { }
}
```

### What the assessment fields mean

| Field | Meaning |
|---|---|
| `risk_score` | Heuristic engineering score in `[0, 1]`. **`null` exactly when `risk_level` is `unknown`** — nothing is invented to fill the gap |
| `risk_level` | `low` / `medium` / `high` / `critical` are scored bands. **`unknown` is not a point on that scale**: it means nothing could be computed, not that risk is low |
| `status` | `assessed`, `insufficient_data`, or `track_lost`. A terminated track is history, not a present concern, and is never scored |
| `closing_speed_mps` | Rate of approach; negative means receding. **`null` when velocity was never measured** — unknown motion is not a standstill (ADR-023). A *measured* standstill reports `0.0` |
| `trajectory` | Closest approach of the predicted path. **Not a collision test and not proof of a collision** — the path is a constant-velocity extrapolation with heuristic uncertainty |
| `map_context.observation` | `observed_occupied` / `observed_empty` / `out_of_bounds` / `no_map`. **`observed_empty` means unobserved, not free** — the cell may be empty or occluded, and Phase 6 cannot tell the difference. Map context **never lowers** risk (ADR-034) |
| `uncertainty` | A **heuristic** scalar with its contributing reasons kept visible. **Reported beside risk, never folded into it** (ADR-033): two tracks identical except for observability get the same risk and different uncertainty |
| `uncertainty.track_confidence` | Phase 3's **geometric fit score** carried through unchanged, not a probability that the classification is correct (ADR-021) |
| `factors` | Only the factors **actually computed**. A factor absent here was not computed — different from computed-and-zero |
| `reason` | Generated from the computed factors. Never free-form text, and never asserts probability, safety or validation |

**No field carries a resolution.** Risk says *how concerning*; deciding *how much spatial
detail* a region receives is a separate decision that is not implemented (ADR-036).

**422** — malformed points, as the other LiDAR endpoints.

---

## `GET /api/v1/prediction/status`

Report the configured predictor, its horizon, and what its numbers mean.

**200 OK** (abridged)

```json
{
  "component": {
    "name": "prediction",
    "readiness": "READY",
    "implementation": "PARTIAL",
    "phase": 5,
    "detail": "deterministic constant-velocity baseline with heuristic uncertainty ..."
  },
  "predictor": "constant_velocity_v1",
  "model_name": "constant_velocity",
  "is_baseline": true,
  "horizon_s": 3.0,
  "interval_s": 0.25,
  "points_per_trajectory": 13,
  "uncertainty_model": "heuristic_linear_growth",
  "uncertainty_is_heuristic": true,
  "prediction_statuses": [
    "predicted", "extrapolated", "insufficient_velocity", "invalid_velocity",
    "stale_observation", "track_lost", "limit_exceeded"
  ],
  "configuration": {
    "horizon_s": 3.0,
    "interval_s": 0.25,
    "max_tracks": 256,
    "max_speed_mps": 80.0,
    "base_uncertainty_m": 0.5,
    "uncertainty_growth_mps": 0.5,
    "confidence_hits_full": 3,
    "modelled_factors": ["measured_velocity"],
    "unmodelled_factors": [
      "acceleration", "turning", "road_and_lane_geometry",
      "object_interaction", "object_class_specific_motion"
    ]
  },
  "summary": { }
}
```

`uncertainty_is_heuristic` is always `true`, and **no accuracy figure is reported** — there
is nothing to measure one against. `implementation` is `PARTIAL` and never `IMPLEMENTED`:
this is a baseline.

`summary` carries counts from the most recent prediction, or a note saying no frame has
been predicted yet.

---

## `POST /api/v1/tracking/reset`

Drop every track and restart identifier allocation from zero.

**200 OK**

```json
{
  "reset": true,
  "cleared_track_count": 3,
  "detail": "All tracks dropped and identifier allocation restarted from zero."
}
```

---

## `GET /api/v1/tracking/status`

Current tracking state: counts, identifiers and effective configuration. No per-track
geometry — use `POST /api/v1/lidar/track` for that.

**200 OK**

```json
{
  "summary": {
    "tracker": "geometric_tracker_v1",
    "is_baseline": true,
    "frames_tracked": 12,
    "active_tracks": 3,
    "confirmed": 2,
    "tentative": 1,
    "coasting": 0,
    "moving": 2,
    "new_last_frame": 0,
    "deleted_last_frame": 0,
    "track_ids": [0, 1, 2],
    "counts_by_class": { "vehicle": 2, "pedestrian": 1 },
    "configuration": { }
  }
}
```

---

## `GET /api/v1/map/status`

Report mapping readiness, the applied resolution and the mapped extent.

**200 OK** (abridged)

```json
{
  "component": {
    "name": "mapping",
    "readiness": "READY",
    "implementation": "PARTIAL",
    "phase": 6,
    "detail": "deterministic frame-local 2.5D fixed-resolution mapping baseline ..."
  },
  "mapper": "fixed_resolution_mapper_v1",
  "is_adaptive": false,
  "adaptive_resolution_implemented": false,
  "lifecycle": "frame_local",
  "resolution_m": 0.5,
  "resolution_source": "fixed",
  "bounds": { "min_x": -60.0, "max_x": 60.0, "min_y": -60.0, "max_y": 60.0 },
  "width": 240,
  "height": 240,
  "total_cells": 57600,
  "active_cells": 0,
  "last_map_timestamp": null,
  "range_m": 60.0,
  "resolution_levels": { "low": 1.0, "medium": 0.5, "high": 0.2, "critical": 0.1 },
  "configuration": {
    "is_adaptive_algorithm_implemented": false,
    "fixed_resolution_baseline_available": true,
    "occupancy": "binary: a cell is occupied iff it holds at least one point",
    "unobserved_height": "null, never zero",
    "modelled": ["occupancy", "point_count", "min/max/mean height"],
    "not_modelled": [
      "probabilistic_occupancy", "temporal_fusion", "adaptive_resolution",
      "risk", "uncertainty", "semantic_labels"
    ]
  },
  "summary": { }
}
```

| Field | Meaning |
|---|---|
| `is_adaptive` / `adaptive_resolution_implemented` | Both **false**. A fixed-resolution mapper exists; nothing allocates resolution by risk |
| `lifecycle` | `frame_local` — each map covers one frame and nothing accumulates. Not a persistent world map (ADR-030) |
| `resolution_m` / `resolution_source` | The cell size the mapper applies, and where it came from |
| `bounds` / `width` / `height` / `total_cells` | The extent actually mapped. Distinct from `range_m`, which is the advertised sensing range retained from the Phase 1 response |
| `active_cells` | Occupied cells in the **most recent** map; `0` before the first frame |
| `last_map_timestamp` | Source time of that map; `null` before the first frame |
| `resolution_levels` | Configuration only: what each level *would* mean to a controller, not a decision that has been made |

`summary` carries counts from the most recent map, or a note saying no frame has been mapped
yet. **No accuracy figure is reported** — no labelled reference map exists to measure one
against.

---

## `GET /api/v1/risk/status`

Report the configured risk engine and what its numbers mean.

**200 OK** (abridged)

```json
{
  "component": {
    "name": "risk",
    "readiness": "READY",
    "implementation": "PARTIAL",
    "phase": 7,
    "detail": "deterministic heuristic risk and uncertainty baseline ..."
  },
  "engine": "heuristic_risk_v1",
  "is_baseline": true,
  "baseline_engine": "baseline_proximity",
  "scoring_model": "heuristic_weighted_factors",
  "score_is_heuristic": true,
  "is_collision_probability": false,
  "uncertainty_is_heuristic": true,
  "decides_resolution": false,
  "risk_levels": ["low", "medium", "high", "critical", "unknown"],
  "thresholds": { "low": 0.0, "medium": 0.35, "high": 0.6, "critical": 0.85 },
  "frames_assessed": 0,
  "last_assessment_timestamp": null,
  "configuration": {
    "proximity_near_m": 5.0,
    "proximity_far_m": 40.0,
    "closing_speed_high_mps": 15.0,
    "weight_proximity": 0.5,
    "weight_closing_speed": 0.25,
    "weight_predicted_proximity": 0.25,
    "modelled_factors": ["proximity", "closing_speed", "predicted_proximity"],
    "unmodelled_factors": [
      "time_to_collision", "trajectory_map_intersection", "object_interaction",
      "road_and_lane_geometry", "ego_planned_path",
      "calibrated_collision_probability"
    ],
    "uncertainty_sources": [
      "unknown_velocity", "stale_observation", "low_track_confidence",
      "no_prediction", "wide_prediction_uncertainty", "tentative_track",
      "coasting_track", "unobserved_map_context"
    ],
    "thresholds_are": "baseline engineering values, not safety-certified limits",
    "unobserved_map_cells": "treated as unobserved, never as free space"
  },
  "summary": { }
}
```

| Field | Meaning |
|---|---|
| `score_is_heuristic` | Always **true**. A deterministic engineering heuristic, not a calibrated model |
| `is_collision_probability` | Always **false**. **No collision-probability model exists in this project** |
| `uncertainty_is_heuristic` | Always **true**. Uncertainty is reported *separately* from risk — an object can be low-risk and poorly observed |
| `decides_resolution` | Always **false**. The risk engine never chooses spatial resolution; that belongs to a resolution controller, which is not implemented (ADR-036) |
| `baseline_engine` | The proximity-only engine retained alongside for comparison |
| `thresholds` | Covers the four **scored** levels only. `unknown` appears in `risk_levels` but has no threshold, because it means nothing was scored |
| `configuration.unmodelled_factors` | What this engine deliberately does not model. Time-to-collision is absent on purpose: over a constant-velocity extrapolation with heuristic uncertainty it would be a precise-looking number resting on two approximations |

**No accuracy figure is reported** — no labelled risk data exists to measure one against.
`implementation` is `PARTIAL` and never `IMPLEMENTED`.

---

## `WS /ws/telemetry`

Live channel for the dashboard. It carries the aggregated system status, the measured
metrics, and **summaries** of detection, tracking and prediction. It emits no risk cells
and no map cells, and names those absent streams in `not_yet_available`.

The perception entries are deliberately summaries — counts, identifiers and configuration.
The channel never carries per-frame geometry, trajectory points or point arrays; those come
from the LiDAR endpoints. A stream leaves `not_yet_available` only once something genuinely
produces it.

**Envelope**

```json
{
  "type": "hello" | "telemetry",
  "sequence": 0,
  "timestamp": "2026-09-09T18:06:30.290000+00:00",
  "data": { }
}
```

**First message** (`type: "hello"`, `sequence: 0`)

```json
{
  "name": "ADAPT-X",
  "environment": "development",
  "interval_s": 1.0,
  "provides": ["system", "metrics"],
  "not_yet_available": ["risk_field", "adaptive_map"]
}
```

**Subsequent messages** (`type: "telemetry"`, `sequence` incrementing from 1), emitted every
`ADAPTX_WEBSOCKET__TELEMETRY_INTERVAL_S` seconds:

```json
{
  "system":  { },
  "metrics": { },
  "detection": { },
  "tracking": { },
  "prediction": {
    "predictor": "constant_velocity_v1",
    "model": "constant_velocity",
    "is_baseline": true,
    "uncertainty_model": "heuristic_linear_growth",
    "uncertainty_is_heuristic": true,
    "horizon_s": 3.0,
    "interval_s": 0.25,
    "frames_predicted": 12,
    "considered_tracks": 3,
    "predicted_tracks": 2,
    "skipped_tracks": 1,
    "predicted_points": 26,
    "predicted_track_ids": [0, 1],
    "counts_by_status": { "predicted": 2, "insufficient_velocity": 1 },
    "duration_ms": 0.27,
    "configuration": { }
  },
  "risk": {
    "risk_engine": "heuristic_risk_v1",
    "is_baseline": true,
    "scoring_model": "heuristic_weighted_factors",
    "score_is_heuristic": true,
    "is_collision_probability": false,
    "frames_assessed": 12,
    "total_objects": 4,
    "low_count": 1,
    "medium_count": 2,
    "high_count": 1,
    "critical_count": 0,
    "unknown_count": 0,
    "highest_risk_level": "high",
    "highest_risk_score": 0.72,
    "max_uncertainty": 0.35,
    "processing_time_ms": 0.41,
    "objects": [
      { "track_id": 0, "risk_level": "high", "risk_score": 0.72,
        "distance_m": 8.4, "uncertainty": 0.0 }
    ],
    "configuration": { }
  },
  "provides": [
    "system", "metrics", "detection", "tracking", "prediction", "mapping", "risk"
  ],
  "not_yet_available": ["risk_field", "adaptive_map"]
}
```

The `mapping` summary never carries the grid itself — a 0.5 m map over the default bounds is
57,600 cells. The `risk` summary carries scene counts plus at most **five** ranked objects,
never every assessment. Cells and full assessments come from `POST /api/v1/lidar/map` and
`POST /api/v1/lidar/risk` instead.

`system` is the `GET /api/v1/system/status` payload; `metrics` is the
`GET /api/v1/system/metrics` payload. Before the first predicted frame the `prediction`
summary carries a `note` saying so instead of the counts.

The channel is send-only; client messages are not interpreted. When
`ADAPTX_WEBSOCKET__MAX_CONNECTIONS` is reached, the connection is closed with code `1008`
and reason `telemetry connection limit reached`.

Example client (the `websockets` package ships with `uvicorn[standard]`):

```python
import asyncio, json, websockets


async def main():
    async with websockets.connect("ws://localhost:8000/ws/telemetry") as ws:
        for _ in range(3):
            print(json.loads(await ws.recv()))


asyncio.run(main())
```

---

## CORS

Origins come from `ADAPTX_API__CORS_ORIGINS` (default `http://localhost:3000` and
`http://localhost:5173`). Allowed methods: `GET`, `POST`, `OPTIONS`. Credentials are not
allowed.
