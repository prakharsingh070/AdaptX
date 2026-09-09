# ADAPT-X API

Base URL (development): `http://localhost:8000`
Versioned prefix: `/api/v1`
Interactive documentation: `/docs` (Swagger UI), `/redoc`, `/openapi.json`

**This document describes only what is implemented.** Endpoints for objects, tracking,
prediction, the adaptive map, the risk map, events, scenarios and benchmarking are sketched
in [`knowledge-base/16_api-contract.md`](knowledge-base/16_api-contract.md) and do not
exist yet.

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
      "detail": "frame validation, bounds and metadata only; no filtering, ground segmentation, voxelisation or clustering",
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
| `gpu_percent` | Always `null` in Phase 1 |
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

Submit a point-cloud frame. The frame is validated, counted and summarised. It is **not**
filtered, detected on, tracked or mapped — those modules do not exist.

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

`bounds` is `null` for an empty frame.

**422** — ragged rows, non-numeric values, wrong column count (not 3 or 4), NaN or infinite
values, a naive timestamp, an unknown `source`, or a point count outside
`[ADAPTX_LIDAR__MIN_POINTS, ADAPTX_LIDAR__MAX_POINTS]`.

Example:

```bash
curl -X POST http://localhost:8000/api/v1/lidar/frame \
  -H "Content-Type: application/json" \
  -d '{"frame_id":1,"sensor_id":"lidar_0","source":"synthetic_test","points":[[0,0,0],[1,2,3]]}'
```

---

## `GET /api/v1/map/status`

Mapping readiness and the configured resolution vocabulary. No mapper exists, so
`active_cells` is always 0.

**200 OK**

```json
{
  "schema_version": "1.0",
  "timestamp": "2026-09-09T18:04:53.171749Z",
  "component": {
    "name": "mapping",
    "readiness": "NOT_READY",
    "implementation": "PLANNED",
    "detail": "2.5D map and resolution contracts only; no mapper and no adaptive resolution algorithm implemented",
    "phase": 5,
    "required": false
  },
  "configuration": {
    "is_adaptive_algorithm_implemented": false,
    "fixed_resolution_baseline_available": false
  },
  "resolution_levels": { "low": 1.0, "medium": 0.5, "high": 0.2, "critical": 0.1 },
  "range_m": 60.0,
  "active_cells": 0
}
```

`resolution_levels` gives the configured cell edge length in metres for each level. It
states what each level *would* mean; no resolution decision has been made.

---

## `GET /api/v1/risk/status`

Risk engine readiness, normalisation bounds and — importantly — which factors the
configured engine actually models.

**200 OK**

```json
{
  "schema_version": "1.0",
  "timestamp": "2026-09-09T18:04:53.211552Z",
  "component": {
    "name": "risk",
    "readiness": "NOT_READY",
    "implementation": "PARTIAL",
    "detail": "contract plus a proximity-only baseline used for testing; the ADAPT-X risk engine is not implemented",
    "phase": 6,
    "required": false
  },
  "configuration": {
    "max_range_m": 60.0,
    "scale": "normalised [0, 1]; dashboards may render 0-100",
    "modelled_factors": ["proximity"],
    "unmodelled_factors": [
      "relative_velocity", "time_to_collision", "trajectory_overlap",
      "object_importance", "uncertainty"
    ]
  },
  "engine": "baseline_proximity",
  "is_baseline": true,
  "risk_levels": ["low", "medium", "high", "critical"],
  "thresholds": { "low": 0.0, "medium": 0.35, "high": 0.6, "critical": 0.85 }
}
```

`thresholds` are lower bounds on the normalised scale. `is_baseline: true` means the
configured engine is a comparison baseline, **not** the ADAPT-X risk engine.

---

## `WS /ws/telemetry`

Live channel for the dashboard. In Phase 1 it carries the aggregated system status and the
measured metrics — nothing else. It emits no detections, tracks, predictions, risk cells or
map cells, and names those absent streams in `not_yet_available`.

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
  "not_yet_available": ["detected_objects", "tracked_objects", "predicted_trajectories", "risk_field", "adaptive_map"]
}
```

**Subsequent messages** (`type: "telemetry"`, `sequence` incrementing from 1), emitted every
`ADAPTX_WEBSOCKET__TELEMETRY_INTERVAL_S` seconds:

```json
{
  "system":  { },
  "metrics": { },
  "provides": ["system", "metrics"],
  "not_yet_available": ["detected_objects", "tracked_objects", "predicted_trajectories", "risk_field", "adaptive_map"]
}
```

`system` is the `GET /api/v1/system/status` payload; `metrics` is the
`GET /api/v1/system/metrics` payload.

The channel is send-only in Phase 1; client messages are not interpreted. When
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
