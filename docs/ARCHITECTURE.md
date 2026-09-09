# ADAPT-X Architecture

This document describes the **actual state of the code**. Domain knowledge, requirements
and the conceptual design of each layer live in [`knowledge-base/`](knowledge-base), which
this document does not restate or override.

Anything marked *Planned* below does not exist in the repository beyond a data contract or
an abstract interface.

---

## 1. Implementation status

| Layer | Status | What exists today |
|---|---|---|
| Configuration | **Implemented** | Typed, environment-driven settings with validation (`config/settings.py`) |
| Logging | **Implemented** | Structured text/JSON logging with contextual fields (`core/logging.py`) |
| Data contracts | **Implemented** | Point cloud, vehicle, object, track, trajectory, map, risk, system models (`models/`) |
| API | **Implemented** | 7 HTTP endpoints + OpenAPI (`api/`) |
| Telemetry (WebSocket) | **Implemented** | `/ws/telemetry`, carrying system status and measured metrics only |
| Metrics | **Implemented** | Measured ingest FPS/latency and process CPU/memory; unmeasured values are `null` |
| LiDAR ingest | **Partial** | Structural validation, point count, bounds, provenance |
| LiDAR preprocessing | **Partial** | Phase 2A: input validation, NaN/Inf removal, ROI filtering, range filtering, per-stage counts and measured duration. **No** voxelisation/downsampling, ground segmentation, statistical outlier removal or clustering |
| Risk | **Partial** | `RiskEngine` contract + a proximity-only **baseline**. The ADAPT-X risk engine does not exist |
| CARLA | **Boundary only** | Interface, real client (connect + world info), deterministic mock, status service. Sensor and actor operations raise explicitly |
| Object detection | *Planned* | `ObjectDetector` contract only |
| Tracking | *Planned* | `ObjectTracker` contract only |
| 2.5D mapping | *Planned* | `AdaptiveMapper` contract + cell/map models only |
| Adaptive resolution | *Planned* | `ResolutionController` contract + `ResolutionContext` model only |
| Trajectory prediction | *Planned* | `TrajectoryPredictor` contract only |
| Uncertainty engine | *Planned* | `uncertainty` fields exist on the models; nothing computes them |
| Fixed-resolution baseline | *Planned* | `AdaptiveMap.is_adaptive` distinguishes the two variants; neither mapper exists |
| Scenario generation, event replay, benchmarking | *Planned* | Not started |
| Dashboard | *Planned* | Not started (`dashboard/README.md`) |

The running backend reports this itself at `GET /api/v1/system/status`. Each component
carries a **readiness** (`READY` / `NOT_READY`) and an **implementation status**
(`IMPLEMENTED` / `PARTIAL` / `PLANNED` / `MOCK`), so a module that does not exist can never
look like one that does.

---

## 2. Layering

The layer model from [`knowledge-base/03_system-architecture.md`](knowledge-base/03_system-architecture.md)
is unchanged. Phase 1 implements the edges of that pipeline (input and output) and the
contracts between the layers in the middle:

```
                        IMPLEMENTED                    PLANNED
                   ┌───────────────────┐   ┌──────────────────────────────────┐
  HTTP / WS  ──▶   │  api/             │   │                                  │
                   │   routes          │   │  perception  → tracking          │
                   │   websocket       │   │       ↓            ↓             │
                   ├───────────────────┤   │  mapping  ←   prediction         │
                   │  services/        │   │       ↑            ↓             │
                   │   lidar_ingest    │   │       └──────  risk              │
                   │   metrics         │   │              (baseline only)     │
                   │   carla           │   └──────────────────────────────────┘
                   │   system status   │
                   ├───────────────────┤
                   │  models/  (contracts shared by every layer)              │
                   └──────────────────────────────────────────────────────────┘
```

**Dependency rule.** `api` → `services` → `perception` / `risk` / `carla` → `models`.
Nothing in `models` imports a service or a route. Perception modules never import
dashboard or API code (knowledge-base boundary rule).

---

## 3. Package map

| Package | Responsibility |
|---|---|
| `adaptx.config` | Typed settings loaded from the environment; validation of ranges and orderings |
| `adaptx.core` | `logging`, `exceptions`, `lifecycle` (the service graph and startup/shutdown) |
| `adaptx.models` | All data contracts. State and validation only — no business logic |
| `adaptx.perception` | `LiDARProcessor` / `ObjectDetector` contracts; `FrameValidationProcessor` (Phase 1); `PointCloudPreprocessor` (Phase 2A) |
| `adaptx.mapping` | `AdaptiveMapper` and `ResolutionController` contracts |
| `adaptx.risk` | `RiskEngine` contract; `BaselineProximityRiskEngine` |
| `adaptx.tracking` | `ObjectTracker` contract |
| `adaptx.prediction` | `TrajectoryPredictor` contract |
| `adaptx.carla` | `CarlaSimulatorClient` contract, real client, mock, simulator-local models |
| `adaptx.services` | Ingest, metrics, CARLA and system-status services |
| `adaptx.api` | Routes, HTTP schemas, WebSocket telemetry, dependency wiring |

---

## 4. Data contracts

Defined in `adaptx/models/`, re-exported from `adaptx.models`.

| Model | Purpose |
|---|---|
| `BasePointCloudFrame` | Shared frame metadata and structural validation (shape, dtype, column layout) |
| `RawPointCloudFrame` | A frame as received; **may** contain NaN/Inf (a sensor reports a non-return that way) |
| `PointCloudFrame` | A frame whose coordinates are all finite - the contract every downstream module consumes |
| `ProcessingMetrics` / `StageMetrics` / `PointCloudProcessingResult` | Per-stage counts and measured duration for one preprocessed frame |
| `PointCloudSummary` / `PointCloudBounds` | JSON-safe metadata and axis-aligned bounds |
| `VehicleState` | Ego position, velocity, acceleration, heading, dimensions |
| `DetectedObject` | Per-frame detection; identity is frame-local |
| `TrackedObject` | Persistent track with status, kinematics, confidence, uncertainty, age |
| `PredictedTrajectory` / `TrajectoryPoint` | Predicted future path, ordered and horizon-bounded |
| `AdaptiveMapCell` / `AdaptiveMap` | 2.5D cell (occupancy, height, resolution, risk, uncertainty) and snapshot |
| `ResolutionContext` | Inputs to the future resolution decision |
| `RiskCell` / `ObjectRisk` / `RiskField` / `RiskFactors` | Normalised risk, attribution and spatial field |
| `SystemStatus` / `ComponentStatus` / `SystemMetrics` | Backend state and measured metrics |

Shared rules enforced by the base classes in `models/common.py`:

- `schema_version` on every public model.
- Timestamps are timezone-aware and normalised to UTC; naive datetimes are rejected.
- `coordinate_frame` on every spatial model (`lidar` / `ego` / `world` / `map`).
- `source` (`live_sensor` / `simulation` / `replay` / `synthetic_test` / `unavailable`) on
  everything that could originate anywhere but a sensor.
- Units are metres, m/s, m/s², radians and seconds unless the field name says otherwise.
- Unknown fields are rejected (`extra="forbid"`), so a schema drift fails loudly.

---

## 5. Provenance and honesty rules in the code

These are constraints from [`knowledge-base/20-constraints.md`](knowledge-base/20-constraints.md)
enforced by the implementation, not just by convention:

1. **`source` is mandatory on frame submission.** `POST /api/v1/lidar/frame` has no default
   for `source`; a caller must declare provenance.
2. **LiDAR status derives from provenance.** Frames labelled simulation, replay or
   synthetic make the channel report `SIMULATED`, never `CONNECTED`.
3. **Unmeasured metrics are `null`.** `SystemMetrics.unavailable` names each missing metric
   and why. GPU utilisation is always `null` in Phase 1.
4. **The mock is never a fallback.** `MockCarlaSimulatorClient` is selected only by
   `ADAPTX_CARLA__USE_MOCK=true`, logs a warning, and is flagged as `is_mock` in the API.
   A failed real connection never silently falls back to it.
5. **Telemetry declares what it does not have.** `/ws/telemetry` lists absent streams in
   `not_yet_available` rather than sending empty or invented ones.
6. **The risk baseline is labelled.** `RiskField.is_baseline` and `/api/v1/risk/status`
   report the modelled and unmodelled factors explicitly.

---

## 6. Request lifecycle

```
uvicorn → create_app()
            ├─ build_context()      construct settings + services (no I/O)
            └─ lifespan startup()   configure logging, optionally connect CARLA
                                    attach context + ConnectionManager to app.state

HTTP request → route → Depends(get_*_service) → service → model → response
                                    │
                       AdaptXError ─┴─▶ exception handler → {"error": {code, message, details}}
```

`ApplicationContext` (`core/lifecycle.py`) owns every long-lived service. Routes reach it
through FastAPI dependencies rather than module globals, so tests build an isolated context
per test.

---

## 7. LiDAR path today

Two paths, selected by the request's `preprocess` flag.

**`preprocess: false`** (default, unchanged from Phase 1)

```
POST /api/v1/lidar/frame
   → LiDARFrameRequest             JSON body, source required
   → PointCloudFrame.from_sequence structural validation + finiteness
   → FrameValidationProcessor      configured min/max point-count limits
   → LiDARIngestService            record summary, count, measured processing time
   → PointCloudSummary
```

**`preprocess: true`** (Phase 2A)

```
POST /api/v1/lidar/frame
   → RawPointCloudFrame            structural validation only; NaN/Inf permitted
   → PointCloudPreprocessor.run
        ├ validation        configured min/max point-count limits (on the RAW input)
        ├ invalid removal   drop rows with any non-finite value, count them
        ├ ROI filter        inclusive axis-aligned box, count rejects
        └ range filter      inclusive 3D Euclidean band, count rejects
   → PointCloudProcessingResult    processed frame + input summary + measured metrics
   → LiDARIngestService            pre_validated, upstream duration folded into latency
   → PointCloudSummary + ProcessingMetrics
```

Points are only ever **removed**; nothing is repaired, clamped or invented. The counts
partition the input exactly, and each stage's output is the next stage's input.

The frame is still **not** stored or interpreted: there is no detector to hand it to.

### Coordinate convention (ADR-009)

Right-handed, origin at the sensor, metres: **+x forward, +y left, +z up**. This is what
Phase 1 already implied, since `BoundingBox3D.yaw_rad` and `VehicleState.heading_rad` are
counter-clockwise about +z from +x.

CARLA uses a **left-handed** frame (+y right). Converting is the CARLA boundary's job in
Phase 9 and does not exist yet; no module currently transforms coordinates.

### Range convention (ADR-011)

`sqrt(x² + y² + z²)` from the sensor origin, **not** the ground-plane distance, because
the minimum range models a physically 3D blind zone. Bounds are inclusive at both ends.
Squared distances are compared against squared bounds to avoid a square root over the
array.

### Point-count limits apply to the input

`min_points` / `max_points` bound how large a **raw** scan may be. A frame that filtering
legitimately reduces to zero points is a valid observation ("everything was out of
range"), not malformed input, so it is accepted and reported with its metrics.

---

## 8. CARLA boundary

`CarlaSimulatorClient` (ABC) has two implementations:

- `CarlaClient` — imports the optional `carla` package lazily inside `connect()`. Without
  it, `connect()` raises `SimulatorUnavailableError` with the reason. `get_sensor_data`,
  `get_vehicle_state`, `spawn_*` and `destroy_actor` raise an explicit "not implemented in
  Phase 1" error rather than returning placeholder data.
- `MockCarlaSimulatorClient` — deterministic (seeded) fake for development and tests. All
  output is labelled `synthetic_test`.

`CarlaService.connect()` never raises: a disabled, missing or unreachable simulator is
recorded as `DISCONNECTED` with a human-readable reason, and the backend keeps running.

---

## 9. System state semantics

| State | Meaning |
|---|---|
| `RUNNING` | Every **required** component is `READY` |
| `DEGRADED` | An optional but *enabled* component is unavailable — CARLA enabled and not connected, or the LiDAR feed went stale after previously delivering frames |
| `ERROR` | A required component is `NOT_READY` |

Required in Phase 1: `api`, `configuration`, `telemetry`, `lidar_ingest`. Components
scheduled for a later phase are `PLANNED` and do not degrade the state — their absence is
expected, not a fault.

---

## 10. Extension points

Each future module plugs in behind an existing contract, with no change to the API,
services or models:

| To add | Implement | Then wire in |
|---|---|---|
| More point-cloud processing (Phase 2B) | `perception.interfaces.LiDARProcessor` | chain after `PointCloudPreprocessor` in `build_context()` |
| Object detection (Phase 3) | `perception.interfaces.ObjectDetector` | a detection service consuming ingested frames |
| Tracking (Phase 4) | `tracking.interfaces.ObjectTracker` | a tracking service consuming detections |
| 2.5D mapping (Phase 5) | `mapping.interfaces.AdaptiveMapper` | map service; set `AdaptiveMap.is_adaptive` per variant |
| Risk (Phase 6) | `risk.interfaces.RiskEngine` | replace `BaselineProximityRiskEngine` in `ApplicationContext`; keep the baseline for comparison |
| Adaptive resolution (Phase 7) | `mapping.interfaces.ResolutionController` | consumed by the adaptive mapper |
| Prediction (Phase 8) | `prediction.interfaces.TrajectoryPredictor` | prediction service feeding predicted risk |

When a module becomes real, update its row in `_declared_components()`
(`services/system_service.py`) and add its stream to the telemetry payload, removing it
from `not_yet_available`.

---

## 11. Deliberate non-decisions

Phase 1 does **not** introduce a database, a message broker, a task queue, an ML framework
or a cloud service. None is needed to ingest, validate and report on a frame, and adding
one now would fix an architecture before the requirement is understood
(`knowledge-base/20-constraints.md`: no unnecessary frameworks).

Open3D is declared as an optional extra (`pip install -e ".[pointcloud]"`) and is **not
installed**: Phase 1 performs no geometric processing. Phase 2 should install it and record
the decision.
