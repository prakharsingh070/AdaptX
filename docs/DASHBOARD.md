# Dashboard (Phase 12)

A local engineering console that makes ADAPT-X observable: what the pipeline is doing now,
what it perceives, where the adaptive map spends its detail, and what the Phase 11
evaluation measured — including everything it measured badly.

> **This is a window into ADAPT-X, not another brain inside it.** The dashboard computes no
> risk, no trajectory, no resolution, no match and no metric. It renders what the backend
> produced and what a stored evaluation report says. Every figure it shows is simulation or
> synthetic evidence labelled as such; nothing here is a safety claim, a collision
> probability or a real-world result.

---

## 1. Purpose

Answer, visually, from data the system already produces:

| Question | Where it is answered |
|---|---|
| What is the system doing now? | Overview, header status pills |
| What objects are perceived, where, with what risk and uncertainty? | Live Scene, Object Inspector |
| What paths are predicted? | Live Scene (predicted path overlay), Inspector |
| What does the spatial map look like; how is detail allocated? | Spatial Map, Adaptive Resolution |
| What did the evaluation measure, including the failures? | Evaluation, Fixed vs Adaptive |
| What happened frame by frame? | Playback of a recorded run |
| Where does the system fail? | Every view: null is "Not available", UNKNOWN is UNKNOWN, negative results stay on the front page |

The visual target is [`design/dashboard-target.png`](design/dashboard-target.png) and
[`UI_UX.md`](UI_UX.md): a dark operational console, left rail, large centre viewport,
right-hand status columns, mid-row map cards, bottom-row stage cards. **The look is copied;
the numbers are not.** Every figure in the mockup that the backend cannot produce (detection
accuracy, collision probability, GPU, routing/decision/planning, camera, radar) renders as
*Not implemented* or *Not measured* in the same card footprint.

## 2. Technology

**Static HTML, CSS and vanilla JavaScript (ES modules), served by the existing FastAPI
application at `/dashboard`. No build step, no npm, no framework, no new Python
dependency.** Canvas 2D for the scene and maps (a dense map has tens of thousands of cells;
one DOM element per cell is ruled out). Frontend unit tests run under Node's built-in
`node --test` runner, which is already installed and needs no package.

Why not React/Vite: nothing in the repository uses them; the dashboard has eight views and
one data layer; the project's rule is the fewest dependencies that do the job, and a build
pipeline would be the largest dependency Phase 12 could add. Why not a chart library: the
charts are bars, lines and grids over small arrays; Canvas draws them in a few dozen lines
and the values are then guaranteed to be the report's, untransformed.

## 3. Architecture

```
                 ┌────────────────────────── backend (FastAPI, existing) ──────────────────────────┐
                 │                                                                                  │
  scenario CLI   │  POST /api/v1/lidar/adaptive-map ──► pipeline ──► SceneService.publish(snapshot)  │
  --publish ───► │  POST /api/v1/scene/frame        ─────────────────────────────┘        │        │
  (own pipeline) │                                                                        ▼        │
                 │  GET  /api/v1/scene/latest, WS /ws/scene  ◄──── latest SceneSnapshot (+sequence) │
                 │  WS   /ws/telemetry (existing)     ◄──── status + metrics + summaries             │
                 │  GET  /api/v1/system/status, /carla/status, /map/status, /risk/status (existing)  │
                 │  GET  /api/v1/reports, /reports/{name}, /reports/compare   ◄── EvaluationReport   │
                 │  GET  /api/v1/runs, /runs/{name}, /runs/{name}/frames/{i}  ◄── ScenarioRunResult  │
                 │  GET  /dashboard/  (static files)                                                 │
                 └──────────────────────────────────────────────────────────────────────────────────┘
                                                        │
                                                        ▼
                     dashboard/ (static)   data/ providers ──► normalised view models ──► views
```

Dependency direction is one way: pipeline → models → API / stored files → dashboard. The
dashboard never imports from the backend and the backend never depends on the dashboard;
the new endpoints are read-only except one ingest path (`POST /api/v1/scene/frame`) that
accepts what a pipeline *already produced* for display and computes nothing.

### 3.1 Two modes, never mixed

| Mode | Source | Label shown | What it is |
|---|---|---|---|
| **LIVE** | `/ws/scene` + `/ws/telemetry` | `LIVE` with last-update time and a stale flag | The most recent frame the backend processed or was handed |
| **STORED EVALUATION** | `/api/v1/reports/*` and `/api/v1/runs/*` | `STORED EVALUATION` with scenario, seed, commit | A Phase 11 report and, for playback, the recorded run it came from |

The header carries the mode. Live panels never draw stored numbers; evaluation panels never
draw live ones. Ground truth appears only in evaluation views, labelled *Ground truth*.

### 3.2 Where the live scene comes from

The backend runs perception only when asked. Two things ask:

1. **Any client posting frames** to `POST /api/v1/lidar/adaptive-map` — the existing full-chain
   endpoint. After the chain runs, the endpoint hands a `SceneSnapshot` (the tracks,
   trajectories, assessments, tile decisions, map summaries, comparison, and a downsampled
   point sample of the processed frame) to the new `SceneService`, which keeps only the latest
   and bumps a sequence number.
2. **A scenario run** started from the existing CLI with a new flag:
   `python -m adaptx.scenarios run vehicle_approach --publish http://127.0.0.1:8000`. The
   runner processes each frame in its own pipeline exactly as before (the record it writes
   is unchanged) and, per frame, POSTs a `SceneSnapshot` built from its outputs to
   `POST /api/v1/scene/frame`. Ground truth is never in a snapshot: the publisher is a frame
   observer that receives the sensor frame and the pipeline outputs and nothing else.

`/ws/scene` pushes the latest snapshot whenever the sequence changes. The dashboard does
not poll perception; it never causes a frame to be processed; it has no start/stop/spawn
control, and the CLI remains the only way to run a scenario (§8).

### 3.3 The point cloud

The scene snapshot carries a **point sample**: at most `max_points` (default 6,000) of the
processed frame's points, chosen by a fixed stride so the sample is deterministic, rounded
to centimetres, with the original count beside it and `is_downsampled` set. That is what the
3-D and top-down viewports draw. Recorded runs carry no points (the Phase 10/11 record holds
counts and results, not clouds), so playback shows objects, paths, risk and tiles without a
cloud and says so.

### 3.4 Live simulation session (post-Phase-12 extension, ADR-056)

The third way a live scene arises, and the one the console is built around now:

```
CARLA ──► CarlaSimulationSession (drive_ego, LiDAR, collision sensor, RGB camera)
            │  session.step()            one tick, one LiDAR sweep
            ▼
        RawPointCloudFrame(source=simulation)
            ▼
        pipeline_processor(context)     the SAME Phase 2-8 chain as the endpoints
            ▼
        RiskGovernedSpeedPolicy.decide(risk, tracking, prediction, ego odometry)
            ▼
        session.apply_ego_control(throttle, brake, steer)   ──► CARLA vehicle
            ▼
        SceneSnapshot(+ ego, control, live timing) ──► SceneService ──► /ws/scene ──► browser
```

`LiveSimulationService` (`adaptx.live`) owns the session on one thread and is the only
thing that ticks the world. A **live scenario** (`adaptx.live.scenarios`) is a running
scene: seeded Traffic Manager traffic plus timed scripted actors placed relative to the
ego's pose *at the moment they appear* and moved in that frame - so an obstacle 45 m
ahead stays where it was put while the ego drives up to it, and leaves after its
lifetime. Six are in the catalogue: `static_obstacle` (the obstacle-stop demo),
`pedestrian_crossing`, `cyclist_crossing`, `vehicle_cut_in`, `random_urban_traffic`,
`mixed_obstacles`. The seed is chosen at start and shown in the header.

The controller (`adaptx.control`) is a **baseline speed governor**, not an
autonomous-driving controller: a target speed per risk level among objects in the ego's
corridor, a hold inside a safe distance, full brake inside an emergency distance, a
resume dwell, and steering that follows the map's lane centre. It reads the risk,
tracking and prediction outputs and the ego's own odometry. It reads **no ground truth**
about other actors; neither does anything else in the loop. The collision sensor is a
safety fallback that ends the session and records the contact.

Ground truth stays out of the live snapshot exactly as before: the contract has no field
for it, and the loop never calls `session.ground_truth()`.

### 3.5 The object record (live perception upgrade, ADR-057)

Every live snapshot carries one `TrackedObjectSnapshot` per track, built in the backend
from the track (Phase 4), its risk assessment (Phase 7) and its predicted path (Phase 5),
joined by `track_id`:

| Field | Source | Meaning |
|---|---|---|
| `object_class`, `tracking_state`, `hits`, `age_frames` | tracker | the geometric classifier's label with the tracker's hysteresis and decay; UNKNOWN stays UNKNOWN |
| `distance_m` | risk assessment | planar (XY) distance from the ego reference; null when unassessed |
| `longitudinal_distance_m`, `lateral_distance_m` | track position | `x` (ahead +) and `y` (left +) in the sensor frame, +X forward, +Y left (ADR-009) |
| `speed_mps` | tracker | scalar of the tracked velocity, **ego-relative**; null until measured |
| `relative_speed_mps` | risk assessment | closing speed, positive = approaching |
| `risk_level`, `risk_score` | risk engine | the object's own level; score null when UNKNOWN |
| `path_relation`, `in_ego_path` | `control.corridor` | IN_PATH / CROSSING / BEHIND / OUTSIDE against the ego corridor (`ADAPTX_CONTROL__PATH_HALF_WIDTH_M`), the same rule that governs the speed |
| `confidence` | detector via tracker | geometric fit score, **not a probability**; null for UNKNOWN |
| `predicted_horizon_s`, `predicted_points` | predictor | the constant-velocity path, if one exists |

The Detected Objects cards, the tracking table's path column, the scene labels
(`#12 VEHICLE 14.8m HIGH` with `IN PATH` beneath) and the inspector's top block all read
this record. Recorded runs predate it and carry none; playback shows the raw contracts.

**Classification, honestly.** Classes come from dimension bands (`perception/
classification.py`, now `geometric_bands_v2`): PEDESTRIAN 0.9-2.2 m tall and under 1.2 m
across; CYCLIST 0.6-2.0 m tall, 1.2-2.6 m long, under 1.0 m wide; VEHICLE 1.0-2.6 m tall,
1.3-2.6 m across, 1.5-6.5 m along; OBSTACLE under 1.0 m tall. Anything fitting no band
or several is UNKNOWN. The detector first drops clusters whose bottom is more than 0.8 m
above the road the ground stage found (overhead signs, foliage - measured to be what the
old "pedestrians" were), and the tracker drops a label after three UNKNOWN observations.
Known confusions remain: a bus shelter's side reads VEHICLE; a bollard reads PEDESTRIAN;
a riderless CARLA bicycle is mostly UNKNOWN/OBSTACLE (Experiment 015).

## 4. Data contracts consumed

| Contract | Owner | Used by |
|---|---|---|
| `SystemStatus`, `SystemMetrics`, `ComponentStatus` | Phase 1 | Overview, header, stage cards |
| `SimulationSessionStatus`, `CarlaStatus` | Phase 9 | Header pill, Run info |
| `TrackedObject`, `TrackingResult` | Phase 4 | Scene, Inspector |
| `PredictedTrajectory` | Phase 5 | Scene overlay, Inspector |
| `RiskAssessment` (score, level, uncertainty, factors, map context, trajectory relevance) | Phase 7 | Inspector, risk panel |
| `SpatialMapSummary`, `AdaptiveSpatialMapSummary`, `TileResolutionDecision`, `MappingComparison` | Phases 6, 8 | Spatial Map, Adaptive Resolution |
| `SceneSnapshot` (new, Phase 12: the above bundled per frame + point sample) | Phase 12 | Live Scene |
| `SceneSnapshot.ego` (`VehicleState`), `.control` (`ControlCommand`), `.live` (`LiveFrameInfo` with measured `LiveTiming`) | live extension | header pills, System Status, Decision / Control, Performance |
| `LiveStatus`, `LiveEvent`, `LiveScenarioSummary` | live extension | rail controls, Simulation / Map Info, Recent Events |
| `ScenarioRunResult` with `frames[].outputs` | Phases 10, 11 | Playback, Run info |
| `EvaluationReport`, `ComparisonReport` | Phase 11 | Evaluation, Fixed vs Adaptive |

The frontend `data/` layer normalises these into view models with exactly three
transformations allowed: unit formatting, null → "Not available" / UNKNOWN → "UNKNOWN",
and metres → pixels. It performs no aggregation, no averaging, no matching, no thresholding.
A test inspects the frontend source for arithmetic over report fields.

## 5. Views

| # | View | Content | Mode |
|---|---|---|---|
| 1 | Overview | pipeline strip, system state (risk level in path and in scene, measured FPS, pipeline latency, sim/wall speed with LAGGING), Recent Events from the live loop, objects by class, Simulation / Map Info (server, map, session, scenario, seed, sim frame/time, ego speed, actors, collisions), Adaptive 2.5D Map, Risk Map (per-tile risk factor as the controller recorded it), Object Tracking & Prediction (nearest 12), stage cards incl. **Decision / Control (baseline)** and **Performance (measured)**, Routing and Planning as Not implemented, latest evaluation summary | LIVE + link to stored |
| 2 | Live Scene | 3-D perspective and top-down canvases of the point sample with boxes, track ids, class, velocity vectors, predicted paths, risk colour; **Front View** = the ego's RGB camera, one PNG per live frame from the backend (display only); pan / zoom / fit / toggles; axes labelled +X forward, +Y left | LIVE or playback |
| 3 | Object Inspector | every `TrackedObject` + `RiskAssessment` + `PredictedTrajectory` field for the selected track id; nulls as "Not available"; UNKNOWN distinct | LIVE or playback |
| 4 | Spatial Map | fixed map summary (bounds, resolution, cells, occupancy ratio, bytes, build time) and the adaptive tile grid with each tile's actual cell size; legend distinguishes occupied / observed-empty / out-of-bounds / unobserved; **no per-cell occupancy in live mode** (the grid is not streamed) | LIVE or playback |
| 5 | Adaptive Resolution | fixed vs adaptive per frame from the report: cells, bytes, build time, controller overhead, tile resolution distribution, actor-tile vs other-tile resolution, resolution by risk level, refinement lead, churn, reversals, holds, budget | STORED |
| 6 | Evaluation | header (scenario, seed, CARLA, commit, gates, primary gate), cards per section with status and reason, tables and small charts, limitations | STORED |
| 7 | Fixed vs Adaptive | dedicated comparison charts from the same report; two-report `ComparisonReport` from the Phase 11 `compare` contract | STORED |
| 8 | Run / Scenario | run metadata, sensor configuration, pipeline configuration, frame list; playback controls | STORED |

Mockup panels with no backend source keep their card and read *Not implemented* (Routing,
Decision, Planning, Camera, Radar, Logs retrieval, Export Report) or *Not measured* (GPU).

## 6. Screen-space transformation

ADAPT-X is right-handed: +X forward, +Y left, +Z up (ADR-009). The top-down canvas maps
`+X → screen up` and `+Y → screen left`, so "ahead" is up and "left" is left, with the ego at
the origin and an axis rose drawn in the corner. The 3-D view is a fixed perspective camera
behind and above the ego looking along +X; it is a presentation projection (documented in
`render/projection.js`), not a physical camera model. No other transformation is applied to
positions.

## 7. Playback

A recorded run is served frame by frame (`GET /api/v1/runs/{name}/frames/{i}`), so the
browser never parses an 80 MB record at once. Controls: first, previous, play/pause, next,
last, slider. Each frame shows the frame id, simulation time, the frame's outputs and the
selected object; nothing is recomputed and the record is never modified (the backend loads
it read-only and serves copies).

## 7b. Modes and session controls

The rail has an explicit **LIVE SIMULATION / STORED EVALUATION** switch. Live is the
default. In live mode the viewport draws the scene channel and nothing else; when CARLA
is disconnected the header says `CARLA DISCONNECTED` and the viewport says so - **no data
is invented and nothing switches to a recording**. Stored evaluation is chosen
explicitly; it plays a recorded run back frame by frame and is labelled on every panel.

Live session controls (rail): scenario, seed, **Start / Pause / Resume / Stop / Reset**.
They call the five allowlisted `POST /api/v1/live/*` endpoints and nothing else. Pause
stops ticking - the world freezes. Stop destroys every actor the session spawned and
restores the world settings. Reset stops and starts the same scenario and seed. The
backend can refuse all five with `ADAPTX_LIVE__CONTROLS_ENABLED=false`.

## 8. Security and access assumptions

Local engineering tool. The backend binds where `ADAPTX_API__HOST/PORT` say (default
`127.0.0.1:8000`); there is no authentication, no analytics, no external request of any kind.
Report and run files are read from directories the operator configures
(`ADAPTX_DASHBOARD__REPORT_DIR`, `ADAPTX_DASHBOARD__RUN_DIR`, default `reports/` and `runs/`);
names are restricted to plain file names inside those directories, so no path escapes them.
`POST /api/v1/scene/frame` trusts the local process posting to it, as every existing POST
already does. **No endpoint spawns, destroys, moves, ticks or drives an actor.** The five
session controls (`/api/v1/live/start|pause|resume|stop|reset`) are high-level, local-only,
allowlisted by full path in the route audit, and switchable off; the scenario layer inside
the process is what touches actors (ADR-056).

## 9. Deliberately not computed by the dashboard

Risk scores · uncertainty · trajectories · resolution decisions · matching · ADE / FDE ·
recall · match rate · continuity · concordance · churn · any ratio between fixed and adaptive
· any aggregate over frames · occupancy from detections · an "efficiency", "safety" or "AI"
score. If a number is not in a backend payload or a stored report, the dashboard does not
show it.

## 9b. Deliberately not done in the live extension

Routing, planning, lane changes, a spatial risk field, ego-motion compensation, camera
perception, GPU measurement, anything real-time by claim. The steering follows the map's
lane centre (road geometry from CARLA waypoints), which is not planning and is labelled
so in the Routing card.

## 10. Limitations

- Live mode shows the most recent frame only; there is no server-side history. A dashboard
  opened after a scenario finished sees that scenario's last frame, labelled with its time,
  until something else runs the pipeline.
- No point cloud in playback (records carry none). The live point sample is a stride, so a
  thin object can lose returns in the picture that the pipeline itself saw.
- Per-cell occupancy is not available live (the grid is not streamed); the spatial map view
  shows tile resolutions and summaries. The full grid can be fetched per request from
  `POST /api/v1/lidar/map` by an operator, not by the dashboard.
- No spatial risk field exists (`risk_field` is in the telemetry `not_yet_available` list) and
  the Risk Map card says so.
- The backend's handlers are synchronous: a random seek during playback waits behind the
  frames being prefetched (about 200 ms measured, Experiment 013), and the first selection
  of a 42 MB run costs the backend ~3 s to parse it once.
- Browser rendering performance is measured separately from the pipeline (Experiment 013)
  and says nothing about ADAPT-X's speed.
- The frontend boundary test is a source scan for arithmetic and metric names, not a proof.
- **The live loop runs at about 0.42x wall-clock speed** on the validation machine
  (Experiment 015: ~120 ms per 50 ms frame, of which ~85 ms is the pipeline; it was 0.31x
  before the tile-geometry memoisation). The header says `PIPELINE LAGGING` with the
  measured ratio; the simulation is consistent, just slow. With the dashboard open in the
  same process it is slower still.
- Velocities are ego-relative (no ego-motion compensation): with the ego moving, every
  static object reports a velocity of minus the ego's, and the risk engine's closing-speed
  factor rises for all of them. The controller keys on in-path objects to contain this.
- The controller's corridor is straight along +X; on a bend, roadside geometry enters it
  and the ego holds until it clears. The obstacle-stop demo runs on the straight from
  spawn point 1.
- A killed backend (not a stopped session) leaves its actors on the CARLA server until
  the **next session opens**: `open()` now reclaims actors ADAPT-X tagged (`ego`,
  `traffic`, `adaptx_scenario`) and their sensors, and releases a synchronous mode the
  dead process left (Experiment 016). Orphaned Traffic-Manager cars stand at 0 m/s - if
  the dashboard shows a stationary VEHICLE in the lane at session start, that is what it
  was; the controller stopping for it is correct. Other clients' actors are never touched.
- Classification is geometric (ADR-057): a parked car is VEHICLE from ~12 m and OBSTACLE
  beyond; a walker is PEDESTRIAN in about 70 % of its tracked frames and UNKNOWN when its
  cluster merges with furniture; a riderless bicycle is mostly not a CYCLIST and switches
  identity while crossing; a bollard can read PEDESTRIAN (Experiment 015). Labels and
  fit scores are shown as produced.
- Not production software: no auth, no persistence beyond a per-tab memory of which report,
  run and overlays were chosen, no multi-user state.

## 11. Running it

Prerequisites: the project's Python environment (nothing new), a browser, and - only for
the frontend unit tests - Node 22.

```bash
python -m uvicorn adaptx.api.app:create_app --factory --host 127.0.0.1 --port 8000
```

Open <http://127.0.0.1:8000/> (redirects to `/dashboard/`). The header pills say whether the
backend, CARLA, telemetry and the scene channel are reachable; each channel reconnects on
its own and shows "stale" or "disconnected" with the last time it heard anything.

**Live simulation (the console's main mode).** Start the backend from the environment that
has the `carla` wheel and point it at the server:

```bash
ADAPTX_CARLA__ENABLED=true ADAPTX_CARLA__HOST=127.0.0.1 ADAPTX_CARLA__EGO_SPAWN_INDEX=1 ADAPTX_LIDAR__GROUND_ENABLED=true .venv312/Scripts/python.exe -m uvicorn adaptx.api.app:create_app --factory --host 127.0.0.1 --port 8000
```

Open the console, confirm the header says `CARLA CONNECTED`, choose a scenario and a seed
in the rail and press **Start**. The ego spawns, the LiDAR streams, the pipeline runs on
every frame, the controller drives, and every panel follows. `Stop` destroys every actor
and restores the world. The obstacle-stop demonstration is `static_obstacle`: the parked
car appears at 1 s, the ego slows, holds ~7.7 m short of it, and resumes when the car
leaves at 23 s (Experiment 014). `ADAPTX_CARLA__EGO_SPAWN_INDEX=1` is the validated
straight on Town10HD_Opt; `ADAPTX_LIDAR__GROUND_ENABLED=true` is the configuration under
which the parked car is detected at all (Experiment 012).

**Other live producers.** Either post frames to `POST /api/v1/lidar/adaptive-map` or run a
scenario against CARLA and publish its frames as they are processed:

```bash
python -m adaptx.scenarios run cyclist_crossing --publish http://127.0.0.1:8000
```

The CLI prints how many frames it published; a backend that is down never fails the run
(the publisher gives up after three consecutive failures and says so).

**Stored evaluation.** Put `EvaluationReport` JSON files in `reports/` and
`ScenarioRunResult` JSON files in `runs/` (both gitignored; override with
`ADAPTX_DASHBOARD__REPORT_DIR` / `ADAPTX_DASHBOARD__RUN_DIR`), then pick them in the left
rail. A file that is not a valid report or run is refused with the validation error shown in
the rail. Choosing a run switches the viewport to **STORED EVALUATION · PLAYBACK** with
first / previous / play / next / last and a slider; choosing "— live —" returns to the live
scene. The chosen report, run and overlay toggles survive a refresh; the view lives in the
URL hash.

**Tests.**

```bash
pytest tests/unit/test_scene_service.py tests/unit/test_evidence_service.py tests/integration/test_dashboard_api.py
```

```bash
cd dashboard && node --test tests/*.test.mjs
```

**Debug hook.** `window.adaptxDebug` exposes the two renderers and `currentScene()` for
manual inspection from the browser console (read-only use; it is how Experiment 013 timed
the draws).
