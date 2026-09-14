# ADAPT-X

**Adaptive Dynamic Perception and Tracking** — a LiDAR-based perception framework whose
goal is to allocate spatial map resolution according to environmental risk and uncertainty
instead of spending it uniformly. Think of it as a perception system that asks, frame by
frame: **“Where does detail matter most right now?”**

The intended behaviour is easy to picture: 🌤️ open road gets a coarser map, while 🚶
pedestrians, 🚗 vehicles, obstacles, motion and uncertainty attract finer detail. The
allocation is implemented as a deterministic, explainable heuristic baseline. It is a
research prototype, not a collision probability or a safety margin.

## Follow one frame

Imagine a LiDAR frame arriving at an intersection. ADAPT-X walks it through a small,
observable story:

```text
raw points → 🧹 clean and filter → 🔎 detect shapes → 🧭 track motion
           → 🔮 predict trajectories → ⚠️ assess risk → 🗺️ allocate map detail/.
```

An open region may stay at a larger cell size, while a region near a moving object receives
smaller cells. The dashboard then shows the scene the backend produced, including the
objects, map tiles and measured timings. Nothing is guessed in the browser.

Try the first checkpoint after starting the API:

```bash
curl http://localhost:8000/api/v1/system/status
```

For a full example, submit a frame to the pipeline, then inspect the result:

```bash
curl -X POST http://localhost:8000/api/v1/lidar/risk \
  -H "Content-Type: application/json" \
  -d '{"frame_id":0,"sensor_id":"demo-lidar","source":"synthetic_test","points":[[4.0,0.2,0.8],[4.2,0.3,0.9],[4.1,0.1,0.7]],"timestamp":"2026-09-14T12:00:00Z"}'
```

That response is the system’s current explanation of the frame: detected and tracked
objects, available risk factors, uncertainty reasons, and measured stage timing. Missing
evidence stays missing; it is never silently turned into a zero.

---

## Implementation status

> **Phases 1 to 12 are implemented** — foundation, LiDAR processing and benchmarking,
> geometric object detection, temporal tracking, trajectory prediction, 2.5D spatial
> mapping, object-level risk and uncertainty, adaptive resolution, CARLA integration,
> scenario generation, offline evaluation and the dashboard. Each is a deterministic,
> explainable baseline, not a finished production subsystem.
> The **adaptive resolution algorithm** exists as of Phase 8 as a deterministic heuristic
> baseline. Its detail priority is an engineering prioritisation score — **not** a probability
> of collision, not a safety margin, never calibrated and never validated, because no labelled
> data exists. Whether the allocation is *appropriate* is unmeasured and unmeasurable; only
> what it costs has been measured (Experiment 007), and that comparison is **not** a clean
> win — see the entry rather than assuming one.
>
> **No accuracy figure appears anywhere in this repository**, because no labelled data
> exists to measure one against. Detection, tracking and prediction are benchmarked for
> **speed only**.

| Subsystem | State | Notes |
|---|---|---|
| Configuration, logging, API, telemetry | **Implemented** | Phase 1 |
| LiDAR ingest | **Partial** | Structural validation, point count, bounds, metadata. |
| LiDAR pipeline | **Partial** | Validation, NaN/Inf removal, ROI and range filtering; opt-in voxel downsampling, baseline ground segmentation and baseline noise filtering; orchestration with per-stage measured timing. No clustering, no coordinate transforms. |
| Benchmarking (pipeline speed) | **Implemented** | Deterministic synthetic datasets and a fixed-resolution baseline. Speed only — not the Phase 11 ADAPT-X evaluation. |
| Risk and uncertainty | **Partial** | Phase 7: deterministic heuristic object-level risk from proximity, rate of approach and predicted approach, with uncertainty reported separately. **Not a probability of collision** — not calibrated, never validated. No time-to-collision, no spatial risk field. Proximity-only baseline retained for comparison. |
| CARLA | **Boundary only** | Connection + world info. Optional dependency; the backend runs without it. |
| Object detection | **Partial** | Phase 3: geometric clustering and baseline size-based classification. No trained model, no oriented boxes, no velocity. |
| Tracking | **Partial** | Phase 4: gated nearest-neighbour association, measured velocity, track lifecycle. No learned model, no re-identification. |
| Trajectory prediction | **Partial** | Phase 5: deterministic constant-velocity baseline with heuristic uncertainty. No acceleration model, no Kalman filter, no learned model, no map conditioning. Accuracy unmeasured. |
| 2.5D mapping | **Partial** | Phase 6: deterministic frame-local fixed-resolution grid with binary occupancy and per-cell height statistics. One cell size everywhere — deliberately, because it is the baseline the adaptive mapper is measured against. No temporal fusion, no occlusion, no SLAM. |
| CARLA simulation | **Partial** | Phase 9: CARLA is an upstream **data source**, not a second perception stack. Deterministic synchronous stepping, one coordinate conversion at the boundary, simulation-authoritative timestamps, and ground truth on a separate path that never reaches perception. Optional - the backend and test suite run without it. Live-validated against CARLA 0.9.16 from a Python 3.12 environment (no wheel exists for 3.13). |
| Scenario framework | **Partial** | Phase 10: declarative, seeded, reproducible scenario definitions with timed scripted motion, run against the CARLA boundary with ground truth recorded beside every frame and fed to no pipeline stage. Four catalogue scenarios. **Records evidence, computes no accuracy.** Live-validated; placed actors do not simulate physics (ADR-054). Event replay deferred. |
| Evaluation | **Partial** | Phase 11: offline evaluation of a recorded run against simulator ground truth — detection/tracking at several gates, ADE/FDE, risk against proximity events, map workload (accuracy **not** evaluated), adaptive vs fixed paired within one run. Missing metrics are null with a reason. **Measured (Experiments 011–012): the baselines lost.** Under the process defaults a parked car and a pedestrian are never detected (their returns cluster with the road; ground segmentation is off by default); with it on both are seen every frame, at a constant 1.7 m centroid offset for the car. Adaptive map at 0.5–0.7 of the fixed cells and 5–8× the build time, finer under the actor. Simulation evidence only; not safety or real-world validation. |
| Live perception (object records) | **Partial** | Post-Phase-12 upgrade (ADR-057): every live object carries a backend-built record — class, distance, longitudinal/lateral, ego-relative speed, risk, IN PATH / CROSSING / BEHIND / OUTSIDE, fit-score confidence — shown as cards, scene labels and the inspector. Measured fixes: elevated-cluster filter, label decay, partial-view vehicle band. **Measured live:** parked car VEHICLE with a stable id, walker PEDESTRIAN ~70 % of frames; a riderless bicycle is mostly not a CYCLIST. Loop 120 ms (0.42× wall-clock). |
| Live simulation + vehicle control | **Partial** | Post-Phase-12 extension: a long-lived CARLA session behind the dashboard drives the ego from the pipeline's own outputs with a **baseline** speed governor (in-path risk level → target speed, safe-distance hold, emergency brake); seeded traffic and scripted actors; collision sensor as safety fallback; ego camera for display. Reads no ground truth. **Measured live:** stops ~7.7 m short of a parked car and resumes when it leaves, 0 collisions, ~0.3× wall-clock speed (shown as LAGGING). Not autonomous driving, not safe by claim. See [`docs/DASHBOARD.md`](docs/DASHBOARD.md) §3.4 and ADR-056. |
| Dashboard | **Partial** | Phase 12: a static browser console served at `/dashboard` — live 3-D / top-down scene of what the pipeline last produced, object inspector, spatial map, adaptive resolution, and a viewer for stored Phase 11 reports and recorded runs with frame playback. **Computes nothing** (every number is a backend field; a test scans the source), controls nothing in the simulator, shows ground truth only in evaluation playback. Routing / decision / planning / risk field: *Not implemented*; GPU: *Not measured*. See [`docs/DASHBOARD.md`](docs/DASHBOARD.md). |
| Adaptive resolution | **Partial** | Phase 8: a controller allocates a cell size per **region** from risk, uncertainty, predicted-motion relevance, density, proximity and motion, and a tiled mapper applies it — so one map holds several resolutions. Stabilised against oscillation by asymmetric hysteresis plus a minimum dwell time. **Not** a probability; no learned policy, no ego planned path, no per-cell risk field. The fixed-resolution mapper is retained unchanged as the baseline. |

The running system reports this itself at `GET /api/v1/system/status`; each component
carries a readiness (`READY` / `NOT_READY`) **and** an implementation status
(`IMPLEMENTED` / `PARTIAL` / `PLANNED` / `MOCK`).

No performance figure in this repository is fabricated. `GET /api/v1/system/metrics`
returns only values measured by the running process; anything not measured is `null` and
is named in the `unavailable` list.

---

## Requirements

- Python 3.11 or newer (developed and verified on 3.13)
- Optional: Docker, for the containerised backend
- Optional: a CARLA 0.9.x server, for simulation work in a later phase

---

## Quick start

```bash
python -m venv .venv
```

```bash
.venv\Scripts\python.exe -m pip install -e ".[dev]"
```

```bash
copy .env.example .env
```

```bash
.venv\Scripts\python.exe -m adaptx
```

On Linux or macOS use `.venv/bin/python` in place of `.venv\Scripts\python.exe`.

The API then serves on <http://localhost:8000>, with interactive documentation at
<http://localhost:8000/docs>.

---

## Common commands

| Task | Command |
|---|---|
| Install (with dev tools) | `.venv\Scripts\python.exe -m pip install -e ".[dev]"` |
| Run the dev server | `.venv\Scripts\python.exe -m adaptx` |
| Run the dev server (explicit) | `.venv\Scripts\python.exe -m uvicorn adaptx.api.app:create_app --factory --reload` |
| Run all tests | `.venv\Scripts\python.exe -m pytest` |
| Unit tests only | `.venv\Scripts\python.exe -m pytest tests/unit` |
| Lint | `.venv\Scripts\python.exe -m ruff check .` |
| Format | `.venv\Scripts\python.exe -m ruff format .` |
| Check formatting only | `.venv\Scripts\python.exe -m ruff format --check .` |
| Type-check | `.venv\Scripts\python.exe -m mypy` |
| Benchmark the LiDAR pipeline | `.venv\Scripts\python.exe -m adaptx.benchmark` |
| Benchmark a stage | `.venv\Scripts\python.exe -m adaptx.benchmark --detect` (or `--track`, `--predict`, `--map`, `--risk`) |
| Evaluate a recorded scenario run | `.venv\Scripts\python.exe -m adaptx.evaluation evaluate runs/approach.json --json reports/approach.json` |
| Check two evaluations agree | `.venv\Scripts\python.exe -m adaptx.evaluation compare reports/a.json reports/b.json` |
| Run, record and evaluate against CARLA | `.venv312\Scripts\python.exe -m adaptx.evaluation run vehicle_approach --json runs/approach.json --report reports/approach.json` |
| Open the dashboard | start the dev server, then open <http://127.0.0.1:8000/> (redirects to `/dashboard/`) |
| Run the backend with CARLA for the live console | `set ADAPTX_CARLA__ENABLED=true& set ADAPTX_CARLA__EGO_SPAWN_INDEX=1& set ADAPTX_LIDAR__GROUND_ENABLED=true& .venv312\Scripts\python.exe -m uvicorn adaptx.api.app:create_app --factory` then open <http://127.0.0.1:8000/>, pick a scenario, press **Start** |
| Stream a recorded-style scenario into the dashboard | `.venv312\Scripts\python.exe -m adaptx.scenarios run cyclist_crossing --publish http://127.0.0.1:8000` |
| Frontend unit tests | `cd dashboard && node --test tests/*.test.mjs` (Node 22, no package) |
| Build the Docker image | `docker compose build` |
| Start the Docker environment | `docker compose up` |

Convenience wrappers are available: `scripts/dev.ps1 <task>` on Windows and
`scripts/dev.sh <task>` elsewhere, where `<task>` is one of `install`, `run`, `test`,
`lint`, `format`, `format-check`, `typecheck`, `check`, `docker-build`, `docker-up`.

---

## API

| Method | Path | Purpose |
|---|---|---|
| GET | `/health` | Liveness probe |
| GET | `/api/v1/system/status` | Backend state and per-subsystem implementation status |
| GET | `/api/v1/system/metrics` | Measured runtime metrics |
| GET | `/api/v1/carla/status` | CARLA connection state |
| POST | `/api/v1/lidar/frame` | Submit a point-cloud frame for validation, optionally preprocessed |
| POST | `/api/v1/lidar/detect` | Process a frame and detect objects in the non-ground points |
| POST | `/api/v1/lidar/track` | Process, detect and track across frames (**stateful**) |
| POST | `/api/v1/lidar/predict` | Process, detect, track and predict trajectories (**stateful**) |
| POST | `/api/v1/lidar/map` | Process a frame and build a 2.5D spatial map (frame-local) |
| POST | `/api/v1/lidar/risk` | Run the whole chain and assess object-level risk (**stateful**) |
| POST | `/api/v1/tracking/reset` | Drop all tracks and restart identifiers |
| GET | `/api/v1/tracking/status` | Current tracking state |
| GET | `/api/v1/map/status` | Mapper, applied resolution, mapped extent and dimensions |
| GET | `/api/v1/risk/status` | Risk engine, thresholds, modelled factors and score semantics |
| GET | `/api/v1/prediction/status` | Predictor, horizon and uncertainty semantics |
| WS | `/ws/telemetry` | Live system status, measured metrics and perception summaries |
| GET | `/api/v1/scene/latest` | The last scene the pipeline produced, for the dashboard (Phase 12) |
| POST | `/api/v1/scene/frame` | Hand over a scene a pipeline already produced; computes nothing (Phase 12) |
| GET | `/api/v1/reports`, `/api/v1/reports/{name}`, `/api/v1/reports/compare` | Stored evaluation reports, read-only (Phase 12) |
| GET | `/api/v1/runs`, `/api/v1/runs/{name}`, `/api/v1/runs/{name}/frames/{i}` | Stored runs, summary and frame by frame, read-only (Phase 12) |
| WS | `/ws/scene` | Pushes the latest scene when it changes (Phase 12) |
| GET | `/api/v1/live/status`, `/scenarios`, `/events`, `/camera` | Live session state, catalogue, events, ego camera (live extension) |
| POST | `/api/v1/live/start`, `/pause`, `/resume`, `/stop`, `/reset` | The five high-level session controls; nothing touches an actor (live extension) |
| — | `/dashboard/` | The static console; `/` redirects to it (Phase 12) |

Full request and response shapes are in [`docs/API.md`](docs/API.md) and in the generated
OpenAPI schema at `/docs`.

Example:

```bash
curl http://localhost:8000/api/v1/system/status
```

---

## Configuration

All settings come from the environment, prefixed `ADAPTX_`, with `__` separating a section
from a field (for example `ADAPTX_API__PORT=8000`). Copy `.env.example` to `.env` and edit.
`.env` is git-ignored and must never contain committed secrets.

Sections: `APP`, `API`, `LOGGING`, `CARLA`, `LIDAR`, `MAP`, `RISK`, `WEBSOCKET`.

---

## CARLA

CARLA is the controlled test environment, not the project. It is an **optional**
dependency: the backend starts, serves every endpoint and passes its whole test suite
without it, reporting `CARLA: DISCONNECTED` with the reason.

```bash
.venv\Scripts\python.exe -m pip install -e ".[carla]"
```

Then set `ADAPTX_CARLA__ENABLED=true` and point `ADAPTX_CARLA__HOST` / `ADAPTX_CARLA__PORT`
at a running server. `ADAPTX_CARLA__USE_MOCK=true` selects an in-process fake simulator for
development; everything it returns is labelled `synthetic_test` and is never sensor data.

---

## Docker

```bash
docker compose up --build
```

The backend is then available on <http://localhost:8000>. CARLA is not containerised at
this stage.

---

## Repository layout

```
src/adaptx/
  api/          FastAPI routes, schemas, WebSocket telemetry
  carla/        Simulator boundary: interface, real client, mock
  config/       Typed environment-driven settings
  core/         Logging, exceptions, lifecycle and service wiring
  mapping/      2.5D map contracts + fixed-resolution grid mapper
  models/       Data contracts shared by every layer
  perception/   LiDAR pipeline, clustering, classification and the geometric detector
  prediction/   Trajectory predictor contract + constant-velocity baseline
  risk/         Risk engine contract + heuristic engine and proximity baseline
  services/     Ingest, metrics, CARLA, status, tracking and prediction services
  tracking/     Tracker contract + geometric tracker and association
  benchmark/    Synthetic datasets and the pipeline/detection/tracking/prediction runners
tests/          unit, integration and shared fixtures
docs/           architecture, API, testing, roadmap, knowledge base, decisions
data/           raw, processed and scenario data (git-ignored contents)
dashboard/      static browser console served at /dashboard (Phase 12, docs/DASHBOARD.md)
```

---

## Documentation

- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — what is implemented now vs planned
- [`docs/API.md`](docs/API.md) — endpoint reference
- [`docs/DASHBOARD.md`](docs/DASHBOARD.md) — the dashboard: what it shows, what it never computes, how to run it
- [`docs/TESTING.md`](docs/TESTING.md) — how to run and extend the suite
- [`docs/BENCHMARKING.md`](docs/BENCHMARKING.md) — benchmark method and what the numbers mean
- [`docs/ROADMAP.md`](docs/ROADMAP.md) — phase plan and current status
- [`docs/knowledge-base/`](docs/knowledge-base) — domain knowledge and requirements
- [`docs/decisions/architecture-decisions.md`](docs/decisions/architecture-decisions.md) — accepted decisions
- [`docs/experiments/experiment-log.md`](docs/experiments/experiment-log.md) — measured experiments only

---

## Project rules

ADAPT-X does not fabricate sensor values, benchmark results or performance numbers, and
does not present simulated, replayed or synthetic data as measurements. See
[`docs/knowledge-base/20-constraints.md`](docs/knowledge-base/20-constraints.md) and
[`CLAUDE.md`](CLAUDE.md).

This system is a research prototype. It is not validated for real-world autonomous-vehicle
deployment.
