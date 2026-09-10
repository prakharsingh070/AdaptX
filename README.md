# ADAPT-X

**Adaptive Dynamic Perception and Tracking** — a LiDAR-based perception framework that
allocates spatial map resolution according to environmental risk instead of spending it
uniformly.

Low-risk, open regions are represented coarsely. Regions containing pedestrians, vehicles,
obstacles, high uncertainty or predicted collision risk receive finer detail.

---

## Implementation status

> **Phase 1 (foundation), Phase 2 (LiDAR processing pipeline and benchmarking),
> Phase 3 (geometric object detection) and Phase 4 (temporal tracking) are what
> exist today.**
> Object detection, tracking, trajectory prediction, 2.5D mapping and the adaptive
> resolution algorithm are **not implemented**. The repository provides their data
> contracts and interfaces so they can be added without architectural rewrites.

| Subsystem | State | Notes |
|---|---|---|
| Configuration, logging, API, telemetry | **Implemented** | Phase 1 |
| LiDAR ingest | **Partial** | Structural validation, point count, bounds, metadata. |
| LiDAR pipeline | **Partial** | Validation, NaN/Inf removal, ROI and range filtering; opt-in voxel downsampling, baseline ground segmentation and baseline noise filtering; orchestration with per-stage measured timing. No clustering, no coordinate transforms. |
| Benchmarking (pipeline speed) | **Implemented** | Deterministic synthetic datasets and a fixed-resolution baseline. Speed only — not the Phase 11 ADAPT-X evaluation. |
| Risk | **Partial** | Contract + a proximity-only *baseline* for testing. Not the ADAPT-X risk engine. |
| CARLA | **Boundary only** | Connection + world info. Optional dependency; the backend runs without it. |
| Object detection | **Partial** | Phase 3: geometric clustering and baseline size-based classification. No trained model, no oriented boxes, no velocity. |
| Tracking | **Partial** | Phase 4: gated nearest-neighbour association, measured velocity, track lifecycle. No learned model, no re-identification. |
| 2.5D mapping + adaptive resolution | Planned | Phases 5, 7 |
| Prediction | Planned | Phase 8 |

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
| POST | `/api/v1/tracking/reset` | Drop all tracks and restart identifiers |
| GET | `/api/v1/tracking/status` | Current tracking state |
| GET | `/api/v1/map/status` | Mapping readiness and configured resolution levels |
| GET | `/api/v1/risk/status` | Risk engine readiness, thresholds and modelled factors |
| WS | `/ws/telemetry` | Live system status and measured metrics |

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
  mapping/      2.5D map + resolution controller contracts (no implementation)
  models/       Data contracts shared by every layer
  perception/   LiDAR processing + detector contracts; Phase 1 frame validator
  prediction/   Trajectory predictor contract (no implementation)
  risk/         Risk engine contract + proximity baseline
  services/     Ingest, metrics, CARLA and status services
  tracking/     Tracker contract (no implementation)
tests/          unit, integration and shared fixtures
docs/           architecture, API, testing, roadmap, knowledge base, decisions
data/           raw, processed and scenario data (git-ignored contents)
dashboard/      dashboard application (not started)
```

---

## Documentation

- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — what is implemented now vs planned
- [`docs/API.md`](docs/API.md) — endpoint reference
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
