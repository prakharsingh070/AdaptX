# ADAPT-X Dashboard

**Status: not started.** No dashboard application exists in this directory yet.

The dashboard is planned for Phase 12. Its requirements are defined in
[`../docs/knowledge-base/15_dashboard-ui.md`](../docs/knowledge-base/15_dashboard-ui.md).

## Backend contract available today

The Phase 1 backend already provides everything the dashboard shell needs:

| Source | What it gives the dashboard |
|---|---|
| `GET /api/v1/system/status` | System / LiDAR / CARLA state, and the implementation status of every perception subsystem |
| `GET /api/v1/system/metrics` | Measured FPS, latency, CPU and memory; unmeasured values are `null` |
| `GET /api/v1/map/status` | Configured resolution levels and their cell sizes |
| `GET /api/v1/risk/status` | Risk levels, thresholds and which factors the configured engine models |
| `GET /api/v1/carla/status` | CARLA connection state, including whether a mock is active |
| `WS /ws/telemetry` | Live status and metrics, plus a `not_yet_available` list naming the streams that do not exist yet |

## Rules for this directory

- Dashboard code must not contain perception logic. It consumes backend contracts
  (`docs/knowledge-base/03_system-architecture.md`, boundary rule).
- Never display a hardcoded metric, detection or map. When live data is unavailable,
  label the view as simulation, replay or unavailable
  (`docs/knowledge-base/20-constraints.md`).
- Every telemetry payload carries a `source` / provenance field. Surface it, so a viewer
  can always tell sensor data from simulated, replayed or synthetic data.
