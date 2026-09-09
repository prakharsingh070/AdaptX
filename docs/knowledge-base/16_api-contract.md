# API Contract

The exact API may evolve, but frontend and backend must share explicit contracts. Suggested endpoints:

## Read APIs

- `GET /api/system/status`
- `GET /api/lidar/current`
- `GET /api/objects`
- `GET /api/tracking`
- `GET /api/prediction`
- `GET /api/adaptive-map`
- `GET /api/risk-map`
- `GET /api/performance`
- `GET /api/events`

## CARLA APIs

- `POST /api/carla/connect`
- `POST /api/carla/start`
- `POST /api/carla/stop`
- `POST /api/carla/scenario`

## Scenario APIs

- `POST /api/scenario/generate`
- `POST /api/scenario/start`
- `POST /api/scenario/stop`

## Benchmark APIs

- `POST /api/benchmark/run`
- `GET /api/benchmark/results`

Every endpoint should document request and response schemas, error responses, timestamps, units, and whether data is live, replayed, simulated, or unavailable. Frontend components must not implement perception logic.
