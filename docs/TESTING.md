# Testing

The strategy for the full system is in
[`knowledge-base/18_testing-strategy.md`](knowledge-base/18_testing-strategy.md). This
document covers the suite that exists today and how to extend it.

---

## Running the suite

```bash
.venv\Scripts\python.exe -m pytest
```

```bash
.venv\Scripts\python.exe -m pytest tests/unit
```

```bash
.venv\Scripts\python.exe -m pytest tests/integration
```

The whole quality gate — lint, format check, type check, tests:

```bash
.\scripts\dev.ps1 check
```

On Linux or macOS use `scripts/dev.sh check`.

**The suite requires no CARLA, no GPU, no network and no running server.** It is
self-contained and deterministic.

---

## Layout

```
tests/
  conftest.py               settings / context / app / client fixtures
  fixtures/point_clouds.py  seeded synthetic frame builders
  unit/
    test_config.py          settings defaults, env overrides, validation, no secrets
    test_logging.py         JSON and text formatters, context, idempotent setup
    test_models.py          vehicle, object, track, trajectory, map and risk contracts
    test_point_cloud.py     point-cloud contract + Phase 1 frame validator
    test_risk_baseline.py   proximity baseline behaviour and thresholds
    test_services.py        metrics, LiDAR ingest, system status aggregation
    test_carla_mock.py      CARLA boundary: real client without CARLA, mock, service
  integration/
    test_api.py             startup, routing, all seven endpoints, error paths
    test_websocket.py       /ws/telemetry envelope, sequencing, connection registry
```

---

## Fixtures

Defined in `tests/conftest.py`. Every one builds explicit settings, so tests never depend
on a developer's `.env` or on ambient environment variables.

| Fixture | Gives you |
|---|---|
| `settings` | Deterministic `Settings`: CARLA off, `max_points=1000`, telemetry interval 0.05 s |
| `context` | An `ApplicationContext` wired from those settings |
| `app` | A `FastAPI` app bound to that context |
| `client` | A `TestClient` that runs the real startup/shutdown lifespan |

`tests/fixtures/point_clouds.py` builds seeded synthetic clouds (`make_points`,
`make_frame`) and JSON request bodies (`frame_request_body`). Everything it produces is
labelled `synthetic_test` and must never stand in for sensor data.

---

## Determinism

- All generated point clouds use a fixed seed (`SEED = 20260101`), as does
  `MockCarlaSimulatorClient`.
- No test depends on wall-clock time except two staleness tests, which use a 1 ms limit
  and a 20 ms sleep — a 20× margin.
- No test depends on network access, an installed CARLA, or the host's `.env`.
- `test_enabled_client_reports_the_missing_package` is skipped when the optional `carla`
  package *is* installed, so the suite stays correct either way.

---

## What the suite verifies

Beyond the usual behaviour checks, several tests exist specifically to keep the project's
honesty constraints ([`knowledge-base/20-constraints.md`](knowledge-base/20-constraints.md))
enforced by CI rather than by review:

| Test | Guards against |
|---|---|
| `test_gpu_is_always_reported_unavailable` | Inventing a GPU utilisation figure |
| `test_reports_nothing_measured_before_any_frame` | Defaulting unmeasured metrics to 0 |
| `test_synthetic_frames_report_simulated_not_connected` | Synthetic input presenting as a live sensor |
| `test_missing_source_is_rejected` | Ingesting data with undeclared provenance |
| `test_no_perception_data_is_fabricated` | Telemetry emitting invented objects, tracks, risk or map cells |
| `test_planned_modules_are_not_presented_as_working` | A planned module appearing implemented |
| `test_mock_is_selected_only_by_configuration` | The mock becoming a silent fallback |
| `test_failed_connection_does_not_raise` | CARLA absence crashing the backend |
| `test_field_is_flagged_as_baseline_and_has_no_cells` | The baseline being mistaken for the ADAPT-X risk engine |
| `test_no_secret_fields_are_declared` | Credentials creeping into the settings surface |

---

## Conventions for new tests

- **Never weaken or delete a test to make the suite green.** If a test fails, either the
  code or the test's premise is wrong — fix the right one.
- Test observable behaviour, not private attributes.
- Seed anything random and state the seed.
- Assert on units and coordinate frames where they matter.
- Distinguish "expected unavailable" from "failure": a `null` metric or a `DISCONNECTED`
  simulator is correct behaviour, not an error.
- Mark tests that genuinely need a CARLA server with `@pytest.mark.carla`, so they can be
  deselected; the default suite must never require one.
- Perception work (Phases 2+) also needs the integration paths named in
  `knowledge-base/18_testing-strategy.md`: LiDAR→detection, detection→tracking,
  tracking→prediction, prediction→risk, risk→adaptive map, adaptive map→API.

---

## Quality tooling

| Tool | Command | Configuration |
|---|---|---|
| Ruff (lint) | `ruff check .` | `[tool.ruff.lint]` in `pyproject.toml`; rules `E, W, F, I, B, C4, UP, SIM, N, RUF` |
| Ruff (format) | `ruff format .` | line length 100, double quotes |
| mypy | `mypy` | `[tool.mypy]`; `disallow_untyped_defs`, checks `src/adaptx` |
| pytest | `pytest` | `[tool.pytest.ini_options]`; `asyncio_mode = "auto"`, `--strict-markers` |

mypy runs against the interpreter in use rather than a pinned version, because the NumPy
stubs shipped with the pinned NumPy require 3.12+ syntax to parse.
