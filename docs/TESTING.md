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
    test_preprocessing.py   Phase 2A pipeline: validation, NaN/Inf, ROI, range, metrics
    test_voxel.py           voxel grouping, boundaries, representative point, sizes
    test_ground.py          per-cell ground level, tolerance, slope, ceiling, limits
    test_noise.py           neighbour counting, thresholds, cell size, guards
    test_pipeline_2b.py     seven-stage chain, accounting, metadata, determinism
    test_benchmark.py       dataset reproducibility, baseline profile, runner, rates
    test_detection.py       clustering, geometry, filtering, classification, determinism
    test_tracking.py        association, velocity, lifecycle, class stability, determinism
    test_risk_baseline.py   proximity baseline behaviour and thresholds
    test_services.py        metrics, LiDAR ingest, system status aggregation
    test_carla_mock.py      CARLA boundary: real client without CARLA, mock, service
  integration/
    test_api.py             startup, routing, all seven endpoints, error paths
    test_websocket.py       /ws/telemetry envelope, sequencing, connection registry
    test_lidar_preprocessing_api.py
                            preprocess flag, backward compatibility, OpenAPI contract
    test_lidar_2b_api.py    2B stages through the API, defaults left unchanged
    test_pipeline_end_to_end.py
                            every synthetic scenario end to end, edge cases, sanity
    test_detection_pipeline.py
                            raw frame -> processing -> detection, and the detect API
    test_tracking_pipeline.py
                            multi-frame chain end to end, plus the track/reset API
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

`tests/fixtures/scenes.py` builds explicit box shells, planes and specks whose expected
cluster counts and dimensions are known by construction — used by the detection tests.
`tests/fixtures/sequences.py` builds temporal detection sequences with explicit positions
and explicit timestamps, so an expected velocity is arithmetic rather than a guess.
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
| `test_detection_is_still_reported_as_planned` | Preprocessing existing making object detection look implemented |
| `test_duration_is_measured_and_positive` | A preprocessing duration that is not actually measured |
| `test_kept_point_is_a_real_input_point` | Voxelisation emitting a synthesised centroid as if measured |
| `test_object_only_cell_misclassifies_its_lowest_point` | A documented ground-segmentation weakness being quietly forgotten |
| `test_an_isolated_stray_point_becomes_its_own_ground` | A stage-ordering consequence going unrecorded |
| `test_baseline_stages_are_labelled_as_baselines` | Baselines being reported as finished work |
| `test_velocity_is_never_invented` | A single-frame detector reporting motion it cannot see |
| `test_geometry_matching_several_bands_is_unknown` | Ambiguous geometry being confidently mislabelled |
| `test_detection_does_not_claim_to_be_finished` | A geometric baseline reporting as IMPLEMENTED |
| `test_response_carries_no_raw_point_arrays` | The API echoing whole point clouds back |
| `test_first_frame_velocity_is_unknown_not_zero` | A single observation claiming measured motion |
| `test_motion_is_unknown_until_observed` | Zero-vector defaults reading as a measured standstill |
| `test_no_future_trajectories_are_exposed` | Tracker extrapolation being mistaken for prediction |
| `test_an_object_returning_after_deletion_gets_a_new_id` | Implying re-identification that does not exist |
| `test_every_frame_is_labelled_synthetic` | Benchmark data passing as sensor data |
| `test_empty_dataset_reports_null_rates_rather_than_zero` | A rate over no points being invented |
| `test_memory_absence_is_declared_not_faked` | An unmeasured metric being filled in |
| `test_output_contains_no_invalid_numbers` | A stage emitting NaN or infinity |
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
- Assert *properties* rather than exact counts where an algorithm's output depends on
  several interacting thresholds. Pinning a count produces a test that breaks when a default
  moves without saying anything about correctness. Conservation, finiteness, ordering,
  determinism and metadata are asserted exactly; point counts usually are not.
- Never assert a timing magnitude. Machines differ; only ordering and positivity are
  portable.
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
