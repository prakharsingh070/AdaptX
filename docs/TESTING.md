# Testing

The strategy for the full system is in
[`knowledge-base/18_testing-strategy.md`](knowledge-base/18_testing-strategy.md). This
document covers the suite that exists today and how to extend it.

---

## Running the suite

CARLA is optional and the default run never needs it:

```bash
pytest                 # everything except the live CARLA tests
pytest -m carla        # live tests; skip cleanly without a server or the package
```


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
  fixtures/scenes.py        ground planes and object geometry for whole-chain tests
  fixtures/sequences.py     deterministic detections, tracked objects and motion
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
    test_mapping.py         grid indexing, occupancy, height statistics, bounds,
                            resolution, memory limits, determinism, accounting,
                            frame-local lifecycle, AdaptiveMap projection
    test_prediction.py      constant-velocity arithmetic, velocity semantics, track
                            lifecycle, horizon/interval, uncertainty growth, limits,
                            accounting, determinism, configuration validation
    test_risk_baseline.py   proximity baseline behaviour and thresholds
    test_risk_engine.py     factors, unknown-velocity semantics, uncertainty,
                            lifecycle, staleness, map context, levels, aggregation,
                            determinism, edge cases, Phase 8 boundary
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
    test_mapping_pipeline.py
                            processed frame -> mapper with no parallel preprocessing
                            path, plus the map/status APIs and telemetry
    test_risk_pipeline.py   whole chain through to risk, plus the risk/status APIs
                            and telemetry
    test_prediction_pipeline.py
                            raw frame -> processing -> detection -> tracking ->
                            prediction, plus the predict/status APIs and telemetry
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
| `test_an_empty_cell_reports_nan_not_zero` | An unobserved map cell claiming a measured height of zero |
| `test_a_point_on_the_upper_bound_is_out_of_bounds` | A boundary point being recorded in a cell it does not belong to |
| `test_consecutive_frames_do_not_contaminate_each_other` | Frame-local mapping quietly becoming an accumulating one |
| `test_the_adaptive_map_stream_remains_unavailable` | A fixed-resolution map making the adaptive one look present |
| `test_mapping_is_partial_because_it_is_a_fixed_resolution_baseline` | A baseline mapper reporting as IMPLEMENTED |
| `test_the_payload_stays_small` | A dense grid being pushed down the status channel |
| `test_unknown_velocity_and_measured_zero_produce_different_scores` | An unmeasured object scoring like a calm one |
| `test_an_empty_cell_never_lowers_risk` | Unobserved space being read as free space |
| `test_uncertainty_is_never_folded_into_the_risk_score` | Two independent signals collapsing into one number |
| `test_the_aggregate_is_a_maximum_not_a_mean` | A critical object vanishing behind ten quiet ones |
| `test_an_empty_scene_reports_unknown_not_low` | An unassessed scene claiming to be safe |
| `test_the_explanation_never_overclaims` | An explanation asserting probability, safety or validation |
| `test_never_claims_a_collision_probability` | A heuristic score presented as a calibrated probability |
| `test_risk_does_not_decide_resolution` | Phase 8's decision leaking into Phase 7 |
| `test_a_lost_track_is_not_scored` | A terminated track reported as a present threat |
| `test_unknown_velocity_produces_no_trajectory` | Extrapolating a track whose motion was never measured |
| `test_a_measured_standstill_produces_a_stationary_trajectory` | Conflating a measured zero with an unknown velocity |
| `test_excessive_speed_is_rejected_not_clipped` | Substituting a corrected velocity no sensor produced |
| `test_the_result_labels_its_uncertainty_model_as_heuristic` | Heuristic uncertainty passing as a calibrated one |
| `test_uncertainty_grows_strictly_with_time_offset` | A three-second extrapolation looking as trustworthy as a fresh measurement |
| `test_the_predictor_never_writes_a_prediction_onto_the_track` | A forecast being stored where a measurement belongs |
| `test_a_scene_where_nothing_is_eligible_is_not_an_empty_scene` | "Predicted nothing" reading as "saw nothing" |
| `test_it_reports_no_accuracy_figure` | The status endpoint claiming an accuracy that was never measured |
| `test_prediction_is_reported_as_partial_not_finished` | A prediction baseline reporting as IMPLEMENTED |
| `test_the_summary_carries_counts_not_trajectory_points` | Frame geometry being pushed down the status channel |
| `test_provenance_survives_the_whole_chain` | Synthetic points becoming live-sensor trajectories |
| `test_no_secret_fields_are_declared` | Credentials creeping into the settings surface |

---

### Adaptive spatial resolution (Phase 8)

- **Spatial differentiation.** A close, high-risk object refines its own region while distant
  unrelated regions stay coarse, and one object never refines the whole map.
- **Low complexity stays coarse.** An open scene with no objects holds every region at the
  base level — the reason adaptive mapping exists.
- **Unknown risk is not low risk.** An assessment with `risk_score is None` drops the risk
  factor rather than scoring it zero, takes a floored level rather than the coarsest, and
  outranks a *confidently quiet* object. The cause is visible in the decision.
- **Uncertainty earns detail on its own.** A low-risk, badly observed region receives a finer
  level than a low-risk, well observed one — the ADR-033 payoff, asserted directly.
- **Predicted motion refines ahead of arrival.** A trajectory raises the priority of regions
  it crosses, weighted towards the near future.
- **No oscillation.** A region driven through 0.61 / 0.59 / 0.60 / 0.58 / 0.61 / 0.59 holds a
  single level. A sustained rise upgrades on the first frame; a sustained fall downgrades only
  after the dwell time; a disappearing object returns the region to the base level.
- **Region partition is exact.** Points swept across every region boundary, on the lower edge,
  on the upper edge, at interior boundaries and at negative coordinates are each counted
  exactly once, and `input == mapped + out_of_bounds` holds across mixed resolutions.
- **Several resolutions in one map**, with a projected cell reporting its own region's cell
  size and level.
- **Budgets bind deterministically**, coarsening the lowest-priority regions first and
  reporting every demotion.
- **Determinism.** Identical input gives identical decisions, and the order assessments arrive
  in does not change them.
- **No overclaiming.** A parametrised test asserts no generated explanation contains
  "probability", "calibrated", "validated", "guaranteed" or "safe".
- **The baseline survives.** The Phase 6 mapper still reports `is_adaptive: false`, still uses
  `source: fixed`, and `POST /api/v1/lidar/map` is unchanged.

### CARLA simulation boundary (Phase 9)

CARLA is optional and is **not installed** in the primary (Python 3.13) environment - no
`carla` wheel exists for 3.13. The suite is split so that never becomes a false failure:

- **Conversion** (`test_carla_conversion.py`) needs no simulator at all, because
  `adaptx.carla.conversion` imports none. Axis flips, handedness (a cross product check),
  yaw sense, round-trip ego/world transforms, buffer decoding, and simulation-time
  anchoring. This is the largest group on purpose: a dropped sign flip mirrors the world
  **silently**.
- **Lifecycle** (`test_carla_session.py`) runs against `tests/fixtures/fake_carla.py`, a
  stand-in. Spawn order, cleanup after a failed setup, world settings restored, a stubborn
  actor not stranding the rest, sensor timeouts, stale-frame rejection, and frames advancing
  by exactly one timestep.
- **Integration** (`test_carla_pipeline.py`) pushes simulated frames through the real
  Phase 2-8 chain. The sharpest case places a target to the ego's **left** and asserts
  detection reports it on the left - which fails if the boundary conversion is ever removed.
- **Separation** - the chain is run twice, once reading ground truth every frame and once
  never reading it, asserting identical output. A second check confirms in a subprocess that
  no perception module imports the CARLA package at all.
- **Live** (`test_carla_live.py`) is the only test that touches a real server. It is
  deselected by default (`-m "not carla"`) *and* skips itself when the package or server is
  absent. Run it with `pytest -m carla`.

**Running the live tests for real** (done on 2026-09-11, 7/7 passed - Experiment 010):

```bash
# Python 3.12 only: the CARLA 0.9.16 wheel ships for cp312 and PyPI's "carla" is 0.9.5.
py -3.12 -m venv .venv312
.venv312\Scripts\python -m pip install -e ".[dev]"      # from THIS checkout, see below
.venv312\Scripts\python -m pip install <path-to>\carla-0.9.16-cp312-cp312-win_amd64.whl
ADAPTX_CARLA__HOST=127.0.0.1 .venv312\Scripts\python -m pytest -m carla -v
```

Two things that cost time: an editable install points at the checkout it was run from,
so a worktree must reinstall or set `PYTHONPATH` to its own `src`; and CARLA's client
prints `INFO: streaming client: connection failed ...` lines at sensor stop and process
exit, which come from inside the CARLA library and are not ADAPT-X errors. The fake
simulator reproduces two server behaviours the live run found - an actor reports the world
origin until the first tick, and a spawn point can refuse - so those regressions fail
without a server.

What the stand-in **cannot** prove: anything about CARLA itself - its API compatibility, its
sensor model or its performance. No figure produced against it is a CARLA measurement.

### Scenario framework (Phase 10)

- **Definitions** (`test_scenario_models.py`) - every validation rule with an explicit
  error: duration, timestep, the two-frame minimum, seed, machine-name ids, duplicate ids,
  the reserved `ego` id, segment ordering and overlap, segments beyond the duration, a
  moving ego. Scripted motion is checked by hand-derived arithmetic: timed start holds,
  timed stop freezes, diagonal moves on both axes, an L-shaped path sums its legs. A
  definition round-trips through JSON.
- **Runner** (`test_scenario_runner.py`) - lifecycle transitions; a malformed definition
  raises before any simulator contact while a run-time failure returns `FAILED` with the
  frames stepped; cleanup on every failure path including a second actor failing after the
  first spawned; same seed resolves identically and a different seed moves a jittered
  placement; resolution never touches the global `random` state; actors land where asked;
  ground truth and sensor frames share identity; the processor receives only the sensor
  frame; scenario B inherits nothing from scenario A; the result carries no evaluation field.
- **Catalogue and pipeline** (`test_scenario_pipeline.py`) - every catalogue scenario is
  valid, exact, runs to completion through the real Phase 2-8 chain, and its ground truth
  sees every scripted actor. The chain run by hand without ever calling `ground_truth()`
  produces identical stage counts to the runner. Source-level checks that the definition
  layer imports nothing from the CARLA boundary and the package never imports `carla`. The
  CLI lists the catalogue, fails honestly without a simulator, and exits with a usage code
  for an unknown id.
- **Live** (`test_carla_live.py`) - `vehicle_approach` and every catalogue scenario against a
  real server. Deselected by default and self-skipping; **never executed here**.

What none of this proves: that the catalogue's blueprints exist on a real server, that a
real simulator places actors where the script says, or anything about perception accuracy.

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
