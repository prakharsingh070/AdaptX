# Architecture Decisions

This log records accepted decisions so future contributors and AI agents do not redefine the architecture without discussion.

## ADR-001: FastAPI Backend

**Decision:** Use FastAPI as the backend API layer.

**Reason:** It provides a lightweight Python interface for perception modules and dashboard integration.

**Status:** Accepted

## ADR-002: CARLA Simulation

**Decision:** Use CARLA as the primary controlled simulation environment.

**Reason:** It provides configurable roads, traffic, weather, actors, and sensors for repeatable experiments.

**Status:** Accepted

## ADR-003: Fixed-Resolution Baseline

**Decision:** Maintain fixed-resolution mapping as an experimental baseline.

**Reason:** A comparable baseline is required for quantitative evaluation of ADAPT-X.

**Status:** Accepted

## ADR-004: Versioned API Prefix `/api/v1`

**Decision:** Serve all application endpoints under `/api/v1`, with `/health` unversioned.

**Reason:** `16_api-contract.md` states the API may evolve and requires explicit shared
contracts. A version segment lets the contract change without breaking a deployed
dashboard. `/health` stays unversioned because container and orchestrator probes should not
depend on an application version.

**Alternatives considered:** Unversioned `/api/...` as sketched in the knowledge base
(simpler, but no migration path); header-based versioning (invisible in logs and browsers).

**Impact:** Endpoint paths in `16_api-contract.md` gain a `/v1` segment. The endpoint set
described there is otherwise unchanged and still pending implementation.

**Risks:** Low. Adds one path segment.

**Status:** Accepted

## ADR-005: Readiness and Implementation Status Reported Separately

**Decision:** Every subsystem in `GET /api/v1/system/status` carries both a readiness
(`READY` / `NOT_READY`) and an implementation status (`IMPLEMENTED` / `PARTIAL` /
`PLANNED` / `MOCK`), declared explicitly in a component table rather than inferred.

**Reason:** `20-constraints.md` forbids presenting absent capability as working. Readiness
alone is ambiguous: `NOT_READY` cannot distinguish "not written yet" from "written but
currently unavailable". The two axes make the difference machine-readable for the API, the
dashboard and the tests.

**Alternatives considered:** Readiness only (ambiguous); inferring status from whether an
implementation class is importable (an interface import would falsely signal readiness).

**Impact:** Adding a module means updating `_phase_1_components()` in
`services/system_service.py`. Tests assert that planned modules never report as
implemented.

**Risks:** The table can drift from reality if not updated. Mitigated by tests and by the
release checklist in `ROADMAP.md`.

**Status:** Accepted

## ADR-006: Risk Normalised to [0, 1]

**Decision:** Risk is normalised to `[0, 1]` in every backend model and API response. The
0-100 form shown in `17_data-schema.md` is a presentation concern, available as the derived
`risk_percent` property.

**Reason:** A single internal scale keeps thresholds, comparisons and aggregation
unambiguous, and matches how confidence and uncertainty are already expressed. Mixing 0-1
and 0-100 in one system invites silent factor-of-100 errors.

**Alternatives considered:** 0-100 throughout (matches the knowledge-base example but
conflicts with the confidence/uncertainty scales); storing both (duplicated state that can
disagree).

**Impact:** `17_data-schema.md` shows `"risk": 82`; the API emits `"risk_score": 0.82`.
Dashboards multiply by 100. `GET /api/v1/risk/status` documents the scale in its response.

**Risks:** A consumer reading the knowledge-base example literally. Mitigated by stating
the scale in the API response and in `API.md`.

**Status:** Accepted

## ADR-007: CARLA Is an Optional Dependency

**Decision:** The `carla` package is an optional extra. The backend starts, serves every
endpoint and passes its entire test suite without it, reporting `CARLA: DISCONNECTED` with
a reason. The mock simulator is selected only by explicit configuration, never as a
fallback for a failed connection.

**Reason:** ADR-002 makes CARLA the test environment, not the product. Making it mandatory
would block development and CI on a large, platform-specific, version-pinned dependency.
`20-constraints.md` forbids silently falling back to fake data, so a failed real connection
must surface as a failure rather than become synthetic output.

**Alternatives considered:** Required dependency (blocks CI and most development
machines); automatic fallback to the mock (violates the no-silent-fake-data rule).

**Impact:** `CarlaClient` imports `carla` lazily inside `connect()`. `CarlaService.connect()`
never raises. `carla_status.is_mock` is exposed so consumers can always tell.

**Risks:** Real CARLA integration paths get less automatic coverage. Mitigated by the
`carla` pytest marker for tests that need a live server.

**Status:** Accepted

## ADR-008: No Database, Broker or ML Framework in Phase 1

**Decision:** Phase 1 adds no database, message broker, task queue, ML framework or cloud
service. Runtime dependencies are FastAPI, Uvicorn, Pydantic, pydantic-settings, NumPy and
psutil.

**Reason:** `20-constraints.md` forbids unnecessary frameworks. Nothing in Phase 1 needs
persistence or asynchronous processing: frames are validated and accounted for in-process.
psutil is included because measuring real CPU and memory is the only alternative to
reporting nothing — and reporting an invented figure is forbidden.

**Alternatives considered:** Adding a time-series store for metrics now (no requirement
yet); adding PyTorch in anticipation of Phase 3 (large, and the detection approach is
undecided).

**Impact:** Metrics are in-memory over a rolling window and reset when the process
restarts. Persistence becomes a decision when benchmarking (Phase 11) needs it.

**Risks:** Metrics history is lost on restart. Acceptable: Phase 11 will define its own
recorded-measurement format.

**Status:** Accepted

## Decision Template

### ADR-XXX: Title

**Decision:**

**Reason:**

**Alternatives considered:**

**Impact:**

**Risks:**

**Status:** Proposed / Accepted / Superseded
