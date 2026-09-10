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

## ADR-009: Coordinate Convention (+x forward, +y left, +z up)

**Decision:** ADAPT-X uses a right-handed frame with the origin at the sensor,
**+x forward, +y left, +z up**, in metres, for the `lidar` and `ego` coordinate
frames.

**Reason:** Phase 1 already implied this convention without stating it:
`BoundingBox3D.yaw_rad` and `VehicleState.heading_rad` are defined as
counter-clockwise about +z measured from the +x axis, which describes only a
right-handed frame. Phase 2A introduces ROI bounds whose meaning depends
entirely on the convention, so it has to be written down. It matches ISO 8855
and ROS REP-103.

**Alternatives considered:** CARLA's left-handed frame (+y right), which would
avoid a conversion at the simulator boundary but contradicts the yaw definition
already in the Phase 1 models and the wider automotive/robotics convention.

**Impact:** ROI configuration is expressed in this frame. CARLA data must be
converted at the CARLA boundary in Phase 9; the conversion does not exist yet
and no module currently transforms coordinates. Frames carry a
`coordinate_frame` field, so a mislabelled frame is visible rather than silent.

**Risks:** Ingesting CARLA data before the Phase 9 conversion exists would
mirror the y axis. Mitigated by documenting it here, in the preprocessing module
docstring and in `docs/ARCHITECTURE.md`.

**Status:** Accepted

## ADR-010: Separate Raw and Validated Point-Cloud Frame Types

**Decision:** `RawPointCloudFrame` may contain NaN and infinite coordinates;
`PointCloudFrame` may not. Both derive from `BasePointCloudFrame`, which holds
the metadata and the structural validation. Preprocessing consumes the former
and produces the latter.

**Reason:** Phase 1 defined a single frame type that rejects non-finite values.
That is the right contract for everything downstream, but a real scanner
reports a non-return as NaN, so the input to the cleaning stage must be able to
hold values that the output forbids. Relaxing `PointCloudFrame` would have
removed a useful guarantee from every consumer; representing "not yet cleaned"
as a separate type keeps the invariant in the type system - holding a
`PointCloudFrame` is proof the data was validated.

**Alternatives considered:** Allowing non-finite values in `PointCloudFrame`
with a flag (weakens the contract for every consumer, and a flag is easy to
ignore); passing a bare NumPy array plus loose metadata keyword arguments into
the pipeline (no typing, and duplicates the metadata fields, which
`20-constraints.md` warns against).

**Impact:** Phase 1 behaviour is unchanged: `PointCloudFrame` still rejects
non-finite values with the same message, and every Phase 1 test passes
untouched. `bounds()` is now defined over finite points so an infinity in a raw
frame cannot propagate into a bound and serialise as a null.

**Risks:** A future contributor could accept `BasePointCloudFrame` where a
validated frame is required, losing the guarantee. Mitigated by the narrow
signatures: only `LiDARProcessingPipeline.run` accepts the base type.

**Status:** Accepted

## ADR-011: Range Filtering Uses 3D Euclidean Distance

**Decision:** Range filtering uses the full 3D distance from the sensor origin,
`sqrt(x^2 + y^2 + z^2)`, not the ground-plane distance `sqrt(x^2 + y^2)`. Both
bounds are inclusive. The implementation compares squared distances against
squared bounds.

**Reason:** Minimum range models the sensor's blind zone and returns off the
ego vehicle, which are physical 3D phenomena: a point 0.3 m directly above the
sensor is inside the blind zone even though its planar distance is zero. Using
planar distance would keep such points. Comparing squared distances avoids a
square root over the whole array and is equivalent because both bounds are
non-negative.

**Alternatives considered:** Planar distance (cheaper and matches how a
bird's-eye ROI is reasoned about, but wrong for the blind zone); per-axis limits
only (already covered by the ROI box, and does not model a radial sensor
limit).

**Impact:** ROI filtering runs before range filtering, so a point failing both
is attributed to the ROI. Squaring a coordinate above roughly 1e154 would
overflow in float64; real LiDAR coordinates are many orders of magnitude below
that.

**Status:** Accepted

## ADR-012: Phase 2B Stages Are Opt-In

**Decision:** Voxel downsampling, ground segmentation and noise filtering are
disabled by default and enabled individually through configuration
(`ADAPTX_LIDAR__VOXEL_ENABLED`, `__GROUND_ENABLED`, `__NOISE_ENABLED`).

**Reason:** All three change what every downstream consumer sees, and none is
neutral. Voxelisation is lossy by construction; ground segmentation and noise
filtering are baselines with documented failure modes. Turning them on by
default would silently alter the meaning of a processed frame, and ADAPT-X
depends on a stable, comparable baseline (ADR-003). Opt-in also means the whole
existing test suite continues to describe real behaviour rather than being
rewritten around a new default.

**Alternatives considered:** Enabled by default (matches the "full pipeline"
picture but silently changes existing behaviour and forces test rewrites); a
single `preprocessing_level` dial (couples three independent choices together).

**Impact:** With stock configuration the pipeline reports the same four stages
as Phase 2A and `ground_frame` is null. A stage that did not run is absent from
the `stages` list rather than reported with zero counts, so the list always
describes what actually happened.

**Risks:** The stages could sit unused because nobody enables them. Mitigated by
documenting them in `.env.example`, `API.md` and the roadmap.

**Status:** Accepted

## ADR-013: No LiDAR-to-Ego Transform in Phase 2B

**Decision:** Phase 2B does not implement a coordinate transformation stage.
All three stages operate in the frame the points arrive in.

**Reason:** Determined per stage rather than assumed. Voxelisation is grid
quantisation, a function of coordinates in whatever frame they arrive in. Noise
filtering measures distances between points, which any rigid transform
preserves exactly. Ground segmentation needs to know which axis is up - already
fixed by ADR-009 - and where the ground is, which the per-cell lowest-point
method *discovers* locally rather than being told. None of the three needs a
transform, so building one would be speculative work with no consumer.

**Alternatives considered:** Adding a transform stage pre-emptively (no
consumer, and the calibration source it would need does not exist yet);
a global height threshold for ground, which *would* have required the sensor
mount height and therefore at least the translation part of a transform.

**Impact:** The only frame assumption is that the sensor is mounted roughly
level, so +z is up. A real transform becomes necessary at the first of: a
tilted or multi-sensor mount, sensor fusion, or CARLA ingestion (Phase 9, which
also carries the left-handed to right-handed flip). At that point it belongs in
its own stage with an explicit calibration source, not folded into these.

**Risks:** Ground segmentation degrades on a significantly rolled or pitched
mount. Documented in the module and asserted by its limitation tests.

**Status:** Accepted

## ADR-014: Grid-Approximated Noise Filtering, Without SciPy

**Decision:** The baseline noise filter counts a point's neighbours over the
3x3x3 block of grid cells around it and drops points below a configured count.
It is named for what it does rather than being called a radius filter, and no
spatial-index dependency is added.

**Reason:** Exact radius outlier removal or statistical (k-nearest-neighbour)
outlier removal needs a spatial index, in practice `scipy.spatial.cKDTree`. The
grid approximation needs only NumPy and is adequate for removing isolated
returns, which is what a baseline is for. Adding a dependency before the cheaper
approach has been shown insufficient would violate the "no unnecessary
frameworks" constraint.

**Alternatives considered:** SciPy cKDTree (exact, but a new dependency not yet
justified); counting only the point's own cell (much cheaper, but a cluster
split across a cell boundary would be wrongly deleted).

**Impact:** The region tested is a cube of side 3x the cell size, not a sphere,
so the effective neighbourhood depends slightly on where a point sits within its
cell. The module says so plainly rather than implying exact radius semantics.
Revisit if the approximation proves inadequate against real data, and record
that decision.

**Status:** Accepted

## ADR-015: Voxel Representative Is a Real Measured Point

**Decision:** The point kept for each occupied voxel is the real input point
nearest that voxel's centroid, with ties broken by the lower input index. The
centroid itself is never emitted.

**Reason:** A centroid is a coordinate no sensor returned. Emitting one would
manufacture a sensor value, which `20-constraints.md` forbids, and it would also
require inventing an intensity for the synthetic point. Keeping a real return
preserves both the measurement and its intensity. Choosing the point nearest the
centroid rather than an arbitrary one keeps the downsampled cloud faithful to
the original spatial structure.

**Alternatives considered:** Centroid averaging, which is the conventional
choice in most libraries and gives smoother output, but synthesises data; first
point per voxel, which is cheapest but arbitrary and biased by input ordering.

**Impact:** Slightly more work per frame than either alternative: a centroid
pass followed by a nearest-point selection. Deterministic given the tie-break
rule.

**Status:** Accepted

## ADR-016: Baseline Ground Segmentation by Per-Cell Lowest Point

**Decision:** Ground is classified per xy cell: the lowest point in a cell
defines the local ground level, and points within a configured tolerance above
it are ground. An absolute height ceiling is available but disabled by default.

**Reason:** A single global height threshold assumes a flat world *and* a known
sensor mount height; on any incline it either keeps a wall of ground points or
erases the road. Working per cell discovers the ground level locally, handles
slope, and needs no mount height (ADR-013). RANSAC plane fitting handles tilt
too but needs a fixed seed to stay deterministic and can latch onto a large
wall, which is a worse failure than the one below.

**Alternatives considered:** Global height threshold (simpler, fails on slope);
seeded RANSAC plane fit (handles tilt, but non-deterministic by nature and can
select the wrong plane).

**Impact:** The known weakness is a cell containing only object returns - a car
roof with no road visible beneath it - whose lowest points become "ground". The
optional ceiling limits this when the mount height is known; it is off by
default because it reintroduces the mount assumption. The limitation is asserted
by a test so it stays visible rather than being forgotten.

**Status:** Accepted

## ADR-017: One Pipeline Orchestrator, Named for What It Does

**Decision:** The orchestrating class is `LiDARProcessingPipeline`, in
`adaptx/perception/pipeline.py`. It sequences stages and accounts for what each
one did; it implements no algorithm itself. It was previously called
`PointCloudPreprocessor`.

**Reason:** With ground segmentation and noise filtering in place the class no
longer does only "pre" processing, and the old name understated it. Keeping an
inaccurate name is a small cost paid on every reading of the code. The rename is
mechanical, has no external consumers, and is covered by the whole test suite.

**Alternatives considered:** Keeping the old name and documenting the mismatch
(free, but leaves every future reader to discover it); introducing a second
class as a facade (two names for one thing, and the orchestration already
existed).

**Impact:** Each algorithm lives in its own module - `voxel`, `ground`, `noise`,
with `grid` shared between them - and is independently constructible and
testable. The orchestrator holds sequencing, accounting and timing only. Adding
a stage means adding a module and one block in `run`.

**Status:** Accepted

## ADR-018: A Fixed-Resolution *Processing* Baseline, Distinct from ADR-003

**Decision:** Phase 2C defines `fixed_resolution_baseline`: the whole pipeline
at one fixed voxel size everywhere, as a pinned named configuration. It is
explicitly **not** the fixed-resolution map baseline of ADR-003.

**Reason:** ADAPT-X's claim is that risk-aware resolution beats uniform
resolution, and that needs a stated reference to be measured against. Fixing the
parameters and writing down the assumptions turns "whatever settings were in the
environment" into something reproducible.

The distinction from ADR-003 is not pedantry. ADR-003 concerns cell size in the
2.5D occupancy map, which does not exist. If the two were conflated, the project
would appear to already possess a mapping baseline it has not built, and a later
comparison could be presented as more complete than it is.

**Alternatives considered:** Waiting until Phase 5 so there is only one baseline
(leaves Phase 2 work unmeasurable against anything); reusing the ADR-003 name
(the conflation this decision exists to prevent).

**Impact:** Two profiles ship: the baseline, and a `filter_only` profile that
runs the Phase 2A stages alone. Running both attributes the cost of the Phase 2B
stages by measurement rather than by guess. The default 0.20 m voxel size is a
plausible starting point, not a tuned or validated optimum, and is documented as
such.

**Risks:** A reader could still take a processing baseline for a perception
baseline. Mitigated by saying so in the module, in `docs/BENCHMARKING.md` and
here.

**Status:** Accepted

## ADR-019: Benchmark on Deterministic Synthetic Data, Clearly Labelled

**Decision:** Benchmarks run on generated point clouds from a fixed seed. Every
frame is labelled `SYNTHETIC_TEST`, every dataset record carries
`synthetic: true` and `ground_truth_available: false`, and every report repeats
that the figures say nothing about real-world performance.

**Reason:** No recorded LiDAR dataset is available to this project yet, and
benchmarking against a live sensor would be neither repeatable nor controlled -
`knowledge-base/12_scenario-generation.md` requires reproducibility. Generated
data gives byte-identical inputs across machines and days, which is what makes
two measurements comparable at all.

The labelling is not decoration. `20-constraints.md` forbids presenting
simulated data as real, and a speed number measured on synthetic geometry is
exactly the kind of figure that gets quoted later without its caveat.

**Alternatives considered:** A live sensor (not repeatable, and none is
attached); a public recorded dataset (none vendored, and licence and size
questions are unresolved); no benchmark at all (leaves Phase 2 unmeasured).

**Impact:** The generator is a crude geometric stand-in - no beam divergence, no
occlusion, no incidence-angle falloff, no intensity physics. It is adequate for
measuring throughput and exercising edge cases, and inadequate for anything
about accuracy. Replacing it with a recorded dataset means changing one module.

**Risks:** Timings on synthetic geometry may not predict timings on real scans,
whose density distribution differs. Stated in the report output itself, not only
in documentation.

**Status:** Accepted

## ADR-020: Grid Connected Components for Clustering, Not DBSCAN

**Decision:** Clustering groups points by connectivity on a regular grid whose
cell size is the cluster tolerance. Touching occupied cells join; each connected
group is a cluster. The module is named `GridConnectedComponentClusterer` rather
than borrowing the name of an algorithm it is not.

**Reason:** Exact Euclidean clustering compares pairwise distances between
candidate neighbours, which needs a spatial index - in practice
`scipy.spatial.cKDTree`. ADR-014 already declined that dependency for noise
filtering on the same grounds, and the approximation is adequate for a baseline
whose purpose is to be replaceable. Grid connectivity is fully vectorisable,
deterministic, and reuses the quantiser the Phase 2B stages already share.

DBSCAN was considered and rejected as the wrong shape for this stage: its
core-point rule is a density filter, and ADAPT-X already filters sparse clusters
explicitly by point count, where the threshold is visible and configurable
rather than buried in the clustering step.

**Alternatives considered:** SciPy cKDTree with true Euclidean clustering
(exact, but a dependency not yet earned); pairwise distances in NumPy
(O(N^2), unusable at 100k points); DBSCAN (density rule duplicates the existing
filter).

**Impact:** Two objects whose points land in touching cells merge into one
cluster even when no pair of points is within the tolerance - two pedestrians
half a metre apart may come back as one. Points sharing a cell can never be
split. Both are stated in the module and asserted by tests, so the behaviour is
visible rather than surprising. Connected components are found by vectorised
label propagation with pointer jumping, so no Python-level union-find loop runs
over millions of edges.

**Status:** Accepted

## ADR-021: Classification by Dimension Bands, and What Its Confidence Means

**Decision:** Objects are classified by comparing measured cluster dimensions
against explicit bands for pedestrian, cyclist, vehicle and obstacle. A cluster
matching no band, **or more than one**, is `UNKNOWN`. The reported confidence is
a geometric fit score, documented as such, not a probability.

**Reason:** Geometry is the only signal available: there is no trained model, no
appearance data and no labelled dataset. Stating the rules as bands makes the
output predictable and reviewable, which a learned model would not be at this
stage.

Returning `UNKNOWN` on ambiguity is the substantive part. Bands for a narrow
pedestrian and a bicycle genuinely overlap, and picking the "best" match would
manufacture a distinction the measurement does not support. The system reports
that it cannot tell.

Calling the score a confidence without qualification would be the more damaging
error. `20-constraints.md` forbids fabricating accuracy figures, and a number in
[0, 1] beside a class label reads as "probability this is correct". It is not:
it is how centrally the cluster sits in its band. No labelled data exists to
calibrate a real probability, so none is offered. `UNKNOWN` scores 0.0.

**Alternatives considered:** Nearest-band matching with a distance score
(always returns a class, hiding genuine ambiguity); a trained classifier (needs
labelled data the project does not have, and Phase 3 is scoped as the
deterministic baseline an ML detector is later measured against).

**Impact:** Ambiguous geometry - and there is a lot of it in a real scene -
comes back `UNKNOWN` rather than confidently wrong. Bands are module constants
with a documented table rather than configuration, because they are the rule
itself; cluster *filtering* thresholds are configurable, as those are policy.
Classification is rotation-tolerant only at 90 degrees: axis-aligned boxes make
a diagonal object measure larger than it is.

**Status:** Accepted

## ADR-022: The Detector Contract Returns a Result, Not a List

**Decision:** `ObjectDetector.detect` returns a full `DetectionResult` -
objects, rejected candidates, counts, measured timings and the configuration
snapshot - rather than `list[DetectedObject]`.

**Reason:** A detector is the only component that knows its own timing, how many
candidates it considered and why it turned each one down. Returning only the
accepted objects would leave every caller unable to distinguish "the scene was
empty" from "twelve candidates were found and all were rejected as noise" - and
that distinction is exactly what makes a detection failure diagnosable.

**Alternatives considered:** Returning a list and exposing metrics on the
detector object (metrics would then belong to the detector rather than the
frame, and would race under any concurrent use); a second `run()` method
alongside `detect()` (two ways to do one thing, and an ML detector would have to
implement both).

**Impact:** The abstract signature changed while it had no implementations, so
nothing was broken. A future `MLObjectDetector` implements the same single
method and drops into the API and service layer unchanged.

**Status:** Accepted

## ADR-023: Motion Is Null Until Measured

**Decision:** `TrackedObject.velocity`, `acceleration` and `heading_rad` are
optional and default to `None`. A track that has been seen once reports no
motion at all, rather than a zero vector and a heading of zero.

**Reason:** The previous defaults were `Vector3(0, 0, 0)` and `0.0`, which are
indistinguishable from *measured* results - a stationary object pointing
straight ahead. Phase 4 requires that first-frame velocity be unknown, and the
contract could not express that. Every consumer reading a fresh track would
have seen a confident claim of a standstill that nothing had observed.

This is the same rule already applied to metrics, where an unmeasured value is
`None` with a stated reason rather than a plausible-looking zero. It also makes
`DetectedObject` and `TrackedObject` consistent: detection velocity was already
`Vector3 | None`.

The distinction is not academic. A stationary vehicle and a vehicle seen for the
first time are different situations, and a risk engine consuming tracks must be
able to tell them apart. Under the old contract it could not.

**Alternatives considered:** Keeping zero defaults and adding a
`has_velocity: bool` flag (two fields that can disagree, and every consumer must
remember to check the flag); leaving the contract alone and having the tracker
emit zeros (exactly the fabricated measurement the project's rules forbid).

**Impact:** `speed_mps` now returns `float | None`, and a new `is_moving`
property returns `None` when motion is unknown. One Phase 1 test asserted
`speed_mps == 0.0` on a default track; it was replaced with stronger assertions
that unknown motion is `None` and that a *measured* standstill is still
expressible and distinguishable. This is the only Phase 1-3 contract Phase 4
changed.

**Status:** Accepted

## ADR-024: Gated Greedy Nearest-Neighbour Association

**Decision:** Detections are matched to tracks by distance within a configurable
gate, considered in ascending distance and claimed greedily, with optional class
and size compatibility checks. Ties break on `(distance, track_id, detection
index)`.

**Reason:** The detector is geometric and produces no appearance features, so
position is the only signal available. Hungarian assignment would minimise total
distance and occasionally do better where two tracks compete for two detections,
but it needs `scipy.optimize` - a dependency the project has declined three
times now for the same reason (ADR-014, ADR-020) - and greedy matching is
explainable line by line. When tracking fails during a close crossing, being
able to read why matters more here than optimality.

The gate is the substantive part. Without it, an object appearing anywhere in
the scene could inherit the identity of an object that vanished on the far side,
which is worse than starting a new track.

**Alternatives considered:** Hungarian assignment (optimal, new dependency,
harder to explain); ungated nearest neighbour (silently teleports identities);
IoU-based matching (needs oriented boxes, which Phase 3 does not produce).

**Impact:** Association is `O(T x D)`. Per-detection work is hoisted out of the
inner loop and gating tests squared distance, measured at 0.06 ms for 5 objects
and 206 ms for 500. That is comfortable at the object counts this pipeline
produces - under a hundred in the large benchmark scene - and the wrong shape
for thousands, where spatial bucketing would be the fix.

Two objects passing close together can swap identities: centroid distance alone
cannot distinguish them at the crossing frame. The behaviour is deterministic
and tested, but it is a real failure mode, not a solved problem.

**Status:** Accepted

## ADR-025: Tracker State Lives in the Application Context

**Decision:** Tracking state is owned by a `TrackingService` held on the
`ApplicationContext`, reached through FastAPI dependencies. It is created at
startup, cleared at shutdown, resettable through
`POST /api/v1/tracking/reset`, and guarded by a lock.

**Reason:** Tracking is the first stateful part of ADAPT-X. Every earlier stage
is a pure function of one frame; a track exists only because of frames that came
before. That state needs an owner with a defined lifetime.

A module-level global was the obvious shortcut and the wrong answer: tests would
leak tracks into one another, two application instances in one process would
share tracks silently, and there would be no way to clear state without reaching
into a module. The context already owns every other long-lived service, so
tracking belongs there too.

**Alternatives considered:** A module-level tracker (leaks between tests and
between app instances); per-request trackers (defeats the point - tracking needs
memory across requests); per-session trackers (needs a session concept the API
does not have).

**Impact:** One tracker serves the whole process. Two clients posting frames to
the same backend therefore feed **one** track set - correct for a single vehicle
with one sensor stream, wrong for anything else. That limitation is documented
in the service and in the API rather than disguised. Shutdown resets tracking
explicitly, so a restarted context cannot inherit tracks from frames it never
saw.

The endpoint is also order-dependent in a way no other endpoint is: frames must
arrive in temporal order, and the request `timestamp` determines measured
velocity. Omitting it makes the measured interval the wall-clock gap between
HTTP requests rather than between scans.

**Status:** Accepted

## ADR-026: Deterministic Constant-Velocity Trajectory Prediction

**Decision:** Phase 5 implements `prediction.interfaces.TrajectoryPredictor` as a
**deterministic constant-velocity baseline**. For a track with a measured velocity, the
predicted position at `t` seconds after the prediction time is
`position + velocity * (age_s + t)`, sampled from `t+0` to a configurable horizon
inclusive. Positional uncertainty is a **heuristic** that grows linearly with
extrapolation time: `base_uncertainty_m + uncertainty_growth_mps * (age_s + t)`.

`age_s` is the *measured* interval between a track's last observation and the prediction
time. It is zero for a track matched in the current frame, so the formula collapses to
`p + v*t`; it is positive for a coasting track, whose stored position is already stale by
exactly that much. One formula therefore covers both cases without special-casing, and
ignoring `age_s` would silently pretend a missed frame never happened.

**Reason:** Constant velocity is the simplest model that uses only what Phase 4 actually
measures. It is explainable frame by frame - a wrong prediction traces to a wrong
velocity and nothing else - and deterministic, so the same tracks always produce the same
trajectory. That makes it a reference an ML predictor would have to beat, exactly as the
geometric detector (ADR-021) and tracker (ADR-024) are for theirs.

It is **not** a claim about how objects move. Real vehicles accelerate, brake, turn and
follow lanes. None of that is modelled, and the module says so in its docstring, its
component detail and its API documentation.

**Alternatives considered:**

- *Constant acceleration.* Rejected: `TrackedObject.acceleration` is a second difference
  of noisy centroids, so projecting it over a 3-second horizon amplifies detection jitter
  into metres of error. NEXT_PHASE.md explicitly excluded it.
- *Kalman filter / IMM.* Rejected for now: needs a process-noise model that cannot be
  chosen honestly without labelled data to fit it against, and IMM needs a mode set that
  would be guesswork. Both would also invite calling the resulting covariance a
  calibrated uncertainty, which it would not be.
- *Learned trajectory prediction.* Rejected: requires a labelled dataset the project does
  not have, and would add an ML framework that ADR-008 declined.
- *Map- or lane-conditioned prediction.* Rejected: there is no map. The 2.5D map is
  Phase 6 under the agreed renumbering.
- *Class-conditioned motion models.* Rejected as premature: the parameters would be
  invented rather than measured, and Phase 3 classification is itself a heuristic that
  returns `UNKNOWN` on ambiguity.
- *Using `heading_rad` to steer the extrapolation.* Rejected: heading is derived from
  velocity and is null below the tracker's speed floor. The velocity vector is the
  authoritative motion vector; forcing motion along a heading that disagreed with it
  would discard measured information.

**Why the uncertainty is heuristic, and labelled as such:** `position_uncertainty_m` is a
documented formula - not a calibrated sigma, not a probability, not a confidence
interval. Calibrating one needs labelled trajectories to measure error against, and none
exist. `GET /api/v1/prediction/status` reports `uncertainty_is_heuristic: true` and every
result carries `uncertainty_model: "heuristic_linear_growth"`, so a consumer cannot
mistake it for a validated quantity.

`base_uncertainty_m` is constrained strictly positive: a zero floor would claim a
perfectly known position, which no detection provides.

Point `confidence` is `track_confidence * (uncertainty(t+0) / uncertainty(t))`. It starts
at the track's evidence score and decays exactly as fast as uncertainty grows, so the two
numbers can never disagree. Track confidence combines the only two evidence signals the
tracker provides - `hits`, saturating at `confidence_hits_full`, and `missed_frames`. It
measures **evidence, not correctness**, in the same sense as the classifier's fit score
(ADR-021).

**Impact:** Prediction is reported `READY` / `PARTIAL`, never `IMPLEMENTED`. Predicted
positions live only in `PredictedTrajectory` and are never written back onto
`TrackedObject.position`. Prediction accuracy is unmeasured and currently unmeasurable;
the benchmark measures speed only.

Replacing the model later means writing a new `TrajectoryPredictor` and changing one line
in `build_context`. Nothing downstream depends on the model, only on the contract.

**Risks:** A constant-velocity path through a turn or a braking event is wrong, and it
looks exactly as confident as a correct one apart from its uncertainty radius. The risk
engine must treat `position_uncertainty_m` and `confidence` as first-class inputs rather
than using the positions alone.

**Status:** Accepted

## ADR-027: Track Eligibility and Skip Reporting in Prediction

**Decision:** The predictor reports what it declined to predict, and why. Every track
handed to it appears either in `PredictionResult.trajectories` or in
`PredictionResult.skipped` with a `PredictionStatus` and a human-readable reason; a model
validator enforces `considered == predicted + skipped`.

The eligibility rules:

| Condition | Outcome |
|---|---|
| `velocity is None` | skipped, `INSUFFICIENT_VELOCITY` |
| measured speed > `max_speed_mps` | skipped, `INVALID_VELOCITY` |
| last observation older than the horizon | skipped, `STALE_OBSERVATION` |
| `status is LOST` | skipped, `TRACK_LOST` |
| beyond `max_tracks` for this call | skipped, `LIMIT_EXCEEDED` |
| `CONFIRMED`, seen this frame | predicted, `PREDICTED` |
| `TENTATIVE` with a measured velocity | predicted, `PREDICTED`, lower confidence |
| `COASTING`, or any stale observation | predicted, `EXTRAPOLATED` |

**Reason:** This is the ADR-022 / ADR-024 rule applied to prediction. A caller given only
the surviving trajectories cannot tell a scene with no tracks from one where every track
was on its first frame - and those mean very different things to a risk engine.

The individual rules follow from ADR-023. `velocity is None` means *not measurable*, not
*zero*: emitting a flat "stays where it is" path would fabricate a measurement from
nothing. A measured `Vector3(0, 0, 0)` is genuinely different - it is an observed
standstill - and legitimately produces a stationary trajectory. The two cases are handled
separately and tested separately.

An over-speed velocity is **rejected, never clipped**: a clipped velocity is a number no
sensor produced, and substituting one silently would be exactly the fabrication the
project rules forbid.

`TENTATIVE` tracks are predicted rather than skipped when they have a velocity, because a
velocity measured from two observations is a real measurement however new the track is.
What differs is the amount of evidence, and that belongs in the confidence score rather
than in a binary include/exclude decision that would discard usable information.

`COASTING` tracks are predicted but marked `EXTRAPOLATED`, with `observation_age_s`
stating the measured staleness and uncertainty widened by it. `STALE_OBSERVATION` bounds
this: once the last observation is older than the horizon, the output would be more
gap-filling than prediction, so none is produced.

**Alternatives considered:** Returning a bare `list[PredictedTrajectory]` (loses every
skip reason); skipping tentative and coasting tracks entirely (discards measured velocity
and hides objects from the risk engine precisely when they are occluded); clipping
over-speed velocities (fabricates data); emitting a stationary trajectory when velocity is
null (the specific error ADR-023 exists to prevent).

**Impact:** `TrajectoryPredictor.predict` returns a `PredictionResult` rather than a list.
The abstract signature was widened to match; it had no implementations, so nothing broke.
`max_tracks` overflow is recorded rather than dropped, but the predictor does **not**
prioritise which tracks to keep - it has no risk signal to prioritise by. That is
Phase 6's job, and inventing a priority here would be an unmeasured heuristic dressed as a
safety feature.

**Status:** Accepted

## ADR-028: A Bounded Dense 2.5D Grid, With Half-Open Cells

**Decision:** The Phase 6 map is a **bounded, dense XY grid** held as NumPy arrays. Cells
are anchored at the map's lower corner and are half-open::

    column = floor((x - min_x) / resolution)
    row    = floor((y - min_y) / resolution)

A point exactly on `min_x`/`min_y` belongs to the first cell; a point exactly on
`max_x`/`max_y` is **out of bounds**. Arrays are indexed `[row, column]` with `row`
stepping along y, so `shape == (height, width)`. Grid dimensions are computed and validated
**before** any array is allocated, against a configured `max_cells` ceiling.

**Reason:** Every part of this has a specific failure it prevents.

*Bounded, not unbounded.* An unbounded grid sized from the data would change dimensions
frame to frame, so two maps of the same scene could not be compared or differenced - which
is precisely what the eventual fixed-versus-adaptive comparison needs to do.

*Dense arrays, not a list of cell objects.* The pre-existing `AdaptiveMap` contract holds
`list[AdaptiveMapCell]`. A 0.25 m map over a 120 m square is 230,400 cells; one validated
Pydantic object each would cost far more than the mapping itself, every frame. Holding a
NumPy array inside a contract is already the project's precedent -
`BasePointCloudFrame.points` does exactly that. `SpatialMap.to_adaptive_map()` projects
**occupied cells only** into the existing contract for consumers that want individual cells,
so nothing is lost.

*Half-open cells.* Without the rule, a point at exactly `max_x` indexes one cell past the
last column. The alternatives are worse: clamping it inward records a measurement in a cell
it does not belong to, and growing the grid by one cell makes dimensions depend on whether a
point happened to land on the edge.

*Dimensions before allocation.* A fine resolution over wide bounds can request an absurd
array. 0.05 m over the default 120 m square is 5.76 million cells and roughly 184 MB across
the four arrays. Checking `width * height` first turns that into an explicit, explained
rejection rather than a memory event.

*Reusing the Phase 2B quantiser.* `perception.grid.cell_indices` carries the int64 overflow
guard added in Phase 2B, where `astype(int64)` was found to wrap silently and place
far-apart points in one cell. Mapping calls it on translated coordinates rather than writing
a second `floor(...).astype(int64)` that would reintroduce the same defect.

**Alternatives considered:** A sparse dict keyed by cell (cheaper for empty maps, but
unpredictable cost, worse cache behaviour, and no natural array to hand a future risk
engine); a full 3D voxel grid (the project is explicitly 2.5D - `05_2.5d-mapping.md`); a
quadtree or other multi-resolution structure (this is what adaptive resolution may
eventually need, and building it now would prejudge Phase 8 with no measurement behind it);
an unbounded grid sized per frame (see above).

**Impact:** Map memory is fixed by bounds and resolution, not by point count, and is
predictable: `width * height * 32` bytes. The grid is directly consumable by a future risk
engine as an array rather than needing conversion. `range_m` in `MapSettings` is retained
for the pre-existing status response but is **not** what bounds the map; the explicit
`min_x_m`/`max_x_m`/`min_y_m`/`max_y_m` fields are.

**Risks:** Dense storage wastes space on a sparse scene - at 0.25 m the measured occupancy
was 1-16% (Experiment 005), so most cells hold nothing. That is a real cost of uniform
resolution, and measuring it is part of the point: it is the number adaptive resolution
exists to improve.

**Status:** Accepted

## ADR-029: Resolution Policy Is Separated From The Mapper

**Decision:** A mapper never chooses its own resolution. It is handed a
`ResolutionDecision` - `resolution_m`, `source`, `reason`, `requested_by` - and applies it.
Two distinct contracts sit either side of the decision::

    ResolutionContext -> [ResolutionController] -> ResolutionDecision -> AdaptiveMapper
       (inputs)              (not implemented)         (output)            (Phase 6)

`ResolutionContext` already existed and is **unchanged**: it carries risk, uncertainty,
object density, object speed and ego-path membership - the *inputs* a controller reasons
over. `ResolutionDecision` is new and carries the *outcome*. Phase 6 produces decisions with
`source = FIXED` only.

**Reason:** The central ADAPT-X claim is that risk-aware resolution beats uniform
resolution. That claim is only testable if the two halves - deciding and applying - can be
varied independently. Fusing them would make "the map" and "the policy" one thing, so a
change in either could not be attributed.

More concretely: the mapper is never *given* tracks, trajectories, risk or uncertainty. A
risk-aware choice is therefore structurally impossible here rather than merely discouraged
by a comment. When a controller exists, it produces a different `ResolutionDecision` through
the same contract and `FixedResolutionMapper` does not change.

The two contracts deliberately keep separate names. Reusing `ResolutionContext` for the
decision would have destroyed the input contract Phase 8 needs, and would have put risk
fields inside the mapper's argument - the exact coupling this decision exists to prevent.

**Alternatives considered:** Repurposing `ResolutionContext` as the decision (breaks the
Phase 8 contract, and reintroduces the coupling); passing a bare `float` (loses provenance,
so a benchmark could not tell a configured baseline from a one-off override); letting the
mapper read settings directly and choose (fuses policy into the mapper); implementing a
trivial distance-banded "adaptive" policy now (it would be an unmeasured heuristic wearing
the name of the thing the project has yet to justify).

**Impact:** `AdaptiveMapper.update(frame, *, ego_state, tracks, risk_field)` became
`AdaptiveMapper.build(frame, resolution)`. The old signature handed the mapper exactly the
material it must not use. It had **no implementations and no importers** outside its own
module, so this is safe - the same situation, and the same precedent, as widening
`TrajectoryPredictor.predict` in Phase 5 (ADR-027).

`ResolutionSource` reserves `ADAPTIVE` and `OVERRIDE` alongside `FIXED`, so the contract
does not change when a controller arrives, and a benchmark sweeping cell sizes is
distinguishable from the configured baseline.

**Risks:** The split is only as good as its enforcement. If a later mapper starts accepting
tracks "just for logging", the boundary is gone. The interface is the guard, and it should
stay narrow.

**Status:** Accepted

## ADR-030: Mapping Is Frame-Local, Not A Persistent World Map

**Decision:** Every mapping call builds a **complete map from one frame**. Nothing
accumulates between frames. `MappingService` holds no map state - only a counter and the
last map's *summary*, for status and telemetry. `AdaptiveMapper.reset()` is retained by the
contract and is a documented **no-op** for the Phase 6 mapper.

**Reason:** Accumulation is a much larger commitment than it looks. A persistent map needs
ego-motion compensation to know where the vehicle has moved between frames, a decay or
eviction policy so stale observations do not linger as phantom obstacles, and a way to
distinguish a moving object's trail from a static structure. ADAPT-X has none of those:
there is no localisation, and `VehicleState` is a contract nothing populates. A map that
accumulated without them would smear every moving vehicle into a wall - and would look
plausible while doing it.

Frame-local mapping is also what makes the map *comparable*. Two mappers run over the same
frame produce two maps that differ only by mapper, which is exactly the measurement ADR-003
requires. With accumulation, each map would also depend on its own history.

The no-op `reset` is deliberate and is asserted by a test. Being unable to contaminate the
next frame is the guarantee; keeping the method means a future accumulating mapper can
implement it without changing any caller.

**Alternatives considered:** A rolling multi-frame buffer (needs ego-motion compensation
that does not exist); temporal occupancy fusion (needs a sensor model and a decay policy,
both of which would be invented rather than measured); full SLAM (out of scope by several
phases, and not what ADAPT-X is about).

**Impact:** Map cost is bounded and predictable per frame. `POST /api/v1/lidar/map` is
stateless, unlike `/track` and `/predict`, so frames may be posted in any order and no reset
endpoint is needed. Documented plainly everywhere it appears: this is not SLAM, not a
persistent world map, and there is no localisation or loop closure.

**Risks:** Occlusion is not remembered. A cell hidden behind a vehicle this frame is
reported unobserved even if it was seen clearly a moment ago, and nothing carries that
knowledge forward. Temporal fusion is the natural future improvement, and it needs the
machinery listed above first.

**Status:** Accepted

## ADR-031: Binary Occupancy, And Null Height For Unobserved Cells

**Decision:** Occupancy is **binary and derived**, not stored: a cell is occupied exactly
when `point_count > 0`. Height statistics for a cell with no points are **NaN** in the
arrays and `null` in the serialised contracts - never `0.0`.

**Reason:** *Binary occupancy* is what the evidence supports. A probability would need a
sensor model - detection likelihood, false-return rate, incidence angle - and a Bayesian
update needs multiple observations of the same cell, which frame-local mapping (ADR-030)
does not have. A single scan can honestly say "at least one return landed here" and no more.
Deriving occupancy from the count rather than storing it separately means the two can never
disagree.

*Null height* follows the same reasoning as ADR-023 did for velocity. In this coordinate
frame `z = 0` is a real height roughly 1.8 m above the road - the sensor's own plane. An
unobserved cell reporting `0.0` would be indistinguishable from a measured flat surface at
sensor height, which is both wrong and dangerous-looking to any consumer that later reasons
about clearance. NaN in the array, `None` in `AdaptiveMapCell.height_m`, and the existing
contract already types those fields as optional.

`min`/`max` use `np.fmin`/`np.fmax`, which ignore NaN, so an untouched cell simply keeps its
initial NaN - the unobserved state is the default rather than something that has to be
restored afterwards.

**Alternatives considered:** Probabilistic occupancy (needs a sensor model that would be
invented); log-odds Bayesian updates (needs temporal fusion, which ADR-030 declines);
sentinel heights such as `-9999` (a magic number that arithmetic will silently consume);
`0.0` for empty cells (indistinguishable from a real measurement - the specific error this
decision exists to prevent).

**Impact:** `AdaptiveMapCell.occupancy` is a float documented as a probability, and it
predates Phase 6. The baseline populates it with `1.0` only, never an intermediate value,
and sets `occupancy_state = OCCUPIED`; the discrete field is authoritative. Consumers must
handle NaN - `np.nanmin`, `np.isnan` - rather than assuming every cell has a height. The
projection converts NaN to `None` at the contract boundary so JSON consumers see `null`.

**Risks:** "No return" and "empty space" are conflated. A cell may be unobserved because
nothing is there, or because something occluded it, and Phase 6 cannot tell the difference.
That distinction matters for safety and is the first thing a future occupancy model should
add; until then, an unobserved cell must never be read as free space.

**Status:** Accepted

## Decision Template

### ADR-XXX: Title

**Decision:**

**Reason:**

**Alternatives considered:**

**Impact:**

**Risks:**

**Status:** Proposed / Accepted / Superseded
