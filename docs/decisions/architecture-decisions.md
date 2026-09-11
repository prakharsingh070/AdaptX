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

## ADR-032: A Deterministic Heuristic Risk Score, With Missing Factors Dropped

**Decision:** Phase 7 scores each tracked object with a **deterministic engineering
heuristic** over three normalised factors - proximity, rate of approach, and how close the
predicted path passes - combined as a weighted mean over the factors that are **actually
available**:

```
risk_score = sum(w_i * f_i) / sum(w_i)     over available i only
```

A factor that could not be computed is **dropped and the remaining weights renormalise**. It
is never substituted with zero. When no factor at all can be computed, the object is reported
`RiskLevel.UNKNOWN` with `risk_score = None`.

**Reason:** The renormalisation rule is the whole decision. The obvious alternative - treat a
missing factor as 0.0 - is quietly catastrophic here, because *low* is exactly what an
unmeasured value would look like. A track whose velocity has never been measured would score
as though it were standing still, and the object we know least about would look like the one
we need to worry about least. That is the precise inversion the project's honesty rules exist
to prevent, and ADR-023 already settled the same question for velocity itself.

Renormalising says something narrower and true: *of the evidence we have, this is how
concerning the object is.* The fact that evidence is missing is not lost - it is reported in
the uncertainty breakdown, which is where a consumer can act on it.

`UNKNOWN` with a null score exists for the same reason. A terminated track, or one where
nothing could be computed, must not be handed a number; a number would be indistinguishable
from a measurement.

Three factors was a deliberate floor rather than a starting point. Each one uses a quantity
the pipeline genuinely measures. Object class, map occupancy and time-to-collision were all
considered and left out - see below.

**Alternatives considered:**

- *Treat missing factors as zero* (the inversion described above).
- *Skip objects with incomplete data.* Worse: it hides the object entirely, and an object we
  cannot fully assess is not an object we can ignore.
- *A fixed formula requiring all factors.* Would make prediction and map mandatory inputs,
  when both are legitimately absent on a first frame.
- *Object class as a score multiplier.* Rejected: it would encode an unmeasured judgement
  that a pedestrian is inherently N times more concerning than a vehicle. Class is reported
  on the assessment and appears in the explanation, so a later phase can weigh it with
  evidence this project does not yet have.
- *Map occupancy as a score term.* Rejected under ADR-034 - see there.
- *Time-to-collision.* Rejected as premature: TTC over a constant-velocity extrapolation with
  heuristic uncertainty would produce a precise-looking number resting on two approximations.
  It is named in `unmodelled_factors` rather than approximated.

**Impact:** Risk is reported `READY` / `PARTIAL`, never `IMPLEMENTED`. The status endpoint
reports `score_is_heuristic: true` and `is_collision_probability: false`, and no accuracy
figure appears anywhere - there is no labelled risk data to measure one against. Thresholds
reuse the existing `threshold_medium/high/critical`, so this engine and the proximity
baseline classify on the same scale and stay directly comparable.

`RiskLevel.UNKNOWN` was **added** to the existing enum. `CRITICAL` is retained: it predates
Phase 7, is exercised by tests, and appears in the API thresholds. `UNKNOWN` has no
threshold, because it is not a point on the scored scale.

**Risks:** A weighted mean is easy to explain and easy to over-read. The score orders objects
by concern; it does not measure anything physical, and two objects scoring 0.6 are not
"equally likely" to do anything. The weights are baseline engineering values that have never
been tuned against outcomes, because no outcomes have been recorded.

**Status:** Accepted

## ADR-033: Uncertainty Is Reported Beside Risk, Never Folded Into It

**Decision:** Uncertainty is **not** a term in the risk score. Every assessment carries a
separate `UncertaintyBreakdown` - a heuristic scalar on `[0, 1]` plus the list of reasons that
produced it: unknown velocity, stale observation, low track confidence, no prediction, wide
prediction uncertainty, tentative track, coasting track, unobserved map context, no map.

**Reason:** The pre-existing contract in `models/risk.py` already states the principle - *an
object can be low-risk but uncertain, or high-risk and well observed* - and Phase 7 keeps it.

The reason it matters is Phase 8. The ADAPT-X thesis is that perception effort should follow
risk **and uncertainty**: a poorly observed region may deserve finer perception precisely
*because* it is poorly observed, even when its computed risk is low. Summing uncertainty into
the score would collapse two independent signals into one number and destroy exactly the
distinction a resolution controller most needs. A low score would then be ambiguous between
"we looked and it is quiet" and "we could barely see it".

Keeping the reasons visible beside the scalar matters for the same reason. A consumer can act
on the cause - a stale track and an unconfirmed one are different problems with different
remedies - rather than on an opaque number.

The weights per reason are baseline engineering values, with unknown velocity weighted
highest because it is the single largest blind spot: it removes a factor from the score
*and* leaves the object's motion entirely unmodelled.

**Alternatives considered:** A single blended "confidence-adjusted risk" (destroys the
signal, as above); uncertainty as a multiplier on risk (makes an uncertain object look
*safer*, which is backwards); a bare scalar with no reasons (unactionable); modelling
uncertainty as a variance (would imply a distribution that has never been characterised).

**Impact:** Two tracks identical except for observability score **identical risk** and
different uncertainty - asserted by test. `RiskAssessmentResult` exposes `max_uncertainty`
alongside `highest_risk_level`, so a scene can report "quiet but poorly observed" as a
first-class state.

**Risks:** A consumer that reads only `risk_score` gets a materially incomplete picture. The
API detail string, the status endpoint and the component detail all say so, but nothing can
force a consumer to look.

**Status:** Accepted

## ADR-034: Unobserved Map Cells Never Reduce Risk

**Decision:** Map context is reported on every assessment as one of `OBSERVED_OCCUPIED`,
`OBSERVED_EMPTY`, `OUT_OF_BOUNDS` or `NO_MAP`, and it is **never used to lower a risk score**.
An empty or out-of-bounds cell raises *uncertainty* instead.

**Reason:** This is ADR-031 carried into the phase that could most easily violate it. Phase 6
records where LiDAR returns landed. A cell with no returns may be empty, or it may be
occluded - the map cannot tell the difference, and it says so.

The tempting move is the dangerous one: if a predicted path crosses cells with no points,
lower the risk. That would treat *unobserved* as *clear*, and would reduce risk exactly where
the sensor could see least - behind the vehicle that is occluding the pedestrian. It is the
single most dangerous inference available in this phase, and it is precisely the inference a
naive "free space" reading invites.

So map context is a **context signal, not a scoring term**. It tells a consumer what the
sensor actually saw, and it is a documented interface point for a future phase that can model
occlusion properly.

**Alternatives considered:** Treating empty cells as free space (the error above); scoring
map occupancy as a fourth factor (would either reward occlusion or need an occlusion model
that does not exist); ray-casting occlusion now (needs sensor-origin geometry and a proper
visibility model - a phase of its own); omitting map context entirely (loses a real signal,
and loses the interface Phase 8 will want).

**Impact:** Adding a map to an assessment can only leave the score unchanged or raise
uncertainty - asserted by test. `MapContext.is_observed` is true **only** for
`OBSERVED_OCCUPIED`, so a consumer cannot accidentally read "we looked and it was empty" out
of a value that means "no returns landed here".

**Risks:** The map component is therefore quite weak: it contributes nothing to the score. It
earns its place as an honest interface and an uncertainty source, and a future occlusion
model can strengthen it without changing the contract.

**Status:** Accepted

## ADR-035: Scene Risk Aggregates By Maximum, Never By Mean

**Decision:** `RiskAssessmentResult.highest_risk_level` is the **maximum** scored level
present, not an average. A scene with no scored objects reports `UNKNOWN`, not `LOW`. Counts
per level are reported alongside, including `unknown_count`.

**Reason:** Averaging is the obvious aggregate and the wrong one. Ten quiet objects and one
critical object average to something reassuring, and the one object that matters disappears
into the arithmetic. A scene-level signal exists to answer "is anything concerning right
now", and only the maximum answers that.

Reporting `UNKNOWN` for an unscored scene follows the same reasoning as ADR-032. "Nothing is
risky" and "nothing could be assessed" are different states, and an empty scene reporting
`LOW` would be a claim about a scene nobody looked at.

The per-level counts are kept because the maximum alone loses scale: one high-risk object and
forty are different situations, and a consumer should not have to re-derive that.

**Alternatives considered:** Mean or median score (buries the outlier); top-N average (same
problem, softened); a count-weighted composite (invents a scale nothing calibrated); treating
an empty scene as `LOW` (a claim without evidence).

**Impact:** `highest_risk_score` and `max_uncertainty` return `None` rather than `0.0` on an
empty scene. Telemetry ranks objects by score descending with `track_id` breaking ties, so
the same result always produces the same ordering.

**Risks:** A maximum is sensitive to a single spurious assessment - one bad detection can
raise the whole scene's reported level. That is the intended failure direction, and the
per-level counts let a consumer see it is a single object.

**Status:** Accepted

## ADR-036: Risk Does Not Decide Resolution

**Decision:** The risk engine evaluates concern and stops. It does not choose spatial
resolution, does not construct a `ResolutionDecision`, does not import `ResolutionController`
or `MapSettings`, and `RiskAssessment` carries **no** `resolution_m`, cell size or resolution
level. The chain of custody is:

```
tracks + trajectories + map -> RiskEngine -> RiskAssessment
                                                  |
                                                  v
                              [Phase 8 ResolutionController] -> ResolutionDecision -> mapper
```

**Reason:** These are two different questions. *How concerning is this object?* is a
perception judgement. *How much spatial detail should this region receive?* is a resource
allocation decision that also depends on the compute budget, the map geometry and a
stabilisation policy that stops resolution oscillating between frames.

Fusing them would make the project's central claim untestable. ADAPT-X asserts that
risk-aware resolution beats uniform resolution; demonstrating that requires holding the risk
formulation fixed while the allocation policy varies, and vice versa. One module doing both
means neither can be attributed.

It also keeps the interface honest by construction, the same way ADR-029 did for the mapper.
The engine is never *given* a cell size or a resolution vocabulary, so a resolution decision
cannot leak in even by accident.

**Alternatives considered:** Emitting a suggested resolution alongside the risk (Phase 8's
decision, made in Phase 7 without the inputs it needs); tagging assessments with a resolution
level (the same thing wearing an enum); letting the engine write `AdaptiveMapCell.risk_score`
directly (couples risk to a map representation and skips the controller entirely).

**Impact:** Phase 8 consumes `RiskAssessment` - `track_id`, `risk_level`, `risk_score`,
`uncertainty`, `distance_m`, `closing_speed_mps`, `trajectory`, `map_context`, `factors` - and
produces a `ResolutionDecision` through the contract Phase 6 already applies. Neither the
mapper nor the risk engine changes when it arrives. The boundary is asserted by tests at both
the model and the wire format, and `GET /api/v1/risk/status` reports
`decides_resolution: false`.

**Risks:** Two phases must agree on a contract before either is finished, so a genuinely
unforeseen Phase 8 need may require an additive field. Additive is the operative word: the
separation should survive it.

**Status:** Accepted

## ADR-037: Region-Partitioned Tiles for Multi-Resolution Maps

**Decision:** An adaptive map partitions the extent into fixed-size square **tiles**, each
holding its own dense sub-grid at its own cell size. Tiles are anchored at the map lower
corner, half-open on their upper edges, and the last row and column are clipped to the
bounds. Resolution is decided per tile, never per point.

`SpatialMap` is unchanged and remains the fixed-resolution baseline. The adaptive map is a
new contract, `AdaptiveSpatialMap`, holding `MapTile` objects that carry the same per-cell
quantities a Phase 6 cell carries, and reusing `MapBounds` and `MapAccounting` rather than
restating them.

**Reason:** A single dense NumPy grid has exactly one cell size, so an adaptive map cannot be
one. Something had to give, and the question was what.

Tiling keeps the two properties that matter most. **Lookup stays arithmetic**: the tile of a
point is one floor division and the cell within it is another, so binning a frame is still
two vectorised passes rather than a tree descent. **The partition is exact**: tiles cover the
extent with no gap and no overlap, so a point lands in exactly one cell and point accounting
survives unchanged - `input == mapped + out_of_bounds` still holds and is still
model-enforced.

It also keeps Phase 6 intact. The binning arithmetic is the same code path per tile, quantised
through the same overflow-guarded `cell_indices`, so the adaptive mapper is not a second
implementation that can drift from the first.

**Alternatives considered:**

*Multiple passes at several uniform resolutions, composed.* Simplest, and reuses the Phase 6
mapper untouched - but it allocates several full grids to produce one map, which is the exact
cost adaptive resolution exists to avoid. It also leaves the composition rule undefined where
two passes disagree.

*A quadtree or hierarchical grid.* The most cell-efficient option, and the most complex.
Lookup becomes a tree descent, serialising it for a dashboard is awkward, and the refinement
rule interacts with the stabilisation policy in ways that would be hard to test. Deferred
rather than rejected: if per-region overhead ever stops dominating (Experiment 007), this is
where to go next.

*Sparse per-cell storage keyed by coordinate.* Flexible, but a dictionary of cells costs far
more per cell than a dense array and makes the whole-grid operations trivial today.

**Impact:** One map genuinely holds several resolutions, and `AdaptiveMapCell.resolution_m`
and `resolution_level` - contracts that existed since Phase 1 and had only ever carried one
value - now carry real varying ones. `tile_size_m` becomes the lever that trades spatial
precision of the allocation against per-region overhead; Experiment 007 measures that cost
directly. `AdaptiveMapper` gains a sibling contract, `RegionAdaptiveMapper`, because a single
`ResolutionDecision` cannot describe a map at several resolutions.

**Risks:** Allocation is quantised to the tile, so a small object refines a whole region
around it - detail is spent on ground that did not ask for it. Region count, not cell count,
drives cost, which is the opposite of the intuition Phase 6 built. A tile size much smaller
than the objects would make both problems worse.

**Status:** Accepted

## ADR-038: Detail Priority is an Engineering Score, and Unknown is Not Low

**Decision:** A region receives a **detail priority** in `[0, 1]`: a weighted mean over six
normalised factors - risk, uncertainty, proximity, predicted-motion relevance, object density
and measured motion - taken over the factors **actually available**. A factor that cannot be
computed is dropped and the remaining weights renormalise; it is never scored zero.

Two consequences are load-bearing:

- `risk_score is None` removes the risk factor **and** raises a floor on the region level
  (`unknown_risk_min_level`, MEDIUM by default). An unscored object may never leave its
  region at the coarsest level.
- A region no object influences has `detail_priority = None`, not `0.0`, and takes the
  configured base level with `no_object_influence` recorded.

**Reason:** This is ADR-032 one phase later, and the failure it prevents is worse here. In
Phase 7 coercing an unknown risk to zero would have mis-ranked an object. In Phase 8 it would
hand the **coarsest spatial representation** to precisely the objects the system understands
least - a lost track, an unconfirmed one, an object whose velocity was never measured. The
system would see least where it knows least.

Priority is kept separate from risk for the same reason ADR-036 separates risk from
resolution: they answer different questions. *How concerning is this object* is a perception
judgement over an object. *How much detail does this region deserve* is a resource allocation
over space, and it depends on things risk does not model - how many objects are nearby, how
well observed they are, and what the compute budget allows.

Uncertainty enters as an **independent factor**, never summed into risk (ADR-033). That is
the whole payoff of having kept them apart: a region can be quiet and badly observed, and
that is an argument for looking harder, not for looking less.

**Alternatives considered:** Reusing the Phase 7 risk score directly as the priority (throws
away uncertainty, density and predicted motion, and makes ADR-033 pointless); summing risk and
uncertainty into one number (destroys the distinction the moment it is needed); treating a
missing factor as zero (the inversion above); a learned policy (no labelled data exists to
learn from, and it would make every decision unexplainable).

**Impact:** Every decision carries its factors, their normalised values, the tracks that
influenced it, which of those could not be scored, and a generated explanation. A parametrised
test asserts the text never contains "probability", "calibrated", "validated", "guaranteed" or
"safe".

**Risks:** The weights and thresholds are baseline engineering values, never tuned against
outcomes because no outcomes have been recorded. The priority orders regions; it measures
nothing physical. Quality is bounded by Phase 7 risk, which is bounded by prediction, tracking
and detection, all of which are baselines.

**Status:** Accepted

## ADR-039: Asymmetric Hysteresis Plus Minimum Dwell Time

**Decision:** Resolution changes are stabilised by two mechanisms that pull in the same
direction:

- **Refinement is immediate.** A region proposed a finer level takes it on that frame.
- **Coarsening is resisted twice.** The priority must first fall a configured
  `hysteresis_margin` below the band the region currently holds before a coarser level is even
  *proposed*; and a coarser level must then be proposed on `min_dwell_frames` consecutive
  frames before it is *applied*.

The controller therefore holds state: each region remembers its level and how long a coarser
one has been proposed. That state lives on the application context and is cleared on shutdown
and by `POST /api/v1/map/adaptive/reset` (ADR-025).

**Reason:** A region whose priority sits on a threshold would otherwise flip every frame -
0.61, 0.59, 0.61, 0.59 - rebuilding its grid each time. That is worse than a uniform map: it
costs more, and it produces output no consumer can rely on.

The asymmetry is the interesting half. Detail is cheap to gain and expensive to lose at the
wrong moment: the cost of refining a region that turns out not to need it is some wasted
cells, while the cost of coarsening one that did need it is missing structure in the region
the system was most concerned about. The policy is deliberately biased towards keeping detail.

Both mechanisms are needed. The margin alone still flips a region whose priority oscillates
with a wide amplitude; the dwell time alone still flips one that hovers exactly on a boundary,
because every frame proposes a genuine change.

**Alternatives considered:** Exponential smoothing of the priority (delays refinement as much
as coarsening, which is the wrong trade, and hides the raw value); a single symmetric
hysteresis band (leaves the wide-amplitude case flickering); a fixed refresh interval
(decouples resolution from the scene, which defeats the purpose); no stabilisation at all
(measured to oscillate, and the knowledge base requires a documented mechanism).

**Impact:** The controller is the only stateful component in the adaptive path, and the only
mapping state that survives a frame. Occupancy still accumulates nowhere (ADR-030) - what
persists is a policy decision, not a measurement. Frames must arrive in temporal order and
unrelated sequences must be separated by a reset, the same contract `/track` and `/predict`
already carry. Tests drive a region through threshold jitter, a sustained rise, a sustained
fall and an object disappearing.

**Risks:** A genuinely quiet region keeps its detail for up to `min_dwell_frames` after it
stops needing it. A scene that changes faster than the dwell time will lag. The parameters are
engineering values, never tuned against a real scene.

**Status:** Accepted

## ADR-040: Bounded Regions and Cells, with Demotion Reported

**Decision:** An adaptive plan is bounded by three configured ceilings: `max_tiles` (checked
before any per-region work), `max_fine_tiles` (regions at HIGH or CRITICAL) and
`max_total_cells`. When a scene wants more detail than the budget allows, the controller
**coarsens the lowest-priority regions first**, one level at a time, until the plan fits.

Ordering is by priority ascending with `tile_index` breaking ties, and a region with no
priority at all sorts first. Every demotion is recorded on the decision
(`budget_demoted` / `fine_tile_limit`) and counted in the plan `ResolutionBudget`, which also
reports `within_budget: false` when a ceiling could not be met even after demotion.

**Reason:** Adaptive allocation without a ceiling is unbounded allocation. A dense scene, a
tile size much smaller than the objects, or a configuration mistake would otherwise ask for an
arbitrary amount of memory - and the failure would arrive as an allocation, not as a message.

Giving up detail where it matters least is the only defensible way to fit a budget. The
alternative - refusing the frame - turns a degraded map into no map, which is worse for a
perception system that must produce something every frame.

Reporting the demotion is what keeps it honest. A budget that silently coarsens the map would
make a benchmark result describe the budget rather than the policy, and Experiment 007 would
be measuring the wrong thing without knowing.

**Alternatives considered:** Rejecting a plan that exceeds the budget (no map at all); scaling
every region down uniformly (throws away the allocation the policy just computed); an
unbounded map with a warning (the warning arrives after the memory does).

**Risks:** Under a binding budget the map is coarser than the policy asked for, and a reader
who ignores `demoted_tile_count` could mistake the result for the policy own choice. The
ceilings are engineering values chosen to be generous; on the default configuration they do
not bind, which is why the dense benchmark scenario exists.

**Status:** Accepted

## ADR-041: The Controller Reads Positions from Tracks, Not Assessments

**Decision:** The resolution controller consumes `RiskAssessment` for concern and uncertainty,
`PredictedTrajectory` for predicted-motion relevance, and `TrackedObject` **for position**. An
assessment whose track is absent is excluded with `missing_position` rather than placed by
guesswork.

**Reason:** Spatial allocation needs to know *where* an object is, and a `RiskAssessment`
deliberately does not say: it carries `distance_m`, a scalar, because Phase 7 was given no
spatial vocabulary on purpose (ADR-036). A distance alone describes a circle around the ego,
not a region.

The alternative was to add a position to `RiskAssessment`. That was rejected: it would push a
spatial concept into the layer ADR-036 exists to keep free of one, for the benefit of a single
consumer, when the position is already available beside the assessment in every caller. Tracks
and assessments are produced together and keyed by the same `track_id`.

**Alternatives considered:** Adding `position` to `RiskAssessment` (erodes ADR-036 for one
consumer); reconstructing a position from `distance_m` and a heading (fabricating a
measurement); having the controller re-read detections (a second association path that could
disagree with tracking).

**Impact:** `AdaptiveMappingService.run_from_pipeline` takes the tracking result alongside the
risk result. Neither Phase 6 nor Phase 7 changed for Phase 8 - the boundaries drawn in ADR-029
and ADR-036 held, which is the outcome `NEXT_PHASE.md` asked to be reported either way.

**Risks:** The controller joins two collections on `track_id` and must handle a mismatch;
it does, by excluding and recording rather than skipping silently.

**Status:** Accepted

## ADR-042: CARLA is a Data Source Behind an Adapter Boundary

**Decision:** CARLA sits **upstream** of the pipeline, not beside it. Everything
simulator-specific lives in `adaptx.carla` and stops there; the boundary emits a
`RawPointCloudFrame` and hands it to the existing ingest path. No module outside that
package imports `carla`, and no downstream stage receives a simulator object.

Two objects divide the work, because they answer different questions with different
lifetimes. `CarlaClient` owns the **connection** and backs the status endpoint.
`CarlaSimulationSession` owns a **simulation**: deterministic settings, actors, the sensor,
ticking and cleanup.

**Reason:** The alternative that destroys the project is a second perception stack - a CARLA
path that detects, tracks and maps separately from the sensor path. Everything downstream
would then have two behaviours to maintain and two sets of results to reconcile, and the
Phase 11 comparison would be measuring the plumbing rather than the perception.

Keeping the boundary at ingest means the simulator changes exactly one thing: where the
points came from. Phases 2-8 needed no modification at all to consume CARLA, which is the
evidence the boundary is in the right place.

The client/session split exists because "am I connected to a server?" and "is a simulation
running?" are genuinely independent - a connected server with no session is a real and
ordinary state - and because actor lifetimes belong to whatever spawned them, not to a
long-lived connection handle.

**Alternatives considered:** Extending `CarlaSimulatorClient` with sensors and ticking (one
object owning a connection *and* actor lifetimes, so a status query and a simulation would
share state); a CARLA-specific ingest endpoint (a second path through preprocessing);
converting CARLA data into `PointCloudFrame` directly (skips Phase 2A validation, which is
where a frame earns its finite-value guarantee, ADR-010).

**Impact:** `python -m adaptx.carla.smoke` drives the boundary. The pipeline is untouched.
Tests assert structurally that no module outside `adaptx.carla` imports `carla`, and that
importing the API never requires the package.

**Risks:** The session duplicates a little of what the client could do. That is the price of
not having one object own two lifetimes, and it is paid once.

**Status:** Accepted

## ADR-043: One Coordinate Conversion, at the Boundary, in a CARLA-Free Module

**Decision:** CARLA's left-handed frame (+y **right**) becomes ADAPT-X's right-handed frame
(+y **left**) exactly once, in `adaptx.carla.conversion`. That module imports no simulator:
it takes plain floats, bytes and arrays. Configuration - including the sensor mount offset -
is expressed in the **ADAPT-X** convention, and the one value travelling outward is converted
by the same function.

**Reason:** ADR-009 predicted this conversion and named Phase 9 as its trigger. What it could
not predict is the failure mode, which is why the placement matters: **a dropped sign flip
is silent**. Every object appears on the wrong side of the vehicle, nothing raises, no
contract is violated, and the pipeline produces confident output about a mirrored world.

That makes testability the deciding constraint. Putting the conversion in a module with no
`import carla` means the arithmetic can be tested exhaustively on any machine, including one
with no simulator - which is every machine this project has run on so far. Had it lived in
the session, it would have been reachable only through a stand-in simulator, and the tests
would have been testing the stand-in.

Configuring the mount in ADAPT-X coordinates follows from the same rule: a reader should
never have to ask which convention a number is in. One function converts it outward at the
moment it is handed to CARLA.

**Alternatives considered:** Converting inside the session (untestable without CARLA);
converting downstream, per consumer (the scattering ADR-009 exists to prevent); configuring
the mount in CARLA's frame (mixes conventions in the one file a user actually edits);
adopting CARLA's frame project-wide (contradicts ADR-009, ISO 8855 and the yaw definition
already in the Phase 1 models).

**Impact:** Yaw converts with the same handedness change expressed as an angle
(`heading = -yaw`), so positions and orientations cannot disagree. An end-to-end test places
a target to the ego's left and asserts detection reports it on the left - which fails if the
flip is ever removed.

**Risks:** CARLA's pitch and roll are not converted, because nothing consumes them yet. A
tilted mount would need them, and that is the trigger to extend this module rather than to
work around it.

**Status:** Accepted

## ADR-044: Synchronous Simulation and Simulation-Authoritative Time

**Decision:** CARLA runs in synchronous mode with a fixed timestep, stepped explicitly. Frame
timestamps come from the simulator clock, anchored to a fixed epoch - never from
`time.time()`, `datetime.now()`, or a sleep. Scenario motion is scripted by setting
transforms, not driven by physics or autopilot.

**Reason:** Three phases already depend on temporal semantics, and all three break quietly
under a free-running simulator. Phase 4 measures velocity from the interval between frames.
Phase 5 extrapolates over that interval. Phase 8 counts frames for its dwell time. Wall-clock
timestamps would make velocity a function of how busy the machine was, and a re-run of the
same scenario would produce different tracks, different trajectories and a different
resolution allocation.

Simulation time is a duration since server start, so it is anchored to a fixed epoch to
become an absolute, ordered instant. The epoch's value is arbitrary and means nothing; that
it never changes is the point.

Scripted motion rather than physics for the same reason: CARLA's physics is not
frame-reproducible, and Phase 9 needs repeatable observations rather than realistic dynamics.

**Alternatives considered:** Asynchronous mode (non-deterministic sensor delivery, and
tracking silently degrades); wall-clock timestamps (velocity becomes a measure of machine
load); autopilot or the traffic manager for motion (introduces simulator randomness the
scenario cannot control); sleeping between ticks (slower and still not deterministic).

**Impact:** The session captures world settings before changing them and restores them on
close, including after a failed setup - a server left in synchronous mode blocks on a client
that has gone away and looks to the next user like a hung simulator. Tests assert the
interval is exactly `fixed_delta_seconds` and that two runs produce identical frames.

**Risks:** Synchronous mode means ADAPT-X paces the simulator, so a slow pipeline slows the
simulation. That is the correct trade for reproducibility, and it makes wall-clock throughput
a meaningless figure - which is why none is claimed.

**Status:** Accepted

## ADR-045: Ground Truth is a Separate Path and Never Enters Perception

**Decision:** The simulator's own knowledge of the scene is published as `GroundTruthFrame`
on a path parallel to the LiDAR frame, sharing its `frame_id` and timestamp so the two can be
joined later. It is **never** passed to detection, tracking, prediction, mapping, risk or
adaptive resolution. Its contracts are distinct from the perception models and carry no
confidence field.

**Reason:** Ground truth is the reason CARLA is worth having: it is the first thing in this
project that can say what was actually there, and therefore the precondition for measuring
accuracy at all (Phase 11). That value exists only while it stays out of the pipeline. A
detector that can see the answer measures nothing, and the resulting accuracy figure would
look exactly like a real one.

The risk is not that someone would do this deliberately; it is that it is *convenient*.
Correcting a track id from ground truth, or labelling a detection with the true class, each
look like a small improvement and each silently invalidate every measurement taken
afterwards.

Separate contracts rather than reused ones for the same reason. `DetectedObject` and
`TrackedObject` carry confidence, uncertainty and lifecycle - the apparatus of something
inferred. Ground truth is exact, so a confidence field on it would be meaningless, and its
presence would invite treating a known quantity as an estimated one.

**Alternatives considered:** Reusing `DetectedObject` for ground truth (makes known and
inferred indistinguishable at a glance); attaching ground truth to the frame (puts it one
attribute access away from every stage); a ground-truth-assisted mode for debugging (the mode
that eventually gets left on).

**Impact:** An integration test runs the chain twice - once reading ground truth every frame,
once never reading it - and asserts identical output. Another asserts structurally, in a
subprocess, that no perception module imports the CARLA package at all.

**Risks:** Ground truth in the ego frame depends on the same conversion as everything else, so
a conversion bug would move both together and the comparison would flatter the perception.
The conversion tests are the mitigation.

**Status:** Accepted

## ADR-046: Scenarios Are Generic Declarative Contracts, Resolved by an Explicit Seed

**Decision:** A scenario is **data**: a `ScenarioDefinition` holding actors, their ego-relative
placement, their scripted motion, the duration, the timestep and an explicit seed. The
definition models (`adaptx.scenarios.models`, `catalogue`) import nothing from the CARLA
boundary and nothing from the simulator package. Every randomised value is drawn from
`random.Random(seed)` - an explicit instance - exactly once, before any simulator is
touched, into a `ResolvedScenario` that is recorded on the result.

The seed is retained and reported even when a scenario has nothing to randomise. The only
randomised element Phase 10 defines is placement jitter; the catalogue uses none, so every
catalogue scenario is exact.

**Reason:** The handoff named the trap: the moment scenarios are Python functions, they are
reproducible from a git commit rather than from a description. A definition that survives a
JSON round-trip and rebuilds the same scene is the property that makes `random seed +
scenario configuration = reproducible experiment` (`12_scenario-generation.md`) actually true.

Keeping the definition layer free of CARLA is what makes the same definition usable by a
second simulator, and it is asserted by a source-level test rather than assumed - importing
a submodule runs the package `__init__`, so an import-based check would be vacuous.

Drawing from an explicit generator rather than the module-level one means nothing else in
the process can perturb a draw, and a resolution never disturbs anyone else's randomness.
Drawing once, up front, means a run's placements are fixed before the first frame and cannot
drift with the order in which frames happen to be processed.

**Alternatives considered:** Scenario classes with a `run()` method (reproducible from code,
not data); seeding the global `random` module (any import that consumes randomness would
silently change the draw); deriving randomness from CARLA's own RNG (ties reproducibility to
a server version); omitting the seed when nothing is randomised (a run would then not record
what it would have used, and adding jitter later would change every result's shape).

**Impact:** `ScenarioRunResult` embeds the full `ResolvedScenario`, so a run is reproducible
from its result without the catalogue. `CarlaSettings.seed` - declared in Phase 9 and consumed
by nothing - is now set from the scenario, so the boundary and the definition agree.

**Risks:** Ground-truth contracts still live in `adaptx.carla` although they are
simulator-generic; the runner and result layers import them from there. A second simulator
would be the trigger to move them, and doing it now would be Phase 9 refactoring for style.

**Status:** Accepted

## ADR-047: Scripted Motion as Timed Constant-Velocity Segments, Placed Not Simulated

**Decision:** An actor's motion is an ordered list of non-overlapping `MotionSegment`s, each
a constant ego-frame velocity over `[start_s, stop_s)`. An actor with no segments is
stationary. The position at scenario time *t* is the base placement plus the sum of every
segment's displacement up to *t* - a closed form. The runner **places** each actor at that
pose every frame with `place_ahead_of_ego`; it never applies a velocity and lets physics
integrate.

**Reason:** Three phases depend on frame timing being exact, and physics is not
frame-reproducible. Placing an actor at a computed pose makes frame *n* a function of the
definition, the seed and *n* alone, which is the determinism the phase exists to provide.

The closed form has a second consequence that matters more than it looks: the scenario can
state where every actor **should** be on every frame without a simulator running. That
expected pose is recorded beside what the simulator reported, and it is the third leg of the
comparison Phase 11 will make - commanded, reported, inferred. A physics-driven actor has no
"commanded pose" to record.

Segments rather than a single velocity because a timed start, a timed stop and a simple
multi-leg path are the cases the catalogue needs (a pedestrian who waits, crosses, and
stops), and a list of segments expresses all three with one representation.

**Alternatives considered:** A single constant velocity per actor (no timed start or stop);
waypoints with arrival times (more general, and harder to validate; segments cover the
catalogue and can be extended); CARLA autopilot or the Traffic Manager (non-deterministic
without seeding, and explicitly out of scope); applying velocity through physics (loses the
commanded pose and the frame-exact determinism).

**Impact:** Tests assert the arithmetic by hand - a timed start holds position until it
begins, a timed stop freezes it after, a diagonal moves on both axes, an L-shaped path sums
its legs - and that the simulator's reported ground truth matches the commanded pose to
within the fake's rounding.

**Risks:** Placed motion has no physics: no acceleration, no turning radius, no collision
response, and an actor placed into a wall will sit in the wall. Scenario authors carry that.
Ego motion is not supported - `EgoDefinition.stationary` is validated `True` so the
limitation is visible in every definition rather than silently assumed.

**Status:** Accepted

## ADR-048: The Runner Drives a Protocol Extracted From the Boundary, and Runs Once

**Decision:** `ScenarioRunner` drives the simulator through `ScenarioSimulator`, a structural
protocol of exactly the methods `CarlaSimulationSession` already exposes - `open`, `close`,
`step`, `ground_truth`, `status`, `spawn_ahead_of_ego`, `place_ahead_of_ego`. No adapter code
changed to satisfy it. A runner is single-use: it holds the lifecycle of one run
(`CREATED → VALIDATING → READY → RUNNING → COMPLETED | FAILED`), and a second scenario gets a
second runner.

Two failure modes are kept distinct. A **malformed definition** raises `ScenarioError` before
any simulator is contacted. A **run-time failure** - a refused spawn, a sensor timeout, an
exception inside processing - returns a `FAILED` result with the reason, after `close()` has
run in a `finally`.

**Reason:** Extracting the protocol from the boundary rather than imposing one on it kept
Phase 9 untouched, which the handoff required, and it is what lets the test suite drive the
runner against a stand-in - and what would let a second simulator plug in without touching
the scenario layer.

Single-use runners are the isolation guarantee. Actor handles, spawn records and lifecycle
state live on the instance; a fresh instance per run means scenario B cannot inherit scenario
A's actors, and a test asserts exactly that with two runs against one world.

The two failure modes are different in kind. A definition that fails validation is a
programming error and should be loud, immediately, and never reach a simulator. A run that
fails part-way is an event worth recording - which frames were stepped, what the error was -
so a batch of runs survives one bad one and the evidence is kept. The first version of the
runner conflated them and crashed inside its own failure path; the distinction was drawn from
that bug.

**Alternatives considered:** Making the runner hold a session and re-run definitions (state
leaks between runs; the isolation test would have been the first thing to fail); returning
`FAILED` for a malformed definition (swallows a programming error into a result that looks
like data); raising for run-time failures (loses the frames stepped so far and stops a batch).

**Impact:** The processor callback receives the sensor frame and nothing else, by signature.
Ground truth is fetched by the runner, recorded on the result, and passed to no stage; an
integration test runs the chain by hand without ever calling `ground_truth()` and asserts
identical stage counts. `carla/smoke.py` is deleted; its constants became the
`vehicle_approach` definition and its seven tests were retargeted onto the framework.

**Risks:** The protocol pins the boundary's method names. Renaming one is now a two-place
change, which is the point.

**Status:** Accepted

## ADR-049: The Ego's Spawn Transform Is the Placement Reference Until the First Tick

**Decision:** `CarlaSimulationSession` keeps the transform the ego was spawned with and uses
it - not `actor.get_transform()` - as the reference frame for every ego-relative placement
(`spawn_ahead_of_ego`, `place_ahead_of_ego`), for `ego_state()` and for ground truth, until
the session has stepped its first frame. From the first tick on it uses the simulator's
reported transform. The spawn points are walked in order and the first that accepts the ego
is used; its index is recorded as `ego_spawn_index` on the session status.

**Reason:** Found by the first live run (Experiment 010). In synchronous mode, CARLA 0.9.16
does not report a freshly spawned actor's pose until the server has ticked: `get_transform()`
answers the world origin with zero yaw. The session read it immediately after spawning, so
"45 m ahead and 3.5 m left" was computed from `(0, 0, 0)` and landed off-road, 70 m from the
ego, and the target spawn was refused. The stand-in never caught this because its ego *is*
at the origin.

Before the first tick, the spawn transform is the only truthful statement of where the ego
is. After it, the simulator's answer is - and the two agree to a centimetre of settling
because the ego is never driven (ADR-047). Switching to the live pose after the first tick
rather than using the spawn transform forever keeps the door open for a moving ego without
another change here.

Walking the spawn points is the same kind of finding: index 0 refused every spawn while
1-11 accepted. Taking the first accepting index is deterministic for a given map and world
state, and recording it keeps a run reproducible from what it reports.

**Corrected in Phase 11 (Experiment 011):** the map was not refusing index 0. Two actors
left there by a killed process - an ADAPT-X ego and its LiDAR - were occupying it, and
destroying them made it accept. The walk stays, because a point occupied by something a
previous process left behind is precisely the case it handles. Scenario placements are
ego-relative, so a different spawn point is a different scene and on this map an off-road
one (from index 0 the approach scenario's target is 3.7 m off the road; from index 1 it is
on a Driving lane). `CarlaSettings.ego_spawn_index` pins the ego to one point; when set,
no other point is tried, and a refusal is an error rather than a different scene.

**Alternatives considered:** Ticking once inside `open()` (would enqueue a sensor frame
before any `step()`, and the frame-matching in `step()` would then have to discard it -
more moving parts to be wrong); always using the spawn transform (correct today, wrong the
day the ego moves); leaving it and telling scenarios to step once before spawning (pushes a
simulator quirk into every scenario author's head).

**Impact:** `SimulationSessionStatus` gains `ego_spawn_index` and `server_version` (both
additive, null when unknown). The fake simulator now reports the origin until ticked, like
the server, so the regression cannot pass against the stand-in and fail live again. Three
placement tests read a pose only after stepping.

**Risks:** A server that *does* report the pose before the first tick is handled identically -
the spawn transform and the reported one agree. A scenario that moves the ego before the
first tick would see the spawn transform; no such scenario exists (ADR-047, stationary ego).

**Status:** Accepted

## ADR-050: Evaluation Is Offline, From the Pipeline Outputs Recorded on the Run

**Decision:** Phase 11 evaluates a recorded `ScenarioRunResult` and nothing else. To make
that possible the Phase 10 record gains two optional, additive fields: `frames[].outputs`,
holding the pipeline's own result contracts for the frame (`DetectionResult`,
`TrackingResult`, `PredictionResult`, `RiskAssessmentResult`, the two map summaries, the
`ResolutionPlan` and the frame's `MappingComparison`), and `sensor`, the LiDAR configuration
the run used including the mount offset. `pipeline_processor` records outputs by default;
`record_outputs=False` gives the Phase 10 counts-only record. The evaluator never contacts a
simulator and never re-runs perception. `python -m adaptx.evaluation evaluate run.json`
works with CARLA closed and the `carla` package absent, and a test asserts it.

**Reason:** Counts cannot support a position error, an ADE or a churn figure, and the
alternatives were worse. Re-running perception inside the evaluator would make the
evaluation depend on the pipeline's current code rather than on what actually ran, and
would need the point cloud on disk (27,000 points a frame). A replay engine was deferred
from Phase 10 with its design open and is not needed for this. Keeping the result objects
the pipeline already built costs no computation; a record grows to 30-80 MB per catalogue
scenario, dominated by 144 tile decisions a frame, which is acceptable for a research
artefact and was not optimised before being measured.

The record stays evidence in the Phase 10 sense: no accuracy, error or match figure is on
it, the Phase 10 test that asserts so still passes, and every earlier consumer is
unaffected because both fields default to `None`.

**Alternatives considered:** Re-running the pipeline offline (evaluates the code of the
day, not the run); storing point clouds (large, and still needs a re-run); a replay
subsystem (deferred design, out of scope); a separate evidence file beside the record (two
files to keep aligned, for no gain).

**Impact:** `FrameProcessor` may return either `StageCounts` or `PipelineFrameOutputs`;
the runner records both counts and outputs. `docs/EVALUATION.md` §1.

**Risks:** Record size; a consumer that loads a record with outputs into memory needs a
few hundred MB for the longest scenario. Nothing streams them.

**Status:** Accepted

## ADR-051: A Metric That Could Not Be Computed Is Absent With a Reason, Never Zero

**Decision:** Every section of an `EvaluationReport` carries a `MetricStatus` - `measured`,
`partial`, `unavailable`, `not_applicable` - and a reason when it is not `measured`. Every
summary is a `Distribution` whose statistics are `None` when it has no samples, and whose
`p95` is `None` below 20 samples with the rule stated. Nothing is defaulted to 0, 0.0,
`LOW` or an empty string that reads as a measurement. An `UNKNOWN` risk assessment
(`risk_score = None`) is counted as unknown and takes part in no rate. Mapping accuracy is
`unavailable` with its reason on every report. Peak memory is `unavailable` with its reason.

**Reason:** The same rule Phase 7 set for risk (ADR-035) and the project's constraints set
for every metric: an absent measurement that looks like one is worse than none. A
prediction section reading `ADE: 0.0` because no trajectory could be aligned would be a
perfect score for a predictor that predicted nothing. A p95 of twelve samples is the maximum
with a different name. `UNKNOWN` read as `LOW` would make the risk engine's most honest
output its worst-scored one.

**Alternatives considered:** Sentinel zeros with a flag (the zero still gets averaged);
omitting the section (the reader cannot tell "not applicable" from "forgot"); NaN (does not
survive JSON and does not carry a reason).

**Impact:** Report consumers must handle `None`; the text renderer prints `n/a` and the
reason. A test asserts the text report prints reasons, not zeros, for a run with nothing
to measure.

**Risks:** More fields to read. That is the point.

**Status:** Accepted

## ADR-052: Greedy Gated Matching, Reported at Several Gates; Reference Velocity by Finite Difference

**Decision:** Tracks (and detections) are matched to ground-truth actors per frame by
greedy nearest neighbour on planar distance within a gate, pairs sorted by (distance,
candidate id, actor id) so the result is deterministic. Every actor the record holds takes
part in matching; only actors within the sensor range and the map bounds enter a rate. A
track with no match is *unlabelled*, and no precision, false-positive count, MOTA or MOTP is
reported. Matching is evaluated at every gate in `match_gates_m` (baseline 1, 2, 4 m) and
the report carries all of them; continuity, prediction and risk use the primary gate. Class
agreement is reported, never required. The velocity reference is the finite difference of
consecutive ground-truth positions; the simulator's reported velocity is not used.

**Reason:** The catalogue never has more than two scenario actors in a frame, so greedy
and optimal assignment coincide unless two actors sit within one gate of one track, and the
Hungarian method would need a dependency the project has avoided for eleven phases or a
hand-written one for a case that does not arise. Deterministic tie-breaking matters more
than optimality here: two evaluations of one record must agree.

Precision is not reported because the map's buildings, poles and parked meshes return
LiDAR points and are not CARLA actors: most tracks in every live run are on unlabelled
geometry, and calling them false positives would fabricate the denominator of every
precision figure. MOTA needs that term and so is not MOTA without it.

Several gates because the association rule is a choice, and a single headline recall
depends on it entirely (`docs/NEXT_PHASE.md` said so before the phase began). The live
results bear it out: vehicle recall is 0.00-0.06 at 1 m and 0.23-0.61 at 2 m.

The simulator's velocity is not used because a placed actor (ADR-047) keeps its physics
velocity between placements: the record shows ~0 m/s in the plane for an actor scripted at
8 m/s and a growing vertical velocity as it falls. The finite difference of positions is
exact for a scripted constant-velocity segment.

**Alternatives considered:** Hungarian assignment (no need, no dependency); requiring class
agreement for a match (the classifier is under evaluation; it never labelled the Audi a
vehicle, and requiring it would have zeroed every vehicle metric); one gate (a single choice
hidden as a result); using the recorded velocity (wrong by construction).

**Impact:** `docs/EVALUATION.md` §2-3. If a future scenario has dense actors, revisit
assignment.

**Risks:** Greedy matching can be suboptimal with three or more actors within one gate.
Recorded, not currently reachable.

**Status:** Accepted

## ADR-053: Fixed Versus Adaptive Is Paired Within One Run, and Every Figure Is Simulation Evidence

**Decision:** The fixed-resolution and adaptive maps are compared **within one run**: the
pipeline builds both from the same processed frame, the record carries both summaries and
their `MappingComparison`, and every ratio (cells, bytes, build time) is per frame, paired
by construction. No second run with a different policy is made. The adaptive policy is
evaluated against its own stated intent - detail at the tile holding each ground-truth
actor versus the rest, detail grouped by matched risk level, refinement lead before the
actor's arrival, churn, reversals and reported holds - and none of those metrics assumes the
adaptive map is better; a ratio above one and a negative lead are reported as measured.

Every `EvaluationReport` carries `source = SIMULATION` and a fixed, non-configurable list of
limitations: simulation evidence only; not safety validation; not collision-probability
validation; not real-world validation; unlabelled tracks are not false positives; mapping
accuracy not evaluated; timings are not real-time claims. The text renderer prints them
after the conclusion, and the conclusion itself states measured findings only - no verdict,
because no threshold for one has been justified.

**Reason:** A paired comparison removes every source of variance except the policy: same
sensor frame, same detections, same tracks, same risk, same bounds, same machine, same
moment. Experiment 011 showed live runs are not bit-repeatable even seeded, so two separate
runs would have compared two slightly different scenes. The pipeline already computed both
maps for every frame; pairing them cost nothing.

The limitations travel with the numbers because a report is copied more often than the
document that qualifies it. `CLAUDE.md` forbids fabricated benchmark results and production
claims; the risk engine's own contract says its score is not a probability (ADR-032); the
catalogue contains no collision; ground truth is what one simulator knew about one map. A
reader who stops at the conclusion must still see this.

**Alternatives considered:** Two runs, one per policy (adds sensor variance; the fixed
map is already built every frame); a verdict line (would need thresholds the project has no
evidence to set); configurable limitations (would let them be switched off).

**Impact:** `MappingComparison` from Phase 8 is the paired contract, recorded per frame.
Experiment 011 records the first paired figures on live scenes: 0.48-0.56 of the fixed
cells, 5x the build time, finer cells under the actor where the actor was perceived.

**Risks:** Pairing within one run cannot measure whether a *different* upstream (a policy
that changed what was detected) would have changed the outcome. That is not what Phase 8
does; if it ever does, this decision must be revisited.

**Status:** Accepted

## ADR-054: Placed Actors Do Not Simulate Physics and Stand on the Ego's Ground Plane

**Decision:** Every actor the simulation session spawns for a scenario has its physics
switched off the moment it exists, and is placed so that the **bottom of its bounding box**
sits `up_m` above the ego's ground plane; `Placement.up_m` defaults to 0.0 and means "on
the road". The ego has its physics switched off too and is set down at the road surface
under its spawn point rather than left at the spawn point's 0.6 m clearance. The session
records each actor's origin-to-box-bottom offset at spawn so `place_ahead_of_ego` keeps the
same meaning of `up_m` on every frame.

**Reason:** ADR-047 said actors are placed, not simulated, and the implementation only half
meant it: it set the transform every tick and left physics on. Experiment 011 recorded the
consequences. A placed walker kept its physics velocity between placements and fell through
the road on 45 of 120 frames (vertical speed -31 m/s), so the pedestrian was invisible to
the LiDAR and "one detection in 120 frames" said nothing about the detector. Vehicles fell
half a metre every tick and were caught by the road, so their reported height jittered and
their returns changed frame to frame. The ego, spawned 0.6 m above the road for clearance,
settled over the first half second of every run, and every ego-relative placement and the
LiDAR settled with it. None of this was the scene the scenario described.

With physics off nothing moves unless placed. Experiment 012 measured the result: the
walker at 0.93 m and the vehicles at 0.00 m on every frame, the ego at 0.000 world z on
every frame, and the stationary scenario bit-repeatable across two runs, which no live run
had been before.

`up_m` is measured to the box bottom rather than the origin because CARLA puts a vehicle's
origin at its wheels and a walker's at the middle of its capsule; "0.5 m above the ego
origin" put a car half a metre in the air and a walker's middle at knee height.

**Alternatives considered:** Zeroing velocity each tick with physics on (works for the
vehicle in a probe, but the walker's behaviour differed between probe and run and physics
would stay a source of variance); ticking the world until the ego settles before READY
(changes the session's frame accounting and leaves the placed actors' physics on); disabling
physics on scenario actors only and leaving the ego to settle (keeps the 0.6 m transient
in every run).

**Impact:** Phase 10 `Placement.up_m` default and meaning changed; no catalogue scenario set
it explicitly, so every scenario now stands its actors on the road. Expected poses carry
`up_m` as before. The fake simulator gained `set_simulate_physics`, a road-surface waypoint
and a box location so the behaviour is tested without a server. Experiment 011's pedestrian
figures are void; Experiment 012 re-measures every scenario.

**Risks:** A physics-less vehicle does not react to anything; a future scenario that wants
a vehicle to be pushed, to brake or to drive must switch physics back on for that actor and
own the consequences. The residual run-to-run difference on scenes with moving placed actors
(at most 8 of 27,000 points on some frames) is not removed by this decision and is not
explained by it.

**Status:** Accepted

## ADR-055: The Dashboard Is a Consumer - Scene Snapshots, Stored Evidence and Canvas Rendering

**Decision:** The Phase 12 dashboard is a static browser application (`dashboard/`, vanilla
ES modules, no build step, no framework, no new dependency) served by the existing FastAPI
process at `/dashboard`. It **displays and never computes**: every number it shows is a
field of a backend contract, and the only arithmetic in the frontend is pixel layout inside
`dashboard/render/`. It has two labelled modes that are never mixed:

- **LIVE.** The pipeline's last output is bundled into a `SceneSnapshot`
  (`models/scene.py`): tracks, trajectories, assessments, tile decisions, budget, map
  summaries, stage timings and a **deterministic stride sample** of the point cloud (at most
  `dashboard.scene_max_points`, default 6000, rounded to 2 dp, labelled with its stage and
  `is_downsampled`). The snapshot has no field for ground truth and refuses one. It is
  published by the full-chain endpoint (`POST /api/v1/lidar/adaptive-map`) and by the
  scenario CLI with `--publish URL`, which hands each frame's outputs to
  `POST /api/v1/scene/frame` after the scenario's own pipeline produced them. The backend
  stores and broadcasts the snapshot; it runs no stage on it. `/ws/scene` pushes the latest
  snapshot whenever a sequence counter changes, polled every 40 ms.
- **STORED EVALUATION.** Phase 11 `EvaluationReport`s and Phase 10 `ScenarioRunResult`s are
  read from two operator-configured directories (`reports/`, `runs/`) by an
  `EvidenceService` in its own package `adaptx.evidence`, downstream of the evaluation
  layer and outside every pipeline package. Reports are served whole; runs are served as a
  summary plus one frame at a time, so a browser never parses a 40 MB record. Comparison
  goes through `compare_reports` unchanged. Ground truth is served **only** on the run-frame
  path, and shown only in the Run / Scenario view, labelled.

Rendering is Canvas 2D: a perspective camera behind and above the ego and a top-down plan
with +X up and +Y left (ADR-009). No DOM element is created per point, cell or tile.
`data/format.js` is the single place where `null` becomes "Not available" and `UNKNOWN`
stays "UNKNOWN"; loaded reports and runs are deep-frozen.

**Reason:** The point of Phase 11 was that the only accuracy figures in the project are the
ones the evaluation layer computed from a recorded run under a stated configuration. A
dashboard that recomputed a distance, a rate or a level in the browser would create a second
source of truth that no test covers and no report records. Making the frontend a pure
consumer - and testing that statically, by scanning its source for `Math.hypot`,
`.reduce(`, `predict(` and assignments to metric names - keeps one source. The same reason
puts the evidence reader beside the evaluation layer rather than in `services/`: the Phase 11
boundary test that no pipeline package imports evaluation stays true, and the API composition
root reaches evaluation through exactly one module, `api/routes/evidence.py`, which the test
now pins.

A snapshot without a ground-truth field, published by producers that never held ground
truth, is the mechanical guarantee that the live view cannot show what the perception stack
did not perceive. Serving runs frame by frame is what makes a 42 MB record usable in a
browser at all. Vanilla modules keep the surface auditable and the dependency count at zero;
nothing in the views needs component state that a framework would manage better.

**Alternatives considered:** A React/Vite frontend (a build step and hundreds of transitive
packages for eight views of key-value cards and two canvases); WebGL point rendering (6000
points draw in 6-10 ms on Canvas 2D, measured in Experiment 013, so the extra surface is not
earned); pushing the snapshot from the producer instead of polling a counter (the producers
are synchronous request handlers; a single integer is the whole coupling); serving the entire
run record to the browser (40 MB parses in seconds and blocks the tab); putting
`EvidenceService` in `services/` beside the others (breaks the Phase 11 import boundary);
recomputing object distances in the browser from positions (rejected on the consumer rule -
the risk assessment's own `distance_m` is shown instead); adding CARLA control endpoints
(spawn, teleport, start, stop) so the dashboard could drive scenarios (rejected: the dashboard
would become a control surface for a simulator and the scenario CLI already exists).

**Impact:** New: `models/scene.py`, `services/scene_service.py`, `adaptx.evidence`,
`api/routes/scene.py`, `api/routes/evidence.py`, `api/websocket/scene.py`,
`scenarios/publish.py`, `DashboardSettings`, `StoredEvidenceError` (404), the `dashboard`
component in system status, a `FrameObserver` hook on `ScenarioRunner`, and the `dashboard/`
application with Node `node --test` unit tests. Additive only: no existing endpoint or
contract changed. `POST /api/v1/lidar/adaptive-map` now also publishes a snapshot as a side
effect. The evaluation component's status text changed from "not served over HTTP" to
"computed only offline; stored reports served read-only".

**Risks:** The scene channel shows the *last* frame, not a history; a dashboard opened after
a scenario finished sees one stale frame, labelled with its time. The point sample is a
stride, so thin objects can lose returns in the picture (the pipeline saw them all). The
backend handlers are synchronous, so eight prefetched frames head-of-line block a random seek
by about 200 ms (Experiment 013). The frontend boundary test is a source scan: it can be
fooled by renaming, and it is not a proof.

**Status:** Accepted

## ADR-056: A Long-Lived Live Session Drives the Ego From the Pipeline's Own Outputs

**Decision:** A post-Phase-12 extension adds a **live simulation loop** behind the
dashboard, as three new packages and one extended boundary, without changing any Phase
1-12 algorithm:

- `adaptx.live` - `LiveSimulationService` owns one long-lived `CarlaSimulationSession` on
  its own thread and is the **only caller of `world.tick()`** while it runs. Per frame it
  advances the scenario, steps the session, runs the frame through the **existing**
  `pipeline_processor` (the same Phase 2-8 chain the endpoints and the scenario runner
  use, unchanged), asks the controller for a command, applies it, publishes a
  `SceneSnapshot` and derives events from frame-to-frame differences in the outputs. A
  live scenario catalogue (`live/scenarios.py`) spawns seeded Traffic Manager traffic and
  timed scripted actors **anchored to the ego's pose at the moment they appear**, moved in
  that anchor frame by the Phase 10 constant-velocity segments and removed after a
  lifetime.
- `adaptx.control` - `RiskGovernedSpeedPolicy`, a pure function of the risk assessments,
  tracks, predicted paths and the ego's own odometry: a target speed per risk level among
  **in-path** objects (ahead, within a corridor, now or by prediction), a hold inside a safe
  distance, full brake inside an emergency distance, a resume dwell, a setpoint
  rate-limited by the acceleration limit on the way up and immediate on the way down,
  and steering that follows the map's lane centre. A `VehicleController` protocol is the
  whole coupling to a vehicle.
- `adaptx.carla.session` (extended) - a `drive_ego` mode that leaves the ego's physics on,
  `apply_ego_control` (the one place a command meets `carla.VehicleControl`, with the
  steer sign flipped there and nowhere else), anchored placement, Traffic Manager
  traffic, a collision sensor as a safety fallback, a display-only RGB camera, and a
  shutdown order that survives the Traffic Manager.

Five high-level session controls exist over HTTP - start, pause, resume, stop, reset -
allowlisted by full path in the route audit; no endpoint spawns, destroys, moves, ticks or
drives anything. The dashboard gained an explicit LIVE SIMULATION / STORED EVALUATION
mode, a Front View fed by the camera, and cards for control, events and measured timing.

**Reason:** Phase 12 could show a stored run or a scenario that was already publishing;
it could not show ADAPT-X *doing* anything. The smallest loop that closes -
perceive, decide, actuate, observe - needed a driven ego and a decision, and the decision
had to consume the pipeline's outputs and nothing else, or the demonstration would prove
nothing about the pipeline. Three findings from the live server shaped the design and are
recorded rather than smoothed over: the Phase 7 engine scores a lamp post 5 m beside the
road HIGH or CRITICAL on proximity alone, so a governor keyed on the scene maximum pinned
the ego at the kerb forever - the speed table is therefore keyed on in-path objects, and
the scene level is shown beside it; a pure proportional throttle stalled 0.4 m/s under
its target because a small error gives a throttle below rolling resistance, so a hold
throttle was added; and the Phase 4 tracker's identity churn on a parked car swung the
level HIGH->LOW->HIGH within frames, so the setpoint ramps up under the acceleration
limit and events for CRUISING/SLOWING flips are coalesced.

Two things the loop never reads: ground truth (the loop has no call to
`session.ground_truth()` and the snapshot has no field for it, ADR-045) and the collision
sensor as an input (it ends the session and records the contact; it steers nothing).

**Alternatives considered:** Driving the ego through the Phase 10 runner (single-use by
design, ADR-048, and the evaluation architecture depends on it staying so); a second,
lighter perception chain for the loop (rejected outright: the dashboard must show the
pipeline, not a proxy); a controller reading CARLA ground truth for other actors "just for
the demo" (would make the demonstration a lie about the pipeline); scene-maximum risk
governing speed (measured to pin the ego, above); a planned route or lane-change logic
(routing and planning stay NOT IMPLEMENTED; the road geometry follow is the minimum to
keep a driven ego on the road); an asynchronous world (breaks ADR-044 determinism and the
frame-interval velocity measurement); raw actor endpoints for the dashboard (rejected in
ADR-055 and still rejected).

**Impact:** New packages `adaptx.control`, `adaptx.live`; `CarlaSimulationSession` extended
additively; `SceneSnapshot` gained optional `ego`, `control`, `live` fields and a
`live:<id>` origin; `ControlSettings`, `LiveSettings`; `GET /api/v1/live/*` reads, the five
controls, `/api/v1/live/camera`; `CarlaStatus.ERROR`; `live_simulation` and
`vehicle_control` components; the scene channel reports `skipped`; the Phase 11
ground-truth boundary tests cover the two new packages; the route audit allowlists the
five controls. The fake simulator gained kinematics, attached sensors, a Traffic Manager,
collision and camera sensors so the loop is tested without a server. Measured live in
Experiment 014.

**Risks:** The loop runs at ~0.3x wall-clock speed on this machine (the pipeline is
115-130 ms per 50 ms frame) and says LAGGING; a viewer sees slow motion, not a slow
vehicle. Velocities are ego-relative with no ego-motion compensation, so a moving ego
makes every static object "approach" at its own speed and the risk engine's closing-speed
factor rises everywhere; the corridor rule contains but does not remove this. The
corridor is straight along +X: on a bend, roadside geometry enters it and the ego holds.
A killed backend leaves actors on the server and, if the Traffic Manager was in
synchronous mode, a server that appears hung; `stop` cleans up, a crash does not. The
controller is a baseline: it is not tuned, not validated and not collision-free by claim -
the safety sensor counts what it fails to prevent.

**Status:** Accepted

## Decision Template

### ADR-XXX: Title

**Decision:**

**Reason:**

**Alternatives considered:**

**Impact:**

**Risks:**

**Status:** Proposed / Accepted / Superseded
