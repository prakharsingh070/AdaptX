"""Phase 9 smoke run: CARLA through the whole ADAPT-X pipeline.

``python -m adaptx.carla.smoke``

Proves one thing, and deliberately only one thing: **a CARLA frame can enter
the existing pipeline and come out the other end as an adaptive map**, with no
stage below the boundary knowing where the points came from.

The scenario
------------
A stationary ego with a roof LiDAR, and one vehicle approaching head-on along
a scripted straight line. That is the minimum that exercises what matters:

* a moving object gives Phase 4 a **measured** velocity (which needs two
  frames, so the run is never shorter than two);
* a measured velocity gives Phase 5 a trajectory;
* an approaching object gives Phase 7 a rate of approach to score;
* a rising risk gives Phase 8 a reason to refine a region.

Motion is scripted by setting the transform each tick rather than driven by
physics or autopilot, so two runs of the same configuration produce the same
frames (ADR-044).

**This is not the Phase 10 scenario framework.** It is one hard-coded scene
that exists to prove the integration, and it should be replaced by the real
thing rather than grown into it.

What this is not
----------------
Not an accuracy measurement. It shows the pipeline *runs* on simulated data;
it says nothing about whether the detections are right. Comparing perception
against the ground truth this module records is Phase 11's job.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import time
from dataclasses import dataclass, field
from itertools import pairwise
from typing import Any

from adaptx.carla.ground_truth import GroundTruthFrame
from adaptx.carla.session import CarlaSimulationSession
from adaptx.config.settings import Settings, get_settings
from adaptx.core.exceptions import SimulatorUnavailableError
from adaptx.core.lifecycle import ApplicationContext, build_context
from adaptx.core.logging import configure_logging, get_logger
from adaptx.models.common import DataSource

logger = get_logger(__name__)

#: Where the approaching vehicle starts, in metres ahead of the ego.
APPROACH_START_M = 45.0
#: How far it closes each simulation second. At the default 0.05 s timestep
#: that is 0.4 m per tick - enough displacement for tracking to measure.
APPROACH_SPEED_MPS = 8.0
#: Lateral offset, so the target is in an adjacent lane rather than on top of
#: the ego reference.
APPROACH_LEFT_M = 3.5


@dataclass(slots=True)
class FrameRecord:
    """What one frame produced, for the run summary.

    Measured or counted only - nothing here is estimated.
    """

    frame_id: int
    simulation_time_s: float
    point_count: int
    processed_points: int
    detections: int
    tracks: int
    trajectories: int
    risk_level: str
    regions_by_level: dict[str, int]
    adaptive_cells: int
    fixed_cells: int
    ground_truth_actors: int
    nearest_ground_truth_m: float | None
    stage_ms: dict[str, float] = field(default_factory=dict)

    @property
    def total_ms(self) -> float:
        """Measured pipeline time for this frame, excluding simulator time."""
        return sum(self.stage_ms.values())


@dataclass(slots=True)
class SmokeResult:
    """Everything one smoke run produced."""

    frames: list[FrameRecord]
    map_name: str
    fixed_delta_seconds: float
    ground_truth: list[GroundTruthFrame]
    carla_version: str

    @property
    def frame_count(self) -> int:
        """Frames stepped."""
        return len(self.frames)

    def timestamps_are_monotonic(self) -> bool:
        """Whether simulation time strictly increased across the run."""
        times = [record.simulation_time_s for record in self.frames]
        return all(later > earlier for earlier, later in pairwise(times))


def run_smoke(
    settings: Settings | None = None,
    *,
    context: ApplicationContext | None = None,
    session: CarlaSimulationSession | None = None,
    frames: int | None = None,
) -> SmokeResult:
    """Run the scenario and push every frame through the whole pipeline.

    Args:
        settings: Configuration to use. Defaults to the process settings.
        context: An application context. Built from ``settings`` when omitted.
        session: A simulation session. Constructed from ``settings`` when
            omitted; injectable so the run can be exercised against a
            stand-in simulator.
        frames: Frames to step. Defaults to ``carla.smoke_frames``.

    Returns:
        A record of every frame, plus the ground truth captured beside it.

    Raises:
        SimulatorUnavailableError: CARLA is disabled or unreachable, or the
            scenario could not be set up.
    """
    resolved = settings if settings is not None else get_settings()
    app = context if context is not None else build_context(resolved)
    sim = session if session is not None else CarlaSimulationSession(resolved.carla)
    total_frames = frames if frames is not None else resolved.carla.smoke_frames

    records: list[FrameRecord] = []
    truths: list[GroundTruthFrame] = []

    sim.open()
    try:
        target = sim.spawn_ahead_of_ego(
            resolved.carla.target_blueprint,
            forward_m=APPROACH_START_M,
            left_m=APPROACH_LEFT_M,
        )

        for index in range(total_frames):
            elapsed = index * resolved.carla.fixed_delta_seconds
            distance = APPROACH_START_M - APPROACH_SPEED_MPS * elapsed
            sim.place_ahead_of_ego(target, forward_m=distance, left_m=APPROACH_LEFT_M)

            frame = sim.step()
            truth = sim.ground_truth()
            records.append(_process(app, frame, truth))
            truths.append(truth)
    finally:
        sim.close()

    status = sim.status()
    return SmokeResult(
        frames=records,
        map_name=status.map_name or "unknown",
        fixed_delta_seconds=resolved.carla.fixed_delta_seconds,
        ground_truth=truths,
        carla_version=_carla_version(),
    )


def _process(context: ApplicationContext, frame: Any, truth: GroundTruthFrame) -> FrameRecord:
    """Push one simulated frame through the existing pipeline, unmodified.

    Every call below is the same call the LiDAR endpoints make. Nothing here
    branches on the frame having come from a simulator, which is exactly the
    property Phase 9 exists to establish.

    ``truth`` is recorded alongside and **never passed to any stage**.
    """
    if frame.source is not DataSource.SIMULATION:
        raise SimulatorUnavailableError(
            f"a simulated frame must be labelled 'simulation', got '{frame.source.value}'",
            details={"source": frame.source.value},
        )

    processed = context.preprocessor.run(frame)
    detection = context.detector.detect(processed.frame)
    tracking = context.tracking.update(
        detection.objects,
        processed.frame.timestamp,
        frame_id=processed.frame.frame_id,
        sensor_id=processed.frame.sensor_id,
    )
    prediction = context.prediction.predict_from_tracking(tracking)
    spatial_map = context.mapping.build(processed.frame)
    risk = context.risk.assess_from_pipeline(
        tracking, prediction=prediction, spatial_map=spatial_map
    )
    adaptive = context.adaptive_mapping.run_from_pipeline(
        processed.frame, risk, tracking, trajectories=prediction.trajectories
    )

    nearest = truth.nearest()
    return FrameRecord(
        frame_id=frame.frame_id,
        simulation_time_s=frame.timestamp.timestamp(),
        point_count=frame.point_count,
        processed_points=processed.frame.point_count,
        detections=len(detection.objects),
        tracks=len(tracking.tracks),
        trajectories=len(prediction.trajectories),
        risk_level=risk.highest_risk_level.value,
        regions_by_level=adaptive.tiles_by_level(),
        adaptive_cells=adaptive.accounting.total_cell_count,
        fixed_cells=spatial_map.accounting.total_cell_count,
        ground_truth_actors=len(truth.others()),
        nearest_ground_truth_m=None if nearest is None else nearest.distance_m,
        stage_ms={
            "processing": processed.metrics.duration_ms,
            "detection": detection.duration_ms,
            "tracking": tracking.duration_ms,
            "prediction": prediction.duration_ms,
            "mapping": spatial_map.duration_ms,
            "risk": risk.duration_ms,
            "controller": adaptive.plan.duration_ms,
            "adaptive_mapping": adaptive.duration_ms,
        },
    )


def _carla_version() -> str:
    """The installed CARLA version, or a plain statement that it is absent."""
    try:
        import carla
    except ImportError:
        return "not installed"
    return str(getattr(carla, "__version__", "unknown"))


def render(result: SmokeResult) -> str:
    """A readable summary of a smoke run."""
    lines = [
        "ADAPT-X Phase 9 CARLA smoke run",
        "=" * 104,
        f"map: {result.map_name} | fixed timestep: {result.fixed_delta_seconds} s "
        f"| carla: {result.carla_version}",
        f"frames: {result.frame_count} | simulation time strictly increasing: "
        f"{result.timestamps_are_monotonic()}",
        "",
        "LIVE CARLA RUN. Durations are measured on this machine for this scenario.",
        "This proves the pipeline CONSUMES simulated LiDAR. It measures no accuracy:",
        "comparing perception against the recorded ground truth is Phase 11.",
        "",
        f"{'frame':>8}{'sim s':>9}{'points':>9}{'kept':>8}{'det':>5}{'trk':>5}{'traj':>6}"
        f"{'risk':>10}{'regions L/M/H/C':>18}{'cells':>9}{'fixed':>9}{'gt':>4}{'gt m':>8}{'ms':>8}",
        "-" * 104,
    ]
    for record in result.frames:
        levels = record.regions_by_level
        spread = f"{levels['low']}/{levels['medium']}/{levels['high']}/{levels['critical']}"
        nearest = (
            "n/a"
            if record.nearest_ground_truth_m is None
            else f"{record.nearest_ground_truth_m:.1f}"
        )
        lines.append(
            f"{record.frame_id:>8}{record.simulation_time_s:>9.2f}{record.point_count:>9}"
            f"{record.processed_points:>8}{record.detections:>5}{record.tracks:>5}"
            f"{record.trajectories:>6}{record.risk_level:>10}{spread:>18}"
            f"{record.adaptive_cells:>9}{record.fixed_cells:>9}"
            f"{record.ground_truth_actors:>4}{nearest:>8}{record.total_ms:>8.1f}"
        )

    lines += [
        "",
        "Ground truth was recorded beside every frame and fed to no stage of the",
        "pipeline. Detection, tracking, prediction, risk and adaptive resolution",
        "saw only the LiDAR points.",
    ]
    return "\n".join(lines)


def main() -> int:
    """Entry point. Returns a process exit code."""
    parser = argparse.ArgumentParser(
        prog="python -m adaptx.carla.smoke",
        description=(
            "Run one deterministic CARLA scenario through the whole ADAPT-X "
            "pipeline. Requires a running CARLA server."
        ),
    )
    parser.add_argument("--frames", type=int, default=None, help="Frames to step.")
    parser.add_argument("--host", type=str, default=None, help="CARLA host.")
    parser.add_argument("--port", type=int, default=None, help="CARLA port.")
    parser.add_argument("--town", type=str, default=None, help="Map to load, e.g. Town03.")
    parser.add_argument("--json", type=pathlib.Path, help="Write the full record here.")
    args = parser.parse_args()

    configure_logging("INFO")
    settings = get_settings().model_copy(deep=True)
    settings.carla.enabled = True
    if args.host:
        settings.carla.host = args.host
    if args.port:
        settings.carla.port = args.port
    if args.town:
        settings.carla.town = args.town

    started = time.perf_counter()
    try:
        result = run_smoke(settings, frames=args.frames)
    except SimulatorUnavailableError as exc:
        print(f"CARLA smoke run could not start: {exc.message}")
        print("\nThis is not a test failure. CARLA is optional; the ADAPT-X test suite")
        print("runs and passes without a simulator. Start a CARLA server and retry.")
        return 1

    print(render(result))
    print(f"\nwall-clock duration: {time.perf_counter() - started:.1f} s")

    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "map": result.map_name,
            "carla_version": result.carla_version,
            "fixed_delta_seconds": result.fixed_delta_seconds,
            "frames": [record.__dict__ for record in result.frames],
            "ground_truth": [truth.model_dump(mode="json") for truth in result.ground_truth],
            "note": (
                "Live CARLA run. Durations measured on this machine. No accuracy "
                "is measured here; ground truth is recorded for later evaluation."
            ),
        }
        args.json.write_text(json.dumps(payload, indent=2, default=str), encoding="utf-8")
        print(f"full record written to {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
