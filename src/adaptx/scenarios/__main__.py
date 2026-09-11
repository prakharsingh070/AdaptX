"""Run a catalogue scenario: ``python -m adaptx.scenarios``.

::

    python -m adaptx.scenarios list
    python -m adaptx.scenarios run vehicle_approach
    python -m adaptx.scenarios run pedestrian_crossing --seed 7 --json runs/ped.json

Requires a running CARLA server for ``run``. Without one the command reports
why it could not start and exits non-zero; that is not a test failure, and
the test suite does not depend on this command.

What the output is
------------------
Frame counts, stage counts and the scripted-versus-reported distance for
the nearest actor. **No accuracy figure**: comparing what the pipeline
inferred against the ground truth this run recorded is Phase 11, and printing
a percentage here would be presenting that phase's result before it exists.
"""

from __future__ import annotations

import argparse
import pathlib
import time

from adaptx.config.settings import get_settings
from adaptx.core.exceptions import SimulatorUnavailableError
from adaptx.core.logging import configure_logging
from adaptx.scenarios.catalogue import load, scenario_ids
from adaptx.scenarios.models import ScenarioState
from adaptx.scenarios.publish import ScenePublisher
from adaptx.scenarios.result import ScenarioRunResult
from adaptx.scenarios.runner import run_scenario


def render(result: ScenarioRunResult) -> str:
    """A readable summary of one run."""
    definition = result.resolved.definition
    lines = [
        f"ADAPT-X scenario: {result.name} ({result.scenario_id})",
        "=" * 100,
        f"seed {result.seed} | timestep {result.fixed_delta_seconds} s | planned "
        f"{result.planned_frame_count} frames | ran {result.frame_count} | "
        f"state {result.state.value}",
        f"map {result.map_name or 'unknown'} | simulator {result.simulator_version} | "
        f"actors {', '.join(f'{a.actor_id}={a.blueprint}' for a in definition.actors) or 'none'}",
    ]
    if result.error:
        lines.append(f"error: {result.error}")
    lines += [
        "",
        "LIVE SIMULATOR RUN. Stage durations are measured on this machine.",
        "Ground truth was recorded beside every frame and fed to no pipeline stage.",
        "Nothing below is an accuracy figure; that comparison is Phase 11.",
        "",
        f"{'idx':>4}{'t s':>7}{'sim frame':>11}{'points':>9}{'kept':>8}{'det':>5}{'trk':>5}"
        f"{'traj':>6}{'risk':>10}{'regions L/M/H/C':>18}{'cells':>9}{'gt':>4}{'ms':>8}",
        "-" * 100,
    ]
    for record in result.frames:
        counts = record.pipeline
        if counts is None:
            lines.append(
                f"{record.frame_index:>4}{record.scenario_time_s:>7.2f}"
                f"{record.simulator_frame_id:>11}{record.point_count:>9}"
                f"{'-':>8}{'-':>5}{'-':>5}{'-':>6}{'-':>10}{'-':>18}{'-':>9}"
                f"{record.ground_truth_actor_count:>4}{'-':>8}"
            )
            continue
        levels = counts.regions_by_level
        spread = (
            f"{levels.get('low', 0)}/{levels.get('medium', 0)}/"
            f"{levels.get('high', 0)}/{levels.get('critical', 0)}"
        )
        lines.append(
            f"{record.frame_index:>4}{record.scenario_time_s:>7.2f}"
            f"{record.simulator_frame_id:>11}{record.point_count:>9}"
            f"{counts.processed_points:>8}{counts.detections:>5}{counts.tracks:>5}"
            f"{counts.trajectories:>6}{counts.risk_level:>10}{spread:>18}"
            f"{counts.adaptive_cells:>9}{record.ground_truth_actor_count:>4}"
            f"{counts.total_ms:>8.1f}"
        )
    intervals = result.frame_intervals_s()
    if intervals:
        lines += [
            "",
            f"frame interval: min {min(intervals):.4f} s, max {max(intervals):.4f} s "
            f"(expected {result.fixed_delta_seconds} s); monotonic: "
            f"{result.timestamps_are_monotonic()}",
        ]
    return "\n".join(lines)


def _list() -> int:
    print("Catalogue scenarios:")
    for scenario_id in scenario_ids():
        definition = load(scenario_id)
        print(
            f"  {scenario_id:<22} {definition.frame_count:>4} frames @ "
            f"{definition.fixed_delta_seconds} s  actors={len(definition.actors)}  "
            f"{definition.name}"
        )
    return 0


def _run(args: argparse.Namespace) -> int:
    try:
        definition = load(args.scenario)
    except KeyError as exc:
        print(str(exc.args[0]))
        return 2
    if args.seed is not None:
        definition = definition.model_copy(update={"seed": args.seed})

    settings = get_settings().model_copy(deep=True)
    if args.host:
        settings.carla.host = args.host
    if args.port:
        settings.carla.port = args.port

    publisher = (
        None
        if args.publish is None
        else ScenePublisher(
            args.publish,
            scenario_id=definition.scenario_id,
            max_points=settings.dashboard.scene_max_points,
        )
    )
    started = time.perf_counter()
    try:
        result = run_scenario(definition, settings=settings, observer=publisher)
    except SimulatorUnavailableError as exc:
        print(f"scenario could not start: {exc.message}")
        print("\nCARLA is optional; the ADAPT-X test suite runs and passes without a")
        print("simulator. Start a CARLA server and retry.")
        return 1

    if result.state is ScenarioState.FAILED and result.frame_count == 0:
        # Nothing was stepped: the simulator could not be opened or the scene
        # could not be set up. Say so plainly rather than rendering an empty
        # table as if a run had happened.
        print(f"scenario could not start: {result.error}")
        print("\nCARLA is optional; the ADAPT-X test suite runs and passes without a")
        print("simulator. Start a CARLA server and retry.")
        return 1

    print(render(result))
    print(f"\nwall-clock duration: {time.perf_counter() - started:.1f} s")
    if publisher is not None:
        print(
            f"published {publisher.published} frames to {publisher.url}"
            + (f"; {publisher.failed} failed" if publisher.failed else "")
            + ("; publishing was disabled after repeated failures" if publisher.disabled else "")
        )

    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(result.model_dump_json(indent=2), encoding="utf-8")
        print(f"full record written to {args.json}")

    return 0 if result.state is ScenarioState.COMPLETED else 1


def main() -> int:
    """Entry point. Returns a process exit code."""
    parser = argparse.ArgumentParser(
        prog="python -m adaptx.scenarios",
        description="Run deterministic ADAPT-X scenarios against a CARLA server.",
    )
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("list", help="List the catalogue.")
    run = commands.add_parser("run", help="Run one catalogue scenario.")
    run.add_argument("scenario", help="Catalogue id, e.g. vehicle_approach.")
    run.add_argument("--seed", type=int, default=None, help="Override the scenario seed.")
    run.add_argument("--host", type=str, default=None, help="CARLA host.")
    run.add_argument("--port", type=int, default=None, help="CARLA port.")
    run.add_argument("--json", type=pathlib.Path, help="Write the full result here.")
    run.add_argument(
        "--publish",
        type=str,
        default=None,
        metavar="URL",
        help=(
            "Hand each processed frame to a running ADAPT-X backend for the dashboard's "
            "live scene, e.g. http://127.0.0.1:8000. The run is unaffected if it is down."
        ),
    )
    args = parser.parse_args()

    configure_logging("INFO")
    if args.command == "list":
        return _list()
    return _run(args)


if __name__ == "__main__":
    raise SystemExit(main())
