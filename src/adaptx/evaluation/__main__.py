"""Command-line evaluation of recorded scenario runs (Phase 11).

Usage::

    python -m adaptx.evaluation evaluate runs/approach.json --json reports/approach.json
    python -m adaptx.evaluation compare reports/a.json reports/b.json
    python -m adaptx.evaluation run vehicle_approach --json runs/approach.json \\
        --report reports/approach.json

``evaluate`` and ``compare`` are **offline**: they read files and need no
simulator. ``run`` is the convenience of the three steps in one - run the
scenario against CARLA, write the record, evaluate it - and needs a server.

Exit codes: 0 success; 1 a run failed or two reports disagree; 2 malformed
input (a missing or invalid file, an unknown scenario, a bad threshold).
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import time

from pydantic import ValidationError

from adaptx.config.settings import get_settings
from adaptx.core.exceptions import SimulatorUnavailableError
from adaptx.core.logging import configure_logging
from adaptx.evaluation.compare import compare_reports
from adaptx.evaluation.dataset import EvaluationInputError
from adaptx.evaluation.evaluator import evaluate
from adaptx.evaluation.models import EvaluationConfiguration, EvaluationReport
from adaptx.evaluation.report import render
from adaptx.models.map import ResolutionLevel
from adaptx.models.risk import RiskLevel
from adaptx.scenarios.catalogue import load
from adaptx.scenarios.models import ScenarioState
from adaptx.scenarios.result import ScenarioRunResult
from adaptx.scenarios.runner import run_scenario


def _configuration(args: argparse.Namespace) -> EvaluationConfiguration:
    overrides: dict[str, object] = {}
    if args.gate:
        overrides["match_gates_m"] = list(args.gate)
    if args.primary_gate is not None:
        overrides["primary_gate_m"] = args.primary_gate
    if args.proximity_event_m is not None:
        overrides["proximity_event_m"] = args.proximity_event_m
    if args.alert_level is not None:
        overrides["alert_level"] = RiskLevel(args.alert_level)
    if args.refined_level is not None:
        overrides["refined_level"] = ResolutionLevel(args.refined_level)
    if args.include_map_actors:
        overrides["include_map_actors"] = True
    return EvaluationConfiguration.model_validate(overrides)


def _read_result(path: pathlib.Path) -> ScenarioRunResult:
    return ScenarioRunResult.model_validate_json(path.read_text(encoding="utf-8"))


def _read_report(path: pathlib.Path) -> EvaluationReport:
    return EvaluationReport.model_validate_json(path.read_text(encoding="utf-8"))


def _write(path: pathlib.Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _evaluate_and_print(
    result: ScenarioRunResult, configuration: EvaluationConfiguration, output: pathlib.Path | None
) -> int:
    report = evaluate(result, configuration=configuration)
    print(render(report))
    if output is not None:
        _write(output, report.model_dump_json(indent=2))
        print(f"report written to {output}")
    return 0


def _evaluate(args: argparse.Namespace) -> int:
    try:
        configuration = _configuration(args)
        result = _read_result(args.run)
    except (OSError, ValueError, ValidationError) as exc:
        print(f"cannot evaluate {args.run}: {exc}")
        return 2
    try:
        return _evaluate_and_print(result, configuration, args.json)
    except EvaluationInputError as exc:
        print(f"cannot evaluate {args.run}: {exc.message}")
        return 2


def _compare(args: argparse.Namespace) -> int:
    try:
        first = _read_report(args.first)
        second = _read_report(args.second)
    except (OSError, ValueError, ValidationError) as exc:
        print(f"cannot compare: {exc}")
        return 2
    comparison = compare_reports(first, second)
    print(f"scenario {first.scenario_id} vs {second.scenario_id}")
    print(f"same scenario/seed/timestep: {comparison.same_scenario}")
    print(f"same evaluation configuration: {comparison.same_configuration}")
    print(f"deterministic content identical: {comparison.deterministic_content_identical}")
    print(f"timing fields ignored: {len(comparison.timing_fields_ignored)}")
    for difference in comparison.differences:
        print(f"  differs: {difference}")
    print("REPEATABLE" if comparison.repeatable else "NOT REPEATABLE")
    return 0 if comparison.repeatable else 1


def _run(args: argparse.Namespace) -> int:
    try:
        definition = load(args.scenario)
        configuration = _configuration(args)
    except KeyError as exc:
        print(str(exc.args[0]))
        return 2
    except ValidationError as exc:
        print(f"invalid evaluation configuration: {exc}")
        return 2
    if args.seed is not None:
        definition = definition.model_copy(update={"seed": args.seed})

    settings = get_settings().model_copy(deep=True)
    if args.host:
        settings.carla.host = args.host
    if args.port:
        settings.carla.port = args.port

    started = time.perf_counter()
    try:
        result = run_scenario(definition, settings=settings)
    except SimulatorUnavailableError as exc:
        print(f"scenario could not start: {exc.message}")
        print("\nCARLA is optional; evaluate a saved record with 'evaluate' instead.")
        return 1
    if result.state is ScenarioState.FAILED and result.frame_count == 0:
        print(f"scenario could not start: {result.error}")
        return 1
    print(
        f"scenario {result.scenario_id}: {result.state.value}, {result.frame_count} frames in "
        f"{time.perf_counter() - started:.1f} s wall clock"
    )
    if args.json is not None:
        _write(args.json, result.model_dump_json(indent=2))
        print(f"run record written to {args.json}")
    try:
        code = _evaluate_and_print(result, configuration, args.report)
    except EvaluationInputError as exc:
        print(f"cannot evaluate the run: {exc.message}")
        return 1
    return code if result.state is ScenarioState.COMPLETED else 1


def _add_configuration_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--gate", type=float, action="append", help="Match gate in metres; repeatable."
    )
    parser.add_argument("--primary-gate", type=float, default=None, help="Primary gate, metres.")
    parser.add_argument(
        "--proximity-event-m", type=float, default=None, help="Proximity event band, metres."
    )
    parser.add_argument(
        "--alert-level",
        choices=[level.value for level in RiskLevel if level is not RiskLevel.UNKNOWN],
        default=None,
        help="Risk level at or above which a track is alerting.",
    )
    parser.add_argument(
        "--refined-level",
        choices=[level.value for level in ResolutionLevel],
        default=None,
        help="Resolution level at or above which a tile counts as refined.",
    )
    parser.add_argument(
        "--include-map-actors",
        action="store_true",
        help="Treat every non-ego simulator actor as ground truth, not only scenario actors.",
    )


def main(argv: list[str] | None = None) -> int:
    """Entry point. Returns a process exit code."""
    parser = argparse.ArgumentParser(
        prog="python -m adaptx.evaluation",
        description="Evaluate recorded ADAPT-X scenario runs against simulator ground truth.",
    )
    commands = parser.add_subparsers(dest="command", required=True)

    evaluate_parser = commands.add_parser("evaluate", help="Evaluate a saved run record.")
    evaluate_parser.add_argument("run", type=pathlib.Path, help="A ScenarioRunResult JSON file.")
    evaluate_parser.add_argument("--json", type=pathlib.Path, help="Write the report here.")
    _add_configuration_arguments(evaluate_parser)

    compare_parser = commands.add_parser(
        "compare", help="Check two evaluation reports agree on deterministic content."
    )
    compare_parser.add_argument("first", type=pathlib.Path)
    compare_parser.add_argument("second", type=pathlib.Path)

    run_parser = commands.add_parser(
        "run", help="Run a catalogue scenario against CARLA, record it and evaluate it."
    )
    run_parser.add_argument("scenario", help="Catalogue id, e.g. vehicle_approach.")
    run_parser.add_argument("--seed", type=int, default=None, help="Override the scenario seed.")
    run_parser.add_argument("--host", type=str, default=None, help="CARLA host.")
    run_parser.add_argument("--port", type=int, default=None, help="CARLA port.")
    run_parser.add_argument("--json", type=pathlib.Path, help="Write the run record here.")
    run_parser.add_argument("--report", type=pathlib.Path, help="Write the report here.")
    _add_configuration_arguments(run_parser)

    args = parser.parse_args(argv)
    configure_logging("WARNING")
    if args.command == "evaluate":
        return _evaluate(args)
    if args.command == "compare":
        return _compare(args)
    return _run(args)


if __name__ == "__main__":
    sys.exit(main())
