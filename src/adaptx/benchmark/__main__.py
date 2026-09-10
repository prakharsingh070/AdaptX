"""Run the LiDAR pipeline benchmark: ``python -m adaptx.benchmark``.

Prints a readable table and optionally writes the full report as JSON, which is
the reproducible artefact: it carries the dataset, seed, configuration, version
and environment alongside every measurement.
"""

from __future__ import annotations

import argparse
import json
import pathlib

from adaptx.benchmark.baseline import (
    BASELINE_PROFILE,
    BASELINE_VOXEL_SIZE_M,
    FILTER_ONLY_PROFILE,
    filter_only_settings,
    fixed_resolution_settings,
)
from adaptx.benchmark.datasets import SIZE_LADDER, DatasetScenario
from adaptx.benchmark.detection import render as render_detection
from adaptx.benchmark.detection import run_detection_benchmark
from adaptx.benchmark.mapping import render as render_mapping
from adaptx.benchmark.mapping import run_mapping_benchmark
from adaptx.benchmark.models import BenchmarkReport
from adaptx.benchmark.prediction import render as render_prediction
from adaptx.benchmark.prediction import run_prediction_benchmark
from adaptx.benchmark.risk import render as render_risk
from adaptx.benchmark.risk import run_risk_benchmark
from adaptx.benchmark.runner import DEFAULT_REPEATS, DEFAULT_WARMUP, BenchmarkRunner
from adaptx.benchmark.tracking import render as render_tracking
from adaptx.benchmark.tracking import run_tracking_benchmark
from adaptx.core.logging import configure_logging


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="python -m adaptx.benchmark",
        description="Benchmark the ADAPT-X LiDAR processing pipeline on synthetic data.",
    )
    parser.add_argument(
        "--profile",
        choices=[BASELINE_PROFILE, FILTER_ONLY_PROFILE],
        default=BASELINE_PROFILE,
        help="Which pipeline configuration to measure.",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=BASELINE_VOXEL_SIZE_M,
        help=f"Fixed voxel size in metres (default {BASELINE_VOXEL_SIZE_M}).",
    )
    parser.add_argument("--repeats", type=int, default=DEFAULT_REPEATS)
    parser.add_argument("--warmup", type=int, default=DEFAULT_WARMUP)
    parser.add_argument(
        "--scenarios",
        nargs="*",
        choices=[s.value for s in DatasetScenario],
        help="Scenarios to run (default: the small/medium/large size ladder).",
    )
    parser.add_argument(
        "--detect",
        action="store_true",
        help="Benchmark object detection on the non-ground output instead.",
    )
    parser.add_argument(
        "--track",
        action="store_true",
        help="Benchmark the tracker on synthetic detections instead.",
    )
    parser.add_argument(
        "--predict",
        action="store_true",
        help="Benchmark trajectory prediction on synthetic tracks instead.",
    )
    parser.add_argument(
        "--map",
        action="store_true",
        help="Benchmark 2.5D mapping across a resolution sweep instead.",
    )
    parser.add_argument(
        "--risk",
        action="store_true",
        help="Benchmark risk assessment on synthetic tracks instead.",
    )
    parser.add_argument("--json", type=pathlib.Path, help="Write the full report here.")
    return parser.parse_args()


def _render(report: BenchmarkReport) -> str:
    """A readable summary. The JSON report remains the record of truth."""
    environment = report.environment
    lines = [
        "ADAPT-X LiDAR pipeline benchmark",
        "=" * 103,
        f"adaptx {environment.adaptx_version} | python {environment.python_version} "
        f"| numpy {environment.numpy_version}",
        f"{environment.platform} | {environment.cpu_count} logical CPUs",
        f"warm-up runs discarded: {report.warmup_runs}",
        "",
        "ALL DATASETS ARE SYNTHETIC. These figures measure pipeline speed on",
        "generated geometry. They say nothing about perception accuracy or",
        "real-world autonomous-driving performance.",
        "",
        f"{'scenario':<20}{'profile':<26}{'points in':>10}{'out':>8}"
        f"{'median ms':>11}{'spread':>16}{'pts/s':>12}{'frames/s':>10}",
        "-" * 103,
    ]
    for result in report.results:
        points_per_second = result.points_per_second
        frames_per_second = result.frames_per_second
        lines.append(
            f"{result.dataset.scenario.value:<20}{result.profile:<26}"
            f"{result.input_point_count:>10}{result.output_point_count:>8}"
            f"{result.timing.median_ms:>11.2f}"
            f"{f'{result.timing.min_ms:.1f}-{result.timing.max_ms:.1f}':>16}"
            f"{('n/a' if points_per_second is None else f'{points_per_second:,.0f}'):>12}"
            f"{('n/a' if frames_per_second is None else f'{frames_per_second:.1f}'):>10}"
        )

    lines += ["", "Per-stage timing (single representative run, not averaged):"]
    for result in report.results:
        lines.append(f"  {result.dataset.scenario.value}:")
        for stage in result.stages:
            lines.append(
                f"    {stage.stage.value:<22}{stage.duration_ms:>9.2f} ms"
                f"{stage.input_points:>10} -> {stage.output_points}"
            )
        peak = result.peak_memory_mb
        lines.append(
            f"    {'peak tracked memory':<22}{('n/a' if peak is None else f'{peak:>9.1f} MB')}"
        )
        for note in result.unavailable:
            lines.append(f"    note: {note}")
    return "\n".join(lines)


def main() -> None:
    """Entry point."""
    args = _parse_args()
    configure_logging("WARNING")

    scenarios = (
        tuple(DatasetScenario(name) for name in args.scenarios) if args.scenarios else SIZE_LADDER
    )

    if args.risk:
        risk_report = run_risk_benchmark(repeats=args.repeats, warmup=args.warmup)
        print(render_risk(risk_report))
        if args.json:
            args.json.parent.mkdir(parents=True, exist_ok=True)
            args.json.write_text(
                json.dumps(risk_report.model_dump(mode="json"), indent=2),
                encoding="utf-8",
            )
            print(f"\nfull report written to {args.json}")
        return

    if args.map:
        mapping_report = run_mapping_benchmark(scenarios, repeats=args.repeats, warmup=args.warmup)
        print(render_mapping(mapping_report))
        if args.json:
            args.json.parent.mkdir(parents=True, exist_ok=True)
            args.json.write_text(
                json.dumps(mapping_report.model_dump(mode="json"), indent=2),
                encoding="utf-8",
            )
            print(f"\nfull report written to {args.json}")
        return

    if args.predict:
        prediction_report = run_prediction_benchmark(repeats=args.repeats, warmup=args.warmup)
        print(render_prediction(prediction_report))
        if args.json:
            args.json.parent.mkdir(parents=True, exist_ok=True)
            args.json.write_text(
                json.dumps(prediction_report.model_dump(mode="json"), indent=2),
                encoding="utf-8",
            )
            print(f"\nfull report written to {args.json}")
        return

    if args.track:
        tracking_report = run_tracking_benchmark(repeats=args.repeats, warmup=args.warmup)
        print(render_tracking(tracking_report))
        if args.json:
            args.json.parent.mkdir(parents=True, exist_ok=True)
            args.json.write_text(
                json.dumps(tracking_report.model_dump(mode="json"), indent=2),
                encoding="utf-8",
            )
        return

    if args.detect:
        detection_report = run_detection_benchmark(
            scenarios,
            lidar_settings=fixed_resolution_settings(voxel_size_m=args.voxel_size),
            repeats=args.repeats,
            warmup=args.warmup,
        )
        print(render_detection(detection_report))
        if args.json:
            args.json.parent.mkdir(parents=True, exist_ok=True)
            args.json.write_text(
                json.dumps(detection_report.model_dump(mode="json"), indent=2),
                encoding="utf-8",
            )
            print(f"\nfull report written to {args.json}")
        return

    settings = (
        fixed_resolution_settings(voxel_size_m=args.voxel_size)
        if args.profile == BASELINE_PROFILE
        else filter_only_settings()
    )
    runner = BenchmarkRunner(
        settings, profile=args.profile, repeats=args.repeats, warmup=args.warmup
    )
    scenarios = (
        tuple(DatasetScenario(name) for name in args.scenarios) if args.scenarios else SIZE_LADDER
    )
    report = runner.run_all(scenarios)

    print(_render(report))

    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report.model_dump(mode="json"), indent=2), encoding="utf-8")
        print(f"\nfull report written to {args.json}")


if __name__ == "__main__":
    main()
