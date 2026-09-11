"""Evaluation of real scenario runs (Phase 11): record -> file -> report.

The runs come from the fake simulator, so **nothing here is a CARLA figure**;
what these tests prove is the plumbing - a catalogue scenario runs, its
record carries the pipeline outputs, the record survives the disk, and the
evaluator reads it back offline with no simulator in the process - and the
boundary: ground truth reaches evaluation and nothing else.
"""

from __future__ import annotations

import json
import pathlib
import subprocess
import sys

import pytest

from adaptx.config.settings import Settings
from adaptx.evaluation import EvaluationReport, MetricStatus, compare_reports, evaluate, render
from adaptx.evaluation.__main__ import main
from adaptx.scenarios.catalogue import load, scenario_ids
from adaptx.scenarios.result import ScenarioRunResult
from tests.integration.test_carla_pipeline import _leaked_modules, sim_settings
from tests.integration.test_scenario_pipeline import run, short

SRC = pathlib.Path(__file__).resolve().parents[2] / "src" / "adaptx"

#: Packages that must never import the evaluation layer or the ground truth.
PRODUCTION_PACKAGES = (
    "perception",
    "tracking",
    "prediction",
    "mapping",
    "risk",
    "services",
    # Post-Phase-12: the ego controller and the live loop read the pipeline's
    # outputs and the ego's own odometry, never ground truth or evaluation.
    "control",
    "live",
)


@pytest.fixture
def settings() -> Settings:
    return sim_settings()


@pytest.fixture(scope="module")
def approach_record() -> ScenarioRunResult:
    """One short fake-simulator run of the approach scenario, shared read-only."""
    return run(short(load("vehicle_approach"), 1.0), sim_settings())


class TestRecordToReport:
    def test_a_processed_run_carries_the_pipeline_outputs(
        self, approach_record: ScenarioRunResult
    ) -> None:
        assert approach_record.has_outputs
        assert approach_record.sensor is not None
        assert approach_record.sensor.mount.z == pytest.approx(1.8)
        outputs = approach_record.frames[0].outputs
        assert outputs is not None
        assert outputs.counts() == approach_record.frames[0].pipeline

    def test_the_record_still_carries_no_evaluation_figure(
        self, approach_record: ScenarioRunResult
    ) -> None:
        """Evaluation belongs to the report, not to the evidence (ADR-050)."""
        payload = json.loads(approach_record.model_dump_json())
        text = json.dumps(payload)
        for forbidden in ('"ade', '"fde', '"match_rate"', '"precision"', '"recall"'):
            assert forbidden not in text

    def test_a_catalogue_run_evaluates_to_measured_sections(
        self, approach_record: ScenarioRunResult
    ) -> None:
        report = evaluate(approach_record)
        assert report.scenario_id == "vehicle_approach"
        assert report.evaluated_frame_count == approach_record.frame_count
        assert report.tracking.status is MetricStatus.MEASURED
        assert report.detection.status is MetricStatus.MEASURED
        assert report.adaptive_resolution.status is MetricStatus.MEASURED
        assert report.mapping.accuracy_status is MetricStatus.UNAVAILABLE
        assert report.resource.status is MetricStatus.MEASURED
        assert "vehicle_approach" in render(report)

    def test_every_catalogue_scenario_evaluates(self, settings: Settings) -> None:
        for scenario_id in scenario_ids():
            # One second: long enough for the approach scenario's actor, which
            # starts 45 m out, to enter the test map's 40 m bounds.
            report = evaluate(run(short(load(scenario_id), 1.0), settings))
            assert report.scenario_id == scenario_id
            assert report.tracking.status is MetricStatus.MEASURED, scenario_id

    def test_the_record_can_be_evaluated_from_disk_without_a_simulator(
        self, approach_record: ScenarioRunResult, tmp_path: pathlib.Path
    ) -> None:
        path = tmp_path / "run.json"
        path.write_text(approach_record.model_dump_json(), encoding="utf-8")
        rebuilt = ScenarioRunResult.model_validate_json(path.read_text(encoding="utf-8"))
        assert rebuilt.has_outputs
        from_disk = evaluate(rebuilt, git_commit="x")
        in_memory = evaluate(approach_record, git_commit="x")
        assert compare_reports(in_memory, from_disk).repeatable

    def test_two_runs_of_the_same_scenario_agree_on_deterministic_content(
        self, settings: Settings
    ) -> None:
        definition = short(load("stationary_vehicle"), 0.5)
        first = evaluate(run(definition, settings), git_commit="x")
        second = evaluate(run(definition, settings), git_commit="x")
        comparison = compare_reports(first, second)
        assert comparison.repeatable, comparison.differences
        assert comparison.timing_fields_ignored  # timings were compared to nothing

    def test_the_report_records_what_is_needed_to_reproduce_it(
        self, approach_record: ScenarioRunResult
    ) -> None:
        report = evaluate(approach_record)
        assert report.seed == approach_record.seed
        assert report.fixed_delta_seconds == approach_record.fixed_delta_seconds
        assert report.simulator_version == approach_record.simulator_version
        assert report.sensor == approach_record.sensor
        assert report.pipeline_configuration["adaptive_resolution"]["tile_size_m"] == 10.0
        assert report.configuration.match_gates_m == [1.0, 2.0, 4.0]


class TestCommandLine:
    def test_evaluate_reads_a_file_and_writes_a_report(
        self,
        approach_record: ScenarioRunResult,
        tmp_path: pathlib.Path,
        capsys: pytest.CaptureFixture[str],
    ) -> None:
        run_path = tmp_path / "run.json"
        run_path.write_text(approach_record.model_dump_json(), encoding="utf-8")
        report_path = tmp_path / "out" / "report.json"
        code = main(
            [
                "evaluate",
                str(run_path),
                "--json",
                str(report_path),
                "--gate",
                "1.5",
                "--gate",
                "3.0",
                "--primary-gate",
                "3.0",
            ]
        )
        assert code == 0
        out = capsys.readouterr().out
        assert "ADAPT-X PHASE 11 EVALUATION" in out
        report = EvaluationReport.model_validate_json(report_path.read_text(encoding="utf-8"))
        assert report.configuration.match_gates_m == [1.5, 3.0]

    def test_evaluate_refuses_malformed_input_with_exit_code_2(
        self, tmp_path: pathlib.Path, capsys: pytest.CaptureFixture[str]
    ) -> None:
        bad = tmp_path / "bad.json"
        bad.write_text('{"scenario_id": "x"}', encoding="utf-8")
        assert main(["evaluate", str(bad)]) == 2
        assert "cannot evaluate" in capsys.readouterr().out
        assert main(["evaluate", str(tmp_path / "missing.json")]) == 2

    def test_evaluate_refuses_a_counts_only_record(
        self, approach_record: ScenarioRunResult, tmp_path: pathlib.Path
    ) -> None:
        stripped = approach_record.model_copy(
            update={
                "frames": [f.model_copy(update={"outputs": None}) for f in approach_record.frames]
            }
        )
        path = tmp_path / "counts.json"
        path.write_text(stripped.model_dump_json(), encoding="utf-8")
        assert main(["evaluate", str(path)]) == 2

    def test_an_invalid_threshold_is_exit_code_2(
        self, approach_record: ScenarioRunResult, tmp_path: pathlib.Path
    ) -> None:
        path = tmp_path / "run.json"
        path.write_text(approach_record.model_dump_json(), encoding="utf-8")
        assert main(["evaluate", str(path), "--primary-gate", "9.0"]) == 2

    def test_compare_distinguishes_repeatable_from_not(
        self,
        approach_record: ScenarioRunResult,
        tmp_path: pathlib.Path,
        capsys: pytest.CaptureFixture[str],
    ) -> None:
        a = tmp_path / "a.json"
        b = tmp_path / "b.json"
        report = evaluate(approach_record, git_commit="x")
        a.write_text(report.model_dump_json(), encoding="utf-8")
        b.write_text(evaluate(approach_record, git_commit="y").model_dump_json(), encoding="utf-8")
        assert main(["compare", str(a), str(b)]) == 0
        assert "REPEATABLE" in capsys.readouterr().out
        other = report.model_copy(update={"seed": report.seed + 1})
        b.write_text(other.model_dump_json(), encoding="utf-8")
        assert main(["compare", str(a), str(b)]) == 1
        assert "NOT REPEATABLE" in capsys.readouterr().out

    def test_run_without_a_simulator_fails_honestly(self) -> None:
        import os
        import socket

        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            closed_port = probe.getsockname()[1]
        completed = subprocess.run(
            [
                sys.executable,
                "-m",
                "adaptx.evaluation",
                "run",
                "vehicle_approach",
                "--host",
                "127.0.0.1",
                "--port",
                str(closed_port),
            ],
            capture_output=True,
            text=True,
            check=False,
            env={**os.environ, "ADAPTX_CARLA__TIMEOUT_S": "2"},
        )
        assert completed.returncode == 1
        assert "could not start" in completed.stdout

    def test_an_unknown_scenario_is_exit_code_2(self) -> None:
        assert main(["run", "no_such_scene"]) == 2


class TestGroundTruthReachesOnlyEvaluation:
    def test_no_production_package_imports_evaluation(self) -> None:
        """Evaluation reads the pipeline; the pipeline never reads evaluation.

        Since Phase 12 the API composition root (``adaptx.api.app``) does
        import the evaluation layer - through ``adaptx.api.routes.evidence``
        only, to serve STORED reports read-only to the dashboard - so it is
        no longer in this list; the static check below pins that single
        allowed site. The pipeline stages, the runner and the application
        context stay clean.
        """
        leaked = _leaked_modules(
            [
                "adaptx.core.lifecycle",
                "adaptx.perception.detector",
                "adaptx.tracking.tracker",
                "adaptx.prediction.constant_velocity",
                "adaptx.mapping.controller",
                "adaptx.mapping.adaptive_mapper",
                "adaptx.risk.heuristic",
                "adaptx.scenarios.runner",
                "adaptx.control.policy",
                "adaptx.live.service",
            ],
            forbidden="adaptx.evaluation",
        )
        assert leaked == [], f"production code imported the evaluation layer: {leaked}"

    def test_no_production_source_imports_evaluation_or_ground_truth(self) -> None:
        """Static check over the source, independent of import laziness.

        Import statements and the ground-truth type names; a status string that
        tells a user to run ``python -m adaptx.evaluation`` is not an import.
        """
        needles = (
            "import adaptx.evaluation",
            "from adaptx.evaluation",
            "import adaptx.carla.ground_truth",
            "from adaptx.carla.ground_truth",
            "GroundTruthFrame",
            "GroundTruthActor",
        )
        offenders: list[str] = []
        for package in PRODUCTION_PACKAGES:
            for path in (SRC / package).rglob("*.py"):
                text = path.read_text(encoding="utf-8")
                offenders.extend(
                    f"{path.relative_to(SRC)}: {needle}" for needle in needles if needle in text
                )
        assert offenders == []

    def test_the_api_reaches_evaluation_only_through_the_evidence_route(self) -> None:
        """Phase 12 serves stored reports; nothing else in api/ or core/ may touch them."""
        allowed = {"api/routes/evidence.py"}
        holders = sorted(
            path.relative_to(SRC).as_posix()
            for package in ("api", "core")
            for path in (SRC / package).rglob("*.py")
            if "adaptx.evaluation" in path.read_text(encoding="utf-8")
        )
        assert holders == sorted(allowed), holders

    def test_evaluation_never_requires_the_carla_package(self) -> None:
        assert _leaked_modules(["adaptx.evaluation", "adaptx.evaluation.__main__"], "carla") == []

    def test_evaluation_does_not_modify_the_record(
        self, approach_record: ScenarioRunResult
    ) -> None:
        before = approach_record.model_dump_json()
        evaluate(approach_record)
        assert approach_record.model_dump_json() == before

    def test_the_pipeline_output_is_the_same_whether_or_not_it_is_evaluated(
        self, settings: Settings
    ) -> None:
        """Evaluating a run changes nothing about the run: the stage counts of a
        fresh identical run match those of the evaluated one exactly."""
        definition = short(load("stationary_vehicle"), 0.5)
        first = run(definition, settings)
        evaluate(first)
        second = run(definition, settings)

        def content(record: ScenarioRunResult) -> list[dict[str, object]]:
            return [
                {k: v for k, v in f.pipeline.model_dump().items() if k != "stage_ms"}
                for f in record.frames
                if f.pipeline is not None
            ]

        assert content(first) == content(second)
