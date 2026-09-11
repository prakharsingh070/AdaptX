"""Stored evidence service (Phase 12): read-only, name-restricted, validated."""

from __future__ import annotations

import hashlib
import json
import pathlib

import pytest

from adaptx.config.settings import DashboardSettings
from adaptx.core.exceptions import StoredEvidenceError
from adaptx.evaluation import evaluate
from adaptx.evidence.service import EvidenceService
from tests.fixtures.evaluation import ActorSpec, FrameSpec, RunBuilder, TrackSpec


def write_run(directory: pathlib.Path, name: str = "run.json", frames: int = 4) -> pathlib.Path:
    builder = RunBuilder({"target": 5})
    for i in range(frames):
        builder.frame(
            FrameSpec(
                tracks=[TrackSpec(0, 10.0 - i * 0.4, 0.0)],
                actors=[ActorSpec(5, 10.0 - i * 0.4, 0.0)],
            )
        )
    path = directory / name
    path.write_text(builder.build().model_dump_json(), encoding="utf-8")
    return path


def write_report(
    directory: pathlib.Path, run_path: pathlib.Path, name: str = "report.json"
) -> pathlib.Path:
    from adaptx.scenarios.result import ScenarioRunResult

    record = ScenarioRunResult.model_validate_json(run_path.read_text(encoding="utf-8"))
    path = directory / name
    path.write_text(evaluate(record, git_commit="x").model_dump_json(), encoding="utf-8")
    return path


@pytest.fixture
def service(tmp_path: pathlib.Path) -> EvidenceService:
    (tmp_path / "reports").mkdir()
    (tmp_path / "runs").mkdir()
    return EvidenceService(
        DashboardSettings(
            report_dir=tmp_path / "reports", run_dir=tmp_path / "runs", run_cache_size=1
        )
    )


def digest(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class TestNames:
    @pytest.mark.parametrize("name", ["../x.json", "a/b.json", "..", "", "run.txt", "C:\\x.json"])
    def test_unsafe_or_wrong_names_are_refused_before_touching_the_disk(
        self, service: EvidenceService, name: str
    ) -> None:
        with pytest.raises(StoredEvidenceError):
            service.load_report(name)
        with pytest.raises(StoredEvidenceError):
            service.load_run(name)

    def test_a_missing_file_is_reported_by_name(self, service: EvidenceService) -> None:
        with pytest.raises(StoredEvidenceError, match=r"no such file: nope.json"):
            service.load_report("nope.json")


class TestReports:
    def test_listing_and_loading_a_valid_report(self, service: EvidenceService) -> None:
        run_path = write_run(service.run_dir)
        report_path = write_report(service.report_dir, run_path)
        before = digest(report_path)
        listed = service.list_reports()
        assert [f.name for f in listed] == ["report.json"]
        assert listed[0].size_bytes == report_path.stat().st_size
        report = service.load_report("report.json")
        assert report.scenario_id == "synthetic"
        assert digest(report_path) == before, "reading never rewrites the file"

    def test_a_malformed_report_fails_with_the_validation_error(
        self, service: EvidenceService
    ) -> None:
        (service.report_dir / "bad.json").write_text('{"scenario_id": "x"}', encoding="utf-8")
        with pytest.raises(StoredEvidenceError, match="not a valid EvaluationReport"):
            service.load_report("bad.json")
        (service.report_dir / "broken.json").write_text("{not json", encoding="utf-8")
        with pytest.raises(StoredEvidenceError, match="not a valid EvaluationReport"):
            service.load_report("broken.json")

    def test_compare_uses_the_phase_11_contract(self, service: EvidenceService) -> None:
        run_path = write_run(service.run_dir)
        write_report(service.report_dir, run_path, "a.json")
        write_report(service.report_dir, run_path, "b.json")
        comparison = service.compare("a.json", "b.json")
        assert comparison.repeatable
        assert comparison.differences == []

    def test_a_missing_directory_lists_nothing(self, tmp_path: pathlib.Path) -> None:
        service = EvidenceService(
            DashboardSettings(report_dir=tmp_path / "missing", run_dir=tmp_path / "missing")
        )
        assert service.list_reports() == [] and service.list_runs() == []


class TestRuns:
    def test_summary_and_frames_are_served_and_the_file_is_untouched(
        self, service: EvidenceService
    ) -> None:
        run_path = write_run(service.run_dir, frames=4)
        before = digest(run_path)
        summary = service.run_summary("run.json")
        assert summary.frame_count == 4 and summary.has_outputs
        assert summary.actors == ["target"]
        frame = service.run_frame("run.json", 2)
        assert frame.frame["frame_index"] == 2
        assert frame.ground_truth["actors"], "ground truth is served in playback, labelled"
        assert "outputs" in frame.frame and frame.frame["outputs"]["tracking"]["tracks"]
        assert digest(run_path) == before

    def test_a_frame_outside_the_run_is_an_error(self, service: EvidenceService) -> None:
        write_run(service.run_dir, frames=2)
        with pytest.raises(StoredEvidenceError, match="outside the run"):
            service.run_frame("run.json", 2)
        with pytest.raises(StoredEvidenceError, match="outside the run"):
            service.run_frame("run.json", -1)

    def test_runs_are_cached_up_to_the_configured_size(self, service: EvidenceService) -> None:
        write_run(service.run_dir, "one.json")
        write_run(service.run_dir, "two.json")
        first = service.load_run("one.json")
        assert service.load_run("one.json") is first
        service.load_run("two.json")  # cache size 1: evicts one.json
        assert service.load_run("one.json") is not first

    def test_a_served_frame_is_a_copy_that_cannot_reach_the_cached_run(
        self, service: EvidenceService
    ) -> None:
        write_run(service.run_dir)
        served = service.run_frame("run.json", 0)
        served.frame["outputs"]["tracking"]["tracks"].clear()
        again = service.run_frame("run.json", 0)
        assert again.frame["outputs"]["tracking"]["tracks"], "the cached run was not mutated"

    def test_a_malformed_run_fails_with_the_validation_error(
        self, service: EvidenceService
    ) -> None:
        (service.run_dir / "bad.json").write_text(json.dumps({"frames": []}), encoding="utf-8")
        with pytest.raises(StoredEvidenceError, match="not a valid ScenarioRunResult"):
            service.run_summary("bad.json")

    def test_summary_reports_directories_and_counts(self, service: EvidenceService) -> None:
        write_run(service.run_dir)
        summary = service.summary()
        assert summary["runs"] == 1 and summary["reports"] == 0
