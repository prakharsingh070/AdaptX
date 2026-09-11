"""Dashboard surface (Phase 12): static app, scene channel, stored evidence, boundaries.

The dashboard is a consumer. These tests prove the backend serves it real
outputs and stored evidence unchanged, that the live channel never carries
ground truth, that no control endpoint exists, and - by scanning the
frontend source - that the browser code computes no metric of its own.
"""

from __future__ import annotations

import hashlib
import pathlib
import re
from collections.abc import Iterator

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from adaptx.api.app import DASHBOARD_DIR, create_app
from adaptx.config.settings import Settings
from adaptx.core.lifecycle import build_context
from adaptx.models.risk_assessment import RiskAssessmentResult
from tests.integration.test_adaptive_mapping_pipeline import frame_body, road_scene
from tests.integration.test_adaptive_mapping_pipeline import settings as pipeline_settings
from tests.unit.test_evidence_service import write_report, write_run
from tests.unit.test_scene_service import snapshot_from, synthetic_outputs

REPO = pathlib.Path(__file__).resolve().parents[2]


def dashboard_settings(tmp_path: pathlib.Path) -> Settings:
    base = pipeline_settings()
    (tmp_path / "reports").mkdir(exist_ok=True)
    (tmp_path / "runs").mkdir(exist_ok=True)
    return base.model_copy(
        update={
            "dashboard": base.dashboard.model_copy(
                update={"report_dir": tmp_path / "reports", "run_dir": tmp_path / "runs"}
            )
        }
    )


@pytest.fixture
def dash_app(tmp_path: pathlib.Path) -> FastAPI:
    resolved = dashboard_settings(tmp_path)
    return create_app(settings=resolved, context=build_context(resolved))


@pytest.fixture
def dash(dash_app: FastAPI) -> Iterator[TestClient]:
    with TestClient(dash_app) as client:
        yield client


@pytest.fixture
def evidence_dir(tmp_path: pathlib.Path) -> pathlib.Path:
    (tmp_path / "runs").mkdir(exist_ok=True)
    (tmp_path / "reports").mkdir(exist_ok=True)
    run_path = write_run(tmp_path / "runs", "run.json", frames=4)
    write_report(tmp_path / "reports", run_path, "a.json")
    write_report(tmp_path / "reports", run_path, "b.json")
    return tmp_path


# -- the app itself ----------------------------------------------------------
class TestStaticApp:
    def test_the_dashboard_is_served_and_the_root_redirects_to_it(self, dash: TestClient) -> None:
        assert DASHBOARD_DIR.is_dir()
        page = dash.get("/dashboard/")
        assert page.status_code == 200
        assert "ADAPT-X" in page.text
        assert page.headers["cache-control"] == "no-cache"
        root = dash.get("/", follow_redirects=False)
        assert root.status_code == 307 and root.headers["location"] == "/dashboard/"

    def test_modules_are_served_with_no_cache_so_edits_are_seen(self, dash: TestClient) -> None:
        module = dash.get("/dashboard/app.js")
        assert module.status_code == 200
        assert module.headers["cache-control"] == "no-cache"
        assert "javascript" in module.headers["content-type"]

    def test_backend_health_is_what_the_status_pill_reads(self, dash: TestClient) -> None:
        assert dash.get("/health").status_code == 200
        status = dash.get("/api/v1/system/status").json()
        assert "components" in status


# -- the live scene ----------------------------------------------------------
class TestScene:
    def test_no_scene_until_the_pipeline_has_run(self, dash: TestClient) -> None:
        body = dash.get("/api/v1/scene/latest").json()
        assert body == {
            "sequence": 0,
            "has_scene": False,
            "scene": None,
            "detail": "no frame has been processed yet",
        }

    def test_the_full_chain_publishes_a_downsampled_scene_without_ground_truth(
        self, dash: TestClient
    ) -> None:
        response = dash.post("/api/v1/lidar/adaptive-map", json=frame_body(road_scene()))
        assert response.status_code == 200
        latest = dash.get("/api/v1/scene/latest").json()
        assert latest["has_scene"] and latest["sequence"] == 1
        scene = latest["scene"]
        assert scene["origin"] == "api" and scene["frame_id"] == 0
        assert "ground_truth" not in scene and "expected_poses" not in scene
        points = scene["points"]
        assert points["stage"] == "processed"
        assert points["sample_count"] <= 6000 and len(points["xyz"]) == points["sample_count"]
        assert points["total_count"] >= points["sample_count"]
        # The scene carries what the chain produced, as produced.
        produced = response.json()
        assert {a["track_id"] for a in scene["assessments"]} == {
            a["track_id"] for a in produced["risk"]["assessments"]
        }
        risk = RiskAssessmentResult.model_validate(produced["risk"])
        assert scene["highest_risk_level"] == risk.highest_risk_level.value
        assert scene["tiles"] == produced["decisions"]
        assert scene["adaptive_map"] == produced["map"]

    def test_a_producer_can_hand_over_a_snapshot_and_a_malformed_one_is_refused(
        self, dash: TestClient
    ) -> None:
        payload = snapshot_from(synthetic_outputs()).model_dump(mode="json")
        accepted = dash.post("/api/v1/scene/frame", json=payload)
        assert accepted.status_code == 202
        assert accepted.headers["x-scene-sequence"] == "1"
        assert accepted.json() == {"accepted": True, "sequence": 1, "frame_id": 7}
        latest = dash.get("/api/v1/scene/latest").json()["scene"]
        unknown = next(a for a in latest["assessments"] if a["track_id"] == 1)
        assert unknown["risk_level"] == "unknown" and unknown["risk_score"] is None
        assert next(t for t in latest["tracks"] if t["track_id"] == 1)["velocity"] is None

        payload["ground_truth"] = {"actors": []}
        refused = dash.post("/api/v1/scene/frame", json=payload)
        assert refused.status_code == 422
        assert dash.get("/api/v1/scene/latest").json()["sequence"] == 1

        assert dash.post("/api/v1/scene/frame", json={"frame_id": 1}).status_code == 422

    def test_the_scene_channel_says_hello_then_pushes_each_new_frame(
        self, dash: TestClient
    ) -> None:
        with dash.websocket_connect("/ws/scene") as ws:
            hello = ws.receive_json()
            assert hello["type"] == "hello" and hello["sequence"] == 0
            assert hello["data"] == {"has_scene": False, "poll_interval_s": 0.04}
            dash.post(
                "/api/v1/scene/frame",
                json=snapshot_from(synthetic_outputs()).model_dump(mode="json"),
            )
            pushed = ws.receive_json()
            assert pushed["type"] == "scene" and pushed["sequence"] == 1
            assert pushed["data"]["frame_id"] == 7
            assert "ground_truth" not in pushed["data"]

    def test_a_late_client_receives_the_current_scene_immediately(self, dash: TestClient) -> None:
        dash.post(
            "/api/v1/scene/frame", json=snapshot_from(synthetic_outputs()).model_dump(mode="json")
        )
        with dash.websocket_connect("/ws/scene") as ws:
            hello = ws.receive_json()
            assert hello["data"]["has_scene"] is True and hello["sequence"] == 1
            assert ws.receive_json()["type"] == "scene"


# -- stored evidence ---------------------------------------------------------
class TestEvidence:
    def test_reports_and_runs_are_listed_from_the_configured_directories(
        self, evidence_dir: pathlib.Path, dash: TestClient
    ) -> None:
        reports = dash.get("/api/v1/reports").json()
        assert reports["directory"].endswith("reports")
        assert [r["name"] for r in reports["files"]] == ["a.json", "b.json"]
        runs = dash.get("/api/v1/runs").json()
        assert [r["name"] for r in runs["files"]] == ["run.json"]

    def test_a_report_is_served_unchanged_and_its_file_untouched(
        self, evidence_dir: pathlib.Path, dash: TestClient
    ) -> None:
        path = evidence_dir / "reports" / "a.json"
        before = hashlib.sha256(path.read_bytes()).hexdigest()
        body = dash.get("/api/v1/reports/a.json")
        assert body.status_code == 200
        report = body.json()
        assert report["scenario_id"] == "synthetic"
        assert set(report) >= {
            "detection",
            "tracking",
            "prediction",
            "risk",
            "mapping",
            "adaptive_resolution",
            "resource",
            "limitations",
        }
        assert hashlib.sha256(path.read_bytes()).hexdigest() == before

    def test_a_malformed_report_is_a_404_with_the_reason(
        self, evidence_dir: pathlib.Path, dash: TestClient
    ) -> None:
        (evidence_dir / "reports" / "bad.json").write_text("{not json", encoding="utf-8")
        response = dash.get("/api/v1/reports/bad.json")
        assert response.status_code == 404
        detail = response.json()
        assert "not a valid EvaluationReport" in str(detail)
        assert dash.get("/api/v1/reports/missing.json").status_code == 404
        assert dash.get("/api/v1/reports/..%2Fa.json").status_code in {404, 422}

    def test_comparison_goes_through_the_phase_11_contract(
        self, evidence_dir: pathlib.Path, dash: TestClient
    ) -> None:
        body = dash.get("/api/v1/reports/compare", params={"first": "a.json", "second": "b.json"})
        assert body.status_code == 200
        comparison = body.json()
        assert comparison["same_scenario"] and comparison["deterministic_content_identical"]
        assert comparison["differences"] == []
        assert dash.get("/api/v1/reports/compare", params={"first": "a.json"}).status_code == 422

    def test_run_playback_serves_frames_with_ground_truth_labelled_as_such(
        self, evidence_dir: pathlib.Path, dash: TestClient
    ) -> None:
        summary = dash.get("/api/v1/runs/run.json").json()
        assert summary["frame_count"] == 4 and summary["has_outputs"] is True
        assert "frames" not in summary, "the summary is not the whole record"
        frame = dash.get("/api/v1/runs/run.json/frames/3").json()
        assert frame["name"] == "run.json"
        assert frame["frame"]["frame_index"] == 3
        assert frame["frame"]["outputs"]["tracking"]["tracks"]
        assert frame["ground_truth"]["actors"], "ground truth is served ONLY on this path"
        assert dash.get("/api/v1/runs/run.json/frames/4").status_code == 404
        assert dash.get("/api/v1/runs/run.json/frames/-1").status_code == 404


# -- boundaries --------------------------------------------------------------
# Raw simulator control never crosses HTTP: nothing spawns, destroys, moves,
# ticks or drives an actor from a route. The live extension added exactly
# five HIGH-LEVEL session controls (ADR-056); they are allowlisted by full
# path so a new "start" or "stop" of anything else still fails this test.
FORBIDDEN_ROUTE_WORDS = (
    "spawn",
    "destroy",
    "teleport",
    "start",
    "stop",
    "control",
    "tick",
    "apply",
    "actor",
    "drive",
)
ALLOWED_SESSION_CONTROLS = {
    "POST /api/v1/live/start",
    "POST /api/v1/live/pause",
    "POST /api/v1/live/resume",
    "POST /api/v1/live/stop",
    "POST /api/v1/live/reset",
}


class TestBoundaries:
    def test_no_route_controls_the_simulator(self, dash_app: FastAPI) -> None:
        paths = dash_app.openapi()["paths"]
        offenders = [
            p
            for p in paths
            if any(word in p.lower() for word in FORBIDDEN_ROUTE_WORDS)
            and f"POST {p}" not in ALLOWED_SESSION_CONTROLS
        ]
        assert offenders == [], offenders
        # Every dashboard-facing write is either the snapshot hand-over or one
        # of the allowlisted session controls; nothing else accepts a write.
        writes = sorted(
            f"{method.upper()} {path}"
            for path, operations in paths.items()
            for method in operations
            if method != "get"
            and any(seg in path for seg in ("/scene", "/reports", "/runs", "/live"))
        )
        assert set(writes) == {"POST /api/v1/scene/frame", *ALLOWED_SESSION_CONTROLS}, writes

    def test_the_status_says_what_the_dashboard_is(self, dash: TestClient) -> None:
        components = dash.get("/api/v1/system/status").json()["components"]
        dashboard = next(c for c in components if c["name"] == "dashboard")
        assert dashboard["phase"] == 12
        assert dashboard["implementation"] == "PARTIAL"
        assert "CONSUMER ONLY" in dashboard["detail"]


def js_sources() -> list[pathlib.Path]:
    return sorted(p for p in DASHBOARD_DIR.rglob("*.js") if "tests" not in p.parts)


# Anything that looks like a metric, a distance, a threshold or a rule being
# computed in the browser. Layout arithmetic is allowed in render/ only.
FORBIDDEN_JS = re.compile(
    r"Math\.(sqrt|hypot|atan2|random)|\.reduce\(|\b(risk_score|match_rate|ade|fde|recall|precision|"
    r"threshold\w*)\s*=[^=]|\bpredict\("
)


class TestFrontendBoundaries:
    def test_frontend_sources_exist(self) -> None:
        names = {p.relative_to(DASHBOARD_DIR).as_posix() for p in js_sources()}
        assert {"app.js", "data/format.js", "data/normalise.js", "render/scene.js"} <= names

    def test_no_metric_is_recomputed_in_the_browser(self) -> None:
        offenders = []
        for path in js_sources():
            if path.parent.name == "render":
                continue
            for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
                if FORBIDDEN_JS.search(line):
                    offenders.append(
                        f"{path.relative_to(DASHBOARD_DIR).as_posix()}:{number}: {line.strip()}"
                    )
        assert offenders == [], "\n".join(offenders)

    def test_the_frontend_only_reads_and_never_talks_to_carla(self) -> None:
        fetches = []
        for path in js_sources():
            text = path.read_text(encoding="utf-8")
            assert "import" not in text or "carla" not in re.findall(
                r"import .* from \"([^\"]+)\"", text
            )
            for match in re.finditer(r"fetch\((.*)\)", text):
                fetches.append((path.name, match.group(1)))
            for match in re.finditer(r"method:\s*[\"']([A-Z]+)[\"']", text):
                assert match.group(1) == "POST", f"{path.name} uses {match.group(1)}"
            if "method:" in text:
                assert path.name == "api.js", f"{path.name} sends a non-GET request"
            for url in re.findall(r"[\"'`](/api/v1/[^\"'`]*)[\"'`]", text):
                assert (
                    not any(w in url for w in FORBIDDEN_ROUTE_WORDS)
                    or f"POST {url}" in ALLOWED_SESSION_CONTROLS
                ), url
                assert "carla" not in url or url == "/api/v1/carla/status", url
        assert [name for name, _ in fetches] == ["api.js"], "one fetch site, in data/api.js"

    def test_ground_truth_reaches_only_the_playback_view_model(self) -> None:
        holders = sorted(
            p.relative_to(DASHBOARD_DIR).as_posix()
            for p in js_sources()
            if re.search(r"ground_truth|groundTruth", p.read_text(encoding="utf-8"))
        )
        assert holders == ["data/normalise.js", "views/run.js"], holders
        live = (DASHBOARD_DIR / "views" / "live.js").read_text(encoding="utf-8")
        assert "groundTruth" not in live and "expectedPoses" not in live

    def test_forbidden_words_never_appear_in_the_ui(self) -> None:
        forbidden = re.compile(
            r"collision probability|production[- ]ready|real[- ]time", re.IGNORECASE
        )
        offenders = []
        for path in [*js_sources(), DASHBOARD_DIR / "index.html"]:
            for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
                found = forbidden.search(line)
                if found and "not" not in line.lower():
                    offenders.append(f"{path.name}:{number}: {line.strip()}")
        assert offenders == [], "\n".join(offenders)

    def test_nothing_in_the_frontend_invents_live_data(self) -> None:
        """No random numbers, no recorded clip, no scripted motion anywhere in the sources.

        The live view draws the snapshot the backend pushed and nothing else;
        the only image is the ego camera served per frame by the backend.
        """
        for path in [*js_sources(), DASHBOARD_DIR / "index.html"]:
            text = path.read_text(encoding="utf-8")
            assert "Math.random" not in text, path.name
            assert not re.search(r"\.(mp4|webm|gif)\b", text), f"{path.name} references a video"
        camera_sites = [
            p.name for p in js_sources() if "live/camera" in p.read_text(encoding="utf-8")
        ]
        assert camera_sites == ["api.js"], "the camera URL is built in one place"

    def test_the_stored_mode_never_reads_the_live_scene(self) -> None:
        """A playback scene is built from a run frame only; a live scene from a snapshot only."""
        normalise = (DASHBOARD_DIR / "data" / "normalise.js").read_text(encoding="utf-8")
        run_branch = normalise[normalise.index("export function sceneFromRunFrame") :]
        run_branch = run_branch[: run_branch.index("function buildScene")]
        assert "snapshot" not in run_branch
        assert "ego: null" in run_branch and "control: null" in run_branch
        assert "live: null" in run_branch
        live_branch = normalise[
            normalise.index("export function sceneFromSnapshot") : normalise.index(
                "export function sceneFromRunFrame"
            )
        ]
        assert "ground_truth" not in live_branch and "groundTruth: null" in live_branch

    def test_null_and_unknown_are_formatted_in_exactly_one_place(self) -> None:
        for path in js_sources():
            if path.name in {"format.js"}:
                continue
            code = [
                line
                for line in path.read_text(encoding="utf-8").splitlines()
                if not line.lstrip().startswith(("//", "/*", "*"))
            ]
            assert not any('"Not available"' in line for line in code), (
                f"{path.name} formats null itself"
            )
