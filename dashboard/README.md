# ADAPT-X Dashboard

**Status: Phase 12 + live simulation extension, partial.** A static engineering console
served by the backend at `/dashboard`, now with an explicit **LIVE SIMULATION / STORED
EVALUATION** mode and the controls of a running CARLA session. Design, views, limitations
and how to run it: [`../docs/DASHBOARD.md`](../docs/DASHBOARD.md). Decision records:
ADR-055 (consumer), ADR-056 (live loop).

Vanilla ES modules, Canvas 2D, no framework, no build step, no dependency. The backend
serves this directory as-is with `Cache-Control: no-cache`.

## What it is

A **consumer**. It draws what the pipeline last produced (live, over `/ws/scene`) and what
a stored Phase 11 evaluation report or Phase 10 run record says (read-only, over
`/api/v1/reports` and `/api/v1/runs`). Every number on screen is a backend field. The only
arithmetic in this directory is pixel layout inside `render/`.

## What it is not

- It computes no risk, trajectory, resolution, match, distance, rate, ratio or metric.
  `tests/integration/test_dashboard_api.py` scans this source for exactly that.
- It touches no actor: its one `fetch` site (`data/api.js`) issues `GET`s plus `POST`s to
  exactly five high-level session controls (`/api/v1/live/start|pause|resume|stop|reset`),
  which start a catalogue scenario inside the backend. No spawn, teleport, tick or drive
  exists over HTTP for it to call.
- In LIVE mode with CARLA disconnected it says so; it never falls back to a recording or
  invents a frame. STORED EVALUATION is chosen explicitly and labelled on every panel.
- The Front View is the ego's RGB camera, one PNG per live frame from the backend -
  display only; perception uses LiDAR.
- It never shows ground truth on the live path; ground truth appears only in the Run /
  Scenario view of a stored run, labelled as evaluation-only.
- It never displays "collision probability", "safe", "production ready" or "real-time".
  `UNKNOWN` stays `UNKNOWN`; `null` is "Not available"; unobserved is never free.

## Layout

| Path | Role |
|---|---|
| `index.html`, `styles.css` | Shell: header pills, left rail (views, report / run pickers, overlay toggles), content, footer |
| `app.js` | State, channels, navigation by URL hash, the LIVE / STORED mode, live status and event polling, the five session controls, per-tab memory of choices |
| `ui.js` | Tiny DOM helpers (`el`, `card`, `kv`, `notice`, `table`, `levelTag`) |
| `data/format.js` | **The only place** null becomes "Not available" and levels are labelled |
| `data/normalise.js` | Backend contracts → view models; joins tracks, assessments and paths by track id; derives nothing |
| `data/api.js`, `data/channel.js`, `data/evidence.js` | GET wrappers; reconnecting WebSocket with a stale indicator; deep-frozen report and run stores with frame prefetch |
| `render/projection.js` | Top-down (+X up, +Y left) and perspective (camera behind and above the ego) transforms |
| `render/scene.js`, `render/charts.js` | Canvas drawing of points, tiles, boxes, paths; small charts of the report's own numbers |
| `views/` | Overview, Live Scene, Object Inspector, Spatial Map, Adaptive Resolution, Evaluation, Fixed vs Adaptive, Run / Scenario, and the shared viewport |
| `tests/` | `node --test` unit tests for format, normalise and projection |

## Tests

```bash
node --test tests/*.test.mjs
```

Node 22 was used; nothing is installed. The backend-side tests are
`tests/unit/test_scene_service.py`, `tests/unit/test_evidence_service.py` and
`tests/integration/test_dashboard_api.py` in the repository root.

## Rules for this directory

- No perception logic, no recomputation of a backend figure, no new dependency, no build
  step.
- Surface provenance: `source` and the live / stored mode label are always visible.
- A panel with no backend source says *Not implemented* or *Not measured* in the same card
  footprint; it never shows a placeholder number.
- The visual target is `../docs/design/dashboard-target.png`; the look is copied, the
  mockup's numbers never are.
