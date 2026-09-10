# Dashboard UI Specification

The agreed visual target for the ADAPT-X dashboard, and — panel by panel — what the backend
can actually feed it today.

Requirements live in [`knowledge-base/15_dashboard-ui.md`](knowledge-base/15_dashboard-ui.md);
this document is the design and build plan. The dashboard itself is **Phase 12** and does not
exist yet (`dashboard/` holds only a README).

---

## Design target

A dark operational console, single screen, no scrolling on a 1536×1024 viewport:

- **Header** — wordmark, subtitle "Adaptive Dynamic Perception and Tracking", system status
  pill, CARLA connection pill, and buttons for CARLA Simulator, Scenario Manager, Settings.
- **Left rail (176px)** — Dashboard, Point Cloud, Adaptive Map, Risk Map, Tracking,
  Prediction, Performance, Scenario Control, Logs. Quick Controls below (sensor toggles,
  overlay toggles, Export Report).
- **Centre** — large 3D LiDAR viewport with 3D / Top / Front / Side tabs and a live/replay
  selector.
- **Right columns** — System Status, Detected Objects, Recent Events, Simulation/Map Info.
- **Mid row** — Adaptive 2.5D Map and Risk Map side by side, each with its own legend;
  Object Tracking & Prediction with a trajectory inset.
- **Bottom row** — six pipeline-stage cards: LiDAR Processing, Perception, Prediction,
  Routing, Decision, Planning, each with a small preview thumbnail.
- **Footer** — version, active simulator, scenario name, run time, tagline.

**Visual language:** near-black background (`#0a0e17`-ish), panel surfaces a step lighter
with 1px borders, cyan/blue primary accent, semantic status colours (green ok, amber
caution, red critical), turbo/jet colour ramp for height and risk fields, monospace for
numerics. Every panel is a titled card with an icon.

This is a good target and the build should match it. What follows is what the target costs.

---

## Data-source audit

The mockup shows roughly forty live values. **Three of the eighteen panels can be fully
populated from the backend as it stands today**; four more partially.

Legend: **Real** = a backend field exists and is measured · **Partial** = some fields exist ·
**None** = no source, and none until the owning phase ships.

| Panel / value | Backend source today | Status |
|---|---|---|
| Header — system status | `GET /api/v1/system/status` → `state` | **Real** |
| Header — CARLA connection | `GET /api/v1/carla/status` | **Real** |
| Footer — version, uptime | `status.version`, `status.uptime_s` | **Real** |
| **LiDAR Processing card** — raw points, filtered points, processing time | `ProcessingMetrics.input_point_count` / `output_point_count` / `duration_ms`, plus per-stage `duration_ms` (Phase 2C) | **Real** |
| System Status — latency | `SystemMetrics.latency_ms` | **Real** |
| System Status — CPU, memory | `SystemMetrics.cpu_percent`, `memory_mb` (psutil) | **Real** |
| System Status — FPS | `SystemMetrics.fps` — *ingest* frame rate, `null` until two frames arrive. Not the end-to-end system FPS the mockup implies | **Partial** |
| System Status — GPU usage | Deliberately never measured; always `null` | **None** |
| System Status — Risk Level | Needs tracked objects; only a proximity baseline exists and nothing feeds it | **None** |
| Point Cloud view — raw points | Points posted to `/api/v1/lidar/frame` can be rendered | **Partial** — real points, no boxes |
| 3D LiDAR view — object boxes | No detector | **None** — Phase 3 |
| Detected Objects — vehicles / pedestrians / bicycles / obstacles | No detector | **None** — Phase 3 |
| Object Tracking & Prediction — track IDs, speeds, confidence | No tracker | **None** — Phase 4 |
| Prediction Trajectories inset | No predictor | **None** — Phase 8 |
| Adaptive 2.5D Map — height field, resolution levels | Contracts exist; no mapper, no adaptive algorithm | **None** — Phases 5, 7 |
| Risk Map — critical/high/medium/low field | No spatial risk field | **None** — Phase 6 |
| Recent Events feed | No event recorder | **None** — Phase 10 |
| Perception card — objects detected, **detection accuracy**, inference time | No detector. Accuracy is *unmeasurable* regardless: no labelled dataset exists | **None** |
| Prediction card — trajectories, horizon, confidence | No predictor | **None** — Phase 8 |
| Routing / Decision / Planning cards | **No backend module, and none in the twelve-phase roadmap** — see gap below | **None** |
| Simulation / Map Info — map, weather, time, traffic | Needs a live CARLA session; the boundary connects but returns no world detail yet | **None** — Phase 9 |
| Scenario Manager | No scenario generator | **None** — Phase 10 |
| Logs view | Structured logging exists; no log-retrieval endpoint | **Partial** |

### The specific figures in the mockup

`FPS 42.3` · `GPU 61%` · `Detection Accuracy 96.3%` · `7 vehicles / 3 pedestrians / 1 bicycle`
· `Collision prob 78%` · `confidence 94% / 88% / 91% / 76%` · `Decision: Slow Down —
Pedestrian Ahead` · `Trajectories Predicted 8`

None of these can be produced today, and several name the exact categories
[`20-constraints.md`](knowledge-base/20-constraints.md) forbids inventing: FPS, accuracy,
GPU usage. Detection accuracy is the hardest of them — it needs labelled ground truth, which
the project has never had; the synthetic benchmark datasets carry
`ground_truth_available: false` precisely so this stays visible.

### Roadmap gap: Routing, Decision, Planning

Three bottom-row cards have no owning phase. They appear in the UI requirement list in
`CLAUDE.md` and in the mockup, but no phase from 1 to 12 builds a router, a decision layer
or a planner — ADAPT-X is scoped as a *perception* system. Either a phase should be added to
own them, or they should be dropped from the dashboard. Left as-is they can only ever be
decoration.

---

## Build rule

Every panel is either fed by real backend data or **visibly labelled as unavailable**. There
is no third option, and no placeholder numbers.

An unavailable panel keeps its full layout — title, icon, legend, footprint — so the design
reads as intended, and shows its state plainly:

> **Not implemented** — Object detection ships in Phase 3.

Panels fed by simulated, replayed or synthetic data carry that label continuously, not just
at load. Every payload already carries a `source` field for exactly this
(`live_sensor` / `simulation` / `replay` / `synthetic_test`); the dashboard surfaces it
rather than discarding it.

This is what `15_dashboard-ui.md` means by "an operational and explanatory interface, not a
decorative mockup". A console showing 96.3% accuracy from a system with no detector is not a
preview of ADAPT-X — it is a claim ADAPT-X cannot support, and it would be indistinguishable
from the real thing in a screenshot.

---

## Suggested build order

1. **Shell now** — full layout, navigation, theming, panel chrome, live WebSocket wiring.
   Populates header, System Status, LiDAR Processing and the point-cloud viewport from real
   data; every other panel renders its "not implemented" state. Delivers the look
   immediately and stays truthful.
2. **Fill per phase** — each phase turns its panels on as its backend module lands:
   Phase 3 → Detected Objects and 3D boxes; Phase 4 → Tracking; Phase 5/7 → Adaptive Map;
   Phase 6 → Risk Map and Risk Level; Phase 8 → Prediction; Phase 9 → Simulation Info;
   Phase 10 → Recent Events and Scenario Manager.
3. **Phase 12** — final integration, Export Report, performance views, polish.

Step 1 is buildable today. Steps 2 and 3 are gated on their backend modules, not on UI work.
