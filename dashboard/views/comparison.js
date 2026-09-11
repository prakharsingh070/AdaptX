// Fixed vs Adaptive: dedicated charts of the paired figures a stored report
// holds, plus two-report comparison through the backend's Phase 11
// `compare` contract (never a second algorithm here).

import { el, card, kv, notice, clear, table } from "../ui.js";
import { state, subscribe } from "../app.js";
import { api } from "../data/api.js";
import { fmt, fmtInt, fmtDistribution } from "../data/format.js";
import { pairedChart, barChart } from "../render/charts.js";

function pairedCharts(r) {
  const a = r.adaptive_resolution, m = r.mapping;
  const c1 = el("canvas", { class: "chart" }), c2 = el("canvas", { class: "chart" }), c3 = el("canvas", { class: "chart" });
  const container = card("Paired over the run (means of per-frame values in the report)", [
    el("div", { class: "chart-title", text: "Cells and bytes — mean per frame" }), c1,
    el("div", { class: "chart-title", text: "Build time — median per frame (ms)" }), c2,
    el("div", { class: "chart-title", text: "Resolution at the actor's tile vs elsewhere — mean cell size (m); smaller is finer" }), c3,
    el("div", { class: "note", text: `Ratios as the report states them: cells ${fmtDistribution(a.cell_ratio, "", 3)}; bytes ${fmtDistribution(a.byte_ratio, "", 3)}; build time (mapping only) ${fmtDistribution(a.mapping_duration_ratio, "×", 2)}; with controller ${fmtDistribution(a.total_duration_ratio, "×", 2)}.` }),
  ], { cls: "full" });
  requestAnimationFrame(() => {
    pairedChart(c1, {
      title: "Cells and grid bytes", unit: "mean per frame", source: "report.adaptive_resolution / report.mapping",
      categories: [
        { label: "cells", fixed: a.fixed_cells.mean, adaptive: a.adaptive_cells.mean, format: (v) => fmtInt(v) },
        { label: "grid bytes", fixed: m.fixed ? m.fixed.grid_bytes.mean : null, adaptive: m.adaptive ? m.adaptive.grid_bytes.mean : null, format: (v) => fmtInt(v) },
        { label: "occupied cells", fixed: m.fixed ? m.fixed.occupied_cells.mean : null, adaptive: m.adaptive ? m.adaptive.occupied_cells.mean : null, format: (v) => fmtInt(v) },
      ],
    });
    pairedChart(c2, {
      title: "Build time", unit: "ms, median", source: "report.mapping.*.duration_ms, report.resource.stage_ms",
      categories: [
        { label: "grid build", fixed: m.fixed ? m.fixed.duration_ms.median : null, adaptive: m.adaptive ? m.adaptive.duration_ms.median : null, format: (v) => fmt(v, { digits: 1 }) },
        { label: "controller", fixed: null, adaptive: r.resource.stage_ms.controller ? r.resource.stage_ms.controller.median : null, format: (v) => fmt(v, { digits: 1 }) },
      ],
    });
    barChart(c3, {
      title: "Mean cell size", unit: "m", source: "report.adaptive_resolution.actor_tile_resolution_m / other_tile_resolution_m / fixed_resolution_m",
      items: [
        { label: "fixed (uniform)", value: r.mapping.fixed_resolution_m, colour: "#94a3b8", format: (v) => fmt(v, { digits: 3 }) },
        { label: "adaptive: actor tile", value: a.actor_tile_resolution_m.mean, colour: "#38bdf8", format: (v) => fmt(v, { digits: 3 }) },
        { label: "adaptive: other tiles", value: a.other_tile_resolution_m.mean, colour: "#22d3ee", format: (v) => fmt(v, { digits: 3 }) },
        ...Object.entries(a.actor_tile_resolution_by_risk_m).map(([k, d]) => ({ label: `actor tile at risk ${k}`, value: d.mean, colour: "#fbbf24", format: (v) => fmt(v, { digits: 3 }) })),
      ],
    });
  });
  return container;
}

function refinementAndChurn(r) {
  const a = r.adaptive_resolution, c = a.churn;
  return card("Refinement timing and churn", [
    kv([
      ["refinement lead (frames; + = refined before the actor arrived)", fmtDistribution(a.refinement_lead_frames, "", 1)],
      ["entries never refined", a.entries_never_refined],
      ["changed tiles per frame", c ? fmtDistribution(c.changed_tiles_per_frame, "", 2) : null],
      ["frames with any change", c ? `${c.frames_with_any_change} / ${c.frames}` : null],
      ["transitions / tiles ever changed", c ? `${c.total_transitions} / ${c.tiles_ever_changed}` : null],
      ["reversals (back within the dwell window)", c ? `${c.reversals} (window ${c.min_dwell_frames})` : null],
      ["hysteresis holds / dwell holds", c ? `${c.hysteresis_holds} / ${c.dwell_holds}` : null],
      ["frames over budget / demoted tiles", `${a.frames_over_budget} / ${a.demoted_tiles_total}`],
    ]),
    table(["actor", "tile", "entered", "refined", "lead"], (a.refinements ?? []).map((x) => [x.actor_id, { num: x.tile_index }, { num: x.entered_frame }, { num: x.refined_frame ?? "never" }, { num: x.lead_frames ?? "never" }])),
    el("div", { class: "note", text: "Whether a hold or a lead was beneficial is not measured; these are counts as the controller reported them." }),
  ], { cls: "full" });
}

function twoReportCompare(root) {
  const first = el("select"), second = el("select");
  const out = el("div");
  const button = el("button", { class: "playback", text: "Compare with the Phase 11 contract", onclick: async () => {
    clear(out);
    if (!first.value || !second.value) { out.append(notice("Choose two reports.")); return; }
    try {
      const c = await api.compareReports(first.value, second.value);
      out.append(kv([
        ["same scenario / seed / timestep", String(c.same_scenario)], ["same evaluation configuration", String(c.same_configuration)],
        ["deterministic content identical", String(c.deterministic_content_identical)],
        ["repeatable", String(c.same_scenario && c.same_configuration && c.deterministic_content_identical)],
        ["timing fields ignored", c.timing_fields_ignored.length],
      ]));
      out.append(c.differences.length ? table(["difference"], c.differences.map((d) => [d])) : notice("No deterministic-content difference."));
    } catch (error) {
      out.append(notice(`Comparison failed: ${error.message}`, "error"));
    }
  } });
  api.reports().then((list) => {
    for (const s of [first, second]) { s.append(new Option("— choose —", "")); for (const f of list.files) s.append(new Option(f.name, f.name)); }
  }).catch(() => {});
  root.append(card("Compare two stored reports (backend GET /api/v1/reports/compare — the CLI's own comparison)", [
    el("div", { style: "display:flex;gap:8px;align-items:center;flex-wrap:wrap" }, ["A", first, "B", second, button]),
    out,
  ], { cls: "full" }));
}

export function renderComparison(root) {
  const grid = el("div", { class: "grid-eval" });
  root.append(grid);
  function draw() {
    clear(grid);
    const r = state.reports.report;
    if (!r) { grid.append(el("div", { class: "full" }, [notice("No evaluation report loaded. Choose one in the rail.")])); }
    else {
      grid.append(card(`${r.scenario_id} · seed ${r.seed} · ${r.simulator_version} · ${r.frame_count} frames`, el("div", { class: "note", text: "Both maps were built from the same processed frame of the same run; only the resolution policy differs (ADR-053)." }), { tag: { text: "STORED EVALUATION", cls: "stored" }, cls: "full" }));
      grid.append(pairedCharts(r), refinementAndChurn(r));
    }
    twoReportCompare(grid);
  }
  draw();
  return subscribe((topic) => { if (topic === "report") draw(); });
}
