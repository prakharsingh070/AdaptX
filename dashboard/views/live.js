// Live Scene: the viewport as the primary content, the inspector beside it,
// playback below when a recorded run is selected.

import { el, clear } from "../ui.js";
import { state, subscribe, currentScene } from "../app.js";
import { mountViewport } from "./viewport.js";
import { inspectorPanel } from "./inspector.js";
import { findObject } from "../data/normalise.js";

export function renderLive(root) {
  const grid = el("div", { class: "grid-live" });
  const viewportHost = el("div", { class: "viewport-host", style: "display:contents" });
  const inspector = el("div", { class: "card inspector" }, [el("h3", { text: "Object Inspector" })]);
  const body = el("div");
  inspector.append(body);
  grid.append(viewportHost, inspector);
  root.append(grid);
  const unmountViewport = mountViewport(viewportHost);

  function drawInspector() {
    clear(body);
    body.append(inspectorPanel(findObject(currentScene(), state.selectedTrackId), currentScene()));
  }
  drawInspector();
  const unsubscribe = subscribe((topic) => { if (["scene", "selection", "run"].includes(topic)) drawInspector(); });
  return () => { unsubscribe(); unmountViewport(); };
}
