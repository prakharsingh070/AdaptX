// Tiny DOM helpers. No templates, no framework.

import { NOT_AVAILABLE, isMissing } from "./data/format.js";

export function el(tag, attrs = {}, children = []) {
  const node = document.createElement(tag);
  for (const [key, value] of Object.entries(attrs)) {
    if (key === "class") node.className = value;
    else if (key === "text") node.textContent = value;
    else if (key === "html") node.innerHTML = value;
    else if (key.startsWith("on") && typeof value === "function") node.addEventListener(key.slice(2), value);
    else if (value !== null && value !== undefined) node.setAttribute(key, value);
  }
  for (const child of [].concat(children)) {
    if (child === null || child === undefined) continue;
    node.append(typeof child === "string" ? document.createTextNode(child) : child);
  }
  return node;
}

export function card(title, body, { tag = null, cls = "" } = {}) {
  const heading = el("h3", {}, [title]);
  if (tag) heading.append(el("span", { class: `tag ${tag.cls ?? ""}`, text: tag.text }));
  return el("div", { class: `card ${cls}` }, [heading, ...[].concat(body)]);
}

/** Definition list of label/value pairs. A missing value renders italic "Not available". */
export function kv(pairs) {
  const list = el("dl", { class: "kv" });
  for (const [label, value, opts = {}] of pairs) {
    list.append(el("dt", { text: label }));
    const missing = isMissing(value) || value === NOT_AVAILABLE;
    const dd = el("dd", { class: missing ? "na" : "" });
    if (value instanceof Node) dd.append(value);
    else dd.textContent = missing ? (opts.missing ?? NOT_AVAILABLE) : String(value);
    list.append(dd);
  }
  return list;
}

export function notice(text, kind = "") {
  return el("div", { class: `notice ${kind}`, text });
}

export function notImplemented(what, reason) {
  return el("div", { class: "not-implemented" }, [`${what}: not implemented`, reason ? ` — ${reason}` : ""]);
}

export function table(headers, rows, { onRow = null, selectedIndex = -1 } = {}) {
  const thead = el("thead", {}, [el("tr", {}, headers.map((h) => el("th", { text: h })))]);
  const tbody = el("tbody");
  rows.forEach((row, index) => {
    const tr = el("tr", { class: `${onRow ? "selectable" : ""} ${index === selectedIndex ? "selected" : ""}` });
    for (const cell of row) {
      const td = el("td", {});
      if (cell && typeof cell === "object" && "num" in cell) { td.className = "num"; td.textContent = cell.num; }
      else if (cell instanceof Node) td.append(cell);
      else td.textContent = cell === null || cell === undefined ? NOT_AVAILABLE : String(cell);
      tr.append(td);
    }
    if (onRow) tr.addEventListener("click", () => onRow(index));
    tbody.append(tr);
  });
  return el("table", { class: "data" }, [thead, tbody]);
}

export function clear(node) {
  while (node.firstChild) node.removeChild(node.firstChild);
  return node;
}

export function levelTag(level) {
  const key = String(level ?? "unknown").toLowerCase();
  return el("span", { class: `lvl lvl-${["low", "medium", "high", "critical"].includes(key) ? key : "unknown"}`, text: isMissing(level) ? "UNKNOWN" : String(level).toUpperCase() });
}
