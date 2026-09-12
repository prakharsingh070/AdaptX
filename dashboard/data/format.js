// Presentation-only formatting. This module is the ONLY place a null or an
// UNKNOWN becomes text, and it never turns either into a number or a level:
// null -> "Not available", UNKNOWN -> "UNKNOWN". It performs no arithmetic on
// report values beyond unit conversion for display (bytes -> MB, ratio -> %).

export const NOT_AVAILABLE = "Not available";
export const NOT_IMPLEMENTED = "Not implemented";
export const NOT_MEASURED = "Not measured";

export function isMissing(value) {
  return value === null || value === undefined || Number.isNaN(value);
}

/** Format a number with a unit; missing values stay "Not available". */
export function fmt(value, { unit = "", digits = 2, missing = NOT_AVAILABLE, sign = false } = {}) {
  if (isMissing(value)) return missing;
  if (typeof value !== "number") return String(value);
  let text = Number.isInteger(value) && digits === 0 ? String(value) : value.toFixed(digits);
  if (sign && value > 0) text = `+${text}`;
  return unit ? `${text}${unit}` : text;
}

export function fmtInt(value, opts = {}) {
  return fmt(value, { ...opts, digits: 0 });
}

/** A ratio in [0,1] shown as a percentage. Null stays "Not available". */
export function fmtPercent(value, digits = 1) {
  if (isMissing(value)) return NOT_AVAILABLE;
  return `${(value * 100).toFixed(digits)}%`;
}

export function fmtBytesMB(bytes, digits = 2) {
  if (isMissing(bytes)) return NOT_AVAILABLE;
  return `${(bytes / (1024 * 1024)).toFixed(digits)} MB`;
}

export function fmtMs(value, digits = 1) {
  return fmt(value, { unit: " ms", digits });
}

export function fmtMetres(value, digits = 2) {
  return fmt(value, { unit: " m", digits });
}

export function fmtSeconds(value, digits = 2) {
  return fmt(value, { unit: " s", digits });
}

/** Risk level label. UNKNOWN is a level of its own and is never folded into LOW. */
export function levelLabel(level) {
  if (isMissing(level)) return NOT_AVAILABLE;
  return String(level).toUpperCase();
}

export function levelClass(level) {
  const known = ["low", "medium", "high", "critical", "unknown"];
  const key = String(level ?? "unknown").toLowerCase();
  return `lvl lvl-${known.includes(key) ? key : "unknown"}`;
}

/** Colour for a risk level; UNKNOWN is violet and dashed, never green. */
export function levelColour(level) {
  switch (String(level ?? "").toLowerCase()) {
    case "low": return "#34d399";
    case "medium": return "#fbbf24";
    case "high": return "#fb923c";
    case "critical": return "#ef4444";
    default: return "#a78bfa";
  }
}

export function resolutionColour(level) {
  switch (String(level ?? "").toLowerCase()) {
    case "low": return "rgba(56, 189, 248, 0.10)";
    case "medium": return "rgba(52, 211, 153, 0.22)";
    case "high": return "rgba(251, 191, 36, 0.32)";
    case "critical": return "rgba(239, 68, 68, 0.40)";
    default: return "rgba(255,255,255,0.05)";
  }
}

/** Summary text of a Phase 11 Distribution; count 0 -> "Not available (no samples)". */
export function fmtDistribution(d, unit = "", digits = 2) {
  if (!d || d.count === 0) return `${NOT_AVAILABLE} (no samples)`;
  const p95 = isMissing(d.p95) ? "n/a" : fmt(d.p95, { unit, digits });
  return `n=${d.count} · mean ${fmt(d.mean, { unit, digits })} · median ${fmt(d.median, { unit, digits })} · p95 ${p95} · max ${fmt(d.max, { unit, digits })}`;
}

export function fmtTime(iso) {
  if (isMissing(iso)) return NOT_AVAILABLE;
  const date = new Date(iso);
  return Number.isNaN(date.getTime()) ? String(iso) : date.toLocaleTimeString();
}

export function fmtVector(v, digits = 2) {
  if (isMissing(v)) return NOT_AVAILABLE;
  return `(${fmt(v.x, { digits })}, ${fmt(v.y, { digits })}, ${fmt(v.z, { digits })})`;
}

/** A glyph per object class, for cards and scene labels. UNKNOWN stays a question mark. */
export function classGlyph(objectClass) {
  switch (String(objectClass ?? "unknown").toLowerCase()) {
    case "vehicle": return "\u{1F697}";
    case "pedestrian": return "\u{1F6B6}";
    case "cyclist": return "\u{1F6B4}";
    case "obstacle": return "\u26A0";
    default: return "?";
  }
}

/** The class as the pipeline labelled it, upper-cased; UNKNOWN stays UNKNOWN. */
export function classLabel(objectClass) {
  return String(objectClass ?? "unknown").toUpperCase();
}

/** "IN PATH" / "CROSSING" / "BEHIND" / "OUTSIDE" from the backend's path relation. */
export function pathLabel(relation) {
  if (isMissing(relation)) return NOT_AVAILABLE;
  return String(relation).replace("_", " ");
}

/** Compact scene label: "#12 CYCLIST 14.8m HIGH", every part as received. */
export function objectLabel(record, track, level) {
  const cls = classLabel(track ? track.object_class : record?.object_class);
  const id = track ? track.track_id : record?.track_id;
  const distance = record && !isMissing(record.distance_m) ? ` ${fmt(record.distance_m, { digits: 1 })}m` : "";
  const risk = isMissing(level) ? "" : ` ${String(level).toUpperCase()}`;
  return `#${id} ${cls}${distance}${risk}`;
}
