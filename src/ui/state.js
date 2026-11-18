// URL parameters <-> lab settings, and small helpers for the controls.
// DOM-free so it can be tested under Node.

import { defaultSettings } from "../core/lab.js";
import { getPlant, plantIds, plantParams } from "../core/plants.js";

/** @typedef {import("../core/lab.js").LabSettings} LabSettings */

/**
 * URL key for each numeric setting.
 * @type {Array<[string, keyof LabSettings]>}
 */
export const NUMERIC_KEYS = [
  ["kp", "kp"],
  ["ki", "ki"],
  ["kd", "kd"],
  ["kc", "kc"],
  ["ti", "ti"],
  ["td", "td"],
  ["n", "n"],
  ["b", "b"],
  ["c", "c"],
  ["umin", "uMin"],
  ["umax", "uMax"],
  ["tt", "tt"],
  ["rate", "rateLimit"],
  ["ts", "sampleTime"],
  ["ff", "feedForward"],
  ["noise", "noiseStd"],
  ["quant", "quantStep"],
  ["lag", "lagTau"],
  ["seed", "seed"],
  ["sp", "r1"],
  ["dist", "disturbance"],
];

/**
 * @typedef {object} UrlState
 * @property {LabSettings} a the active controller
 * @property {LabSettings | null} b the pinned comparison controller
 * @property {number} run plant steps to simulate before the first frame
 * @property {boolean} paused
 */

/**
 * Compact number formatting for URLs: up to 6 significant digits.
 * @param {number} v
 * @returns {string}
 */
function num(v) {
  return String(Number(v.toPrecision(6)));
}

/**
 * Write the settings that differ from the plant defaults.
 * @param {URLSearchParams} q
 * @param {LabSettings} s
 * @param {string} prefix
 */
function writeSettings(q, s, prefix) {
  const d = defaultSettings(s.plant);
  if (s.form !== d.form) q.set(prefix + "form", s.form);
  if (s.antiWindup !== d.antiWindup) q.set(prefix + "aw", s.antiWindup);
  for (const [key, field] of NUMERIC_KEYS) {
    const v = /** @type {number} */ (s[field]);
    if (num(v) !== num(/** @type {number} */ (d[field]))) q.set(prefix + key, num(v));
  }
  for (const [k, v] of Object.entries(s.params)) q.set(`${prefix}p.${k}`, num(v));
}

/**
 * @param {UrlState} state
 * @returns {string} query string without the leading "?"
 */
export function encodeState(state) {
  const q = new URLSearchParams();
  q.set("plant", state.a.plant);
  writeSettings(q, state.a, "");
  if (state.b) {
    q.set("compare", "1");
    writeSettings(q, state.b, "B.");
  }
  if (state.run > 0) q.set("run", String(state.run));
  if (state.paused) q.set("paused", "1");
  return q.toString();
}

/**
 * Read settings with a prefix; invalid values fall back to the default.
 * @param {URLSearchParams} q
 * @param {string} plant
 * @param {string} prefix
 * @returns {LabSettings}
 */
function readSettings(q, plant, prefix) {
  const s = defaultSettings(plant);
  const form = q.get(prefix + "form");
  if (form === "parallel" || form === "ideal") s.form = form;
  const aw = q.get(prefix + "aw");
  if (aw === "none" || aw === "clamp" || aw === "backcalc") s.antiWindup = aw;
  for (const [key, field] of NUMERIC_KEYS) {
    const raw = q.get(prefix + key);
    if (raw === null || raw.trim() === "") continue;
    const v = Number(raw);
    if (Number.isFinite(v)) /** @type {Record<string, unknown>} */ (s)[field] = v;
  }
  if (!(s.uMax > s.uMin)) {
    const d = defaultSettings(plant);
    s.uMin = d.uMin;
    s.uMax = d.uMax;
  }
  const minTs = getPlant(plant).defaults.dt;
  if (!(s.sampleTime >= minTs) || s.sampleTime > 1000 * minTs) {
    s.sampleTime = defaultSettings(plant).sampleTime;
  }
  /** @type {Record<string, number>} */
  const params = {};
  const plantDef = getPlant(plant);
  for (const spec of plantDef.params) {
    const raw = q.get(`${prefix}p.${spec.key}`);
    if (raw === null) continue;
    const v = Number(raw);
    try {
      plantParams(plantDef, { [spec.key]: v });
      params[spec.key] = v;
    } catch {
      // out of range: keep the default
    }
  }
  s.params = params;
  return s;
}

/**
 * @param {string} search location.search, with or without "?"
 * @returns {UrlState}
 */
export function decodeState(search) {
  const q = new URLSearchParams(search);
  const p = q.get("plant");
  const plant = p !== null && plantIds().includes(p) ? p : "fopdt";
  const a = readSettings(q, plant, "");
  const b = q.get("compare") === "1" ? readSettings(q, plant, "B.") : null;
  const runRaw = Number(q.get("run") ?? 0);
  const run = Number.isInteger(runRaw) ? Math.min(Math.max(runRaw, 0), 200000) : 0;
  return { a, b, run, paused: q.get("paused") === "1" };
}

/**
 * Map a value to a 0..1000 slider position on a logarithmic scale.
 * Values at or below min (including 0) map to 0.
 * @param {number} value
 * @param {number} min > 0
 * @param {number} max
 * @returns {number}
 */
export function toSlider(value, min, max) {
  if (!(value > min)) return 0;
  if (value >= max) return 1000;
  return Math.round((1000 * Math.log(value / min)) / Math.log(max / min));
}

/**
 * Inverse of toSlider; position 0 means 0 (term switched off).
 * @param {number} pos
 * @param {number} min
 * @param {number} max
 * @returns {number}
 */
export function fromSlider(pos, min, max) {
  if (pos <= 0) return 0;
  return Number((min * (max / min) ** (pos / 1000)).toPrecision(3));
}

/**
 * Slider range for a gain: two decades either side of the plant default,
 * or around `fallback` when the default is zero.
 * @param {number} value
 * @param {number} fallback
 * @returns {[number, number]}
 */
export function gainRange(value, fallback) {
  const centre = value > 0 ? value : fallback;
  return [centre / 100, centre * 100];
}

/**
 * Human formatting for metric values.
 * @param {number} v
 * @param {string} [unit]
 * @returns {string}
 */
export function formatValue(v, unit = "") {
  if (Number.isNaN(v)) return "n/a";
  if (!Number.isFinite(v)) return "inf";
  const a = Math.abs(v);
  const s =
    a !== 0 && (a >= 1e5 || a < 1e-3) ? v.toExponential(2) : String(Number(v.toPrecision(3)));
  return unit ? `${s} ${unit}` : s;
}
