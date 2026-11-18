// Browser app: DOM wiring, live simulation loop and drawing. Everything that
// computes something lives in src/core and is tested under Node.

import { PLANTS, getPlant } from "../core/plants.js";
import { autotune, defaultSettings, evaluate, makeConfig } from "../core/lab.js";
import { Simulation } from "../core/sim.js";
import { idealGains, parallelGains, pidSettings } from "../core/pid.js";
import { defaultRange, frequencyResponse, logspace, pidLoop } from "../core/frequency.js";
import { decodeState, encodeState, formatValue, fromSlider, gainRange, toSlider } from "./state.js";
import { drawBode, drawChart, drawNyquist, drawSchematic } from "./draw.js";

/** @typedef {import("../core/lab.js").LabSettings} LabSettings */
/** @typedef {import("../core/lab.js").Evaluation} Evaluation */
/** @typedef {import("./draw.js").Series} Series */
/** @typedef {import("./draw.js").FreqCurve} FreqCurve */

/**
 * @template {HTMLElement} T
 * @param {string} id
 * @returns {T}
 */
function $(id) {
  const el = document.getElementById(id);
  if (!el) throw new Error(`missing #${id}`);
  return /** @type {T} */ (el);
}

/** @param {string} id */
const input = (id) => /** @type {HTMLInputElement} */ ($(id));
/** @param {string} id */
const select = (id) => /** @type {HTMLSelectElement} */ ($(id));
/** @param {string} id */
const button = (id) => /** @type {HTMLButtonElement} */ ($(id));
/** @param {string} id */
const canvas = (id) => /** @type {HTMLCanvasElement} */ ($(id));

// ---------------------------------------------------------------- state

const initial = decodeState(location.search);
/** @type {LabSettings} */
let A = initial.a;
/** @type {LabSettings | null} */
let B = initial.b;
let paused = initial.paused;
let speed = 1;
let manual = false;

/**
 * One live simulation with its recorded history.
 * @typedef {object} Live
 * @property {Simulation} sim
 * @property {number[]} t
 * @property {number[]} r
 * @property {number[]} y
 * @property {number[]} ym
 * @property {number[]} u
 */

/** @type {Live | null} */
let liveA = null;
/** @type {Live | null} */
let liveB = null;
let stride = 1;
let stepCount = 0;
let scenarioStepDone = false;
let scenarioDistDone = false;
let distOn = false;
let noiseOn = A.noiseStd > 0;

// --------------------------------------------------------------- builders

const plantSel = select("plant");
for (const p of PLANTS) {
  const o = document.createElement("option");
  o.value = p.id;
  o.textContent = p.name;
  plantSel.append(o);
}

/**
 * Gain definitions for the current form: key, label, default-based range.
 * @param {LabSettings} s
 * @returns {Array<{key: "kp" | "ki" | "kd" | "kc" | "ti" | "td", label: string, title: string, range: [number, number]}>}
 */
function gainDefs(s) {
  const d = defaultSettings(s.plant);
  const dur = getPlant(s.plant).defaults.duration;
  if (s.form === "ideal") {
    return [
      { key: "kc", label: "Kc", title: "Controller gain", range: gainRange(d.kc, 1) },
      {
        key: "ti",
        label: "Ti (s)",
        title: "Integral time; 0 = off",
        range: gainRange(d.ti, dur / 10),
      },
      { key: "td", label: "Td (s)", title: "Derivative time", range: gainRange(d.td, dur / 100) },
    ];
  }
  return [
    { key: "kp", label: "Kp", title: "Proportional gain", range: gainRange(d.kp, 1) },
    {
      key: "ki",
      label: "Ki",
      title: "Integral gain (1/s)",
      range: gainRange(d.ki, (10 * d.kp) / dur),
    },
    {
      key: "kd",
      label: "Kd",
      title: "Derivative gain (s)",
      range: gainRange(d.kd, (d.kp * dur) / 100),
    },
  ];
}

function buildGains() {
  const box = $("gains");
  box.textContent = "";
  for (const g of gainDefs(A)) {
    const row = document.createElement("div");
    row.className = "gain";
    const label = document.createElement("label");
    label.htmlFor = `g-${g.key}`;
    label.textContent = g.label;
    label.title = g.title;
    const range = document.createElement("input");
    range.type = "range";
    range.min = "0";
    range.max = "1000";
    range.id = `g-${g.key}-range`;
    range.setAttribute("aria-label", `${g.label} slider (logarithmic, far left = 0)`);
    const num = document.createElement("input");
    num.type = "number";
    num.step = "any";
    num.min = "0";
    num.id = `g-${g.key}`;
    const sync = () => {
      range.value = String(toSlider(A[g.key], g.range[0], g.range[1]));
      num.value = String(Number(A[g.key].toPrecision(4)));
    };
    sync();
    range.addEventListener("input", () => {
      A[g.key] = fromSlider(Number(range.value), g.range[0], g.range[1]);
      num.value = String(A[g.key]);
      onControllerChange();
    });
    num.addEventListener("change", () => {
      const v = Number(num.value);
      if (Number.isFinite(v) && v >= 0) {
        A[g.key] = v;
        range.value = String(toSlider(v, g.range[0], g.range[1]));
        onControllerChange();
      } else sync();
    });
    row.append(label, range, num);
    box.append(row);
  }
  for (const r of /** @type {NodeListOf<HTMLInputElement>} */ (
    document.querySelectorAll('input[name="form"]')
  )) {
    r.checked = r.value === A.form;
  }
}

function buildPlant() {
  const plant = getPlant(A.plant);
  plantSel.value = plant.id;
  $("plant-summary").textContent = plant.summary;
  $("equations").textContent =
    plant.equations.join("\n") +
    `\n\ny: ${plant.outputLabel}\nu: ${plant.inputLabel}\nd: ${plant.disturbanceLabel}`;
  const box = $("params");
  box.textContent = "";
  for (const spec of plant.params) {
    const label = document.createElement("label");
    label.textContent = `${spec.label}${spec.unit ? ` (${spec.unit})` : ""}`;
    const inp = document.createElement("input");
    inp.type = "number";
    inp.step = "any";
    inp.min = String(spec.min);
    inp.max = String(spec.max);
    inp.value = String(A.params[spec.key] ?? spec.value);
    inp.addEventListener("change", () => {
      const v = Number(inp.value);
      if (Number.isFinite(v) && v >= spec.min && v <= spec.max) {
        A.params = { ...A.params, [spec.key]: v };
        if (v === spec.value) delete A.params[spec.key];
        if (B) B.params = { ...A.params };
        onStructuralChange();
      } else {
        inp.value = String(A.params[spec.key] ?? spec.value);
      }
    });
    label.append(inp);
    box.append(label);
  }
  $("quant-note").textContent = `Plant default quantisation ${formatValue(
    plant.defaults.quantStep,
  )}: ${plant.defaults.quantNote}. Default noise std ${formatValue(plant.defaults.noiseStd)}.`;
  drawSchematic(canvas("schematic"), plant.id);
}

/** Copy the numeric settings into the advanced and sensor inputs. */
function fillInputs() {
  /** @type {Array<[string, number]>} */
  const pairs = [
    ["n", A.n],
    ["ts", A.sampleTime],
    ["b", A.b],
    ["c", A.c],
    ["umin", A.uMin],
    ["umax", A.uMax],
    ["tt", A.tt],
    ["rate", A.rateLimit],
    ["ff", A.feedForward],
    ["lag", A.lagTau],
    ["seed", A.seed],
    ["sp", A.r1],
    ["dist", A.disturbance],
  ];
  for (const [id, v] of pairs) input(id).value = String(Number(v.toPrecision(6)));
  select("aw").value = A.antiWindup;
  const d = getPlant(A.plant).defaults;
  input("noise").value = String(A.noiseStd > 0 ? A.noiseStd : d.noiseStd);
  input("noise-on").checked = A.noiseStd > 0;
  input("quant").value = String(
    Number((A.quantStep > 0 ? A.quantStep : d.quantStep).toPrecision(4)),
  );
  input("quant-on").checked = A.quantStep > 0;
  noiseOn = A.noiseStd > 0;
  button("btn-noise").setAttribute("aria-pressed", String(noiseOn));
}

// ------------------------------------------------------------ live sim

/**
 * @param {LabSettings} s
 * @returns {Live}
 */
function makeLive(s) {
  const { config } = makeConfig(s);
  // Live runs start without noise unless it is switched on; the toggle
  // changes the sensor settings in place.
  return { sim: new Simulation(config), t: [], r: [], y: [], ym: [], u: [] };
}

function resetLive() {
  liveA = makeLive(A);
  liveB = B ? makeLive(B) : null;
  const d = getPlant(A.plant).defaults;
  stride = Math.max(1, Math.round(d.duration / d.dt / 1500));
  stepCount = 0;
  scenarioStepDone = false;
  scenarioDistDone = false;
  distOn = false;
  manual = false;
  button("btn-dist").setAttribute("aria-pressed", "false");
  button("btn-manual").setAttribute("aria-pressed", "false");
  input("manual-u").disabled = true;
}

/** @param {(sim: Simulation) => void} fn */
function eachSim(fn) {
  if (liveA) fn(liveA.sim);
  if (liveB) fn(liveB.sim);
}

/**
 * Advance both live simulations by n plant steps, applying the standard
 * scenario events (step, then disturbance) the first time round.
 * @param {number} n
 */
function advance(n) {
  if (!liveA) return;
  const d = getPlant(A.plant).defaults;
  const keep = d.duration;
  for (let k = 0; k < n; k++) {
    const t = liveA.sim.t;
    if (!scenarioStepDone && t >= d.stepTime - 1e-12) {
      scenarioStepDone = true;
      eachSim((s) => (s.r = A.r1));
    }
    if (!scenarioDistDone && t >= d.disturbanceTime - 1e-12) {
      scenarioDistDone = true;
      if (A.disturbance !== 0) setDisturbance(true);
    }
    for (const live of [liveA, liveB]) {
      if (!live) continue;
      const s = live.sim.step();
      if (stepCount % stride === 0) {
        live.t.push(s.t);
        live.r.push(s.r);
        live.y.push(s.y);
        live.ym.push(s.ym);
        live.u.push(s.u);
      }
    }
    stepCount++;
  }
  // Trim history older than one window.
  for (const live of [liveA, liveB]) {
    if (!live || live.t.length === 0) continue;
    const cut = live.t[live.t.length - 1] - keep;
    let i = 0;
    while (i < live.t.length && live.t[i] < cut) i++;
    if (i > 500) {
      for (const key of /** @type {const} */ (["t", "r", "y", "ym", "u"])) live[key].splice(0, i);
    }
  }
}

/** @param {boolean} on */
function setDisturbance(on) {
  distOn = on;
  eachSim((s) => (s.d = on ? A.disturbance : 0));
  button("btn-dist").setAttribute("aria-pressed", String(on));
}

/** @param {boolean} on */
function setNoise(on) {
  noiseOn = on;
  const std = Number(input("noise").value) || getPlant(A.plant).defaults.noiseStd;
  A.noiseStd = on ? std : 0;
  if (B) B.noiseStd = A.noiseStd;
  input("noise-on").checked = on;
  eachSim((s) => (s.config.sensor.noiseStd = A.noiseStd));
  button("btn-noise").setAttribute("aria-pressed", String(on));
  scheduleAnalysis();
}

/** @param {boolean} on */
function setManual(on) {
  manual = on;
  const slider = input("manual-u");
  if (liveA) {
    liveA.sim.pid.setManual(on);
    slider.value = String(
      Math.round(((liveA.sim.pid.manualOutput - A.uMin) / (A.uMax - A.uMin)) * 1000),
    );
  }
  if (liveB) liveB.sim.pid.setManual(on);
  slider.disabled = !on;
  button("btn-manual").setAttribute("aria-pressed", String(on));
}

function applyManualSlider() {
  const v = A.uMin + (Number(input("manual-u").value) / 1000) * (A.uMax - A.uMin);
  eachSim((s) => (s.pid.manualOutput = v));
}

function toggleStep() {
  if (!liveA) return;
  const d = getPlant(A.plant).defaults;
  scenarioStepDone = true;
  const target = Math.abs(liveA.sim.r - A.r1) < 1e-12 ? d.r0 : A.r1;
  eachSim((s) => (s.r = target));
}

// ----------------------------------------------------------- drawing

function drawLive() {
  if (!liveA) return;
  const d = getPlant(A.plant).defaults;
  const plant = getPlant(A.plant);
  const tNow = liveA.sim.t;
  const xMax = Math.max(d.duration, tNow);
  const xMin = xMax - d.duration;
  /** @type {Series[]} */
  const ys = [{ x: liveA.t, y: liveA.r, color: "--c-set", dash: [5, 4], width: 1.2 }];
  if (A.noiseStd > 0 || A.quantStep > 0 || A.lagTau > 0) {
    ys.push({ x: liveA.t, y: liveA.ym, color: "--c-meas", width: 1 });
  }
  ys.push({ x: liveA.t, y: liveA.y, color: "--c-a" });
  if (liveB) ys.push({ x: liveB.t, y: liveB.y, color: "--c-b", dash: [6, 3] });
  drawChart(canvas("c-y"), {
    series: ys,
    xMin,
    xMax,
    yMin: d.chartMin,
    yMax: Math.max(d.chartMax, A.r1 * 1.3),
    yLabel: `output ${plant.outputLabel}`,
  });
  /** @type {Series[]} */
  const us = [{ x: liveA.t, y: liveA.u, color: "--c-a" }];
  if (liveB) us.push({ x: liveB.t, y: liveB.u, color: "--c-b", dash: [6, 3] });
  const span = A.uMax - A.uMin;
  drawChart(canvas("c-u"), {
    series: us,
    xMin,
    xMax,
    yMin: A.uMin - 0.08 * span,
    yMax: A.uMax + 0.08 * span,
    yLabel: `control ${plant.inputLabel}`,
    hlines: [
      { y: A.uMin, color: "--c-limit" },
      { y: A.uMax, color: "--c-limit" },
    ],
  });
  const la = liveA;
  /** @type {Series[]} */
  const es = [{ x: la.t, y: la.r.map((r, i) => r - la.y[i]), color: "--c-a" }];
  if (liveB) {
    const lb = liveB;
    es.push({ x: lb.t, y: lb.r.map((r, i) => r - lb.y[i]), color: "--c-b", dash: [6, 3] });
  }
  drawChart(canvas("c-e"), { series: es, xMin, xMax, yLabel: "error r - y", xLabel: "time (s)" });
  $("live-time").textContent = `t = ${tNow.toFixed(tNow < 10 ? 3 : 2)} s`;
}

// ---------------------------------------------------------- analysis

const METRICS = /** @type {const} */ ([
  ["riseTime", "Rise time 10-90 %", "s"],
  ["settlingTime", "Settling time 2 %", "s"],
  ["overshoot", "Overshoot", "%"],
  ["steadyStateError", "Steady-state error", ""],
  ["iae", "IAE", ""],
  ["ise", "ISE", ""],
  ["itae", "ITAE", ""],
  ["effort", "Control effort, integral |u| dt", ""],
  ["totalVariation", "Total variation of u", ""],
  ["saturationTime", "Time at output limit", "s"],
]);

/** @type {Array<[string, string]>} */
const ROWS = [
  ...METRICS.map(([k, label]) => /** @type {[string, string]} */ ([k, label])),
  ["distPeak", "Disturbance: peak deviation"],
  ["distIae", "Disturbance: IAE"],
  ["distRecovery", "Disturbance: recovery 2 %"],
  ["gm", "Gain margin"],
  ["pm", "Phase margin"],
  ["wc", "Gain crossover"],
  ["dm", "Delay margin"],
];

function buildMetricsTable() {
  const body = /** @type {HTMLTableSectionElement} */ (document.querySelector("#metrics tbody"));
  body.textContent = "";
  for (const [key, label] of ROWS) {
    const tr = document.createElement("tr");
    const th = document.createElement("td");
    th.textContent = label;
    const a = document.createElement("td");
    a.id = `m-a-${key}`;
    const b = document.createElement("td");
    b.id = `m-b-${key}`;
    b.className = "col-b";
    tr.append(th, a, b);
    body.append(tr);
  }
}

/**
 * @param {Evaluation | null} e
 * @param {"a" | "b"} col
 */
function fillMetrics(e, col) {
  for (const [key] of ROWS) $(`m-${col}-${key}`).textContent = e ? "" : "-";
  if (!e) return;
  for (const [key, , unit] of METRICS)
    $(`m-${col}-${key}`).textContent = formatValue(e.step[key], unit);
  const dist = e.disturbance;
  $(`m-${col}-distPeak`).textContent = dist ? formatValue(dist.peakDeviation) : "-";
  $(`m-${col}-distIae`).textContent = dist ? formatValue(dist.iae) : "-";
  $(`m-${col}-distRecovery`).textContent = dist ? formatValue(dist.recoveryTime, "s") : "-";
  const m = e.margins;
  $(`m-${col}-gm`).textContent = m ? formatValue(m.gainMarginDb, "dB") : "-";
  $(`m-${col}-pm`).textContent = m ? formatValue(m.phaseMargin, "deg") : "-";
  $(`m-${col}-wc`).textContent = m ? formatValue(m.gainCrossover, "rad/s") : "-";
  $(`m-${col}-dm`).textContent = m ? formatValue(m.delayMargin, "s") : "-";
}

/**
 * Loop frequency response for the plots.
 * @param {LabSettings} s
 * @param {Evaluation} e
 * @param {string} color
 * @param {number[]} [dash]
 * @returns {FreqCurve}
 */
function freqCurve(s, e, color, dash) {
  const { config } = makeConfig(s);
  const tf = config.plant.transferFunction(config.params, s.r1);
  const T = Math.max(1, Math.round(s.sampleTime / config.dt)) * config.dt;
  const [w0, w1] = defaultRange(T);
  const fr = frequencyResponse(
    pidLoop(tf, { ...config.pid, sampleTime: T }, s.lagTau),
    logspace(w0, w1, 800),
  );
  return { ...fr, color, dash, margins: e.margins };
}

/** @type {number | undefined} */
let analysisTimer;
function scheduleAnalysis() {
  if (analysisTimer !== undefined) clearTimeout(analysisTimer);
  analysisTimer = window.setTimeout(runAnalysis, 120);
}

function runAnalysis() {
  analysisTimer = undefined;
  const plant = getPlant(A.plant);
  const d = plant.defaults;
  let eA;
  let eB = null;
  try {
    eA = evaluate(A);
    eB = B ? evaluate(B) : null;
  } catch (err) {
    $("scenario-note").textContent = `Cannot simulate: ${err instanceof Error ? err.message : err}`;
    return;
  }
  $("scenario-note").textContent =
    `Setpoint ${formatValue(d.r0)} to ${formatValue(A.r1)} at ${formatValue(d.stepTime)} s` +
    (A.disturbance !== 0
      ? `, load disturbance ${formatValue(A.disturbance)} (${plant.disturbanceLabel}) at ${formatValue(d.disturbanceTime)} s`
      : "") +
    `. Metrics use the true output; disturbance metrics start at the disturbance.`;
  /** @type {Series[]} */
  const ys = [
    { x: eA.trace.t, y: eA.trace.r, color: "--c-set", dash: [5, 4], width: 1.2 },
    { x: eA.trace.t, y: eA.trace.y, color: "--c-a" },
  ];
  /** @type {Series[]} */
  const us = [{ x: eA.trace.t, y: eA.trace.u, color: "--c-a" }];
  if (eB) {
    ys.push({ x: eB.trace.t, y: eB.trace.y, color: "--c-b", dash: [6, 3] });
    us.push({ x: eB.trace.t, y: eB.trace.u, color: "--c-b", dash: [6, 3] });
  }
  const markers = [{ x: d.stepTime, label: "step" }];
  if (A.disturbance !== 0) markers.push({ x: d.disturbanceTime, label: "disturbance" });
  drawChart(canvas("c-step-y"), {
    series: ys,
    xMin: 0,
    xMax: d.duration,
    yLabel: `output ${plant.outputLabel}`,
    markers,
  });
  drawChart(canvas("c-step-u"), {
    series: us,
    xMin: 0,
    xMax: d.duration,
    yLabel: `control ${plant.inputLabel}`,
    xLabel: "time (s)",
    markers,
  });
  fillMetrics(eA, "a");
  fillMetrics(eB, "b");
  $("metrics").classList.toggle("no-b", !eB);
  const curves = [freqCurve(A, eA, "--c-a")];
  if (B && eB) curves.push(freqCurve(B, eB, "--c-b", [6, 3]));
  drawBode(canvas("c-bode"), curves);
  drawNyquist(canvas("c-nyq"), curves);
  $("margin-note").textContent =
    `From the ${eA.marginNote}, with the zero-order hold and sample time included; ` +
    `feed-forward and setpoint weights do not change the loop.` +
    (plant.linearity === "nonlinear" ? " The arm's real behaviour changes with angle." : "");
  canvas("c-bode").setAttribute(
    "aria-label",
    `Bode plot. Gain margin ${formatValue(eA.margins?.gainMarginDb ?? NaN, "dB")}, phase margin ${formatValue(eA.margins?.phaseMargin ?? NaN, "degrees")}.`,
  );
  canvas("c-step-y").setAttribute(
    "aria-label",
    `Step test output. Overshoot ${formatValue(eA.step.overshoot, "%")}, settling time ${formatValue(eA.step.settlingTime, "s")}.`,
  );
}

// ----------------------------------------------------------- events

function updateUrl() {
  const qs = encodeState({ a: A, b: B, run: 0, paused: false });
  history.replaceState(null, "", `${location.pathname}?${qs}`);
}

/** Gains, limits, filter: change the running controllers without a restart. */
function onControllerChange() {
  if (A.form === "ideal") {
    const g = parallelGains(pidSettings({ ...A }));
    A.kp = g.kp;
    A.ki = g.ki;
    A.kd = g.kd;
  } else {
    const g = idealGains(A);
    A.kc = g.kc;
    A.ti = g.ti;
    A.td = Number.isNaN(g.td) ? 0 : g.td;
  }
  try {
    if (liveA) liveA.sim.pid.update(pidSettings({ ...A }), liveA.sim.r, liveA.sim.ym);
  } catch {
    // invalid combination (e.g. limits crossed): keep the old controller
  }
  updateUrl();
  scheduleAnalysis();
}

/** Plant, sample time, sensor or scenario: restart the live run. */
function onStructuralChange() {
  resetLive();
  updateUrl();
  scheduleAnalysis();
}

plantSel.addEventListener("change", () => {
  A = defaultSettings(plantSel.value);
  B = null;
  button("btn-clear-b").disabled = true;
  buildPlant();
  buildGains();
  fillInputs();
  onStructuralChange();
});

for (const r of /** @type {NodeListOf<HTMLInputElement>} */ (
  document.querySelectorAll('input[name="form"]')
)) {
  r.addEventListener("change", () => {
    if (!r.checked) return;
    A.form = r.value === "ideal" ? "ideal" : "parallel";
    buildGains();
    onControllerChange();
  });
}

/** @type {Array<[string, keyof LabSettings, boolean]>} id, field, structural */
const NUMERIC_INPUTS = [
  ["n", "n", false],
  ["b", "b", false],
  ["c", "c", false],
  ["umin", "uMin", false],
  ["umax", "uMax", false],
  ["tt", "tt", false],
  ["rate", "rateLimit", false],
  ["ts", "sampleTime", true],
  ["ff", "feedForward", true],
  ["lag", "lagTau", true],
  ["seed", "seed", true],
  ["sp", "r1", true],
  ["dist", "disturbance", true],
];
for (const [id, field, structural] of NUMERIC_INPUTS) {
  const el = input(id);
  el.addEventListener("change", () => {
    const v = Number(el.value);
    const trial = { ...A, [field]: v };
    let ok = Number.isFinite(v) && el.value.trim() !== "";
    if (ok) {
      try {
        makeConfig(trial);
      } catch {
        ok = false;
      }
    }
    if (!ok) {
      el.value = String(/** @type {number} */ (A[field]));
      return;
    }
    A = trial;
    if (B && structural && field !== "sampleTime" && field !== "feedForward") {
      B = { ...B, [field]: v };
    }
    if (structural) onStructuralChange();
    else onControllerChange();
  });
}

select("aw").addEventListener("change", () => {
  const v = select("aw").value;
  A.antiWindup = v === "none" || v === "clamp" ? v : "backcalc";
  onControllerChange();
});

input("noise-on").addEventListener("change", () => setNoise(input("noise-on").checked));
input("noise").addEventListener("change", () => {
  if (noiseOn) setNoise(true);
});
const applyQuant = () => {
  A.quantStep = input("quant-on").checked ? Math.max(0, Number(input("quant").value) || 0) : 0;
  if (B) B.quantStep = A.quantStep;
  eachSim((s) => (s.config.sensor.quantStep = A.quantStep));
  updateUrl();
  scheduleAnalysis();
};
input("quant-on").addEventListener("change", applyQuant);
input("quant").addEventListener("change", applyQuant);

button("btn-tune").addEventListener("click", () => {
  const out = $("tune-result");
  const method = select("tune-method").value;
  const typeRaw = select("tune-type").value;
  const type = typeRaw === "pi" || typeRaw === "p" ? typeRaw : "pid";
  try {
    const res = autotune(A, method, type);
    A = { ...res.settings, form: A.form };
    const lines = [];
    if (res.model) {
      lines.push(
        `FOPDT fit: K ${formatValue(res.model.K)}, tau ${formatValue(res.model.tau, "s")}, theta ${formatValue(res.model.theta, "s")}.`,
      );
    }
    if (res.relay) {
      lines.push(
        `Relay test: Ku ${formatValue(res.relay.ku)}, Pu ${formatValue(res.relay.pu, "s")}.`,
      );
    }
    lines.push(
      `${method} ${res.type.toUpperCase()}: Kp ${formatValue(res.gains.kp)}, Ki ${formatValue(res.gains.ki)}, Kd ${formatValue(res.gains.kd)}.`,
    );
    out.textContent = lines.join(" ");
    for (const w of res.warnings) {
      const p = document.createElement("p");
      p.className = "warn";
      p.textContent = w;
      out.append(p);
    }
    buildGains();
    onControllerChange();
  } catch (err) {
    out.textContent = err instanceof Error ? err.message : String(err);
  }
});

button("btn-run").addEventListener("click", () => setPaused(!paused));
button("btn-reset").addEventListener("click", () => onStructuralChange());
button("btn-step").addEventListener("click", toggleStep);
button("btn-dist").addEventListener("click", () => {
  scenarioDistDone = true;
  setDisturbance(!distOn);
});
button("btn-noise").addEventListener("click", () => setNoise(!noiseOn));
button("btn-manual").addEventListener("click", () => setManual(!manual));
input("manual-u").addEventListener("input", applyManualSlider);
select("speed").addEventListener("change", () => (speed = Number(select("speed").value) || 1));
button("btn-pin").addEventListener("click", pinB);
button("btn-clear-b").addEventListener("click", () => {
  B = null;
  button("btn-clear-b").disabled = true;
  onStructuralChange();
});
button("btn-link").addEventListener("click", async () => {
  const btn = button("btn-link");
  try {
    await navigator.clipboard.writeText(location.href);
    btn.textContent = "Link copied";
  } catch {
    btn.textContent = "Copy from the address bar";
  }
  setTimeout(() => (btn.textContent = "Copy link"), 2000);
});

function pinB() {
  B = structuredClone(A);
  button("btn-clear-b").disabled = false;
  onStructuralChange();
}

/** @param {boolean} p */
function setPaused(p) {
  paused = p;
  button("btn-run").textContent = paused ? "Run" : "Pause";
}

document.addEventListener("keydown", (ev) => {
  const t = /** @type {HTMLElement} */ (ev.target);
  if (ev.ctrlKey || ev.metaKey || ev.altKey) return;
  // Leave typing and native control keys alone.
  if (t.closest("input, select, textarea, summary")) return;
  if (t instanceof HTMLButtonElement && (ev.key === " " || ev.key === "Enter")) return;
  const key = ev.key.toLowerCase();
  if (key === " ") setPaused(!paused);
  else if (key === "r") onStructuralChange();
  else if (key === "s") toggleStep();
  else if (key === "d") {
    scenarioDistDone = true;
    setDisturbance(!distOn);
  } else if (key === "n") setNoise(!noiseOn);
  else if (key === "m") setManual(!manual);
  else if (key === "p") pinB();
  else return;
  ev.preventDefault();
});

// ------------------------------------------------------------- start

let last = performance.now();
function frame(/** @type {number} */ now) {
  const dtReal = Math.min(0.1, (now - last) / 1000);
  last = now;
  if (!paused && liveA) {
    const d = getPlant(A.plant).defaults;
    // One window of the plant's scenario takes about 10 s at x1.
    const simSeconds = (d.duration / 10) * speed * dtReal;
    advance(Math.max(1, Math.round(simSeconds / d.dt)));
  }
  drawLive();
  requestAnimationFrame(frame);
}

buildMetricsTable();
buildPlant();
buildGains();
fillInputs();
resetLive();
if (B) button("btn-clear-b").disabled = false;
setPaused(paused);
advance(initial.run);
drawLive();
runAnalysis();
window.addEventListener("resize", () => {
  drawSchematic(canvas("schematic"), A.plant);
  drawLive();
  scheduleAnalysis();
});
$("status").textContent = "ready";
requestAnimationFrame(frame);
