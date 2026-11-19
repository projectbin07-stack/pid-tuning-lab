// Headless command-line interface. Runs the same simulation code as the
// browser app, without a DOM, for scripting and reproducible numbers.

import { parseArgs } from "node:util";
import { PLANTS, getPlant } from "./core/plants.js";
import { autotune, defaultSettings, evaluate } from "./core/lab.js";
import { TUNING_METHODS } from "./core/tuning.js";
import { idealGains, parallelGains, pidSettings } from "./core/pid.js";

/** @typedef {import("./core/lab.js").LabSettings} LabSettings */
/** @typedef {import("./core/lab.js").Evaluation} Evaluation */

export const VERSION = "0.1.1";

/**
 * Everything the CLI touches outside itself, so tests can run it in memory.
 * @typedef {object} CliIo
 * @property {(text: string) => void} out
 * @property {(text: string) => void} err
 * @property {(path: string, data: string) => void} writeFile
 */

const HELP = `pid-tuning-lab ${VERSION}: discrete PID design on simulated plants

Usage:
  pid-tuning-lab plants
  pid-tuning-lab run  --plant <id> [controller, sensor and scenario options]
  pid-tuning-lab tune --plant <id> --method <method> [--type pid|pi|p] [--ku df|harmonic] [options]
  pid-tuning-lab --help | --version

run simulates a setpoint step followed by a load disturbance and prints
time-domain metrics and, from the linear(ised) model, stability margins.
tune runs the experiment the method needs on the simulated plant (an
open-loop step or a relay test), applies the tuning rule and evaluates
the result. Relay results are checked for validity (is the oscillation
set by the plant or by the controller's sampling?) and a warning explains
a failed check.

Plants: ${PLANTS.map((p) => p.id).join(", ")}
Methods: ${TUNING_METHODS.join(", ")}

Tuning:
  --type pid|pi|p          controller type for the rule (default pid; simc is PI)
  --ku df|harmonic         relay methods: Ku from the describing function
                           (default) or from the first Fourier harmonic

Controller (defaults depend on the plant):
  --kp, --ki, --kd <x>     parallel-form gains
  --form ideal --kc <x> --ti <s> --td <s>
                           ideal form instead (Ti <= 0 turns integral off)
  --n <x>                  derivative filter, Tf = Td / N (default 10, 0 = off)
  --b <x>, --c <x>         setpoint weights (default 1 and 0)
  --umin, --umax <x>       output limits
  --aw none|clamp|backcalc anti-windup (default backcalc)
  --tt <s>                 back-calculation tracking time (default auto)
  --rate <x>               output rate limit per second (default off)
  --ts <s>                 controller sample time
  --ff <0..1>              feed-forward weight on the plant's static inverse
Sensor:
  --noise <std|default>    white measurement noise (default 0)
  --quant <step|default>   quantisation step (default 0)
  --lag <s>                first-order sensor lag (default 0)
  --seed <n>               noise seed (default 1)
Scenario and plant:
  --setpoint <x>           setpoint after the step
  --disturbance <x>        load disturbance size (0 = none)
  --param <key=value>      override a plant parameter (repeatable)
Output:
  --json                   print JSON instead of text
  --csv <file>             (run) write t,r,y,ym,u,d samples to a CSV file
`;

class UsageError extends Error {}

/**
 * @param {string} value
 * @param {string} name
 * @returns {number}
 */
function num(value, name) {
  const v = Number(value);
  if (value.trim() === "" || !Number.isFinite(v))
    throw new UsageError(`--${name} must be a number`);
  return v;
}

const OPTIONS = /** @type {const} */ ({
  plant: { type: "string" },
  method: { type: "string" },
  type: { type: "string" },
  ku: { type: "string" },
  form: { type: "string" },
  kp: { type: "string" },
  ki: { type: "string" },
  kd: { type: "string" },
  kc: { type: "string" },
  ti: { type: "string" },
  td: { type: "string" },
  n: { type: "string" },
  b: { type: "string" },
  c: { type: "string" },
  umin: { type: "string" },
  umax: { type: "string" },
  aw: { type: "string" },
  tt: { type: "string" },
  rate: { type: "string" },
  ts: { type: "string" },
  ff: { type: "string" },
  noise: { type: "string" },
  quant: { type: "string" },
  lag: { type: "string" },
  seed: { type: "string" },
  setpoint: { type: "string" },
  disturbance: { type: "string" },
  param: { type: "string", multiple: true },
  json: { type: "boolean" },
  csv: { type: "string" },
  help: { type: "boolean", short: "h" },
  version: { type: "boolean", short: "v" },
});

/**
 * Turn parsed options into lab settings.
 * @param {Record<string, string | boolean | string[] | undefined>} v
 * @returns {LabSettings}
 */
export function settingsFromOptions(v) {
  const plantId = typeof v.plant === "string" ? v.plant : "fopdt";
  const plant = getPlant(plantId);
  const s = defaultSettings(plant.id);
  /** @type {Array<[string, keyof LabSettings]>} */
  const numeric = [
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
    ["lag", "lagTau"],
    ["seed", "seed"],
    ["setpoint", "r1"],
    ["disturbance", "disturbance"],
  ];
  /** @type {Record<string, unknown>} */
  const out = { ...s };
  for (const [opt, key] of numeric) {
    const value = v[opt];
    if (typeof value === "string") out[key] = num(value, opt);
  }
  if (typeof v.form === "string") {
    if (v.form !== "parallel" && v.form !== "ideal")
      throw new UsageError("--form must be parallel or ideal");
    out.form = v.form;
  } else if (["kc", "ti", "td"].some((k) => typeof v[k] === "string")) {
    out.form = "ideal";
  }
  if (out.form === "ideal" && !["kc", "ti", "td"].some((k) => typeof v[k] === "string")) {
    // Ideal form requested without ideal gains: convert the parallel ones.
    const g = idealGains(parallelGains(pidSettings({ ...s, ...out, form: "parallel" })));
    out.kc = g.kc;
    out.ti = g.ti;
    out.td = Number.isNaN(g.td) ? 0 : g.td;
  }
  if (typeof v.aw === "string") {
    if (!["none", "clamp", "backcalc"].includes(v.aw)) {
      throw new UsageError("--aw must be none, clamp or backcalc");
    }
    out.antiWindup = v.aw;
  }
  if (typeof v.noise === "string") {
    out.noiseStd = v.noise === "default" ? plant.defaults.noiseStd : num(v.noise, "noise");
  }
  if (typeof v.quant === "string") {
    out.quantStep = v.quant === "default" ? plant.defaults.quantStep : num(v.quant, "quant");
  }
  /** @type {Record<string, number>} */
  const params = {};
  for (const p of Array.isArray(v.param) ? v.param : []) {
    const m = /^([A-Za-z0-9_]+)=(.+)$/.exec(p);
    if (!m) throw new UsageError(`--param must look like key=value, got "${p}"`);
    params[m[1]] = num(m[2], `param ${m[1]}`);
  }
  out.params = params;
  return /** @type {LabSettings} */ (out);
}

/**
 * JSON with Infinity and NaN written as null, numbers rounded to 6
 * significant digits.
 * @param {unknown} value
 * @returns {string}
 */
export function toJson(value) {
  return JSON.stringify(
    value,
    (_k, v) => (typeof v === "number" ? (Number.isFinite(v) ? Number(v.toPrecision(6)) : null) : v),
    2,
  );
}

/**
 * @param {number} v
 * @param {number} [digits]
 * @returns {string}
 */
function fmt(v, digits = 4) {
  if (Number.isNaN(v)) return "n/a";
  if (!Number.isFinite(v)) return v > 0 ? "inf" : "-inf";
  if (v === 0) return "0";
  return Math.abs(v) >= 1e5 || Math.abs(v) < 1e-3
    ? v.toExponential(2)
    : String(Number(v.toPrecision(digits)));
}

/**
 * @param {LabSettings} s
 * @returns {string[]}
 */
function controllerLines(s) {
  const g = parallelGains(pidSettings({ ...s }));
  const i = idealGains(g);
  return [
    `controller   Kp ${fmt(g.kp)}  Ki ${fmt(g.ki)}  Kd ${fmt(g.kd)}` +
      `   (ideal: Kc ${fmt(i.kc)}, Ti ${i.ti > 0 ? fmt(i.ti) + " s" : "off"}, Td ${fmt(i.td)} s)`,
    `             N ${fmt(s.n)}, b ${fmt(s.b)}, c ${fmt(s.c)}, sample time ${fmt(s.sampleTime)} s, ` +
      `limits [${fmt(s.uMin)}, ${fmt(s.uMax)}], anti-windup ${s.antiWindup}` +
      (s.rateLimit > 0 ? `, rate limit ${fmt(s.rateLimit)}/s` : "") +
      (s.feedForward > 0 ? `, feed-forward ${fmt(s.feedForward)}` : ""),
  ];
}

/**
 * @param {LabSettings} s
 * @param {Evaluation} e
 * @returns {string}
 */
export function formatEvaluation(s, e) {
  const plant = getPlant(s.plant);
  const d = plant.defaults;
  const sc = e.scenario;
  const lines = [
    `plant        ${plant.name} (${plant.id})`,
    ...controllerLines(s),
    `sensor       noise ${fmt(s.noiseStd)}, quantisation ${fmt(s.quantStep)}, lag ${fmt(s.lagTau)} s, seed ${s.seed}`,
    `scenario     setpoint ${fmt(d.r0)} -> ${fmt(sc.r1)} at ${fmt(sc.stepTime)} s` +
      (sc.disturbance !== 0
        ? `, disturbance ${fmt(sc.disturbance)} at ${fmt(sc.disturbanceTime)} s`
        : "") +
      `, ${fmt(sc.duration)} s`,
    "step response",
    `  rise time 10-90 %     ${fmt(e.step.riseTime)} s`,
    `  settling time 2 %     ${fmt(e.step.settlingTime)} s`,
    `  overshoot             ${fmt(e.step.overshoot, 3)} %`,
    `  steady-state error    ${fmt(e.step.steadyStateError)}`,
    `  IAE / ISE / ITAE      ${fmt(e.step.iae)} / ${fmt(e.step.ise)} / ${fmt(e.step.itae)}`,
    `  effort integral |u|   ${fmt(e.step.effort)}`,
    `  total variation of u  ${fmt(e.step.totalVariation)}`,
    `  time saturated        ${fmt(e.step.saturationTime)} s`,
  ];
  if (e.disturbance) {
    lines.push(
      "load disturbance",
      `  peak deviation        ${fmt(e.disturbance.peakDeviation)}`,
      `  IAE                   ${fmt(e.disturbance.iae)}`,
      `  recovery time 2 %     ${fmt(e.disturbance.recoveryTime)} s`,
    );
  }
  if (e.margins) {
    const m = e.margins;
    lines.push(
      `stability margins (${e.marginNote})`,
      `  gain margin           ${fmt(m.gainMarginDb, 3)} dB at ${fmt(m.phaseCrossover)} rad/s`,
      `  phase margin          ${fmt(m.phaseMargin, 3)} deg at ${fmt(m.gainCrossover)} rad/s`,
      `  delay margin          ${fmt(m.delayMargin)} s`,
    );
  }
  return lines.join("\n") + "\n";
}

/**
 * @param {Evaluation} e
 * @returns {string}
 */
export function traceCsv(e) {
  const { t, r, y, ym, u, d } = e.trace;
  const rows = ["t,r,y,ym,u,d"];
  for (let k = 0; k < t.length; k++) {
    rows.push([t[k], r[k], y[k], ym[k], u[k], d[k]].map((v) => Number(v.toPrecision(8))).join(","));
  }
  return rows.join("\n") + "\n";
}

/**
 * @param {Evaluation} e
 */
function summary(e) {
  return {
    scenario: e.scenario,
    step: e.step,
    disturbance: e.disturbance,
    margins: e.margins,
    marginNote: e.marginNote,
  };
}

/**
 * Run the CLI. Returns the process exit code.
 * @param {string[]} argv
 * @param {CliIo} io
 * @returns {number}
 */
export function main(argv, io) {
  try {
    const { values, positionals } = parseArgs({
      args: argv,
      options: OPTIONS,
      allowPositionals: true,
    });
    if (values.version) {
      io.out(`${VERSION}\n`);
      return 0;
    }
    const command = positionals[0];
    if (values.help || !command) {
      io.out(HELP);
      return command || values.help ? 0 : 2;
    }
    if (positionals.length > 1) throw new UsageError(`unexpected argument "${positionals[1]}"`);

    if (command === "plants") {
      if (values.json) {
        io.out(
          toJson(
            PLANTS.map((p) => ({
              id: p.id,
              name: p.name,
              linearity: p.linearity,
              selfRegulating: p.selfRegulating,
              equations: p.equations,
              params: p.params,
            })),
          ) + "\n",
        );
      } else {
        for (const p of PLANTS) io.out(`${p.id.padEnd(12)} ${p.name.padEnd(38)} ${p.linearity}\n`);
      }
      return 0;
    }

    const settings = settingsFromOptions(values);

    if (command === "run") {
      const e = evaluate(settings);
      if (typeof values.csv === "string") io.writeFile(values.csv, traceCsv(e));
      if (values.json) io.out(toJson({ plant: settings.plant, settings, ...summary(e) }) + "\n");
      else io.out(formatEvaluation(settings, e));
      return 0;
    }

    if (command === "tune") {
      if (typeof values.method !== "string") throw new UsageError("tune needs --method");
      if (!TUNING_METHODS.includes(values.method)) {
        throw new UsageError(`--method must be one of ${TUNING_METHODS.join(", ")}`);
      }
      const type = values.type ?? "pid";
      if (type !== "pid" && type !== "pi" && type !== "p")
        throw new UsageError("--type must be pid, pi or p");
      const kuOption = values.ku ?? "df";
      if (kuOption !== "df" && kuOption !== "harmonic")
        throw new UsageError("--ku must be df or harmonic");
      const res = autotune(settings, values.method, type, {
        kuEstimate: kuOption === "harmonic" ? "harmonic" : "describing-function",
      });
      const e = evaluate(res.settings);
      if (values.json) {
        io.out(
          toJson({
            plant: settings.plant,
            method: res.method,
            type: res.type,
            model: res.model,
            modelTwoPoint: res.modelTwoPoint,
            relay: res.relay,
            gains: res.gains,
            warnings: res.warnings,
            notes: res.notes,
            ...summary(e),
          }) + "\n",
        );
      } else {
        const lines = [`method       ${res.method} (${res.type.toUpperCase()})`];
        if (res.model && res.modelTwoPoint) {
          const m = res.model;
          const t = res.modelTwoPoint;
          lines.push(
            `FOPDT fit    least squares: K ${fmt(m.K)}, tau ${fmt(m.tau)} s, theta ${fmt(m.theta)} s`,
            `             two-point:     K ${fmt(t.K)}, tau ${fmt(t.tau)} s, theta ${fmt(t.theta)} s`,
          );
        }
        if (res.relay) {
          const r = res.relay;
          const v = r.validity;
          const used = r.kuEstimate === "harmonic" ? "first harmonic" : "describing function";
          const shift = Number.isFinite(v.delaySensitivity)
            ? `${(100 * v.delaySensitivity).toFixed(1)} %`
            : "stopped the oscillation";
          lines.push(
            `relay test   Ku ${fmt(r.ku)}, Pu ${fmt(r.pu)} s (output amplitude ${fmt(r.amplitude)})`,
            `             Ku by describing function ${fmt(r.kuDescribingFunction)}, by first harmonic ${fmt(r.kuHarmonic)}; the rule uses the ${used}`,
            `validity     ${v.valid ? "passed" : "FAILED"}: ${fmt(v.samplesPerPeriod)} samples per period, one extra sample of delay changes Pu by ${shift}`,
          );
        }
        for (const n of res.notes) lines.push(`note         ${n}`);
        for (const w of res.warnings) lines.push(`warning      ${w}`);
        io.out(lines.join("\n") + "\n" + formatEvaluation(res.settings, e));
      }
      return 0;
    }
    throw new UsageError(`unknown command "${command}"`);
  } catch (error) {
    const message = error instanceof Error ? error.message : String(error);
    io.err(`pid-tuning-lab: ${message}\n`);
    if (error instanceof UsageError || (error instanceof Error && "code" in error)) {
      io.err("Run with --help for usage.\n");
      return 2;
    }
    return 1;
  }
}
