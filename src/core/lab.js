// One flat settings object shared by the browser app, the URL and the CLI,
// and the functions that turn it into a simulation, metrics and margins.

import { getPlant, plantParams } from "./plants.js";
import { idealGains, pidSettings } from "./pid.js";
import { runScenario } from "./sim.js";
import { disturbanceMetrics, stepMetrics } from "./metrics.js";
import { defaultRange, pidLoop, stabilityMargins } from "./frequency.js";
import {
  cohenCoon,
  identifyLeastSquares,
  identifyTwoPoint,
  relayExperiment,
  simc,
  stepExperiment,
  tyreusLuyben,
  zieglerNicholsStep,
  zieglerNicholsUltimate,
} from "./tuning.js";

/** @typedef {import("./pid.js").PidForm} PidForm */
/** @typedef {import("./pid.js").AntiWindup} AntiWindup */
/** @typedef {import("./sim.js").SimConfig} SimConfig */
/** @typedef {import("./sim.js").Scenario} Scenario */
/** @typedef {import("./sim.js").Trace} Trace */
/** @typedef {import("./metrics.js").StepMetrics} StepMetrics */
/** @typedef {import("./metrics.js").DisturbanceMetrics} DisturbanceMetrics */
/** @typedef {import("./frequency.js").Margins} Margins */
/** @typedef {import("./tuning.js").Fopdt} Fopdt */
/** @typedef {import("./tuning.js").Gains} Gains */
/** @typedef {import("./tuning.js").ControllerType} ControllerType */

/**
 * @typedef {object} LabSettings
 * @property {string} plant
 * @property {Record<string, number>} params overrides of the plant defaults
 * @property {PidForm} form
 * @property {number} kp
 * @property {number} ki
 * @property {number} kd
 * @property {number} kc
 * @property {number} ti
 * @property {number} td
 * @property {number} n
 * @property {number} b
 * @property {number} c
 * @property {number} uMin
 * @property {number} uMax
 * @property {AntiWindup} antiWindup
 * @property {number} tt
 * @property {number} rateLimit
 * @property {number} sampleTime
 * @property {number} feedForward
 * @property {number} noiseStd
 * @property {number} quantStep
 * @property {number} lagTau
 * @property {number} seed
 * @property {number} r1 setpoint after the step
 * @property {number} disturbance load disturbance size (0 = none)
 *
 * @typedef {object} Evaluation
 * @property {Trace} trace
 * @property {StepMetrics} step
 * @property {DisturbanceMetrics | null} disturbance
 * @property {Margins | null} margins null for a nonlinear plant
 * @property {string} marginNote how the linear model was obtained
 * @property {Scenario} scenario
 */

/**
 * @param {string} plantId
 * @returns {LabSettings}
 */
export function defaultSettings(plantId) {
  const plant = getPlant(plantId);
  const d = plant.defaults;
  const ideal = idealGains(d.gains);
  return {
    plant: plant.id,
    params: {},
    form: "parallel",
    ...d.gains,
    kc: ideal.kc,
    ti: ideal.ti,
    td: Number.isNaN(ideal.td) ? 0 : ideal.td,
    n: 10,
    b: 1,
    c: 0,
    uMin: d.uMin,
    uMax: d.uMax,
    antiWindup: "backcalc",
    tt: 0,
    rateLimit: 0,
    sampleTime: d.sampleTime,
    feedForward: 0,
    noiseStd: 0,
    quantStep: 0,
    lagTau: 0,
    seed: 1,
    r1: d.r1,
    disturbance: d.disturbance,
  };
}

/**
 * Build the simulation configuration and the standard step + disturbance
 * scenario for a set of lab settings.
 * @param {LabSettings} s
 * @returns {{config: SimConfig, scenario: Scenario}}
 */
export function makeConfig(s) {
  const plant = getPlant(s.plant);
  const params = plantParams(plant, s.params);
  const d = plant.defaults;
  const pid = pidSettings({
    form: s.form,
    kp: s.kp,
    ki: s.ki,
    kd: s.kd,
    kc: s.kc,
    ti: s.ti,
    td: s.td,
    n: s.n,
    b: s.b,
    c: s.c,
    uMin: s.uMin,
    uMax: s.uMax,
    antiWindup: s.antiWindup,
    tt: s.tt,
    rateLimit: s.rateLimit,
    sampleTime: s.sampleTime,
  });
  if (s.sampleTime < d.dt) throw new RangeError(`sample time must be at least ${d.dt} s`);
  return {
    config: {
      plant,
      params,
      pid,
      sensor: { noiseStd: s.noiseStd, quantStep: s.quantStep, lagTau: s.lagTau },
      dt: d.dt,
      substeps: d.substeps,
      seed: s.seed,
      r0: d.r0,
      feedForward: s.feedForward,
    },
    scenario: {
      duration: d.duration,
      r1: s.r1,
      stepTime: d.stepTime,
      disturbance: s.disturbance,
      disturbanceTime: d.disturbanceTime,
    },
  };
}

/**
 * Stability margins of the loop around the linear(ised) plant model.
 * @param {SimConfig} config
 * @param {number} operatingPoint
 * @returns {{margins: Margins | null, note: string}}
 */
export function loopMargins(config, operatingPoint) {
  const { plant, params, pid } = config;
  const tf = plant.transferFunction(params, operatingPoint);
  const T = Math.max(1, Math.round(pid.sampleTime / config.dt)) * config.dt;
  const effective = { ...pid, sampleTime: T };
  const [wMin, wMax] = defaultRange(T);
  const loop = pidLoop(tf, effective, config.sensor.lagTau);
  const margins = stabilityMargins(loop, wMin, wMax);
  const notes = {
    linear: "exact linear model",
    "linear-except-friction": "linear model without Coulomb friction",
    nonlinear: `linearised about y = ${operatingPoint.toPrecision(3)}`,
  };
  return { margins, note: notes[plant.linearity] };
}

/**
 * Run the standard scenario and compute every metric.
 * @param {LabSettings} s
 * @returns {Evaluation}
 */
export function evaluate(s) {
  const { config, scenario } = makeConfig(s);
  const trace = runScenario(config, scenario);
  const d = config.plant.defaults;
  const hasDist = scenario.disturbance !== 0 && scenario.disturbanceTime < scenario.duration;
  const step = stepMetrics(trace, {
    stepTime: scenario.stepTime,
    endTime: hasDist ? scenario.disturbanceTime : scenario.duration,
    r0: d.r0,
    r1: scenario.r1,
    uMin: s.uMin,
    uMax: s.uMax,
  });
  const disturbance = hasDist
    ? disturbanceMetrics(trace, {
        time: scenario.disturbanceTime,
        band: 0.02 * Math.abs(scenario.r1 - d.r0),
      })
    : null;
  const { margins, note } = loopMargins(config, scenario.r1);
  return { trace, step, disturbance, margins, marginNote: note, scenario };
}

/**
 * @typedef {object} TuneResult
 * @property {string} method
 * @property {ControllerType} type
 * @property {Fopdt | null} model identified FOPDT model (step-based methods)
 * @property {Fopdt | null} modelTwoPoint two-point estimate, for comparison
 * @property {{ku: number, pu: number, amplitude: number} | null} relay relay results (relay methods)
 * @property {Gains} gains
 * @property {LabSettings} settings the input settings with the new gains
 * @property {string[]} warnings reasons to distrust the result
 */

/**
 * Run the experiment a tuning method needs on the simulated plant and
 * apply the rule.
 * @param {LabSettings} s
 * @param {string} method one of TUNING_METHODS
 * @param {ControllerType} [type]
 * @returns {TuneResult}
 */
export function autotune(s, method, type = "pid") {
  const { config, scenario } = makeConfig(s);
  const plant = config.plant;
  const d = plant.defaults;
  /** @type {Gains} */
  let gains;
  /** @type {Fopdt | null} */
  let model = null;
  /** @type {Fopdt | null} */
  let modelTwoPoint = null;
  /** @type {{ku: number, pu: number, amplitude: number} | null} */
  let relay = null;
  /** @type {string[]} */
  const warnings = [];

  if (method === "zn-relay" || method === "tyreus-luyben") {
    // Oscillate around the target operating point.
    const u0 = plant.steadyInput(scenario.r1, config.params);
    const room = Math.min(s.uMax - u0, u0 - s.uMin);
    const amplitude = Math.min(0.2 * (s.uMax - s.uMin), room);
    if (!(amplitude > 0)) throw new RangeError("no room for a relay around the operating point");
    const res = relayExperiment(
      { ...config, r0: scenario.r1 },
      {
        amplitude,
        hysteresis: 3 * s.noiseStd + s.quantStep,
        maxTime: 20 * d.duration,
      },
    );
    relay = { ku: res.ku, pu: res.pu, amplitude: res.amplitude };
    if (!res.converged) {
      warnings.push(
        "The relay oscillation had not settled to a steady period when the test ended.",
      );
    }
    const T = Math.max(1, Math.round(s.sampleTime / config.dt)) * config.dt;
    if (res.pu < 10 * T) {
      warnings.push(
        `The relay oscillation period (${res.pu.toPrecision(3)} s) is under 10 controller samples. ` +
          "The plant has too little phase lag for a relay test, so the oscillation is set by the " +
          "sampling delay and the gains derived from it are likely to be too aggressive.",
      );
    }
    gains =
      method === "zn-relay"
        ? zieglerNicholsUltimate(res.ku, res.pu, type)
        : tyreusLuyben(res.ku, res.pu, type);
  } else if (method === "simc" || method === "cohen-coon" || method === "zn-step") {
    if (!plant.selfRegulating) {
      throw new RangeError(
        `${method} needs a self-regulating plant; "${plant.id}" integrates. Use zn-relay or tyreus-luyben.`,
      );
    }
    const du =
      plant.steadyInput(scenario.r1, config.params) - plant.steadyInput(d.r0, config.params);
    const exp = stepExperiment(config, { du, duration: d.duration });
    const ls = identifyLeastSquares(exp);
    model = { K: ls.K, tau: ls.tau, theta: ls.theta };
    modelTwoPoint = identifyTwoPoint(exp);
    const span = Math.max(...exp.y.map((v) => Math.abs(v - exp.y0)));
    const tail = exp.y.slice(Math.floor(exp.y.length * 0.9));
    if (Math.max(...tail) - Math.min(...tail) > 0.05 * span) {
      warnings.push(
        "The output was still moving at the end of the step test (no steady state), " +
          "so the identified gain and time constant are unreliable.",
      );
    }
    if (ls.rms > 0.03 * span) {
      warnings.push(
        `The FOPDT model fits the step response poorly (rms error ${((100 * ls.rms) / span).toFixed(1)} % ` +
          "of the step), so the tuning rule's assumptions do not hold for this plant.",
      );
    }
    if (method !== "simc" && model.theta < 0.05 * model.tau) {
      warnings.push(
        `theta/tau = ${(model.theta / model.tau).toPrecision(2)} is very small; this rule gives ` +
          "very high gains when the dead time is small compared with the time constant.",
      );
    }
    if (method === "simc") gains = simc(model);
    else if (method === "cohen-coon") gains = cohenCoon(model, type);
    else gains = zieglerNicholsStep(model, type);
  } else {
    throw new RangeError(`unknown tuning method "${method}"`);
  }

  const settings = {
    ...s,
    form: /** @type {PidForm} */ ("parallel"),
    kp: gains.kp,
    ki: gains.ki,
    kd: gains.kd,
    kc: gains.kc,
    ti: gains.ti,
    td: gains.td,
  };
  return {
    method,
    type: method === "simc" ? "pi" : type,
    model,
    modelTwoPoint,
    relay,
    gains,
    settings,
    warnings,
  };
}
