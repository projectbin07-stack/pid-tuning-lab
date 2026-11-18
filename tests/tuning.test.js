import test from "node:test";
import assert from "node:assert/strict";
import {
  cohenCoon,
  fopdtStep,
  fromIdeal,
  identifyLeastSquares,
  identifyTwoPoint,
  nelderMead,
  relayExperiment,
  simc,
  stepExperiment,
  tyreusLuyben,
  zieglerNicholsStep,
  zieglerNicholsUltimate,
} from "../src/core/tuning.js";
import { autotune, defaultSettings, evaluate, makeConfig } from "../src/core/lab.js";
import { pidSettings } from "../src/core/pid.js";
import { near, nearRel } from "./helpers.js";

/** @typedef {import("../src/core/plants.js").Plant} Plant */
/** @typedef {import("../src/core/sim.js").SimConfig} SimConfig */

/**
 * Test plant 1/(s+1)^3 written as three cascaded lags. Hand-derived:
 * phase -180 deg at w = sqrt(3), where |G| = 1/8, so Ku = 8 and
 * Pu = 2 pi / sqrt(3) = 3.628 s.
 * @type {Plant}
 */
const lag3 = {
  id: "lag3",
  name: "test",
  summary: "",
  equations: [],
  outputLabel: "y",
  inputLabel: "u",
  disturbanceLabel: "d",
  linearity: "linear",
  selfRegulating: true,
  params: [],
  defaults: /** @type {Plant["defaults"]} */ ({}),
  initialState: (y0) => Float64Array.of(y0, y0, y0),
  derivative: (x, u, d) => Float64Array.of(u + d - x[0], x[0] - x[1], x[1] - x[2]),
  output: (x) => x[2],
  deadTime: () => 0,
  steadyInput: (y) => y,
  transferFunction: () => ({ num: [1], den: [1, 3, 3, 1], delay: 0 }),
};

/** @returns {SimConfig} */
function lag3Config() {
  return {
    plant: lag3,
    params: {},
    pid: pidSettings({ sampleTime: 0.002 }),
    sensor: { noiseStd: 0, quantStep: 0, lagTau: 0 },
    dt: 0.002,
    substeps: 1,
    seed: 1,
    r0: 0,
    feedForward: 0,
  };
}

test("relay autotune recovers Ku = 8 and Pu = 2 pi / sqrt 3 of 1/(s+1)^3", () => {
  const res = relayExperiment(lag3Config(), { amplitude: 1, maxTime: 200 });
  // The describing-function approximation is accurate to a few percent for
  // a plant that filters the relay's harmonics this well.
  nearRel(res.pu, (2 * Math.PI) / Math.sqrt(3), 0.02, "Pu");
  nearRel(res.ku, 8, 0.06, "Ku");
  assert.equal(res.periods, 4);
});

test("relay with hysteresis lands where the describing function predicts", () => {
  // With hysteresis eps the oscillation sits where G(jw) has phase
  // -180 deg + asin(eps / a), not at the ultimate point; 4 d / (pi sqrt(a^2 - eps^2))
  // is then 1 / |G| at that frequency, which is below Ku.
  const res = relayExperiment(lag3Config(), { amplitude: 1, hysteresis: 0.02, maxTime: 200 });
  const w = (2 * Math.PI) / res.pu;
  const phase = (-3 * Math.atan(w) * 180) / Math.PI;
  near(
    phase,
    -180 + (Math.asin(0.02 / res.amplitude) * 180) / Math.PI,
    2,
    "phase at the oscillation",
  );
  nearRel(res.ku, (1 + w * w) ** 1.5, 0.03, "1/|G| at the oscillation");
  assert.ok(res.pu > (2 * Math.PI) / Math.sqrt(3));
  assert.ok(res.ku < 8);
});

test("relay experiment rejects bad input and reports a missing oscillation", () => {
  assert.throws(() => relayExperiment(lag3Config(), { amplitude: 0, maxTime: 10 }), /positive/);
  assert.throws(() => relayExperiment(lag3Config(), { amplitude: 1, maxTime: 2 }), /no sustained/);
});

test("closed-loop rules from Ku = 10, Pu = 2", () => {
  assert.deepEqual(zieglerNicholsUltimate(10, 2, "p"), fromIdeal(5, 0, 0));
  const pi = zieglerNicholsUltimate(10, 2, "pi");
  near(pi.kc, 4.5, 1e-12);
  near(pi.ti, 2 / 1.2, 1e-12);
  const pid = zieglerNicholsUltimate(10, 2);
  assert.deepEqual([pid.kp, pid.ti, pid.td, pid.ki, pid.kd], [6, 1, 0.25, 6, 1.5]);
  const tl = tyreusLuyben(10, 2);
  near(tl.kc, 10 / 2.2, 1e-12);
  near(tl.ti, 4.4, 1e-12);
  near(tl.td, 2 / 6.3, 1e-12);
  near(tyreusLuyben(10, 2, "pi").kc, 3.125, 1e-12);
  near(tyreusLuyben(10, 2, "p").kc, 5, 1e-12);
});

test("open-loop rules from K = 2, tau = 5, theta = 1", () => {
  const m = { K: 2, tau: 5, theta: 1 };
  // SIMC with tauc = theta: Kc = 5 / (2 * 2) = 1.25, Ti = min(5, 8) = 5.
  const s = simc(m);
  near(s.kc, 1.25, 1e-12);
  near(s.ti, 5, 1e-12);
  assert.equal(s.td, 0);
  near(simc(m, 3).kc, 5 / 8, 1e-12);
  near(simc(m, 3).ti, 5, 1e-12);
  // tauc = max(theta, tau / 10) = 1, so Ti = min(10, 4 * 1.5) = 6.
  near(simc({ K: 1, tau: 10, theta: 0.5 }).ti, 6, 1e-12);
  near(simc({ K: 1, tau: 1, theta: 0.5 }, 0.1).ti, 1, 1e-12);
  // No dead time: tauc floors at tau / 10.
  near(simc({ K: 1, tau: 1, theta: 0 }).kc, 10, 1e-12);
  // ZN reaction curve: a = K theta / tau = 0.4.
  const zn = zieglerNicholsStep(m);
  near(zn.kc, 1.2 / 0.4, 1e-12);
  near(zn.ti, 2, 1e-12);
  near(zn.td, 0.5, 1e-12);
  near(zieglerNicholsStep(m, "pi").kc, 0.9 / 0.4, 1e-12);
  near(zieglerNicholsStep(m, "p").kc, 1 / 0.4, 1e-12);
  // Cohen-Coon, r = theta / tau = 0.2.
  const cc = cohenCoon(m);
  near(cc.kc, (1 / (2 * 0.2)) * (4 / 3 + 0.05), 1e-12);
  near(cc.ti, (1 * (32 + 1.2)) / (13 + 1.6), 1e-12);
  near(cc.td, 4 / 11.4, 1e-12);
  near(cohenCoon(m, "pi").kc, 2.5 * (0.9 + 0.2 / 12), 1e-12);
  near(cohenCoon(m, "pi").ti, (30 + 0.6) / (9 + 4), 1e-12);
  near(cohenCoon(m, "p").kc, 2.5 * (1 + 0.2 / 3), 1e-12);
});

/**
 * @param {number} noise
 */
function fopdtExperiment(noise) {
  const s = {
    ...defaultSettings("fopdt"),
    params: { K: 1.5, tau: 4, theta: 1.5 },
    noiseStd: noise,
    seed: 3,
  };
  const { config } = makeConfig(s);
  return stepExperiment(config, { du: 1, duration: 40 });
}

test("FOPDT identification recovers known parameters from a clean step", () => {
  const exp = fopdtExperiment(0);
  const ls = identifyLeastSquares(exp);
  nearRel(ls.K, 1.5, 0.005, "LS K");
  nearRel(ls.tau, 4, 0.02, "LS tau");
  near(ls.theta, 1.5, 0.06, "LS theta"); // sampled at 50 ms
  const tp = identifyTwoPoint(exp);
  nearRel(tp.K, 1.5, 0.005, "two-point K");
  nearRel(tp.tau, 4, 0.03, "two-point tau");
  near(tp.theta, 1.5, 0.1, "two-point theta");
});

test("least squares stays accurate with measurement noise", () => {
  const exp = fopdtExperiment(0.03);
  const ls = identifyLeastSquares(exp);
  nearRel(ls.K, 1.5, 0.02, "LS K");
  nearRel(ls.tau, 4, 0.06, "LS tau");
  near(ls.theta, 1.5, 0.15, "LS theta");
  assert.ok(ls.rms < 0.05);
});

test("identification needs an output change", () => {
  const exp = { t: [0, 1, 2], y: [0, 0, 0], yTrue: [0, 0, 0], u0: 0, du: 0, y0: 0 };
  assert.throws(() => identifyTwoPoint(exp), /no output change/);
  assert.throws(() => identifyLeastSquares(exp), /no output change/);
});

test("fopdtStep and Nelder-Mead helpers", () => {
  near(fopdtStep({ K: 2, tau: 1, theta: 1 }, 2), 2 * (1 - Math.exp(-1)), 1e-15);
  assert.equal(fopdtStep({ K: 2, tau: 1, theta: 1 }, 0.5), 0);
  const q = nelderMead((x) => (x[0] - 3) ** 2 + 10 * (x[1] + 1) ** 2, [0, 0], [1, 1], 300);
  near(q[0], 3, 1e-4);
  near(q[1], -1, 1e-4);
});

test("SIMC autotune on the FOPDT plant gives a well-damped loop", () => {
  const res = autotune(defaultSettings("fopdt"), "simc");
  assert.equal(res.type, "pi");
  assert.ok(res.model);
  nearRel(res.model.K, 2, 0.01);
  nearRel(res.model.tau, 5, 0.03);
  assert.deepEqual(res.warnings, []);
  const e = evaluate(res.settings);
  assert.ok(e.step.overshoot < 10);
  assert.ok(e.margins && e.margins.phaseMargin > 45);
});

test("Ziegler-Nichols is more aggressive than Tyreus-Luyben on the same relay test", () => {
  const zn = autotune(defaultSettings("thermal"), "zn-relay");
  const tl = autotune(defaultSettings("thermal"), "tyreus-luyben");
  assert.ok(zn.relay && tl.relay);
  near(zn.relay.ku, tl.relay.ku, 1e-12);
  assert.ok(zn.gains.kp > tl.gains.kp);
  assert.ok(zn.gains.ti < tl.gains.ti);
});

test("step-based methods refuse an integrating plant", () => {
  assert.throws(() => autotune(defaultSettings("integrating"), "simc"), /self-regulating/);
  assert.throws(() => autotune(defaultSettings("fopdt"), "magic"), /unknown tuning method/);
});

test("autotune warns when the experiment does not fit the rule's assumptions", () => {
  const motorRelay = autotune(defaultSettings("motor"), "zn-relay");
  assert.ok(motorRelay.warnings.some((w) => /10 controller samples/.test(w)));
  const msdStep = autotune(defaultSettings("msd"), "simc");
  assert.ok(msdStep.warnings.some((w) => /fits the step response poorly/.test(w)));
  // Open loop, the holding torque for 90 degrees flips the arm over the top.
  const armStep = autotune(defaultSettings("arm"), "cohen-coon");
  assert.ok(armStep.warnings.some((w) => /still moving/.test(w)));
  const armRelay = autotune(defaultSettings("arm"), "zn-relay");
  assert.ok(armRelay.warnings.some((w) => /had not settled/.test(w)));
  const motorStep = autotune(defaultSettings("motor"), "zn-step");
  assert.ok(motorStep.warnings.some((w) => /theta\/tau/.test(w)));
});

test("relay autotune needs room around the operating point", () => {
  const s = { ...defaultSettings("thermal"), r1: 25 }; // needs u = 0, at the lower limit
  assert.throws(() => autotune(s, "zn-relay"), /no room/);
});
