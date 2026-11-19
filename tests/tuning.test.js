import test from "node:test";
import assert from "node:assert/strict";
import {
  cohenCoon,
  fopdtStep,
  fromIdeal,
  identifyLeastSquares,
  identifyTwoPoint,
  nelderMead,
  RELAY_LIMITS,
  relayExperiment,
  relayValidity,
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
  const msdStep = autotune(defaultSettings("msd"), "simc");
  assert.ok(msdStep.warnings.some((w) => /fits the step response poorly/.test(w)));
  // Open loop, the holding torque for 90 degrees flips the arm over the top.
  const armStep = autotune(defaultSettings("arm"), "cohen-coon");
  assert.ok(armStep.warnings.some((w) => /still moving/.test(w)));
  const motorStep = autotune(defaultSettings("motor"), "zn-step");
  assert.ok(motorStep.warnings.some((w) => /theta\/tau/.test(w)));
});

test("relay autotune needs room around the operating point", () => {
  const s = { ...defaultSettings("thermal"), r1: 25 }; // needs u = 0, at the lower limit
  assert.throws(() => autotune(s, "zn-relay"), /no room/);
});

// --- Relay validity test -------------------------------------------------

/**
 * @param {string} plant
 * @param {number} [sampleTime]
 * @param {string} [method]
 */
function relayTune(plant, sampleTime, method = "zn-relay") {
  const s = defaultSettings(plant);
  if (sampleTime !== undefined) s.sampleTime = sampleTime;
  const res = autotune(s, method);
  assert.ok(res.relay);
  return { res, relay: res.relay, v: res.relay.validity };
}

test("relay validity passes on the hotend and FOPDT plants at default settings", () => {
  for (const plant of ["thermal", "fopdt"]) {
    for (const method of ["zn-relay", "tyreus-luyben"]) {
      const { res, v } = relayTune(plant, undefined, method);
      assert.equal(v.valid, true, `${plant} ${method}: ${v.failures.join("; ")}`);
      assert.deepEqual(v.failures, []);
      assert.deepEqual(res.warnings, []);
      assert.ok(v.samplesPerPeriod >= 10);
      assert.ok(v.delaySensitivity < 0.1, `${plant}: ${v.delaySensitivity}`);
      assert.ok(v.converged);
    }
  }
});

test("relay validity fails on motor, mass-spring-damper, integrating axis and arm", () => {
  // None of these models reaches -180 deg of phase on its own (at most two
  // lags, no dead time), so the limit cycle needs the controller's delay.
  for (const plant of ["motor", "msd", "integrating", "arm"]) {
    const { res, relay, v } = relayTune(plant);
    assert.equal(v.valid, false, plant);
    // The general criterion catches every one of them: one sample of extra
    // delay changes the period by far more than 25 %.
    assert.ok(v.delaySensitivity > 0.5, `${plant}: ${v.delaySensitivity}`);
    assert.ok(
      v.failures.some((f) => /one extra sample of loop delay/.test(f)),
      plant,
    );
    assert.equal(res.warnings.length, 1, plant);
    assert.match(res.warnings[0], /failed its validity check/);
    assert.match(res.warnings[0], /sampling and loop delay rather than by the plant/);
    assert.match(res.warnings[0], /not trustworthy/);
    assert.match(
      res.warnings[0],
      plant === "integrating"
        ? /do not apply here/
        : /Use a step-test method \(SIMC or Cohen-Coon\)/,
    );
    assert.deepEqual(res.notes, [], plant);
    assert.ok(relay.ku > 0);
  }
  // Motor: the period is 4 samples, which the samples criterion also flags.
  assert.ok(relayTune("motor").v.failures.some((f) => /only 4\.0 controller samples/.test(f)));
  // Arm: nonlinear, the cycle also had not settled.
  const arm = relayTune("arm");
  assert.equal(arm.v.converged, false);
  assert.match(arm.res.warnings[0], /not steady and symmetric/);
});

test("the relay validity result moves with the sample time in the expected direction", () => {
  // FOPDT: a longer sample time is a larger share of the loop's delay, so
  // one more sample moves the period more; at 0.3 s (30 % of the dead time)
  // the test fails.
  const sens = [0.03, 0.05, 0.2].map((ts) => relayTune("fopdt", ts).v.delaySensitivity);
  assert.ok(sens[0] < sens[1] && sens[1] < sens[2], sens.join(", "));
  const slow = relayTune("fopdt", 0.3);
  assert.equal(slow.v.valid, false);
  assert.ok(slow.v.delaySensitivity > 0.25);
  assert.ok(
    relayTune("thermal", 0.05).v.delaySensitivity < relayTune("thermal", 0.4).v.delaySensitivity,
  );
  assert.equal(relayTune("thermal", 1).v.valid, false);
  // Motor: the period is set by the sampling, so it scales with the sample
  // time and a faster controller does not make the test valid.
  const fast = relayTune("motor", 0.0005);
  const slowMotor = relayTune("motor", 0.002);
  nearRel(slowMotor.relay.pu / fast.relay.pu, 4, 1e-9, "Pu ratio");
  assert.equal(fast.v.valid, false);
  assert.equal(slowMotor.v.valid, false);
  // Mass-spring-damper with a five times faster controller: still invalid.
  assert.equal(relayTune("msd", 0.002).v.valid, false);
});

test("relayValidity on 1/(s+1)^3 passes and reports a small delay sensitivity", () => {
  const opts = { amplitude: 1, maxTime: 200 };
  const res = relayExperiment(lag3Config(), opts);
  const v = relayValidity(lag3Config(), opts, res);
  assert.equal(v.valid, true, v.failures.join("; "));
  // Phase slope of 3 lags at sqrt 3 rad/s is 3/(1 + 3) = 0.75 s, so one
  // 2 ms sample should move the period by about 0.002 / 0.75 = 0.3 %.
  assert.ok(v.delaySensitivity > 0 && v.delaySensitivity < 0.01, String(v.delaySensitivity));
  assert.equal(v.timingDominated, false);
  assert.equal(v.irregular, false);
  assert.equal(RELAY_LIMITS.maxDelaySensitivity, 0.25);
});

test("relayValidity flags irregular cycles and a missing delayed oscillation", () => {
  const opts = { amplitude: 1, maxTime: 200 };
  const res = relayExperiment(lag3Config(), opts);
  const bad = relayValidity(lag3Config(), opts, {
    ...res,
    converged: false,
    amplitudeSpread: 0.2,
    asymmetry: 0.3,
  });
  assert.equal(bad.valid, false);
  assert.equal(bad.timingDominated, false);
  assert.equal(bad.irregular, true);
  assert.equal(bad.failures.length, 3);
  assert.match(bad.failures.join("; "), /had not settled.*amplitude varied.*unequal times/);
  // maxTime too short for the delayed run to oscillate.
  const none = relayValidity(lag3Config(), { amplitude: 1, maxTime: 2 }, res);
  assert.equal(none.puDelayed, Infinity);
  assert.match(none.failures.join(";"), /no sustained oscillation/);
});

test("first-harmonic Ku is the plant's gain at the oscillation frequency", () => {
  // 1/(s+1)^3: |G(jw)| = (1 + w^2)^-1.5 exactly; the describing function is
  // a few percent off, the first harmonic should be within 0.5 %.
  const res = relayExperiment(lag3Config(), { amplitude: 1, maxTime: 200 });
  const w = (2 * Math.PI) / res.pu;
  nearRel(res.kuHarmonic, (1 + w * w) ** 1.5, 0.005, "lag3 first-harmonic Ku");
  const expected = (-3 * Math.atan(w) * 180) / Math.PI;
  near(((res.phaseDeg - expected + 540) % 360) - 180, 0, 0.5, "phase of G at the oscillation");
  // FOPDT K = 2, tau = 5, theta = 1 with a 10 ms controller (nearly a
  // continuous relay). Hand-derived for an ideal relay, see docs/models.md:
  // a = K d (1 - e^(-theta/tau)), so the describing function gives
  // Ku = 4 / (pi K (1 - e^-0.2)) = 3.512, and Pu = 2 theta + 2 tau ln(2 - e^-0.2) = 3.666 s,
  // while 1 / |G| at that frequency is about 4.30: the describing function
  // is 18 % low and the first harmonic is not.
  const { relay } = relayTune("fopdt", 0.01);
  nearRel(relay.kuDescribingFunction, 4 / (Math.PI * 2 * (1 - Math.exp(-0.2))), 0.01, "DF Ku");
  nearRel(relay.pu, 2 + 10 * Math.log(2 - Math.exp(-0.2)), 0.01, "Pu");
  const wf = (2 * Math.PI) / relay.pu;
  nearRel(relay.kuHarmonic, Math.hypot(1, 5 * wf) / 2, 0.002, "first-harmonic Ku");
});

test("relay notes state the describing-function error, and the harmonic option uses the other Ku", () => {
  const df = autotune(defaultSettings("fopdt"), "zn-relay");
  assert.ok(df.relay);
  assert.equal(df.relay.kuEstimate, "describing-function");
  assert.equal(df.notes.length, 1);
  assert.match(df.notes[0], /is 19 % below the first-harmonic estimate/);
  const h = autotune(defaultSettings("fopdt"), "zn-relay", "pid", { kuEstimate: "harmonic" });
  assert.ok(h.relay);
  assert.equal(h.relay.ku, h.relay.kuHarmonic);
  assert.match(h.notes[0], /The gains use the first-harmonic estimate/);
  assert.deepEqual(h.gains, zieglerNicholsUltimate(h.relay.kuHarmonic, h.relay.pu));
  // The hotend's relay output is close to a sinusoid: the estimates agree
  // within 1 % and no note is attached.
  const th = autotune(defaultSettings("thermal"), "zn-relay");
  assert.ok(th.relay);
  nearRel(th.relay.kuHarmonic, th.relay.kuDescribingFunction, 0.01);
  assert.deepEqual(th.notes, []);
  assert.throws(
    () =>
      autotune(defaultSettings("fopdt"), "zn-relay", "pid", {
        kuEstimate: /** @type {any} */ ("exact"),
      }),
    /Ku estimate must be one of/,
  );
});

test("regression: v0.1.0 tuning outputs are unchanged by default", () => {
  // Values printed by v0.1.0 (README) for the same commands.
  const th = autotune(defaultSettings("thermal"), "zn-relay");
  assert.ok(th.relay);
  nearRel(th.relay.ku, 0.3151, 2e-4, "thermal Ku");
  nearRel(th.relay.pu, 11.6, 1e-9, "thermal Pu");
  nearRel(th.gains.kp, 0.1891, 3e-4, "thermal Kp");
  nearRel(th.gains.ki, 0.0326, 2e-3, "thermal Ki");
  nearRel(th.gains.kd, 0.2742, 3e-4, "thermal Kd");
  assert.deepEqual(th.gains, zieglerNicholsUltimate(th.relay.ku, th.relay.pu));
  const motor = autotune(defaultSettings("motor"), "zn-relay");
  assert.ok(motor.relay);
  nearRel(motor.relay.ku, 4.882, 2e-4, "motor Ku");
  nearRel(motor.relay.pu, 0.004, 1e-9, "motor Pu");
  const fopdt = autotune(defaultSettings("fopdt"), "zn-relay");
  assert.ok(fopdt.relay);
  nearRel(fopdt.relay.ku, 3.388, 2e-4, "FOPDT Ku");
  nearRel(fopdt.relay.pu, 3.8, 1e-9, "FOPDT Pu");
  const s = autotune(defaultSettings("fopdt"), "simc");
  nearRel(s.gains.kp, 1.225, 5e-4, "SIMC Kp");
  nearRel(s.gains.ki, 0.2451, 5e-4, "SIMC Ki");
  assert.equal(s.relay, null);
  assert.deepEqual(s.notes, []);
});
