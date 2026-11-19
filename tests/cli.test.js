import test from "node:test";
import assert from "node:assert/strict";
import { spawnSync } from "node:child_process";
import { fileURLToPath } from "node:url";
import { VERSION, main, settingsFromOptions, toJson } from "../src/cli.js";

/**
 * Run the CLI in memory.
 * @param {string[]} argv
 */
function run(argv) {
  let out = "";
  let err = "";
  /** @type {Record<string, string>} */
  const files = {};
  const code = main(argv, {
    out: (t) => (out += t),
    err: (t) => (err += t),
    writeFile: (p, d) => (files[p] = d),
  });
  return { code, out, err, files };
}

test("run --json has the documented shape", () => {
  const r = run(["run", "--plant", "motor", "--kp", "0.05", "--ki", "2", "--kd", "0", "--json"]);
  assert.equal(r.code, 0, r.err);
  const j = JSON.parse(r.out);
  assert.equal(j.plant, "motor");
  assert.deepEqual(Object.keys(j).sort(), [
    "disturbance",
    "marginNote",
    "margins",
    "plant",
    "scenario",
    "settings",
    "step",
  ]);
  for (const k of [
    "riseTime",
    "settlingTime",
    "overshoot",
    "steadyStateError",
    "iae",
    "ise",
    "itae",
    "effort",
    "totalVariation",
    "saturationTime",
  ]) {
    assert.equal(typeof j.step[k], "number", k);
  }
  for (const k of [
    "gainMargin",
    "gainMarginDb",
    "phaseMargin",
    "gainCrossover",
    "phaseCrossover",
    "delayMargin",
  ]) {
    assert.equal(typeof j.margins[k], "number", k);
  }
  assert.deepEqual(Object.keys(j.disturbance).sort(), ["iae", "peakDeviation", "recoveryTime"]);
  assert.equal(j.settings.kp, 0.05);
  assert.equal(j.marginNote, "linear model without Coulomb friction");
});

test("run output is deterministic for a seed, including noise", () => {
  const args = [
    "run",
    "--plant",
    "thermal",
    "--noise",
    "default",
    "--quant",
    "default",
    "--seed",
    "4",
    "--json",
  ];
  const a = run(args).out;
  const b = run(args).out;
  assert.equal(a, b);
  const c = run([...args.slice(0, -2), "--seed", "5", "--json"]).out;
  assert.notEqual(a, c);
});

test("tune --json reports the model, the gains and the evaluation", () => {
  const r = run(["tune", "--plant", "fopdt", "--method", "simc", "--json"]);
  assert.equal(r.code, 0, r.err);
  const j = JSON.parse(r.out);
  assert.equal(j.method, "simc");
  assert.equal(j.type, "pi");
  assert.ok(Math.abs(j.model.K - 2) < 0.02);
  assert.equal(j.relay, null);
  assert.deepEqual(Object.keys(j.gains).sort(), ["kc", "kd", "ki", "kp", "td", "ti"]);
  assert.deepEqual(j.warnings, []);
  assert.equal(typeof j.step.overshoot, "number");
  const relay = JSON.parse(
    run(["tune", "--plant", "thermal", "--method", "zn-relay", "--type", "pi", "--json"]).out,
  );
  assert.equal(relay.type, "pi");
  assert.equal(relay.model, null);
  assert.ok(relay.relay.ku > 0 && relay.relay.pu > 0);
  assert.equal(relay.relay.kuEstimate, "describing-function");
  assert.equal(relay.relay.ku, relay.relay.kuDescribingFunction);
  assert.equal(relay.relay.validity.valid, true);
  assert.deepEqual(relay.relay.validity.failures, []);
  assert.deepEqual(relay.warnings, []);
  assert.deepEqual(relay.notes, []);
  assert.equal(relay.gains.kd, 0);
});

test("text output of run and tune", () => {
  const r = run(["run", "--plant", "fopdt"]);
  assert.match(r.out, /^plant {8}First order \+ dead time \(fopdt\)/);
  assert.match(r.out, /overshoot {13}\d/);
  assert.match(r.out, /phase margin {10}\d/);
  const t = run(["tune", "--plant", "fopdt", "--method", "cohen-coon"]);
  assert.match(t.out, /FOPDT fit {4}least squares: K 2/);
  assert.match(t.out, /two-point/);
  const w = run(["tune", "--plant", "motor", "--method", "zn-relay"]);
  assert.match(w.out, /relay test {3}Ku /);
  assert.match(w.out, /validity {5}FAILED: 4 samples per period/);
  assert.match(w.out, /warning {6}The relay test failed its validity check/);
  assert.match(w.out, /Use a step-test method \(SIMC or Cohen-Coon\)/);
  const integ = run(["tune", "--plant", "integrating", "--method", "tyreus-luyben"]);
  assert.match(integ.out, /Tyreus-Luyben gains/);
  assert.match(integ.out, /need a self-regulating plant and do not apply here/);
  const ok = run(["tune", "--plant", "fopdt", "--method", "zn-relay"]);
  assert.match(ok.out, /validity {5}passed: 76 samples per period/);
  assert.match(ok.out, /note {9}The describing-function Ku \(3\.388\) is 19 % below/);
  assert.doesNotMatch(ok.out, /warning/);
  const h = run(["tune", "--plant", "fopdt", "--method", "zn-relay", "--ku", "harmonic"]);
  assert.match(h.out, /relay test {3}Ku 4\.164,/);
  assert.match(h.out, /the rule uses the first harmonic/);
  const arm = run(["run", "--plant", "arm", "--rate", "50", "--ff", "1", "--disturbance", "0"]);
  assert.match(arm.out, /rate limit 50\/s, feed-forward 1/);
  assert.match(arm.out, /linearised about y = 1\.57/);
  assert.doesNotMatch(arm.out, /load disturbance/);
});

test("ideal form, parameters and sensor options are parsed", () => {
  const s = settingsFromOptions({
    plant: "fopdt",
    kc: "2",
    ti: "4",
    td: "0.5",
    param: ["K=3", "theta=0.5"],
    aw: "clamp",
    noise: "0.1",
    quant: "0.01",
    lag: "0.2",
  });
  assert.equal(s.form, "ideal");
  assert.deepEqual([s.kc, s.ti, s.td], [2, 4, 0.5]);
  assert.deepEqual(s.params, { K: 3, theta: 0.5 });
  assert.equal(s.antiWindup, "clamp");
  assert.deepEqual([s.noiseStd, s.quantStep, s.lagTau], [0.1, 0.01, 0.2]);
  // --form ideal alone converts the default parallel gains.
  const conv = settingsFromOptions({ plant: "fopdt", form: "ideal" });
  assert.equal(conv.kc, 1.1);
  assert.ok(Math.abs(conv.ti - 5) < 1e-12);
  const zero = settingsFromOptions({ plant: "integrating", form: "ideal", kp: "0", ki: "1" });
  assert.equal(zero.td, 0);
  const def = settingsFromOptions({});
  assert.equal(def.plant, "fopdt");
  const r = run(["run", "--plant", "fopdt", "--form", "ideal", "--kc", "1", "--ti", "5", "--json"]);
  assert.equal(JSON.parse(r.out).settings.form, "ideal");
});

test("CSV export writes every sample", () => {
  const r = run(["run", "--plant", "msd", "--csv", "trace.csv"]);
  assert.equal(r.code, 0);
  const lines = r.files["trace.csv"].trim().split("\n");
  assert.equal(lines[0], "t,r,y,ym,u,d");
  assert.equal(lines.length, 1 + 15001);
  assert.equal(lines[1].split(",").length, 6);
});

test("plants, help and version", () => {
  const p = run(["plants"]);
  assert.equal(p.out.trim().split("\n").length, 6);
  assert.match(p.out, /^fopdt/);
  const pj = JSON.parse(run(["plants", "--json"]).out);
  assert.equal(pj.length, 6);
  assert.ok(pj[0].equations.length > 0);
  assert.match(run(["--help"]).out, /Usage:/);
  assert.equal(run(["--version"]).out, `${VERSION}\n`);
  const none = run([]);
  assert.equal(none.code, 2);
  assert.match(none.out, /Usage:/);
});

test("usage errors exit with code 2, model errors with code 1", () => {
  /** @type {Array<[string[], RegExp]>} */
  const cases = [
    [["run", "--kp", "abc"], /--kp must be a number/],
    [["run", "--kp", " "], /--kp must be a number/],
    [["run", "--form", "series"], /--form/],
    [["run", "--aw", "magic"], /--aw/],
    [["run", "--param", "K"], /key=value/],
    [["tune", "--plant", "fopdt"], /needs --method/],
    [["tune", "--method", "best"], /--method must be/],
    [["tune", "--method", "simc", "--type", "pd"], /--type/],
    [["tune", "--method", "zn-relay", "--ku", "exact"], /--ku must be df or harmonic/],
    [["fly"], /unknown command/],
    [["run", "extra"], /unexpected argument/],
    [["run", "--bogus"], /Unknown option/],
  ];
  for (const [argv, re] of cases) {
    const r = run(argv);
    assert.equal(r.code, 2, argv.join(" "));
    assert.match(r.err, re);
  }
  const model = run(["run", "--plant", "nope"]);
  assert.equal(model.code, 1);
  assert.match(model.err, /unknown plant/);
  const integ = run(["tune", "--plant", "integrating", "--method", "simc"]);
  assert.equal(integ.code, 1);
  assert.match(integ.err, /self-regulating/);
});

test("JSON writes non-finite numbers as null", () => {
  assert.equal(
    toJson({ a: Infinity, b: NaN, c: 1 / 3 }),
    '{\n  "a": null,\n  "b": null,\n  "c": 0.333333\n}',
  );
});

test("the bin script runs as a process", () => {
  const bin = fileURLToPath(new URL("../bin/pid-tuning-lab.js", import.meta.url));
  const r = spawnSync(process.execPath, [bin, "run", "--plant", "fopdt", "--json"], {
    encoding: "utf8",
  });
  assert.equal(r.status, 0, r.stderr);
  assert.equal(JSON.parse(r.stdout).plant, "fopdt");
  const bad = spawnSync(process.execPath, [bin, "fly"], { encoding: "utf8" });
  assert.equal(bad.status, 2);
});
