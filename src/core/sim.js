// Closed-loop simulation: plant (RK4), input dead time, sensor chain and a
// sampled controller with zero-order hold.
//
// Timing per plant interval dt, at t = k dt:
//   1. if k is a multiple of the controller period, sample the sensor and
//      compute a new control signal (held until the next sample);
//   2. push the held signal through the dead-time buffer;
//   3. record t, r, y, measurement, u, d;
//   4. integrate the plant over [t, t + dt] with RK4 sub-steps.

import { DelayBuffer, integrate } from "./integrate.js";
import { Pid, pidSettings } from "./pid.js";
import { createRng } from "./rng.js";

/** @typedef {import("./plants.js").Plant} Plant */
/** @typedef {import("./plants.js").Params} Params */
/** @typedef {import("./pid.js").PidSettings} PidSettings */

/**
 * @typedef {object} SensorSettings
 * @property {number} noiseStd standard deviation of white measurement noise; 0 = off
 * @property {number} quantStep resolution in output units; 0 = off
 * @property {number} lagTau first-order sensor time constant (s); 0 = off
 *
 * @typedef {object} SimConfig
 * @property {Plant} plant
 * @property {Params} params
 * @property {PidSettings} pid
 * @property {SensorSettings} sensor
 * @property {number} dt plant interval (s)
 * @property {number} substeps RK4 steps per dt
 * @property {number} seed noise seed
 * @property {number} r0 initial setpoint; the plant starts at rest there
 * @property {number} feedForward 0..1 weight on the plant's static inverse u = f(r)
 * @property {((r: number, ym: number, t: number) => number) | null} [controlLaw]
 *   replaces the PID (used by the relay experiment)
 *
 * @typedef {object} Trace
 * @property {number[]} t
 * @property {number[]} r setpoint
 * @property {number[]} y true plant output
 * @property {number[]} ym measurement seen by the controller
 * @property {number[]} u control signal (controller output, before dead time)
 * @property {number[]} d load disturbance
 */

/** @type {SensorSettings} */
export const NO_SENSOR_EFFECTS = { noiseStd: 0, quantStep: 0, lagTau: 0 };

/**
 * Round to the nearest multiple of `step` (mid-tread quantiser).
 * @param {number} value
 * @param {number} step
 * @returns {number}
 */
export function quantize(value, step) {
  return step > 0 ? Math.round(value / step) * step : value;
}

/**
 * Resolution of a speed computed by differencing encoder counts.
 * @param {number} countsPerRev counts per revolution after quadrature decoding
 * @param {number} sampleTime seconds between count readings
 * @returns {number} rad/s per count
 */
export function encoderSpeedStep(countsPerRev, sampleTime) {
  return (2 * Math.PI) / countsPerRev / sampleTime;
}

/**
 * Resolution of an ADC spanning `range` output units with `bits` bits.
 * @param {number} bits
 * @param {number} range
 * @returns {number}
 */
export function adcStep(bits, range) {
  return range / 2 ** bits;
}

export class Simulation {
  /**
   * @param {SimConfig} config
   */
  constructor(config) {
    this.config = config;
    const { plant, params, dt } = config;
    if (!(dt > 0)) throw new RangeError("dt must be positive");
    this.pid = new Pid(pidSettings(config.pid));
    this.period = Math.max(1, Math.round(config.pid.sampleTime / dt));
    this.rng = createRng(config.seed);
    this.k = 0;
    this.t = 0;
    this.r = config.r0;
    this.d = 0;
    this.x = plant.initialState(config.r0, params);
    const u0 = plant.steadyInput(config.r0, params);
    this.delay = new DelayBuffer(plant.deadTime(params), dt, u0);
    this.yLag = plant.output(this.x, params);
    this.ym = this.yLag;
    this.u = u0;
    this.pid.reset(config.r0, this.ym, u0, this.feedForward(config.r0));
    this.u = this.pid.uPrev;
  }

  /**
   * @param {number} r
   * @returns {number}
   */
  feedForward(r) {
    const w = this.config.feedForward;
    return w > 0 ? w * this.config.plant.steadyInput(r, this.config.params) : 0;
  }

  /** @returns {number} true plant output */
  output() {
    return this.config.plant.output(this.x, this.config.params);
  }

  /**
   * Read the sensor: lagged output, plus seeded noise, then quantised.
   * @returns {number}
   */
  measure() {
    const { noiseStd, quantStep } = this.config.sensor;
    const noise = noiseStd > 0 ? noiseStd * this.rng.gaussian() : 0;
    return quantize(this.yLag + noise, quantStep);
  }

  /**
   * Advance one plant interval and return the sample recorded at its start.
   * @returns {{t: number, r: number, y: number, ym: number, u: number, d: number}}
   */
  step() {
    const { plant, params, dt, substeps, controlLaw } = this.config;
    if (this.k % this.period === 0) {
      this.ym = this.measure();
      this.u = controlLaw
        ? controlLaw(this.r, this.ym, this.t)
        : this.pid.step(this.r, this.ym, this.feedForward(this.r));
    }
    const uPlant = this.delay.push(this.u);
    const sample = { t: this.t, r: this.r, y: this.output(), ym: this.ym, u: this.u, d: this.d };
    const d = this.d;
    this.x = integrate(
      (_t, x) => plant.derivative(x, uPlant, d, params),
      this.t,
      this.x,
      dt,
      substeps,
    );
    this.k += 1;
    this.t = this.k * dt;
    const lag = this.config.sensor.lagTau;
    const y = this.output();
    this.yLag = lag > 0 ? this.yLag + (1 - Math.exp(-dt / lag)) * (y - this.yLag) : y;
    return sample;
  }
}

/**
 * @typedef {object} Scenario
 * @property {number} duration seconds
 * @property {number} r1 setpoint after the step
 * @property {number} stepTime seconds
 * @property {number} disturbance load disturbance size (0 = none)
 * @property {number} disturbanceTime seconds
 */

/**
 * Run a setpoint step followed by a load disturbance step.
 * @param {SimConfig} config
 * @param {Scenario} scenario
 * @returns {Trace}
 */
export function runScenario(config, scenario) {
  const sim = new Simulation(config);
  /** @type {Trace} */
  const trace = { t: [], r: [], y: [], ym: [], u: [], d: [] };
  const n = Math.round(scenario.duration / config.dt);
  for (let k = 0; k <= n; k++) {
    const t = k * config.dt;
    if (t >= scenario.stepTime - 1e-12) sim.r = scenario.r1;
    if (scenario.disturbance !== 0 && t >= scenario.disturbanceTime - 1e-12) {
      sim.d = scenario.disturbance;
    }
    const s = sim.step();
    trace.t.push(s.t);
    trace.r.push(s.r);
    trace.y.push(s.y);
    trace.ym.push(s.ym);
    trace.u.push(s.u);
    trace.d.push(s.d);
  }
  return trace;
}
