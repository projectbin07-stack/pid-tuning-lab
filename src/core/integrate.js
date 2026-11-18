// Fixed-step numerical integration and a transport-delay buffer.

/**
 * Right-hand side of dx/dt = f(t, x). Must not modify x.
 * @typedef {(t: number, x: Float64Array) => Float64Array} Derivative
 */

/**
 * One classical fourth-order Runge-Kutta step. Returns a new state array.
 * @param {Derivative} f
 * @param {number} t
 * @param {Float64Array} x
 * @param {number} h
 * @returns {Float64Array}
 */
export function rk4Step(f, t, x, h) {
  const n = x.length;
  const k1 = f(t, x);
  const tmp = new Float64Array(n);
  for (let i = 0; i < n; i++) tmp[i] = x[i] + 0.5 * h * k1[i];
  const k2 = f(t + 0.5 * h, tmp);
  for (let i = 0; i < n; i++) tmp[i] = x[i] + 0.5 * h * k2[i];
  const k3 = f(t + 0.5 * h, tmp);
  for (let i = 0; i < n; i++) tmp[i] = x[i] + h * k3[i];
  const k4 = f(t + h, tmp);
  const out = new Float64Array(n);
  for (let i = 0; i < n; i++) out[i] = x[i] + (h / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
  return out;
}

/**
 * Advance by `dt` using `substeps` equal RK4 steps. Inputs are held constant
 * over the interval (zero-order hold), so `f` may close over them.
 * @param {Derivative} f
 * @param {number} t
 * @param {Float64Array} x
 * @param {number} dt
 * @param {number} [substeps]
 * @returns {Float64Array}
 */
export function integrate(f, t, x, dt, substeps = 1) {
  const n = Math.max(1, Math.floor(substeps));
  const h = dt / n;
  let state = x;
  for (let i = 0; i < n; i++) state = rk4Step(f, t + i * h, state, h);
  return state;
}

/**
 * Pure transport delay of `delay` seconds for a signal sampled every `dt`
 * seconds. The delay is rounded to a whole number of samples; values pushed
 * before the buffer has filled come out as `initial`.
 */
export class DelayBuffer {
  /**
   * @param {number} delay seconds, >= 0
   * @param {number} dt sample interval in seconds, > 0
   * @param {number} [initial]
   */
  constructor(delay, dt, initial = 0) {
    if (!(dt > 0)) throw new RangeError("dt must be positive");
    if (!(delay >= 0)) throw new RangeError("delay must be non-negative");
    this.samples = Math.round(delay / dt);
    /** @type {Float64Array} */
    this.buffer = new Float64Array(this.samples).fill(initial);
    this.index = 0;
  }

  /**
   * Push the newest value and return the value from `samples` pushes ago.
   * @param {number} value
   * @returns {number}
   */
  push(value) {
    if (this.samples === 0) return value;
    const out = this.buffer[this.index];
    this.buffer[this.index] = value;
    this.index = (this.index + 1) % this.samples;
    return out;
  }
}
