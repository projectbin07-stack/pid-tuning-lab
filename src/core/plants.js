// Plant models. Every plant is a set of ordinary differential equations with
// one input u, one load disturbance d and one measured output y. Parameters
// are illustrative values chosen to give interesting dynamics; they are not
// measurements of any particular motor, heater or mechanism.

/**
 * @typedef {Record<string, number>} Params
 *
 * @typedef {object} ParamSpec
 * @property {string} key
 * @property {string} label
 * @property {string} unit
 * @property {number} value default
 * @property {number} min
 * @property {number} max
 *
 * @typedef {object} TransferFunction
 * Rational transfer function num(s)/den(s) times exp(-delay s). Coefficients
 * are in ascending powers of s: [a0, a1, a2] means a0 + a1 s + a2 s^2.
 * @property {number[]} num
 * @property {number[]} den
 * @property {number} delay seconds
 *
 * @typedef {object} PlantDefaults
 * @property {number} dt plant integration interval, seconds
 * @property {number} substeps RK4 steps per dt
 * @property {number} sampleTime controller sample time, seconds
 * @property {number} duration scenario length, seconds
 * @property {number} r0 initial setpoint (the plant starts at rest there)
 * @property {number} r1 setpoint after the step
 * @property {number} stepTime seconds
 * @property {number} disturbanceTime seconds
 * @property {number} disturbance load disturbance applied at disturbanceTime
 * @property {number} uMin
 * @property {number} uMax
 * @property {{kp: number, ki: number, kd: number}} gains parallel-form gains
 * @property {number} quantStep a realistic sensor resolution in output units
 * @property {string} quantNote where quantStep comes from
 * @property {number} noiseStd a realistic measurement noise level
 * @property {number} chartMax upper end of the output axis
 * @property {number} chartMin lower end of the output axis
 *
 * @typedef {object} Plant
 * @property {string} id
 * @property {string} name
 * @property {string} summary
 * @property {string[]} equations plain-text model equations
 * @property {string} outputLabel label of y, with unit
 * @property {string} inputLabel label of u, with unit
 * @property {string} disturbanceLabel label of d, with unit
 * @property {"linear" | "linear-except-friction" | "nonlinear"} linearity
 * @property {boolean} selfRegulating false for plants with an integrator
 * @property {ParamSpec[]} params
 * @property {PlantDefaults} defaults
 * @property {(y0: number, p: Params) => Float64Array} initialState state at rest with output y0
 * @property {(x: Float64Array, u: number, d: number, p: Params) => Float64Array} derivative
 * @property {(x: Float64Array, p: Params) => number} output
 * @property {(p: Params) => number} deadTime input transport delay, seconds
 * @property {(y: number, p: Params) => number} steadyInput u that holds y with d = 0
 * @property {(p: Params, y0: number) => TransferFunction} transferFunction
 *   linear model from u to y (linearised about output y0 where needed)
 */

const G = 9.81;

/** @type {Plant} */
const fopdt = {
  id: "fopdt",
  name: "First order + dead time",
  summary: "Generic process: one lag and a transport delay, the model most tuning rules assume.",
  equations: ["tau dy/dt = -y + K (u(t - theta) + d)"],
  outputLabel: "y",
  inputLabel: "u",
  disturbanceLabel: "d (input units)",
  linearity: "linear",
  selfRegulating: true,
  params: [
    { key: "K", label: "Gain K", unit: "", value: 2, min: 0.1, max: 10 },
    { key: "tau", label: "Time constant tau", unit: "s", value: 5, min: 0.1, max: 50 },
    { key: "theta", label: "Dead time theta", unit: "s", value: 1, min: 0, max: 20 },
  ],
  defaults: {
    dt: 0.01,
    substeps: 1,
    sampleTime: 0.05,
    duration: 60,
    r0: 0,
    r1: 1,
    stepTime: 1,
    disturbanceTime: 35,
    disturbance: -0.3,
    uMin: -2,
    uMax: 2,
    gains: { kp: 1.1, ki: 0.22, kd: 0.4 },
    quantStep: 0.001,
    quantNote: "0.1 % of a unit span",
    noiseStd: 0.01,
    chartMin: -0.2,
    chartMax: 1.6,
  },
  initialState: (y0) => Float64Array.of(y0),
  derivative: (x, u, d, p) => Float64Array.of((p.K * (u + d) - x[0]) / p.tau),
  output: (x) => x[0],
  deadTime: (p) => p.theta,
  steadyInput: (y, p) => y / p.K,
  transferFunction: (p) => ({ num: [p.K], den: [1, p.tau], delay: p.theta }),
};

/** @type {Plant} */
const massSpring = {
  id: "msd",
  name: "Mass-spring-damper",
  summary:
    "Second-order mechanical resonance (position of a mass on a spring), underdamped by default.",
  equations: ["m x'' + c x' + k x = u + d", "omega_n = sqrt(k/m), zeta = c / (2 sqrt(k m))"],
  outputLabel: "x (m)",
  inputLabel: "F (N)",
  disturbanceLabel: "force (N)",
  linearity: "linear",
  selfRegulating: true,
  params: [
    { key: "m", label: "Mass m", unit: "kg", value: 1, min: 0.05, max: 20 },
    { key: "c", label: "Damping c", unit: "N s/m", value: 0.8, min: 0, max: 50 },
    { key: "k", label: "Stiffness k", unit: "N/m", value: 4, min: 0.1, max: 200 },
  ],
  defaults: {
    dt: 0.001,
    substeps: 1,
    sampleTime: 0.01,
    duration: 15,
    r0: 0,
    r1: 1,
    stepTime: 0.5,
    disturbanceTime: 9,
    disturbance: 2,
    uMin: -20,
    uMax: 20,
    gains: { kp: 12, ki: 12, kd: 3 },
    quantStep: 0.0001,
    quantNote: "0.1 mm linear scale",
    noiseStd: 0.002,
    chartMin: -0.2,
    chartMax: 1.6,
  },
  initialState: (y0) => Float64Array.of(y0, 0),
  derivative: (x, u, d, p) => Float64Array.of(x[1], (u + d - p.c * x[1] - p.k * x[0]) / p.m),
  output: (x) => x[0],
  deadTime: () => 0,
  steadyInput: (y, p) => p.k * y,
  transferFunction: (p) => ({ num: [1], den: [p.k, p.c, p.m], delay: 0 }),
};

/** @type {Plant} */
const dcMotor = {
  id: "motor",
  name: "DC motor (speed)",
  summary:
    "Brushed DC motor driven by a voltage, with winding inductance, viscous and Coulomb friction; output is shaft speed.",
  equations: [
    "L di/dt = V - R i - Ke w",
    "J dw/dt = Kt i - b w - Tc tanh(w / ws) - TL",
    "V is clamped to +/- Vmax by the drive (the controller output limits)",
  ],
  outputLabel: "w (rad/s)",
  inputLabel: "V (V)",
  disturbanceLabel: "load torque TL (N m)",
  linearity: "linear-except-friction",
  selfRegulating: true,
  params: [
    { key: "R", label: "Resistance R", unit: "ohm", value: 2, min: 0.1, max: 20 },
    { key: "L", label: "Inductance L", unit: "H", value: 0.001, min: 0.0001, max: 0.02 },
    { key: "Kt", label: "Torque constant Kt", unit: "N m/A", value: 0.02, min: 0.002, max: 0.5 },
    {
      key: "Ke",
      label: "Back-EMF constant Ke",
      unit: "V s/rad",
      value: 0.02,
      min: 0.002,
      max: 0.5,
    },
    { key: "J", label: "Inertia J", unit: "kg m^2", value: 2e-5, min: 1e-6, max: 1e-2 },
    { key: "b", label: "Viscous friction b", unit: "N m s", value: 1e-5, min: 0, max: 1e-2 },
    { key: "Tc", label: "Coulomb friction Tc", unit: "N m", value: 0.004, min: 0, max: 0.1 },
    { key: "ws", label: "Friction smoothing ws", unit: "rad/s", value: 0.5, min: 0.01, max: 10 },
  ],
  defaults: {
    dt: 0.0001,
    substeps: 1,
    sampleTime: 0.001,
    duration: 0.6,
    r0: 0,
    r1: 200,
    stepTime: 0.02,
    disturbanceTime: 0.35,
    disturbance: 0.02,
    uMin: -12,
    uMax: 12,
    gains: { kp: 0.05, ki: 2, kd: 0 },
    quantStep: (2 * Math.PI) / 4096 / 0.001,
    quantNote: "1024-line quadrature encoder (4096 counts/rev) differenced every 1 ms",
    noiseStd: 0.5,
    chartMin: -20,
    chartMax: 300,
  },
  // Steady state at speed y0: the current supplies the friction torque.
  initialState: (y0, p) => Float64Array.of((p.b * y0 + p.Tc * Math.tanh(y0 / p.ws)) / p.Kt, y0),
  derivative: (x, u, d, p) => {
    const [i, w] = x;
    return Float64Array.of(
      (u - p.R * i - p.Ke * w) / p.L,
      (p.Kt * i - p.b * w - p.Tc * Math.tanh(w / p.ws) - d) / p.J,
    );
  },
  output: (x) => x[1],
  deadTime: () => 0,
  steadyInput: (y, p) => (y * (p.R * p.b + p.Kt * p.Ke) + p.R * p.Tc * Math.tanh(y / p.ws)) / p.Kt,
  // Linear part only: Coulomb friction is left out of the frequency response.
  transferFunction: (p) => ({
    num: [p.Kt],
    den: [p.R * p.b + p.Kt * p.Ke, p.L * p.b + p.R * p.J, p.L * p.J],
    delay: 0,
  }),
};

/** @type {Plant} */
const integrating = {
  id: "integrating",
  name: "Integrating (position from velocity)",
  summary:
    "Position of an axis whose velocity loop is already closed: a velocity command, a first-order velocity lag and an integrator.",
  equations: ["tau dv/dt = -v + K (u + d)", "dx/dt = v"],
  outputLabel: "x (m)",
  inputLabel: "velocity command (m/s)",
  disturbanceLabel: "velocity offset (m/s)",
  linearity: "linear",
  selfRegulating: false,
  params: [
    { key: "K", label: "Velocity gain K", unit: "", value: 1, min: 0.1, max: 10 },
    { key: "tau", label: "Velocity lag tau", unit: "s", value: 0.2, min: 0.001, max: 5 },
  ],
  defaults: {
    dt: 0.001,
    substeps: 1,
    sampleTime: 0.01,
    duration: 10,
    r0: 0,
    r1: 1,
    stepTime: 0.5,
    disturbanceTime: 6,
    disturbance: 0.1,
    uMin: -0.5,
    uMax: 0.5,
    gains: { kp: 2, ki: 1, kd: 0 },
    quantStep: 0.00001,
    quantNote: "10 um linear encoder",
    noiseStd: 0.0005,
    chartMin: -0.2,
    chartMax: 1.4,
  },
  initialState: (y0) => Float64Array.of(y0, 0),
  derivative: (x, u, d, p) => Float64Array.of(x[1], (p.K * (u + d) - x[1]) / p.tau),
  output: (x) => x[0],
  deadTime: () => 0,
  steadyInput: () => 0,
  transferFunction: (p) => ({ num: [p.K], den: [0, 1, p.tau], delay: 0 }),
};

/**
 * @param {number[]} a
 * @param {number[]} b
 * @returns {number[]} product of two ascending-coefficient polynomials
 */
export function polyMul(a, b) {
  const out = new Array(a.length + b.length - 1).fill(0);
  for (let i = 0; i < a.length; i++) for (let j = 0; j < b.length; j++) out[i + j] += a[i] * b[j];
  return out;
}

/**
 * @param {number[]} a
 * @param {number[]} b
 * @param {number} [kb] scale applied to b
 * @returns {number[]} a + kb b
 */
export function polyAdd(a, b, kb = 1) {
  const out = new Array(Math.max(a.length, b.length)).fill(0);
  for (let i = 0; i < a.length; i++) out[i] += a[i];
  for (let i = 0; i < b.length; i++) out[i] += kb * b[i];
  return out;
}

/** @type {Plant} */
const thermal = {
  id: "thermal",
  name: "Thermal (hotend)",
  summary:
    "3D-printer style hotend: heater cartridge, aluminium block losing heat to ambient, and a thermistor behind a thermal resistance. Heating only.",
  equations: [
    "C0 dTh/dt = Pmax u - (Th - T1)/R01",
    "C1 dT1/dt = (Th - T1)/R01 + d - (T1 - Ta)/Ra - (T1 - T2)/R12",
    "C2 dT2/dt = (T1 - T2)/R12",
    "u in [0, 1] is the heater duty cycle; y = T2 (thermistor)",
  ],
  outputLabel: "T (degC)",
  inputLabel: "heater duty (0..1)",
  disturbanceLabel: "heat flow into block (W)",
  linearity: "linear",
  selfRegulating: true,
  params: [
    { key: "Pmax", label: "Heater power Pmax", unit: "W", value: 40, min: 5, max: 200 },
    { key: "C0", label: "Cartridge capacity C0", unit: "J/K", value: 3, min: 0.1, max: 50 },
    { key: "R01", label: "Cartridge-block R01", unit: "K/W", value: 0.8, min: 0.05, max: 10 },
    { key: "C1", label: "Block capacity C1", unit: "J/K", value: 8, min: 1, max: 100 },
    { key: "Ra", label: "Loss to ambient Ra", unit: "K/W", value: 6, min: 1, max: 30 },
    { key: "C2", label: "Sensor capacity C2", unit: "J/K", value: 0.5, min: 0.05, max: 10 },
    { key: "R12", label: "Block-sensor R12", unit: "K/W", value: 4, min: 0.1, max: 30 },
    { key: "Ta", label: "Ambient Ta", unit: "degC", value: 25, min: -10, max: 50 },
  ],
  defaults: {
    dt: 0.05,
    substeps: 1,
    sampleTime: 0.1,
    duration: 300,
    r0: 25,
    r1: 200,
    stepTime: 5,
    disturbanceTime: 200,
    disturbance: -8,
    uMin: 0,
    uMax: 1,
    gains: { kp: 0.08, ki: 0.004, kd: 0.2 },
    quantStep: 0.3,
    quantNote: "10-bit ADC over roughly 0..300 degC (linearised thermistor)",
    noiseStd: 0.3,
    chartMin: 0,
    chartMax: 260,
  },
  // In steady state the heater core is hotter than the block by the heat it
  // passes, (y0 - Ta) / Ra, times R01; no heat flows into the sensor.
  initialState: (y0, p) => Float64Array.of(y0 + ((y0 - p.Ta) / p.Ra) * p.R01, y0, y0),
  derivative: (x, u, d, p) => {
    const [th, t1, t2] = x;
    return Float64Array.of(
      (p.Pmax * u - (th - t1) / p.R01) / p.C0,
      ((th - t1) / p.R01 + d - (t1 - p.Ta) / p.Ra - (t1 - t2) / p.R12) / p.C1,
      (t1 - t2) / p.R12 / p.C2,
    );
  },
  output: (x) => x[2],
  deadTime: () => 0,
  steadyInput: (y, p) => (y - p.Ta) / (p.Ra * p.Pmax),
  // Deviation from ambient; the model is linear, so this is exact.
  // With A = C2 R12 s + 1, B = C0 R01 s + 1 and g = 1/Ra + 1/R12 + 1/R01:
  // T2 / (Pmax u) = 1 / (A B (C1 s + g) - B / R12 - A / R01).
  transferFunction: (p) => {
    const A = [1, p.C2 * p.R12];
    const B = [1, p.C0 * p.R01];
    const g = 1 / p.Ra + 1 / p.R12 + 1 / p.R01;
    let den = polyMul(polyMul(A, B), [g, p.C1]);
    den = polyAdd(den, B, -1 / p.R12);
    den = polyAdd(den, A, -1 / p.R01);
    return { num: [p.Pmax], den, delay: 0 };
  },
};

/** @type {Plant} */
const arm = {
  id: "arm",
  name: "Robot arm joint (nonlinear)",
  summary:
    "Single link swinging in gravity, driven by a torque-limited joint motor. Gravity makes the gain depend on the angle.",
  equations: [
    "J q'' = tau + d - b q' - m g l sin(q),  J = m l^2 + Jm",
    "q = 0 hangs straight down; tau is clamped to the actuator limit",
  ],
  outputLabel: "q (rad)",
  inputLabel: "torque (N m)",
  disturbanceLabel: "torque (N m)",
  linearity: "nonlinear",
  selfRegulating: true,
  params: [
    { key: "m", label: "Payload mass m", unit: "kg", value: 0.5, min: 0.01, max: 5 },
    { key: "l", label: "Link length l", unit: "m", value: 0.3, min: 0.05, max: 1 },
    { key: "Jm", label: "Motor inertia Jm", unit: "kg m^2", value: 0.005, min: 0, max: 0.1 },
    { key: "b", label: "Joint damping b", unit: "N m s", value: 0.05, min: 0, max: 2 },
  ],
  defaults: {
    dt: 0.001,
    substeps: 1,
    sampleTime: 0.005,
    duration: 5,
    r0: 0,
    r1: Math.PI / 2,
    stepTime: 0.2,
    disturbanceTime: 3,
    disturbance: -0.4,
    uMin: -2.5,
    uMax: 2.5,
    gains: { kp: 4, ki: 4, kd: 0.6 },
    quantStep: (2 * Math.PI) / 4096,
    quantNote: "12-bit absolute magnetic encoder",
    noiseStd: 0.002,
    chartMin: -0.3,
    chartMax: 2.2,
  },
  initialState: (y0) => Float64Array.of(y0, 0),
  derivative: (x, u, d, p) => {
    const J = p.m * p.l * p.l + p.Jm;
    return Float64Array.of(x[1], (u + d - p.b * x[1] - p.m * G * p.l * Math.sin(x[0])) / J);
  },
  output: (x) => x[0],
  deadTime: () => 0,
  steadyInput: (y, p) => p.m * G * p.l * Math.sin(y),
  // Linearised about the angle y0: sin(q) ~ sin(q0) + cos(q0)(q - q0).
  transferFunction: (p, y0) => ({
    num: [1],
    den: [p.m * G * p.l * Math.cos(y0), p.b, p.m * p.l * p.l + p.Jm],
    delay: 0,
  }),
};

/** @type {Plant[]} */
export const PLANTS = [fopdt, massSpring, dcMotor, integrating, thermal, arm];

/**
 * @param {string} id
 * @returns {Plant}
 */
export function getPlant(id) {
  const plant = PLANTS.find((p) => p.id === id);
  if (!plant)
    throw new RangeError(`unknown plant "${id}"; expected one of ${plantIds().join(", ")}`);
  return plant;
}

/** @returns {string[]} */
export function plantIds() {
  return PLANTS.map((p) => p.id);
}

/**
 * Default parameters merged with overrides. Unknown keys are rejected so a
 * typo on the command line or in a URL does not silently do nothing.
 * @param {Plant} plant
 * @param {Record<string, number>} [overrides]
 * @returns {Params}
 */
export function plantParams(plant, overrides = {}) {
  /** @type {Params} */
  const p = {};
  for (const spec of plant.params) p[spec.key] = spec.value;
  for (const [key, value] of Object.entries(overrides)) {
    const spec = plant.params.find((s) => s.key === key);
    if (!spec) throw new RangeError(`plant "${plant.id}" has no parameter "${key}"`);
    if (!Number.isFinite(value) || value < spec.min || value > spec.max) {
      throw new RangeError(`${plant.id}.${key} must be in [${spec.min}, ${spec.max}]`);
    }
    p[key] = value;
  }
  return p;
}
