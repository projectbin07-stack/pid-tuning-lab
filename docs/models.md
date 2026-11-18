# Models, controller and methods

This document describes exactly what the code computes. Every plant is a
simulated model with illustrative parameters. None of them was fitted to, or
checked against, a real motor, heater or mechanism.

Notation: `u` is the controller output (plant input), `y` the plant output,
`d` the load disturbance, `r` the setpoint. All coefficients of transfer
functions in the code are stored in ascending powers of `s`.

## Simulation

Source: `src/core/sim.js`, `src/core/integrate.js`.

- **Integration.** Each plant is a set of ODEs `dx/dt = f(x, u, d)`. The
  simulator advances them with the classical fourth-order Runge-Kutta method
  at a fixed plant interval `dt` (optionally split into sub-steps). `u` and
  `d` are held constant over each interval. The test suite checks the
  observed order of convergence (4.0 +/- 0.15) on a problem with a known
  solution.
- **Dead time.** A ring buffer delays the plant input by `round(theta / dt)`
  samples, so dead time is exact when it is a multiple of `dt` and rounded
  otherwise.
- **Sampling.** The controller runs every `Ts` (rounded to a whole number of
  plant intervals) and its output is held between samples (zero-order hold).
- **Sensor chain**, applied in this order: first-order lag with time
  constant `tau_s` (exact discretisation per plant interval), white Gaussian
  noise drawn from a seeded generator (mulberry32 + Box-Muller) at each
  controller sample, then mid-tread quantisation to a step `q`. The
  controller sees only this measurement; all metrics use the true output.
- **Determinism.** Same settings and seed give bit-identical traces in the
  browser and the CLI.

## Plants

Source: `src/core/plants.js`. Default parameter values are in the UI and
`node bin/pid-tuning-lab.js plants --json`.

### First order plus dead time (`fopdt`)

```
tau dy/dt = -y + K (u(t - theta) + d)
G(s) = K e^(-theta s) / (tau s + 1)
```

Defaults K = 2, tau = 5 s, theta = 1 s. The model most tuning rules assume.
The disturbance enters at the plant input (input units) and is not delayed.

### Mass-spring-damper (`msd`)

```
m x'' + c x' + k x = u + d
G(s) = 1 / (m s^2 + c s + k),  omega_n = sqrt(k/m),  zeta = c / (2 sqrt(k m))
```

Defaults m = 1 kg, c = 0.8 N s/m, k = 4 N/m: omega_n = 2 rad/s, zeta = 0.2
(underdamped). Larger `c` gives critically damped (`c = 2 sqrt(k m)`) or
overdamped behaviour.

### DC motor, speed output (`motor`)

```
L di/dt = V - R i - Ke w
J dw/dt = Kt i - b w - Tc tanh(w / ws) - TL
G(s) = w/V = Kt / ((L s + R)(J s + b) + Kt Ke)       (Tc = 0)
```

Defaults R = 2 ohm, L = 1 mH, Kt = Ke = 0.02, J = 2e-5 kg m^2,
b = 1e-5 N m s, Tc = 4 mN m, ws = 0.5 rad/s, supply +/- 12 V (the controller
output limits). Electrical time constant L/R = 0.5 ms, mechanical about
100 ms. Coulomb friction is smoothed with `tanh(w / ws)` so the right-hand
side stays continuous for RK4; the smoothing speed `ws` is a modelling
choice, not a physical constant, and the model has no true stiction
(stick-slip). The transfer function used for margins leaves Coulomb friction
out. The disturbance is a load torque that opposes positive speed.

### Integrating plant (`integrating`)

```
tau dv/dt = -v + K (u + d)
dx/dt = v
G(s) = K / (s (tau s + 1))
```

An axis whose velocity loop is already closed: the controller commands a
velocity and measures position. Defaults K = 1, tau = 0.2 s, velocity
command limited to +/- 0.5 m/s. There is no steady state for a constant
input, so the step-test tuning methods refuse this plant.

### Thermal, hotend style (`thermal`)

Three thermal capacities in a chain: heater cartridge (`Th`, C0), aluminium
block (`T1`, C1, losing heat to ambient through Ra) and thermistor (`T2`,
C2, measured):

```
C0 dTh/dt = Pmax u - (Th - T1)/R01
C1 dT1/dt = (Th - T1)/R01 + d - (T1 - Ta)/Ra - (T1 - T2)/R12
C2 dT2/dt = (T1 - T2)/R12
```

With `A = C2 R12 s + 1`, `B = C0 R01 s + 1` and
`g = 1/Ra + 1/R12 + 1/R01`, the transfer function from heater power to
sensor temperature (deviation from ambient) is

```
T2 / (Pmax u) = 1 / (A B (C1 s + g) - B/R12 - A/R01)
```

Defaults Pmax = 40 W, C0 = 3 J/K, R01 = 0.8 K/W, C1 = 8 J/K, Ra = 6 K/W,
C2 = 0.5 J/K, R12 = 4 K/W, Ta = 25 degC. Steady state `T2 = Ta + Pmax Ra u`,
so the maximum reachable temperature is 265 degC. `u` is a duty cycle limited
to [0, 1]: the heater cannot cool, which makes integrator windup easy to
provoke on a large setpoint step. The disturbance is a heat flow into the
block (negative = a part-cooling fan). Real hotends have a nonlinear
thermistor, radiation losses and PWM heating; none of that is modelled.

### Robot arm joint (`arm`, nonlinear)

```
J q'' = tau + d - b q' - m g l sin(q),   J = m l^2 + Jm
linearised about q0:  G(s) = 1 / (J s^2 + b s + m g l cos(q0))
```

A single link with a point mass `m` at length `l`, driven by a
torque-limited joint (default +/- 2.5 N m, while holding the link horizontal
takes m g l = 1.47 N m). `q = 0` hangs straight down. Gravity makes the
effective stiffness depend on the angle, so a controller tuned at one angle
behaves differently at another. Bode, Nyquist and margins for this plant use
the model linearised about the setpoint and are labelled as such.

## Controller

Source: `src/core/pid.js`. The structure follows Astrom and Hagglund,
_Advanced PID Control_ (ISA, 2006), chapters 3 and 13.

Continuous form:

```
u = Kp (b r - y) + Ki integral(r - y) dt + Kd d/dt (c r - y) filtered by 1/(1 + Tf s) + ff
```

- **Forms.** Parallel gains (Kp, Ki, Kd) or ideal/ISA gains (Kc, Ti, Td)
  with `Kp = Kc`, `Ki = Kc/Ti`, `Kd = Kc Td`. Both produce identical outputs
  (tested). `Ti <= 0` switches integral action off.
- **Derivative filter.** `Tf = Td / N` (with `Td = Kd/Kp`; if Kp = 0,
  `Tf = Kd/N`). `N <= 0` disables the filter. Discretised with the backward
  difference, which is stable for every Tf:
  `D[k] = Tf/(Tf+T) D[k-1] + Kd/(Tf+T) (ed[k] - ed[k-1])`.
- **Setpoint weights.** `b` on the proportional term, `c` on the derivative
  term. The default `c = 0` is derivative on measurement (no derivative kick
  on setpoint steps). The weights do not change the feedback loop, so they do
  not change the margins.
- **Integral.** Forward Euler: `I[k+1] = I[k] + Ki T e[k]`.
- **Output limits and rate limit.** `u` is clamped to [umin, umax], then its
  change per sample is limited to `rate * T`.
- **Anti-windup.**
  - _Clamping_ (conditional integration): the integral is frozen while the
    output is limited and the error would drive it further into the limit.
  - _Back-calculation_: `I[k+1] = I[k] + Ki T e[k] + (T/Tt)(u[k] - v[k])`,
    where `v` is the unconstrained output and `u` the applied one. The default
    `Tt = sqrt(Ti Td)` (or `Ti/2` without derivative action), as suggested by
    Astrom and Hagglund.
  - _None_ is available to show what windup looks like.
- **Bumpless transfer.** In manual mode the operator sets the output and the
  integral tracks `I = u_manual - P - D - ff`, so the first automatic sample
  continues from the manual value. Entering manual keeps the last output.
  Changing gains while running adjusts the integral so that a change of Kp
  or b does not step the output.
- **Feed-forward.** Optional static feed-forward `ff = w * u_ss(r)`, where
  `u_ss` is the plant's steady-state input for output `r` (for example the
  gravity torque `m g l sin r` for the arm, or `(r - Ta)/(Ra Pmax)` for the
  hotend) and `w` in [0, 1] is the weight. It uses the model's own
  parameters, so it is a perfect model inverse; a real one would not be.

## Metrics

Source: `src/core/metrics.js`. Computed on the true plant output over the
window from the setpoint step to the disturbance (or the end).

| Metric             | Definition                                                                              |
| ------------------ | --------------------------------------------------------------------------------------- |
| Rise time          | 10 % to 90 % of the step, linearly interpolated                                         |
| Settling time      | from the step until `y` stays within +/- 2 % of the step (band configurable in the API) |
| Overshoot          | peak beyond the new setpoint, percent of the step                                       |
| Steady-state error | mean `r - y` over the last 10 % of the window                                           |
| IAE, ISE, ITAE     | integrals of `                                                                          | e             | `, `e^2`, `(t - t_step)                                          | e   | `   |
| Control effort     | integral of `                                                                           | u             | dt`                                                              |
| Total variation    | sum of `                                                                                | u[k] - u[k-1] | `; shows how much noise reaches the actuator                     |
| Saturation time    | time with `u` at a limit                                                                |
| Disturbance        | peak `                                                                                  | r - y         | `, IAE and recovery time into a band of 2 % of the setpoint step |

`n/a` means the quantity was never reached in the window (for example a
response that never settles).

## Stability margins

Source: `src/core/frequency.js`. The loop transfer function is evaluated as

```
L(jw) = C(e^{jwT}) * ZOH(jw) * G(jw) * 1/(1 + jw tau_s)
ZOH(jw) = (1 - e^{-jwT}) / (jwT) = e^{-jwT/2} sinc(wT/2)
```

where `C(z)` is the discrete PID feedback path exactly as implemented
(proportional, forward-Euler integral, backward-difference filtered
derivative). This is the standard continuous approximation of a sampled
loop: valid below the Nyquist frequency `pi/T`, without aliasing. The plant's
dead time and the hold's half-sample delay are added to the phase
analytically; the rational part's phase is unwrapped on a 4000-point
logarithmic grid spanning six decades below Nyquist.

- **Gain margin**: `1/|L|` where the phase crosses -180 deg (+ multiples of
  360 deg), refined by bisection; the smallest is reported.
- **Phase margin**: `180 + arg L` at `|L| = 1`; the smallest is reported.
- **Delay margin**: phase margin (rad) / gain-crossover frequency.

Phase crossings below the gain crossover where `|L| > 1` (gain-reduction
margins of conditionally stable loops, or a type-2 loop whose phase sits on
-180 deg at low frequency, such as the arm held horizontally) are not
reported as the gain margin unless the phase margin is negative. This is a
heuristic, not a Nyquist encirclement count; for an open-loop unstable plant
(none of the built-in ones is) the margins would not establish stability.

Tests compare the result with hand-derived values: an integrator with delay
(`PM = 90 deg - K theta`, `GM = pi/(2 theta K)`), `K/(s+1)^3` (`GM = 8/K` at
`sqrt 3` rad/s), and P control of an FOPDT plant.

## Identification and tuning

Source: `src/core/tuning.js`, `src/core/lab.js`. The experiments run on the
simulated plant with the configured sensor noise, quantisation and limits.

### Step test and FOPDT fit

The plant rests at the initial setpoint; the output is averaged over a
baseline (10 % of the test) to estimate `y0`; then the input steps by the
amount that would move the output to the target setpoint in steady state.

- **Two-point method** (Smith): from the times `t28` and `t63` at which the
  response reaches 28.3 % and 63.2 % of its final change,
  `tau = 1.5 (t63 - t28)`, `theta = t63 - tau`, `K = dy/du`. Fast and
  transparent, but it uses two points of a noisy curve and needs the
  response to have settled (the final value is the mean of the last 5 %).
- **Least squares**: minimises the sum of squared errors between the
  measurement and `K du (1 - exp(-(t - theta)/tau))`. For fixed
  `(tau, theta)` the optimal K has a closed form; `(log tau, theta)` are
  found by a coarse grid and refined with Nelder-Mead. Much less sensitive to
  noise than the two-point method (tested with noise), but still a
  first-order fit: higher-order or oscillatory plants are approximated, and
  the dead time absorbs the extra lags.

Warnings are attached when the fit is poor (RMS error over 3 % of the
response), when the output had not settled, and when `theta/tau < 0.05` for
the rules that then give very high gains.

### Tuning rules

| Method                                                                       | Needs      | Rule (Kc, Ti, Td)                                                                                                                                        | Known weaknesses                                                                                               |
| ---------------------------------------------------------------------------- | ---------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| `simc` (Skogestad 2003)                                                      | FOPDT fit  | PI: `Kc = tau / (K (tauc + theta))`, `Ti = min(tau, 4 (tauc + theta))`, `tauc = theta` (floored at `tau/10` here)                                        | Model-based; quality depends on the fit. PI only in this implementation (SIMC PID needs a second-order model). |
| `cohen-coon` (1953)                                                          | FOPDT fit  | PID: `Kc = (tau/(K theta)) (4/3 + theta/(4 tau))`, `Ti = theta (32 + 6 r)/(13 + 8 r)`, `Td = 4 theta/(11 + 2 r)`, `r = theta/tau`; PI and P variants too | Designed for quarter-amplitude decay: aggressive, small margins, very high gains when `theta/tau` is small.    |
| `zn-step` (Ziegler-Nichols 1942, reaction curve)                             | FOPDT fit  | PID: `Kc = 1.2 tau/(K theta)`, `Ti = 2 theta`, `Td = theta/2`                                                                                            | Same quarter-decay target as Cohen-Coon; oscillatory, sensitive to the dead-time estimate.                     |
| `zn-relay` (Ziegler-Nichols ultimate sensitivity, via Astrom-Hagglund relay) | Relay test | PID: `Kc = 0.6 Ku`, `Ti = Pu/2`, `Td = Pu/8`; PI `0.45 Ku, Pu/1.2`; P `0.5 Ku`                                                                           | Aggressive (quarter decay). Ku from the describing function is approximate (see below).                        |
| `tyreus-luyben` (1992)                                                       | Relay test | PID: `Kc = Ku/2.2`, `Ti = 2.2 Pu`, `Td = Pu/6.3`; PI `Ku/3.2, 2.2 Pu`                                                                                    | Deliberately detuned: more robust, slower disturbance rejection.                                               |

### Relay autotune

Astrom and Hagglund (1984): the loop is closed through a relay
`u = u0 +/- d` around the operating point (the target setpoint, with the
plant started at rest there). If the plant has enough phase lag, a limit
cycle appears near the frequency where the loop phase is -180 deg. The
experiment runs until the last four periods agree within 0.5 %, then

```
Pu = mean period,   Ku = 4 d / (pi sqrt(a^2 - eps^2))
```

where `a` is the measured output amplitude and `eps` the relay hysteresis
(set to `3 * noise std + quantisation step` in the app so that noise does
not cause chattering).

Weaknesses, all visible in the lab:

- `Ku` comes from the describing-function approximation, which assumes the
  plant filters the relay's square wave to a near-sinusoid. On the test plant
  `1/(s+1)^3` the estimate is 7.78 against the true 8, 2.7 % low (the test
  allows 6 %); on the default thermal
  model 2 % low (Ku 0.315 vs 0.322 from the frequency response); on the
  default FOPDT plant, whose output is closer to a triangle wave, 18 % low
  (3.39 vs 4.16).
- With hysteresis the oscillation settles where the loop phase is
  `-180 deg + asin(eps/a)`, not at the ultimate point (tested), so the
  formula gives `1/|G|` at a slightly lower frequency.
- Plants with at most two lags and no dead time (motor, mass-spring-damper,
  integrating, arm) never reach -180 deg on their own; the oscillation that
  appears is caused by the sampling delay, and its period can be a few
  controller samples. The app warns when `Pu < 10 Ts`; ZN gains from such a
  test are usually far too aggressive.
- The arm is nonlinear, so the relay limit cycle may not settle to a
  constant period; a warning is shown when it has not.

## References

- K. J. Astrom, T. Hagglund, _Advanced PID Control_, ISA, 2006.
- K. J. Astrom, T. Hagglund, "Automatic tuning of simple regulators with
  specifications on phase and amplitude margins", _Automatica_ 20(5), 1984.
- S. Skogestad, "Simple analytic rules for model reduction and PID controller
  tuning", _Journal of Process Control_ 13, 2003.
- G. H. Cohen, G. A. Coon, "Theoretical consideration of retarded control",
  _Trans. ASME_ 75, 1953.
- J. G. Ziegler, N. B. Nichols, "Optimum settings for automatic controllers",
  _Trans. ASME_ 64, 1942.
- B. D. Tyreus, W. L. Luyben, "Tuning PI controllers for integrator/dead time
  processes", _Ind. Eng. Chem. Res._ 31, 1992.
- C. L. Smith, _Digital Computer Process Control_, Intext, 1972 (two-point
  FOPDT method).
