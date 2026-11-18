# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/).

## [0.1.0] - 2026-11-19

First release.

### Added

- Six simulated plants with documented equations: first order plus dead time,
  mass-spring-damper, DC motor with Coulomb friction, integrating axis,
  three-node hotend thermal model, nonlinear arm joint.
- Fixed-step RK4 simulation with sub-steps, dead-time buffer, controller
  sample time separate from the plant step, seeded sensor noise,
  quantisation and sensor lag.
- Discrete PID in parallel and ideal form with filtered derivative,
  setpoint weights, output and rate limits, clamping and back-calculation
  anti-windup, bumpless manual/auto transfer and feed-forward.
- Step and disturbance metrics, and gain, phase and delay margins from the
  sampled-loop frequency response.
- Step-test FOPDT identification (two-point and least squares), relay
  autotune, and SIMC, Cohen-Coon, Ziegler-Nichols and Tyreus-Luyben rules
  with warnings when a test does not fit a rule's assumptions.
- Browser app with live strip charts, compare mode, Bode and Nyquist plots,
  shareable URLs, keyboard shortcuts, phone layout and light/dark themes.
- Headless CLI (`run`, `tune`, `plants`) with text, JSON and CSV output.
- Tests with `node:test`, JSDoc type checking, CI on Ubuntu and Windows with
  Node 22 and 24, a headless Chrome smoke test, and GitHub Pages deployment.

[0.1.0]: https://github.com/projectbin07-stack/pid-tuning-lab/releases/tag/v0.1.0
