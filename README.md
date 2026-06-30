# CP27E Torque Vectoring (MATLAB/Simulink)

Torque vectoring control development for Cal Poly Racing's CP27E Formula SAE
electric vehicle, running four AMK DD5-14-10-POW hub motors (4WD) via
KW26-S5-FSE-4Q inverters. This repo holds the MATLAB/Simulink simulation and
control-design work: a single-track (bicycle model) plant with direct
yaw-moment input for initial controller development, progressing to a
nonlinear double-track model with Pacejka tire data from the FSAE Tire Test
Consortium. Covers yaw-rate reference generation, feedforward+PID (and later
LQR/SMC) yaw-moment control, and torque allocation across the four motors
under friction-circle and motor-saturation constraints, with the goal of
validating a controller architecture suitable for deployment to the
vehicle's VCU.

## Getting started

1. Open `CP27E-TV.prj` in MATLAB — this sets the path and runs `config/startup.m`.
2. Run `scripts/run_sim.m` to simulate the default maneuver (step steer) on
   the current plant.
3. See `models/` for the Simulink models and `src/` for the underlying
   algorithm code (reference generator, plant dynamics, allocator).

## Structure

```
config/      project startup/shutdown, path setup
data/        vehicle/controller parameters, tire data + fits
models/      .slx models (controller, plant(s), top-level harness)
src/         algorithm code as plain .m files (referenced by MATLAB Function blocks)
lib/         reusable block libraries
maneuvers/   test maneuver definitions (step steer, ramp to limit, skidpad)
scripts/     entry points (run_sim.m) and post-processing
tests/       unit/regression tests on controller logic
results/     run outputs, figures, logs (gitignored)
codegen/     auto-coded VCU output (gitignored, added later)
```

## Status

Stage 0/1: single-track (bicycle) plant + direct yaw-moment input, feedforward
+ PID yaw-rate controller, simple proportional torque allocator across four
motors with saturation. See `models/cp27e_tv_top.slx`.

Next: lateral load transfer, double-track plant with combined-slip Pacejka
tires, QP/WLS allocation, slip-ratio traction control, sideslip estimation.
