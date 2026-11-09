# Control roadmap: sysid → pure pursuit → MPPI + UKF → dynamic model → U-MPPI

The architecture: the **Jetson Xavier NX** runs the UKF and the controller. The **ESP32** handles
PWM in/out and the driveshaft encoder. The **remote PC** only supervises; it is never in the
control loop.

Each stage below has gates that must pass before moving on. Most can be checked from logs with
the existing tools (`sysid_log.py`, `sysid_model.py`, `closed_loop.py`).
Status as of 2026-11-09: everything below has been rehearsed in simulation only.

## The UKF ↔ MPPI pairing (chosen design, "level 1")

1. **The UKF models the GPS bias.** It carries gyro bias plus GPS bias states (Gauss–Markov,
   set from the receiver spec), with lagged GPS fixes applied by rollback/replay. Modelling
   the bias keeps the position covariance honest: an unobservable bias cannot average away.
   Without it, only 63–73% of position errors fell inside 2σ (about 86% expected); with it,
   they fall inside at roughly the expected rate.
2. **MPPI tightens the track limits** by `z·σ_e`, where `σ_e` is the UKF's position standard
   deviation projected on the track normal (z = 2). This is the "use the covariance to weigh
   the path penalties" idea, as a chance constraint. It costs nothing: one projection per tick.
3. MPPI plans from the state predicted `input_delay` ahead, using the commands already sent.

Alternatives tested in sim and **not adopted** (code kept: `MPPI(scenarios=..., use_state_cov=...,
param_uncertainty=...)`, `closed_loop.py --controller mppi_scen`):
- **Independent state/parameter draws per rollout.** No safer than ignoring uncertainty: a
  control sequence paired with a lucky draw wins the weighted average.
- **Shared scenarios with risk-weighted cost.** Every control sequence is scored under the same
  4 (state, parameter) draws. This was correct but dominated by tightening: at the same pace it
  had 1–3% time over the line versus 0%.
- **Parameter uncertainty from the sysid fit** barely mattered because the fits are tight (about
  1–2%). The model error that matters is structural (tires), which sampling cannot represent.
  Revisit this when the tire model arrives, since its parameters will be far less certain.

## Simulation baseline (`python closed_loop.py`, 3 seeds, 3 flying laps)

Course: a 92 m kidney loop, 3 m wide, in the 40 × 25 m lot. Lateral acceleration cap 0.7 g for
all controllers. Each car is identified first with the designed sysid passes; the controllers
then use only the fitted parameters.

| scenario | pure pursuit | MPPI (point estimate) | MPPI + UKF (level 1) |
|---|---|---|---|
| planner model = car, GPS ±0.5 m + 0.3 m bias | 11.34 s, 0% over line | 10.59 s, **7.3%** over | 12.58 s, 0% |
| tire-model car, same GPS | 11.44 s, 0% | 10.38 s, **10.8%** over | 12.17 s, 0% |
| tire-model car, GPS ±1 m + 0.6 m bias | 11.42 s, 0.6% | 10.25 s, 13.5%, **spun (73° slip)** | 17.03 s, 1.2% |
| tire-model car, **RTK GPS** | 11.42 s, 0% | 10.12 s, 0.5% | **10.33 s, 0%** |

"Over line" means the share of time any part of the car (±0.2 m) is past the track edge.

What this says:
- **Localization limits lap time, not the controller.** With non-RTK GPS, the margin that keeps
  MPPI honest eats the racing line, and pure pursuit is as fast and simpler. With RTK, MPPI + UKF
  is about 10% faster than pure pursuit and never crosses the line. **An RTK GPS (or another
  cm-level position source) is the highest-value hardware upgrade for this goal.**
- Ignoring estimate uncertainty is not an option: point-estimate MPPI crosses the line 7–14% of
  the time and, with poor GPS, loses the car.
- On the tire-model car, side slip reaches about 6° at the 0.7 g cap. The kinematic model's
  validity ends about there (see Stage 3).

## Stages and gates

### Stage 0: system ID on the real car → gate A

Run the designed passes (`sysid_maneuvers.py`) with `sysid_logger.py`, then fit.
- [ ] Wheelbase measured (axle to axle) and set in `sysid_model.WHEELBASE_M`. Throttle/steer
      PWM channels confirmed. OLA set to ±16 g. `read_pwm.ino` flashed (millis stamps).
- [ ] Fits converge with no |correlation| > 0.95. 1σ < 5% for every gain and time constant,
      steer center ±2 µs, input delay ±10 ms.
- [ ] **Hold-out check**: refit with one pass of each kind left out and predict that pass. Speed
      RMSE ≤ 0.25 m/s and yaw-rate RMSE ≤ 0.08 rad/s, at up to about 0.5 g (sim: 0.09 m/s and
      0.02–0.05 rad/s).
- [ ] Rerun `closed_loop.py` with the real fitted parameters. The gate for MPPI + UKF is 0% over
      the line on `nominal` and `dynamic` across 3 seeds.

### Stage 1: estimator on the car → gate B (before anything drives itself)
- [ ] UKF replayed offline on a hand-driven log: GPS innovation NIS mean between 1.5 and 3
      (2 dof) and ≥ 90% of NIS < 6. Gyro bias settles within 10 s of a standstill.
- [ ] On the Xavier, sensor-to-command latency p99 < 30 ms. Logger alignment shows no "fell
      behind" warnings.
- [ ] Failsafe: losing the link or the Xavier gives neutral PWM within 100 ms (ESP32 side).
      Tested on the bench, then at walking pace.

### Stage 2: pure pursuit on the car → gate C
- [ ] 10 consecutive laps, zero cone touches, speed profile at 0.5 g, lap-time spread < 3%.
- [ ] Model holds in closed loop: speed predicted from the commands is within 0.5 m/s RMSE
      of measured over a lap, and yaw rate within 0.15 rad/s.
- [ ] Raise the profile toward 0.7 g in 0.05 g steps. This sets the pure pursuit lap to beat.

### Stage 3: MPPI + UKF level 1 on the car → gate D
- [ ] Compute: p99 tick ≤ 15 ms on the Xavier NX at K ≥ 1000 rollouts and H ≥ 60 steps
      (50 Hz with headroom). Numpy on an M5 Pro takes 9.7 ms. The Xavier's ARM CPU will be
      several times slower, so **plan on a GPU port** of `KinematicModel.step` and the cost
      (CuPy or PyTorch; the code is already batched).
- [ ] Same course, 10 laps: zero cone touches and best lap ≥ 5% faster than gate C, or
      equal with RTK absent (the sim predicts parity without RTK).
- [ ] Log the tightening margin used. If it is routinely more than half the track width,
      localization is the bottleneck (see RTK above).

### Stage 4: dynamic (tire) model. Start when ANY of these hold
- Lateral acceleration above 0.5 g is routine in the speed profile, or
- estimated side slip (GPS velocity heading minus yaw) exceeds 3°, or
- closed-loop yaw-rate prediction error exceeds 20% in corners.

Needs: the ±16 g accelerometer, a side-slip reference (RTK velocity, or a heading-capable
GPS), and cornering passes added to `sysid_maneuvers.py`. `vehicle_dynamics.DynamicPlant` is
the model structure to fit. It replaces `KinematicModel` in MPPI and drops the 0.7 g
lateral-acceleration cap.

### Stage 5: U-MPPI (sigma points along every rollout). Adopt only if ALL of these hold
- [ ] With level 1 on the car, over-the-line events are traced to **state** uncertainty:
      they coincide with high UKF σ or GPS outages, not with model error at the grip limit.
- [ ] In sim, replaying that run's uncertainty profile, U-MPPI is ≥ 3% faster than level 1
      with z re-tuned, with no more violations.
- [ ] (2n+1)·K rollouts (about 9× for a 4-dimensional planning state) fit the 15 ms GPU
      budget on the Xavier.

## Files
- `course.py`: course geometry with a fast (s, e) lookup grid and speed profile.
- `vehicle_dynamics.py`: batched sysid model, tire-model plant, `ParamUncertainty`.
- `state_estimator.py`: the UKF (gyro, encoder, lagged GPS with rollback, GPS bias states).
- `controllers.py`: `PurePursuit`, `MPPI`, `DelayCompensator`.
- `closed_loop.py`: identify → drive study; `--quick`, `--plot`, `--json`.
- `tests/test_control.py`.
