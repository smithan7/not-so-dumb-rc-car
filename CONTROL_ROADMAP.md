# Control roadmap: sysid → pure pursuit → MPPI + UKF → dynamic model → U-MPPI

The architecture: the **Jetson Xavier NX** runs the UKF and the controller. The **ESP32** handles
PWM in/out and the driveshaft encoder. The **remote PC** only supervises; it is never in the
control loop.

Each stage below has gates that must pass before moving on. Most can be checked from logs with
the existing tools (`sysid_log.py`, `sysid_model.py`, `closed_loop.py`).
Status as of 2026-11-09: everything below has been rehearsed in simulation only.
Hardware plan: RTK GPS + Intel RealSense L515 on the Xavier.

## Sensors: RTK GPS + RealSense L515 (decided 2026-11-09)

- **RTK GPS** is the primary position source. It is what makes a racing line possible on a 3 m track.
- **The L515 measures the car's lateral offset relative to the track**, from cones or track
  edges, as a map-relative measurement in the UKF. It is not biased like GPS, so it keeps the
  estimate honest when RTK drops from fixed to float (near buildings, trees, cars). The L515 is
  an indoor sensor, so sunlight range is the open question; see gate B.
- **The UKF uses the receiver's reported fix type.** When the mode changes (fixed ↔ float), it
  resets its GPS-bias state to that mode's spread.

### Cone plan: brightly coloured cones on black asphalt
- **Detect in the L515's RGB stream, then range.** Bright cones on black asphalt are close to
  ideal for a hue/saturation threshold, and that works in full sun. Use a short manual exposure
  against motion blur. Range comes from L515 depth on the cone pixels when it returns.
  Otherwise, project the cone's base pixel onto the ground plane: the lot is flat and the
  camera height and pitch are fixed. This means sunlight washing out the depth (black asphalt
  returns little IR anyway) degrades accuracy but does not kill the sensor.
- **Two colours, one per side** (e.g. left orange, right yellow). Left versus right then never
  depends on the position estimate.
- **Survey every cone with the RTK rover** to make a cone map. A detection then becomes a
  range/bearing landmark measurement against a mapped cone. That corrects along-track position
  and heading as well as lateral offset, which is better than the lateral-only measurement
  rehearsed in sim. Data association is easy: nearest same-colour mapped cone, with an
  innovation gate. The gate also rejects cones that got knocked over.
- Spacing target: about 3–4 m per side on straights and tighter in corners (about 50–60 cones
  for the 92 m loop). Placement will be approximate. That is fine because **the survey is the
  truth**: the cone map, not the designed spline, defines the track edges the planner uses, and
  the course is rebuilt from it. Check the surveyed map for gaps; a gap > 6 m on one side
  means no landmarks there, so add a cone. Use tall cones (≥ 30 cm), not flat discs, so they
  show above the asphalt at 3–8 m.

## The UKF ↔ MPPI pairing (chosen design, "level 1")

1. **UKF**: gyro + encoder + GPS (lagged fixes applied at their fix time by rollback/replay) +
   optional L515 lateral offset. It carries gyro-bias and GPS-bias states (GPS bias reset on a
   fix-mode change). It also has side-velocity process noise scaled by lateral acceleration,
   because the tires slide in corners and a heading-only motion model was overconfident
   whenever GPS degraded. All of this keeps the covariance honest, which matters because:
2. **MPPI tightens the track limits** by `z·σ_e`, where `σ_e` is the UKF's position standard
   deviation projected on the track normal (z = 2). This is the "use the covariance to weigh
   the path penalties" idea, as a chance constraint. It costs nothing: one projection per tick.
   When the margin eats more than half the track, MPPI caps speed in proportion.
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
then use only the fitted parameters. In every scenario except the first, the car has saturating
tires that the planner does not model. "Over" means the share of time any part of the car
(±0.2 m) is past the track edge.

| position sensing | pure pursuit | MPPI (point estimate) | MPPI + UKF (level 1) |
|---|---|---|---|
| plain GPS ±0.5 m + 0.3 m bias, planner = car | 11.35 s, 0% | 10.56 s, **4.6%** over | 12.63 s, 0% |
| plain GPS | 11.45 s, 0% | 10.48 s, **7.6%** | 12.90 s, 0% |
| poor GPS ±1 m + 0.6 m bias | 11.44 s, 0.5% | 10.28 s, **12.9%** | 21.1 s, **8.9%** (unusable) |
| plain GPS + L515 | 11.44 s, 0% | 10.29 s, 1.2% | 10.67 s, 0% |
| RTK | 11.44 s, 0% | 10.22 s, 0.9% | 10.48 s, 0% |
| **RTK + L515** | 11.44 s, 0% | 10.20 s, 0.5% | **10.33 s, 0%** |
| RTK with float dropouts (8 s every 25 s) | 11.53 s, **spun once** | 10.15 s, 4.6% | 10.44 s, 0.8% |
| **RTK with dropouts + L515** | 11.44 s, 0% | 10.33 s, 0.7% | **10.53 s, 0%** |

What this says:
- **Localization limits lap time, not the controller.** With plain GPS, the margin that keeps MPPI
  honest eats the racing line, and pure pursuit is faster. With RTK, MPPI + UKF is about 10%
  faster than pure pursuit and stays inside the lines.
- **The L515 earns its place during RTK dropouts.** Without it, MPPI + UKF crosses the line, and
  pure pursuit spun once when RTK recovered: the estimate jumped about 0.5 m and pure pursuit
  steered hard at speed. With it, all controllers stay clean.
- Ignoring estimate uncertainty is not an option. With poor GPS, nothing should race: fall back
  to slow pure pursuit or stop (see gate B).
- On the tire-model car, side slip reaches about 6° at the 0.7 g cap. The kinematic model's
  validity ends about there (see Stage 4).

**How good must the L515 lateral measurement be?** From a sweep with plain GPS, run before the
last filter refinements, so treat it as approximate:

| lateral noise | frames with cones detected | rate | MPPI + UKF lap | over |
|---|---|---|---|---|
| 0.1 m | 70% | 15 Hz | 10.43 s | 0% |
| 0.2 m | 70% | 15 Hz | 10.62 s | 0% |
| 0.3 m | 70% | 15 Hz | 10.81 s | 0% |
| 0.1 m | 30% | 15 Hz | 10.47 s | 0% |
| 0.1 m | 10% | 15 Hz | 10.83 s | 0% |
| 0.2 m | 30% | 5 Hz | 11.10 s | 0.1% |

Even a weak measurement helps. Target **≤ 0.2 m at ≥ 30% of frames, ≥ 10 Hz**.

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
- [ ] RTK: fixed for ≥ 90% of a lap around the lot. The fix type (GGA quality 4/5) and the
      accuracy fields are logged and fed to the UKF (mode-change bias reset).
- [ ] Cones on the asphalt, at the time of day you drive. RGB colour detection: ≥ 95% of cones
      within 3–8 m detected, colour (side) right ≥ 99%, and no false detections in shadows or on
      paint lines. Range error ≤ 0.2 m at 3 m and ≤ 0.4 m at 6 m against a tape measure. Record
      both the L515 depth and the ground-plane range, and note how often depth returns in sun.
      At least one cone in view in ≥ 30% of frames around the lap.
- [ ] Cone map: every cone surveyed with the RTK rover (fixed). Re-survey after any cone is hit.
- [ ] Extrinsics: L515 pose on the car measured (lever arm and yaw). A 1° yaw error is 5 cm
      at 3 m.
- [ ] UKF replayed offline on a hand-driven log: GPS innovation NIS mean between 1.5 and 3
      (2 dof) and ≥ 90% of NIS < 6. Gyro bias settles within 10 s of a standstill.
- [ ] On the Xavier, sensor-to-command latency p99 < 30 ms. Logger alignment shows no "fell
      behind" warnings.
- [ ] Failsafe: losing the link or the Xavier gives neutral PWM within 100 ms (ESP32 side).
      Tested on the bench, then at walking pace.
- [ ] Localization fallback: if `σ_e` exceeds a quarter of the track width for more than 1 s
      (RTK lost and no cones in view), the car drops to a slow pure-pursuit or stop mode.

### Stage 2: pure pursuit on the car → gate C
- [ ] 10 consecutive laps, zero cone touches, speed profile at 0.5 g, lap-time spread < 3%.
- [ ] Model holds in closed loop: speed predicted from the commands is within 0.5 m/s RMSE
      of measured over a lap, and yaw rate within 0.15 rad/s.
- [ ] Raise the profile toward 0.7 g in 0.05 g steps. This sets the pure pursuit lap to beat.
- [ ] Estimate-jump tolerance: inject a 0.5 m step into the estimate (sim first, then at low
      speed). Steering must stay within the servo slew limit with no spin, since an RTK
      recovery does exactly this. Pure pursuit failed this in sim once; rate-limit its
      steering or blend the correction before running it fast.

### Stage 3: MPPI + UKF level 1 on the car → gate D
- [ ] Compute: p99 tick ≤ 15 ms on the Xavier NX at K ≥ 1000 rollouts and H ≥ 60 steps
      (50 Hz with headroom). Numpy on an M5 Pro takes 9.7 ms. The Xavier's ARM CPU will be
      several times slower, so **plan on a GPU port** of `KinematicModel.step` and the cost
      (CuPy or PyTorch; the code is already batched).
- [ ] Same course, 10 laps: zero cone touches and best lap ≥ 5% faster than gate C (sim with
      RTK + L515: about 10%).
- [ ] Log the tightening margin used. If it is routinely more than half the track width,
      localization is the bottleneck (see RTK above).

### Stage 4: dynamic (tire) model. Start when ANY of these hold
- Lateral acceleration above 0.5 g is routine in the speed profile, or
- estimated side slip (GPS velocity heading minus yaw) exceeds 3°, or
- closed-loop yaw-rate prediction error exceeds 20% in corners.

Needs: the ±16 g accelerometer, a side-slip reference (RTK velocity versus the
gyro-integrated heading), and cornering passes added to `sysid_maneuvers.py`. `vehicle_dynamics.DynamicPlant` is
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
- `state_estimator.py`: the UKF (gyro, encoder, lagged GPS with rollback, fix-mode-aware GPS bias,
  L515 lateral offset).
- `controllers.py`: `PurePursuit`, `MPPI`, `DelayCompensator`.
- `closed_loop.py`: identify → drive study; `--quick`, `--plot`, `--json`.
- `tests/test_control.py`.
