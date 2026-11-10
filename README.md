# not-so-dumb-rc-car

This is my attempt to build a not-so-dumb RC car: a 1/7-scale (Arrma Infraction class) car with
physics models for simulation and controller tuning, plus logging/telemetry tools for the real car.

## Layout

```
AutonomousCar/
  code/                 Python models, tools and notebooks
    units.py            shared unit conversions (a_to_b = multiply a by this to get b)
    Motor.py, Battery.py, Tire.py, DriveTrain.py   component models (+ *Params.py)
    SimulatedCarState.py  kinematic bicycle model with side-slip gain
    PID_Controller.py, CarController.py, step_response.py
    Track.py            track geometry, nearest-edge search, curvature speed profile
    data_collect_GUI*.py  Tk telemetry GUIs (serial / UDP)
    read_pwm/read_pwm.ino  Arduino/ESP32 RC-receiver PWM reader
    tests/              pytest suite for the models
    *.ipynb             experiments, calibration and optimization notebooks
  archived_code/        older experiments
  data/                 track data (Laguna Seca), test circles, terrain
```

## Setup

```sh
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

Notebooks import the modules directly, so start Jupyter from `AutonomousCar/code/`.

## Logging for system ID

```sh
cd AutonomousCar/code
python sysid_logger.py --duration 300 --notes "throttle steps, parking lot"   # auto-detects OLA/GPS/PWM ports
python sysid_logger.py --port ola=/dev/tty.usbmodem1101 --port pwm=/dev/tty.usbserial-0001
```

Each run writes `AutonomousCar/logs/session_<time>/` with one raw CSV per serial stream and `meta.json`.
Load and time-align it in a notebook:

```python
from sysid_log import load_session, summarize, ola_to_si
s = load_session("../logs/session_20261108_120000")
print(summarize(s))          # rates, gaps, and per-stream latency; flags a logger that fell behind
imu = ola_to_si(s["ola"])    # m/s^2 and rad/s; accel range auto-detected, gyro range must match OLA config
s["pwm"], s["gps"]           # all share one time column, "t" (s since session start)
```

`load_legacy_pickles()` re-times the 2023 `*_data.pkl` logs the same way.

## System ID model and test plan

- `sysid_model.py`: grey-box model driven by raw PWM (servo + ESC in front of the bicycle model)
  and `fit_longitudinal` / `fit_lateral` (scipy least squares, with 1-sigma and correlations).
- `sysid_maneuvers.py`: the test passes (`longitudinal_passes`, `lateral_passes`), checked against the
  test area with `check_fits_area`, plus synthetic sensors to rehearse fits.
- `python sysid_identifiability.py`: Monte Carlo study of how well each parameter is pinned down
  by the designed passes vs. manual driving, with encoder vs. GPS-only speed.

Measure the wheelbase and set `sysid_model.WHEELBASE_M`: yaw rate only identifies steer_gain / wheelbase.

## Control (simulation)

UKF state estimation, plus pure pursuit and MPPI controllers, rehearsed end to end: identify the
simulated car, then drive laps on the estimate. See [CONTROL_ROADMAP.md](CONTROL_ROADMAP.md) for
the design, the results and the gates for moving each stage onto the real car.

```sh
cd AutonomousCar/code
python closed_loop.py --quick              # ~30 s smoke run
python closed_loop.py --plot paths.png     # full study, 4 scenarios x 3 controllers x 3 seeds
```

## Tests

```sh
cd AutonomousCar/code && python -m pytest tests
```

## Notes

- `PID.tic(s, s_d, dt=1.0)`: pass the real `dt` for gains that don't depend on loop rate. The
  default of 1.0 matches the original per-tick behaviour. `max_error_sum` (anti-windup) defaults
  to unlimited.
- `Motor.calc_motor_torque` is `kt * I`. Use `calc_current_from_voltage(V, rpm)` /
  `calc_max_torque_at_rpm(V, rpm)` for the back-EMF limited torque-speed curve.
- `Track` detects closed circuits and wraps nearest-edge searches and the speed profile around
  the start/finish line.
