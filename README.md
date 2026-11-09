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
