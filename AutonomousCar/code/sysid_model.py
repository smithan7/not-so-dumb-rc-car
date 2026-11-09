"""Grey-box car model driven by raw PWM, and fitting tools for system identification.

The model is the kinematic bicycle from SimulatedCarState with the actuators in front of it, so
its inputs are exactly what we log (receiver/command PWM in microseconds):

Steering   delta_cmd = clip(steer_gain * (steer_us - steer_center_us), +-delta_max)
           d(delta)/dt = clip((delta_cmd - delta) / tau_steer, +-steer_rate_max)
Yaw        r = v * tan(delta) / wheelbase / (1 + (v / v_char)^2) + gyro_bias
Throttle   u = throttle_us - throttle_center_us, with a symmetric deadband
           u > 0:  dv/dt = (speed_gain * u - v) / tau_speed      (ESC ~ sets motor voltage)
           u < 0:  dv/dt = -brake_gain * |u|                     (ESC brake, while moving)
           always: dv/dt -= roll_decel + drag_coeff * v^2        (while moving), v >= 0

Longitudinal and lateral are fitted separately: speed from throttle, then yaw rate from steering
using the *measured* speed. That keeps each problem small and well conditioned.

wheelbase is fixed, not fitted: from yaw rate alone only steer_gain / wheelbase is identifiable.
"""

import math
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares

from CarParameters import CarParam

# name: (default, lower, upper, unit, description)
PARAMS = {
    # steering - defaults from the hand-measured bench points in benchtop_calibration_code
    "steer_center_us": (1500.0, 1400.0, 1600.0, "us", "PWM giving zero steering"),
    "steer_gain": (math.radians(0.1), 1e-4, 1e-2, "rad/us", "steering angle per us of PWM"),
    "delta_max": (math.radians(22.5), 0.1, 0.8, "rad", "steering lock"),
    "tau_steer": (0.08, 0.005, 1.0, "s", "servo first-order time constant"),
    "steer_rate_max": (6.0, 0.5, 30.0, "rad/s", "servo slew-rate limit"),
    "v_char": (CarParam.characteristic_velocity, 2.0, 200.0, "m/s", "side-slip speed (yaw-rate roll-off)"),
    "gyro_bias": (0.0, -0.2, 0.2, "rad/s", "gyro z bias"),
    "input_delay": (0.03, 0.0, 0.3, "s", "PWM -> actuator transport delay"),
    # longitudinal - defaults are placeholders until the first fit
    "throttle_center_us": (1500.0, 1450.0, 1550.0, "us", "ESC neutral"),
    "deadband_us": (40.0, 0.0, 120.0, "us", "ESC deadband either side of neutral"),
    "speed_gain": (0.15, 0.01, 1.0, "(m/s)/us", "steady-state speed per us past the deadband"),
    "tau_speed": (0.8, 0.05, 5.0, "s", "speed time constant under throttle"),
    "brake_gain": (0.08, 0.0, 1.0, "(m/s^2)/us", "brake deceleration per us past the deadband"),
    "roll_decel": (0.3, 0.0, 3.0, "m/s^2", "rolling resistance deceleration"),
    "drag_coeff": (0.01, 0.0, 0.2, "1/m", "aero drag, decel = drag_coeff * v^2"),
}

LATERAL_FREE = ["steer_center_us", "steer_gain", "tau_steer", "v_char", "gyro_bias", "input_delay"]
LONGITUDINAL_FREE = ["deadband_us", "speed_gain", "tau_speed", "brake_gain", "roll_decel", "drag_coeff"]

# CarParam.length (28 in) looks like overall length, not axle-to-axle. MEASURE the wheelbase.
WHEELBASE_M = CarParam.length


def default_params(**overrides):
    p = {k: v[0] for k, v in PARAMS.items()}
    unknown = set(overrides) - set(p)
    if unknown:
        raise KeyError(f"unknown parameters: {sorted(unknown)}")
    p.update(overrides)
    return p


# ---------------------------------------------------------------------------------------------
# inputs
# ---------------------------------------------------------------------------------------------

def input_signal(t_in, u_in, neutral=1500.0):
    """Callable u(t) by linear interpolation of logged PWM. NaN (signal lost) -> neutral, which is
    what the receiver failsafe commands. Linear (not zero-order hold) so the fitted delay has a
    gradient; at 20 Hz logging the difference is small."""
    t_in = np.asarray(t_in, dtype=float)
    u = np.asarray(u_in, dtype=float).copy()
    u[~np.isfinite(u)] = neutral
    order = np.argsort(t_in)
    t_in, u = t_in[order], u[order]

    def f(t):
        return np.interp(t, t_in, u)

    return f


def _deadband(u, db):
    return np.sign(u) * np.maximum(np.abs(u) - db, 0.0)


# ---------------------------------------------------------------------------------------------
# simulation
# ---------------------------------------------------------------------------------------------

def simulate_speed(p, t_grid, throttle, v0=0.0):
    """Speed v(t) on t_grid from throttle(t) [callable, us]. Explicit Euler on the grid."""
    t_grid = np.asarray(t_grid, dtype=float)
    u = _deadband(throttle(t_grid - p["input_delay"]) - p["throttle_center_us"], p["deadband_us"])
    v = np.empty_like(t_grid)
    v[0] = v0
    sg, ts, bg = p["speed_gain"], p["tau_speed"], p["brake_gain"]
    rd, dc = p["roll_decel"], p["drag_coeff"]
    for k in range(len(t_grid) - 1):
        dt = t_grid[k + 1] - t_grid[k]
        vk, uk = v[k], u[k]
        a = 0.0
        if uk > 0.0:
            a = (sg * uk - vk) / ts
        elif uk < 0.0 and vk > 0.0:
            a = bg * uk
        if vk > 0.0:
            a -= rd + dc * vk * vk
        vn = vk + a * dt
        v[k + 1] = vn if vn > 0.0 else 0.0
    return v


def simulate_steering(p, t_grid, steer):
    """Front steering angle delta(t) on t_grid from steer(t) [callable, us]."""
    t_grid = np.asarray(t_grid, dtype=float)
    cmd = np.clip(p["steer_gain"] * (steer(t_grid - p["input_delay"]) - p["steer_center_us"]),
                  -p["delta_max"], p["delta_max"])
    d = np.empty_like(t_grid)
    d[0] = cmd[0]
    tau, rmax = p["tau_steer"], p["steer_rate_max"]
    for k in range(len(t_grid) - 1):
        dt = t_grid[k + 1] - t_grid[k]
        rate = (cmd[k] - d[k]) / tau
        rate = rmax if rate > rmax else (-rmax if rate < -rmax else rate)
        # never overshoot the command in one step (keeps coarse grids stable)
        step = rate * dt
        if abs(step) > abs(cmd[k] - d[k]):
            step = cmd[k] - d[k]
        d[k + 1] = d[k] + step
    return d


def yaw_rate(p, v, delta, wheelbase=WHEELBASE_M):
    return v * np.tan(delta) / wheelbase / (1.0 + (v / p["v_char"]) ** 2) + p["gyro_bias"]


def simulate(p, t_grid, throttle, steer, v0=0.0, x0=(0.0, 0.0, 0.0), wheelbase=WHEELBASE_M):
    """Full open-loop run -> dict of arrays: t, v, delta, r (yaw rate, incl. gyro bias), x, y, psi."""
    t_grid = np.asarray(t_grid, dtype=float)
    v = simulate_speed(p, t_grid, throttle, v0)
    delta = simulate_steering(p, t_grid, steer)
    r = yaw_rate(p, v, delta, wheelbase)
    dt = np.diff(t_grid, prepend=t_grid[0])
    psi = x0[2] + np.cumsum((r - p["gyro_bias"]) * dt)
    x = x0[0] + np.cumsum(v * np.cos(psi) * dt)
    y = x0[1] + np.cumsum(v * np.sin(psi) * dt)
    return {"t": t_grid, "v": v, "delta": delta, "r": r, "x": x, "y": y, "psi": psi}


# ---------------------------------------------------------------------------------------------
# fitting
# ---------------------------------------------------------------------------------------------

@dataclass
class FitResult:
    params: dict
    free: list
    std: dict            # 1-sigma from the Jacobian (assumes white residuals - treat as optimistic)
    corr: np.ndarray     # correlation between free parameters
    rmse: float
    n: int
    success: bool
    message: str
    residuals: np.ndarray = field(repr=False, default=None)

    def table(self, truth=None):
        rows = [f"{'param':>18} {'value':>12} {'1-sigma':>10} {'rel':>7}" + ("  truth      err" if truth else "")]
        for k in self.free:
            v, s = self.params[k], self.std[k]
            rel = abs(s / v) if v else math.inf
            line = f"{k:>18} {v:12.5g} {s:10.3g} {rel:6.1%}"
            if truth:
                tv = truth[k]
                err = (v - tv) / tv if tv else v - tv
                line += f"  {tv:<10.5g} {err:+.1%}" if tv else f"  {tv:<10.5g} {err:+.3g}"
            rows.append(line)
        rows.append(f"rmse {self.rmse:.4g}, n {self.n}" + ("" if self.success else f"  NOT CONVERGED: {self.message}"))
        return "\n".join(rows)

    def strongly_correlated(self, threshold=0.9):
        out = []
        for i in range(len(self.free)):
            for j in range(i + 1, len(self.free)):
                if abs(self.corr[i, j]) >= threshold:
                    out.append((self.free[i], self.free[j], float(self.corr[i, j])))
        return out


def _fit(residual_fn, p0, free, loss="soft_l1", f_scale=1.0):
    lo = np.array([PARAMS[k][1] for k in free])
    hi = np.array([PARAMS[k][2] for k in free])
    x0 = np.clip(np.array([p0[k] for k in free], dtype=float), lo + 1e-12, hi - 1e-12)
    scale = np.maximum(np.abs(x0), 1e-3 * (hi - lo))

    def unpack(z):
        p = dict(p0)
        p.update(zip(free, z * scale))
        return p

    res = least_squares(lambda z: residual_fn(unpack(z)), x0 / scale, bounds=(lo / scale, hi / scale),
                        loss=loss, f_scale=f_scale, x_scale="jac", diff_step=1e-4)
    p = unpack(res.x)
    r = residual_fn(p)
    n, m = r.size, len(free)
    J = res.jac / scale  # d r / d param (unscaled)
    sigma2 = float(r @ r) / max(n - m, 1)
    try:
        cov = np.linalg.pinv(J.T @ J) * sigma2
    except np.linalg.LinAlgError:
        cov = np.full((m, m), np.nan)
    sd = np.sqrt(np.clip(np.diag(cov), 0, None))
    with np.errstate(invalid="ignore", divide="ignore"):
        corr = cov / np.outer(sd, sd)
    return FitResult(params=p, free=list(free), std=dict(zip(free, sd)), corr=corr,
                     rmse=float(np.sqrt(np.mean(r ** 2))), n=n, success=bool(res.success),
                     message=res.message, residuals=r)


def fit_longitudinal(segments, p0=None, free=LONGITUDINAL_FREE, dt=0.01):
    """Fit speed parameters. segments: list of dicts with
    t_cmd, throttle_us (logged command), t_meas, v_meas (measured speed, m/s).
    Each segment starts from its first measured speed."""
    p0 = default_params(**(p0 or {}))
    prepared = []
    for s in segments:
        t_meas = np.asarray(s["t_meas"], float)
        grid = np.arange(t_meas[0], t_meas[-1] + dt, dt)
        prepared.append((grid, input_signal(s["t_cmd"], s["throttle_us"], p0["throttle_center_us"]),
                         t_meas, np.asarray(s["v_meas"], float)))

    def residuals(p):
        out = []
        for grid, thr, t_meas, v_meas in prepared:
            v = simulate_speed(p, grid, thr, v0=max(v_meas[0], 0.0))
            out.append(np.interp(t_meas, grid, v) - v_meas)
        return np.concatenate(out)

    return _fit(residuals, p0, free, f_scale=0.3)


def fit_lateral(segments, p0=None, free=LATERAL_FREE, wheelbase=WHEELBASE_M, dt=0.005):
    """Fit steering/yaw parameters. segments: list of dicts with
    t_cmd, steer_us (logged command), t_meas, yaw_rate (gyro z, rad/s), v_meas (speed at t_meas)."""
    p0 = default_params(**(p0 or {}))
    prepared = []
    for s in segments:
        t_meas = np.asarray(s["t_meas"], float)
        grid = np.arange(t_meas[0] - 0.5, t_meas[-1] + dt, dt)  # 0.5 s lead-in for the servo state
        prepared.append((grid, input_signal(s["t_cmd"], s["steer_us"], p0["steer_center_us"]),
                         t_meas, np.asarray(s["yaw_rate"], float), np.asarray(s["v_meas"], float)))

    def residuals(p):
        out = []
        for grid, steer, t_meas, r_meas, v_meas in prepared:
            delta = np.interp(t_meas, grid, simulate_steering(p, grid, steer))
            out.append(yaw_rate(p, v_meas, delta, wheelbase) - r_meas)
        return np.concatenate(out)

    return _fit(residuals, p0, free, f_scale=0.05)
