"""Car models for control: the sysid model batched for rollouts, and a dynamic tire model 'true car'.

KinematicModel  - exactly the PWM-driven model in sysid_model.py (same parameters), stepped for
                  thousands of rollouts at once. This is what the controller plans with.
DynamicPlant    - dynamic bicycle with saturating (simplified Pacejka) tires, same actuators
                  and ESC. Used as the simulated real car so the controller faces a model it
                  does not have: tire saturation, side slip, yaw transients. Its linear region
                  matches the kinematic model's v_char understeer term, so the mismatch is
                  concentrated where it matters - near the grip limit.
ParamUncertainty - a Gaussian over sysid parameters built from FitResult covariances, sampled
                  per rollout by the controller.
"""

import math

import numpy as np

import sysid_model as M

G = 9.81
STATE = ("x", "y", "psi", "v", "delta")  # kinematic state used by the planner


def _deadband(u, db):
    return np.sign(u) * np.maximum(np.abs(u) - db, 0.0)


def longitudinal_accel(p, throttle_us, v):
    """ESC model from sysid_model.simulate_speed, as an acceleration (works on arrays)."""
    u = _deadband(throttle_us - p["throttle_center_us"], p["deadband_us"])
    a = np.where(u > 0.0, (p["speed_gain"] * u - v) / p["tau_speed"],
                 np.where((u < 0.0) & (v > 0.0), p["brake_gain"] * u, 0.0))
    return a - np.where(v > 0.0, p["roll_decel"] + p["drag_coeff"] * v * v, 0.0)


def steer_command(p, steer_us):
    return np.clip(p["steer_gain"] * (steer_us - p["steer_center_us"]), -p["delta_max"], p["delta_max"])


def servo_step(p, delta, steer_us, dt):
    cmd = steer_command(p, steer_us)
    err = cmd - delta
    step = np.clip(err / p["tau_steer"], -p["steer_rate_max"], p["steer_rate_max"]) * dt
    return delta + np.where(np.abs(step) > np.abs(err), err, step)


class KinematicModel:
    """Batched sysid model. Parameters may be scalars or (K,) arrays (one set per rollout)."""

    def __init__(self, params=None, wheelbase=M.WHEELBASE_M):
        self.p = dict(params or M.default_params())
        self.wheelbase = wheelbase

    def yaw_rate(self, v, delta, p=None):
        p = p or self.p
        return v * np.tan(delta) / self.wheelbase / (1.0 + (v / p["v_char"]) ** 2)

    def step(self, x, throttle_us, steer_us, dt, p=None):
        """x: (..., 5) [x, y, psi, v, delta]. Commands are what the actuators see (already delayed)."""
        p = p or self.p
        X, Y, psi, v, d = (x[..., i] for i in range(5))
        r = self.yaw_rate(v, d, p)
        a = longitudinal_accel(p, throttle_us, v)
        out = np.empty_like(x)
        out[..., 0] = X + v * np.cos(psi + 0.5 * r * dt) * dt
        out[..., 1] = Y + v * np.sin(psi + 0.5 * r * dt) * dt
        out[..., 2] = psi + r * dt
        out[..., 3] = np.maximum(v + a * dt, 0.0)
        out[..., 4] = servo_step(p, d, steer_us, dt)
        return out, r, a


class DynamicPlant:
    """Single-track model with saturating tires. State: x, y, psi, vx, vy, r, delta.

    Tire: Fy = mu * Fz * sin(C * atan(B * alpha)), with B set so the cornering stiffness
    reproduces the kinematic model's v_char in the linear region. Below ~1 m/s the slip
    equations are ill-conditioned, so the state is blended toward the kinematic solution.
    """

    def __init__(self, params=None, wheelbase=M.WHEELBASE_M, mass=7.7, mu=0.9, pacejka_c=1.5,
                 front_stiffness_per_load=12.0, yaw_inertia_scale=0.3):
        self.p = dict(params or M.default_params())
        self.L = wheelbase
        self.m, self.mu = mass, mu
        self.lf = self.lr = wheelbase / 2
        self.Iz = mass * (yaw_inertia_scale * wheelbase) ** 2
        self.Fzf = mass * G * self.lr / wheelbase
        self.Fzr = mass * G * self.lf / wheelbase
        # linear understeer gradient K = m/L (lr/Cf - lf/Cr) and v_char^2 = L / K
        self.Cf = front_stiffness_per_load * self.Fzf
        K = wheelbase / self.p["v_char"] ** 2
        inv_cr = self.lr / self.Cf / self.lf - K * wheelbase / (mass * self.lf)
        if inv_cr <= 0:
            raise ValueError("v_char too low for this front stiffness (needs oversteer)")
        self.Cr = 1.0 / inv_cr
        self.C = pacejka_c
        self.Bf = self.Cf / (pacejka_c * mu * self.Fzf)
        self.Br = self.Cr / (pacejka_c * mu * self.Fzr)
        self.state = np.zeros(7)

    def reset(self, x=0.0, y=0.0, psi=0.0, v=0.0):
        self.state = np.array([x, y, psi, v, 0.0, 0.0, 0.0])

    def _tire(self, B, Fz, alpha):
        return self.mu * Fz * np.sin(self.C * np.arctan(B * alpha))

    def step(self, throttle_us, steer_us, dt):
        X, Y, psi, vx, vy, r, d = self.state
        p = self.p
        ax = longitudinal_accel(p, throttle_us, vx)
        if vx > 0.3:
            af = d - math.atan2(vy + self.lf * r, vx)
            ar = -math.atan2(vy - self.lr * r, vx)
            Fyf, Fyr = self._tire(self.Bf, self.Fzf, af), self._tire(self.Br, self.Fzr, ar)
            vx_dot = ax + vy * r - Fyf * math.sin(d) / self.m
            vy_dot = (Fyf * math.cos(d) + Fyr) / self.m - vx * r
            r_dot = (self.lf * Fyf * math.cos(d) - self.lr * Fyr) / self.Iz
        else:
            vx_dot, vy_dot, r_dot = ax, 0.0, 0.0
        vx_n = max(vx + vx_dot * dt, 0.0)
        vy_n, r_n = vy + vy_dot * dt, r + r_dot * dt
        # low-speed blend toward the no-slip solution
        lam = min(max((vx_n - 0.5) / 1.0, 0.0), 1.0)
        r_kin = vx_n * math.tan(d) / self.L
        vy_kin = self.lr * r_kin
        vy_n = lam * vy_n + (1 - lam) * vy_kin
        r_n = lam * r_n + (1 - lam) * r_kin
        d_n = float(servo_step(p, d, steer_us, dt))
        c, s = math.cos(psi), math.sin(psi)
        self.state = np.array([X + (vx * c - vy * s) * dt, Y + (vx * s + vy * c) * dt, psi + r * dt,
                               vx_n, vy_n, r_n, d_n])
        return self.state

    # what the sensors see
    @property
    def yaw_rate(self):
        return self.state[5]

    @property
    def wheel_speed(self):
        return self.state[3]

    @property
    def lateral_accel(self):
        return self.state[3] * self.state[5]

    @property
    def sideslip(self):
        return math.atan2(self.state[4], max(self.state[3], 0.1))


class KinematicPlant:
    """The planner's own model as the 'real car' (no structural mismatch) - same API as DynamicPlant."""

    def __init__(self, params=None, wheelbase=M.WHEELBASE_M):
        self.model = KinematicModel(params, wheelbase)
        self.p = self.model.p
        self.state7 = np.zeros(7)
        self._x = np.zeros(5)

    def reset(self, x=0.0, y=0.0, psi=0.0, v=0.0):
        self._x = np.array([x, y, psi, v, 0.0])
        self._r = 0.0

    def step(self, throttle_us, steer_us, dt):
        self._x, self._r, _ = self.model.step(self._x, throttle_us, steer_us, dt)
        return self.state

    @property
    def state(self):
        X, Y, psi, v, d = self._x
        return np.array([X, Y, psi, v, 0.0, self._r, d])

    @property
    def yaw_rate(self):
        return float(self._r)

    @property
    def wheel_speed(self):
        return float(self._x[3])

    @property
    def lateral_accel(self):
        return float(self._x[3] * self._r)

    @property
    def sideslip(self):
        return 0.0


class ParamUncertainty:
    """Gaussian over model parameters: mean (all params) + covariance over the fitted ones."""

    def __init__(self, mean, names=(), cov=None, inflate=1.0):
        self.mean = dict(mean)
        self.names = list(names)
        self.cov = np.zeros((0, 0)) if cov is None else np.asarray(cov, float) * inflate ** 2

    @classmethod
    def from_fits(cls, fits, inflate=2.0, exclude=("gyro_bias", "input_delay")):
        """Combine FitResults (e.g. longitudinal + lateral). Jacobian covariances assume white
        residuals and are optimistic, hence the default 2x inflation. gyro_bias is the
        estimator's job and input_delay is compensated, so neither is sampled."""
        mean = dict(fits[0].params)
        for f in fits[1:]:
            mean.update({k: f.params[k] for k in f.free})
        names, blocks = [], []
        for f in fits:
            keep = [i for i, k in enumerate(f.free) if k not in exclude]
            names += [f.free[i] for i in keep]
            sd = np.array([f.std[f.free[i]] for i in keep])
            corr = np.nan_to_num(f.corr[np.ix_(keep, keep)], nan=0.0)
            np.fill_diagonal(corr, 1.0)
            blocks.append(corr * np.outer(sd, sd))
        cov = np.zeros((len(names), len(names)))
        i = 0
        for b in blocks:
            n = b.shape[0]
            cov[i:i + n, i:i + n] = b
            i += n
        return cls(mean, names, cov, inflate)

    def sample(self, k, rng):
        """-> dict of (k,) arrays for sampled params, scalars for the rest. Clipped to PARAMS bounds."""
        out = dict(self.mean)
        if not self.names or k == 0:
            return out
        mu = np.array([self.mean[n] for n in self.names])
        L = np.linalg.cholesky(self.cov + 1e-18 * np.eye(len(mu)))
        draws = mu + rng.standard_normal((k, len(mu))) @ L.T
        for j, n in enumerate(self.names):
            lo, hi = M.PARAMS[n][1], M.PARAMS[n][2]
            out[n] = np.clip(draws[:, j], lo, hi)
        return out

    def relative_std(self):
        return {n: math.sqrt(self.cov[i, i]) / abs(self.mean[n]) for i, n in enumerate(self.names) if self.mean[n]}
