"""Path-following controllers: a pure-pursuit baseline and MPPI with UKF/sysid uncertainty.

Both take the UKF estimate (z, P) and return PWM (throttle_us, steer_us), like the ESP32 would
output. Both compensate the identified input delay: commands sent now act `input_delay` later,
so they plan from the state predicted that far ahead (DelayCompensator).

UKF-MPPI pairing used here ("level 1", see CONTROL_ROADMAP.md):
  1. each rollout starts from a draw of the UKF estimate N(z, P)          (state uncertainty)
  2. each rollout uses a draw of the sysid parameters N(theta, Sigma)     (model uncertainty)
  3. track limits are tightened by z_sigma * the UKF's cross-track std    (chance constraint)
Level 2 (U-MPPI: propagate sigma points along every rollout) is the next step, gated in the roadmap.
"""

import math
import time
from collections import deque

import numpy as np

import sysid_model as M
from state_estimator import IPSI, IV, IX, IY
from vehicle_dynamics import KinematicModel, servo_step

G = 9.81


class DelayCompensator:
    """Tracks the servo angle the car should have (the planner state has no steering sensor) and
    predicts the planning state at t + input_delay using the commands already in flight."""

    def __init__(self, params, wheelbase=M.WHEELBASE_M, sub_dt=0.005):
        self.p = params
        self.model = KinematicModel(params, wheelbase)
        self.sub_dt = sub_dt
        self.sent = deque()  # (t_sent, throttle, steer)
        self.delta = 0.0
        self.t = None

    def _active(self, t):
        """Command acting on the car at time t (sent at or before t - delay); neutral before any."""
        tau = t - self.p["input_delay"]
        cmd = (self.p["throttle_center_us"], self.p["steer_center_us"])
        for ts, th, st in self.sent:
            if ts <= tau + 1e-12:
                cmd = (th, st)
            else:
                break
        return cmd

    def advance(self, t):
        """Integrate the servo-angle estimate up to t."""
        if self.t is None:
            self.t = t
            return
        tt = self.t
        while tt < t - 1e-12:
            h = min(self.sub_dt, t - tt)
            _, st = self._active(tt)
            self.delta = float(servo_step(self.p, self.delta, st, h))
            tt += h
        self.t = t
        keep_after = t - self.p["input_delay"] - 0.5
        while len(self.sent) > 1 and self.sent[1][0] < keep_after:
            self.sent.popleft()

    def record(self, t, throttle, steer):
        self.sent.append((t, float(throttle), float(steer)))

    def predict(self, t, x5):
        """Roll x5 = [x, y, psi, v, delta] from t to t + delay with the in-flight commands."""
        x = np.array(x5, float)
        tt, t_end = t, t + self.p["input_delay"]
        while tt < t_end - 1e-12:
            h = min(self.sub_dt, t_end - tt)
            th, st = self._active(tt)
            x, _, _ = self.model.step(x, th, st, h)
            tt += h
        return x


def _planner_state(z, delta):
    return np.array([z[IX], z[IY], z[IPSI], max(z[IV], 0.0), delta])


def throttle_for_speed(p, v_target, v, kp_us=25.0):
    """Invert the ESC model: steady-state throttle for v_target, plus proportional correction;
    brake when well above target."""
    if v_target <= 0.05 and v < 0.3:
        return p["throttle_center_us"]
    hold = (v_target + p["tau_speed"] * (p["roll_decel"] + p["drag_coeff"] * v_target ** 2)) / p["speed_gain"]
    u = p["throttle_center_us"] + p["deadband_us"] + hold + kp_us * (v_target - v)
    if v > v_target + 0.7:
        decel = min((v - v_target) * 3.0, 8.0)
        u = p["throttle_center_us"] - p["deadband_us"] - decel / max(p["brake_gain"], 1e-3)
    return u


class PurePursuit:
    """Geometric baseline: pure pursuit on the centerline + curvature speed profile + ESC inverse."""

    name = "pure_pursuit"

    def __init__(self, course, params, wheelbase=M.WHEELBASE_M, a_lat=0.7 * G, v_max=12.0,
                 lookahead=(1.5, 0.35, 6.0), thr_limits=(1350.0, 1700.0)):
        self.course, self.p, self.L = course, dict(params), wheelbase
        self.v_ref = course.speed_profile(a_lat=a_lat, v_max=v_max)
        self.la = lookahead
        self.thr_limits = thr_limits
        self.delay = DelayCompensator(self.p, wheelbase)
        self.compute_s = []

    def __call__(self, t, z, P):
        t0 = time.perf_counter()
        self.delay.advance(t)
        x = self.delay.predict(t, _planner_state(z, self.delay.delta))
        X, Y, psi, v, _ = x
        s, _ = self.course.project(X, Y)
        ld = float(np.clip(self.la[0] + self.la[1] * v, self.la[0], self.la[2]))
        target, _, _, _ = self.course.at(s + ld)
        dx, dy = target[0] - X, target[1] - Y
        y_body = -math.sin(psi) * dx + math.cos(psi) * dy
        kappa = 2.0 * y_body / max(dx * dx + dy * dy, 1e-6)
        # invert the understeer term so the car actually achieves kappa
        delta = math.atan(kappa * self.L * (1.0 + (v / self.p["v_char"]) ** 2))
        delta = float(np.clip(delta, -self.p["delta_max"], self.p["delta_max"]))
        steer = self.p["steer_center_us"] + delta / self.p["steer_gain"]
        i = int(round(((s + 0.5 * v) % self.course.length) / self.course.ds)) % len(self.v_ref)
        thr = float(np.clip(throttle_for_speed(self.p, self.v_ref[i], v), *self.thr_limits))
        self.delay.record(t, thr, steer)
        self.compute_s.append(time.perf_counter() - t0)
        return thr, steer


class MPPI:
    """Model predictive path integral control on the batched sysid model.

    Cost per rollout: -progress along the course, off-track penalties against the (tightened)
    half width, a lateral-acceleration cap standing in for the grip limit the kinematic model
    lacks, and command smoothness.
    """

    def __init__(self, course, params, wheelbase=M.WHEELBASE_M, dt=0.02, horizon=60, samples=1000,
                 sigma_us=(40.0, 25.0), noise_corr=0.9, temperature=0.05, a_lat_max=0.7 * G, v_max=12.0,
                 thr_limits=(1350.0, 1700.0), car_half_width=0.2,
                 param_uncertainty=None, use_state_cov=False, z_sigma=0.0, scenarios=1, risk=0.5,
                 seed=0, name="mppi", w_progress=10.0, w_track=400.0, w_alat=0.5, w_smooth=0.5):
        self.course, self.p, self.L = course, dict(params), wheelbase
        self.model = KinematicModel(self.p, wheelbase)
        self.dt, self.H, self.K = dt, horizon, samples
        self.sigma = np.asarray(sigma_us, float)
        self.noise_corr, self.temperature = noise_corr, temperature
        self.a_lat_max, self.v_max = a_lat_max, v_max
        st_span = self.p["delta_max"] / self.p["steer_gain"]
        self.lo = np.array([thr_limits[0], self.p["steer_center_us"] - st_span])
        self.hi = np.array([thr_limits[1], self.p["steer_center_us"] + st_span])
        self.half_width = course.width / 2 - car_half_width
        self.min_half_width = 0.25
        self.v_cap = v_max
        self.unc = param_uncertainty
        self.use_state_cov, self.z_sigma = use_state_cov, z_sigma
        self.M = scenarios if (use_state_cov or param_uncertainty is not None) else 1
        self.risk = risk
        self.rng = np.random.default_rng(seed)
        self.name = name
        self.w = dict(progress=w_progress, track=w_track, alat=w_alat, smooth=w_smooth)
        self.U = np.tile([self.p["throttle_center_us"] + self.p["deadband_us"] + 20, self.p["steer_center_us"]],
                         (self.H, 1))
        self.delay = DelayCompensator(self.p, wheelbase)
        self.compute_s, self.margin_used, self.last_rollouts = [], [], None

    def _noise(self):
        w = self.rng.standard_normal((self.K, self.H, 2))
        a = self.noise_corr
        eps = np.empty_like(w)
        eps[:, 0] = w[:, 0]
        for h in range(1, self.H):
            eps[:, h] = a * eps[:, h - 1] + math.sqrt(1 - a * a) * w[:, h]
        eps *= self.sigma
        eps[0] = 0.0  # rollout 0 = the warm-started nominal
        return eps

    def _scenarios(self, x_pred, P):
        """M shared (initial state, parameter) draws; scenario 0 is the nominal estimate/model.
        Every control sample is scored under all M - common random numbers, so a sequence
        cannot win by being paired with a lucky draw."""
        X0 = np.tile(x_pred, (self.M, 1))
        if self.use_state_cov and self.M > 1:
            idx = [IX, IY, IPSI, IV]
            Ls = np.linalg.cholesky(P[np.ix_(idx, idx)] + 1e-9 * np.eye(4))
            X0[1:, :4] += self.rng.standard_normal((self.M - 1, 4)) @ Ls.T
            X0[:, 3] = np.maximum(X0[:, 3], 0.0)
        p = self.p
        if self.unc is not None and self.M > 1:
            p = self.unc.sample(self.M, self.rng)
            for k, v in p.items():
                if np.ndim(v):
                    v[0] = self.p[k]
        return X0, p

    def _half_width(self, x_pred, P):
        if self.z_sigma <= 0:
            return self.half_width
        s, _ = self.course.project(x_pred[0], x_pred[1])
        _, _, _, n = self.course.at(s)
        sig_e = math.sqrt(max(n @ P[np.ix_([IX, IY], [IX, IY])] @ n, 0.0))
        return max(self.half_width - self.z_sigma * sig_e, self.min_half_width)

    def rollout_costs(self, X0, U_samp, p, half):
        K, H = U_samp.shape[:2]
        x = X0.copy()
        cost = np.zeros(K)
        s_prev, _ = self.course.lookup(x[:, 0], x[:, 1])
        progress = np.zeros(K)
        traj = np.empty((K, H, 2))
        for h in range(H):
            x, r, _ = self.model.step(x, U_samp[:, h, 0], U_samp[:, h, 1], self.dt, p)
            s, e = self.course.lookup(x[:, 0], x[:, 1])
            progress += self.course.wrap_ds(s - s_prev)
            s_prev = s
            excess = np.maximum(np.abs(e) - half, 0.0)
            cost += self.w["track"] * ((excess > 0) + 5.0 * excess)
            alat = np.maximum(np.abs(x[:, 3] * r) - self.a_lat_max, 0.0)
            cost += self.w["alat"] * alat ** 2 + 50.0 * np.maximum(x[:, 3] - self.v_cap, 0.0)
            traj[:, h] = x[:, :2]
        du = np.diff(U_samp, axis=1) / 100.0
        cost += self.w["smooth"] * np.sum(du ** 2, axis=(1, 2))
        cost -= self.w["progress"] * progress
        return cost, traj

    def __call__(self, t, z, P):
        t0 = time.perf_counter()
        self.delay.advance(t)
        x_pred = self.delay.predict(t, _planner_state(z, self.delay.delta))
        eps = self._noise()
        U_samp = np.clip(self.U[None] + eps, self.lo, self.hi)
        half = self._half_width(x_pred, P)
        X0, p = self._scenarios(x_pred, P)
        K, Mc = self.K, self.M
        # rollout k*M + m = control sample k under scenario m
        p_t = {k: (np.tile(v, K) if np.ndim(v) else v) for k, v in p.items()}
        # graceful degradation: when the margin eats most of the track the estimate is too poor
        # to race on - cap speed in proportion instead of squeezing the path to nothing
        self.v_cap = self.v_max * float(np.clip(half / (0.5 * self.half_width), 0.35, 1.0))
        c_all, traj = self.rollout_costs(np.tile(X0, (K, 1)), np.repeat(U_samp, Mc, axis=0), p_t, half)
        c_all = c_all.reshape(K, Mc)
        # risk-aware aggregate: blend of the mean and the worst scenario
        cost = (1 - self.risk) * c_all.mean(axis=1) + self.risk * c_all.max(axis=1) if Mc > 1 else c_all[:, 0]
        traj = traj[::Mc]
        beta = cost.min()
        lam = self.temperature * max(np.median(cost) - beta, 1e-6)
        wts = np.exp(-(cost - beta) / lam)
        wts /= wts.sum()
        self.U = np.einsum("k,khj->hj", wts, U_samp)
        u = self.U[0].copy()
        self.U = np.vstack([self.U[1:], self.U[-1:]])  # warm start for the next tick
        self.delay.record(t, *u)
        self.compute_s.append(time.perf_counter() - t0)
        self.margin_used.append(half)
        self.ess = 1.0 / np.sum(wts ** 2)
        self.last_rollouts = traj
        return float(u[0]), float(u[1])
