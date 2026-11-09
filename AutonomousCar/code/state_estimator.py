"""UKF for the car: gyro + driveshaft encoder + (lagged) GPS position.

State z = [x, y, psi, v, r, gyro_bias, gps_bias_x, gps_bias_y]. Process: constant speed /
constant turn rate with random-walk acceleration, yaw acceleration and gyro bias; the GPS bias is
first-order Gauss-Markov (non-RTK receivers wander by tens of cm over tens of seconds).
Measurements:
  gyro     r + bias                   (OLA, ~88 Hz)
  encoder  v                          (ESP32, ~50 Hz)
  gps      x + bx, y + by at fix time (10 Hz, arrives `lag` s late)

The GPS bias is not observable from these sensors; modelling it keeps the position covariance
honest (it cannot shrink below the bias spread), which matters because the MPPI pairing uses
that covariance to tighten the track limits.

GPS lag is handled exactly by rollback/replay: snapshots are kept for the last second; a late
fix restores the snapshot before its fix time, applies the update there, and replays the gyro
and encoder updates that arrived since. Its covariance is what the MPPI pairing consumes.
"""

import math

import numpy as np

N = 8
IX, IY, IPSI, IV, IR, IB, IGX, IGY = range(N)


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class UKF:
    def __init__(self, z0, P0, q_accel=3.0, q_yaw_accel=8.0, q_bias=1e-3,
                 r_gyro=0.02, r_enc=0.05, r_enc_frac=0.02, r_gps=0.5, gps_bias_sd=0.3, gps_bias_tau=30.0,
                 history_s=1.0,
                 alpha=0.5, beta=2.0, kappa=0.0):
        self.z = np.asarray(z0, float).copy()
        self.P = np.asarray(P0, float).copy()
        self.t = 0.0
        self.q = (q_accel, q_yaw_accel, q_bias)
        self.r_gyro, self.r_enc, self.r_enc_frac, self.r_gps = r_gyro, r_enc, r_enc_frac, r_gps
        self.gps_bias_sd, self.gps_bias_tau = gps_bias_sd, gps_bias_tau
        lam = alpha ** 2 * (N + kappa) - N
        self.c = N + lam
        self.Wm = np.full(2 * N + 1, 0.5 / self.c)
        self.Wc = self.Wm.copy()
        self.Wm[0] = lam / self.c
        self.Wc[0] = lam / self.c + (1 - alpha ** 2 + beta)
        self.history_s = history_s
        self._snap = []   # (t, z, P) after each update, oldest first
        self._meas = []   # (t, kind, value) gyro/encoder updates, for replay

    # -- core ------------------------------------------------------------------------------
    def _sigma(self):
        try:
            S = np.linalg.cholesky(self.c * (self.P + 1e-12 * np.eye(N)))
        except np.linalg.LinAlgError:
            # numerical loss of definiteness (e.g. long sensor dropout): repair rather than crash
            w, V = np.linalg.eigh(0.5 * (self.P + self.P.T))
            self.P = (V * np.clip(w, 1e-9, None)) @ V.T
            S = np.linalg.cholesky(self.c * self.P)
        return np.vstack([self.z, self.z + S.T, self.z - S.T])

    def _f(self, Z, dt):
        Z = Z.copy()
        Z[:, [IGX, IGY]] *= math.exp(-dt / self.gps_bias_tau)
        psi, v, r = Z[:, IPSI], Z[:, IV], Z[:, IR]
        mid = psi + 0.5 * r * dt
        Z[:, IX] += v * np.cos(mid) * dt
        Z[:, IY] += v * np.sin(mid) * dt
        Z[:, IPSI] = psi + r * dt
        return Z

    def _mean(self, Z):
        m = self.Wm @ Z
        m[IPSI] = math.atan2(self.Wm @ np.sin(Z[:, IPSI]), self.Wm @ np.cos(Z[:, IPSI]))
        return m

    def _predict(self, dt):
        if dt <= 0:
            return
        Z = self._f(self._sigma(), dt)
        self.z = self._mean(Z)
        D = Z - self.z
        D[:, IPSI] = _wrap(D[:, IPSI])
        qa, qr, qb = self.q
        Q = np.zeros((N, N))
        # white-noise acceleration on v (and the position it drives), yaw accel on r (and psi)
        Q[np.ix_([IX, IY], [IX, IY])] = np.eye(2) * (qa ** 2) * dt ** 3 / 3
        Q[IV, IV] = qa ** 2 * dt
        Q[IPSI, IPSI] = qr ** 2 * dt ** 3 / 3
        Q[IPSI, IR] = Q[IR, IPSI] = qr ** 2 * dt ** 2 / 2
        Q[IR, IR] = qr ** 2 * dt
        # without gyro updates r would random-walk forever; cap its spread at a physical 5 rad/s
        if self.P[IR, IR] > 25.0:
            Q[IR, IR] = 0.0
        Q[IB, IB] = qb ** 2 * dt
        Q[IGX, IGX] = Q[IGY, IGY] = self.gps_bias_sd ** 2 * (1 - math.exp(-2 * dt / self.gps_bias_tau))
        self.P = (self.Wc[:, None] * D).T @ D + Q
        self.z[IV] = max(self.z[IV], 0.0)

    def _update(self, h, y, R):
        Z = self._sigma()
        Y = h(Z)
        y_hat = self.Wm @ Y
        DY = Y - y_hat
        DZ = Z - self.z
        DZ[:, IPSI] = _wrap(DZ[:, IPSI])
        S = (self.Wc[:, None] * DY).T @ DY + R
        C = (self.Wc[:, None] * DZ).T @ DY
        K = C @ np.linalg.inv(S)
        innov = np.atleast_1d(y - y_hat)
        self.z = self.z + K @ innov
        self.z[IPSI] = _wrap(self.z[IPSI])
        self.P = self.P - K @ S @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        return float(innov @ np.linalg.solve(S, innov))  # NIS

    def _apply(self, kind, value):
        if kind == "gyro":
            return self._update(lambda Z: Z[:, [IR]] + Z[:, [IB]], np.array([value]), np.eye(1) * self.r_gyro ** 2)
        if kind == "enc":
            sd = self.r_enc + self.r_enc_frac * abs(value)
            return self._update(lambda Z: Z[:, [IV]], np.array([value]), np.eye(1) * sd ** 2)
        if kind == "gps":
            return self._update(lambda Z: Z[:, [IX, IY]] + Z[:, [IGX, IGY]], np.asarray(value, float),
                                np.eye(2) * self.r_gps ** 2)
        raise ValueError(kind)

    # -- public API ------------------------------------------------------------------------
    def predict_to(self, t):
        self._predict(t - self.t)
        self.t = max(self.t, t)

    def update(self, t, kind, value):
        """Measurement taken at time t (gyro/enc: now; gps: its fix time, may be in the past)."""
        if kind == "gps" and t < self.t - 1e-9:
            return self._late_update(t, value)
        self.predict_to(t)
        nis = self._apply(kind, value)
        if kind != "gps":
            self._meas.append((t, kind, value))
        self._snap.append((self.t, self.z.copy(), self.P.copy()))
        self._trim()
        return nis

    def _late_update(self, t_fix, value):
        t_now = self.t
        older = [s for s in self._snap if s[0] <= t_fix]
        if not older:
            return None  # fix older than history: drop it
        t0, z0, P0 = older[-1]
        self.t, self.z, self.P = t0, z0.copy(), P0.copy()
        self._snap = [s for s in self._snap if s[0] <= t0]
        replay = [m for m in self._meas if m[0] > t0]
        self._meas = [m for m in self._meas if m[0] <= t0]
        self.predict_to(t_fix)
        nis = self._apply("gps", value)
        self._snap.append((self.t, self.z.copy(), self.P.copy()))
        for tm, kind, val in replay:
            self.update(tm, kind, val)
        self.predict_to(t_now)
        return nis

    def _trim(self):
        cut = self.t - self.history_s
        while self._snap and self._snap[0][0] < cut:
            self._snap.pop(0)
        while self._meas and self._meas[0][0] < cut:
            self._meas.pop(0)

    def estimate_at(self, t):
        """Predicted (z, P) at time t >= self.t without changing the filter."""
        z, P, t0 = self.z.copy(), self.P.copy(), self.t
        self._predict(t - self.t)
        out = (self.z.copy(), self.P.copy())
        self.z, self.P, self.t = z, P, t0
        return out


def initial_ukf(x, y, psi, v=0.0, gps_bias_sd=0.3, **kw):
    """Car placed at a known start point: position known to the GPS bias spread."""
    P0 = np.diag([0.1, 0.1, math.radians(5), 0.2, 0.05, 0.02, gps_bias_sd, gps_bias_sd]) ** 2
    return UKF([x, y, psi, v, 0.0, 0.0, 0.0, 0.0], P0, gps_bias_sd=gps_bias_sd, **kw)
