"""Experiment design for system ID: what is the data missing, and which maneuver would fix it?

The fit's covariance says how well each parameter is known. For any maneuver we could drive
next, simulating it with the current best-fit car gives the sensitivity of every predicted
measurement (encoder speed, gyro yaw rate) to every parameter; J^T J / sigma^2 is the
information that maneuver would add. Adding it to what is already known predicts the
uncertainty after driving it - before driving it. suggest() greedily picks the maneuvers that
cut the most uncertainty on the parameters still above their target, per second of driving,
from a library that respects the stage's speed cap, the firmware throttle clamp and the lot.

Caveats: the prediction uses the current estimate (a bad estimate gives a worse plan - refit
and re-plan after each batch), and the fit's 1-sigma assumes white residuals (optimistic).
Unidentified parameters are not "known": they fall back to a weak prior (half their range).
"""

from dataclasses import dataclass, field

import numpy as np

import sysid_maneuvers as D
import sysid_model as M

BLOCKS = {"longitudinal": M.LONGITUDINAL_FREE, "lateral": M.LATERAL_FREE}

# 1-sigma targets. Gating ones are gate A in CONTROL_ROADMAP.md (gains and time constants 5%,
# steer centre 2 us, input delay 10 ms); the rest are advisory and weigh half in planning.
TARGETS = {
    "speed_gain": ("rel", 0.05), "tau_speed": ("rel", 0.05), "brake_gain": ("rel", 0.05),
    "steer_gain": ("rel", 0.05), "tau_steer": ("rel", 0.05),
    "steer_center_us": ("abs", 2.0), "input_delay": ("abs", 0.010),
    "deadband_us": ("abs", 3.0), "roll_decel": ("rel", 0.10), "drag_coeff": ("rel", 0.25),
    "v_char": ("rel", 0.10), "gyro_bias": ("abs", 0.005),
}
GATING = {"speed_gain", "tau_speed", "brake_gain", "steer_gain", "tau_steer", "steer_center_us", "input_delay"}
TURNAROUND_S = 15.0  # operator resets the car between maneuvers - counts against each one
CONSERVATISM = 2.0   # plan as if each maneuver gives sigma this much worse than the white-noise
                     # prediction (model error makes residuals correlated); fits after driving
                     # will usually come out better than planned


@dataclass
class Limits:
    max_speed_mps: float = 3.0          # stage speed cap (predicted with the current fit)
    throttle_max_us: int = 1600         # firmware clamp (failsafe.h auto_speed_max_us)
    area_m: tuple = D.TEST_AREA_M       # usable lot
    brake_us: int = 1400
    steer_min_us: int = 1300            # firmware steering clamp
    steer_max_us: int = 1700


@dataclass
class Noise:
    speed: float = 0.05     # m/s, encoder + model error (use the fit's rmse once there is one)
    yaw: float = 0.03       # rad/s
    speed_hz: float = 50.0
    yaw_hz: float = 88.0


@dataclass
class Maneuver:
    name: str
    kind: str               # "longitudinal" | "lateral"
    schedule: list          # [(duration_s, throttle_us, steer_us), ...]
    brakes: bool = False
    cruise_mps: float = None
    duration_s: float = 0.0


@dataclass
class Suggestion:
    maneuver: Maneuver
    improves: list          # [(param, sigma_before, sigma_after)], biggest gains first
    gain_per_s: float
    predicted_std: dict = field(default_factory=dict)

    def describe(self, params):
        parts = []
        for k, b, a in self.improves:
            parts.append("%s %s -> %s (target %s)" % (k, _fmt_sigma(k, b, params), _fmt_sigma(k, a, params),
                                                       _fmt_target(k)))
        return "; ".join(parts)


def _ref(k, value):
    """Magnitude a relative target refers to: the estimate, but not less than a quarter of the
    model default - an estimate stuck near 0 (e.g. brake_gain with no real braking in the data)
    would otherwise ask for 5% of nothing."""
    return max(abs(value), 0.25 * abs(M.PARAMS[k][0]), 1e-12)


def _target_abs(k, value):
    kind, t = TARGETS[k]
    return t if kind == "abs" else t * _ref(k, value)


def _fmt_sigma(k, s, params):
    kind, _ = TARGETS[k]
    if kind == "rel":
        pct = 100 * s / _ref(k, params[k])
        return "+/-%.1f%%" % pct if pct < 10 else ("+/-%.0f%%" % pct if pct < 1000 else "unknown")
    if k == "input_delay":
        return "+/-%.1f ms" % (1e3 * s)
    return "+/-%.3g" % s


def _fmt_target(k):
    kind, t = TARGETS[k]
    if kind == "rel":
        return "%.0f%%" % (100 * t)
    return "%.0f ms" % (1e3 * t) if k == "input_delay" else "%g" % t


# ---------------------------------------------------------------------------------------------
# candidate maneuvers
# ---------------------------------------------------------------------------------------------

def throttle_for_speed(p, v):
    """Throttle (us) whose steady state is v m/s under model p (drag and rolling included)."""
    u = (v + p["tau_speed"] * (p["roll_decel"] + p["drag_coeff"] * v * v)) / p["speed_gain"]
    return p["throttle_center_us"] + p["deadband_us"] + u


def _fits(sched, p, limits):
    fp = D.footprint(sched, p)
    long_side, short_side = max(fp["x_extent_m"], fp["y_extent_m"]), min(fp["x_extent_m"], fp["y_extent_m"])
    return (fp["top_speed_mps"] <= limits.max_speed_mps * 1.05 and long_side <= max(limits.area_m)
            and short_side <= min(limits.area_m))


def candidates(p, limits):
    """Library of maneuvers sized from the current model p, filtered to the stage limits."""
    out = []
    n0 = 1500.0
    vmax = limits.max_speed_mps

    def level(v):
        return int(round(min(throttle_for_speed(p, v), limits.throttle_max_us)))

    still = (2.0, n0, n0)  # standstill first: initial condition and gyro bias
    # longitudinal: steps to a target speed, then brake straight from speed (separates braking
    # from rolling resistance), or coast first (coast-down: rolling + drag), optionally brake
    for frac in (0.35, 0.6, 0.85, 1.0):
        v = frac * vmax
        lv = level(v)
        out.append(Maneuver("step %d us 3s (~%.1f m/s), brake %d from speed" % (lv, v, limits.brake_us),
                            "longitudinal", D.holds(still, (3.0, lv, n0), (1.5, limits.brake_us, n0), (2.0, n0, n0)),
                            brakes=True))
        for hold in (2.5, 5.0):
            out.append(Maneuver("step %d us %.1fs (~%.1f m/s), coast, brake %d" % (lv, hold, v, limits.brake_us),
                                "longitudinal", D.holds(still, (hold, lv, n0), (2.5, n0, n0), (1.5, limits.brake_us, n0),
                                                        (1.5, n0, n0)), brakes=True))
        out.append(Maneuver("step %d us 4s (~%.1f m/s), long coast" % (lv, v), "longitudinal",
                            D.holds(still, (4.0, lv, n0), (6.0, n0, n0))))
    # slow ramp through the deadband
    top = level(0.6 * vmax)
    ramp = [still] + [(0.25, n0 + (top - n0) * i / 31, n0) for i in range(32)] + [(3.0, n0, n0),
                                                                                   (1.5, limits.brake_us, n0),
                                                                                   (1.5, n0, n0)]
    out.append(Maneuver("ramp 1500->%d us over 8 s, coast, brake" % top, "longitudinal", D.holds(*ramp), brakes=True))
    # lateral: doublets and a sweep at two cruise speeds, off the steering lock
    amp_max = min(0.85 * p["delta_max"] / p["steer_gain"], limits.steer_max_us - n0, n0 - limits.steer_min_us)
    for frac in (0.4, 0.85):
        v = max(0.8, frac * vmax)
        lv = level(v)
        for a in sorted({int(min(a, amp_max)) for a in (60, 120, 180)}):
            out.append(Maneuver("doublet +/-%d us at ~%.1f m/s" % (a, v), "lateral",
                                D.holds(still, (3.0, lv, n0), (1.0, lv, n0 + a), (1.0, lv, n0 - a), (1.0, lv, n0),
                                        (2.0, n0, n0)), cruise_mps=v))
        a = int(min(80, amp_max))
        f = np.geomspace(0.3, 3.0, 60)
        phase = np.cumsum(2 * np.pi * f * 0.1)
        out.append(Maneuver("sweep 0.3-3 Hz +/-%d us at ~%.1f m/s" % (a, v), "lateral",
                            D.holds(still, (3.0, lv, n0)) + [(0.1, lv, n0 + a * np.sin(ph)) for ph in phase]
                            + D.holds((2.0, n0, n0)), cruise_mps=v))
    kept = []
    for m in out:
        if _fits(m.schedule, p, limits):
            m.duration_s = float(sum(s[0] for s in m.schedule))
            kept.append(m)
    return kept


# ---------------------------------------------------------------------------------------------
# information
# ---------------------------------------------------------------------------------------------

def _step(k, value):
    _, lo, hi, _, _ = M.PARAMS[k]
    return 1e-3 * max(abs(value), 1e-3 * (hi - lo))


def _jacobian(f, p, free):
    cols = []
    for k in free:
        h = _step(k, p[k])
        cols.append((f(dict(p, **{k: p[k] + h})) - f(dict(p, **{k: p[k] - h}))) / (2 * h))
    return np.column_stack(cols)


def information(p, schedule, noise=None):
    """Fisher information {block: (n_free x n_free)} that driving `schedule` would add."""
    noise = noise or Noise()
    t_cmd, thr, st = D.schedule_signal(schedule)
    T = t_cmd[-1]
    thr_f, st_f = M.input_signal(t_cmd, thr), M.input_signal(t_cmd, st)

    grid = np.arange(0.0, T, 0.01)
    t_v = np.arange(0.5, T - 0.5, 1.0 / noise.speed_hz)

    def speed(q):
        return np.interp(t_v, grid, M.simulate_speed(q, grid, thr_f, v0=0.0))

    J = _jacobian(speed, p, M.LONGITUDINAL_FREE)
    info = {"longitudinal": J.T @ J / noise.speed ** 2}

    grid_l = np.arange(0.0, T, 0.005)
    t_r = np.arange(0.5, T - 0.5, 1.0 / noise.yaw_hz)
    v_r = np.interp(t_r, grid_l, M.simulate_speed(p, grid_l, thr_f, v0=0.0))  # the fit uses measured speed

    def yaw(q):
        return M.yaw_rate(q, v_r, np.interp(t_r, grid_l, M.simulate_steering(q, grid_l, st_f)))

    J = _jacobian(yaw, p, M.LATERAL_FREE)
    info["lateral"] = J.T @ J / noise.yaw ** 2
    return info


def prior_information(free):
    """Weak prior: each parameter known to about half its allowed range."""
    return np.diag([1.0 / ((M.PARAMS[k][2] - M.PARAMS[k][1]) / 2.0) ** 2 for k in free])


def std_from_information(info):
    return np.sqrt(np.clip(np.diag(np.linalg.pinv(info)), 0, None))


class State:
    """What is known now: information per block (from fits), plus the weak prior."""

    def __init__(self, info=None):
        self.info = info or {b: np.zeros((len(f), len(f))) for b, f in BLOCKS.items()}

    @classmethod
    def from_fits(cls, fit_long=None, fit_lat=None):
        info = {}
        for block, fit in (("longitudinal", fit_long), ("lateral", fit_lat)):
            n = len(BLOCKS[block])
            if fit is None:
                info[block] = np.zeros((n, n))
                continue
            assert list(fit.free) == list(BLOCKS[block]), "fit with the standard free parameters"
            sd = np.array([fit.std[k] for k in fit.free])
            corr = np.nan_to_num(fit.corr)
            np.fill_diagonal(corr, 1.0)
            cov = corr * np.outer(sd, sd)
            info[block] = np.linalg.pinv(cov)  # = J^T J / sigma^2; unidentified directions stay 0
        return cls(info)

    def total(self, block):
        return self.info[block] + prior_information(BLOCKS[block])

    def std(self, block):
        return dict(zip(BLOCKS[block], std_from_information(self.total(block))))

    def correlated(self, threshold=0.95):
        """Strongly correlated parameter pairs, from the prior-backed covariance."""
        out = []
        for b, free in BLOCKS.items():
            cov = np.linalg.pinv(self.total(b))
            sd = np.sqrt(np.clip(np.diag(cov), 1e-300, None))
            corr = cov / np.outer(sd, sd)
            for i in range(len(free)):
                for j in range(i + 1, len(free)):
                    if abs(corr[i, j]) >= threshold:
                        out.append((free[i], free[j], float(corr[i, j])))
        return out

    def all_std(self):
        out = {}
        for b in BLOCKS:
            out.update(self.std(b))
        return out


# ---------------------------------------------------------------------------------------------
# planning
# ---------------------------------------------------------------------------------------------

def shortfall(std, params):
    """How far the uncertainties are above target (0 = every target met)."""
    s = 0.0
    for k, sd in std.items():
        excess = np.log(max(sd / _target_abs(k, params[k]), 1.0))
        s += excess * (1.0 if k in GATING else 0.5)
    return s


def suggest(state, params, limits=None, n=3, noise=None, kinds=None, conservatism=CONSERVATISM):
    """Greedy plan of up to n maneuvers that best close the gap to the targets.

    kinds: restrict to {"longitudinal"} or {"lateral"} maneuvers (default both)."""
    limits = limits or Limits()
    state = state or State()
    cands = [c for c in candidates(params, limits) if kinds is None or c.kind in kinds]
    infos = [{b: i / conservatism ** 2 for b, i in information(params, c.schedule, noise).items()} for c in cands]
    cur = {b: state.total(b) for b in BLOCKS}

    def std_of(info_by_block):
        out = {}
        for b, f in BLOCKS.items():
            out.update(zip(f, std_from_information(info_by_block[b])))
        return out

    plan = []
    for _ in range(n):
        before = std_of(cur)
        base = shortfall(before, params)
        best = None
        for c, inf in zip(cands, infos):
            new = {b: cur[b] + inf[b] for b in BLOCKS}
            gain = (base - shortfall(std_of(new), params)) / (c.duration_s + TURNAROUND_S)
            if best is None or gain > best[0]:
                best = (gain, c, new)
        if best is None or best[0] <= 1e-9:
            break
        gain, c, new = best
        after = std_of(new)
        helped = []
        for k in before:
            tb, ta = before[k] / _target_abs(k, params[k]), after[k] / _target_abs(k, params[k])
            if tb > 1.0 and ta < tb * 0.9:
                helped.append((np.log(tb) - np.log(max(ta, 1.0)), k))
        improves = [(k, before[k], after[k]) for _, k in sorted(helped, reverse=True)[:4]]
        plan.append(Suggestion(c, improves, gain, after))
        cur = new
    return plan


def gate(params, std):
    """Gate A table: one row per parameter with a target."""
    rows = []
    for k, sd in std.items():
        if k not in TARGETS:
            continue
        target = _target_abs(k, params[k])
        rows.append({"param": k, "value": params[k], "std": sd, "target": target, "ok": bool(sd <= target),
                     "gating": k in GATING, "std_text": _fmt_sigma(k, sd, params), "target_text": _fmt_target(k)})
    return rows
