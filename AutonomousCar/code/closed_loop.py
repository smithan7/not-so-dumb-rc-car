"""Closed-loop rehearsal: identify the car, then drive the course with each controller.

    python closed_loop.py                 # full study: 4 scenarios x 3 controllers x 3 seeds (~3 min, 8 cores)
    python closed_loop.py --quick         # 1 seed, 2 laps, fewer MPPI samples
    python closed_loop.py --plot out.png  # also save the driven paths

Pipeline per scenario, mirroring what will happen on the real car:
  1. run the designed sysid passes (sysid_maneuvers) open loop on the simulated car, with the
     logger's sensors, and fit the sysid model -> parameters + covariance
  2. drive laps with the UKF in the loop (gyro 88 Hz, encoder 50 Hz, GPS 10 Hz with 0.2 s lag
     and a slowly wandering bias), controllers running at 50 Hz on the estimate only
Scenarios: 'nominal' (car = the planning model), 'dynamic' (car has saturating tires the
planner does not model), 'dynamic_badgps' (same, GPS twice as noisy and biased), 'dynamic_rtk'
(same, RTK-grade GPS) - the last two bracket how much localization is worth in lap time.
"""

import argparse
import math
import sys
import time
import warnings
from dataclasses import dataclass, field

import numpy as np

import sysid_maneuvers as D
import sysid_model as M
from controllers import MPPI, PurePursuit
from course import parking_lot_course
from state_estimator import IX, IY, initial_ukf
from vehicle_dynamics import DynamicPlant, KinematicPlant, ParamUncertainty

TRUE_CAR = M.default_params(steer_center_us=1492.0, steer_gain=math.radians(0.09), tau_steer=0.07,
                            gyro_bias=0.012, input_delay=0.045, deadband_us=35.0, speed_gain=0.14,
                            tau_speed=0.7, brake_gain=0.09, roll_decel=0.35, drag_coeff=0.012)
CAR_HALF_WIDTH = 0.2

GPS_GOOD = {"rate_hz": 10.0, "lag_s": 0.2, "noise": 0.5, "bias_sd": 0.3, "bias_tau_s": 30.0}
GPS_BAD = {"rate_hz": 10.0, "lag_s": 0.2, "noise": 1.0, "bias_sd": 0.6, "bias_tau_s": 30.0}
GPS_RTK = {"rate_hz": 10.0, "lag_s": 0.1, "noise": 0.03, "bias_sd": 0.03, "bias_tau_s": 30.0}  # RTK fixed


def make_plant(kind, params):
    return DynamicPlant(params) if kind == "dynamic" else KinematicPlant(params)


class Sensors:
    """Gyro, encoder and lagged GPS sampled from a plant, as the logger would see them."""

    def __init__(self, p_true, gps, rng):
        self.p, self.gps, self.rng = p_true, gps, rng
        self.next = {"gyro": 0.0, "enc": 0.0, "gps": 0.0}
        self.period = {"gyro": 1 / D.SENSORS["gyro"]["rate_hz"], "enc": 1 / D.SENSORS["encoder"]["rate_hz"],
                       "gps": 1 / gps["rate_hz"]}
        self.bias = gps["bias_sd"] * rng.standard_normal(2)  # stationary from the start
        self.pending = []  # (arrival_t, fix_t, xy)

    def poll(self, t, plant):
        out = []
        if t >= self.next["gyro"]:
            self.next["gyro"] += self.period["gyro"]
            out.append((t, "gyro", plant.yaw_rate + self.p["gyro_bias"] + self.rng.normal(0, D.SENSORS["gyro"]["noise"])))
        if t >= self.next["enc"]:
            self.next["enc"] += self.period["enc"]
            v = plant.wheel_speed
            s = D.SENSORS["encoder"]
            out.append((t, "enc", max(v + self.rng.normal(0, s["noise"]) + self.rng.normal(0, s["noise_frac"]) * v, 0.0)))
        if t >= self.next["gps"]:
            g = self.gps
            dt = self.period["gps"]
            a = math.exp(-dt / g["bias_tau_s"])
            self.bias = a * self.bias + g["bias_sd"] * math.sqrt(1 - a * a) * self.rng.standard_normal(2)
            self.next["gps"] += dt
            xy = plant.state[:2] + self.bias + self.rng.normal(0, g["noise"], 2)
            self.pending.append((t + g["lag_s"], t, xy))
        while self.pending and self.pending[0][0] <= t:
            _, t_fix, xy = self.pending.pop(0)
            out.append((t_fix, "gps", xy))
        return out


# ---------------------------------------------------------------------------------------------
# 1. system identification on the simulated car
# ---------------------------------------------------------------------------------------------

def run_open_loop(plant, schedule, p_true, rng, dt=0.001):
    """Drive a sysid schedule open loop (with the car's true input delay) -> log dict for the fitters."""
    t_cmd, thr, st = D.schedule_signal(schedule)
    plant.reset()
    n = int(t_cmd[-1] / dt)
    delay = p_true["input_delay"]
    tg, g, tv, v = [], [], [], []
    next_g = next_v = 0.5
    for k in range(n):
        t = k * dt
        i = max(np.searchsorted(t_cmd, t - delay, side="right") - 1, 0)
        th_k, st_k = (thr[i], st[i]) if t >= delay else (1500.0, 1500.0)
        plant.step(th_k, st_k, dt)
        if t >= next_g:
            next_g += 1 / D.SENSORS["gyro"]["rate_hz"]
            tg.append(t)
            g.append(plant.yaw_rate + p_true["gyro_bias"] + rng.normal(0, D.SENSORS["gyro"]["noise"]))
        if t >= next_v:
            next_v += 1 / D.SENSORS["encoder"]["rate_hz"]
            s = D.SENSORS["encoder"]
            vv = plant.wheel_speed
            tv.append(t)
            v.append(max(vv + rng.normal(0, s["noise"]) + rng.normal(0, s["noise_frac"]) * vv, 0.0))
    # command log: millis()-stamped, so no PC jitter
    return {"t_cmd": t_cmd, "throttle_us": thr, "steer_us": st, "t_gyro": np.array(tg), "yaw_rate": np.array(g),
            "t_speed": np.array(tv), "speed": np.array(v)}


def identify(kind, p_true, seed=0):
    rng = np.random.default_rng(seed)
    long_logs = [run_open_loop(make_plant(kind, p_true), s, p_true, rng) for s in D.longitudinal_passes()]
    lat_logs = [run_open_loop(make_plant(kind, p_true), s, p_true, rng) for s in D.lateral_passes()]
    fl = M.fit_longitudinal([D.longitudinal_segment(l) for l in long_logs])
    fa = M.fit_lateral([D.lateral_segment(l) for l in lat_logs], p0={k: fl.params[k] for k in fl.free})
    params = dict(fa.params)
    params.update({k: fl.params[k] for k in fl.free})
    params["gyro_bias"] = 0.0  # the UKF estimates it on line
    return params, ParamUncertainty.from_fits([fl, fa]), (fl, fa)


# ---------------------------------------------------------------------------------------------
# 2. closed loop
# ---------------------------------------------------------------------------------------------

@dataclass
class RunResult:
    controller: str
    scenario: str
    laps: list
    off_track_frac: float
    max_excess_m: float
    left_track: bool
    mean_speed: float
    max_lat_g: float
    max_sideslip_deg: float
    pos_rmse_m: float
    pos_2sigma_frac: float
    compute_ms_p50: float
    compute_ms_p99: float
    log: dict = field(repr=False, default=None)

    @property
    def best_lap(self):
        flying = self.laps[1:] or self.laps
        return min(flying) if flying else math.nan


def drive(controller, course, kind, p_true, gps, duration_s=40.0, laps=3, seed=1, dt=0.001,
          control_hz=50.0, scenario=""):
    rng = np.random.default_rng(seed)
    plant = make_plant(kind, p_true)
    (x0, y0), psi0 = course.xy[0], course.psi[0]
    plant.reset(x0, y0, psi0)
    # the filter is told the receiver spec (noise, bias spread, wander time) - not the realised bias
    ukf = initial_ukf(x0, y0, psi0, r_gps=gps["noise"], gps_bias_sd=gps["bias_sd"], gps_bias_tau=gps["bias_tau_s"])
    sensors = Sensors(p_true, gps, rng)
    sent = [(-1.0, 1500.0, 1500.0)]
    delay = p_true["input_delay"]
    ctrl_period, next_ctrl = 1.0 / control_hz, 0.0
    half_limit = course.width / 2 - CAR_HALF_WIDTH
    s_prev, progress = 0.0, 0.0
    lap_marks, log = [0.0], {k: [] for k in ("t", "x", "y", "v", "e", "xe", "ye", "sx", "sy", "alat", "beta")}
    left = False
    i_sent = 0
    n = int(duration_s / dt)
    for k in range(n):
        t = k * dt
        if t >= next_ctrl:
            next_ctrl += ctrl_period
            ukf.predict_to(t)
            thr, st = controller(t, ukf.z, ukf.P)
            sent.append((t, thr, st))
        while i_sent + 1 < len(sent) and sent[i_sent + 1][0] <= t - delay:
            i_sent += 1
        _, th_a, st_a = sent[i_sent]
        plant.step(th_a, st_a, dt)
        for tm, kind_m, val in sensors.poll(t, plant):
            ukf.update(tm, kind_m, val)
        if k % 10 == 0:
            X, Y = plant.state[:2]
            s, e = course.project(X, Y)
            progress += course.wrap_ds(s - s_prev)
            s_prev = s
            if progress >= len(lap_marks) * course.length:
                lap_marks.append(t)
            z, P = ukf.estimate_at(t) if ukf.t < t else (ukf.z, ukf.P)
            log["t"].append(t); log["x"].append(X); log["y"].append(Y); log["v"].append(plant.wheel_speed)
            log["e"].append(float(e)); log["xe"].append(z[IX]); log["ye"].append(z[IY])
            log["sx"].append(math.sqrt(P[IX, IX])); log["sy"].append(math.sqrt(P[IY, IY]))
            log["alat"].append(plant.lateral_accel); log["beta"].append(plant.sideslip)
            if abs(e) > course.width / 2 + 1.0:
                left = True  # into the cones / curb: end the run
                break
            if len(lap_marks) > laps:
                break
    L = {k: np.asarray(v) for k, v in log.items()}
    excess = np.maximum(np.abs(L["e"]) - half_limit, 0.0)
    err = np.hypot(L["x"] - L["xe"], L["y"] - L["ye"])
    # 2-D 2-sigma containment (chi2 with 2 dof at 86.5%): consistent filter -> ~0.865
    nees_ok = ((L["x"] - L["xe"]) / L["sx"]) ** 2 + ((L["y"] - L["ye"]) / L["sy"]) ** 2 <= 4.0
    comp = np.asarray(controller.compute_s) * 1e3
    return RunResult(controller=getattr(controller, "name", type(controller).__name__), scenario=scenario,
                     laps=list(np.diff(lap_marks)), off_track_frac=float(np.mean(excess > 0)),
                     max_excess_m=float(excess.max()), left_track=left, mean_speed=float(L["v"].mean()),
                     max_lat_g=float(np.max(np.abs(L["alat"])) / 9.81),
                     max_sideslip_deg=float(np.degrees(np.max(np.abs(L["beta"])))),
                     pos_rmse_m=float(np.sqrt(np.mean(err ** 2))), pos_2sigma_frac=float(np.mean(nees_ok)),
                     compute_ms_p50=float(np.percentile(comp, 50)), compute_ms_p99=float(np.percentile(comp, 99)),
                     log=L)


SCENARIOS = {"nominal": ("kinematic", GPS_GOOD), "dynamic": ("dynamic", GPS_GOOD),
             "dynamic_badgps": ("dynamic", GPS_BAD), "dynamic_rtk": ("dynamic", GPS_RTK)}
DEFAULT_CONTROLLERS = ["pure_pursuit", "mppi", "mppi_ukf"]


def make_controller(name, course, params, unc, samples=1000, seed=0):
    if name == "pure_pursuit":
        return PurePursuit(course, params)
    if name == "mppi":  # plans on the point estimate, full track width
        return MPPI(course, params, samples=samples, seed=seed, name=name)
    if name == "mppi_ukf":  # level-1 pairing: limits tightened by 2 sigma of UKF cross-track error
        return MPPI(course, params, samples=samples, seed=seed, name=name, z_sigma=2.0)
    if name == "mppi_scen":  # alternative studied and not adopted: shared state+parameter scenarios
        return MPPI(course, params, samples=max(samples // 4, 50), seed=seed, name=name, scenarios=4, risk=0.25,
                    use_state_cov=True, param_uncertainty=unc, z_sigma=1.0)
    raise KeyError(name)


def _job(args):
    sc, name, seed, params, unc, laps, samples = args
    warnings.simplefilter("ignore")
    kind, gps = SCENARIOS[sc]
    course = parking_lot_course()
    r = drive(make_controller(name, course, params, unc, samples, seed), course, kind, TRUE_CAR, gps,
              duration_s=20.0 * laps, laps=laps, seed=seed, scenario=sc)
    r.log = {k: r.log[k] for k in ("x", "y")} if seed == 1 else None
    return r


def summarize(results):
    """Mean over seeds per (scenario, controller); off-track and excess as the worst seed too."""
    groups = {}
    for r in results:
        groups.setdefault((r.scenario, r.controller), []).append(r)
    rows = []
    for (sc, name), rs in groups.items():
        rows.append({"scenario": sc, "controller": name, "seeds": len(rs),
                     "best_lap_s": float(np.mean([r.best_lap for r in rs])),
                     "mean_speed": float(np.mean([r.mean_speed for r in rs])),
                     "off_track_pct": 100 * float(np.mean([r.off_track_frac for r in rs])),
                     "off_track_worst_pct": 100 * float(np.max([r.off_track_frac for r in rs])),
                     "max_excess_m": float(np.max([r.max_excess_m for r in rs])),
                     "left_track": int(sum(r.left_track for r in rs)),
                     "max_lat_g": float(np.max([r.max_lat_g for r in rs])),
                     "max_slip_deg": float(np.max([r.max_sideslip_deg for r in rs])),
                     "pos_rmse_m": float(np.mean([r.pos_rmse_m for r in rs])),
                     "pos_in_2sigma_pct": 100 * float(np.mean([r.pos_2sigma_frac for r in rs])),
                     "compute_ms_p50": float(np.median([r.compute_ms_p50 for r in rs])),
                     "compute_ms_p99": float(np.max([r.compute_ms_p99 for r in rs]))})
    return rows


def format_summary(rows):
    head = (f"{'scenario':>15} {'controller':>13} {'best lap':>8} {'v mean':>6} {'over line':>9} {'worst':>6} "
            f"{'max over':>8} {'left':>4} {'lat g':>5} {'slip':>5} {'pos rmse':>8} {'in 2sig':>7} {'ms p50':>6} {'p99':>5}")
    out = [head, "-" * len(head)]
    for r in rows:
        out.append(f"{r['scenario']:>15} {r['controller']:>13} {r['best_lap_s']:>7.2f}s {r['mean_speed']:>6.2f} "
                   f"{r['off_track_pct']:>8.1f}% {r['off_track_worst_pct']:>5.1f}% {r['max_excess_m']:>7.2f}m "
                   f"{r['left_track']:>4} {r['max_lat_g']:>5.2f} {r['max_slip_deg']:>4.1f}d {r['pos_rmse_m']:>7.2f}m "
                   f"{r['pos_in_2sigma_pct']:>6.0f}% {r['compute_ms_p50']:>6.1f} {r['compute_ms_p99']:>5.1f}")
    return "\n".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quick", action="store_true", help="1 seed, 2 laps, 400 MPPI samples")
    ap.add_argument("--scenario", choices=list(SCENARIOS), action="append")
    ap.add_argument("--controller", choices=DEFAULT_CONTROLLERS + ["mppi_scen"], action="append")
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--jobs", type=int, default=0, help="worker processes (default: CPU count)")
    ap.add_argument("--plot", help="save driven paths (seed 1) to this PNG")
    ap.add_argument("--json", help="save the summary rows to this JSON file")
    args = ap.parse_args(argv)
    warnings.simplefilter("ignore")

    laps, samples, seeds = (2, 400, 1) if args.quick else (4, 1000, args.seeds)
    scenarios = args.scenario or list(SCENARIOS)
    ident = {}
    for kind in sorted({SCENARIOS[s][0] for s in scenarios}):
        t0 = time.time()
        ident[kind] = identify(kind, TRUE_CAR, seed=1)
        fl, fa = ident[kind][2]
        print(f"[{kind}] sysid in {time.time() - t0:.0f}s: rmse speed {fl.rmse:.3f} m/s, yaw {fa.rmse:.3f} rad/s",
              file=sys.stderr)
    jobs = [(sc, name, seed, ident[SCENARIOS[sc][0]][0], ident[SCENARIOS[sc][0]][1], laps, samples)
            for sc in scenarios for name in (args.controller or DEFAULT_CONTROLLERS) for seed in range(1, seeds + 1)]
    from concurrent.futures import ProcessPoolExecutor
    import os
    t0 = time.time()
    with ProcessPoolExecutor(args.jobs or os.cpu_count()) as ex:
        results = list(ex.map(_job, jobs))
    print(f"{len(jobs)} runs in {time.time() - t0:.0f}s", file=sys.stderr)
    rows = summarize(results)
    print()
    print(format_summary(rows))
    if args.json:
        import json
        with open(args.json, "w") as f:
            json.dump(rows, f, indent=1)
    if args.plot:
        plot(parking_lot_course(), [r for r in results if r.log is not None], args.plot)
    return 0


def plot(course, results, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    scen = sorted({r.scenario for r in results}, key=list(SCENARIOS).index)
    fig, axes = plt.subplots(1, len(scen), figsize=(7 * len(scen), 5), squeeze=False)
    for ax, sc in zip(axes[0], scen):
        h = course.width / 2
        for sgn in (1, -1):
            edge = course.xy + sgn * h * course.normal
            ax.plot(*np.vstack([edge, edge[:1]]).T, "k", lw=1)
        for r in results:
            if r.scenario == sc:
                ax.plot(r.log["x"], r.log["y"], lw=1, label=f"{r.controller} ({r.best_lap:.1f}s)")
        ax.set_aspect("equal")
        ax.set_title(sc)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=120)


if __name__ == "__main__":
    sys.exit(main())
