"""Test-input schedules for system ID, plus synthetic sensors to rehearse the fits offline.

A schedule is a list of (duration_s, throttle_us, steer_us) holds, or arrays from a generator.
`schedule_signal` turns it into sampled command arrays (what the logger would record).
"""

import numpy as np

from sysid_model import default_params, simulate

NEUTRAL = 1500.0


def holds(*segments):
    """[(duration, throttle_us, steer_us), ...] -> list (keeps call sites readable)."""
    return [tuple(map(float, s)) for s in segments]


def schedule_signal(schedule, rate_hz=20.0, t0=0.0):
    """Sample a hold-schedule at the command/log rate -> (t, throttle_us, steer_us)."""
    ts, thr, st = [], [], []
    t = t0
    for dur, th, sv in schedule:
        n = max(1, int(round(dur * rate_hz)))
        ts.append(t + np.arange(n) / rate_hz)
        thr.append(np.full(n, th))
        st.append(np.full(n, sv))
        t += n / rate_hz
    return np.concatenate(ts), np.concatenate(thr), np.concatenate(st)


# ---------------------------------------------------------------------------------------------
# designs
# ---------------------------------------------------------------------------------------------

TEST_AREA_M = (40.0, 25.0)  # usable lot (2023 GPS track spanned ~46 x 29 m); pass your own


def longitudinal_passes(levels_us=(1560, 1575, 1590, 1610), max_run_m=30.0, still_s=3.0, coast_s=4.0,
                        brake_us=1400, p=None):
    """One straight pass per throttle level: stand still, step, hold, coast, brake to a stop.

    The hold is as long as possible (up to 6 s, ~5 tau_speed) while the pass stays under
    max_run_m by the prior model. The operator turns the car around between passes.
    A first pass ramps slowly through the deadband.
    """
    p = p or default_params()
    passes = [holds((still_s, NEUTRAL, NEUTRAL))
              + [(0.25, NEUTRAL + 10 * i / 4, NEUTRAL) for i in range(33)]   # 1500 -> 1580 over 8 s
              + holds((coast_s, NEUTRAL, NEUTRAL), (1.0, brake_us, NEUTRAL), (2.0, NEUTRAL, NEUTRAL))]
    for lv in levels_us:
        for hold in np.arange(6.0, 0.9, -0.25):
            sched = holds((still_s, NEUTRAL, NEUTRAL), (hold, lv, NEUTRAL), (coast_s, NEUTRAL, NEUTRAL),
                          (1.5, brake_us, NEUTRAL), (2.0, NEUTRAL, NEUTRAL))
            if footprint(sched, p)["x_extent_m"] <= max_run_m:
                break
        passes.append(sched)
    return passes


def lateral_passes(cruise_us=(1560, 1580), amplitudes_us=(60, 120, 180), step_s=1.0, still_s=3.0,
                   spinup_s=3.0, sweep_s=8.0, area_m=None, p=None):
    """Short passes at constant throttle: one steering doublet (left, right, centre) per
    amplitude, plus one 0.3 -> 3 Hz sweep per speed. Two cruise speeds separate v_char from
    steer_gain. Each pass starts standing still so the gyro bias is observed.
    Sweeps are shortened (down to 3 s) until they fit the test area under the prior model."""
    area_m = area_m or TEST_AREA_M
    passes = []
    for c in cruise_us:
        for a in amplitudes_us:
            passes.append(holds((still_s, NEUTRAL, NEUTRAL), (spinup_s, c, NEUTRAL),
                                (step_s, c, NEUTRAL + a), (step_s, c, NEUTRAL - a), (step_s, c, NEUTRAL),
                                (2.0, NEUTRAL, NEUTRAL)))
        for dur in np.arange(sweep_s, 2.9, -0.5):
            n = int(dur / 0.1)
            f = np.geomspace(0.3, 3.0, n)
            phase = np.cumsum(2 * np.pi * f * 0.1)
            sweep = (holds((still_s, NEUTRAL, NEUTRAL), (spinup_s, c, NEUTRAL))
                     + [(0.1, c, NEUTRAL + 80 * np.sin(ph)) for ph in phase]
                     + holds((2.0, NEUTRAL, NEUTRAL)))
            if check_fits_area([sweep], area_m, p)[0][2]:
                break
        passes.append(sweep)
    return passes


def check_fits_area(passes, area_m=TEST_AREA_M, p=None):
    """Footprint of each pass vs the test area -> list of (index, footprint, fits)."""
    out = []
    for i, sched in enumerate(passes):
        fp = footprint(sched, p)
        long_side, short_side = max(fp["x_extent_m"], fp["y_extent_m"]), min(fp["x_extent_m"], fp["y_extent_m"])
        out.append((i, fp, long_side <= max(area_m) and short_side <= min(area_m)))
    return out


def manual_like_design(duration_s=120.0, seed=0, rate_hz=20.0):
    """Smooth random driving, roughly like the 2023 hand-driven log - the baseline to beat."""
    rng = np.random.default_rng(seed)
    n = int(duration_s * rate_hz)

    def smooth(scale, corr_s):
        a = np.exp(-1.0 / (corr_s * rate_hz))
        w = rng.normal(0, scale * np.sqrt(1 - a * a), n)
        out = np.empty(n)
        out[0] = 0.0
        for k in range(1, n):
            out[k] = a * out[k - 1] + w[k]
        return out

    thr = np.clip(NEUTRAL + 45 + smooth(35, 3.0), 1420, 1650)
    st = np.clip(NEUTRAL + smooth(120, 1.5), 1250, 1750)
    return [(1.0 / rate_hz, a, b) for a, b in zip(thr, st)]


def footprint(schedule, p=None, wheelbase=None):
    """Simulated path extent (m) and top speed for a schedule - does it fit in the test area?"""
    p = p or default_params()
    t_cmd, thr, st = schedule_signal(schedule)
    from sysid_model import input_signal, WHEELBASE_M
    grid = np.arange(0, t_cmd[-1], 0.01)
    out = simulate(p, grid, input_signal(t_cmd, thr), input_signal(t_cmd, st), wheelbase=wheelbase or WHEELBASE_M)
    return {"x_extent_m": float(np.ptp(out["x"])), "y_extent_m": float(np.ptp(out["y"])),
            "top_speed_mps": float(out["v"].max()), "duration_s": float(t_cmd[-1])}


# ---------------------------------------------------------------------------------------------
# synthetic sensors
# ---------------------------------------------------------------------------------------------

SENSORS = {
    # rate Hz, noise
    "gyro": {"rate_hz": 88.0, "noise": 0.02},                   # rad/s, OLA at ~88 Hz
    "encoder": {"rate_hz": 50.0, "noise_frac": 0.02, "noise": 0.05},  # m/s
    "gps": {"rate_hz": 10.0, "noise": 0.15, "lag_s": 0.2},      # m/s ground speed; receiver filter lag
}


def synthesize(schedule, p_true, speed_sensor="encoder", seed=0, cmd_rate_hz=20.0, jitter_s=0.005,
               wheelbase=None):
    """Run the model as 'the real car' and return what the logger would record."""
    from sysid_model import input_signal, WHEELBASE_M
    rng = np.random.default_rng(seed)
    t_cmd, thr, st = schedule_signal(schedule, cmd_rate_hz)
    grid = np.arange(0, t_cmd[-1], 0.002)
    truth = simulate(p_true, grid, input_signal(t_cmd, thr), input_signal(t_cmd, st),
                     wheelbase=wheelbase or WHEELBASE_M)

    def sample(rate):
        t = np.arange(0.5, t_cmd[-1] - 0.5, 1.0 / rate)
        return t + rng.normal(0, jitter_s, t.size).clip(-0.4 / rate, 0.4 / rate)

    tg = np.sort(sample(SENSORS["gyro"]["rate_hz"]))
    gyro = np.interp(tg, grid, truth["r"]) + rng.normal(0, SENSORS["gyro"]["noise"], tg.size)

    s = SENSORS[speed_sensor]
    tv = np.sort(sample(s["rate_hz"]))
    v = np.interp(tv - s.get("lag_s", 0.0), grid, truth["v"])
    v_meas = v + rng.normal(0, s["noise"], tv.size) + rng.normal(0, s.get("noise_frac", 0.0), tv.size) * v
    v_meas = np.maximum(v_meas, 0.0)

    # command log: timestamps jittered like the PC receive time
    t_log = t_cmd + rng.normal(0, jitter_s, t_cmd.size).clip(0, None)
    return {"t_cmd": t_log, "throttle_us": thr, "steer_us": st,
            "t_gyro": tg, "yaw_rate": gyro, "t_speed": tv, "speed": v_meas, "truth": truth}


def lateral_segment(log):
    """Package a synthetic (or real) log for fit_lateral, speed interpolated to gyro times."""
    return {"t_cmd": log["t_cmd"], "steer_us": log["steer_us"], "t_meas": log["t_gyro"],
            "yaw_rate": log["yaw_rate"], "v_meas": np.interp(log["t_gyro"], log["t_speed"], log["speed"])}


def longitudinal_segment(log):
    return {"t_cmd": log["t_cmd"], "throttle_us": log["throttle_us"], "t_meas": log["t_speed"],
            "v_meas": log["speed"]}
