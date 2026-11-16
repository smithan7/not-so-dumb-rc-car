"""System ID from logged sessions: extract, fit, check gate A, and plan what to drive next.

    python sysid_run.py logs/session_*_gs [more sessions] --max-speed 3 --out fit.json
    python sysid_run.py --max-speed 2            # no data yet: just the first plan

Input: ground_station.py sessions (esp32 + ola streams; sysid_log.load_session aligns clocks).
- input signal = the ESP32's *actual outputs* (out_speed_us / out_steer_us, 50 Hz, ESP32 clock):
  what the ESC and servo were given, after arbitration and clamping, whoever was driving. The
  fitted input_delay is therefore ESP32 output -> wheels (servo frame + actuator), not the link.
- speed = driveshaft encoder: one sample per rising edge, distance per edge / edge interval,
  stamped mid-interval; 0 when no edge for 0.5 s
- yaw rate = OLA gyro z (+-250 dps), sign/axis configurable for how the board is mounted
- the log is cut into pieces at standstills, so every piece starts from a known state

Fit: lateral first (it identifies input_delay; multi-start on v_char, which steering at a single
speed can't pin), then longitudinal with that delay. Gate A uses uncertainties backed by a weak
prior, so a parameter the data never excited (e.g. brake_gain with no braking) shows as unknown
rather than as the fit's misleading 0. Then sysid_design plans the next maneuvers.
"""

import argparse
import json
import math
import os
import sys
import warnings
from dataclasses import dataclass

import numpy as np

import esp32_protocol as P
import sysid_design as SD
import sysid_log as L
import sysid_model as M

STOPPED_S = 0.5          # no encoder edge for this long = standing still
PIECE_PAD_S = 1.0        # standstill kept either side of a piece
MIN_STOP_S = 0.6         # a shorter stop doesn't split pieces
MIN_PIECE_SPEED = 0.3    # m/s; slower pieces carry too little information
STEERING_ACTIVE_US = 40  # steering swing that makes a piece useful for the lateral fit
_U32 = 2 ** 32


@dataclass
class Piece:
    session: str
    t0: float
    t1: float
    long: dict          # fit_longitudinal segment
    lat: dict           # fit_lateral segment, or None (no IMU)
    steering: bool
    max_speed: float
    max_throttle_us: float


@dataclass
class Data:
    pieces: list
    inputs: str = "out"
    notes: list = None


# ---------------------------------------------------------------------------------------------
# extraction
# ---------------------------------------------------------------------------------------------

def encoder_speed(s, meters_per_edge):
    """Speed samples from status rows (aligned 't', ESP32 now_us / enc_*): (t, v) arrays."""
    t = s["t"].to_numpy()
    now = s["now_us"].to_numpy(np.int64)
    count = s["enc_count"].to_numpy(np.int64)
    last = s["enc_last_edge_us"].to_numpy(np.int64)
    dt = s["enc_dt_us"].to_numpy(np.int64)
    since = ((now - last) % _U32) / 1e6
    out_t, out_v = [], []
    prev = None
    for i in range(len(t)):
        if count[i] == 0 or since[i] > STOPPED_S:
            out_t.append(t[i])
            out_v.append(0.0)
        elif count[i] != prev and 0 < dt[i] < STOPPED_S * 1e6:
            d = dt[i] / 1e6
            out_t.append(t[i] - since[i] - d / 2.0)  # mid-interval of the latest edge pair
            out_v.append(meters_per_edge / d)
        prev = count[i]
    order = np.argsort(out_t, kind="stable")
    return np.asarray(out_t)[order], np.asarray(out_v)[order]


def _runs(t, max_gap=0.3):
    cuts = np.flatnonzero(np.diff(t) > max_gap) + 1
    return np.split(np.arange(len(t)), cuts)


def extract(session_dirs, yaw_axis="gz", yaw_sign=1.0, gyro_fs_dps=250, meters_per_edge=None):
    """Sessions -> Data (pieces ready for the fits)."""
    mpe = meters_per_edge or P.meters_per_edge()
    pieces, notes = [], []
    for sd in session_dirs:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            sess = L.load_session(sd)
        esp = sess.get("esp32")
        if esp is None or esp.empty or "kind" not in esp:
            notes.append("%s: no ESP32 status" % sd)
            continue
        s = esp[esp["kind"] == "S"].sort_values("t").reset_index(drop=True)
        gyro = None
        ola = sess.get("ola")
        if ola is not None and len(ola) > 10:
            ola = L.ola_to_si(ola.sort_values("t").reset_index(drop=True), accel_fs_g=16, gyro_fs_dps=gyro_fs_dps)
            gyro = (ola["t"].to_numpy(), yaw_sign * ola[yaw_axis].to_numpy())
        else:
            notes.append("%s: no IMU stream - lateral fit not possible" % os.path.basename(sd))
        for idx in _runs(s["t"].to_numpy()):
            run = s.iloc[idx]
            t_cmd = run["t"].to_numpy()
            thr = run["out_speed_us"].to_numpy(float)
            st = run["out_steer_us"].to_numpy(float)
            tv, v = encoder_speed(run, mpe)
            if len(tv) < 5:
                continue
            moving = np.interp(t_cmd, tv, v) > 0.05
            # pieces: motion intervals padded with standstill, merged if they touch
            spans, i = [], 0
            while i < len(moving):
                if moving[i]:
                    j = i
                    while j < len(moving) and moving[j]:
                        j += 1
                    spans.append([t_cmd[i] - PIECE_PAD_S, t_cmd[j - 1] + PIECE_PAD_S])
                    i = j
                else:
                    i += 1
            # neighbours: split the standstill between them (each side gets up to PIECE_PAD_S);
            # merge only if they never really stopped (gap shorter than MIN_STOP_S)
            merged = []
            for a, b in spans:
                a, b = a + PIECE_PAD_S, b - PIECE_PAD_S  # back to the raw motion interval
                if merged:
                    gap = a - merged[-1][1]
                    if gap < MIN_STOP_S:
                        merged[-1][1] = b
                        continue
                merged.append([a, b])
            for i, (a, b) in enumerate(merged):
                pad_a = PIECE_PAD_S if i == 0 else min(PIECE_PAD_S, (a - merged[i - 1][1]) / 2)
                pad_b = PIECE_PAD_S if i == len(merged) - 1 else min(PIECE_PAD_S, (merged[i + 1][0] - b) / 2)
                merged[i] = [max(a - pad_a, t_cmd[0]), min(b + pad_b, t_cmd[-1])]
            for a, b in merged:
                mv = (tv >= a) & (tv <= b)
                mc = (t_cmd >= a - 0.5) & (t_cmd <= b)
                if mv.sum() < 10 or v[mv].max() < MIN_PIECE_SPEED:
                    continue
                tvp, vp = tv[mv], v[mv]
                if vp[0] != 0.0:  # pieces start from standstill
                    tvp, vp = np.r_[a, tvp], np.r_[0.0, vp]
                long = {"t_cmd": t_cmd[mc], "throttle_us": thr[mc], "t_meas": tvp, "v_meas": vp}
                lat = None
                steering = bool(np.ptp(st[mc]) > STEERING_ACTIVE_US)
                if gyro is not None:
                    mg = (gyro[0] >= a + 0.5) & (gyro[0] <= b)
                    if mg.sum() > 50:
                        tg = gyro[0][mg]
                        lat = {"t_cmd": t_cmd[mc], "steer_us": st[mc], "t_meas": tg, "yaw_rate": gyro[1][mg],
                               "v_meas": np.interp(tg, tv, v)}
                pieces.append(Piece(os.path.basename(sd), a, b, long, lat, steering, float(vp.max()),
                                    float(thr[mc].max())))
    return Data(pieces, "out", notes)


# ---------------------------------------------------------------------------------------------
# fitting
# ---------------------------------------------------------------------------------------------

def _cost(fit):
    return float(np.sum(fit.residuals ** 2))


def fit_lateral_multistart(segments, p0=None, starts=(None, 5.0, 40.0)):
    best = None
    for vc in starts:
        p = dict(p0 or {})
        if vc is not None:
            p["v_char"] = vc
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            f = M.fit_lateral(segments, p0=p)
        if best is None or _cost(f) < _cost(best):
            best = f
    return best


def fit_all(pieces, p0=None):
    """-> (params, fit_long or None, fit_lat or None)."""
    p = M.default_params(**(p0 or {}))
    lat_segs = [pc.lat for pc in pieces if pc.lat is not None and pc.steering]
    fit_lat = fit_long = None
    if lat_segs:
        fit_lat = fit_lateral_multistart(lat_segs, p0=p)
        p.update({k: fit_lat.params[k] for k in fit_lat.free})
    long_segs = [pc.long for pc in pieces]
    if long_segs:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            fit_long = M.fit_longitudinal(long_segs, p0=p)
        p.update({k: fit_long.params[k] for k in fit_long.free})
    return p, fit_long, fit_lat


def predict_piece(p, pc, kind):
    """Model prediction for one piece -> (measured, predicted)."""
    if kind == "longitudinal":
        s = pc.long
        grid = np.arange(s["t_meas"][0], s["t_meas"][-1] + 0.01, 0.01)
        v = M.simulate_speed(p, grid, M.input_signal(s["t_cmd"], s["throttle_us"]), v0=max(s["v_meas"][0], 0.0))
        return s["v_meas"], np.interp(s["t_meas"], grid, v)
    s = pc.lat
    grid = np.arange(s["t_meas"][0] - 0.5, s["t_meas"][-1] + 0.005, 0.005)
    delta = np.interp(s["t_meas"], grid, M.simulate_steering(p, grid, M.input_signal(s["t_cmd"], s["steer_us"])))
    return s["yaw_rate"], M.yaw_rate(p, s["v_meas"], delta)


def holdout(pieces, params):
    """Gate A hold-out: refit without one piece of each kind, predict it."""
    out = {}
    for kind in ("longitudinal", "lateral"):
        cand = [i for i, pc in enumerate(pieces) if kind == "longitudinal" or (pc.lat is not None and pc.steering)]
        if len(cand) < 2:
            out[kind] = None
            continue
        held = cand[-1]
        rest = [pc for i, pc in enumerate(pieces) if i != held]
        p_fit, _, _ = fit_all(rest, p0=params)
        meas, pred = predict_piece(p_fit, pieces[held], kind)
        out[kind] = {"piece": held, "rmse": float(np.sqrt(np.mean((meas - pred) ** 2))),
                     "limit": 0.25 if kind == "longitudinal" else 0.08}
        out[kind]["ok"] = out[kind]["rmse"] <= out[kind]["limit"]
    return out


# ---------------------------------------------------------------------------------------------
# the pipeline
# ---------------------------------------------------------------------------------------------

def run(session_dirs, limits=None, n_suggest=3, holdout_check=True, yaw_sign=1.0, kinds=None, **kw):
    """Everything: -> report dict (JSON-serialisable)."""
    holdout_check = kw.pop("holdout", holdout_check)
    limits = limits or SD.Limits()
    data = extract(session_dirs, yaw_sign=yaw_sign) if session_dirs else Data([], "out", [])
    params, fit_long, fit_lat = fit_all(data.pieces) if data.pieces else (M.default_params(), None, None)
    state = SD.State.from_fits(fit_long=fit_long, fit_lat=fit_lat)
    std = state.all_std()
    gate = SD.gate(params, std)
    corr = [{"a": a, "b": b, "corr": c} for a, b, c in state.correlated(0.95)]
    if holdout_check and data.pieces:
        ho = holdout(data.pieces, params)
    else:
        ho = {"longitudinal": {"skipped": True}, "lateral": {"skipped": True}}
    noise = SD.Noise(speed=max(fit_long.rmse, 0.03) if fit_long else 0.05, yaw=max(fit_lat.rmse, 0.02) if fit_lat else 0.03)
    plan = SD.suggest(state, params, limits, n=n_suggest, noise=noise, kinds=kinds)
    after = plan[-1].predicted_std if plan else std
    remaining = [r["param"] for r in SD.gate(params, after) if not r["ok"]]
    gate_pass = (all(r["ok"] for r in gate if r["gating"]) and not corr
                 and all(h is not None and h.get("ok") for h in ho.values()))
    return {
        "sessions": [os.path.abspath(s) for s in session_dirs],
        "notes": data.notes or [],
        "pieces": [{"session": pc.session, "t0": pc.t0, "t1": pc.t1, "steering": pc.steering,
                    "max_speed_mps": pc.max_speed, "max_throttle_us": pc.max_throttle_us,
                    "imu": pc.lat is not None} for pc in data.pieces],
        "params": {k: float(v) for k, v in params.items()},
        "std": {k: float(v) for k, v in std.items()},
        "rmse": {"speed": fit_long.rmse if fit_long else None, "yaw": fit_lat.rmse if fit_lat else None},
        "gate": gate, "correlated": corr, "holdout": ho, "gate_pass": bool(gate_pass),
        "limits": {"max_speed_mps": limits.max_speed_mps, "throttle_max_us": limits.throttle_max_us},
        "suggestions": [{"name": "suggested: " + s.maneuver.name, "kind": s.maneuver.kind,
                         "schedule": [list(map(float, x)) for x in s.maneuver.schedule],
                         "duration_s": s.maneuver.duration_s, "why": s.describe(params),
                         "improves": [[k, float(b), float(a)] for k, b, a in s.improves]} for s in plan],
        "remaining_after_plan": remaining,
        "wheelbase_m": M.WHEELBASE_M,
    }


def format_report(rep):
    lines = []
    lines.append("Data: %d piece(s) from %d session(s)" % (len(rep["pieces"]), len(rep["sessions"])))
    for i, pc in enumerate(rep["pieces"]):
        lines.append("  %2d  %s  %6.1f-%6.1f s  up to %.1f m/s, %d us%s%s" % (
            i, pc["session"], pc["t0"], pc["t1"], pc["max_speed_mps"], pc["max_throttle_us"],
            ", steering" if pc["steering"] else "", "" if pc["imu"] else ", no IMU"))
    for n in rep["notes"]:
        lines.append("  note: " + n)
    if rep["rmse"]["speed"] is not None or rep["rmse"]["yaw"] is not None:
        lines.append("Fit rmse: speed %s m/s, yaw %s rad/s" % (
            "-" if rep["rmse"]["speed"] is None else "%.3f" % rep["rmse"]["speed"],
            "-" if rep["rmse"]["yaw"] is None else "%.3f" % rep["rmse"]["yaw"]))
    lines.append("")
    lines.append("Gate A %s   (1-sigma vs target; * = gating; 1-sigma is optimistic)" % ("PASS" if rep["gate_pass"] else "not yet"))
    lines.append("  %-17s %12s %12s %10s" % ("param", "value", "1-sigma", "target"))
    for r in rep["gate"]:
        lines.append("  %-17s %12.5g %12s %10s  %s%s" % (r["param"], r["value"], r["std_text"], r["target_text"],
                                                      "ok" if r["ok"] else "--", " *" if r["gating"] else ""))
    for c in rep["correlated"]:
        lines.append("  correlated: %s / %s  %.3f (> 0.95: not separable with this data)" % (c["a"], c["b"], c["corr"]))
    for kind, h in rep["holdout"].items():
        if h is None:
            text = "needs >= 2 pieces of this kind"
        elif h.get("skipped"):
            text = "skipped"
        else:
            text = "rmse %.3f (limit %.2f) %s" % (h["rmse"], h["limit"], "ok" if h["ok"] else "--")
        lines.append("  hold-out %-12s %s" % (kind, text))
    lines.append("")
    lines.append("Suggested next (stage cap %.1f m/s, throttle <= %d us):" % (rep["limits"]["max_speed_mps"],
                                                                           rep["limits"]["throttle_max_us"]))
    if not rep["suggestions"]:
        lines.append("  nothing - every target is predicted to be met")
    for i, s in enumerate(rep["suggestions"], 1):
        lines.append("  %d. %s  (%.0f s)" % (i, s["name"].replace("suggested: ", ""), s["duration_s"]))
        lines.append("     " + (s["why"] or "general improvement"))
    if rep["remaining_after_plan"]:
        lines.append("  still short after this plan: %s - refit after driving it; if it stays short, the "
                     "stage's speed cap may be too low to excite it (drag ~ v^2)" % ", ".join(rep["remaining_after_plan"]))
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="*", help="session directories (ground_station.py logs)")
    ap.add_argument("--max-speed", type=float, default=3.0, help="stage speed cap for suggestions, m/s")
    ap.add_argument("--throttle-max", type=int, default=1600, help="firmware throttle clamp, us")
    ap.add_argument("--n", type=int, default=3, help="number of maneuvers to suggest")
    ap.add_argument("--yaw-sign", type=float, default=1.0, help="-1 if the OLA's z axis points down")
    ap.add_argument("--no-holdout", action="store_true", help="skip the hold-out refits (faster)")
    ap.add_argument("--out", help="write the report as JSON")
    args = ap.parse_args(argv)
    rep = run(args.sessions, SD.Limits(max_speed_mps=args.max_speed, throttle_max_us=args.throttle_max),
              n_suggest=args.n, holdout_check=not args.no_holdout, yaw_sign=args.yaw_sign)
    print(format_report(rep))
    if args.out:
        with open(args.out, "w") as fh:
            json.dump(rep, fh, indent=1)
        print("\nwrote " + args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
