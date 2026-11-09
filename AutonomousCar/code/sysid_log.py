"""Parse and time-align logs recorded by sysid_logger.py (and salvage the 2023 pickles).

A session directory holds one raw CSV per serial stream (``t_pc_ns,line``) plus ``meta.json``.
Nothing is parsed while recording; everything happens here, offline.

Time alignment
--------------
Every line is stamped with the PC's monotonic clock when it was *read*. That stamp is late by a
variable amount (USB/serial buffering, logger load). Streams that carry their own clock (OLA RTC,
GPS UTC, Arduino millis) are re-timed onto the PC timebase:

    t_pc = t_dev + offset + drift * t_dev + latency,   latency >= 0

offset/drift come from a line fit to the lower envelope of (t_pc - t_dev), i.e. the samples that
arrived fastest. ``latency`` is then reported per sample, so a logger that fell behind is obvious
instead of silently corrupting the data (which is what happened to the 2023 OLA log).

All aligned times ``t`` are seconds since the session start on the PC monotonic clock.
"""

import csv
import json
import math
import os
import re
import warnings

import numpy as np
import pandas as pd

# ---------------------------------------------------------------------------------------------
# line classification / parsing
# ---------------------------------------------------------------------------------------------

_OLA_DATE = re.compile(r"^\d{2}/\d{2}/\d{4}$")
_OLA_TIME = re.compile(r"^\d{1,2}:\d{2}:\d{2}(\.\d+)?$")

OLA_FIELDS = ["q1", "q2", "q3", "head_acc",
              "ax_raw", "ay_raw", "az_raw", "gx_raw", "gy_raw", "gz_raw",
              "mx_raw", "my_raw", "mz_raw", "output_hz"]


def classify_line(line):
    """Return the stream kind a line belongs to: 'ola', 'gps', 'pwm', 'cmd' or None."""
    s = line.strip()
    if not s:
        return None
    if s.startswith("$G"):  # NMEA ($GNGGA, $GPRMC, ...) and the custom "$GPS," line
        return "gps"
    if s.startswith("PWM"):
        return "pwm"
    head = s.split(",", 1)[0].strip()
    if _OLA_DATE.match(head):
        return "ola"
    if head in ("C", "Z", "E"):
        return "cmd"
    return None


def _hms_to_s(hms):
    """'hh:mm:ss.ss' -> seconds since midnight."""
    h, m, s = hms.split(":")
    return int(h) * 3600 + int(m) * 60 + float(s)


def _ola_date_to_day(date):
    """OLA RTC date 'mm/dd/yyyy' -> ordinal day, so midnight rollovers don't break timing."""
    mm, dd, yyyy = (int(x) for x in date.split("/"))
    return pd.Timestamp(year=yyyy, month=mm, day=dd).toordinal()


def parse_ola(line):
    f = [x.strip() for x in line.strip().split(",")]
    if len(f) < 2 + len(OLA_FIELDS) or not _OLA_DATE.match(f[0]) or not _OLA_TIME.match(f[1]):
        return None
    try:
        vals = [float(x) for x in f[2:2 + len(OLA_FIELDS)]]
        day = _ola_date_to_day(f[0])
    except ValueError:
        return None
    row = dict(zip(OLA_FIELDS, vals))
    row["t_dev"] = day * 86400.0 + _hms_to_s(f[1])
    return row


def nmea_checksum_ok(sentence):
    s = sentence.strip()
    if not s.startswith("$") or "*" not in s:
        return False
    body, _, cs = s[1:].partition("*")
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    try:
        return calc == int(cs[:2], 16)
    except ValueError:
        return False


def _nmea_deg(value, hemi):
    """ddmm.mmmm / dddmm.mmmm + N/S/E/W -> signed decimal degrees."""
    if not value:
        return math.nan
    v = float(value)
    deg = int(v // 100)
    out = deg + (v - deg * 100) / 60.0
    return -out if hemi in ("S", "W") else out


def _nmea_utc(value):
    """hhmmss.ss -> seconds since midnight."""
    v = float(value)
    return int(v // 10000) * 3600 + int(v // 100 % 100) * 60 + (v % 100)


def parse_gps(line):
    """Parse GGA / RMC NMEA sentences and the firmware's custom '$GPS,' line."""
    s = line.strip()
    f = [x.strip() for x in s.split(",")]
    tag = f[0]
    try:
        if tag == "$GPS":
            # custom line from the car firmware: lat*1e7, lon*1e7, then fields whose meaning isn't
            # documented in the repo - keep them raw. No device time in this format.
            if len(f) < 3:
                return None
            row = {"sentence": "GPS", "lat": int(f[1]) * 1e-7, "lon": int(f[2]) * 1e-7}
            for i, x in enumerate(f[3:], start=3):
                row[f"f{i}"] = float(x) if x else math.nan
            return row
        if len(tag) == 6 and tag[3:] in ("GGA", "RMC"):
            if not nmea_checksum_ok(s):
                return None
            f[-1] = f[-1].split("*")[0]
            if tag[3:] == "GGA":
                if not f[1]:
                    return None
                return {"sentence": "GGA", "t_utc": _nmea_utc(f[1]),
                        "lat": _nmea_deg(f[2], f[3]), "lon": _nmea_deg(f[4], f[5]),
                        "fix_quality": int(f[6] or 0), "sats": int(f[7] or 0),
                        "hdop": float(f[8]) if f[8] else math.nan,
                        "alt_m": float(f[9]) if f[9] else math.nan}
            if not f[1]:
                return None
            return {"sentence": "RMC", "t_utc": _nmea_utc(f[1]), "valid": f[2] == "A",
                    "lat": _nmea_deg(f[3], f[4]), "lon": _nmea_deg(f[5], f[6]),
                    "speed_mps": float(f[7]) * 0.514444 if f[7] else math.nan,
                    "course_deg": float(f[8]) if f[8] else math.nan}
    except (ValueError, IndexError):
        return None
    return None


def parse_pwm(line):
    """'PWM, status, throttle_us, steer_us[, millis]' (field order as printed by read_pwm.ino)."""
    f = [x.strip() for x in line.strip().split(",")]
    if f[0] != "PWM" or len(f) < 4:
        return None
    try:
        row = {"status": int(float(f[1])), "throttle_us": float(f[2]), "steer_us": float(f[3])}
        if len(f) >= 5 and f[4]:
            row["t_dev"] = float(f[4]) / 1000.0
    except ValueError:
        return None
    if row["status"] != 1:
        row["throttle_us"] = math.nan
        row["steer_us"] = math.nan
    return row


PARSERS = {"ola": parse_ola, "gps": parse_gps, "pwm": parse_pwm}

# ---------------------------------------------------------------------------------------------
# clock alignment
# ---------------------------------------------------------------------------------------------

MAX_DRIFT = 1e-3  # 1000 ppm; real crystal/RTC drift is far below this


def unwrap_clock(t_dev, period):
    """Undo wraps of a device clock that resets every ``period`` seconds (e.g. UTC at midnight)."""
    t = np.asarray(t_dev, dtype=float).copy()
    jumps = np.diff(t) < -period / 2
    t[1:] += np.cumsum(jumps) * period
    return t


def align_device_clock(t_pc, t_dev, bin_s=5.0):
    """Map device timestamps onto the PC clock.

    Returns (t_aligned, latency, info). See module docstring for the model.
    """
    t_pc = np.asarray(t_pc, dtype=float)
    t_dev = np.asarray(t_dev, dtype=float)
    ok = np.isfinite(t_pc) & np.isfinite(t_dev)
    if ok.sum() < 2:
        raise ValueError("need at least two samples with device time to align")
    tp, td = t_pc[ok], t_dev[ok]
    d = tp - td

    # lower envelope: fastest-arriving sample in each bin
    bins = np.floor((td - td[0]) / bin_s).astype(int)
    order = np.lexsort((d, bins))
    first = np.r_[True, np.diff(bins[order]) != 0]
    env_t, env_d = td[order][first], d[order][first]

    drift = 0.0
    if len(env_t) >= 3:
        drift = float(np.polyfit(env_t - td[0], env_d, 1)[0])
    backlog_suspected = abs(drift) > MAX_DRIFT
    if backlog_suspected:
        # a slope this large is the logger falling behind, not clock drift
        drift = 0.0
    offset = float(np.min(d - drift * (td - td[0])))

    t_aligned = t_dev + offset + drift * (t_dev - td[0])
    latency = t_pc - t_aligned
    info = {"offset_s": offset, "drift_ppm": drift * 1e6, "backlog_suspected": bool(backlog_suspected),
            "latency_p50_s": float(np.nanpercentile(latency, 50)),
            "latency_p99_s": float(np.nanpercentile(latency, 99)),
            "latency_max_s": float(np.nanmax(latency))}
    if backlog_suspected or info["latency_p99_s"] > 0.5:
        warnings.warn(f"stream fell behind the logger: latency p99 {info['latency_p99_s']:.2f} s, "
                      f"max {info['latency_max_s']:.2f} s - device-time alignment used", stacklevel=2)
    return t_aligned, latency, info


def _apply_alignment(df, dev_col, period=None):
    """Add 't' (aligned) and 'latency' columns; falls back to t_pc where there is no device time."""
    if dev_col in df and df[dev_col].notna().sum() >= 2:
        td = df[dev_col].to_numpy(dtype=float)
        if period:
            good = np.isfinite(td)
            td[good] = unwrap_clock(td[good], period)
        t, lat, info = align_device_clock(df["t_pc"].to_numpy(), td)
        df["t"] = t
        df["latency"] = lat
        df.loc[~np.isfinite(td), "t"] = df.loc[~np.isfinite(td), "t_pc"]
        return info
    df["t"] = df["t_pc"]
    df["latency"] = np.nan
    return {"offset_s": None, "note": "no device clock - PC receive time used"}


# ---------------------------------------------------------------------------------------------
# sessions
# ---------------------------------------------------------------------------------------------

def read_raw_stream(path):
    """Read a raw '<stream>.csv' into a DataFrame with t_pc (s since session start) and line."""
    with open(path, newline="") as fh:
        rows = list(csv.reader(fh))
    if rows and rows[0] == ["t_pc_ns", "line"]:
        rows = rows[1:]
    t = np.array([int(r[0]) for r in rows], dtype=np.int64)
    lines = [r[1] if len(r) > 1 else "" for r in rows]
    return t, lines


def parse_stream(t_pc_s, lines, kind=None):
    """Parse raw lines of one stream into a DataFrame. Lines that don't parse are counted, not lost."""
    out = []
    bad = 0
    for t, line in zip(t_pc_s, lines):
        k = kind or classify_line(line)
        parser = PARSERS.get(k)
        row = parser(line) if parser else None
        if row is None:
            bad += 1
            continue
        row["t_pc"] = t
        out.append(row)
    df = pd.DataFrame(out)
    df.attrs["unparsed_lines"] = bad
    return df


def load_session(session_dir):
    """Load a sysid_logger session -> dict of aligned DataFrames ('ola', 'gps', 'pwm', ...).

    The returned dict also has 'meta' (meta.json) and 'alignment' (per-stream timing report).
    """
    with open(os.path.join(session_dir, "meta.json")) as fh:
        meta = json.load(fh)
    t0 = int(meta["start_monotonic_ns"])
    out = {"meta": meta, "alignment": {}, "unparsed": {}}
    for name, info in meta["streams"].items():
        path = os.path.join(session_dir, info["file"])
        if not os.path.exists(path):  # port never opened; meta.json has the error
            continue
        t_ns, lines = read_raw_stream(path)
        t_pc = (t_ns - t0) / 1e9
        kind = info.get("kind")
        if kind == "gps":
            df = parse_stream(t_pc, lines, "gps")
            report = {}
            # each sentence type is aligned on its own (custom $GPS lines carry no time)
            parts = []
            for sentence, part in df.groupby("sentence", sort=False) if len(df) else []:
                part = part.copy()
                report[sentence] = _apply_alignment(part, "t_utc", period=86400.0)
                parts.append(part)
            df = pd.concat(parts).sort_values("t") if parts else df
            out["alignment"][name] = report
        elif kind in PARSERS:
            df = parse_stream(t_pc, lines, kind)
            out["alignment"][name] = _apply_alignment(df, "t_dev") if len(df) else {}
        else:  # unknown / command streams: keep raw lines with PC time
            df = pd.DataFrame({"t_pc": t_pc, "line": lines})
            df["t"] = df["t_pc"]
        out["alignment"].setdefault(name, {})
        out["unparsed"][name] = df.attrs.get("unparsed_lines", 0)
        out[name] = df.reset_index(drop=True)
    return out


def load_legacy_pickles(code_dir="."):
    """Re-time the 2023 data_logger_good_place pickles (OLA/GPS/PWM_data.pkl) with device clocks.

    Note on PWM: the old logger labelled the fields (Steer, Throttle) but read_pwm.ino prints
    (throttle, steer) - so the old 'Steer' column is the firmware's throttle channel (pin 2).
    Confirm against the receiver wiring before using it.

    These pickles are this repo's own committed logs (pickle can run code, so never point this
    at files from elsewhere).
    """
    out = {"alignment": {}, "unparsed": {}}
    ola = pd.read_pickle(os.path.join(code_dir, "OLA_data.pkl"))
    df = pd.DataFrame({"t_pc": ola.Wall_Time.astype(float)})
    df["t_dev"] = [_ola_date_to_day(d) * 86400.0 + _hms_to_s(t) for d, t in zip(ola.rtcDate, ola.rtcTime)]
    legacy = dict(zip(["Q9_1", "Q9_2", "Q9_3", "HeadAcc", "RawAX", "RawAY", "RawAZ", "RawGX", "RawGY", "RawGZ",
                       "RawMX", "RawMY", "RawMZ", "output_Hz"], OLA_FIELDS))
    for old, new in legacy.items():
        df[new] = ola[old].astype(float).to_numpy()
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        out["alignment"]["ola"] = _apply_alignment(df, "t_dev")
    out["ola"] = df

    gps = pd.read_pickle(os.path.join(code_dir, "GPS_data.pkl"))
    df = pd.DataFrame({"t_pc": gps.Wall_Time.astype(float), "sentence": "GGA",
                       "t_utc": [_nmea_utc(u) for u in gps["utc time"]],
                       "lat": gps.Lat.astype(float), "lon": gps.Lon.astype(float),
                       "fix_quality": gps.Quality_Indicator.astype(int), "sats": gps["Satellites Used"].astype(int),
                       "hdop": gps.HDOP.astype(float), "alt_m": gps.Altitude.astype(float)})
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        out["alignment"]["gps"] = _apply_alignment(df, "t_utc", period=86400.0)
    out["gps"] = df

    pwm = pd.read_pickle(os.path.join(code_dir, "pwm_data.pkl"))
    df = pd.DataFrame({"t_pc": pwm.Wall_Time.astype(float), "status": pwm.Status.astype(int),
                       "throttle_us": pwm.Steer.astype(float), "steer_us": pwm.Throttle.astype(float)})
    df.loc[df.status != 1, ["throttle_us", "steer_us"]] = np.nan
    out["alignment"]["pwm"] = _apply_alignment(df, "t_dev")
    out["pwm"] = df
    return out


# ---------------------------------------------------------------------------------------------
# IMU units
# ---------------------------------------------------------------------------------------------

ACCEL_LSB_PER_G = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}     # ICM-20948 full-scale options
GYRO_LSB_PER_DPS = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}
G = 9.80665


def estimate_accel_full_scale(ola, window_s=2.0):
    """Infer the accelerometer full-scale range (g) from the stillest stretch (|a| = 1 g there)."""
    a = np.sqrt(ola.ax_raw**2 + ola.ay_raw**2 + ola.az_raw**2).to_numpy()
    t = ola["t"].to_numpy()
    if len(a) < 10:
        raise ValueError("not enough IMU samples")
    n = max(5, int(np.searchsorted(t, t[0] + window_s)))
    stds = pd.Series(a).rolling(n).std().to_numpy()
    i = int(np.nanargmin(stds))
    one_g = float(np.median(a[max(0, i - n + 1):i + 1]))
    fs = min(ACCEL_LSB_PER_G, key=lambda k: abs(math.log(ACCEL_LSB_PER_G[k] / one_g)))
    return fs, one_g


def ola_to_si(ola, accel_fs_g=None, gyro_fs_dps=250):
    """Add ax/ay/az (m/s^2), gx/gy/gz (rad/s) and accel_saturated columns to an OLA DataFrame.

    accel_fs_g=None estimates the range from a still period. The gyro range can't be inferred from
    data, so it must match the OLA config (ICM-20948 default is 250 dps).
    """
    if accel_fs_g is None:
        accel_fs_g, _ = estimate_accel_full_scale(ola)
    a_scale = G / ACCEL_LSB_PER_G[accel_fs_g]
    g_scale = math.radians(1.0) / GYRO_LSB_PER_DPS[gyro_fs_dps]
    out = ola.copy()
    for ax in "xyz":
        out[f"a{ax}"] = out[f"a{ax}_raw"] * a_scale
        out[f"g{ax}"] = out[f"g{ax}_raw"] * g_scale
    out["accel_saturated"] = (out[["ax_raw", "ay_raw", "az_raw"]].abs() >= 32760).any(axis=1)
    out.attrs.update(accel_fs_g=accel_fs_g, gyro_fs_dps=gyro_fs_dps)
    return out


def summarize(session):
    """Human-readable per-stream report: rate, gaps, latency, unparsed lines."""
    lines = []
    for name, df in session.items():
        if name in ("meta", "alignment", "unparsed") or not isinstance(df, pd.DataFrame) or df.empty:
            continue
        t = np.sort(df["t"].to_numpy())
        dt = np.diff(t)
        span = t[-1] - t[0] if len(t) > 1 else 0.0
        rate = (len(t) - 1) / span if span > 0 else math.nan
        gap = dt.max() if len(dt) else math.nan
        lines.append(f"{name:>4}: {len(t):6d} rows, {span:7.1f} s, {rate:6.1f} Hz, max gap {gap:5.2f} s")
        al = session.get("alignment", {}).get(name, {})
        reports = list(al.values()) if al and all(isinstance(v, dict) for v in al.values()) else [al]
        for rep in reports:
            if rep.get("latency_p99_s") is not None:
                flag = "  <-- LOGGER FELL BEHIND" if rep.get("backlog_suspected") or rep["latency_p99_s"] > 0.5 else ""
                lines.append(f"        latency p50 {rep['latency_p50_s']*1e3:6.0f} ms, p99 {rep['latency_p99_s']*1e3:7.0f} ms,"
                             f" max {rep['latency_max_s']:6.2f} s, drift {rep['drift_ppm']:7.1f} ppm{flag}")
            elif rep.get("note"):
                lines.append(f"        {rep['note']}")
        bad = session.get("unparsed", {}).get(name)
        if bad:
            lines.append(f"        {bad} unparsed lines")
    return "\n".join(lines)
