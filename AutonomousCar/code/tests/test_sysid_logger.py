import math
import os
import sys
import threading
import time
import warnings

import numpy as np
import pytest

CODE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, CODE_DIR)

import sysid_log as L  # noqa: E402
import sysid_logger as R  # noqa: E402


def nmea(body):
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return f"${body}*{cs:02X}"


# --- parsing --------------------------------------------------------------------------------

OLA_LINE = "01/03/2000,17:56:24.64,0.09383,0.05380,-0.01004,316,-68,-136,-8012,27,7,-38,20,-116,-360,88.333,"
GPS_CUSTOM = "$GPS, 394265129, -1198031113, 1463029, 161, 22599525, 0"
GGA = nmea("GNGGA,192300.10,3926.34888,N,11947.09958,W,2,10,0.85,1393.8,M,-23.9,M,,0000")
RMC = nmea("GNRMC,192300.10,A,3926.34888,N,11947.09958,W,10.0,45.0,080323,,,D")


def test_classify():
    assert L.classify_line(OLA_LINE) == "ola"
    assert L.classify_line(GPS_CUSTOM) == "gps"
    assert L.classify_line(GGA) == "gps"
    assert L.classify_line("PWM, 1, 1500, 1480, 1234") == "pwm"
    assert L.classify_line("Artemis OpenLog v2.3") is None
    assert L.classify_line("") is None


def test_parse_ola():
    r = L.parse_ola(OLA_LINE)
    assert r["az_raw"] == -8012 and r["output_hz"] == pytest.approx(88.333)
    assert r["t_dev"] % 86400 == pytest.approx(17 * 3600 + 56 * 60 + 24.64)
    assert L.parse_ola("01/03/2000,17:56:24.64,0.1,0.2") is None  # truncated line


def test_parse_nmea():
    g = L.parse_gps(GGA)
    assert g["lat"] == pytest.approx(39 + 26.34888 / 60)
    assert g["lon"] == pytest.approx(-(119 + 47.09958 / 60))
    assert g["t_utc"] == pytest.approx(19 * 3600 + 23 * 60 + 0.1)
    assert g["sats"] == 10 and g["fix_quality"] == 2
    r = L.parse_gps(RMC)
    assert r["speed_mps"] == pytest.approx(5.14444) and r["valid"]
    assert L.parse_gps(GGA[:-2] + "00") is None  # bad checksum rejected


def test_parse_custom_gps_and_pwm():
    g = L.parse_gps(GPS_CUSTOM)
    assert g["lat"] == pytest.approx(39.4265129) and g["lon"] == pytest.approx(-119.8031113)
    p = L.parse_pwm("PWM, 1, 1520, 1480, 61234")
    assert p == {"status": 1, "throttle_us": 1520.0, "steer_us": 1480.0, "t_dev": 61.234}
    lost = L.parse_pwm("PWM, 0, -1, -1, 100")
    assert math.isnan(lost["throttle_us"])
    assert "t_dev" not in L.parse_pwm("PWM, 1, 1500, 1500")  # old firmware without millis


# --- alignment ------------------------------------------------------------------------------

def test_align_recovers_offset_and_drift():
    rng = np.random.default_rng(0)
    t_dev = np.arange(0, 300, 0.01)
    true = 12.5 + t_dev * (1 + 80e-6)                 # 80 ppm crystal drift
    lat = rng.exponential(0.004, t_dev.size)          # USB jitter
    lat[rng.random(t_dev.size) < 0.01] += 0.2         # occasional stalls
    t_aligned, latency, info = L.align_device_clock(true + lat, t_dev)
    assert not info["backlog_suspected"]
    assert info["drift_ppm"] == pytest.approx(80, abs=5)
    assert np.max(np.abs(t_aligned - true)) < 0.003
    assert latency.min() >= -1e-9


def test_align_flags_logger_backlog():
    t_dev = np.arange(0, 100, 0.01)
    t_pc = t_dev * 2.5                                # reading slower than the device produces
    with pytest.warns(UserWarning, match="fell behind"):
        t_aligned, latency, info = L.align_device_clock(t_pc, t_dev)
    assert info["backlog_suspected"]
    assert latency[-1] == pytest.approx(150, rel=0.01)
    assert t_aligned[-1] - t_aligned[0] == pytest.approx(t_dev[-1] - t_dev[0])


def test_unwrap_utc_midnight():
    t = np.array([86399.8, 86399.9, 0.0, 0.1])
    assert np.allclose(L.unwrap_clock(t, 86400), [86399.8, 86399.9, 86400.0, 86400.1])


# --- recorder end to end --------------------------------------------------------------------

class FakeSerial:
    """Emits lines at a fixed rate like a real device; readline() blocks up to ``timeout``."""

    def __init__(self, make_line, rate_hz, timeout):
        self.make_line = make_line
        self.period = 1.0 / rate_hz
        self.timeout = timeout
        self.t0 = time.monotonic()
        self.i = 0

    def readline(self):
        due = self.t0 + self.i * self.period
        wait = due - time.monotonic()
        if wait > self.timeout:
            time.sleep(self.timeout)
            return b""
        if wait > 0:
            time.sleep(wait)
        line = self.make_line(self.i, self.i * self.period)
        self.i += 1
        return (line + "\r\n").encode()

    def close(self):
        pass


def ola_line(i, t):
    s = 17 * 3600 + 56 * 60 + 24.0 + t
    hh, mm, ss = int(s // 3600), int(s // 60 % 60), s % 60
    return f"01/03/2000,{hh:02d}:{mm:02d}:{ss:05.2f},0.1,0.0,0.0,316,0,0,-8192,0,0,0,0,0,0,88.3,"


def gga_line(i, t):
    s = 19 * 3600 + 23 * 60 + t
    hh, mm, ss = int(s // 3600), int(s // 60 % 60), s % 60
    return nmea(f"GNGGA,{hh:02d}{mm:02d}{ss:05.2f},3926.34888,N,11947.09958,W,2,10,0.85,1393.8,M,-23.9,M,,0000")


def pwm_line(i, t):
    return f"PWM, 1, {1500 + i % 50}, 1480, {int(5000 + t * 1000)}"


FAKE_DEVICES = {"/dev/fake-ola": (ola_line, 88.0), "/dev/fake-gps": (gga_line, 10.0),
                "/dev/fake-pwm": (pwm_line, 20.0), "/dev/fake-silent": (None, 1.0)}


def fake_opener(port, baud, timeout):
    make, rate = FAKE_DEVICES[port]
    if make is None:
        return FakeSerial(lambda i, t: "boot noise", rate, timeout)
    return FakeSerial(make, rate, timeout)


def test_identify_ports():
    found = R.identify_ports(list(FAKE_DEVICES), listen_s=0.6, opener=fake_opener)
    assert found == {"ola": "/dev/fake-ola", "gps": "/dev/fake-gps", "pwm": "/dev/fake-pwm"}


def test_record_and_load_session(tmp_path):
    ports = {"ola": "/dev/fake-ola", "gps": "/dev/fake-gps", "pwm": "/dev/fake-pwm"}
    session = R.record(ports, str(tmp_path), duration_s=1.5, opener=fake_opener, printer=lambda *_: None)
    with warnings.catch_warnings():
        warnings.simplefilter("error")  # a healthy recording must not trigger the backlog warning
        s = L.load_session(session)
    assert s["meta"]["streams"]["ola"]["lines"] > 100
    assert len(s["ola"]) == s["meta"]["streams"]["ola"]["lines"]
    assert len(s["gps"]) >= 12 and len(s["pwm"]) >= 25
    assert s["unparsed"] == {"ola": 0, "gps": 0, "pwm": 0}
    for name in ("ola", "pwm"):
        assert s["alignment"][name]["latency_p99_s"] < 0.05
        assert np.all(np.diff(s[name]["t"]) > 0)
        # aligned times land within the recording window
        assert -0.1 < s[name]["t"].iloc[0] < 0.5 and s[name]["t"].iloc[-1] < 1.6
    assert s["alignment"]["gps"]["GGA"]["latency_p99_s"] < 0.05
    assert "LOGGER FELL BEHIND" not in L.summarize(s)


def test_record_stops_on_event_and_reports_open_errors(tmp_path):
    stop = threading.Event()

    def opener(port, baud, timeout):
        if port == "/dev/missing":
            raise OSError("no such port")
        return fake_opener(port, baud, timeout)

    threading.Timer(0.4, stop.set).start()
    session = R.record({"pwm": "/dev/fake-pwm", "ola": "/dev/missing"}, str(tmp_path),
                       opener=opener, stop_event=stop, printer=lambda *_: None)
    s = L.load_session(session)
    assert "no such port" in s["meta"]["streams"]["ola"]["error"]
    assert s["meta"]["duration_s"] < 2.0 and len(s["pwm"]) > 3


# --- units and legacy data ------------------------------------------------------------------

def test_ola_to_si_detects_range():
    import pandas as pd
    n = 400
    df = pd.DataFrame({"t": np.arange(n) / 88.0, "ax_raw": 0.0, "ay_raw": 0.0, "az_raw": -2048.0,
                       "gx_raw": 0.0, "gy_raw": 0.0, "gz_raw": 131.0})
    out = L.ola_to_si(df)
    assert out.attrs["accel_fs_g"] == 16
    assert out.az.iloc[0] == pytest.approx(-9.80665)
    assert out.gz.iloc[0] == pytest.approx(math.radians(1.0))


def test_legacy_2023_log_shows_backlog():
    s = L.load_legacy_pickles(CODE_DIR)
    assert s["alignment"]["ola"]["latency_max_s"] == pytest.approx(179.6, abs=1.0)
    assert L.estimate_accel_full_scale(s["ola"])[0] == 4
