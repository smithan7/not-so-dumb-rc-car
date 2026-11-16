"""Ground station: a web page to watch and drive the car through the ESP32 (protocol v2).

    python ground_station.py --sim                  # against the car emulator, on this machine
    python ground_station.py --port /dev/ttyUSB0    # the real ESP32 (with esp32_node NOT running)
    then open the printed URL. --host 0.0.0.0 to reach it from a phone/laptop on the car's network.

What it does
- shows live: who is driving (mode / armed), receiver vs Jetson vs actual outputs, speed from the
  driveshaft encoder, link round-trip, the car on a map of the lot (emulator: true pose)
- sends Jetson commands at 50 Hz: neutral hold, hold-to-drive jog (deadman: neutral within 0.3 s
  of letting go), or one of the sysid test passes (sysid_maneuvers) / a custom throttle step
- "Release" stops sending entirely, so the firmware's 200 ms timeout puts the car in failsafe
- logs every line received and command sent into a session directory that
  sysid_log.load_session() reads (same format as sysid_logger.py)

It is not a safety device: stopping the car is the transmitter's ch3 switch / the ESP32's job.
Anyone who can reach the page can drive the car - keep --host on 127.0.0.1 unless you need it.
No dependencies beyond pyserial (standard-library HTTP server, Server-Sent Events).
"""

import argparse
import csv
import datetime
import json
import math
import os
import platform
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

import esp32_protocol as P
import failsafe as F
import sysid_maneuvers as D

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ground_station_web")
COMMAND_HZ = 50.0
JOG_TIMEOUT_S = 0.3
SNAPSHOT_HZ = 20.0


# ---------------------------------------------------------------------------------------------
# session log (sysid_logger format)
# ---------------------------------------------------------------------------------------------

class SessionLog:
    def __init__(self, log_dir, port, notes=""):
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path, i = os.path.join(log_dir, "session_%s_gs" % stamp), 1
        while os.path.exists(path):
            i += 1
            path = os.path.join(log_dir, "session_%s_gs%d" % (stamp, i))
        os.makedirs(path)
        self.path = path
        self.start_ns = time.monotonic_ns()
        self.meta = {"start_wall": datetime.datetime.now().astimezone().isoformat(),
                     "start_monotonic_ns": self.start_ns, "host": platform.node(),
                     "python": sys.version.split()[0], "notes": notes, "recorder": "ground_station.py",
                     "streams": {"esp32": {"port": port, "kind": "esp32", "file": "esp32.csv"},
                                 "cmd": {"port": port, "kind": "esp32_cmd", "file": "cmd.csv"}}}
        self.lock = threading.Lock()
        self.files, self.writers, self.counts = {}, {}, {}
        for name in ("esp32", "cmd"):
            fh = open(os.path.join(path, self.meta["streams"][name]["file"]), "w", newline="")
            w = csv.writer(fh)
            w.writerow(["t_pc_ns", "line"])
            self.files[name], self.writers[name], self.counts[name] = fh, w, 0
        self._last_flush = time.monotonic()
        self._write_meta()

    def _write_meta(self):
        with open(os.path.join(self.path, "meta.json"), "w") as fh:
            json.dump(self.meta, fh, indent=2)

    def write(self, stream, t_ns, line):
        with self.lock:
            if stream not in self.writers:
                return
            self.writers[stream].writerow([t_ns, line])
            self.counts[stream] += 1
            if time.monotonic() - self._last_flush > 0.5:
                for fh in self.files.values():
                    fh.flush()
                self._last_flush = time.monotonic()

    def close(self):
        with self.lock:
            for fh in self.files.values():
                fh.close()
            self.writers = {}
            self.meta["duration_s"] = (time.monotonic_ns() - self.start_ns) / 1e9
            for name, n in self.counts.items():
                self.meta["streams"][name]["lines"] = n
            self._write_meta()


# ---------------------------------------------------------------------------------------------
# serial link
# ---------------------------------------------------------------------------------------------

class Link:
    """Owns the ESP32 serial port: reader thread (parse + log), send() for commands."""

    def __init__(self, port, baud, log, edges_per_rev=P.ENCODER_EDGES_PER_REV, diff_ratio=P.DIFF_RATIO,
                 wheel_diameter_m=P.WHEEL_DIAMETER_M):
        import serial  # lazy: tests of other modules don't need pyserial

        self._serial_mod = serial
        self.port, self.baud, self.log = port, baud, log
        self.drivetrain = (edges_per_rev, diff_ratio, wheel_diameter_m)
        self.lock = threading.Lock()
        self.seq = 0
        self.latency = P.LatencyTracker()
        self.rtts = []
        self.status = None
        self.status_t = None
        self.lines = 0
        self.unparsed = 0
        self.error = None
        self.ser = None
        self._running = True
        self._open()
        self._thread = threading.Thread(target=self._read_loop, name="link_reader", daemon=True)
        self._thread.start()

    def _open(self):
        try:
            s = self._serial_mod.Serial()
            s.port, s.baudrate, s.timeout = self.port, self.baud, 0.2
            s.dtr = False  # don't reset the ESP32 (see esp32_node)
            s.rts = False
            s.open()
            self.ser, self.error = s, None
        except (self._serial_mod.SerialException, OSError) as e:
            self.ser, self.error = None, str(e)

    @property
    def connected(self):
        return self.ser is not None

    def send(self, steer_us, speed_us):
        line = P.format_command(self.seq, steer_us, speed_us)
        with self.lock:
            if self.ser is None:
                return False
            try:
                self.latency.sent(self.seq, time.monotonic())
                self.ser.write(line)
            except (self._serial_mod.SerialException, OSError) as e:
                self.error = str(e)
                return False
            self.seq = (self.seq + 1) % P.SEQ_MOD
        self.log.write("cmd", time.monotonic_ns(), line.decode().strip())
        return True

    def _read_loop(self):
        while self._running:
            ser = self.ser
            if ser is None:
                time.sleep(0.5)
                if self._running:
                    self._open()
                continue
            try:
                raw = ser.readline()
            except (self._serial_mod.SerialException, OSError, TypeError) as e:
                self.error = str(e)
                with self.lock:
                    try:
                        ser.close()
                    except Exception:
                        pass
                    self.ser = None
                continue
            if not raw:
                continue
            t_ns = time.monotonic_ns()
            text = raw.decode("ascii", "replace").strip()
            self.log.write("esp32", t_ns, text)
            d = P.parse_line(text)
            self.lines += 1
            if d is None:
                self.unparsed += 1
            elif d["kind"] == "A":
                with self.lock:
                    rtt = self.latency.acked(d["seq"], t_ns / 1e9)
                if rtt is not None:
                    self.rtts = (self.rtts + [rtt])[-200:]
            else:
                n, gear, dia = self.drivetrain
                rpm = P.driveshaft_rpm(d["now_us"], d["enc_last_edge_us"], d["enc_dt_us"], d["enc_count"], n)
                d["driveshaft_rpm"] = rpm
                d["speed_mps"] = P.wheel_speed_mps(rpm, gear, dia)
                d["mode_name"] = P.MODE_NAMES.get(d["mode"], str(d["mode"]))
                self.status, self.status_t = d, time.monotonic()

    def snapshot(self):
        r = sorted(self.rtts[-100:])
        return {"connected": self.connected, "port": self.port, "error": self.error, "lines": self.lines,
                "unparsed": self.unparsed,
                "status_age_ms": None if self.status_t is None else 1e3 * (time.monotonic() - self.status_t),
                "rtt_ms": 1e3 * self.rtts[-1] if self.rtts else None,
                "rtt_p95_ms": 1e3 * r[int(0.95 * (len(r) - 1))] if r else None}

    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)
        with self.lock:
            if self.ser is not None:
                self.ser.close()
                self.ser = None


# ---------------------------------------------------------------------------------------------
# commands
# ---------------------------------------------------------------------------------------------

def _pass_name(kind, sched, i, cruise=None, amp=None):
    if kind == "longitudinal":
        return "throttle ramp through deadband" if i == 0 else "throttle step %d us" % max(s[1] for s in sched)
    return ("steer doublet +/-%d us @ %d us" % (amp, cruise)) if amp else ("steer sweep 0.3-3 Hz @ %d us" % cruise)


def pass_catalog(limit_us):
    """The designed sysid passes, with what the (prior) model predicts for each."""
    out = []
    for i, sched in enumerate(D.longitudinal_passes()):
        out.append(("longitudinal", _pass_name("longitudinal", sched, i), sched))
    lat = D.lateral_passes()
    k = 0
    for c in (1560, 1580):
        for a in (60, 120, 180):
            out.append(("lateral", _pass_name("lateral", None, 0, c, a), lat[k]))
            k += 1
        out.append(("lateral", _pass_name("lateral", None, 0, c), lat[k]))
        k += 1
    cat = []
    for kind, name, sched in out:
        fp = D.footprint(sched)
        top = max(s[1] for s in sched)
        cat.append({"name": name, "kind": kind, "schedule": sched, "duration_s": fp["duration_s"],
                    "top_speed_mps": fp["top_speed_mps"], "extent_m": [fp["x_extent_m"], fp["y_extent_m"]],
                    "max_throttle_us": top, "clamped": top > limit_us})
    return cat


class Commander:
    """What the 'Jetson' sends, at COMMAND_HZ: off | hold (neutral) | jog (deadman) | pass."""

    def __init__(self, link):
        self.link = link
        self.lock = threading.Lock()
        self.state = "off"
        self.jog = (1500, 1500)
        self.jog_until = 0.0
        self.pass_ = None  # {"name", "t0", "ends", "values", "duration"}
        self.sent = None
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="commander", daemon=True)
        self._thread.start()

    def hold(self):
        with self.lock:
            self.state, self.pass_ = "hold", None

    def release(self):
        with self.lock:
            self.state, self.pass_ = "off", None

    def set_jog(self, speed_us, steer_us):
        with self.lock:
            self.state, self.pass_ = "jog", None
            self.jog = (int(steer_us), int(speed_us))
            self.jog_until = time.monotonic() + JOG_TIMEOUT_S

    def start_pass(self, name, schedule):
        durs = np.array([s[0] for s in schedule], float)
        with self.lock:
            self.state = "pass"
            self.pass_ = {"name": name, "t0": time.monotonic(), "ends": np.cumsum(durs),
                          "values": [(int(round(s[2])), int(round(s[1]))) for s in schedule],
                          "duration": float(durs.sum())}

    def stop_pass(self):
        with self.lock:
            if self.state == "pass":
                self.state, self.pass_ = "hold", None

    def _target(self, now):
        if self.state == "jog" and now > self.jog_until:
            self.state = "hold"
        if self.state == "pass":
            t = now - self.pass_["t0"]
            i = int(np.searchsorted(self.pass_["ends"], t, side="right"))
            if i >= len(self.pass_["values"]):
                self.state, self.pass_ = "hold", None
            else:
                return self.pass_["values"][i]
        if self.state == "jog":
            return self.jog
        if self.state == "hold":
            return (1500, 1500)
        return None

    def _loop(self):
        period = 1.0 / COMMAND_HZ
        nxt = time.monotonic()
        while self._running:
            with self.lock:
                target = self._target(time.monotonic())
            if target is not None:
                self.link.send(*target)
            self.sent = target
            nxt += period
            time.sleep(max(0.0, nxt - time.monotonic()))
            if time.monotonic() - nxt > 0.5:  # fell far behind (machine asleep): don't burst
                nxt = time.monotonic()

    def snapshot(self):
        with self.lock:
            ps = None
            if self.pass_:
                ps = {"name": self.pass_["name"], "t": time.monotonic() - self.pass_["t0"],
                      "duration": self.pass_["duration"]}
            sent = self.sent
        return {"state": self.state, "pass": ps,
                "sent_steer_us": sent[0] if sent else None, "sent_speed_us": sent[1] if sent else None}

    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)


# ---------------------------------------------------------------------------------------------
# the station
# ---------------------------------------------------------------------------------------------

def course_outline(n_max=600):
    from course import parking_lot_course

    c = parking_lot_course()
    step = max(1, len(c.xy) // n_max)
    xy, nrm = c.xy[::step], c.normal[::step]
    half = c.width / 2.0
    return {"center": xy.round(3).tolist(), "left": (xy + half * nrm).round(3).tolist(),
            "right": (xy - half * nrm).round(3).tolist(), "width_m": c.width, "length_m": c.length}


class GroundStation:
    def __init__(self, port, baud, log_dir, emulator=None, notes=""):
        self.emulator = emulator
        self.cfg = F.Config()
        os.makedirs(log_dir, exist_ok=True)
        self.log = SessionLog(log_dir, port, notes=notes)
        self.link = Link(port, baud, self.log)
        self.commander = Commander(self.link)
        self.passes = {p["name"]: p for p in pass_catalog(self.cfg.auto_speed_max_us)}
        self._config = None
        self.t0 = time.monotonic()

    @classmethod
    def simulated(cls, log_dir, **kw):
        import car_emulator

        emu = car_emulator.CarEmulator().start()
        return cls(emu.port, emu.baud, log_dir, emulator=emu, notes="car_emulator", **kw)

    def config(self):
        if self._config is None:
            self._config = {
                "sim": self.emulator is not None,
                "port": self.link.port,
                "course": course_outline(),
                "limits": {"auto_speed_min_us": self.cfg.auto_speed_min_us,
                           "auto_speed_max_us": self.cfg.auto_speed_max_us,
                           "auto_steer_min_us": self.cfg.auto_steer_min_us,
                           "auto_steer_max_us": self.cfg.auto_steer_max_us,
                           "jetson_timeout_ms": self.cfg.jetson_timeout_ms},
                "drivetrain": {"edges_per_rev": P.ENCODER_EDGES_PER_REV, "diff_ratio": P.DIFF_RATIO,
                               "wheel_diameter_m": P.WHEEL_DIAMETER_M, "meters_per_edge": P.meters_per_edge()},
                "passes": [{k: v for k, v in p.items() if k != "schedule"} for p in self.passes.values()],
                "modes": {str(k): v for k, v in P.MODE_NAMES.items()},
                "log_dir": self.log.path,
            }
        return self._config

    def snapshot(self):
        snap = {"t": time.monotonic() - self.t0, "link": self.link.snapshot(), "status": self.link.status,
                "cmd": self.commander.snapshot(), "truth": None, "rc": None}
        if self.emulator is not None:
            snap["truth"] = self.emulator.truth()
            snap["rc"] = self.emulator.rc_state()
        return snap

    def close(self):
        self.commander.close()
        self.link.close()
        if self.emulator is not None:
            self.emulator.stop()
        self.log.close()


# ---------------------------------------------------------------------------------------------
# HTTP
# ---------------------------------------------------------------------------------------------

class ApiError(Exception):
    pass


def _int_in(body, key, lo, hi):
    v = body.get(key)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not lo <= v <= hi:
        raise ApiError("%s must be a number in %d..%d" % (key, lo, hi))
    return int(round(v))


class Handler(BaseHTTPRequestHandler):
    server_version = "ground_station/1"

    def log_message(self, fmt, *args):  # quiet: the page polls a lot
        pass

    @property
    def gs(self):
        return self.server.gs

    def _send(self, code, body, ctype="application/json"):
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            return self._static("index.html")
        if path.startswith("/static/"):
            return self._static(path[len("/static/"):])
        if path == "/api/config":
            return self._send(200, self.gs.config())
        if path == "/api/stream":
            return self._stream()
        self._send(404, {"error": "not found"})

    def _static(self, name):
        files = set(os.listdir(STATIC_DIR))
        if name not in files:  # whitelist: no paths, no traversal
            return self._send(404, {"error": "not found"})
        ctype = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
                 ".css": "text/css; charset=utf-8", ".svg": "image/svg+xml"}.get(os.path.splitext(name)[1],
                                                                                "application/octet-stream")
        with open(os.path.join(STATIC_DIR, name), "rb") as fh:
            self._send(200, fh.read(), ctype)

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        period = 1.0 / SNAPSHOT_HZ
        try:
            while True:
                self.wfile.write(b"data: " + json.dumps(self.gs.snapshot()).encode() + b"\n\n")
                self.wfile.flush()
                time.sleep(period)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        try:
            n = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(n) or b"{}") if n else {}
            if not isinstance(body, dict):
                raise ApiError("body must be a JSON object")
            result = self._post(path, body)
        except (ApiError, ValueError) as e:
            return self._send(400, {"error": str(e)})
        if result is None:
            return self._send(404, {"error": "not found"})
        self._send(200, result)

    def _post(self, path, body):
        gs, cmd = self.gs, self.gs.commander
        if path == "/api/hold":
            cmd.hold()
        elif path == "/api/release":
            cmd.release()
        elif path == "/api/jog":
            cmd.set_jog(_int_in(body, "speed_us", P.PULSE_MIN_US, P.PULSE_MAX_US),
                        _int_in(body, "steer_us", P.PULSE_MIN_US, P.PULSE_MAX_US))
        elif path == "/api/pass/start":
            if "custom" in body:
                c = body["custom"] if isinstance(body["custom"], dict) else {}
                thr = _int_in(c, "throttle_us", 1300, 1700)
                hold_s = c.get("hold_s")
                if not isinstance(hold_s, (int, float)) or not 0.2 <= hold_s <= 10:
                    raise ApiError("hold_s must be 0.2..10 s")
                steer = _int_in(c, "steer_us", P.PULSE_MIN_US, P.PULSE_MAX_US)
                sched = D.holds((1.0, 1500, 1500), (hold_s, thr, steer), (3.0, 1500, 1500))
                cmd.start_pass("custom step %d us for %.1f s" % (thr, hold_s), sched)
            else:
                p = gs.passes.get(body.get("name"))
                if p is None:
                    raise ApiError("unknown pass %r" % body.get("name"))
                cmd.start_pass(p["name"], p["schedule"])
        elif path == "/api/pass/stop":
            cmd.stop_pass()
        elif path == "/api/sim/rc":
            if gs.emulator is None:
                raise ApiError("not in --sim mode")
            kw = {}
            if "on" in body:
                kw["on"] = bool(body["on"])
            for k in ("ch3", "speed_us", "steer_us"):
                if k in body:
                    kw[k] = _int_in(body, k, P.PULSE_MIN_US, P.PULSE_MAX_US)
            gs.emulator.set_rc(**kw)
        elif path == "/api/sim/reset":
            if gs.emulator is None:
                raise ApiError("not in --sim mode")
            gs.emulator.reset_car()
        else:
            return None
        return {"ok": True, "cmd": cmd.snapshot()}


def make_server(gs, host, port):
    srv = ThreadingHTTPServer((host, port), Handler)
    srv.daemon_threads = True
    srv.gs = gs
    return srv


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--sim", action="store_true", help="drive the car emulator")
    src.add_argument("--port", help="ESP32 serial port")
    ap.add_argument("--baud", type=int, default=P.BAUD)
    ap.add_argument("--host", default="127.0.0.1", help="0.0.0.0 = reachable from other machines")
    ap.add_argument("--http-port", type=int, default=8080)
    ap.add_argument("--log-dir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs"))
    ap.add_argument("--notes", default="")
    args = ap.parse_args(argv)
    log_dir = os.path.abspath(args.log_dir)
    gs = (GroundStation.simulated(log_dir) if args.sim
          else GroundStation(args.port, args.baud, log_dir, notes=args.notes))
    srv = make_server(gs, args.host, args.http_port)
    host = "localhost" if args.host in ("127.0.0.1", "0.0.0.0") else args.host
    print("ground station: http://%s:%d   (%s, logging to %s)" % (host, srv.server_address[1],
          "emulator " + gs.link.port if args.sim else gs.link.port, gs.log.path), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
        gs.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
