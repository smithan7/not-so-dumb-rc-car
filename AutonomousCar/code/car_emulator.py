"""Car + ESP32 emulator: everything downstream of the Jetson, on a virtual serial port.

    python car_emulator.py            # prints the port, e.g. /dev/ttys012; Ctrl-C to stop
    python car_emulator.py --manual   # virtual transmitter on, ch3 in MANUAL

Anything that talks to the real ESP32 (esp32_node, ground_station.py, sysid tools) can be
pointed at the printed port instead and will see what it would see on the car:

- firmware: protocol v2 (esp32_protocol), the same arbitration/failsafe (failsafe.py, a tested
  port of failsafe.h), acks per command, 50 Hz status, outputs latched on a 50 Hz servo frame
- car: the sysid_model dynamics (input delay, ESC deadband/gains, servo lag and rate limit,
  bicycle model with understeer) plus position, stepped in real time
- driveshaft encoder: rising edges from the drivetrain seeds (2.8:1 diff, 100 mm tyre, 2 edges/rev)
- a virtual RC transmitter: power, the ch3 MANUAL/AUTO switch, throttle and steering sticks

The true pose (truth()) is only for display / testing - the real car has no such thing.
"""

import argparse
import collections
import math
import os
import threading
import time
import tty

import esp32_protocol as P
import failsafe as F
import sysid_model as M

WHEELBASE_M = 0.406  # Arrma Infraction published wheelbase (measure the real one)
_U32 = 2 ** 32


class Plant:
    """sysid_model's equations, stepped one dt at a time, with position and a driveshaft encoder.

    Inputs are the pulse widths at the actuators (the input delay is applied by the caller)."""

    def __init__(self, p, wheelbase=WHEELBASE_M, pose=(0.0, 0.0, 0.0), diff_ratio=P.DIFF_RATIO,
                 wheel_diameter_m=P.WHEEL_DIAMETER_M, edges_per_rev=P.ENCODER_EDGES_PER_REV):
        self.p = dict(p)
        self.wheelbase = wheelbase
        self.diff_ratio, self.wheel_diameter_m, self.edges_per_rev = diff_ratio, wheel_diameter_m, edges_per_rev
        self.reset(pose)

    def reset(self, pose=(0.0, 0.0, 0.0)):
        self.x, self.y, self.psi = (float(v) for v in pose)
        self.v = 0.0
        self.delta = 0.0
        self.odometer_m = 0.0
        self.shaft_edges = 0.0  # continuous edge position; an edge at each integer crossing

    @property
    def yaw_rate(self):
        p = self.p
        return self.v * math.tan(self.delta) / self.wheelbase / (1.0 + (self.v / p["v_char"]) ** 2)

    def step(self, dt, throttle_us, steer_us):
        """Advance dt seconds. Returns the times (s from the start of this step) of encoder edges."""
        p = self.p
        # pose, from the state at the start of the step (as sysid_model.simulate does)
        r = self.yaw_rate
        self.psi += r * dt
        self.x += self.v * math.cos(self.psi) * dt
        self.y += self.v * math.sin(self.psi) * dt
        # encoder
        edges = []
        ds = self.v * dt
        if ds > 0.0:
            per_m = self.diff_ratio * self.edges_per_rev / (math.pi * self.wheel_diameter_m)
            e0 = self.shaft_edges
            e1 = e0 + ds * per_m
            k = math.floor(e0) + 1
            while k <= e1:
                edges.append((k - e0) / (e1 - e0) * dt)
                k += 1
            self.shaft_edges = e1
            self.odometer_m += ds
        # speed (simulate_speed)
        u = throttle_us - p["throttle_center_us"]
        u = math.copysign(max(abs(u) - p["deadband_us"], 0.0), u)
        a = 0.0
        if u > 0.0:
            a = (p["speed_gain"] * u - self.v) / p["tau_speed"]
        elif u < 0.0 and self.v > 0.0:
            a = p["brake_gain"] * u
        if self.v > 0.0:
            a -= p["roll_decel"] + p["drag_coeff"] * self.v * self.v
        self.v = max(self.v + a * dt, 0.0)
        # steering servo (simulate_steering)
        cmd = max(-p["delta_max"], min(p["delta_max"], p["steer_gain"] * (steer_us - p["steer_center_us"])))
        rate = (cmd - self.delta) / p["tau_steer"]
        rate = max(-p["steer_rate_max"], min(p["steer_rate_max"], rate))
        step = rate * dt
        if abs(step) > abs(cmd - self.delta):
            step = cmd - self.delta
        self.delta += step
        return edges


class CarEmulator:
    """The ESP32 firmware + car, running in a background thread behind a pty."""

    def __init__(self, params=None, seed=0, micros_start=0, pose=(7.0, 3.5, 0.0), cfg=None, servo_hz=50.0,
                 loop_s=0.001, status_period_s=0.020, wheelbase=WHEELBASE_M):
        self.p = M.default_params(gyro_bias=0.0, **(params or {}))
        self.plant = Plant(self.p, wheelbase=wheelbase, pose=pose)
        self.start_pose = pose
        self.arbiter = F.Arbiter(cfg or F.Config())
        self.cfg = self.arbiter.cfg
        self.servo_period = 1.0 / servo_hz
        self.loop_s = loop_s
        self.status_period = status_period_s
        self.micros_start = int(micros_start)
        self.seed = seed
        self.lock = threading.Lock()
        self.rc = {"on": False, "ch3": 1100, "speed_us": 1500, "steer_us": 1500}
        self._running = False
        self._thread = None
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)   # no echo / line editing: a plain byte pipe like a UART
        os.set_blocking(self.master, False)
        self.port = os.ttyname(self.slave)
        # any baud works on a pty; 115200 avoids pyserial's macOS-only ioctl for 460800, which ptys reject
        self.baud = 115200
        self._reset_firmware()

    # ---- control (thread-safe) ----------------------------------------------------------
    def set_rc(self, on=None, ch3=None, speed_us=None, steer_us=None):
        """The virtual transmitter. on=False = transmitter off (no receiver pulses)."""
        with self.lock:
            for k, v in (("on", on), ("ch3", ch3), ("speed_us", speed_us), ("steer_us", steer_us)):
                if v is not None:
                    self.rc[k] = bool(v) if k == "on" else int(v)

    def rc_state(self):
        with self.lock:
            return dict(self.rc)

    def reset_car(self, pose=None):
        with self.lock:
            self.plant.reset(pose or self.start_pose)
            self._delay_line.clear()

    def truth(self):
        with self.lock:
            pl = self.plant
            return {"x": pl.x, "y": pl.y, "psi": pl.psi, "v": pl.v, "yaw_rate": pl.yaw_rate, "delta": pl.delta,
                    "odometer_m": pl.odometer_m}

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, name="car_emulator", daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass

    # ---- the "firmware" -------------------------------------------------------------------
    def _reset_firmware(self):
        self._t0 = time.monotonic()
        self._rx = b""
        self._jetson = {"seen": False, "last_ms": 0, "seq": 0, "speed_us": 1500, "steer_us": 1500}
        self._rc_last_ms, self._rc_seen = 0, False
        self._enc = {"count": 0, "last_us": 0, "last_ms": 0, "dt_us": 0}
        self._bad_cmds = 0
        self._out = F.Output(F.MODE_FAILSAFE_NO_RC, 1500, 1500, False)
        self._latched = (1500, 1500)
        self._delay_line = collections.deque()  # (t, speed_us, steer_us) as the actuators get them
        self._next_frame = 0.0
        self._next_status = 0.0

    def _micros(self, t):
        return (int((t - self._t0) * 1e6) + self.micros_start) % _U32

    def _millis(self, t):
        return int((t - self._t0) * 1e3) % _U32

    def _write(self, line):
        try:
            os.write(self.master, line.encode("ascii"))
        except (BlockingIOError, OSError):
            pass  # nobody reading: the UART buffer overflows and lines are lost, as on the car

    def _actuators_at(self, t):
        """Pulse widths the ESC/servo act on at time t: latched frames, delayed by input_delay."""
        t_eff = t - self.p["input_delay"]
        dl = self._delay_line
        while len(dl) > 1 and dl[1][0] <= t_eff:
            dl.popleft()
        if dl and dl[0][0] <= t_eff:
            return dl[0][1], dl[0][2]
        return 1500.0, 1500.0

    def _read_commands(self, t):
        acks = []
        try:
            self._rx += os.read(self.master, 4096)
        except (BlockingIOError, OSError):
            pass
        while b"\n" in self._rx:
            line, self._rx = self._rx.split(b"\n", 1)
            if len(line) > 63:
                self._bad_cmds += 1
                continue
            cmd = F.parse_command(line)
            if cmd is None:
                if line:
                    self._bad_cmds += 1
                continue
            seq, steer, speed = cmd
            self._jetson.update(seen=True, last_ms=self._millis(t), seq=seq, speed_us=speed, steer_us=steer)
            acks.append((seq, self._micros(t)))
        if len(self._rx) > 4096:
            self._rx = b""
        return acks

    def _run(self):
        t_last = time.monotonic()
        self._next_frame = t_last
        self._next_status = t_last
        while self._running:
            t = time.monotonic()
            with self.lock:
                # physics in <= 2 ms sub-steps, with the delayed actuator inputs
                dt_total, t_sub = t - t_last, t_last
                n = max(1, int(math.ceil(dt_total / 0.002)))
                for _ in range(n):
                    h = dt_total / n
                    thr, st = self._actuators_at(t_sub)
                    for e in self.plant.step(h, thr, st):
                        te = t_sub + e
                        us = self._micros(te)
                        if self._enc["count"] > 0:
                            self._enc["dt_us"] = (us - self._enc["last_us"]) % _U32
                        self._enc.update(count=(self._enc["count"] + 1) % _U32, last_us=us, last_ms=self._millis(te))
                    t_sub += h
                t_last = t
                rc = dict(self.rc)
            if rc["on"]:
                self._rc_seen, self._rc_last_ms = True, self._millis(t)
            acks = self._read_commands(t)

            j = self._jetson
            out = self.arbiter.update(F.Inputs(
                now_ms=self._millis(t), rc_seen=self._rc_seen, rc_last_ms=self._rc_last_ms, rc_ch3_us=rc["ch3"],
                rc_speed_us=rc["speed_us"], rc_steer_us=rc["steer_us"], jetson_seen=j["seen"],
                jetson_last_ms=j["last_ms"], jetson_speed_us=j["speed_us"], jetson_steer_us=j["steer_us"],
                enc_seen=self._enc["count"] > 0, enc_last_edge_ms=self._enc["last_ms"]))
            self._out = out
            if t >= self._next_frame:  # the servo/ESC see a new pulse once per frame
                self._next_frame += self.servo_period * max(1, math.floor((t - self._next_frame) / self.servo_period) + 1)
                with self.lock:
                    self._delay_line.append((t, float(out.speed_us), float(out.steer_us)))

            for seq, rx_us in acks:
                self._write("A,%d,%d,%d,%d,%d,$\n" % (seq, rx_us, out.mode, out.speed_us, out.steer_us))
            if t >= self._next_status:
                self._next_status += self.status_period * max(1, math.floor((t - self._next_status) / self.status_period) + 1)
                now_ms = self._millis(t)
                rc_age = (now_ms - self._rc_last_ms) % _U32 if self._rc_seen else -1
                j_age = (now_ms - j["last_ms"]) % _U32 if j["seen"] else -1
                rc_live = rc if rc["on"] else {"ch3": rc["ch3"], "speed_us": rc["speed_us"], "steer_us": rc["steer_us"]}
                self._write("S,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,$\n" % (
                    self._micros(t), out.mode, 1 if out.armed else 0, rc_age, j_age, rc_live["ch3"],
                    rc_live["speed_us"], rc_live["steer_us"], j["seq"], j["speed_us"], j["steer_us"], out.speed_us,
                    out.steer_us, self._enc["count"], self._enc["last_us"], self._enc["dt_us"], self._bad_cmds))
            time.sleep(self.loop_s)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--manual", action="store_true", help="transmitter on, ch3 in MANUAL")
    args = ap.parse_args(argv)
    emu = CarEmulator().start()
    if args.manual:
        emu.set_rc(on=True, ch3=1900)
    print(emu.port, flush=True)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        emu.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
