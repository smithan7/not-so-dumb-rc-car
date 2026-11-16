"""Python port of the ESP32 firmware's failsafe.h (arbitration + failsafe), for car_emulator.py.

It mirrors failsafe.h line for line, including uint32 millis() wraparound. Do not change one
without the other: tests/test_failsafe_port.py replays thousands of random input histories through
both and requires identical outputs. See failsafe.h for what the modes and rules mean.
"""

from dataclasses import dataclass

MODE_MANUAL = 0
MODE_AUTO = 1
MODE_FAILSAFE_NO_RC = -1
MODE_FAILSAFE_NO_JETSON = -2
MODE_NOT_ARMED = -3

_U32 = 2 ** 32


@dataclass
class Config:
    neutral_us: int = 1500
    steer_center_us: int = 1500
    ch3_threshold_us: int = 1500
    brake_us: int = 1400
    auto_speed_min_us: int = 1350
    auto_speed_max_us: int = 1600
    auto_steer_min_us: int = 1300
    auto_steer_max_us: int = 1700
    rc_timeout_ms: int = 100
    jetson_timeout_ms: int = 200
    enc_moving_ms: int = 300
    brake_min_ms: int = 1000
    brake_max_ms: int = 3000


@dataclass
class Inputs:
    now_ms: int = 0
    rc_seen: bool = False
    rc_last_ms: int = 0
    rc_ch3_us: int = 0
    rc_speed_us: int = 1500
    rc_steer_us: int = 1500
    jetson_seen: bool = False
    jetson_last_ms: int = 0
    jetson_speed_us: int = 1500
    jetson_steer_us: int = 1500
    enc_seen: bool = False
    enc_last_edge_ms: int = 0


@dataclass
class Output:
    mode: int
    speed_us: int
    steer_us: int
    armed: bool


def clamp_int(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def fresh(seen, last_ms, now_ms, timeout_ms):
    return seen and (now_ms - last_ms) % _U32 < timeout_ms


class Arbiter:
    def __init__(self, cfg):
        self.cfg = cfg
        self.armed = False
        self.in_failsafe = False
        self.brake_allowed = False
        self.failsafe_start_ms = 0
        self.last_speed_us = 1500

    def update(self, i):
        c = self.cfg
        rc_ok = fresh(i.rc_seen, i.rc_last_ms, i.now_ms, c.rc_timeout_ms)
        jetson_ok = fresh(i.jetson_seen, i.jetson_last_ms, i.now_ms, c.jetson_timeout_ms)
        moving = fresh(i.enc_seen, i.enc_last_edge_ms, i.now_ms, c.enc_moving_ms)

        if not rc_ok:
            mode = MODE_FAILSAFE_NO_RC
        elif i.rc_ch3_us > c.ch3_threshold_us:
            mode = MODE_MANUAL
        elif not jetson_ok:
            mode = MODE_FAILSAFE_NO_JETSON
        elif not self.armed:
            mode = MODE_NOT_ARMED
        else:
            mode = MODE_AUTO

        if mode == MODE_MANUAL:
            self.armed = True
            self.in_failsafe = False
            speed, steer = i.rc_speed_us, i.rc_steer_us
        elif mode == MODE_AUTO:
            self.in_failsafe = False
            speed = clamp_int(i.jetson_speed_us, c.auto_speed_min_us, c.auto_speed_max_us)
            steer = clamp_int(i.jetson_steer_us, c.auto_steer_min_us, c.auto_steer_max_us)
        else:
            self.armed = False
            if not self.in_failsafe:
                self.in_failsafe = True
                self.failsafe_start_ms = i.now_ms
                self.brake_allowed = moving and self.last_speed_us >= c.neutral_us
            elapsed = (i.now_ms - self.failsafe_start_ms) % _U32
            if self.brake_allowed and elapsed < c.brake_max_ms and (elapsed < c.brake_min_ms or moving):
                speed = c.brake_us
            else:
                self.brake_allowed = False
                speed = c.neutral_us
            steer = c.steer_center_us
        self.last_speed_us = speed
        return Output(mode, speed, steer, self.armed)


def parse_command(line):
    """'C,<seq>,<steer>,<speed>,<cs>' -> (seq, steer_us, speed_us) or None, with failsafe.h's rules."""
    if isinstance(line, bytes):
        line = line.decode("ascii", "replace")
    if not line.startswith("C,"):
        return None
    f = line[2:].rstrip("\r\n").split(",")
    if len(f) != 4 or not all(x.isdigit() and len(x) <= 9 for x in f):
        return None
    seq, steer, speed, cs = (int(x) for x in f)
    if not (900 <= steer <= 2100 and 900 <= speed <= 2100):
        return None
    if (seq + steer + speed) % 1000 != cs:
        return None
    return seq, steer, speed
