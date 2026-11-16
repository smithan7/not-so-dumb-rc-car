"""Serial protocol v2 between the Jetson and the ESP32 autopilot interface firmware.

Pure Python, no ROS, Python 2 and 3. Canonical copy: smithan7/not-so-dumb-rc-car
AutonomousCar/code/esp32_protocol.py, next to the firmware (esp32_autopilot_interface/).
ROS_nsd_rc_car esp32_interface/scripts/esp32_protocol.py is a copy - keep them identical.

  Jetson -> ESP32   C,<seq>,<steer_us>,<speed_us>,<(seq+steer+speed)%1000>\\n
  ESP32 -> Jetson   A,<seq>,<rx_us>,<mode>,<out_speed_us>,<out_steer_us>,$        (per command)
                    S,<now_us>,<mode>,<armed>,...,$                               (50 Hz status)
"""

from __future__ import division

import collections
import math

BAUD = 460800
SEQ_MOD = 1000000
PULSE_MIN_US, PULSE_MAX_US = 900, 2100
MICROS_WRAP = 2 ** 32

# ---- drivetrain seed values: Arrma Infraction 6S BLX (published specs) -------------------
# Encoder disc is on the rear driveshaft: after the spur/center diff, before the rear diff.
# Refine over time: the tooth-count ratio is exact; the effective rolling diameter drifts
# (wear, load, tyre growth at speed) - recalibrate it with calibrate_wheel_diameter().
DIFF_RATIO = 2.8           # driveshaft revs per wheel rev (ARRMA "internal gear ratio" 2.8:1;
                           # final drive 8.58:1 with the stock 15T pinion / 46T spur)
WHEEL_DIAMETER_M = 0.100   # stock tyre, 3.94 in
ENCODER_EDGES_PER_REV = 2  # disc: 4 quarters, 2 blocked + 2 open -> 2 rising edges per rev

MODE_MANUAL = 0
MODE_AUTO = 1
MODE_FAILSAFE_NO_RC = -1
MODE_FAILSAFE_NO_JETSON = -2
MODE_NOT_ARMED = -3
MODE_NAMES = {MODE_MANUAL: "manual", MODE_AUTO: "auto", MODE_FAILSAFE_NO_RC: "failsafe: no receiver",
              MODE_FAILSAFE_NO_JETSON: "failsafe: no jetson", MODE_NOT_ARMED: "auto not armed"}

STATUS_FIELDS = ("now_us", "mode", "armed", "rc_age_ms", "jetson_age_ms", "rc_ch3_us", "rc_speed_us",
                 "rc_steer_us", "jetson_seq", "jetson_speed_us", "jetson_steer_us", "out_speed_us",
                 "out_steer_us", "enc_count", "enc_last_edge_us", "enc_dt_us", "bad_cmds")
ACK_FIELDS = ("seq", "rx_us", "mode", "out_speed_us", "out_steer_us")
_LAYOUT = {"S": STATUS_FIELDS, "A": ACK_FIELDS}


def format_command(seq, steer_us, speed_us):
    """Command line (bytes) for the firmware. Values are rounded to whole microseconds."""
    seq = int(seq) % SEQ_MOD
    steer, speed = int(round(steer_us)), int(round(speed_us))
    for name, v in (("steer", steer), ("speed", speed)):
        if not PULSE_MIN_US <= v <= PULSE_MAX_US:
            raise ValueError("%s %d us outside %d..%d" % (name, v, PULSE_MIN_US, PULSE_MAX_US))
    return ("C,%d,%d,%d,%d\n" % (seq, steer, speed, (seq + steer + speed) % 1000)).encode("ascii")


def parse_line(line):
    """One line from the ESP32 -> dict with 'kind' ('S' or 'A') and integer fields, or None."""
    if isinstance(line, bytes):
        try:
            line = line.decode("ascii")
        except UnicodeDecodeError:
            return None
    f = line.strip().split(",")
    layout = _LAYOUT.get(f[0])
    if layout is None or len(f) != len(layout) + 2 or f[-1] != "$":
        return None
    try:
        values = [int(x) for x in f[1:-1]]
    except ValueError:
        return None
    out = dict(zip(layout, values))
    out["kind"] = f[0]
    return out


def driveshaft_rpm(now_us, last_edge_us, dt_us, count, edges_per_rev=1, stopped_after_us=1000000):
    """Driveshaft rpm from one status line, from the latest edge interval.

    The encoder is a reflective optical sensor on a segmented disc; the firmware counts rising
    edges, so edges_per_rev = number of reflective segments (measure it: turn the shaft once by
    hand and watch enc_count). dt between consecutive rising edges spans one full light+dark
    period, so it does not depend on the segments' light/dark width ratio - but if the
    segments are unevenly spaced, use rpm_from_counts over a window instead.

    Between edges the shaft can't be turning faster than one edge per (time since the last
    edge), so the estimate decays smoothly to 0 when the car stops instead of holding the last
    value (or flickering to 0 between edges at low speed)."""
    if edges_per_rev < 1:
        raise ValueError("edges_per_rev must be >= 1")
    if count < 2 or dt_us <= 0:
        return 0.0
    since = (now_us - last_edge_us) % MICROS_WRAP
    if since > stopped_after_us:
        return 0.0
    return 60e6 / (max(dt_us, since) * edges_per_rev)


def rpm_from_counts(count0, t0_us, count1, t1_us, edges_per_rev=1):
    """Average driveshaft rpm between two status lines (edge count + ESP32 time, wrap-safe).
    Immune to uneven disc segments; resolution is one edge per window."""
    if edges_per_rev < 1:
        raise ValueError("edges_per_rev must be >= 1")
    dt = (t1_us - t0_us) % MICROS_WRAP
    if dt == 0:
        return 0.0
    edges = (count1 - count0) % MICROS_WRAP
    return edges / float(edges_per_rev) / (dt * 1e-6) * 60.0


class LatencyTracker(object):
    """Command round trip: Jetson send time -> ESP32 ack received (seconds)."""

    def __init__(self, max_pending=200):
        self.max_pending = max_pending
        self.pending = collections.OrderedDict()

    def sent(self, seq, t_sent):
        self.pending[seq % SEQ_MOD] = t_sent
        while len(self.pending) > self.max_pending:
            self.pending.popitem(last=False)

    def acked(self, seq, t_received):
        t_sent = self.pending.pop(seq % SEQ_MOD, None)
        return None if t_sent is None else t_received - t_sent


def wheel_speed_mps(driveshaft_rpm_value, diff_ratio=DIFF_RATIO, wheel_diameter_m=WHEEL_DIAMETER_M):
    """Ground speed (m/s) from rear driveshaft rpm, assuming no wheel slip. The rear diff
    averages the two rear wheels, and wheelspin reads high."""
    return driveshaft_rpm_value / 60.0 / diff_ratio * math.pi * wheel_diameter_m


def meters_per_edge(edges_per_rev=ENCODER_EDGES_PER_REV, diff_ratio=DIFF_RATIO, wheel_diameter_m=WHEEL_DIAMETER_M):
    """Distance travelled per encoder rising edge (~5.6 cm with the seed values)."""
    return math.pi * wheel_diameter_m / (diff_ratio * edges_per_rev)


def calibrate_wheel_diameter(edges, distance_m, edges_per_rev=ENCODER_EDGES_PER_REV, diff_ratio=DIFF_RATIO):
    """Effective rolling diameter from a measured roll: push (or drive slowly, no wheelspin) the
    car a taped-out distance in a straight line and count encoder edges (enc_count difference)."""
    if edges <= 0 or distance_m <= 0:
        raise ValueError("need a positive edge count and distance")
    wheel_revs = edges / float(edges_per_rev) / diff_ratio
    return distance_m / wheel_revs / math.pi
