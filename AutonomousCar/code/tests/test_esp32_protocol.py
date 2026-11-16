"""Tests for esp32_protocol (no ROS needed):  python3 -m pytest tests/test_esp32_protocol.py"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import esp32_protocol as P  # noqa: E402

STATUS = "S,123456789,1,1,6,12,1100,1580,1520,42,1580,1520,1580,1520,977,123400000,25000,3,$"


def test_format_command_matches_firmware_checksum():
    assert P.format_command(7, 1520, 1580) == b"C,7,1520,1580,107\n"


def test_format_command_rejects_out_of_range():
    with pytest.raises(ValueError):
        P.format_command(1, 800, 1500)
    with pytest.raises(ValueError):
        P.format_command(1, 1500, 2200)


def test_format_command_rounds_floats_and_wraps_seq():
    assert P.format_command(P.SEQ_MOD + 3, 1520.4, 1579.6) == b"C,3,1520,1580,103\n"


def test_parse_status():
    s = P.parse_line(STATUS)
    assert s["kind"] == "S"
    assert s["now_us"] == 123456789
    assert s["mode"] == P.MODE_AUTO and s["armed"] == 1
    assert s["rc_age_ms"] == 6 and s["jetson_age_ms"] == 12
    assert (s["rc_ch3_us"], s["rc_speed_us"], s["rc_steer_us"]) == (1100, 1580, 1520)
    assert (s["jetson_seq"], s["jetson_speed_us"], s["jetson_steer_us"]) == (42, 1580, 1520)
    assert (s["out_speed_us"], s["out_steer_us"]) == (1580, 1520)
    assert (s["enc_count"], s["enc_last_edge_us"], s["enc_dt_us"], s["bad_cmds"]) == (977, 123400000, 25000, 3)


def test_parse_status_never_seen_ages_are_minus_one():
    s = P.parse_line("S,1000,-1,0,-1,-1,0,1500,1500,0,1500,1500,1500,1500,0,0,0,0,$")
    assert s["mode"] == P.MODE_FAILSAFE_NO_RC and s["rc_age_ms"] == -1


def test_parse_ack():
    a = P.parse_line("A,42,123450000,1,1580,1520,$\r\n")
    assert a == {"kind": "A", "seq": 42, "rx_us": 123450000, "mode": 1, "out_speed_us": 1580, "out_steer_us": 1520}


@pytest.mark.parametrize("line", [
    "", "garbage", "S,1,2,$", STATUS.replace(",$", ""), STATUS.replace("977", "9x7"),
    "A,42,123,1,1580,$",                     # short
    "C,7,1520,1580,107",                     # old v1 status lines also start with C: not ours
    STATUS[:-2] + ",5,$",                    # extra field
])
def test_parse_rejects_malformed(line):
    assert P.parse_line(line) is None


def test_parse_accepts_bytes():
    assert P.parse_line(STATUS.encode() + b"\r\n")["kind"] == "S"


def test_driveshaft_rpm_from_edge_interval():
    # one edge per driveshaft rev; 25 ms between edges = 2400 rpm, last edge just now
    assert P.driveshaft_rpm(now_us=1000000, last_edge_us=1000000, dt_us=25000, count=10) == pytest.approx(2400.0)


def test_driveshaft_rpm_decays_while_no_new_edge():
    # 100 ms since the last edge: shaft can be at most 600 rpm, even though dt says 2400
    assert P.driveshaft_rpm(now_us=1100000, last_edge_us=1000000, dt_us=25000, count=10) == pytest.approx(600.0)


def test_driveshaft_rpm_zero_when_stopped_or_unknown():
    assert P.driveshaft_rpm(now_us=3000000, last_edge_us=1000000, dt_us=25000, count=10) == 0.0  # > 1 s
    assert P.driveshaft_rpm(now_us=1000, last_edge_us=900, dt_us=0, count=1) == 0.0              # one edge only
    assert P.driveshaft_rpm(now_us=1000, last_edge_us=0, dt_us=0, count=0) == 0.0


def test_driveshaft_rpm_with_several_edges_per_rev():
    # optical disc with 2 reflective segments: 2 rising edges per driveshaft revolution
    assert P.driveshaft_rpm(now_us=1000000, last_edge_us=1000000, dt_us=25000, count=10,
                            edges_per_rev=2) == pytest.approx(1200.0)
    with pytest.raises(ValueError):
        P.driveshaft_rpm(1000000, 1000000, 25000, 10, edges_per_rev=0)


def test_rpm_from_counts_averages_over_uneven_segments():
    # 8 edges in 50 ms with 2 edges/rev = 4 revs / 0.05 s = 4800 rpm, whatever the segment spacing
    assert P.rpm_from_counts(100, 1000000, 108, 1050000, edges_per_rev=2) == pytest.approx(4800.0)
    assert P.rpm_from_counts(100, 1000000, 100, 1050000, edges_per_rev=2) == 0.0
    assert P.rpm_from_counts(2 ** 32 - 2, 2 ** 32 - 25000, 2, 25000, edges_per_rev=1) == pytest.approx(4 / 0.05 * 60)
    assert P.rpm_from_counts(100, 1000000, 108, 1000000, edges_per_rev=2) == 0.0  # no time elapsed


def test_seed_drivetrain_constants():
    # Infraction 6S BLX: 2.8:1 diff ("internal gear ratio"), 100 mm tyres, disc 2 blocked + 2 open
    assert P.DIFF_RATIO == 2.8
    assert P.WHEEL_DIAMETER_M == 0.100
    assert P.ENCODER_EDGES_PER_REV == 2
    assert P.meters_per_edge() == pytest.approx(math.pi * 0.100 / (2.8 * 2))  # ~5.6 cm


def test_wheel_speed_from_driveshaft_rpm():
    # 2800 driveshaft rpm = 1000 wheel rpm = 1000 * pi * 0.1 m / 60 s
    assert P.wheel_speed_mps(2800.0) == pytest.approx(1000 * math.pi * 0.1 / 60)
    assert P.wheel_speed_mps(2800.0, diff_ratio=2.0, wheel_diameter_m=0.2) == pytest.approx(1400 * math.pi * 0.2 / 60)
    assert P.wheel_speed_mps(0.0) == 0.0


def test_calibrate_wheel_diameter_from_a_measured_roll():
    # roll a known distance and count edges; the tooth-count ratio is exact, the effective
    # rolling diameter is what drifts (wear, load, tyre growth)
    d = P.calibrate_wheel_diameter(edges=200, distance_m=10.0)
    assert d == pytest.approx(10.0 / (200 / 2 / 2.8) / math.pi)  # 200 edges = 100 shaft revs = 35.7 wheel revs
    assert P.meters_per_edge(wheel_diameter_m=d) * 200 == pytest.approx(10.0)
    with pytest.raises(ValueError):
        P.calibrate_wheel_diameter(edges=0, distance_m=10.0)


def test_driveshaft_rpm_micros_wraparound():
    last = 2 ** 32 - 10000
    assert P.driveshaft_rpm(now_us=15000, last_edge_us=last, dt_us=25000, count=10) == pytest.approx(2400.0)


def test_latency_tracker_round_trip():
    lt = P.LatencyTracker()
    lt.sent(5, 100.000)
    lt.sent(6, 100.020)
    assert lt.acked(6, 100.026) == pytest.approx(0.006)
    assert lt.acked(5, 100.030) == pytest.approx(0.030)
    assert lt.acked(5, 100.040) is None  # duplicate / unknown


def test_latency_tracker_is_bounded():
    lt = P.LatencyTracker(max_pending=10)
    for i in range(100):
        lt.sent(i, float(i))
    assert len(lt.pending) <= 10
    assert lt.acked(99, 99.5) == pytest.approx(0.5)
