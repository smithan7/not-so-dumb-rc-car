"""The emulator must behave like the car: same dynamics as sysid_model, same firmware behaviour."""

import math
import os
import sys
import time

import numpy as np
import pytest

CODE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, CODE_DIR)

import car_emulator as E  # noqa: E402
import esp32_protocol as P  # noqa: E402
import sysid_maneuvers as D  # noqa: E402
import sysid_model as M  # noqa: E402

serial = pytest.importorskip("serial")


# ---- plant -------------------------------------------------------------------------------

def test_plant_matches_sysid_model():
    """Stepping the plant with the actuator inputs reproduces sysid_model.simulate."""
    p = M.default_params(input_delay=0.0, v_char=9.0, steer_gain=0.0021)
    sched = D.holds((1, 1500, 1500), (3, 1590, 1500), (2, 1590, 1620), (1, 1590, 1380), (3, 1400, 1500))
    t_cmd, thr, st = D.schedule_signal(sched)
    dt = 0.002
    grid = np.arange(0, t_cmd[-1], dt)
    ref = M.simulate(p, grid, M.input_signal(t_cmd, thr), M.input_signal(t_cmd, st), wheelbase=E.WHEELBASE_M)
    plant = E.Plant(p, wheelbase=E.WHEELBASE_M)
    thr_f, st_f = M.input_signal(t_cmd, thr), M.input_signal(t_cmd, st)
    v, r, x, y = [], [], [], []
    for t in grid:
        v.append(plant.v), r.append(plant.yaw_rate), x.append(plant.x), y.append(plant.y)
        plant.step(dt, float(thr_f(t)), float(st_f(t)))
    assert np.max(np.abs(np.array(v) - ref["v"])) < 0.02
    assert np.max(np.abs(np.array(r) - ref["r"])) < 0.02
    assert np.hypot(x[-1] - ref["x"][-1], y[-1] - ref["y"][-1]) < 0.3  # after ~30 m


def test_plant_encoder_counts_distance():
    p = M.default_params(input_delay=0.0)
    plant = E.Plant(p, wheelbase=E.WHEELBASE_M)
    plant.v = 3.0
    edges = 0
    for _ in range(5000):  # 10 s at 2 ms, rolling with no throttle -> slows down
        edges += len(plant.step(0.002, 1500.0, 1500.0))
    assert edges == pytest.approx(plant.odometer_m / P.meters_per_edge(), abs=1.0)
    assert plant.odometer_m > 5.0


# ---- firmware over the virtual serial port -------------------------------------------------

@pytest.fixture
def emu():
    e = E.CarEmulator(seed=1)
    e.start()
    port = serial.Serial(e.port, e.baud, timeout=0.5)
    yield e, port
    port.close()
    e.stop()


def read_lines(port, seconds):
    out, t_end = [], time.time() + seconds
    while time.time() < t_end:
        raw = port.readline()
        if raw:
            d = P.parse_line(raw)
            assert d is not None, raw
            out.append(d)
    return out


def wait_for(port, pred, seconds=1.5):
    t_end = time.time() + seconds
    while time.time() < t_end:
        d = P.parse_line(port.readline())
        if d is not None and pred(d):
            return d
    return None


def test_status_stream_at_50_hz_and_failsafe_without_receiver(emu):
    e, port = emu
    e.set_rc(on=False)
    lines = read_lines(port, 1.0)
    status = [d for d in lines if d["kind"] == "S"]
    assert 40 <= len(status) <= 60
    assert all(d["mode"] == P.MODE_FAILSAFE_NO_RC and d["out_speed_us"] == 1500 for d in status)
    t = [d["now_us"] for d in status]
    assert all(b > a for a, b in zip(t, t[1:]))  # micros() increasing


def test_command_is_acked_with_its_seq(emu):
    e, port = emu
    t0 = time.time()
    port.write(P.format_command(41, 1520, 1550))
    ack = wait_for(port, lambda d: d["kind"] == "A")
    assert ack is not None and ack["seq"] == 41
    assert time.time() - t0 < 0.1


def test_bad_command_is_counted(emu):
    e, port = emu
    port.write(b"C,1,1500,1500,999\n")
    assert wait_for(port, lambda d: d["kind"] == "S" and d["bad_cmds"] == 1) is not None


def test_arming_then_auto_then_jetson_timeout(emu):
    e, port = emu
    e.set_rc(on=True, ch3=1100)  # switch in AUTO at power-up

    def send(speed, steer=1500, n=1):
        for _ in range(n):
            port.write(P.format_command(send.seq, steer, speed))
            send.seq += 1
            time.sleep(0.02)
    send.seq = 0

    send(1500, n=5)
    assert wait_for(port, lambda d: d["kind"] == "S" and d["mode"] == P.MODE_NOT_ARMED) is not None
    e.set_rc(ch3=1900)  # operator: MANUAL
    assert wait_for(port, lambda d: d["kind"] == "S" and d["mode"] == P.MODE_MANUAL) is not None
    e.set_rc(ch3=1100)  # ... and back: armed
    send(1580, n=40)    # 0.8 s of throttle
    s = wait_for(port, lambda d: d["kind"] == "S" and d["mode"] == P.MODE_AUTO and d["out_speed_us"] == 1580)
    assert s is not None and s["armed"] == 1
    assert e.truth()["v"] > 0.5
    count_moving = s["enc_count"]
    # Jetson goes silent -> failsafe, brake while moving
    fs = wait_for(port, lambda d: d["kind"] == "S" and d["mode"] == P.MODE_FAILSAFE_NO_JETSON, seconds=1.0)
    assert fs is not None and fs["out_speed_us"] == 1400 and fs["enc_count"] >= count_moving


def test_clamps_jetson_throttle_to_stage_limit(emu):
    e, port = emu
    e.set_rc(on=True, ch3=1900)
    time.sleep(0.1)
    e.set_rc(ch3=1100)
    for i in range(10):
        port.write(P.format_command(i, 1500, 1900))
        time.sleep(0.02)
    assert wait_for(port, lambda d: d["kind"] == "S" and d["mode"] == P.MODE_AUTO and d["out_speed_us"] == 1600)


def test_manual_driving_moves_the_car_and_encoder(emu):
    e, port = emu
    e.set_rc(on=True, ch3=1900, speed_us=1580, steer_us=1600)
    time.sleep(1.5)
    port.reset_input_buffer()  # drop the status lines buffered while we slept
    s = wait_for(port, lambda d: d["kind"] == "S")
    tr = e.truth()
    assert tr["v"] > 1.0 and abs(tr["yaw_rate"]) > 0.1
    assert s["enc_count"] > 5 and s["enc_dt_us"] > 0
    rpm = P.driveshaft_rpm(s["now_us"], s["enc_last_edge_us"], s["enc_dt_us"], s["enc_count"], 2)
    assert P.wheel_speed_mps(rpm) == pytest.approx(tr["v"], rel=0.3)


def test_micros_wraps_like_the_esp32():
    e = E.CarEmulator(seed=2, micros_start=2 ** 32 - 300000)  # wraps 0.3 s in
    e.start()
    try:
        port = serial.Serial(e.port, e.baud, timeout=0.5)
        t = [d["now_us"] for d in read_lines(port, 0.8) if d["kind"] == "S"]
        port.close()
    finally:
        e.stop()
    assert max(t) < 2 ** 32 and min(t) < 1000000 < max(t)  # saw both sides of the wrap
