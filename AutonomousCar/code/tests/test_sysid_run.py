"""sysid pipeline on a real (emulated) session: ground station log -> pieces -> fits -> gate -> plan."""

import json
import os
import subprocess
import sys
import time

import numpy as np
import pytest

CODE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, CODE_DIR)

pytest.importorskip("serial")
import ground_station as G  # noqa: E402
import sysid_design as SD  # noqa: E402
import sysid_model as M  # noqa: E402
import sysid_run as R  # noqa: E402

# the "real car" the emulator plays - not the defaults the fit starts from
CAR = dict(speed_gain=0.05, tau_speed=0.6, deadband_us=30, brake_gain=0.05, roll_decel=0.4, drag_coeff=0.02,
           steer_gain=0.0019, steer_center_us=1506, tau_steer=0.07, v_char=8.0, input_delay=0.04)


@pytest.fixture(scope="module")
def session(tmp_path_factory):
    """~25 s of driving on the emulator: a braking step and two steering doublets, logged."""
    gs = G.GroundStation.simulated(log_dir=str(tmp_path_factory.mktemp("logs")), emulator_params=CAR)
    try:
        emu, cmd = gs.emulator, gs.commander
        emu.set_rc(on=True, ch3=1900)
        time.sleep(0.3)
        cmd.hold()
        time.sleep(0.3)
        emu.set_rc(ch3=1100)  # armed, auto
        p_car = M.default_params(**CAR)
        cands = SD.candidates(p_car, SD.Limits(max_speed_mps=3.0))
        plan = [next(c for c in cands if c.brakes and "2.5s" in c.name and "~1.8" in c.name),
                next(c for c in cands if c.kind == "lateral" and c.cruise_mps > 1.5 and "120" in c.name),
                next(c for c in cands if c.kind == "lateral" and c.cruise_mps < 1.5 and "175" in c.name)]
        for m in plan:
            emu.reset_car()
            cmd.start_pass(m.name, m.schedule)
            time.sleep(m.duration_s + 0.5)
        path = gs.log.path
    finally:
        gs.close()
    return path, M.default_params(**CAR)


def test_extract_pieces_speed_and_yaw(session):
    path, car = session
    data = R.extract([path])
    assert len(data.pieces) == 3
    for pc in data.pieces:
        assert pc.long["t_meas"].size > 50 and pc.long["v_meas"].max() > 1.0
        assert pc.lat is not None and pc.lat["t_meas"].size > 500
        assert pc.long["v_meas"][0] == 0.0  # each piece starts at standstill
    assert [pc.steering for pc in data.pieces] == [False, True, True]
    assert data.inputs == "out"  # the ESP32's actual outputs are the fit input


def test_fit_recovers_the_car(session):
    path, car = session
    rep = R.run([path], limits=SD.Limits(max_speed_mps=3.0), n_suggest=2, holdout=True)
    p = rep["params"]
    for k in ("speed_gain", "tau_speed", "steer_gain"):
        assert p[k] == pytest.approx(car[k], rel=0.15), k
    assert abs(p["steer_center_us"] - car["steer_center_us"]) < 4
    # the fit's delay includes the ESP32's 50 Hz servo frame (0-20 ms, ~10 on average)
    assert 0.03 < p["input_delay"] < 0.07
    assert p["v_char"] == pytest.approx(car["v_char"], rel=0.5)
    gate = {r["param"]: r for r in rep["gate"]}
    assert set(gate) == set(SD.TARGETS)
    assert rep["holdout"]["longitudinal"] is None or rep["holdout"]["longitudinal"]["rmse"] < 0.3
    assert rep["holdout"]["lateral"]["rmse"] < 0.1
    assert rep["suggestions"] and all(s["schedule"] for s in rep["suggestions"])
    # one braking step can't pin the brake / coast-down terms - the plan should notice something
    assert rep["gate_pass"] is False or rep["suggestions"]


def test_cli_writes_json(session, tmp_path):
    path, _ = session
    out = tmp_path / "fit.json"
    r = subprocess.run([sys.executable, os.path.join(CODE_DIR, "sysid_run.py"), path, "--out", str(out),
                        "--max-speed", "3", "--n", "2", "--no-holdout"], capture_output=True, text=True, timeout=600)
    assert r.returncode == 0, r.stderr
    assert "Gate A" in r.stdout and "Suggested next" in r.stdout
    rep = json.loads(out.read_text())
    assert {"params", "std", "gate", "suggestions", "sessions"} <= set(rep)


def test_no_data_still_plans():
    rep = R.run([], limits=SD.Limits(max_speed_mps=2.0), n_suggest=3)
    assert rep["params"] == M.default_params()
    assert len(rep["suggestions"]) >= 2
