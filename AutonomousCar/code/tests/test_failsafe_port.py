"""failsafe.py (Python, used by the emulator) must behave exactly like the firmware's failsafe.h.

Differential test: random but realistic input histories (receiver dropping in and out, the ch3
switch flipping, the Jetson stalling, the car moving and stopping, millis() wrapping) are fed to
both, step by step, and every output must match.
"""

import os
import shutil
import subprocess
import sys

import numpy as np
import pytest

CODE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, CODE_DIR)
sys.path.insert(0, os.path.join(CODE_DIR, "tests"))

import failsafe as F  # noqa: E402
from test_esp32_failsafe import FW_DIR, _compile  # noqa: E402


def scenario(rng, n=400, t0=0):
    """Event-driven input history -> list of input tuples (as the replay harness reads them)."""
    rows = []
    t = t0
    rc_on, ch3, rc_seen, rc_last = rng.random() < 0.8, 1100, False, 0
    j_on, j_seen, j_last = rng.random() < 0.8, False, 0
    moving, enc_seen, enc_last = False, False, 0
    speed_rc, steer_rc, speed_j, steer_j = 1500, 1500, 1500, 1500
    for _ in range(n):
        t = (t + int(rng.choice([1, 5, 10, 20, 50, 150, 400]))) % 2 ** 32
        if rng.random() < 0.05:
            rc_on = not rc_on
        if rng.random() < 0.06:
            ch3 = 1900 if ch3 < 1500 else 1100
        if rng.random() < 0.05:
            j_on = not j_on
        if rng.random() < 0.05:
            moving = not moving
        if rng.random() < 0.2:
            speed_rc = int(rng.choice([1300, 1450, 1500, 1550, 1700, 1900]))
            steer_rc = int(rng.integers(1100, 1900))
        if rng.random() < 0.2:
            speed_j = int(rng.choice([1000, 1400, 1500, 1580, 1650, 2000]))
            steer_j = int(rng.integers(1000, 2000))
        if rc_on:
            rc_seen, rc_last = True, t
        if j_on:
            j_seen, j_last = True, t
        if moving:
            enc_seen, enc_last = True, t
        rows.append((t, int(rc_seen), rc_last, ch3, speed_rc, steer_rc, int(j_seen), j_last, speed_j, steer_j,
                     int(enc_seen), enc_last))
    return rows


def run_python(rows):
    arb = F.Arbiter(F.Config())
    out = []
    for r in rows:
        o = arb.update(F.Inputs(now_ms=r[0], rc_seen=bool(r[1]), rc_last_ms=r[2], rc_ch3_us=r[3], rc_speed_us=r[4],
                                rc_steer_us=r[5], jetson_seen=bool(r[6]), jetson_last_ms=r[7], jetson_speed_us=r[8],
                                jetson_steer_us=r[9], enc_seen=bool(r[10]), enc_last_edge_ms=r[11]))
        out.append((o.mode, o.speed_us, o.steer_us, int(o.armed)))
    return out


@pytest.fixture(scope="module")
def replay_exe(tmp_path_factory):
    if shutil.which("c++") is None:
        pytest.skip("no C++ compiler")
    exe = str(tmp_path_factory.mktemp("fw") / "arbiter_replay")
    build = _compile(os.path.join(FW_DIR, "test", "arbiter_replay.cpp"), exe)
    assert build.returncode == 0, build.stderr
    return exe


@pytest.mark.parametrize("seed", range(40))
def test_python_port_matches_firmware(replay_exe, seed):
    rng = np.random.default_rng(seed)
    rows = scenario(rng, t0=0 if seed % 4 else 2 ** 32 - 3000)  # every 4th run crosses the millis() wrap
    stdin = "\n".join(" ".join(map(str, r)) for r in rows) + "\n"
    cpp = subprocess.run([replay_exe], input=stdin, capture_output=True, text=True, check=True).stdout.split("\n")
    cpp = [tuple(map(int, line.split())) for line in cpp if line]
    py = run_python(rows)
    assert len(cpp) == len(rows)
    for i, (a, b) in enumerate(zip(cpp, py)):
        assert a == b, "step %d: input %s -> firmware %s, python %s" % (i, rows[i], a, b)
    # the scenarios must actually exercise every mode, or the test proves little
    modes = {m for m, *_ in cpp}
    if seed == 0:
        assert modes >= {F.MODE_MANUAL, F.MODE_FAILSAFE_NO_RC}, modes


def test_scenarios_cover_all_modes_and_braking(replay_exe):
    seen_modes, braked = set(), False
    for seed in range(40):
        for m, speed, _, _ in run_python(scenario(np.random.default_rng(seed))):
            seen_modes.add(m)
            braked |= m < 0 and speed == F.Config().brake_us
    assert seen_modes == {F.MODE_MANUAL, F.MODE_AUTO, F.MODE_FAILSAFE_NO_RC, F.MODE_FAILSAFE_NO_JETSON,
                          F.MODE_NOT_ARMED}
    assert braked


def test_parse_command_matches_firmware_rules():
    assert F.parse_command("C,7,1520,1580,107\r") == (7, 1520, 1580)
    for bad in ["C,7,1520,1580,108", "C,7,1520,1580", "C,7,800,1580,387", "C,,1520,1580,100",
                "C,7,15x0,1580,107", "C,7,1520,1580,107,9", "C,1234567890,1500,1500,0"]:
        assert F.parse_command(bad) is None, bad
