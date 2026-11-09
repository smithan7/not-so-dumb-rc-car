import math
import os
import sys

import numpy as np
import pytest

CODE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, CODE_DIR)

import sysid_maneuvers as D  # noqa: E402
import sysid_model as M  # noqa: E402

TRUE = M.default_params(steer_center_us=1488, steer_gain=math.radians(0.085), tau_steer=0.12, v_char=9.0,
                        gyro_bias=0.012, input_delay=0.045, deadband_us=32, speed_gain=0.11, tau_speed=1.1,
                        brake_gain=0.06, roll_decel=0.45, drag_coeff=0.02)


def const(u):
    return lambda t: np.full_like(np.asarray(t, float), u)


def test_steady_state_speed_and_deadband():
    p = M.default_params(roll_decel=0.0, drag_coeff=0.0, input_delay=0.0)
    t = np.arange(0, 10, 0.01)
    v = M.simulate_speed(p, t, const(1600))
    assert v[-1] == pytest.approx(p["speed_gain"] * (100 - p["deadband_us"]), rel=1e-3)
    assert np.all(M.simulate_speed(p, t, const(1500 + p["deadband_us"] - 1)) == 0.0)


def test_brake_and_coast_stop_at_zero():
    p = M.default_params(input_delay=0.0)
    t = np.arange(0, 5, 0.01)
    v = M.simulate_speed(p, t, const(1400), v0=5.0)
    assert v.min() == 0.0 and v[-1] == 0.0
    coast = M.simulate_speed(p, t, const(1500), v0=5.0)
    assert 0.0 < coast[100] < 5.0  # drag + rolling resistance only


def test_steering_saturation_lag_and_rate_limit():
    p = M.default_params(input_delay=0.0, tau_steer=0.1, steer_rate_max=100.0)
    t = np.arange(0, 2, 0.001)
    d = M.simulate_steering(p, t, lambda tt: np.where(tt < 0.5, 1500.0, 2000.0))
    assert d[-1] == pytest.approx(p["delta_max"])  # 500 us would be 50 deg: clipped
    i63 = np.searchsorted(d, (1 - math.exp(-1)) * p["delta_max"])
    assert t[i63] - 0.5 == pytest.approx(0.1, abs=0.005)
    slow = M.simulate_steering(dict(p, steer_rate_max=0.5), t, lambda tt: np.where(tt < 0.5, 1500.0, 2000.0))
    assert np.max(np.diff(slow)) / 0.001 == pytest.approx(0.5, rel=1e-6)


def test_yaw_rate_kinematic_limit():
    p = M.default_params(v_char=1e9, gyro_bias=0.0)
    assert M.yaw_rate(p, 3.0, 0.2, wheelbase=0.5) == pytest.approx(3.0 * math.tan(0.2) / 0.5)


def test_input_signal_failsafe_neutral():
    f = M.input_signal([0, 1, 2], [1600, np.nan, 1600], neutral=1500)
    assert f(1.0) == 1500.0


def test_designed_passes_fit_test_area():
    for passes in (D.longitudinal_passes(), D.lateral_passes()):
        assert all(ok for _, _, ok in D.check_fits_area(passes))


def test_longitudinal_fit_recovers_truth():
    logs = [D.synthesize(s, TRUE, "encoder", seed=i) for i, s in enumerate(D.longitudinal_passes())]
    f = M.fit_longitudinal([D.longitudinal_segment(l) for l in logs])
    assert f.success
    for k in M.LONGITUDINAL_FREE:
        assert f.params[k] == pytest.approx(TRUE[k], rel=0.05), k


def test_lateral_fit_recovers_truth():
    logs = [D.synthesize(s, TRUE, "encoder", seed=50 + i) for i, s in enumerate(D.lateral_passes())]
    f = M.fit_lateral([D.lateral_segment(l) for l in logs])
    assert f.success
    for k in ("steer_gain", "tau_steer", "v_char"):
        assert f.params[k] == pytest.approx(TRUE[k], rel=0.06), k
    assert f.params["steer_center_us"] == pytest.approx(TRUE["steer_center_us"], abs=0.5)
    assert f.params["gyro_bias"] == pytest.approx(TRUE["gyro_bias"], abs=0.002)
    assert f.params["input_delay"] == pytest.approx(TRUE["input_delay"], abs=0.008)
    assert not f.strongly_correlated(0.95)


def test_fit_result_table_mentions_every_free_param():
    logs = [D.synthesize(D.longitudinal_passes()[1], TRUE, "encoder", seed=3)]
    f = M.fit_longitudinal([D.longitudinal_segment(l) for l in logs], free=["speed_gain", "tau_speed"])
    table = f.table(TRUE)
    assert "speed_gain" in table and "tau_speed" in table and "rmse" in table
