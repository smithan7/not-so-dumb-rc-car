import math
import os
import sys

import numpy as np
import pytest

CODE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, CODE_DIR)

import closed_loop as C  # noqa: E402
import sysid_model as M  # noqa: E402
from controllers import MPPI, DelayCompensator, PurePursuit  # noqa: E402
from course import parking_lot_course  # noqa: E402
from state_estimator import IX, IY, initial_ukf  # noqa: E402
from vehicle_dynamics import DynamicPlant, KinematicModel, KinematicPlant, ParamUncertainty  # noqa: E402


@pytest.fixture(scope="module")
def course():
    return parking_lot_course()


def test_course_fits_lot_and_lookup_matches_projection(course):
    w, h = course.extent()
    assert w <= 40 and h <= 25
    assert course.min_radius() > course.width  # edges never fold over
    rng = np.random.default_rng(0)
    s0 = rng.uniform(0, course.length, 200)
    e0 = rng.uniform(-1.4, 1.4, 200)
    xy, _, _, n = course.at(s0)
    p = xy + e0[:, None] * n
    s, e = course.lookup(p[:, 0], p[:, 1])
    assert np.max(np.abs(e - e0)) < 0.1
    assert np.max(np.abs(course.wrap_ds(s - s0))) < 0.15
    assert course.lookup(np.array([-100.0]), np.array([0.0]))[1][0] > 100  # off the grid = off track


def test_batched_model_matches_sysid_simulation():
    p = M.default_params(input_delay=0.0)
    t = np.arange(0, 6, 0.002)
    thr = lambda tt: np.where(tt < 1, 1500.0, 1600.0)
    st = lambda tt: np.where(tt < 3, 1500.0, 1580.0)
    ref = M.simulate(p, t, thr, st)
    model = KinematicModel(p)
    x = np.zeros(5)
    for k in range(len(t) - 1):
        x, _, _ = model.step(x, thr(t[k]), st(t[k]), t[k + 1] - t[k])
    assert x[3] == pytest.approx(ref["v"][-1], rel=1e-3)
    assert x[4] == pytest.approx(ref["delta"][-1], rel=1e-3)
    assert math.hypot(x[0] - ref["x"][-1], x[1] - ref["y"][-1]) < 0.1


def test_dynamic_plant_matches_kinematic_in_linear_region_only():
    p = M.default_params(input_delay=0.0)
    out = {}
    for thr in (1560, 1610):
        dyn, kin = DynamicPlant(p), KinematicPlant(p)
        dyn.reset(), kin.reset()
        for _ in range(12000):
            dyn.step(thr, 1550, 0.001), kin.step(thr, 1550, 0.001)
        out[thr] = (dyn.yaw_rate, kin.yaw_rate, dyn.lateral_accel)
    lin, sat = out[1560], out[1610]
    assert lin[0] == pytest.approx(lin[1], rel=0.03)      # ~0.1 g: same car
    assert sat[2] > 0.6 * 9.81 and sat[0] < 0.95 * sat[1]  # near the limit: tires saturate


def test_ukf_late_gps_equals_in_order_processing():
    rng = np.random.default_rng(1)
    ts = np.arange(0.01, 2.0, 0.01)
    meas = [(t, "gyro", 0.3 + rng.normal(0, 0.02)) for t in ts] + [(t, "enc", 3.0) for t in ts[::2]]
    gps = [(t, np.array([3.0 * t, 0.2 + rng.normal(0, 0.3)])) for t in np.arange(0.1, 2.0, 0.1)]
    a, b = initial_ukf(0, 0, 0, 3.0), initial_ukf(0, 0, 0, 3.0)
    events = sorted(meas + [(t, "gps", v) for t, v in gps], key=lambda m: m[0])
    for t, kind, v in events:  # a: every measurement on time
        a.update(t, kind, v)
    pending = list(gps)
    for t, kind, v in sorted(meas, key=lambda m: m[0]):  # b: each fix arrives 0.2 s late
        b.update(t, kind, v)
        while pending and pending[0][0] + 0.2 <= t:
            tf, xy = pending.pop(0)
            b.update(tf, "gps", xy)
    for tf, xy in pending:
        b.update(tf, "gps", xy)
    a.predict_to(b.t)
    assert np.allclose(a.z, b.z, atol=1e-6) and np.allclose(a.P, b.P, atol=1e-8)


def test_ukf_covariance_bounded_below_by_gps_bias():
    ukf = initial_ukf(0, 0, 0, 0.0, r_gps=0.5, gps_bias_sd=0.3)
    for k in range(1, 600):  # 60 s parked: gyro/encoder read zero, GPS reads the start point
        for j in range(5):
            ukf.update(k * 0.1 + j * 0.02 - 0.08, "gyro", 0.0)
            ukf.update(k * 0.1 + j * 0.02 - 0.08, "enc", 0.0)
        ukf.update(k * 0.1, "gps", np.zeros(2))
    assert math.sqrt(ukf.P[IX, IX]) > 0.2  # an unobservable bias must not average away


def test_delay_compensator_predicts_in_flight_commands():
    p = M.default_params(input_delay=0.1)
    dc = DelayCompensator(p)
    dc.advance(0.0)
    dc.record(0.0, 1500.0, 1700.0)  # full lock, acts at t = 0.1
    x = dc.predict(0.0, [0, 0, 0, 3.0, 0.0])
    assert x[0] == pytest.approx(0.3, abs=0.01) and abs(x[4]) < 1e-9  # nothing has acted yet
    dc.advance(0.25)
    assert dc.delta > 0.5 * p["delta_max"]


def test_param_uncertainty_samples_within_bounds():
    class Fit:
        free = ["speed_gain", "tau_speed", "gyro_bias"]
        params = M.default_params()
        std = {"speed_gain": 0.5, "tau_speed": 0.01, "gyro_bias": 1.0}
        corr = np.eye(3)
    u = ParamUncertainty.from_fits([Fit()], inflate=1.0)
    assert u.names == ["speed_gain", "tau_speed"]
    s = u.sample(500, np.random.default_rng(0))
    assert s["speed_gain"].shape == (500,) and s["speed_gain"].min() >= M.PARAMS["speed_gain"][1]
    assert np.std(s["tau_speed"]) == pytest.approx(0.01, rel=0.2)


def test_track_tightening_grows_with_estimate_uncertainty(course):
    m = MPPI(course, M.default_params(), samples=10, horizon=5, z_sigma=2.0)
    x = np.array([*course.xy[0], course.psi[0], 5.0, 0.0])
    tight = np.diag([0.01, 0.01, 0, 0, 0, 0, 0, 0]) ** 2
    loose = np.diag([0.4, 0.4, 0, 0, 0, 0, 0, 0]) ** 2
    assert m._half_width(x, tight) == pytest.approx(m.half_width - 0.02, abs=1e-3)
    assert m._half_width(x, loose) == pytest.approx(m.half_width - 0.8, abs=0.01)


@pytest.mark.parametrize("name", ["pure_pursuit", "mppi_ukf"])
def test_closed_loop_lap_stays_on_track(course, name):
    p = M.default_params(input_delay=0.04)
    ctl = C.make_controller(name, course, p, None, samples=150, seed=0)
    r = C.drive(ctl, course, "kinematic", p, C.GPS_RTK, duration_s=30, laps=1, seed=0)
    assert len(r.laps) == 1 and not r.left_track
    assert r.off_track_frac < 0.01


def test_ukf_survives_gyro_and_encoder_dropout():
    ukf = initial_ukf(0, 0, 0, 0.0)
    for k in range(1, 600):  # 60 s of GPS only
        ukf.update(k * 0.1, "gps", np.zeros(2))
    assert np.all(np.isfinite(ukf.z)) and np.all(np.isfinite(ukf.P))
