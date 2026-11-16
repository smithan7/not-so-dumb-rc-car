"""Experiment design: does the information math predict real fits, and does it ask for what's missing?"""

import os
import sys
import warnings

import numpy as np
import pytest

CODE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, CODE_DIR)

import sysid_design as SD  # noqa: E402
import sysid_maneuvers as D  # noqa: E402
import sysid_model as M  # noqa: E402

TRUE = M.default_params(speed_gain=0.05, tau_speed=0.6, deadband_us=30, brake_gain=0.05, roll_decel=0.4,
                        drag_coeff=0.02, steer_gain=0.0019, steer_center_us=1506, tau_steer=0.07, v_char=8.0,
                        gyro_bias=0.008, input_delay=0.04)
LIMITS = SD.Limits(max_speed_mps=3.0, throttle_max_us=1600, area_m=(40.0, 25.0))


def test_targets_cover_every_free_parameter():
    for k in M.LONGITUDINAL_FREE + M.LATERAL_FREE:
        assert k in SD.TARGETS
    assert SD.TARGETS["input_delay"] == ("abs", 0.010) and SD.TARGETS["steer_center_us"] == ("abs", 2.0)
    assert SD.TARGETS["tau_speed"] == ("rel", 0.05)


def test_candidates_respect_the_stage_limits():
    cands = SD.candidates(TRUE, LIMITS)
    assert len(cands) >= 15
    kinds = {c.kind for c in cands}
    assert kinds == {"longitudinal", "lateral"}
    for c in cands:
        fp = D.footprint(c.schedule, TRUE)
        assert fp["top_speed_mps"] <= LIMITS.max_speed_mps * 1.05, c.name
        assert max(s[1] for s in c.schedule) <= LIMITS.throttle_max_us
        assert max(fp["x_extent_m"], fp["y_extent_m"]) <= max(LIMITS.area_m), c.name
        assert min(fp["x_extent_m"], fp["y_extent_m"]) <= min(LIMITS.area_m), c.name
    # a lower speed cap gives gentler maneuvers
    slow = SD.candidates(TRUE, SD.Limits(max_speed_mps=1.0, throttle_max_us=1600, area_m=(40, 25)))
    assert max(D.footprint(c.schedule, TRUE)["top_speed_mps"] for c in slow) <= 1.05


def _fit(schedules, kind, seed=0):
    logs = [D.synthesize(s, TRUE, "encoder", seed=seed + i, wheelbase=M.WHEELBASE_M) for i, s in enumerate(schedules)]
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        if kind == "longitudinal":
            return M.fit_longitudinal([D.longitudinal_segment(l) for l in logs], p0={"input_delay": TRUE["input_delay"]})
        return M.fit_lateral([D.lateral_segment(l) for l in logs], wheelbase=M.WHEELBASE_M)


def test_predicted_uncertainty_matches_an_actual_fit():
    """Information from simulated sensitivities ~ the covariance the real fit reports."""
    cands = {c.name: c for c in SD.candidates(TRUE, LIMITS)}
    long_names = [n for n, c in cands.items() if c.kind == "longitudinal"][:4]
    # two speeds: steering at one speed alone can't separate steer_gain from v_char, and the fit
    # then settles on an arbitrary point with an over-confident covariance
    lat = [c for c in cands.values() if c.kind == "lateral"]
    lat_names = [c.name for c in lat if c.cruise_mps < 1.5][:2] + [c.name for c in lat if c.cruise_mps > 1.5][:2]
    for names, kind, free in ((long_names, "longitudinal", M.LONGITUDINAL_FREE), (lat_names, "lateral", M.LATERAL_FREE)):
        fit = _fit([cands[n].schedule for n in names], kind)
        info = sum(SD.information(TRUE, cands[n].schedule, SD.Noise(speed=fit.rmse, yaw=fit.rmse))[kind] for n in names)
        pred = SD.std_from_information(info)
        for i, k in enumerate(free):
            if fit.std[k] > 0 and np.isfinite(fit.std[k]):
                assert 0.4 < pred[i] / fit.std[k] < 2.5, (k, pred[i], fit.std[k])


def test_with_no_data_it_asks_for_the_basics():
    plan = SD.suggest(None, TRUE, LIMITS, n=4)
    assert len(plan) >= 3
    assert {s.maneuver.kind for s in plan} == {"longitudinal", "lateral"}
    assert any(s.maneuver.brakes for s in plan)
    assert all(s.improves for s in plan)  # every pick says which parameters it helps


def test_missing_braking_is_noticed_and_asked_for():
    no_brake = [D.holds((2, 1500, 1500), (4, lv, 1500), (5, 1500, 1500)) for lv in (1560, 1575, 1590)]
    fit = _fit(no_brake, "longitudinal")
    state = SD.State.from_fits(fit_long=fit, fit_lat=None)
    before = state.std("longitudinal")["brake_gain"]
    plan = SD.suggest(state, TRUE, LIMITS, n=1, kinds={"longitudinal"})
    first = plan[0]
    assert first.maneuver.kind == "longitudinal" and first.maneuver.brakes
    assert "brake_gain" in [k for k, _, _ in first.improves]
    after = dict((k, a) for k, b, a in first.improves)["brake_gain"]
    assert after < before / 3


def test_missing_second_speed_is_asked_for_lateral():
    """v_char needs steering at two speeds; with only slow doublets it should ask for faster ones."""
    slow = [s for s in SD.candidates(TRUE, LIMITS) if s.kind == "lateral" and s.cruise_mps < 1.5][:3]
    state = SD.State()
    for c in slow:
        state.info["lateral"] = state.info["lateral"] + SD.information(TRUE, c.schedule)["lateral"]
    assert state.std("lateral")["v_char"] / TRUE["v_char"] > 0.2  # one speed: v_char unknown
    plan = SD.suggest(state, TRUE, LIMITS, n=1, kinds={"lateral"})
    assert plan[0].maneuver.kind == "lateral" and plan[0].maneuver.cruise_mps > 1.5
    assert "v_char" in [k for k, _, _ in plan[0].improves]


def test_gate_report():
    rows = SD.gate({"tau_speed": 1.0, "steer_center_us": 1500.0, "input_delay": 0.05},
                   {"tau_speed": 0.03, "steer_center_us": 3.0, "input_delay": 0.004})
    by = {r["param"]: r for r in rows}
    assert by["tau_speed"]["ok"] and not by["steer_center_us"]["ok"] and by["input_delay"]["ok"]
    assert by["tau_speed"]["gating"]
