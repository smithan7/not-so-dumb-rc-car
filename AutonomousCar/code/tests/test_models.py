import math
import os
import sys

import numpy as np
import pytest

CODE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, CODE_DIR)

from units import mph_to_mps, ft_lbs_to_Nm, oz_in_to_Nm  # noqa: E402
from DriveTrain import DriveTrain  # noqa: E402
from PID_Controller import PID  # noqa: E402
from CarController import CarController  # noqa: E402
from CarParameters import CarParam  # noqa: E402
from SimulatedCarState import SimulatedCarState  # noqa: E402
from Battery import Battery  # noqa: E402
from BatteryParams import BatteryParams  # noqa: E402
from Motor import Motor  # noqa: E402
from Control_Response_Evaluator import Control_Response_Evaluator  # noqa: E402
from Track import Track  # noqa: E402

TRACK_CSV = os.path.join(CODE_DIR, "..", "data", "track_data.csv")


# --- units / drivetrain -------------------------------------------------------

def test_unit_constants():
    assert ft_lbs_to_Nm == pytest.approx(1.355818)
    assert 141.61193227806 * oz_in_to_Nm == pytest.approx(1.0)


def test_driveshaft_rpm_mph_matches_mps():
    dt = DriveTrain()
    assert dt.calc_driveshaft_rpm_from_car_speed_mph(10.0) == pytest.approx(
        dt.calc_driveshaft_rpm_from_car_speed_mps(10.0 * mph_to_mps))


def test_speed_round_trip():
    dt = DriveTrain()
    rpm = dt.calc_driveshaft_rpm_from_car_speed_mph(25.0)
    assert dt.calc_car_speed_mph_from_driveshaft_rpm(rpm) == pytest.approx(25.0)


# --- PID / controller ---------------------------------------------------------

def test_pid_integral_active_by_default():
    pid = PID()
    pid.ki = 1.0
    pid.tic(0.0, 1.0)
    pid.tic(0.0, 1.0)
    assert pid.u_i == pytest.approx(2.0)


def test_pid_integral_clamped():
    pid = PID(ki=1.0, max_error_sum=1.5)
    for _ in range(10):
        pid.tic(0.0, 1.0)
    assert pid.u_i == pytest.approx(1.5)


def test_pid_dt_scaling():
    pid = PID(ki=1.0, kd=1.0)
    pid.tic(0.0, 1.0, dt=0.1)
    pid.tic(0.5, 1.0, dt=0.1)
    assert pid.error_sum == pytest.approx(0.15)
    assert pid.u_d == pytest.approx(-5.0)  # (0.5 - 1.0) / 0.1


def test_pid_default_dt_matches_old_behaviour():
    pid = PID(kp=2.0, ki=0.5, kd=0.25)
    pid.tic(0.0, 1.0)
    u = pid.tic(0.4, 1.0)
    assert u == pytest.approx(2.0 * 0.6 + 0.5 * 1.6 + 0.25 * (0.6 - 1.0))


def test_car_controllers_do_not_share_pids():
    a, b = CarController(), CarController()
    assert a.speed_pid is not b.speed_pid
    a.speed_pid.tic(0.0, 5.0)
    assert b.speed_pid.error_sum == 0.0


def test_step_metrics_both_directions_and_zero_step():
    ev = Control_Response_Evaluator()
    ts = [0, 1, 2, 3]
    up = ev.evaluate_performance(ts, [0, 0.8, 1.1, 1.0], 1.0)
    down = ev.evaluate_performance(ts, [0, -0.8, -1.1, -1.0], -1.0)
    assert up == pytest.approx(down)
    assert up[0] == 1 and up[1] == pytest.approx(0.1)
    assert ev.evaluate_performance(ts, [1, 1, 1, 1], 1.0) == [0.0, 0.0, 0.0]
    assert PID().extract_performance(ts, [0, 0.8, 1.1, 1.0], 1.0) == pytest.approx(up)


# --- simulated car ------------------------------------------------------------

def test_sim_first_steering_command_not_truncated():
    car = SimulatedCarState(np.array([0] * 11), CarParam())  # int array, as the notebooks build it
    car.input_steering(0.5)
    assert car.get_steering_rate() == pytest.approx(CarParam.max_steering_rate)


def test_sim_accel_uses_accel_damping():
    params = CarParam()
    params.accel_damping = 2.0
    params.steer_damping = 99.0
    car = SimulatedCarState(np.zeros(11), params)
    car.input_accel(1.0)
    assert car.x[car.fji] == pytest.approx(2.0)


# --- battery / motor ----------------------------------------------------------

def _battery():
    b = Battery()
    b.read_params(BatteryParams())
    b.calc_vals()
    return b


def test_battery_starts_at_full_pack_voltage():
    b = _battery()
    assert b.max_voltage == pytest.approx(8 * 4.2)
    assert b.min_voltage == pytest.approx(8 * 3.5)
    assert b.voltage == pytest.approx(b.max_voltage)


def test_battery_discharge():
    b = _battery()
    b.draw_current(8.0, 1800.0)  # 8 A for half an hour from an 8 Ah pack
    assert b.percent_capacity == pytest.approx(50.0)
    assert b.voltage == pytest.approx((b.max_voltage + b.min_voltage) / 2)


def _motor(kv=1000.0, cells=6, resistance=0.01):
    m = Motor()
    m.kv, m.max_battery_cells, m.resistance = kv, cells, resistance
    m.diam, m.length, m.specific_heat_coeff, m.mass = 0.05, 0.07, 0.9, 500.0
    m.calc_vals()
    return m


def test_motor_torque_is_kt_times_current():
    m = _motor()
    kt_Nm = 1352.0 / 1000.0 * oz_in_to_Nm
    assert m.calc_motor_torque(50.0, rpm=20000) == pytest.approx(kt_Nm * 50.0)
    assert m.calc_current_from_torque(kt_Nm * 50.0, rpm=20000) == pytest.approx(50.0)


def test_motor_full_charge_rpm_not_past_max():
    m = _motor()
    assert m.calc_motor_rpm_from_voltage(6 * 4.2) <= m.max_rpm


def test_motor_current_from_voltage_back_emf():
    m = _motor()
    assert m.calc_current_from_voltage(25.2, rpm=0) == pytest.approx(2520.0)
    assert m.calc_current_from_voltage(25.2, rpm=m.max_rpm) == pytest.approx(0.0)
    assert m.calc_current_from_voltage(10.0, rpm=m.max_rpm) == 0.0  # no negative current


# --- track --------------------------------------------------------------------

@pytest.fixture(scope="module")
def track():
    return Track(TRACK_CSV)


def test_track_detected_closed(track):
    assert track.closed


def test_nearest_edge_matches_scalar_version(track):
    p = [10.0, 10.0]
    k, d, t = track.find_nearest_edge(p)
    rows = track.track_df
    ref = [track.nearest_point_on_line_segment([r.x, r.y], [r.xn, r.yn], p) for r in rows.itertuples()]
    ref_k = int(np.argmin([r[2] for r in ref]))
    assert k == ref_k
    assert d == pytest.approx(ref[ref_k][2])


def test_seeded_search_wraps_start_finish(track):
    n = len(track.track_df)
    p = [track.track_df["x"][1] + 0.5, track.track_df["y"][1]]  # just past the line, on segment 1
    k, _, _ = track.find_nearest_edge_with_seed(p, n - 2, window_size=5)
    assert k == track.find_nearest_edge(p)[0]


def test_speed_profile_respects_brake_and_accel_limits(track):
    params = CarParam()
    track.get_speed_limits_from_roc(params)
    track.back_propagate_speed_limits_to_set_speed(params)
    v = track.track_df["speed"].to_numpy()
    d = track.track_df["dist"].to_numpy()
    v_next = np.roll(v, -1)
    assert np.all(v <= track.track_df["speed limit"].to_numpy() + 1e-6)
    assert np.all(v_next**2 <= v**2 + 2 * params.max_forward_accel * d + 1e-6)
    assert np.all(v**2 <= v_next**2 + 2 * abs(params.max_braking_accel) * d + 1e-6)
