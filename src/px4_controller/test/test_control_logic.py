"""Unit tests for px4_controller.control_logic (no ROS, no aircraft).

    python3 -m pytest src/px4_controller/test/test_control_logic.py
"""
import math
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from px4_controller import control_logic as cl  # noqa: E402

LIM = cl.Limits(**cl.LIMIT_DEFAULTS).validate()
MAX_DT = 0.1


def test_defaults_match_console_launch():
    launch = os.path.join(os.path.dirname(__file__), '..', 'launch', 'console.launch.py')
    text = open(launch).read()
    for name, value in cl.LIMIT_DEFAULTS.items():
        assert "DeclareLaunchArgument('{}', default_value='{}'".format(name, value) in text


@pytest.mark.parametrize('kw', [
    dict(min_altitude_m=0.0),                          # min must be > 0
    dict(takeoff_altitude_m=0.5),                      # takeoff must be > min
    dict(takeoff_altitude_m=3.0),                      # takeoff must be <= max
    dict(max_altitude_m=0.4),                          # max below min
    dict(max_climb_rate_mps=0.0),
    dict(cmd_timeout_s=0.0),
    dict(max_altitude_m=float('nan')),
])
def test_limits_reject_inconsistent(kw):
    with pytest.raises(ValueError):
        cl.Limits(**{**cl.LIMIT_DEFAULTS, **kw}).validate()


def test_climb_rate_clamped():
    assert cl.clamp_climb_rate(5.0, 0.3) == 0.3
    assert cl.clamp_climb_rate(-5.0, 0.3) == -0.3
    assert cl.clamp_climb_rate(0.1, 0.3) == 0.1
    assert cl.clamp_climb_rate(float('nan'), 0.3) == 0.0
    assert cl.clamp_climb_rate(float('inf'), 0.3) == 0.0


def test_integrate_altitude_rate_and_feedforward():
    t, rate = cl.integrate_altitude(1.0, 10.0, 0.05, LIM, MAX_DT)  # clamped to 0.3 m/s
    assert t == pytest.approx(1.0 + 0.3 * 0.05)
    assert rate == pytest.approx(0.3)


def test_integrate_altitude_one_second_climb():
    t = LIM.takeoff_altitude_m
    for _ in range(20):
        t, _ = cl.integrate_altitude(t, 1.0, 0.05, LIM, MAX_DT)
    assert t == pytest.approx(LIM.takeoff_altitude_m + LIM.max_climb_rate_mps)


def test_integrate_altitude_stays_in_envelope():
    t = LIM.takeoff_altitude_m
    for _ in range(1000):
        t, rate = cl.integrate_altitude(t, 1.0, 0.05, LIM, MAX_DT)
    assert t == LIM.max_altitude_m and rate == 0.0          # no feed-forward at the limit
    for _ in range(1000):
        t, rate = cl.integrate_altitude(t, -1.0, 0.05, LIM, MAX_DT)
    assert t == LIM.min_altitude_m and rate == 0.0


def test_integrate_altitude_dt_guard():
    # a stalled timer (10 s gap) integrates at most MAX_DT
    t, _ = cl.integrate_altitude(1.0, 0.3, 10.0, LIM, MAX_DT)
    assert t == pytest.approx(1.0 + 0.3 * MAX_DT)
    # negative / NaN dt does nothing
    assert cl.integrate_altitude(1.0, 0.3, -1.0, LIM, MAX_DT) == (1.0, 0.0)
    assert cl.integrate_altitude(1.0, 0.3, float('nan'), LIM, MAX_DT) == (1.0, 0.0)


def test_is_fresh():
    assert not cl.is_fresh(None, 100.0, 0.5)
    assert cl.is_fresh(99.6, 100.0, 0.5)
    assert cl.is_fresh(99.5, 100.0, 0.5)
    assert not cl.is_fresh(99.4, 100.0, 0.5)
    assert not cl.is_fresh(100.1, 100.0, 0.5)   # receipt in the future: not trusted


def _sel(**kw):
    args = dict(flying=True, deadman_pressed=True, deadman_fresh=True, allow_nav=False,
                cmd_vel_fresh=True, cmd_vel_nav_fresh=True)
    args.update(kw)
    return cl.select_command(**args)


def test_select_command_timeouts_hover():
    assert _sel() == cl.USE_MANUAL
    assert _sel(allow_nav=True) == cl.USE_NAV
    assert _sel(flying=False) == cl.HOVER_NOT_FLYING
    assert _sel(deadman_pressed=False) == cl.HOVER_DEADMAN_RELEASED
    # deadman publisher died while "pressed": hover
    assert _sel(deadman_fresh=False) == cl.HOVER_DEADMAN_STALE
    # cmd_vel publisher died: hover, even with the deadman held
    assert _sel(cmd_vel_fresh=False) == cl.HOVER_MANUAL_STALE
    assert _sel(allow_nav=True, cmd_vel_nav_fresh=False) == cl.HOVER_NAV_STALE
    # a fresh manual cmd_vel does not drive while nav is selected
    assert _sel(allow_nav=True, cmd_vel_nav_fresh=False, cmd_vel_fresh=True) == cl.HOVER_NAV_STALE


def test_select_command_end_to_end_timeout():
    """A cmd_vel received once keeps driving only until cmd_timeout_s."""
    rx = 10.0
    for now, expected in [(10.2, cl.USE_MANUAL), (10.5, cl.USE_MANUAL), (10.51, cl.HOVER_DEADMAN_STALE)]:
        fresh = cl.is_fresh(rx, now, LIM.cmd_timeout_s)
        assert _sel(deadman_fresh=fresh, cmd_vel_fresh=fresh) == expected


def test_lateral_flu_to_ned_level_heading_north():
    q_identity = [1.0, 0.0, 0.0, 0.0]   # wxyz: FRD aligned with NED
    np.testing.assert_allclose(cl.lateral_flu_to_ned(1.0, 0.0, q_identity), [1, 0, 0], atol=1e-12)
    # +y (left) in FLU is -East
    np.testing.assert_allclose(cl.lateral_flu_to_ned(0.0, 1.0, q_identity), [0, -1, 0], atol=1e-12)


def test_lateral_flu_to_ned_yawed_east_and_tilted():
    # heading east (yaw +90 deg in NED), pitched 10 deg: forward command still horizontal-only
    r = Rotation.from_euler('ZYX', [90, 10, 0], degrees=True)
    x, y, z, w = r.as_quat()
    v = cl.lateral_flu_to_ned(1.0, 0.0, [w, x, y, z])
    assert v[2] == 0.0
    assert v[1] > 0.9 and abs(v[0]) < 1e-9


def test_base_footprint_level():
    t, q = cl.base_link_to_footprint([1.0, 2.0, 1.5], [0, 0, 0, 1])
    np.testing.assert_allclose(t, [0, 0, -1.5], atol=1e-12)
    np.testing.assert_allclose(np.abs(q), [0, 0, 0, 1], atol=1e-12)


def test_base_footprint_composes_to_ground_yaw_only():
    r_ob = Rotation.from_euler('ZYX', [40, 8, -5], degrees=True)
    p_ob = np.array([3.0, -1.0, 1.2])
    t_bf, q_bf = cl.base_link_to_footprint(p_ob, r_ob.as_quat())
    # odom -> footprint = (odom -> base_link) * (base_link -> footprint)
    p_of = p_ob + r_ob.apply(t_bf)
    r_of = r_ob * Rotation.from_quat(q_bf)
    np.testing.assert_allclose(p_of, [3.0, -1.0, 0.0], atol=1e-9)
    roll_pitch_yaw = r_of.as_euler('xyz', degrees=True)
    np.testing.assert_allclose(roll_pitch_yaw[:2], [0, 0], atol=1e-9)
    assert roll_pitch_yaw[2] == pytest.approx(40.0)


class _FakeStatusMeta(type):
    @property
    def NAVIGATION_STATE_OFFBOARD(cls):  # noqa: N802 (rosidl constant style)
        return 14

    @property
    def NAVIGATION_STATE_MAX(cls):  # noqa: N802
        return 31


class _FakeStatus(metaclass=_FakeStatusMeta):
    NAVIGATION_STATE_POSCTL = 2


def test_nav_state_names_reads_metaclass_constants():
    names = cl.nav_state_names(_FakeStatus)
    assert names == {14: 'OFFBOARD', 2: 'POSCTL'}
    assert cl.nav_state_name(14, names) == 'OFFBOARD'
    assert cl.nav_state_name(31, names) == 'UNKNOWN(31)'


def test_nav_state_names_real_px4_msgs():
    px4_msgs = pytest.importorskip('px4_msgs.msg')
    names = cl.nav_state_names(px4_msgs.VehicleStatus)
    assert names[px4_msgs.VehicleStatus.NAVIGATION_STATE_OFFBOARD] == 'OFFBOARD'
    assert names[px4_msgs.VehicleStatus.NAVIGATION_STATE_AUTO_LAND] == 'AUTO_LAND'


def test_battery_fields():
    px4 = SimpleNamespace(voltage_v=15.2, current_a=12.0, remaining=0.62, capacity=5000,
                          discharged_mah=1900.0, temperature=float('nan'), connected=True,
                          cell_count=4, voltage_cell_v=[3.8] * 4 + [0.0] * 10)
    f = cl.battery_fields(px4)
    assert f['voltage'] == pytest.approx(15.2)
    assert f['current'] == pytest.approx(-12.0)     # discharging is negative in BatteryState
    assert f['percentage'] == pytest.approx(0.62)
    assert f['capacity'] == pytest.approx(5.0)
    assert f['charge'] == pytest.approx(3.1)
    assert f['present'] is True
    assert f['cell_voltage'] == [pytest.approx(3.8)] * 4


def test_battery_fields_unknowns_are_nan():
    px4 = SimpleNamespace(voltage_v=0.0, current_a=-1.0, remaining=-1.0, capacity=0,
                          discharged_mah=-1.0, temperature=float('nan'), connected=False,
                          cell_count=0, voltage_cell_v=[0.0] * 14)
    f = cl.battery_fields(px4)
    for k in ('voltage', 'current', 'percentage', 'capacity', 'charge'):
        assert math.isnan(f[k]), k
    assert f['cell_voltage'] == []


def _rate(arrivals_s, period_s):
    due, sent = None, 0
    for t in arrivals_s:
        send, due = cl.rate_limit(t, due, period_s)
        sent += send
    return sent / (arrivals_s[-1] - arrivals_s[0])


def test_rate_limit_caps_fast_stream():
    t = np.arange(0.0, 10.0, 0.01)                     # 100 Hz odometry
    assert _rate(t, 1 / 30) == pytest.approx(30.0, rel=0.05)


def test_rate_limit_does_not_halve_jittery_stream_near_rate():
    rng = np.random.default_rng(0)
    t = np.cumsum(1 / 30 + rng.uniform(-0.004, 0.004, 3000))   # 30 Hz +-4 ms
    assert _rate(t, 1 / 30) > 25.0


def test_rate_limit_passes_slow_stream():
    t = np.arange(0.0, 10.0, 0.05)                     # 20 Hz: every message
    assert _rate(t, 1 / 30) == pytest.approx(20.0, rel=0.02)
