"""Unit tests for px4_controller.mavros_logic (no ROS, no MAVROS, no aircraft).

    python3 -m pytest src/px4_controller/test/test_mavros_logic.py
"""
import math
import os
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from px4_controller import control_logic as cl  # noqa: E402
from px4_controller import mavros_logic as ml  # noqa: E402

LIM = cl.Limits(**cl.LIMIT_DEFAULTS).validate()
GAIN = 1.0
HZ20 = 0.05


# ---------------------------------------------------------------------- #
# OFFBOARD start: the stream is the heartbeat
# ---------------------------------------------------------------------- #

def _run(clock, requester, t0, t1, connected=True, armed=False, mode='POSCTL', period=HZ20):
    """Stream from t0 to t1 as mavros_io.stream_tick does; return [(t, action)]."""
    out = []
    for i in range(int(round((t1 - t0) / period)) + 1):
        t = round(t0 + i * period, 6)       # no float drift across the 1 s thresholds
        clock.note_sent(t)
        r = requester.poll(t, mode, armed, connected, clock.streaming_s(t))
        if r:
            out.append((round(t, 3), r))
    return out


def _pair():
    return ml.StreamClock(), cl.OffboardRequester()


def test_no_offboard_request_at_start_while_disarmed():
    # parity with XRCE: streaming alone never requests OFFBOARD
    c, r = _pair()
    assert _run(c, r, 0.0, 30.0) == []


def test_no_offboard_request_before_one_second_of_stream():
    c, r = _pair()
    r.start_arm(0.0)
    assert _run(c, r, 0.0, 0.95) == []
    assert _run(c, r, 1.0, 1.0) == [(1.0, 'offboard')]


def test_offboard_request_needs_connection():
    c, r = _pair()
    r.start_arm(0.0)
    assert _run(c, r, 0.0, 3.0, connected=False) == []


def test_stream_gap_restarts_the_one_second():
    c, r = _pair()
    r.start_arm(0.0)
    _run(c, r, 0.0, 0.9)
    # 0.6 s gap (> max_gap 0.5 s: PX4 would have seen the stream stop)
    assert c.streaming_s(1.5) == 0.0
    assert _run(c, r, 1.5, 2.45) == []                    # only 0.95 s since the restart
    assert _run(c, r, 2.5, 2.5) == [(2.5, 'offboard')]


def test_slow_stream_never_counts():
    c, r = _pair()
    r.start_arm(0.0)
    # 1 Hz (< 2 Hz): every sample is a fresh start
    assert _run(c, r, 0.0, 10.0, period=1.0) == []


def test_arm_sequence_offboard_then_arm_once_seen():
    c, r = _pair()
    r.start_arm(0.0)
    assert _run(c, r, 0.0, 1.0) == [(1.0, 'offboard')]
    req = _run(c, r, 1.05, 1.2, mode='OFFBOARD')
    assert req == [(1.05, 'arm')]


def test_takeoff_requests_offboard_while_armed_until_seen():
    c, r = _pair()
    _run(c, r, 0.0, 2.0, armed=True, mode='POSCTL')
    r.start_takeoff(2.0)
    req = _run(c, r, 2.05, 4.1, armed=True, mode='POSCTL')
    assert [a for _, a in req] == ['offboard'] * 3        # 2.05, 3.05, 4.05
    _run(c, r, 4.15, 4.2, armed=True, mode='OFFBOARD')
    assert _run(c, r, 4.25, 8.0, armed=True, mode='POSCTL') == []


def test_takeoff_request_cancelled_by_land():
    c, r = _pair()
    _run(c, r, 0.0, 2.0, armed=True, mode='POSCTL')
    r.start_takeoff(2.0)
    r.cancel()
    assert _run(c, r, 2.05, 6.0, armed=True, mode='POSCTL') == []


def test_disarm_does_not_reopen_a_request():
    c, r = _pair()
    _run(c, r, 0.0, 2.0, armed=True, mode='OFFBOARD')
    assert _run(c, r, 2.05, 8.0, armed=False, mode='AUTO.LAND') == []


# ---------------------------------------------------------------------- #
# Stream contents
# ---------------------------------------------------------------------- #

def test_hold_tracks_while_disarmed_latches_when_armed():
    h = ml.HoldTracker()
    assert h.update(False, False, None) is None                       # no odometry: nothing
    np.testing.assert_allclose(h.update(False, False, (1, 2, 0)), [1, 2, 0])
    np.testing.assert_allclose(h.update(False, False, (1.1, 2, 0)), [1.1, 2, 0])  # tracks
    np.testing.assert_allclose(h.update(True, False, (1.2, 2, 0)), [1.2, 2, 0])   # latched here
    np.testing.assert_allclose(h.update(True, False, (5.0, 5, 3)), [1.2, 2, 0])   # held
    assert h.update(True, True, (5.0, 5, 3)) is None                  # core commands
    np.testing.assert_allclose(h.update(True, False, (6.0, 5, 3)), [6.0, 5, 3])   # re-latched


def test_stream_setpoint_only_while_armed():
    sp = object()
    assert ml.stream_setpoint(True, True, sp, None) == 'setpoint'
    assert ml.stream_setpoint(False, True, sp, (0, 0, 0)) == 'hold'   # stale climb never meets an arm
    assert ml.stream_setpoint(True, False, sp, (0, 0, 0)) == 'hold'
    assert ml.stream_setpoint(True, True, None, (0, 0, 0)) == 'hold'
    assert ml.stream_setpoint(False, False, None, None) is None


def test_hold_target_fields():
    f = ml.hold_target((1.0, 2.0, 0.1))
    assert f['coordinate_frame'] == ml.FRAME_LOCAL_NED
    assert f['type_mask'] == ml.IGNORE_VEL | ml.IGNORE_ACC | ml.IGNORE_YAW
    assert not f['type_mask'] & (ml.IGNORE_POS | ml.IGNORE_YAW_RATE)
    assert f['position'] == (1.0, 2.0, 0.1) and f['yaw_rate'] == 0.0


def _vel_sp(vx, vy, yaw_speed_ned, climb, target):
    return cl.Setpoint(position_ned=np.array([np.nan, np.nan, -target]),
                       velocity_ned=np.array([9.9, 9.9, -climb]),   # NED: must not be used
                       yaw_speed=yaw_speed_ned, body_velocity_flu=(vx, vy),
                       climb_rate_mps=climb, target_altitude_m=target)


def test_body_velocity_is_flu_body_ned_no_own_flip():
    # forward 0.5, left 0.2, yaw left (CCW) 0.3 rad/s: core yaw_speed is NED (= -0.3)
    f = ml.position_target(_vel_sp(0.5, 0.2, -0.3, 0.0, 1.0), 1.0, LIM.max_climb_rate_mps, GAIN)
    assert f['coordinate_frame'] == ml.FRAME_BODY_NED
    assert f['velocity'] == pytest.approx((0.5, 0.2, 0.0))   # FLU as given; MAVROS flips to FRD
    assert f['yaw_rate'] == pytest.approx(0.3)               # ROS sign; MAVROS flips to NED
    m = f['type_mask']
    assert m & ml.IGNORE_POS == ml.IGNORE_POS
    assert m & ml.IGNORE_ACC == ml.IGNORE_ACC
    assert m & ml.IGNORE_YAW
    assert not m & (ml.IGNORE_VEL | ml.IGNORE_YAW_RATE)


def test_body_velocity_holds_altitude_through_vz():
    # climb feed-forward 0.1 m/s, 0.1 m below target -> 0.1 + 1.0 * 0.1
    f = ml.position_target(_vel_sp(1.0, 0.0, 0.0, 0.1, 1.0), 0.9, LIM.max_climb_rate_mps, GAIN)
    assert f['velocity'][2] == pytest.approx(0.2)
    # far below target: clamped to max_climb_rate
    f = ml.position_target(_vel_sp(1.0, 0.0, 0.0, 0.0, 2.0), 0.5, LIM.max_climb_rate_mps, GAIN)
    assert f['velocity'][2] == pytest.approx(LIM.max_climb_rate_mps)
    # above target: descend, clamped
    f = ml.position_target(_vel_sp(1.0, 0.0, 0.0, 0.0, 1.0), 2.0, LIM.max_climb_rate_mps, GAIN)
    assert f['velocity'][2] == pytest.approx(-LIM.max_climb_rate_mps)
    # no altitude measurement: feed-forward only
    f = ml.position_target(_vel_sp(1.0, 0.0, 0.0, 0.1, 1.0), None, LIM.max_climb_rate_mps, GAIN)
    assert f['velocity'][2] == pytest.approx(0.1)


def test_hover_and_takeoff_are_local_position_in_enu():
    # hover at NED (n=3, e=1, d=-0.75) with a 0.2 m/s climb feed-forward, yaw rate NED +0.1
    sp = cl.Setpoint(position_ned=np.array([3.0, 1.0, -0.75]), velocity_ned=np.array([0, 0, -0.2]),
                     yaw_speed=0.1, climb_rate_mps=0.2, target_altitude_m=0.75)
    f = ml.position_target(sp, 0.7, LIM.max_climb_rate_mps, GAIN)
    assert f['coordinate_frame'] == ml.FRAME_LOCAL_NED
    assert f['position'] == pytest.approx((1.0, 3.0, 0.75))   # ENU: east, north, up
    assert f['velocity'] == pytest.approx((0.0, 0.0, 0.2))    # up
    assert f['yaw_rate'] == pytest.approx(-0.1)
    assert f['type_mask'] == ml.IGNORE_ACC | ml.IGNORE_YAW


def test_yaw_step_is_enu_yaw():
    # NED yaw 0 (north) -> ENU yaw +90 deg; NED +90 (east) -> ENU 0
    for yaw_ned, yaw_enu in [(0.0, math.pi / 2), (math.pi / 2, 0.0), (-math.pi / 2, math.pi), (math.pi, -math.pi / 2)]:
        sp = cl.Setpoint(position_ned=np.array([0.0, 0.0, -1.0]), velocity_ned=np.zeros(3), yaw_rad=yaw_ned)
        f = ml.position_target(sp, 1.0, LIM.max_climb_rate_mps, GAIN)
        assert not f['type_mask'] & ml.IGNORE_YAW
        assert math.cos(f['yaw'] - yaw_enu) == pytest.approx(1.0)


def test_nan_position_axis_is_ignored():
    sp = cl.Setpoint(position_ned=np.array([np.nan, 2.0, -1.0]), velocity_ned=np.zeros(3))
    f = ml.position_target(sp, 1.0, LIM.max_climb_rate_mps, GAIN)
    # MAVROS converts the values ENU -> NED but passes type_mask through, so
    # the ignore bits name MAVLink (NED) axes: NaN north -> IGNORE_PX.
    assert f['type_mask'] & ml.IGNORE_PX
    assert not f['type_mask'] & (ml.IGNORE_PY | ml.IGNORE_PZ)
    assert all(math.isfinite(v) for v in f['position'])


def test_enu_ned_roundtrip_and_orientation():
    np.testing.assert_allclose(ml.enu_to_ned([1.0, 2.0, 3.0]), [2.0, 1.0, -3.0])
    np.testing.assert_allclose(ml.ned_to_enu(ml.enu_to_ned([1.0, 2.0, 3.0])), [1.0, 2.0, 3.0])
    # ROS identity orientation = facing east = PX4 yaw +90 deg
    w, x, y, z = ml.q_enu_to_ned_wxyz([0, 0, 0, 1])
    ypr = Rotation.from_quat([x, y, z, w]).as_euler('ZYX', degrees=True)
    np.testing.assert_allclose(ypr, [90, 0, 0], atol=1e-9)
    # facing north in ENU (yaw +90) = PX4 yaw 0; nose-up pitch stays nose-up
    q_enu = Rotation.from_euler('ZYX', [90, -10, 0], degrees=True).as_quat()   # ROS: -pitch = nose up
    w, x, y, z = ml.q_enu_to_ned_wxyz(q_enu)
    ypr = Rotation.from_quat([x, y, z, w]).as_euler('ZYX', degrees=True)
    np.testing.assert_allclose(ypr, [0, 10, 0], atol=1e-9)                     # FRD: +pitch = nose up


def test_core_lateral_conversion_agrees_with_px4_body_ned():
    """The q from MAVROS odometry gives the core the same NED velocity PX4 makes of BODY_NED."""
    q_enu = Rotation.from_euler('z', 30, degrees=True).as_quat()   # heading 30 deg left of east
    q_ned = ml.q_enu_to_ned_wxyz(q_enu)
    v_core = cl.lateral_flu_to_ned(1.0, 0.5, q_ned)
    yaw_ned = math.radians(60)                                       # 30 deg left of east = 60 from north
    vx, vy = 1.0, -0.5                                               # MAVROS FLU -> FRD
    v_px4 = [math.cos(yaw_ned) * vx - math.sin(yaw_ned) * vy, math.sin(yaw_ned) * vx + math.cos(yaw_ned) * vy]
    np.testing.assert_allclose(v_core[:2], v_px4, atol=1e-9)


# ---------------------------------------------------------------------- #
# State mapping
# ---------------------------------------------------------------------- #

@pytest.mark.parametrize('mavros, contract', [
    ('OFFBOARD', 'OFFBOARD'), ('POSCTL', 'POSCTL'), ('ALTCTL', 'ALTCTL'), ('MANUAL', 'MANUAL'),
    ('AUTO.LAND', 'AUTO_LAND'), ('AUTO.LOITER', 'AUTO_LOITER'), ('AUTO.RTL', 'AUTO_RTL'),
    ('AUTO.TAKEOFF', 'AUTO_TAKEOFF'), ('AUTO.MISSION', 'AUTO_MISSION'), ('STABILIZED', 'STAB'),
])
def test_mode_names_match_nav_state_names(mavros, contract):
    assert ml.mode_name(mavros) == contract


def test_mode_names_exist_in_px4_msgs():
    px4_msgs = pytest.importorskip('px4_msgs.msg')
    names = set(cl.nav_state_names(px4_msgs.VehicleStatus).values())
    for m in ('OFFBOARD', 'POSCTL', 'ALTCTL', 'MANUAL', 'AUTO.LAND', 'AUTO.LOITER', 'AUTO.RTL',
              'AUTO.TAKEOFF', 'AUTO.MISSION', 'STABILIZED', 'ACRO'):
        assert ml.mode_name(m) in names, m


def test_preflight_and_failsafe():
    assert ml.preflight_ok(0x10000000 | 0x3)
    assert not ml.preflight_ok(0x0FFFFFFF)
    assert ml.failsafe(5) and ml.failsafe(7)
    assert not ml.failsafe(3) and not ml.failsafe(4)      # STANDBY, ACTIVE


def test_battery_fixups():
    v, c, p = ml.battery_fixups(15.1, -9.0, 0.62)
    assert (v, c, p) == pytest.approx((15.1, -9.0, 0.62))
    v, c, p = ml.battery_fixups(0.0, float('nan'), -0.01)
    assert math.isnan(v) and math.isnan(c) and math.isnan(p)
    # the real Pixhawk 6X on USB with no battery, via MAVROS (float32 values as echoed)
    v, c, p = ml.battery_fixups(65.53500366210938, 0.009999999776482582, -0.009999999776482582)
    assert math.isnan(v) and math.isnan(c) and math.isnan(p)


def test_safe_mode_is_loiter():
    assert ml.SAFE_MODE == 'AUTO.LOITER'
    assert ml.mode_name(ml.SAFE_MODE) == 'AUTO_LOITER'


def test_constants_match_mavros_msgs():
    msg = pytest.importorskip('mavros_msgs.msg')
    P = msg.PositionTarget
    assert (P.FRAME_LOCAL_NED, P.FRAME_BODY_NED) == (ml.FRAME_LOCAL_NED, ml.FRAME_BODY_NED)
    for name in ('IGNORE_PX', 'IGNORE_PY', 'IGNORE_PZ', 'IGNORE_VX', 'IGNORE_VY', 'IGNORE_VZ',
                 'IGNORE_AFX', 'IGNORE_AFY', 'IGNORE_AFZ', 'IGNORE_YAW', 'IGNORE_YAW_RATE'):
        assert getattr(P, name) == getattr(ml, name), name


# ---------------------------------------------------------------------- #
# Console telemetry derived from MAVLink
# ---------------------------------------------------------------------- #

from types import SimpleNamespace  # noqa: E402
from px4_controller import telemetry_logic as tl  # noqa: E402


def _yaw_deg(q_xyzw):
    return Rotation.from_quat(q_xyzw).as_euler('ZYX', degrees=True)[0]


def test_odom_nwu_bench_numbers():
    """Bench 2026-11-04: DDS yaw 39.5 deg, MAVROS (ENU) 127.9 deg, same aircraft."""
    q_enu = Rotation.from_euler('ZYX', [127.9, 1.1, 9.6], degrees=True).as_quat()
    _, q_nwu, _ = ml.odom_enu_to_nwu([0.0, 0.0, 0.0], q_enu)
    assert abs(_yaw_deg(q_nwu) - 39.5) < 2.0
    assert _yaw_deg(q_nwu) == pytest.approx(127.9 - 90.0, abs=1e-6)
    e = Rotation.from_quat(q_nwu).as_euler('ZYX', degrees=True)
    assert e[1] == pytest.approx(1.1, abs=1e-6) and e[2] == pytest.approx(9.6, abs=1e-6)


def test_odom_nwu_equals_xrce_convention_exactly():
    """MAVROS ENU -> NWU gives the same pose as the XRCE path (Rx180 * q_ned * Rx180)."""
    rx = Rotation.from_euler('x', 180, degrees=True)
    q_ned = Rotation.from_euler('ZYX', [39.5, 1.2, 9.6], degrees=True) * rx   # body FRD in NED
    p_ned = np.array([4.64, 1.53, -0.68])
    # what MAVROS would publish for that pose
    q_enu = (ml._R_NED_ENU.inv() * q_ned * ml._R_FLU_FRD.inv()).as_quat()
    p_enu = ml.ned_to_enu(p_ned)
    p, q, _ = ml.odom_enu_to_nwu(p_enu, q_enu)
    want_q = (rx * q_ned * rx).as_quat()
    assert np.allclose(p, rx.apply(p_ned), atol=1e-9)
    assert np.allclose(Rotation.from_quat(q).as_matrix(), Rotation.from_quat(want_q).as_matrix(), atol=1e-9)


def test_odom_nwu_position_axes_and_cov():
    p, _, cov = ml.odom_enu_to_nwu([1.0, 2.0, 3.0], [0, 0, 0, 1], np.diag([4.0, 9.0, 16.0, 1.0, 1.0, 1.0]).ravel())
    assert np.allclose(p, [2.0, -1.0, 3.0])            # x north = ENU y, y west = -ENU x
    assert np.allclose(np.diag(cov.reshape(6, 6)), [9.0, 4.0, 16.0, 1.0, 1.0, 1.0])


EST_OK = {n: True for n in ml.ESTIMATOR_FIELDS}
EST_OK.update(const_pos_mode=False)
GYRO_OK = (ml.SENSOR_GYRO, ml.SENSOR_GYRO)


def test_failsafe_flags_none_until_estimator_heard():
    assert ml.failsafe_flag_names(None, *GYRO_OK) is None
    assert not ml.offboard_ready(None)


def test_failsafe_flags_all_ok_is_empty_and_ready():
    names = ml.failsafe_flag_names(EST_OK, *GYRO_OK)
    assert names == ()
    assert ml.offboard_ready(names)


def test_failsafe_flags_names_match_xrce_fields():
    est = dict(EST_OK, attitude=False, velocity_horiz=False, pos_horiz_rel=False,
               pos_horiz_abs=False, pred_pos_horiz_rel=False, pred_pos_horiz_abs=False)
    names = ml.failsafe_flag_names(est, ml.SENSOR_GYRO, 0)    # gyro present, unhealthy
    assert set(names) == {'attitude_invalid', 'angular_velocity_invalid', 'local_velocity_invalid',
                          'local_position_invalid_relaxed', 'global_position_invalid'}
    assert names == tuple(sorted(names))
    assert not ml.offboard_ready(names)


def test_global_position_invalid_does_not_block_offboard():
    """Flow-only indoor: no absolute position, OFFBOARD still ready (as on XRCE)."""
    est = dict(EST_OK, pos_horiz_abs=False, pred_pos_horiz_abs=False)
    names = ml.failsafe_flag_names(est, *GYRO_OK)
    assert names == ('global_position_invalid',)
    assert ml.offboard_ready(names)


def test_gyro_absent_blocks():
    assert 'angular_velocity_invalid' in ml.failsafe_flag_names(EST_OK, 0, 0)


@pytest.mark.parametrize('flag', ['attitude', 'velocity_horiz'])
def test_blocking_estimator_flag(flag):
    assert not ml.offboard_ready(ml.failsafe_flag_names(dict(EST_OK, **{flag: False}), *GYRO_OK))


def test_estimator_flags_from_message():
    msg = SimpleNamespace(**{n + '_status_flag': (n == 'attitude') for n in ml.ESTIMATOR_FIELDS})
    est = ml.estimator_flags(msg)
    assert est['attitude'] and not est['velocity_horiz']


def test_ekf_validity_mapping():
    v = ml.ekf_validity(dict(EST_OK, pos_horiz_rel=False, pos_horiz_abs=False, velocity_vert=False))
    assert v == {'xy_valid': False, 'z_valid': True, 'v_xy_valid': True, 'v_z_valid': False}
    assert ml.ekf_validity(None) is None


def test_sensor_state():
    assert ml.sensor_state(0, 0, ml.SENSOR_BATTERY) is None
    assert ml.sensor_state(ml.SENSOR_BATTERY, 0, ml.SENSOR_BATTERY) is False
    assert ml.sensor_state(ml.SENSOR_BATTERY, ml.SENSOR_BATTERY, ml.SENSOR_BATTERY) is True


def test_preflight_raw_is_prearm_bit():
    assert ml.preflight_raw(0x10000000) and not ml.preflight_raw(0x0FFFFFFF)


def test_battery_present_follows_voltage():
    _, _, _ = ml.battery_fixups(65.535, 0.01, -0.01)
    v, _, _ = ml.battery_fixups(65.535, 0.01, -0.01)
    assert not ml.battery_present(v)             # no pack: the bench state
    v, _, _ = ml.battery_fixups(16.4, -1.0, 0.8)
    assert ml.battery_present(v)


def test_range_from_altitude():
    assert ml.range_from_altitude(0.231) == {'range': pytest.approx(0.231)}
    assert math.isnan(ml.range_from_altitude(float('nan'))['range'])
    assert math.isnan(ml.range_from_altitude(-1.0)['range'])


def test_statustext_levels():
    assert [ml.statustext_log_level(s) for s in (0, 3, 4, 6)] == ['error', 'error', 'warn', 'info']


def test_navsat_no_fix_is_nan():
    f = tl.navsat_to_fields(-1, 0.0, 0.0, 0.0, [0.0] * 9, 0)
    assert f['status'] == -1 and math.isnan(f['latitude']) and math.isnan(f['altitude'])
    assert f['covariance_type'] == 0 and f['covariance'] == [0.0] * 9


def test_navsat_fix_passes_through():
    cov = [4.0, 0, 0, 0, 4.0, 0, 0, 0, 9.0]
    f = tl.navsat_to_fields(0, 47.3, 8.5, 410.0, cov, 2)
    assert (f['status'], f['latitude'], f['longitude'], f['altitude']) == (0, 47.3, 8.5, 410.0)
    assert f['covariance'] == cov and f['covariance_type'] == 2
    h = tl.navsat_health(0, 9, cov, 2)
    assert h == {'fix_type': 3, 'satellites': 9, 'hdop': 2.0}


def test_navsat_health_unknowns():
    h = tl.navsat_health(-1, None, [0.0] * 9, 0)
    assert h['fix_type'] is None and h['satellites'] is None and math.isnan(h['hdop'])


def _items(inputs, ages=None):
    return {i['name']: i for i in tl.health_items(inputs, ages or {'vehicle_status': 0.1})}


def test_health_items_mavros_unknowns_are_not_invented():
    inputs = {'ekf': ml.ekf_validity(EST_OK), 'fusion': ml.fusion_unknown(),
              'gps': tl.navsat_health(-1, None, [0.0] * 9, 0),
              'battery_mav': ml.battery_health(False, 0, 0),
              'failsafe_flags': ()}
    it = _items(inputs)
    assert list(it) == ['ekf', 'fusion', 'gps', 'battery', 'failsafe', 'link']
    assert it['ekf']['level'] == tl.OK
    assert it['fusion']['level'] == tl.STALE and it['fusion']['values']['opt_flow'] == 'unknown'
    assert it['gps']['level'] == tl.WARN and it['gps']['values']['satellites'] == 'unknown'
    assert it['battery']['level'] == tl.STALE
    assert it['failsafe']['level'] == tl.OK


def test_health_battery_mav_levels():
    ok = _items({'battery_mav': ml.battery_health(True, ml.SENSOR_BATTERY, ml.SENSOR_BATTERY)})['battery']
    bad = _items({'battery_mav': ml.battery_health(True, ml.SENSOR_BATTERY, 0)})['battery']
    assert ok['level'] == tl.OK and bad['level'] == tl.WARN


def test_health_link_carries_statustext():
    it = _items({'statustext': {'severity': 3, 'text': 'Arming denied: Mag uncalibrated'}})
    assert 'Arming denied' in it['link']['message']
    assert it['link']['values']['fc_message_severity'] == '3'
    assert 'fc_message' not in _items({})['link']['values']
