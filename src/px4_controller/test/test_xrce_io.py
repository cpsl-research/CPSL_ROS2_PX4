"""XrceIO request timing and preflight_ok, against a stub core (needs px4_msgs, no DDS)."""
import os
import sys
from types import SimpleNamespace

import pytest

pytest.importorskip('px4_msgs')
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from px4_msgs.msg import FailsafeFlags, VehicleStatus  # noqa: E402
from px4_controller import control_logic as cl  # noqa: E402
from px4_controller import xrce_io  # noqa: E402


class Pub:
    def __init__(self):
        self.sent = []

    def publish(self, m):
        self.sent.append(m)


class Core:
    def __init__(self):
        self.states = []
        self.health = {}
        self.gps = self.range = None
        self.vehicle_state = cl.VehicleState(False, '', False, False)
        self.current_position_ned = [0.0, 0.0, 0.0]
        self.clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1000))

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return SimpleNamespace(info=lambda *a: None, error=lambda *a: None)

    def on_gps_fix(self, fields, health):
        self.gps = (fields, health)

    def on_range(self, fields):
        self.range = fields

    def on_health_input(self, name, value):
        self.health[name] = value

    def on_vehicle_state(self, s):
        self.vehicle_state = s
        self.states.append(s)


def _io(monkeypatch):
    io = object.__new__(xrce_io.XrceIO)
    io.core = Core()
    io.offboard_control_mode_publisher = Pub()
    io.trajectory_setpoint_publisher = Pub()
    io.vehicle_command_publisher = Pub()
    io.nav_state_names = cl.nav_state_names(VehicleStatus)
    io.last_status = None
    io.flag_names = None
    io.requester = cl.OffboardRequester()
    io.guard = cl.DisarmGuard()
    io.prestream = False
    io.stream_since_s = None
    io.armed_seen = False
    io.active_command = None
    io.active_command_timer = None
    clock = SimpleNamespace(t=100.0)
    monkeypatch.setattr(xrce_io.time, 'monotonic', lambda: clock.t)
    return io, clock


def _status(armed=False, nav=VehicleStatus.NAVIGATION_STATE_POSCTL, mask=0xFFFFFFFF, raw=False):
    s = VehicleStatus()
    s.arming_state = VehicleStatus.ARMING_STATE_ARMED if armed else VehicleStatus.ARMING_STATE_DISARMED
    s.nav_state = nav
    s.can_set_nav_states_mask = mask
    s.pre_flight_checks_pass = raw
    return s


def _cmds(io):
    return [(m.command, m.param1) for m in io.vehicle_command_publisher.sent]


def _run(io, clock, seconds, status=None):
    for _ in range(int(seconds / 0.2)):
        clock.t += 0.2
        if status is not None:
            io.core.vehicle_state = io.core.vehicle_state   # unchanged
        io.publish_offboard_control()


def test_preflight_ok_is_offboard_ready_not_px4_flag(monkeypatch):
    io, _ = _io(monkeypatch)
    io.vehicle_status_callback(_status(raw=False))
    assert io.core.states[-1].preflight_ok is False          # no failsafe_flags yet
    f = FailsafeFlags()
    f.home_position_invalid = True
    f.manual_control_signal_lost = True
    io.failsafe_flags_callback(f)
    s = io.core.states[-1]
    assert s.preflight_ok is True and s.preflight_raw is False
    assert s.failsafe_flags == ('home_position_invalid', 'manual_control_signal_lost')
    f.attitude_invalid = True
    io.failsafe_flags_callback(f)
    assert io.core.states[-1].preflight_ok is False


def test_no_offboard_request_at_start_while_disarmed(monkeypatch):
    io, clock = _io(monkeypatch)
    io.vehicle_status_callback(_status())
    io.failsafe_flags_callback(FailsafeFlags())
    _run(io, clock, 10.0)
    assert _cmds(io) == [] and io.offboard_control_mode_publisher.sent   # heartbeat only


def test_arm_streams_then_offboard_then_arm(monkeypatch):
    io, clock = _io(monkeypatch)
    io.vehicle_status_callback(_status())
    io.failsafe_flags_callback(FailsafeFlags())
    assert io.arm() is True
    _run(io, clock, 0.8)
    assert _cmds(io) == [] and io.trajectory_setpoint_publisher.sent   # stream < 1 s: no request
    _run(io, clock, 1.0)
    assert _cmds(io) == [(176, 1.0)]                                   # DO_SET_MODE offboard
    io.vehicle_status_callback(_status(nav=VehicleStatus.NAVIGATION_STATE_OFFBOARD))
    _run(io, clock, 0.4)
    cmds = _cmds(io)
    assert cmds[-1] == (VehicleStatusArm, 1.0) and len(cmds) == 2


VehicleStatusArm = 400   # VEHICLE_CMD_COMPONENT_ARM_DISARM


def test_arm_without_odometry_does_not_start(monkeypatch):
    io, clock = _io(monkeypatch)
    io.core.current_position_ned = None
    assert io.arm() is False
    _run(io, clock, 3.0)
    assert _cmds(io) == []


def test_disarm_stops_the_stream(monkeypatch):
    io, clock = _io(monkeypatch)
    io.vehicle_status_callback(_status())
    io.failsafe_flags_callback(FailsafeFlags())
    io.arm()
    _run(io, clock, 0.4)
    n = len(io.trajectory_setpoint_publisher.sent)
    io.disarm()
    _run(io, clock, 2.0)
    assert len(io.trajectory_setpoint_publisher.sent) == n


# ---- odometry conversion (frame audit: docs/research/px4_frame_conventions.md) ----
import numpy as np  # noqa: E402
from builtin_interfaces.msg import Time  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402


def _odom(q_wxyz=(1.0, 0.0, 0.0, 0.0), pos=(0.0, 0.0, 0.0), vel=(0.0, 0.0, 0.0),
          omega=(0.0, 0.0, 0.0)):
    return SimpleNamespace(q=list(q_wxyz), position=list(pos), velocity=list(vel),
                           angular_velocity=list(omega))


def _convert(o, monkeypatch=None):
    io = object.__new__(xrce_io.XrceIO)
    io.core = Core()
    io.core.clock = SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    return io.convert_px4_odometry_to_nav2(o)


def _q_ned(yaw=0.0, pitch=0.0, roll=0.0):
    x, y, z, w = Rotation.from_euler('ZYX', [yaw, pitch, roll], degrees=True).as_quat()
    return (w, x, y, z)


def _rot_out(m):
    o = m.pose.pose.orientation
    return Rotation.from_quat([o.x, o.y, o.z, o.w])


def test_odom_level_body_z_up_and_rpy_zero():
    r = _rot_out(_convert(_odom()))
    np.testing.assert_allclose(r.apply([0, 0, 1]), [0, 0, 1], atol=1e-9)
    np.testing.assert_allclose(r.apply([1, 0, 0]), [1, 0, 0], atol=1e-9)  # facing north = world x
    np.testing.assert_allclose(r.as_euler('xyz', degrees=True), [0, 0, 0], atol=1e-9)


def test_odom_yaw_30_heading_sign_matches_position():
    # NED yaw +30 (north toward east) is a -30 turn in the x-north/y-west/z-up world
    r = _rot_out(_convert(_odom(_q_ned(yaw=30))))
    np.testing.assert_allclose(r.apply([0, 0, 1]), [0, 0, 1], atol=1e-9)
    np.testing.assert_allclose(r.as_euler('xyz', degrees=True), [0, 0, -30], atol=1e-9)
    # forward axis points where a NED (cos30, sin30, 0) displacement lands in the world
    p = _convert(_odom(pos=(np.cos(np.radians(30)), np.sin(np.radians(30)), 0)))
    fwd = r.apply([1, 0, 0])
    np.testing.assert_allclose(
        fwd, [p.pose.pose.position.x, p.pose.pose.position.y, p.pose.pose.position.z], atol=1e-9)


def test_odom_tilted_body_axes():
    # roll 20 deg in NED (right wing down); body +z
    q = _q_ned(yaw=0, roll=20)
    r = _rot_out(_convert(_odom(q)))
    # reference: map the FLU body axes through FRD -> NED (q) -> NWU by hand
    flip = np.diag([1, -1, -1])
    r_ned = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
    np.testing.assert_allclose(r.as_matrix(), flip @ r_ned @ flip, atol=1e-9)
    assert r.apply([0, 0, 1])[2] == pytest.approx(np.cos(np.radians(20)))
    np.testing.assert_allclose(r.as_euler('xyz', degrees=True), [20, 0, 0], atol=1e-9)


def test_odom_bench_sample_quaternion():
    q = (0.648, 0.045, -0.071, -0.757)
    r = _rot_out(_convert(_odom(q)))
    assert r.apply([0, 0, 1])[2] > 0.95   # level aircraft reads right side up


def test_odom_position_and_velocity_unchanged_table():
    m = _convert(_odom(pos=(1, 0, -1), vel=(1, 0, 0)))
    p = m.pose.pose.position
    np.testing.assert_allclose([p.x, p.y, p.z], [1, 0, 1], atol=1e-12)
    m = _convert(_odom(pos=(0, 1, -1)))
    p = m.pose.pose.position
    np.testing.assert_allclose([p.x, p.y, p.z], [0, -1, 1], atol=1e-12)
    # level, facing north, flying north at 1 m/s: body FLU forward
    v = _convert(_odom(vel=(1, 0, 0))).twist.twist.linear
    np.testing.assert_allclose([v.x, v.y, v.z], [1, 0, 0], atol=1e-12)


def test_odom_body_rate_is_not_rotated_by_attitude():
    # omega_frd (0.1,0,0) at yaw 90 stays (0.1,0,0) in FLU; z flips sign
    w = _convert(_odom(_q_ned(yaw=90), omega=(0.1, 0.0, 0.0))).twist.twist.angular
    np.testing.assert_allclose([w.x, w.y, w.z], [0.1, 0, 0], atol=1e-12)
    w = _convert(_odom(_q_ned(yaw=90, roll=10), omega=(0.0, 0.0, 0.2))).twist.twist.angular
    np.testing.assert_allclose([w.x, w.y, w.z], [0, 0, -0.2], atol=1e-12)


def test_footprint_level_after_fixed_base_link():
    # base_link from the fixed conversion (tilted + yawed bench-like attitude) -> footprint is
    # a level, yaw-only frame at ground: z drop, no 180 deg roll
    m = _convert(_odom(_q_ned(yaw=-112, pitch=1.4, roll=9.6), pos=(0.9, -0.75, -1.8)))
    r_ob = _rot_out(m)
    p_ob = np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
    t_bf, q_bf = cl.base_link_to_footprint(p_ob, r_ob.as_quat())
    r_of = r_ob * Rotation.from_quat(q_bf)
    np.testing.assert_allclose(p_ob + r_ob.apply(t_bf), [p_ob[0], p_ob[1], 0.0], atol=1e-9)
    rpy = r_of.as_euler('xyz', degrees=True)
    np.testing.assert_allclose(rpy[:2], [0, 0], atol=1e-9)
    assert rpy[2] == pytest.approx(r_ob.as_euler('xyz', degrees=True)[2])
    # level aircraft: footprint relative to base_link is a pure z drop, identity rotation
    t, q = cl.base_link_to_footprint([0.9, 0.75, 1.8], _rot_out(_convert(_odom())).as_quat())
    np.testing.assert_allclose(t, [0, 0, -1.8], atol=1e-9)
    np.testing.assert_allclose(Rotation.from_quat(q).magnitude(), 0, atol=1e-9)


def test_telemetry_callbacks_decode_to_core_hooks(monkeypatch):
    from px4_msgs.msg import SensorGps, VehicleLocalPosition, EstimatorStatusFlags, BatteryStatus
    io, _ = _io(monkeypatch)
    g = SensorGps()
    g.fix_type = 3
    g.latitude_deg, g.longitude_deg, g.altitude_ellipsoid_m = 32.0, -117.0, 10.0
    g.satellites_used, g.hdop, g.eph, g.epv = 9, 1.1, 0.4, 0.6
    io.gps_callback(g)
    fields, health = io.core.gps
    assert fields['status'] == 0 and fields['latitude'] == 32.0
    assert health == {'fix_type': 3, 'satellites': 9, 'hdop': pytest.approx(1.1)}
    lp = VehicleLocalPosition()
    lp.dist_bottom, lp.dist_bottom_valid = 0.8, True
    lp.xy_valid, lp.z_valid = True, True
    io.local_position_callback(lp)
    assert io.core.range == {'range': pytest.approx(0.8)}
    assert io.core.health['ekf'] == {'xy_valid': True, 'z_valid': True,
                                     'v_xy_valid': False, 'v_z_valid': False}
    lp.dist_bottom_valid = False
    io.local_position_callback(lp)
    assert io.core.range['range'] != io.core.range['range']          # NaN
    fl = EstimatorStatusFlags()
    fl.cs_opt_flow = True
    io.estimator_flags_callback(fl)
    assert io.core.health['fusion'] == {'opt_flow': True, 'rng_hgt': False}
    f = FailsafeFlags()
    f.home_position_invalid = True
    io.failsafe_flags_callback(f)
    assert io.core.health['failsafe_flags'] == ('home_position_invalid',)
    io.core.on_battery = lambda *a: None
    b = BatteryStatus()
    b.warning = 1
    io.battery_status_callback(b)
    assert io.core.health['battery_warning'] == 1


def test_disarm_in_offboard_requests_auto_loiter(monkeypatch):
    io, clock = _io(monkeypatch)
    io.core.vehicle_state = cl.VehicleState(True, 'OFFBOARD', True, False)
    _run(io, clock, 0.4)
    assert _cmds(io) == []
    io.core.vehicle_state = cl.VehicleState(False, 'OFFBOARD', True, False)
    _run(io, clock, 0.4)
    m = io.vehicle_command_publisher.sent[0]
    assert (m.command, m.param1, m.param2, m.param3) == (176, 1.0, 4.0, 3.0)   # DO_SET_MODE AUTO/LOITER
    io.core.vehicle_state = cl.VehicleState(False, 'AUTO_LOITER', True, False)
    n = len(io.vehicle_command_publisher.sent)
    _run(io, clock, 3.0)
    assert len(io.vehicle_command_publisher.sent) == n
