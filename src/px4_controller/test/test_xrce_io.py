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
        self.vehicle_state = cl.VehicleState(False, '', False, False)
        self.current_position_ned = [0.0, 0.0, 0.0]
        self.clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1000))

    def get_clock(self):
        return self.clock

    def get_logger(self):
        return SimpleNamespace(info=lambda *a: None, error=lambda *a: None)

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
