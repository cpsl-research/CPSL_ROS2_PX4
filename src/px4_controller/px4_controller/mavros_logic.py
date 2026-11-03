"""Pure logic of the MAVROS backend (no rclpy, no mavros_msgs): see test/test_mavros_logic.py.

Frame rules (MAVROS ros2 setpoint_raw.cpp; PX4 v1.15 mavlink_receiver.cpp):

  - MAVROS takes ROS frames and converts them itself. FRAME_LOCAL_NED
    position/velocity are given in ENU and yaw as an ENU yaw; FRAME_BODY_NED
    velocity is given in body FLU. yaw_rate is always a ROS (FLU, CCW
    positive) rate. Converting here as well would flip twice.
  - PX4 drops the position of a FRAME_BODY_NED setpoint and rotates the
    velocity by yaw only, so a body velocity command never controls altitude
    by position: the altitude target is held through vz instead
    (alt_hold_climb_rate).
  - Over MAVLink there is no separate OffboardControlMode heartbeat: the
    setpoint stream itself keeps OFFBOARD alive (> 2 Hz, and > 1 s of it before
    PX4 accepts the switch), see OffboardStarter.
"""
import math

import numpy as np
from scipy.spatial.transform import Rotation

# mavros_msgs/PositionTarget constants (checked against mavros_msgs in the tests)
FRAME_LOCAL_NED = 1
FRAME_BODY_NED = 8
IGNORE_PX, IGNORE_PY, IGNORE_PZ = 1, 2, 4
IGNORE_VX, IGNORE_VY, IGNORE_VZ = 8, 16, 32
IGNORE_AFX, IGNORE_AFY, IGNORE_AFZ = 64, 128, 256
IGNORE_YAW, IGNORE_YAW_RATE = 1024, 2048
IGNORE_POS = IGNORE_PX | IGNORE_PY | IGNORE_PZ
IGNORE_VEL = IGNORE_VX | IGNORE_VY | IGNORE_VZ
IGNORE_ACC = IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ

OFFBOARD = 'OFFBOARD'
LAND = 'AUTO.LAND'

# MAV_SYS_STATUS_PREARM_CHECK: PX4 v1.15 sets it in SYS_STATUS
# onboard_control_sensors_health (only there, not in "present") when it can
# arm in the current nav_state (streams/SYS_STATUS.hpp).
MAV_SYS_STATUS_PREARM_CHECK = 0x10000000
# MAV_STATE: PX4 v1.15 heartbeat reports CRITICAL when armed with failsafe
# active, FLIGHT_TERMINATION on termination (streams/HEARTBEAT.hpp).
MAV_STATE_CRITICAL, MAV_STATE_EMERGENCY, MAV_STATE_FLIGHT_TERMINATION = 5, 6, 7

# MAVROS PX4 mode strings that differ from the nav_state names the console
# contract uses (status/mode, e.g. OFFBOARD, POSCTL, AUTO_LAND).
_MODE_TO_NAV_STATE = {'STABILIZED': 'STAB'}


# ---------------------------------------------------------------------- #
# Frames
# ---------------------------------------------------------------------- #

def enu_to_ned(v):
    """ENU vector -> NED (and back: the map is its own inverse)."""
    return np.array([v[1], v[0], -v[2]], dtype=float)


ned_to_enu = enu_to_ned

# ENU world -> NED world, and body FRD -> body FLU (both 180 deg rotations).
_R_NED_ENU = Rotation.from_matrix([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
_R_FLU_FRD = Rotation.from_euler('x', 180, degrees=True)


def q_enu_to_ned_wxyz(q_enu_xyzw):
    """ROS orientation (body FLU in ENU, [x,y,z,w]) -> PX4 q (body FRD in NED, [w,x,y,z])."""
    r = _R_NED_ENU * Rotation.from_quat(q_enu_xyzw) * _R_FLU_FRD
    x, y, z, w = r.as_quat()
    return np.array([w, x, y, z])


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_ned_to_enu(yaw_ned):
    """NED yaw (0 north, clockwise) -> ENU yaw (0 east, counter-clockwise)."""
    return wrap_pi(math.pi / 2.0 - yaw_ned)


# ---------------------------------------------------------------------- #
# Setpoints (fields of mavros_msgs/PositionTarget)
# ---------------------------------------------------------------------- #

def alt_hold_climb_rate(target_m, current_m, feed_forward_mps, max_rate_mps, gain_per_s):
    """vz (up) for a body-velocity setpoint: feed-forward plus a P term on the altitude target.

    Clamped to +-max_rate_mps. With no target or no altitude measurement only
    the (clamped) feed-forward is used.
    """
    rate = float(feed_forward_mps) if math.isfinite(feed_forward_mps) else 0.0
    if (target_m is not None and current_m is not None
            and math.isfinite(target_m) and math.isfinite(current_m)):
        rate += gain_per_s * (target_m - current_m)
    return max(-max_rate_mps, min(max_rate_mps, rate))


def _target(frame, mask, position=(0.0, 0.0, 0.0), velocity=(0.0, 0.0, 0.0), yaw=0.0, yaw_rate=0.0):
    return {
        'coordinate_frame': frame,
        'type_mask': mask,
        'position': tuple(float(v) for v in position),
        'velocity': tuple(float(v) for v in velocity),
        'yaw': float(yaw),
        'yaw_rate': float(yaw_rate),
    }


def position_target(sp, current_altitude_m, max_climb_rate_mps, alt_hold_gain_per_s):
    """control_logic.Setpoint -> PositionTarget fields, in the frames MAVROS expects.

    Lateral velocity command (sp.body_velocity_flu set): FRAME_BODY_NED with
    the FLU velocity as given, vz from alt_hold_climb_rate, ROS-sign yaw rate;
    position, acceleration and yaw ignored.

    Otherwise (hover, takeoff, yaw step): FRAME_LOCAL_NED with the position
    and velocity feed-forward in ENU and the yaw as an ENU yaw; an axis whose
    position is NaN is ignored, a NaN yaw is ignored.
    """
    yaw_rate = -float(sp.yaw_speed)          # NED rate -> ROS rate; MAVROS flips it back
    if not math.isfinite(yaw_rate):
        yaw_rate = 0.0
    if sp.body_velocity_flu is not None:
        vx, vy = (float(v) for v in sp.body_velocity_flu)
        vz = alt_hold_climb_rate(sp.target_altitude_m, current_altitude_m, sp.climb_rate_mps,
                                 max_climb_rate_mps, alt_hold_gain_per_s)
        return _target(FRAME_BODY_NED, IGNORE_POS | IGNORE_ACC | IGNORE_YAW,
                       velocity=(vx, vy, vz), yaw_rate=yaw_rate)

    # MAVROS converts the values ENU -> NED but passes type_mask through
    # unchanged, so the ignore bits name MAVLink (NED) axes.
    p_ned = np.asarray(sp.position_ned, dtype=float)
    mask = IGNORE_ACC
    for i, bit in enumerate((IGNORE_PX, IGNORE_PY, IGNORE_PZ)):
        if not math.isfinite(p_ned[i]):
            mask |= bit
    p_enu = np.nan_to_num(ned_to_enu(p_ned), nan=0.0)
    v_enu = np.nan_to_num(ned_to_enu(np.asarray(sp.velocity_ned, dtype=float)), nan=0.0)
    if math.isfinite(sp.yaw_rad):
        yaw = yaw_ned_to_enu(float(sp.yaw_rad))
    else:
        yaw, mask = 0.0, mask | IGNORE_YAW
    return _target(FRAME_LOCAL_NED, mask, position=p_enu, velocity=v_enu, yaw=yaw, yaw_rate=yaw_rate)


def hold_target(position_enu):
    """Hold a position (ENU), zero yaw rate: the stream's setpoint whenever the core has none."""
    return _target(FRAME_LOCAL_NED, IGNORE_VEL | IGNORE_ACC | IGNORE_YAW, position=position_enu)


class HoldTracker:
    """Which position the stream holds when the core is not commanding.

    Disarmed: the current position, every tick, so arming in OFFBOARD holds
    where the aircraft sits. Armed with no core command: the position latched
    when that started (a hover, not a chase of the estimate).
    """

    def __init__(self):
        self.latched = None

    def update(self, armed, commanding, current_enu):
        if current_enu is None:
            return None
        if not armed:
            self.latched = None
            return np.asarray(current_enu, dtype=float)
        if commanding:
            self.latched = None
            return None
        if self.latched is None:
            self.latched = np.asarray(current_enu, dtype=float)
        return self.latched


def stream_setpoint(armed, commanding, setpoint, hold):
    """Decide what one stream tick sends: 'setpoint', 'hold' or None (nothing to hold yet).

    A core setpoint is only sent while armed; disarmed always holds the
    current position, so a stale climb setpoint can never be what an
    arm-in-OFFBOARD meets.
    """
    if armed and commanding and setpoint is not None:
        return 'setpoint'
    return 'hold' if hold is not None else None


class OffboardStarter:
    """When to request OFFBOARD over MAVLink.

    PX4 accepts OFFBOARD only after setpoints have streamed for more than
    1 s at > 2 Hz; the stream is that heartbeat, so it must not have gaps.

      - start-up: while disarmed and not yet seen in OFFBOARD, request it every
        retry_s once streaming >= min_stream_s, at most max_attempts times.
        Seeing OFFBOARD ends this; a disarm (after being armed) re-opens it,
        so the next flight starts in OFFBOARD too. While armed nothing is
        requested on its own: a pilot's mode change is respected.
      - takeoff: an explicit takeoff requests OFFBOARD (retrying every retry_s,
        at most max_attempts) until it is seen, or land/disarm cancels it.
    """

    def __init__(self, min_stream_s=1.0, max_gap_s=0.5, retry_s=1.0, max_attempts=5):
        self.min_stream_s = min_stream_s
        self.max_gap_s = max_gap_s
        self.retry_s = retry_s
        self.max_attempts = max_attempts
        self.stream_start_s = None
        self.last_sent_s = None
        self.seen_offboard = False
        self.startup_attempts = 0
        self.takeoff_pending = False
        self.takeoff_attempts = 0
        self.last_request_s = None
        self.was_armed = False

    def note_sent(self, now_s):
        if self.last_sent_s is None or now_s - self.last_sent_s > self.max_gap_s:
            self.stream_start_s = now_s
        self.last_sent_s = now_s

    def streaming_s(self, now_s):
        if self.last_sent_s is None or now_s - self.last_sent_s > self.max_gap_s:
            return 0.0
        return now_s - self.stream_start_s

    def request_takeoff(self):
        self.takeoff_pending = True
        self.takeoff_attempts = 0

    def cancel_takeoff(self):
        self.takeoff_pending = False

    def poll(self, now_s, connected, armed, mode):
        """Return 'startup', 'takeoff' or None: whether to request OFFBOARD now."""
        if self.was_armed and not armed:
            self.seen_offboard = False
            self.startup_attempts = 0
            self.takeoff_pending = False
        self.was_armed = armed
        if mode == OFFBOARD:
            self.seen_offboard = True
            self.takeoff_pending = False
            return None
        if not connected or self.streaming_s(now_s) < self.min_stream_s:
            return None
        if self.last_request_s is not None and now_s - self.last_request_s < self.retry_s:
            return None
        if self.takeoff_pending and armed:
            if self.takeoff_attempts >= self.max_attempts:
                self.takeoff_pending = False
                return None
            self.takeoff_attempts += 1
            self.last_request_s = now_s
            return 'takeoff'
        if not armed and not self.seen_offboard and self.startup_attempts < self.max_attempts:
            self.startup_attempts += 1
            self.last_request_s = now_s
            return 'startup'
        return None


# ---------------------------------------------------------------------- #
# State
# ---------------------------------------------------------------------- #

def mode_name(mavros_mode):
    """MAVROS PX4 mode string -> the nav_state name used by status/mode (AUTO.LAND -> AUTO_LAND)."""
    mode = str(mavros_mode)
    return _MODE_TO_NAV_STATE.get(mode, mode.replace('.', '_'))


def preflight_ok(sensors_health):
    return bool(int(sensors_health) & MAV_SYS_STATUS_PREARM_CHECK)


def failsafe(system_status):
    return int(system_status) in (MAV_STATE_CRITICAL, MAV_STATE_EMERGENCY, MAV_STATE_FLIGHT_TERMINATION)


# MAVLink "unknown" as MAVROS passes it on (seen from this Pixhawk 6X with no
# battery, 2026-11-03): voltage UINT16_MAX mV -> 65.535 V, current -1 cA ->
# -(-1)/100 = +0.01 A, remaining -1 % -> -0.01.
_UNKNOWN_VOLTAGE_V = 65.535
_UNKNOWN_CURRENT_A = 0.01


def battery_fixups(voltage, current, percentage):
    """MAVROS BatteryState fields -> the contract's: unknown is NaN, percentage 0..1.

    MAVROS already reports discharge current as negative (ROS convention),
    but passes MAVLink's unknown sentinels through as numbers.
    """
    nan = float('nan')
    v = float(voltage)
    c = float(current)
    p = float(percentage)
    v_ok = math.isfinite(v) and 0.0 < v and abs(v - _UNKNOWN_VOLTAGE_V) > 1e-3
    c_ok = math.isfinite(c) and abs(c - _UNKNOWN_CURRENT_A) > 1e-6
    p_ok = math.isfinite(p) and 0.0 <= p <= 1.0
    return (v if v_ok else nan, c if c_ok else nan, p if p_ok else nan)
