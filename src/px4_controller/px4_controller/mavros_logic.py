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
    PX4 accepts the switch), see StreamClock / control_logic.OffboardRequester.
"""
import math

import numpy as np
from scipy.spatial.transform import Rotation

from px4_controller import telemetry_logic as tl
from px4_controller.control_logic import OFFBOARD_BLOCKING_FLAGS

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
SAFE_MODE = 'AUTO.LOITER'   # HOLD after a disarm, see control_logic.DisarmGuard

# MAV_SYS_STATUS_PREARM_CHECK: PX4 v1.15 sets it in SYS_STATUS
# onboard_control_sensors_health (only there, not in "present") when it can
# arm in the current nav_state (streams/SYS_STATUS.hpp).
MAV_SYS_STATUS_PREARM_CHECK = 0x10000000
# MAV_SYS_STATUS_SENSOR_* bits of SYS_STATUS (common.xml)
SENSOR_GYRO = 0x1
SENSOR_BATTERY = 0x2000000

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


# World frame of the published odom: NWU (x north, y west, z up), the same as
# the XRCE backend (Rx180 * NED). MAVROS gives ENU, so the pose is turned by
# -90 deg about z (x_nwu = y_enu, y_nwu = -x_enu; yaw_nwu = yaw_enu - 90 deg).
_R_NWU_ENU = Rotation.from_euler('z', -90, degrees=True)


def odom_enu_to_nwu(position_enu, q_enu_xyzw, pose_covariance=None):
    """MAVROS odom pose (ENU world, body FLU) -> the console's (NWU world, body FLU).

    Returns (position, quaternion [x,y,z,w], covariance 36 or None). The body
    frame is untouched (FLU both sides), so a body-frame twist needs no
    conversion; only the world-frame position, the orientation's world side
    and the pose covariance rotate.
    """
    p = _R_NWU_ENU.apply(np.asarray(position_enu, dtype=float))
    q = (_R_NWU_ENU * Rotation.from_quat(q_enu_xyzw)).as_quat()
    cov = None
    if pose_covariance is not None:
        r = _R_NWU_ENU.as_matrix()
        big = np.kron(np.eye(2), r)       # position block and orientation block
        cov = (big @ np.asarray(pose_covariance, dtype=float).reshape(6, 6) @ big.T).ravel()
    return p, q, cov


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


class StreamClock:
    """How long the setpoint stream has run without a gap.

    PX4 accepts OFFBOARD only after setpoints have streamed for more than 1 s
    at > 2 Hz; the stream is that heartbeat, so a gap > max_gap_s restarts the
    count. Whether to request OFFBOARD is control_logic.OffboardRequester's
    decision (arm / takeoff only, same as the XRCE backend), fed with
    streaming_s(); this only measures the stream.
    """

    def __init__(self, max_gap_s=0.5):
        self.max_gap_s = max_gap_s
        self.stream_start_s = None
        self.last_sent_s = None

    def note_sent(self, now_s):
        if self.last_sent_s is None or now_s - self.last_sent_s > self.max_gap_s:
            self.stream_start_s = now_s
        self.last_sent_s = now_s

    def streaming_s(self, now_s):
        if self.last_sent_s is None or now_s - self.last_sent_s > self.max_gap_s:
            return 0.0
        return now_s - self.stream_start_s


# ---------------------------------------------------------------------- #
# State
# ---------------------------------------------------------------------- #

def mode_name(mavros_mode):
    """MAVROS PX4 mode string -> the nav_state name used by status/mode (AUTO.LAND -> AUTO_LAND)."""
    mode = str(mavros_mode)
    return _MODE_TO_NAV_STATE.get(mode, mode.replace('.', '_'))


def preflight_raw(sensors_health):
    """PX4's own "can arm in the current mode" (SYS_STATUS PREARM bit = pre_flight_checks_pass)."""
    return bool(int(sensors_health) & MAV_SYS_STATUS_PREARM_CHECK)


def preflight_ok(sensors_health):
    """Deprecated name of preflight_raw (the PREARM bit); status/preflight_ok no longer uses it."""
    return bool(int(sensors_health) & MAV_SYS_STATUS_PREARM_CHECK)


def failsafe(system_status):
    return int(system_status) in (MAV_STATE_CRITICAL, MAV_STATE_EMERGENCY, MAV_STATE_FLIGHT_TERMINATION)


# MAVLink "unknown" as MAVROS passes it on (seen from this Pixhawk 6X with no
# battery, 2026-11-03): voltage UINT16_MAX mV -> 65.535 V, current -1 cA ->
# -(-1)/100 = +0.01 A, remaining -1 % -> -0.01.
_UNKNOWN_VOLTAGE_V = 65.535
_UNKNOWN_CURRENT_A = 0.01


def battery_present(voltage):
    """A pack is present when the (fixed-up) voltage is a real number: MAVROS's `present`
    is true even with no pack, the XRCE one is BatteryStatus.connected."""
    return math.isfinite(voltage)


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


# ---------------------------------------------------------------------- #
# Console telemetry derived from MAVLink (status/health, failsafe_flags,
# gps/fix, range): see README "Console interface" parity table
# ---------------------------------------------------------------------- #

ESTIMATOR_FIELDS = ('attitude', 'velocity_horiz', 'velocity_vert', 'pos_horiz_rel',
                    'pos_horiz_abs', 'pos_vert_abs', 'pos_vert_agl', 'const_pos_mode',
                    'pred_pos_horiz_rel', 'pred_pos_horiz_abs')


def estimator_flags(msg):
    """mavros_msgs/EstimatorStatus -> {name: bool} (ESTIMATOR_STATUS_FLAGS, field *_status_flag)."""
    return {n: bool(getattr(msg, n + '_status_flag')) for n in ESTIMATOR_FIELDS}


def sensor_state(present, health, bit):
    """SYS_STATUS bit -> True (present, healthy), False (present, unhealthy), None (not present)."""
    if not int(present) & bit:
        return None
    return bool(int(health) & bit)


def failsafe_flag_names(est, sensors_present, sensors_health):
    """The PX4 failsafe_flags names derivable over MAVLink, set ones only, sorted.

    None until ESTIMATOR_STATUS has been heard. Same names as the XRCE
    FailsafeFlags fields, approximated from the estimator solution flags and
    the SYS_STATUS gyro bit:
      attitude_invalid                 not attitude
      angular_velocity_invalid         gyro not present or unhealthy
      local_velocity_invalid           not velocity_horiz
      local_position_invalid_relaxed   none of pos_horiz_rel/abs, pred_pos_horiz_rel/abs
      global_position_invalid          not pos_horiz_abs
    An absent name is NOT "false": gcs_connection_lost, home_position_invalid,
    auto_mission_missing and manual_control_signal_lost have no MAVLink source here.
    """
    if est is None:
        return None
    gyro = sensor_state(sensors_present, sensors_health, SENSOR_GYRO)
    names = []
    if not est['attitude']:
        names.append('attitude_invalid')
    if gyro is not True:
        names.append('angular_velocity_invalid')
    if not est['velocity_horiz']:
        names.append('local_velocity_invalid')
    if not (est['pos_horiz_rel'] or est['pos_horiz_abs']
            or est['pred_pos_horiz_rel'] or est['pred_pos_horiz_abs']):
        names.append('local_position_invalid_relaxed')
    if not est['pos_horiz_abs']:
        names.append('global_position_invalid')
    return tuple(sorted(names))


def offboard_ready(flag_names):
    """"Can fly OFFBOARD now" as on XRCE (control_logic.offboard_ready), minus the nav-state
    mask MAVLink does not carry: estimator heard and none of the OFFBOARD-blocking flags set."""
    if flag_names is None:
        return False
    return not (set(flag_names) & set(OFFBOARD_BLOCKING_FLAGS))


def ekf_validity(est):
    """Estimator flags -> telemetry_logic.ekf_validity()'s dict (None est: not heard).

    xy_valid: relative or absolute horizontal position; z_valid: absolute vertical
    position; v_xy_valid / v_z_valid: velocity_horiz / velocity_vert.
    """
    if est is None:
        return None
    return {'xy_valid': bool(est['pos_horiz_rel'] or est['pos_horiz_abs']),
            'z_valid': bool(est['pos_vert_abs']),
            'v_xy_valid': bool(est['velocity_horiz']),
            'v_z_valid': bool(est['velocity_vert'])}


def fusion_unknown():
    """MAVLink has no per-source fusion flags (opt_flow, rng_hgt): unknown, never a guess."""
    return {'opt_flow': None, 'rng_hgt': None}


def battery_health(present, sensors_present, sensors_health):
    """status/health 'battery_mav' input: present (BatteryState.present) and the SYS_STATUS health."""
    return {'present': bool(present),
            'healthy': sensor_state(sensors_present, sensors_health, SENSOR_BATTERY)}


def range_from_altitude(bottom_clearance):
    """mavros_msgs/Altitude.bottom_clearance (PX4: vehicle_local_position.dist_bottom while
    dist_bottom_valid, else NaN) -> Range fields, identical to the XRCE range."""
    d = tl._f(bottom_clearance)
    return tl.range_fields(d, not math.isnan(d))


# MAV_SEVERITY (mavros_msgs/StatusText) -> logger level name
def statustext_log_level(severity):
    s = int(severity)
    if s <= 3:
        return 'error'
    return 'warn' if s == 4 else 'info'
