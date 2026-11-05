"""Pure command-path logic for px4_control_node (no rclpy, no px4_msgs).

Kept separate so the safety-relevant parts (limits, command timeout,
altitude integration, frame maths) can be unit tested without ROS or an
aircraft: see test/test_control_logic.py.
"""
import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

# Parameter names read by the node. Defaults are the console.launch.py
# defaults, so launches that predate the parameters (control_only_launch.py,
# joy_control_launch.py, keyop_control_launch.py) still start the node; any
# launch that passes them overrides these.
LIMIT_DEFAULTS = {
    'takeoff_altitude_m': 0.75,
    'min_altitude_m': 0.5,
    'max_altitude_m': 2.5,
    'max_climb_rate_mps': 0.3,
    'cmd_timeout_s': 0.5,
}


@dataclass(frozen=True)
class Limits:
    takeoff_altitude_m: float
    min_altitude_m: float
    max_altitude_m: float
    max_climb_rate_mps: float
    cmd_timeout_s: float

    def validate(self):
        """Raise ValueError unless 0 < min < takeoff <= max, rate > 0, timeout > 0.

        takeoff must be strictly above min: takeoff completes when the
        measured altitude reaches min_altitude_m while the setpoint is
        takeoff_altitude_m, so the setpoint must lie above that threshold.
        """
        vals = [self.takeoff_altitude_m, self.min_altitude_m, self.max_altitude_m,
                self.max_climb_rate_mps, self.cmd_timeout_s]
        if not all(math.isfinite(v) for v in vals):
            raise ValueError('altitude/rate/timeout limits must be finite: {}'.format(self))
        if not 0.0 < self.min_altitude_m < self.takeoff_altitude_m <= self.max_altitude_m:
            raise ValueError(
                'need 0 < min_altitude_m < takeoff_altitude_m <= max_altitude_m, got '
                'min={} takeoff={} max={}'.format(
                    self.min_altitude_m, self.takeoff_altitude_m, self.max_altitude_m))
        if self.max_climb_rate_mps <= 0.0:
            raise ValueError('max_climb_rate_mps must be > 0, got {}'.format(
                self.max_climb_rate_mps))
        if self.cmd_timeout_s <= 0.0:
            raise ValueError('cmd_timeout_s must be > 0, got {}'.format(self.cmd_timeout_s))
        return self


@dataclass
class Setpoint:
    """One setpoint from the core, before any transport encodes it.

    The first four fields are PX4's TrajectorySetpoint in the vehicle frame
    (local NED, body FRD), exactly what the XRCE backend sends. The rest
    describe the same command in the console's terms, for a backend that
    must not convert frames itself (MAVROS converts ENU/FLU to NED/FRD).

    position_ned       m, NaN component = not controlled on that axis
    velocity_ned       m/s, feed-forward (or the command when position is NaN)
    yaw_rad            NED yaw (0 north, positive clockwise from above); NaN = not controlled
    yaw_speed          NED yaw rate, rad/s (= -angular.z of the FLU command)
    body_velocity_flu  (vx forward, vy left) m/s when the setpoint is a lateral
                       velocity command; None for position setpoints
    climb_rate_mps     clamped climb rate actually applied (up positive)
    target_altitude_m  altitude target, metres up; NaN when none
    caller             who produced it (log text)
    """
    position_ned: np.ndarray
    velocity_ned: np.ndarray
    yaw_rad: float = float('nan')
    yaw_speed: float = 0.0
    body_velocity_flu: tuple = None
    climb_rate_mps: float = 0.0
    target_altitude_m: float = float('nan')
    caller: str = 'N/A'


@dataclass(frozen=True)
class VehicleState:
    """Backend-neutral vehicle state behind status/* (one tuple per update)."""
    armed: bool
    mode: str
    preflight_ok: bool
    failsafe: bool
    # Extra state (additive; backends that cannot report it leave the defaults):
    # the names of the PX4 failsafe_flags that are set, and PX4's own
    # pre_flight_checks_pass (None = not reported).
    failsafe_flags: tuple = ()
    preflight_raw: object = None

    def as_tuple(self):
        return (self.armed, self.mode, self.preflight_ok, self.failsafe,
                tuple(self.failsafe_flags), self.preflight_raw)


# --- "can fly OFFBOARD now" (XRCE backend) -----------------------------------

OFFBOARD_NAV_STATE = 14   # VehicleStatus.NAVIGATION_STATE_OFFBOARD

# failsafe_flags that stop OFFBOARD from being entered or flown. global_position,
# home_position, mission and manual_control (RC) are deliberately not here, and
# offboard_control_signal_lost only holds while no setpoints stream.
OFFBOARD_BLOCKING_FLAGS = (
    'attitude_invalid', 'angular_velocity_invalid',
    'local_position_invalid_relaxed', 'local_velocity_invalid',
)


def set_failsafe_flag_names(flags):
    """Names of the true bool fields of a FailsafeFlags message (or a dict), sorted.

    Non-bool fields (timestamp, mode_req_* masks, battery_warning) are skipped.
    """
    if isinstance(flags, dict):
        items = flags.items()
    else:
        fields = flags.get_fields_and_field_types()
        items = ((n, getattr(flags, n)) for n, t in fields.items() if t == 'boolean')
    return tuple(sorted(n for n, v in items if v is True or (isinstance(v, (bool, np.bool_)) and bool(v))))


def offboard_ready(can_set_nav_states_mask, flag_names):
    """True when OFFBOARD can be flown now: PX4 lets the user select it and none of
    OFFBOARD_BLOCKING_FLAGS is set. `flag_names` None (no failsafe_flags heard yet)
    is not ready."""
    if flag_names is None:
        return False
    if not (int(can_set_nav_states_mask) >> OFFBOARD_NAV_STATE) & 1:
        return False
    return not (set(flag_names) & set(OFFBOARD_BLOCKING_FLAGS))


class OffboardRequester:
    """When the XRCE driver may request OFFBOARD (VEHICLE_CMD_DO_SET_MODE).

    Never on its own at start-up: only for an arm request, or while a take-off
    is pending, and only while preflight_ok (offboard_ready), the setpoint
    stream has run > min_stream_s, and PX4 is not already in OFFBOARD.

    Arm sequence: arm() -> if PX4 is already in OFFBOARD the arm command goes at
    once; otherwise OFFBOARD is requested first (retry every retry_s, at most
    max_attempts) and the arm command follows when OFFBOARD is seen. After
    timeout_s without arming the sequence is dropped. Take-off: start_takeoff()
    requests OFFBOARD the same way until seen. poll() returns the action to take
    now: 'offboard', 'arm', 'safe' or None.

    Failed arm: if the arm sequence itself requested OFFBOARD (the operator was
    not already in it) and the vehicle is not armed arm_confirm_s after the arm
    command was sent, or the sequence was dropped (timeout / attempts), the FC
    would be left in OFFBOARD while disarmed (DisarmGuard only fires on an armed
    true -> false edge). poll() then returns 'safe' (request SAFE_MODE: AUTO.LOITER),
    retried every rollback_retry_s, at most rollback_max_attempts, while the mode
    is still OFFBOARD. A mode the operator set is never touched, nor is anything
    after the vehicle armed, a new sequence started, or cancel() (disarm/land).
    """

    def __init__(self, min_stream_s=1.0, retry_s=1.0, max_attempts=5, timeout_s=15.0,
                 arm_confirm_s=5.0, rollback_retry_s=1.0, rollback_max_attempts=3):
        self.min_stream_s = min_stream_s
        self.retry_s = retry_s
        self.max_attempts = max_attempts
        self.timeout_s = timeout_s
        self.arm_confirm_s = arm_confirm_s
        self.rollback_retry_s = rollback_retry_s
        self.rollback_max_attempts = rollback_max_attempts
        self.reset()

    def reset(self):
        self.node_offboard = False    # this node requested OFFBOARD for the arm sequence
        self.arm_sent_s = None        # when the arm command was last returned
        self.rollback = False         # a failed arm left (or may leave) OFFBOARD to undo
        self.rollback_attempts = 0
        self.rollback_last_s = None
        self.arming = False
        self.takeoff = False
        self.started_s = None
        self.attempts = 0
        self.last_request_s = None
        self.was_armed = False

    @property
    def active(self):
        return self.arming or self.takeoff

    def start_arm(self, now_s):
        self.arming = True
        self.node_offboard = False
        self.arm_sent_s = None
        self._begin(now_s)

    def start_takeoff(self, now_s):
        self.takeoff = True
        self._begin(now_s)

    def _begin(self, now_s):
        if self.started_s is None:
            self.started_s = now_s
        self.attempts = 0
        self.last_request_s = None
        self.rollback = False         # a new sequence wants OFFBOARD

    def cancel(self):
        self.reset()

    def poll(self, now_s, mode, armed, preflight_ok, streaming_s):
        if self.was_armed and not armed:
            self.reset()
        self.was_armed = armed
        if armed:                        # the arm worked: nothing to undo
            self.node_offboard = False
            self.arm_sent_s = None
            self.rollback = False
        action = self._poll_sequence(now_s, mode, armed, preflight_ok, streaming_s)
        if action is not None:
            return action
        return self._poll_rollback(now_s, mode, armed)

    def _abandon(self, armed):
        """Drop the sequence; a dropped arm that we put into OFFBOARD is rolled back."""
        failed = self.arming and not armed and self.node_offboard
        self.reset()
        self.was_armed = armed
        self.rollback = failed

    def _poll_rollback(self, now_s, mode, armed):
        if (self.arm_sent_s is not None and not armed
                and now_s - self.arm_sent_s > self.arm_confirm_s):
            self.arm_sent_s = None       # arm not reflected: refused or lost
            self.rollback = self.node_offboard
            self.node_offboard = False
        if not self.rollback:
            return None
        if armed or self.active or mode != 'OFFBOARD':
            self.rollback = False
            return None
        if self.rollback_attempts >= self.rollback_max_attempts:
            return None
        if (self.rollback_last_s is not None
                and now_s - self.rollback_last_s < self.rollback_retry_s):
            return None
        self.rollback_attempts += 1
        self.rollback_last_s = now_s
        return 'safe'

    def _poll_sequence(self, now_s, mode, armed, preflight_ok, streaming_s):
        if not self.active:
            return None
        if armed:
            self.arming = False          # armed: nothing left to arm
        if self.started_s is not None and now_s - self.started_s > self.timeout_s:
            self._abandon(armed)
            return None
        if mode == 'OFFBOARD':
            if self.arming and not armed:
                self.arming = False
                self.arm_sent_s = now_s
                self._finish_if_idle()
                return 'arm'
            self.takeoff = False
            self._finish_if_idle()
            return None
        if not preflight_ok or streaming_s < self.min_stream_s:
            return None
        if self.last_request_s is not None and now_s - self.last_request_s < self.retry_s:
            return None
        if self.attempts >= self.max_attempts:
            self._abandon(armed)
            return None
        self.attempts += 1
        self.last_request_s = now_s
        if self.arming:
            self.node_offboard = True
        return 'offboard'

    def _finish_if_idle(self):
        if not self.active:
            self.started_s = None


# After a disarm the FC must not be left in OFFBOARD. The safe mode is HOLD:
# PX4 AUTO_LOITER (nav_state 4 = VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,
# the XRCE DO_SET_MODE custom main mode 4 AUTO / sub mode 3 LOITER; MAVROS
# custom mode 'AUTO.LOITER'). The component had no hold mode before; this is
# the same "hold" PX4 itself falls back to on an OFFBOARD loss. PX4 refuses it
# without a valid position estimate, so requests are bounded, never forced.
SAFE_NAV_STATE = 4
SAFE_MAIN_MODE, SAFE_SUB_MODE = 4.0, 3.0   # PX4_CUSTOM_MAIN_MODE_AUTO, _SUB_MODE_AUTO_LOITER


class DisarmGuard:
    """Leave OFFBOARD after a disarm (either backend).

    poll(now_s, mode, armed, suppress) returns True when a request for the safe
    mode should be sent now: armed went true -> false and the mode is still
    OFFBOARD. Retried every retry_s, at most max_attempts, until the mode is no
    longer OFFBOARD or the vehicle arms again. `suppress` (an arm or takeoff
    sequence is active and wants OFFBOARD) drops a pending request.
    """

    def __init__(self, retry_s=1.0, max_attempts=3):
        self.retry_s = retry_s
        self.max_attempts = max_attempts
        self.was_armed = False
        self.pending = False
        self.attempts = 0
        self.last_request_s = None

    def poll(self, now_s, mode, armed, suppress=False):
        if self.was_armed and not armed:
            self.pending = True
            self.attempts = 0
            self.last_request_s = None
        self.was_armed = armed
        if armed or suppress or mode != 'OFFBOARD':
            self.pending = False
            return False
        if not self.pending or self.attempts >= self.max_attempts:
            return False
        if self.last_request_s is not None and now_s - self.last_request_s < self.retry_s:
            return False
        self.attempts += 1
        self.last_request_s = now_s
        return True


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def clamp_climb_rate(vz, max_rate):
    """linear.z (m/s, up) clamped to +-max_rate; NaN/inf command 0."""
    vz = float(vz)
    if not math.isfinite(vz):
        return 0.0
    return clamp(vz, -max_rate, max_rate)


def integrate_altitude(target_m, vz, dt_s, limits, max_dt_s):
    """Integrate a climb-rate command into the altitude target.

    Args:
        target_m: current altitude target, metres up from the odom origin.
        vz: commanded climb rate (linear.z, FLU, m/s up), clamped to
            +-limits.max_climb_rate_mps.
        dt_s: time since the previous integration step; clamped to
            [0, max_dt_s] so a stalled timer cannot produce a jump.
        limits: Limits; the result stays in [min_altitude_m, max_altitude_m].

    Returns:
        (new_target_m, applied_rate_mps): applied_rate is the rate actually
        realised after the altitude clamp (0 at a limit), used as the
        velocity feed-forward.
    """
    rate = clamp_climb_rate(vz, limits.max_climb_rate_mps)
    dt = clamp(float(dt_s), 0.0, max_dt_s) if math.isfinite(dt_s) else 0.0
    new_target = clamp(target_m + rate * dt, limits.min_altitude_m, limits.max_altitude_m)
    applied = (new_target - target_m) / dt if dt > 0.0 else 0.0
    return new_target, applied


def is_fresh(rx_time_s, now_s, timeout_s):
    """True iff a message received at rx_time_s (monotonic) is <= timeout_s old."""
    if rx_time_s is None:
        return False
    age = now_s - rx_time_s
    return 0.0 <= age <= timeout_s


def rate_limit(now_s, next_due_s, period_s):
    """Rate limiter for events driven by an incoming stream (tf from odometry).

    Returns (send, new_next_due_s). The due time advances by period_s from
    the previous due time rather than from now, so jittery input at a rate
    close to 1/period_s is not halved; after a gap longer than one period it
    re-anchors to now.
    """
    if next_due_s is not None and now_s < next_due_s:
        return False, next_due_s
    base = now_s if next_due_s is None or now_s - next_due_s > period_s else next_due_s
    return True, base + period_s


# select_command results
HOVER_NOT_FLYING = 'hover: not flying'
HOVER_DEADMAN_RELEASED = 'hover: deadman not pressed'
HOVER_DEADMAN_STALE = 'hover: deadman_pressed older than cmd_timeout_s'
HOVER_NAV_STALE = 'hover: cmd_vel_nav missing or older than cmd_timeout_s'
HOVER_MANUAL_STALE = 'hover: cmd_vel missing or older than cmd_timeout_s'
USE_NAV = 'nav'
USE_MANUAL = 'manual'


def select_command(flying, deadman_pressed, deadman_fresh, allow_nav,
                   cmd_vel_fresh, cmd_vel_nav_fresh):
    """Which velocity source drives the aircraft this tick, or why it hovers."""
    if not flying:
        return HOVER_NOT_FLYING
    if not deadman_pressed:
        return HOVER_DEADMAN_RELEASED
    if not deadman_fresh:
        return HOVER_DEADMAN_STALE
    if allow_nav:
        return USE_NAV if cmd_vel_nav_fresh else HOVER_NAV_STALE
    return USE_MANUAL if cmd_vel_fresh else HOVER_MANUAL_STALE


def lateral_flu_to_ned(vx, vy, q_ned_wxyz):
    """Body FLU lateral velocity (x fwd, y left) to NED velocity, z dropped.

    linear.z is excluded before rotating, so a climb command never leaks
    into the horizontal setpoint when the aircraft is tilted; altitude is
    handled by integrate_altitude.
    """
    frd = np.array([vx, -vy, 0.0])  # FLU -> FRD (rotate 180 deg about x)
    q = q_ned_wxyz
    r = Rotation.from_quat([q[1], q[2], q[3], q[0]])
    ned = r.apply(frd)
    return np.array([ned[0], ned[1], 0.0])


def nav_state_names(msg_cls):
    """{nav_state value: 'OFFBOARD', 'POSCTL', ...} from VehicleStatus' constants.

    rosidl exposes message constants as properties of the metaclass, so they
    are looked up on type(msg_cls) as well as on the class itself.
    """
    prefix = 'NAVIGATION_STATE_'
    names = {}
    for attr in sorted(set(dir(msg_cls)) | set(dir(type(msg_cls)))):
        if attr.startswith(prefix) and attr != prefix + 'MAX':
            names.setdefault(int(getattr(msg_cls, attr)), attr[len(prefix):])
    return names


def nav_state_name(value, names):
    """nav_state value -> name from nav_state_names(); 'UNKNOWN(<n>)' otherwise."""
    return names.get(int(value), 'UNKNOWN({})'.format(value))


def base_link_to_footprint(t_odom_base, q_odom_base_xyzw):
    """Transform base_link -> base_footprint, given odom -> base_link.

    base_footprint is the base_link position projected onto the odom z=0
    plane with yaw only (roll/pitch removed); expressed as a child of
    base_link so base_link keeps the single parent odom (TF_AND_TOPICS §4).

    Returns (translation[3], quaternion_xyzw[4]).
    """
    r_ob = Rotation.from_quat(q_odom_base_xyzw)
    yaw = r_ob.as_euler('xyz')[2]
    r_of = Rotation.from_euler('z', yaw)
    p_ob = np.asarray(t_odom_base, dtype=float)
    p_of = np.array([p_ob[0], p_ob[1], 0.0])
    r_bo = r_ob.inv()
    t_bf = r_bo.apply(p_of - p_ob)
    q_bf = (r_bo * r_of).as_quat()
    return t_bf, q_bf


def battery_fields(px4):
    """px4_msgs/BatteryStatus -> dict of sensor_msgs/BatteryState fields.

    Units follow sensor_msgs/BatteryState: current is negative while
    discharging (PX4 reports discharge as positive), capacity in Ah,
    percentage 0..1 or NaN when PX4 reports unknown (-1).
    """
    nan = float('nan')
    current = float(px4.current_a)
    remaining = float(px4.remaining)
    capacity_mah = float(px4.capacity)
    cells = int(px4.cell_count)
    temperature = float(px4.temperature)
    return {
        'voltage': float(px4.voltage_v) if px4.voltage_v > 0.0 else nan,
        'current': -current if current != -1.0 else nan,  # PX4: -1 = unknown
        'percentage': remaining if 0.0 <= remaining <= 1.0 else nan,
        'capacity': capacity_mah / 1000.0 if capacity_mah > 0.0 else nan,
        'design_capacity': capacity_mah / 1000.0 if capacity_mah > 0.0 else nan,
        'charge': ((capacity_mah - float(px4.discharged_mah)) / 1000.0
                   if capacity_mah > 0.0 and px4.discharged_mah >= 0.0 else nan),
        'temperature': temperature,
        'present': bool(px4.connected),
        'cell_voltage': [float(v) for v in list(px4.voltage_cell_v)[:cells]],
    }


# No pack: MAVROS reports percentage NaN and temperature NaN/garbage, while the
# XRCE driver (PX4's BatteryStatus with nothing connected) reports remaining 0.0
# and temperature 0.0. battery_no_pack_fixup() gives MAVROS the DDS values so
# both backends publish the same BatteryState (px4-06 review follow-up 4).
NO_PACK_PERCENTAGE = 0.0
NO_PACK_TEMPERATURE = 0.0


def battery_no_pack_fixup(present, percentage, temperature):
    """(percentage, temperature) as the DDS backend reports them. With a pack
    present the values pass through unchanged (unknown stays NaN, as on DDS)."""
    if present:
        return percentage, temperature
    return NO_PACK_PERCENTAGE, NO_PACK_TEMPERATURE
