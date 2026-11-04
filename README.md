# CPSL_ROS2_PX4

A collection of ROS2 packages for controlling a PX4-based X500 UAV from a companion computer. Provides offboard position/velocity control, keyboard and joystick teleoperation, odometry conversion (PX4 NED → ROS2 ENU), TF broadcasting, and a Nav2 integration interface.

---

## Prerequisites

### Hardware

- X500 UAV with Pixhawk 6X running PX4 v1.15 — see the [PX4 X500 Setup Guide](CPSL_Manuals/UAVs/PX4_X500.md) for firmware flashing and vehicle configuration
- Companion computer running Ubuntu 24.04 (tested on Intel NUC-13 and GMKTec)
- Ethernet connection between the companion computer and the Pixhawk
  - **Note:** The NUC-13 requires a USB ethernet adapter or ethernet switch; its built-in port cannot connect directly to the Pixhawk 6X
  - Companion computer static IP: `10.41.10.1`, subnet: `255.255.255.0`
  - Pixhawk IP: `10.41.10.2`

### Software

- Ubuntu 24.04
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Micro XRCE-DDS Agent](https://docs.px4.io/v1.15/en/ros2/user_guide.html#setup-micro-xrce-dds-agent-client) (built from source)
- [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html) (Ubuntu Linux instructions)
- Python: `numpy`, `scipy`, `python3-pynput`
- ROS2 packages: `joy`, `nav2_bringup`

---

## Installation

### 1. Install ROS2 Jazzy

Follow the [ROS2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), then set up `rosdep`:

```bash
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
```

### 2. Install the Micro XRCE-DDS Agent

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### 3. Clone this repository

```bash
git clone --recurse-submodules https://github.com/cpsl-research/CPSL_ROS2_PX4.git
cd CPSL_ROS2_PX4
```

If you forgot to clone submodules:

```bash
git submodule update --init --recursive
```

### 4. Install ROS2 dependencies

```bash
rosdep install -i --from-path src --rosdistro jazzy -y
```

### 5. Build the workspace

```bash
colcon build --symlink-install
```

### 6. Source the workspace

```bash
source install/setup.bash
# or, if using zsh:
source install/setup.zsh
```

Add this to your `~/.bashrc` / `~/.zshrc` to source automatically on every new terminal.

---

## Pixhawk setup (one-time)

Source: the CPSL manual, [PX4 X500 Setup Guide](CPSL_Manuals/UAVs/PX4_X500.md) sections 5-6. Do the steps in order; `python3 scripts/host_setup.py --iface <iface> --guide [--json] [--no-probe]` prints them with a live, read-only check where one exists (the console's PX4 setup page shows the same list).

1. **Companion address.** Give the wired interface facing the Pixhawk the static address `10.41.10.1/24` (`python3 scripts/host_setup.py --iface <iface> --apply`). Checked by `interface`, `address`, `profile`.
2. **Pixhawk `net.cfg`.** On the Pixhawk's SD card (`etc/net.cfg`), then reboot the Pixhawk:

   ```
   DEVICE=eth0
   BOOTPROTO=fallback
   IPADDR=10.41.10.2
   NETMASK=255.255.255.0
   ROUTER=10.41.10.254
   DNS=10.41.10.254
   ```

   Checked by `fcu` (ping `10.41.10.2`). If it fails, `fcu-address` looks for the FCU on that interface only (ARP table, `arping`) at the PX4 defaults `192.168.0.3`/`192.168.0.4`. In QGC (Analyze Tools > MAVLink Console) run `ifconfig`: `192.168.0.3` means `net.cfg` was not read; `0.0.0.0` means the `BOOTPROTO=fallback` DHCP attempt is still pending (about a minute after boot).
3. **`UXRCE_DDS_CFG`.** In QGroundControl (**Q (top-left) -> Vehicle Setup -> Parameters**) set `UXRCE_DDS_CFG = Ethernet`, then reboot. `UXRCE_DDS_PRT` and `UXRCE_DDS_AG_IP` only appear after this reboot. Checked by `xrce-session` (the agent sees a client).
4. **DDS port and agent address.** Set `UXRCE_DDS_PRT = 8888` and `UXRCE_DDS_AG_IP = 170461697` (encodes `10.41.10.1`, see the [PX4 docs](https://docs.px4.io/main/en/middleware/uxrce_dds.html#starting-the-client)), then reboot. Checked by `xrce-session`.

Also checked: `usb` (the by-id device) and `firmware`, which compares the PX4 version the FC reports (a read-only MAVLink `AUTOPILOT_VERSION` request through `scripts/firmware.py`, or the versioned `/fmu/out/*_vN` topic names) with the `px4_msgs` pin in `firmware/versions.json`. On a mismatch you either reflash the FC (see Firmware below) or re-pin `px4_msgs`; the check does not choose. Nothing here writes parameters or the SD card.

---

## Running

### Step 1 — Start the XRCE-DDS Agent (required for all modes)

Run this in a dedicated terminal before launching any ROS2 nodes:

```bash
MicroXRCEAgent udp4 -p 8888
```

By default, PX4 topics will be published under the `/fmu` namespace. To use a custom namespace (e.g. `/cpsl_uav_1/fmu`), enter the following in QGroundControl's **Analyze Tools → MAVLink Console** before starting the agent:

```
uxrce_dds_client stop
uxrce_dds_client start -n cpsl_uav_1
```

---

### Keyboard (keyop) Control

Launch the control node and the keyboard teleoperation node in separate terminals.

**Terminal 1 — control node:**
```bash
source install/setup.bash
ros2 launch px4_controller control_only_launch.py
```

**Terminal 2 — keyboard node:**
```bash
source install/setup.bash
ros2 launch px4_controller keyop_only.launch.py
```

> **Warning:** The keyop node captures all keyboard input globally as soon as it starts. Only launch it when you are ready to control the UAV.

**Key bindings:**

| Key | Action |
|-----|--------|
| `a` | Arm |
| `d` | Disarm |
| `t` | Takeoff |
| `l` | Land |
| `Space` | Hover (stop and hold position) |
| `↑` | Translate forward |
| `↓` | Translate backward |
| `←` | Translate left |
| `→` | Translate right |
| `r` | Rotate (yaw) |
| `[` | Rotate 90° counter-clockwise |
| `]` | Rotate 90° clockwise |

---

### Joystick (PS4) Control

```bash
source install/setup.bash
ros2 launch px4_controller joy_control_launch.py \
    joy_enable:=true \
    control_enable:=true \
    namespace:=cpsl_uav_1
```

**Button mappings:**

| Input | Action |
|-------|--------|
| `R2` | Arm |
| `L2` | Disarm |
| `Triangle` | Takeoff |
| `X` | Land |
| `L1` | Deadman switch — must be held for velocity commands to execute |
| `R1` | Enable autonomous navigation commands (from Nav2 `cmd_vel_nav`) |
| Left joystick | Translate (forward/back and left/right) |
| Right joystick | Yaw |
| D-pad up/down | Increase/decrease max linear velocity (0.125–0.75 m/s) |
| D-pad left/right | Increase/decrease max angular velocity (0.1–0.5 rad/s) |

---

### Control Node Only (Nav2 / external cmd_vel)

Use this when velocity commands will come from an external source such as Nav2:

```bash
source install/setup.bash
ros2 launch px4_controller control_only_launch.py
```

**Subscribed topics** (relative to node namespace):

| Topic | Type | Description |
|-------|------|-------------|
| `armed_status` | `std_msgs/Bool` | `true` = arm, `false` = disarm |
| `takeoff` | `std_msgs/Bool` | Trigger takeoff |
| `land` | `std_msgs/Bool` | Trigger landing |
| `cmd_vel` | `geometry_msgs/TwistStamped` | Manual velocity command (body-frame FLU) |
| `cmd_vel_nav` | `geometry_msgs/TwistStamped` | Autonomous velocity command from Nav2 |
| `deadman_pressed` | `std_msgs/Bool` | Must be `true` for velocity commands to be forwarded |
| `allow_nav_cmds` | `std_msgs/Bool` | `true` = use `cmd_vel_nav`; `false` = use `cmd_vel` |
| `rotate` | `std_msgs/Bool` | Trigger 90° yaw rotation |

**Published topics:**

| Topic | Type | Description |
|-------|------|-------------|
| `odom` | `nav_msgs/Odometry` | PX4 odometry converted to ENU (for Nav2) |
| `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | Arm/disarm/mode commands to PX4 |
| `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | Offboard heartbeat |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | Position/velocity setpoints to PX4 |

The node also broadcasts `odom → base_link` and `base_link → base_footprint` TF transforms (see [Console interface](#console-interface) for rate and frames).

---

## Console interface

This section is the contract between this component and the console GUI. It is made only of standard ROS message types (no `px4_msgs`), so any flight backend that implements the same topics, types and QoS can replace this one. The PX4 side (`/fmu/...`) is not part of the contract and stays absolute: one PX4 per DDS domain.

### Backends

`px4_control_node` is a transport-free core (flight state, deadman and timeout, altitude target and limits, the topics below) plus one I/O adapter, chosen with `backend` (launch argument and node parameter). Both implement this whole contract; console does not know which one runs.

| `backend` | Adapter | Link to the Pixhawk | Started by `console.launch.py` |
|-----------|---------|---------------------|--------------------------------|
| `xrce` (default) | `xrce_io.py` | uXRCE-DDS over Ethernet: `/fmu/in/*`, `/fmu/out/*` (absolute, one PX4 per DDS domain) | `MicroXRCEAgent udp4 -p <agent_port>` (unless `agent:=false`) + the node |
| `mavros` | `mavros_io.py` | MAVLink over the Pixhawk's USB-C, through MAVROS at `<namespace>/mavros/...` | `mavros_node` (`launch/mavros.launch.py`, `config/px4_mavros.yaml`, `fcu_url`) + the node |

Only one backend commands at a time. XRCE may stay up on Ethernet while the MAVROS backend runs over USB; nothing in either backend uses the other. How the two differ underneath (same behaviour at the contract):

| | `xrce` | `mavros` |
|---|---|---|
| OFFBOARD heartbeat | `OffboardControlMode` at 5 Hz from start; `TrajectorySetpoint` at 20 Hz only while a command is active, plus a neutral hold-position setpoint (5 Hz) from an arm request until disarm/land | none exists over MAVLink: the `setpoint_raw/local` stream is the heartbeat, so it runs at 20 Hz from the first odometry on, holding the current position (disarmed: tracking it; armed with no command: latched) when the core has no setpoint |
| Entering OFFBOARD | never at start-up. Only for an arm request or a `takeoff`, when `status/preflight_ok` is true and the setpoint stream has run > 1 s (retry each second, max 5, 15 s limit). Arm path: hold setpoint streams, OFFBOARD is requested, and the arm command is sent when PX4 reports OFFBOARD (so arming needs no RC/POSCTL); if PX4 is already in OFFBOARD it arms at once. Disarm/land cancel it. | `set_mode OFFBOARD` only after >= 1 s of continuous stream; retried each second (max 5) while disarmed until PX4 reports OFFBOARD, re-opened after each disarm; never on its own while armed (a pilot's mode change stands). An explicit `takeoff` also requests it. |
| Setpoint frames | NED position/velocity, NED yaw rate | hover/takeoff/yaw step: `FRAME_LOCAL_NED` with ENU position and ENU yaw (MAVROS converts); lateral velocity: `FRAME_BODY_NED` with the FLU velocity as commanded and the ROS-sign yaw rate (MAVROS converts; no second flip here), position/acceleration/yaw ignored. PX4 drops position in body frame, so altitude is held through `vz` = climb feed-forward + `alt_hold_gain_per_s` x altitude error, clamped to `max_climb_rate_mps` |
| Arm / land | `VehicleCommand` arm, `NAV_LAND` | `cmd/arming`, `set_mode AUTO.LAND` |
| `status/mode` | `nav_state` name | MAVROS mode string mapped to the same names (`AUTO.LAND` -> `AUTO_LAND`, `STABILIZED` -> `STAB`) |
| `status/preflight_ok` | "can fly OFFBOARD now" (see below), not PX4's `pre_flight_checks_pass` | `SYS_STATUS` health bit `MAV_SYS_STATUS_PREARM_CHECK` (PX4: can arm in the current mode) |
| `status/failsafe` | `failsafe` | heartbeat `system_status` CRITICAL/EMERGENCY/FLIGHT_TERMINATION (PX4 reports CRITICAL only while armed) |
| `battery` | `/fmu/out/battery_status` | `mavros/battery` (BATTERY_STATUS, about 0.5 Hz: slow is normal, not a link loss); MAVLink unknowns become NaN |
| `odom` world frame | x north, y west, z up (PX4 NED turned 180 deg about x) | ENU (x east, y north, z up), from `local_position/odom` |

Both `odom` frames are z-up and right-handed with the PX4 local origin; they differ by a 90 deg yaw of the `odom` axes.

All topics are relative to the node namespace (`cpsl_uav_1` under `console.launch.py`).

### Commands in (subscribed)

All command subscribers are `BEST_EFFORT` + `VOLATILE`, depth 1. `VOLATILE` means a command is never replayed to a restarted node; `BEST_EFFORT` matches both reliable and best-effort publishers. Publish commands `VOLATILE` (a `TRANSIENT_LOCAL` publisher would also connect, but nothing is latched).

| Topic | Type | Meaning |
|-------|------|---------|
| `armed_status` | `std_msgs/Bool` | `true` arm, `false` disarm |
| `takeoff` | `std_msgs/Bool` | `true` takes off to `takeoff_altitude_m` (ignored if not armed); `false` is ignored |
| `land` | `std_msgs/Bool` | any message lands |
| `deadman_pressed` | `std_msgs/Bool` | must be `true` and fresh for velocity commands to act |
| `cmd_vel` | `geometry_msgs/TwistStamped` | manual velocity, body FLU: `linear.x` forward, `linear.y` left (m/s), `linear.z` climb rate (m/s, clamped to `max_climb_rate_mps`, integrated into the altitude target), `angular.z` yaw rate |
| `cmd_vel_nav` | `geometry_msgs/TwistStamped` | autonomous velocity (Nav2), same convention; used instead of `cmd_vel` while `allow_nav_cmds` is `true` |
| `allow_nav_cmds` | `std_msgs/Bool` | `true` selects `cmd_vel_nav`, `false` selects `cmd_vel` |
| `rotate` | `std_msgs/Bool` | 90 degree yaw step (`true` clockwise, `false` counter-clockwise); ignored when not flying |

**Commands must be republished at more than 2 Hz by every publisher** (the Joy node needs `autorepeat_rate`, set to 20 Hz in `config/px4_joy.yaml`). The node only acts on `deadman_pressed` and on the selected velocity topic for `cmd_timeout_s` (default 0.5 s) after the last message. The last command keeps acting until that timeout, then the aircraft hovers (position held, altitude target held). A publisher that dies, or a deadman released, therefore ends in a hover, not a continued motion. The next fresh command resumes control.

Altitude: `target_altitude_m` is set at takeoff, then `linear.z` (clamped to +/- `max_climb_rate_mps`) is integrated into it and kept within [`min_altitude_m`, `max_altitude_m`]. Velocity and hover setpoints both hold that target. Takeoff completes when the measured altitude reaches `min_altitude_m`.

### State out (published)

| Topic | Type | QoS | Meaning |
|-------|------|-----|---------|
| `odom` | `nav_msgs/Odometry` | `BEST_EFFORT` + `TRANSIENT_LOCAL`, depth 1 | PX4 odometry, frame `odom` (z up; axes per backend, table above), child `base_link`, twist in `base_link`; one message per PX4 odometry message |
| `battery` | `sensor_msgs/BatteryState` | `RELIABLE` + `TRANSIENT_LOCAL`, depth 1 | from the backend (table above); unknown values are NaN, current is negative while discharging, capacity in Ah, percentage 0..1 |
| `status/armed` | `std_msgs/Bool` | `RELIABLE` + `TRANSIENT_LOCAL`, depth 1 | armed |
| `status/mode` | `std_msgs/String` | same | PX4 nav_state name, e.g. `OFFBOARD`, `POSCTL`, `AUTO_LAND` |
| `status/preflight_ok` | `std_msgs/Bool` | same | PX4 pre-flight checks pass |
| `status/failsafe` | `std_msgs/Bool` | same | PX4 failsafe active |
| `status/failsafe_flags` | `std_msgs/String` | same | JSON list of the PX4 `failsafe_flags` that are set (sorted names, e.g. `["global_position_invalid","manual_control_signal_lost"]`); `[]` when none or not reported. Additive: for warnings, never a gate |
| `status/px4_preflight_raw` | `std_msgs/Bool` | same | PX4's own `pre_flight_checks_pass` (needs RC, mag, GPS ... as the current mode requires). Published by the `xrce` backend only |

**`status/preflight_ok` (xrce) means "OFFBOARD can be flown now"**: OFFBOARD (bit 14) is set in `VehicleStatus.can_set_nav_states_mask` and none of `failsafe_flags.attitude_invalid`, `angular_velocity_invalid`, `local_position_invalid_relaxed`, `local_velocity_invalid` is set. `global_position`, `home_position`, `auto_mission`, `manual_control_signal_lost` (RC) and `offboard_control_signal_lost` (true until setpoints stream) are ignored. False until `/fmu/out/failsafe_flags` has been heard. The `mavros` backend still reports the PREARM health bit.

`status/*` and `battery` are published at 2 Hz and immediately on change (battery: on connect or warning change). Nothing is published until PX4 has been heard from.

**`odom` is `BEST_EFFORT` + `TRANSIENT_LOCAL`**, as the PX4 side is. Subscribers must match: a `RELIABLE` subscriber will not connect to it. A `BEST_EFFORT` subscriber (volatile or transient-local) does.

### TF

`odom -> base_link` and `base_link -> base_footprint` (yaw-only ground projection) are broadcast from the odometry callback, rate-limited to `tf_rate_hz` (node parameter, default 30, must be >= 10). Frame names are unprefixed. Under `console.launch.py` the namespace is carried by the topic: `/tf` and `/tf_static` are remapped to `tf` and `tf_static` inside the namespace, per console's `docs/TF_AND_TOPICS.md`.

**Note for the older launch files** (`control_only_launch.py`, `joy_control_launch.py`, `keyop_control_launch.py`): they do not remap tf, so they now publish the unprefixed frames `odom`/`base_link`/`base_footprint` on the global `/tf`. Anything that relied on the previous prefixed frame names must be updated.

### Parameters

All are read once at startup (changing them needs a restart). The node defaults equal the `console.launch.py` defaults, so launches that do not pass them still start.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `takeoff_altitude_m` | 0.75 | altitude held after takeoff |
| `min_altitude_m` | 0.5 | lowest commanded altitude while flying |
| `max_altitude_m` | 2.5 | highest commanded altitude |
| `max_climb_rate_mps` | 0.3 | clamp on `linear.z` |
| `cmd_timeout_s` | 0.5 | hover when `deadman_pressed` or the selected velocity topic is older than this |
| `tf_rate_hz` | 30 | tf publish rate (>= 10); node parameter only, not a launch argument |
| `backend` | `xrce` | `xrce` or `mavros`; also a launch argument |
| `mavros_namespace` | `mavros` | backend `mavros`: where MAVROS's topics are, relative to the node's namespace |
| `alt_hold_gain_per_s` | 1.0 | backend `mavros`: altitude-error gain of `vz` in body-velocity setpoints (>= 0) |

Startup fails if the limits are inconsistent (need `0 < min_altitude_m < takeoff_altitude_m <= max_altitude_m`, rate and timeout positive).

### Running under console

```bash
bash scripts/deps/px4.sh                 # dependencies, Micro-XRCE-DDS-Agent, MAVROS + geoid (--check needs no root)
python3 scripts/host_setup.py --iface <wired-if> --apply   # 10.41.10.1/24 towards the FCU at 10.41.10.2
colcon build --symlink-install --packages-up-to px4_controller
source install/setup.bash
ros2 launch px4_controller console.launch.py          # --show-args lists the arguments
```

Launch arguments: `namespace` (`cpsl_uav_1`), `backend` (`xrce` or `mavros`), `fcu_url` (backend `mavros`; default `/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00:115200`), `agent` (backend `xrce`; `true`, set `false` if an agent already runs), `agent_port` (8888), `takeoff_altitude_m`, `min_altitude_m`, `max_altitude_m`, `max_climb_rate_mps`, `cmd_timeout_s` (defaults in the Parameters table). Outdoor use only changes these arguments. `scripts/host_setup.py --undo` removes the network profile it created.

#### Parameters against the lab

`config/lab.params` holds the lab's EKF2_*, UAVCAN_*, battery and flow-sensor parameters (QGC format). `scripts/host_setup.py --params-check` (or `scripts/params_check.py`) reads them from the flight controller over USB and prints `{name,status,detail,fix,why}` rows: WARN per differing parameter (the fix is to set it in QGroundControl), PASS when all match. Differences in `ALLOWED_DIFFS` (`EKF2_GPS_*`, `EKF2_BARO_*`, `EKF2_HGT_REF`: GPS/baro/height reference vary between indoor and outdoor) are INFO. It only sends `PARAM_REQUEST_READ`; the port must be free (not held by MAVROS).

#### MAVROS over USB (`backend:=mavros`, bench only for now)

```bash
python3 scripts/host_setup.py --usb --json        # by-id device present; ModemManager told to ignore it
python3 scripts/host_setup.py --params-check --json   # live FC params vs config/lab.params, read-only (PARAM_REQUEST_READ); never writes
python3 scripts/host_setup.py --usb --apply       # PRINTS the udev rule command and its undo; runs nothing
ros2 launch px4_controller console.launch.py backend:=mavros   # fcu_url:=/dev/serial/by-id/<id>:115200
ros2 launch px4_controller mavros.launch.py       # MAVROS alone (reads the FC, commands nothing), topics at /mavros/...
```

- **Device.** Use the `/dev/serial/by-id/` name in `fcu_url`, never `/dev/ttyACM<N>`: the number changes when the FC re-enumerates (after a reboot the node disappears and returns; MAVROS retries the link). It is USB CDC ACM, so the baud is ignored, but MAVROS's `path:baud` syntax needs one. In console's container (privileged, `/dev` mounted) a hot-plugged port is visible and root needs no `dialout`; on a host the user needs `dialout`.
- **ModemManager** probes new ttyACM ports as modems; a probe in flight makes MAVROS fail with "Device or resource busy". `host_setup.py --usb` reads `ID_MM_DEVICE_IGNORE` with `udevadm` and prints a udev rule (VID 3185, PID 0035) plus its undo; installing it needs sudo and is left to the user.
- **One reader per port.** MAVROS takes no exclusive lock: QGroundControl on the same tty splits the byte stream. Keep QGC off the port, or bridge it with `gcs_url`.
- **PX4 side, no change needed by default:** USB MAVLink starts by itself (`SYS_USB_AUTO=2`) in onboard mode (`USB_MAV_MODE=2`), and setpoints are only accepted with `MAV_FWDEXTSP=1` (default). OFFBOARD loss: `COM_OF_LOSS_T` (1 s), then `COM_OBL_RC_ACT`. With no RC, `COM_RC_IN_MODE` decides whether arming is allowed. Record these on the bench.
- **`CBRK_USB_CHK`**: unless it is 197848 (the PX4 default, check off), arming fails with "Flying with USB is not safe" once a USB MAVLink link has been up, until the FC reboots. USB is bench only (props off); do not change the breaker to fly on USB.
- **USB back-powering**: the FC can run from USB alone; that says nothing about motor or servo power. Props off.
- **GeographicLib**: MAVROS aborts at start without the `egm96-5` geoid; `scripts/deps/px4.sh` installs it with MAVROS's `install_geographiclib_datasets.sh` and checks it.
- **Plugins**: `config/px4_mavros.yaml` loads only `sys_status`, `sys_time`, `command`, `imu` (attitude for `local_position/odom`), `local_position` and `setpoint_raw`, with MAVROS tf off; the node is the only tf source.

---

## Firmware

The Pixhawk 6X runs **PX4 v1.15.4**, the firmware the pinned `px4_msgs` (`release/1.15`) describes.
`firmware/versions.json` is the one place the pairing is recorded (firmware tag and commit,
px4_msgs commit, board target `px4_fmu-v6x_default`, board_id 53, USB `3185:0035`), and
`firmware/PX4-Autopilot` is the shallow submodule at that commit. `scripts/firmware.py` (stdlib
Python) checks, backs up, builds (prints) and flashes it. It never writes parameters: re-applying
a param file (`CPSL_Manuals/UAVs/cpsl_px4_v15_4_params.params`) is done in QGroundControl.

```bash
git submodule update --init --depth 1 firmware/PX4-Autopilot src/px4_msgs
python3 scripts/firmware.py --check [--json] [--no-probe]   # board, pins, current version, artifact, ModemManager
python3 scripts/firmware.py --build                         # PRINTS the commands below; runs none
```

`--build` prints five host commands: create the (gitignored) `firmware/out/`, init only the 7
NuttX/driver submodules the board needs (`--depth 1`), fetch the one NuttX release tag
(`nuttx.tag` in `versions.json`; a depth-1 NuttX has no tags and PX4's version-header script
fails without one), the Docker build in PX4's own toolchain
image (`px4io/px4-dev-nuttx-focal:2022-08-12`, output to `firmware/out/`), and
`firmware.py --record`, which stores the artifact's sha256 next to it. Run them yourself. The
build is long (first run downloads the image and the submodules).

The board is identified without `udevadm` (so it also works inside the console container, which
has none): VID:PID, product string and serial come from sysfs (`/sys/class/tty/ttyACMn/device` and
its parent USB device: `idVendor`, `idProduct`, `product`, `serial`), reached from the
`/dev/serial/by-id` symlink; `udevadm` is only an optional extra. Application vs bootloader is still
told apart by the product string (`PX4 FMU v6X.x` vs `PX4 BL FMU v6X.x`). The "current version" row
uses `pymavlink`, which Ubuntu 24.04 / ROS Jazzy does not package (`ros-jazzy-mavros` does not pull
one in), so `scripts/deps/px4.sh` installs `python3-serial` and pip-installs a pinned pymavlink
(`--no-deps`) into `/opt/pymavlink`; the probe adds that directory (or `$PYMAVLINK_PATH`) to its
import path. `bash scripts/deps/px4.sh --check` (no root) reports it missing. Without it the row
is INFO `pymavlink not installed`.

Before a flash, take a read-only param backup and compare it with the lab file. The backup is
self-contained: it starts its own temporary `mavros_node` (namespace `fw_backup`, plugins
`sys_status`, `sys_time`, `param` only, no tf, isolated `ROS_DOMAIN_ID` 95 with
`ROS_LOCALHOST_ONLY=1`; override with `--backup-domain` or `CPSL_FW_BACKUP_DOMAIN`) on the board's
by-id USB device, waits for the connection, runs `param/pull`, reads the parameters with
`ros2 param dump`, and always stops the node and checks the port is released. It needs the USB port
free (QGroundControl closed, no MAVROS driver running); the XRCE driver does not use that port and
may keep running. It never sets or loads a parameter, arms, changes mode or publishes, and never
overwrites an existing file. The QGC-format file's type column is approximate (names and values are
exact).

```bash
python3 scripts/firmware.py --backup-params ~/px4_before.params --diff /path/to/cpsl_px4_v15_4_params.params
python3 scripts/firmware.py --diff-params A.params B.params    # any two QGC files, by name and value
```

Flashing needs you present with **props off**. Stop the XRCE agent, MAVROS and QGroundControl,
then:

```bash
python3 scripts/firmware.py --flash --dry-run --backup ~/px4_before.params --confirm-serial <serial>
python3 scripts/firmware.py --flash --apply   --backup ~/px4_before.params --confirm-serial <serial>
```

`--flash` refuses (and says which guard) unless: exactly one board with `3185:0035` is in
**application** mode (the bootloader has the same VID:PID and is told apart by its product string
`PX4 BL FMU v6X.x`); the typed serial equals the board's USB serial (shown by `--check`); the backup
is from `--backup-params`, under an hour old, unchanged, from this board; no `MicroXRCEAgent`,
`mavros_node`, `px4_control_node` or QGroundControl is running; the artifact's sha256 equals the one
`--record` stored, names board_id 53 and was built from the pinned commit; `fuser`/`lsof` (else a scan of `/proc/*/fd`, so a container without psmisc still works; `scripts/deps/px4.sh`
installs `psmisc`) find the
port free; and `Tools/px_uploader.py` plus pyserial are present. `--apply` then runs the pinned
`px_uploader.py`, which reboots the board into its bootloader itself, so do not unplug it.
ModemManager is reported as a warning (`host_setup.py --usb` has the fix); if the upload cannot
find the bootloader, stop it and retry. Flashing from console's container is untested.

## Helpful Links

- [PX4 ROS2 User Guide](https://docs.px4.io/v1.15/en/ros2/user_guide.html)
- [PX4 Offboard Control Example](https://docs.px4.io/v1.15/en/ros2/offboard_control.html)
- [uORB Message Definitions](https://docs.px4.io/main/en/msg_docs/)
- [PX4 Ethernet Network Setup](https://docs.px4.io/v1.15/en/advanced_config/ethernet_setup.html)
- [ROS2 Jazzy Installation Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Micro XRCE-DDS Agent](https://docs.px4.io/v1.15/en/ros2/user_guide.html#setup-micro-xrce-dds-agent-client)
- [joy ROS2 package](https://docs.ros.org/en/ros2_packages/jazzy/api/joy/index.html)
