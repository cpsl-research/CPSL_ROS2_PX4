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

## PX4 Configuration (one-time)

The PX4 must be configured to communicate with the companion computer over ethernet via XRCE-DDS. Open QGroundControl, navigate to **Q (top-left) → Vehicle Setup → Parameters**, and set:

| Parameter | Value | Notes |
|-----------|-------|-------|
| `UXRCE_DDS_CFG` | Ethernet | Requires reboot before the next parameters appear |
| `UXRCE_DDS_PRT` | 8888 | UDP port for the DDS agent |
| `UXRCE_DDS_AG_IP` | 170461697 | Encodes `10.41.10.1` — see [PX4 docs](https://docs.px4.io/main/en/middleware/uxrce_dds.html#starting-the-client) |

Reboot the Pixhawk after making these changes.

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
| `odom` | `nav_msgs/Odometry` | `BEST_EFFORT` + `TRANSIENT_LOCAL`, depth 1 | PX4 odometry converted to ENU, frame `odom`, child `base_link`; one message per PX4 odometry message |
| `battery` | `sensor_msgs/BatteryState` | `RELIABLE` + `TRANSIENT_LOCAL`, depth 1 | from `/fmu/out/battery_status`; unknown values are NaN, current is negative while discharging, capacity in Ah, percentage 0..1 |
| `status/armed` | `std_msgs/Bool` | `RELIABLE` + `TRANSIENT_LOCAL`, depth 1 | armed |
| `status/mode` | `std_msgs/String` | same | PX4 nav_state name, e.g. `OFFBOARD`, `POSCTL`, `AUTO_LAND` |
| `status/preflight_ok` | `std_msgs/Bool` | same | PX4 pre-flight checks pass |
| `status/failsafe` | `std_msgs/Bool` | same | PX4 failsafe active |

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

Startup fails if the limits are inconsistent (need `0 < min_altitude_m < takeoff_altitude_m <= max_altitude_m`, rate and timeout positive).

### Running under console

```bash
bash scripts/deps/px4.sh                 # dependencies and Micro-XRCE-DDS-Agent (--check needs no root)
python3 scripts/host_setup.py --iface <wired-if> --apply   # 10.41.10.1/24 towards the FCU at 10.41.10.2
colcon build --symlink-install --packages-up-to px4_controller
source install/setup.bash
ros2 launch px4_controller console.launch.py          # --show-args lists the arguments
```

Launch arguments: `namespace` (`cpsl_uav_1`), `backend` (`xrce`; accepted so console can pass `backend:=xrce`, currently ignored, reserved for the MAVROS backend), `agent` (`true`; set `false` if an agent already runs), `agent_port` (8888), `takeoff_altitude_m`, `min_altitude_m`, `max_altitude_m`, `max_climb_rate_mps`, `cmd_timeout_s` (defaults in the Parameters table). Outdoor use only changes these arguments. `scripts/host_setup.py --undo` removes the network profile it created.

---

## Helpful Links

- [PX4 ROS2 User Guide](https://docs.px4.io/v1.15/en/ros2/user_guide.html)
- [PX4 Offboard Control Example](https://docs.px4.io/v1.15/en/ros2/offboard_control.html)
- [uORB Message Definitions](https://docs.px4.io/main/en/msg_docs/)
- [PX4 Ethernet Network Setup](https://docs.px4.io/v1.15/en/advanced_config/ethernet_setup.html)
- [ROS2 Jazzy Installation Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Micro XRCE-DDS Agent](https://docs.px4.io/v1.15/en/ros2/user_guide.html#setup-micro-xrce-dds-agent-client)
- [joy ROS2 package](https://docs.ros.org/en/ros2_packages/jazzy/api/joy/index.html)
