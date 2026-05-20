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

The node also broadcasts `odom → base_link` and `odom → base_footprint` TF transforms at 10 Hz.

---

## Helpful Links

- [PX4 ROS2 User Guide](https://docs.px4.io/v1.15/en/ros2/user_guide.html)
- [PX4 Offboard Control Example](https://docs.px4.io/v1.15/en/ros2/offboard_control.html)
- [uORB Message Definitions](https://docs.px4.io/main/en/msg_docs/)
- [PX4 Ethernet Network Setup](https://docs.px4.io/v1.15/en/advanced_config/ethernet_setup.html)
- [ROS2 Jazzy Installation Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [Micro XRCE-DDS Agent](https://docs.px4.io/v1.15/en/ros2/user_guide.html#setup-micro-xrce-dds-agent-client)
- [joy ROS2 package](https://docs.ros.org/en/ros2_packages/jazzy/api/joy/index.html)
