# Gati Bot — ROS 2 Differential Drive AMR

A ROS 2 workspace for **Gati Bot**, a custom differential-drive Autonomous Mobile Robot (AMR) built on top of an ESP32 microcontroller. The robot uses an RPLidar A1 for perception, implements wheel-encoder odometry, and supports full autonomous navigation via Nav2.

---

## Table of Contents

- [Overview](#overview)
- [Hardware](#hardware)
- [Package Structure](#package-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Building the Workspace](#building-the-workspace)
- [Usage](#usage)
  - [1. Launch the Robot (Bringup)](#1-launch-the-robot-bringup)
  - [2. Teleoperation](#2-teleoperation)
  - [3. SLAM (Mapping)](#3-slam-mapping)
  - [4. Localization](#4-localization)
  - [5. Autonomous Navigation](#5-autonomous-navigation)
- [ROS 2 Topics](#ros-2-topics)
- [Configuration & Tuning](#configuration--tuning)
- [Maps](#maps)
- [TF Tree](#tf-tree)
- [Contributing](#contributing)

---

## Overview

Gati Bot is a two-wheeled differential-drive robot with a front caster wheel. It communicates with an ESP32 motor controller over a serial USB link. The ESP32 sends raw encoder tick counts back to the host PC, where ROS 2 computes odometry and publishes it on `/odom`. The RPLidar A1 laser scanner is mounted on top for SLAM and navigation.

**Key capabilities:**
- Serial-based motor control (ESP32 ↔ ROS 2 via `/dev/gati_bot_mb`)
- Wheel encoder odometry with TF broadcasting (`odom → base_footprint`)
- SLAM mapping with `slam_toolbox`
- AMCL/Nav2 localization on pre-built maps
- Autonomous point-to-point navigation (Nav2 stack)
- Keyboard teleoperation

---

## Hardware

| Component | Specification |
|---|---|
| Chassis | Custom differential drive |
| Microcontroller | ESP32 (serial @ 115200 baud) |
| Drive wheels | 65 mm diameter, 275 ticks/revolution |
| Wheel base (physical) | 134 mm |
| Caster | Front passive caster |
| LiDAR | SLAMTEC RPLidar A1 |
| Host PC | Ubuntu 22.04 + ROS 2 Humble |

---

## Package Structure

```
gati_bot_ws/
└── src/
    ├── gati_bot_model_description/   # URDF/Xacro robot model
    │   └── urdf/
    │       ├── robot_model.urdf.xacro
    │       ├── lidar.urdf.xacro
    │       ├── materials.urdf.xacro
    │       └── my_robot.urdf.xacro   # top-level entry point
    │
    ├── gati_bot_node/                # Motor driver & odometry node
    │   └── gati_bot_node/
    │       ├── motor.py              # v1 motor node
    │       └── motor2.py             # v2 motor node (active — use this)
    │
    ├── gati_bot_bringup/             # Launch files & config
    │   ├── launch/
    │   │   ├── bot.launch.py             # Main bringup (robot + LiDAR + RViz)
    │   │   ├── online_async_launch.py    # SLAM mapping
    │   │   ├── localization_launch.py    # AMCL localization
    │   │   └── navigation_launch.py      # Nav2 autonomous navigation
    │   ├── config/
    │   │   ├── nav2_params.yaml
    │   │   ├── slam_params.yaml
    │   │   ├── mapper_params_online_async.yaml
    │   │   └── rviz2.rviz
    │   ├── maps/                         # Saved map files (.pgm + .yaml)
    │   └── src/
    │       └── bt.xml                    # Nav2 Behavior Tree
    │
    ├── gatibot_teleop/               # Keyboard teleoperation node
    │   └── gatibot_teleop/
    │       └── teleop_twist_keyboard
    │
    └── rplidar_ros/                  # RPLidar ROS 2 driver (vendored)
```

---

## Prerequisites

- **OS:** Ubuntu 22.04 LTS
- **ROS 2:** Humble Hawksbill
- **Python:** 3.10

Install the following ROS 2 dependencies:

```bash
sudo apt update && sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-tf2-ros \
  ros-humble-tf-transformations \
  python3-serial \
  python3-transforms3d
```

---

## Installation

```bash
# Clone the repository
cd gati_bot_ws/src/
git clone https://github.com/AJBhandurge/Gatibot-Differential-Drive-Robot-for-SLAM-and-Navigation.git

# Source ROS 2
source /opt/ros/humble/setup.bash
```

### udev Rule for ESP32 Serial Port

The robot node expects the ESP32 at `/dev/gati_bot_mb`. Create a persistent udev rule so the device always gets this name:

```bash
# Find your ESP32 vendor/product IDs
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct'

# Create the rule
sudo nano /etc/udev/rules.d/99-gatibot.rules
```

Add the following line (replace `xxxx` with your actual IDs):

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="xxxx", SYMLINK+="gati_bot_mb"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## Building the Workspace

```bash
cd gati_bot_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Usage

Always source the workspace before running any commands:

```bash
source install/setup.bash
```

### 1. Launch the Robot (Bringup)

This launches the robot state publisher, joint state publisher, motor driver node, RPLidar A1, and RViz2:

```bash
ros2 launch gati_bot_bringup bot.launch.py
```

### 2. Teleoperation

In a new terminal:

```bash
ros2 run gatibot_teleop teleop_twist_keyboard
```

Use the keyboard to drive the robot. The node publishes `geometry_msgs/Twist` on `/cmd_vel`.

### 3. SLAM (Mapping)

First start the bringup, then in a new terminal:

```bash
ros2 launch slam_toolbox online_async_launch.py
```

Drive the robot around to build the map. When satisfied, save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f src/gati_bot_bringup/maps/<map_name>
```

### 4. Localization and Navigation

Use this to localize and naviagtaion autonomously on a previously saved map :


```bash
ros2 launch gati_bot_bringup navigation_launch.py  map:=<path_to_map.yaml>
```

Set a **2D Goal Pose** in RViz2 to send the robot to a target location autonomously.

---

## ROS 2 Topics

| Topic | Type | Description |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands to the robot |
| `/odom` | `nav_msgs/Odometry` | Wheel encoder odometry |
| `/scan` | `sensor_msgs/LaserScan` | RPLidar laser scan data |
| `/robot_description` | `std_msgs/String` | URDF robot model |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree (`odom → base_footprint`) |

---

## Configuration & Tuning

### Odometry Calibration

The motor node (`motor2.py`) uses two separate wheel base values:

```python
self.wheel_base_physical = 0.134   # Real wheel spacing — used for motor commands
self.wheel_base_odom     = 0.134   # Effective value — tune for accurate rotation
```

To calibrate `wheel_base_odom`, command the robot to spin exactly 360° and observe the angle reported in RViz2:

```
new_value = current_value × (rviz_degrees / 360)
```

### Robot Parameters (motor2.py)

| Parameter | Value | Description |
|---|---|---|
| `wheel_radius` | 0.0325 m | Wheel radius (65 mm diameter) |
| `ticks_per_rev` | 275 | Encoder ticks per full revolution |
| `max_speed` | 0.5 m/s | Used to normalize PWM output |

### Nav2 Parameters

Navigation tuning is in `src/gati_bot_bringup/config/nav2_params.yaml`. Key parameters to adjust for your environment include `robot_radius`, `inflation_radius`, and controller velocities.

---

## Maps

Pre-built maps are stored in `src/gati_bot_bringup/maps/`. Each map consists of two files:

- `<map_name>.pgm` — occupancy grid image
- `<map_name>.yaml` — map metadata (resolution, origin)

Additional maps captured during development are in the workspace root as `svr5.pgm`, `svr6.pgm`, etc.

---

## TF Tree

```
odom
 └── base_footprint          (published by motor2.py)
      └── base_link           (from robot_state_publisher)
           ├── right_wheel_link
           ├── left_wheel_link
           ├── caster_wheel_link
           └── laser_frame    (from rplidar_ros)
```

---

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/my-feature`)
3. Commit your changes (`git commit -m 'Add my feature'`)
4. Push to the branch (`git push origin feature/my-feature`)
5. Open a Pull Request
