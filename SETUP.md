# GatiBot / Smorphi — ROS2 Humble Setup Guide

Complete setup guide for the GatiBot four-wheel-drive robot running ROS2 Humble with SLAM, Nav2, and YDLIDAR on Ubuntu 22.04 (Raspberry Pi or NUC).

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Remote Desktop (XRDP)](#remote-desktop-xrdp)
3. [ROS2 Package Dependencies](#ros2-package-dependencies)
4. [Python Dependencies](#python-dependencies)
5. [YDLIDAR SDK](#ydlidar-sdk)
6. [Workspace Setup](#workspace-setup)
7. [Build & Source](#build--source)
8. [udev Rules (Stable Device Symlinks)](#udev-rules-stable-device-symlinks)
9. [Firewall Configuration](#firewall-configuration)
10. [Device & Connection Layout](#device--connection-layout)
11. [Camera Setup (v4l2_camera)](#camera-setup-v4l2_camera)
12. [Troubleshooting](#troubleshooting)

---

## Prerequisites

- Ubuntu 22.04 (Jammy) — bare metal or Raspberry Pi OS with Ubuntu
- ROS2 Humble already installed and sourced (`/opt/ros/humble/setup.bash`)
- Internet access for `apt` and `git`
- User in the `sudo` group

---

## Remote Desktop (XRDP)

Enables Windows Remote Desktop (RDP) access to the robot's desktop over LAN.

```bash
sudo apt update
sudo apt install xrdp -y

# Add xrdp to ssl-cert group (required for TLS)
sudo usermod -a -G ssl-cert xrdp

sudo systemctl restart xrdp
sudo systemctl status xrdp        # Confirm: active (running)
```

> **Connecting:** From Windows, open **Remote Desktop Connection** and enter the robot's IP (visible via `ifconfig`). Port: `3389`.

---

## ROS2 Package Dependencies

Install all required ROS2 Humble packages:

```bash
# URDF / Robot Description
sudo apt install ros-humble-urdf \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui -y

# Geographic & Transform libraries
sudo apt-get install libgeographic-dev -y
sudo apt-get install ros-humble-geographic-msgs -y
sudo apt-get install ros-humble-tf-transformations -y
sudo apt-get install ros-humble-diagnostics -y

# Navigation & SLAM
sudo apt install ros-humble-navigation2 -y
sudo apt install ros-humble-nav2-* -y
sudo apt install ros-humble-slam-toolbox -y

# Camera driver
sudo apt install ros-humble-v4l2-camera -y
```

---

## Python Dependencies

```bash
sudo apt install python3-pip -y

# transforms3d — required for tf_transformations
sudo pip3 install transforms3d

# pyserial — for ESP32 serial communication
pip install pyserial
```

> **Note:** Avoid bare `sudo pip3 install` for project packages. Prefer a virtual environment or `--break-system-packages` on Ubuntu 22.04+ if pip complains.

---

## Workspace Setup

```bash
mkdir -p ~/gati_bot_ws/src
cd ~/gati_bot_ws/src

git clone https://github.com/AJBhandurge/Gatibot-Differential-Drive-Robot-for-SLAM-and-Navigation.git

# Flatten repo contents into src/
cp -r Gatibot-Differential-Drive-Robot-for-SLAM-and-Navigation/* ~/gati_bot_ws/src/
rm -rf Gatibot-Differential-Drive-Robot-for-SLAM-and-Navigation/
```

---

## Build & Source

```bash
cd ~/gati_bot_ws
colcon build

# Source for current session
source install/setup.bash

# Persist across sessions
echo "source ~/gati_bot_ws/install/setup.bash" >> ~/.bashrc
```

> **Tip:** If `colcon build` fails on a single package, use `--packages-select <pkg>` to isolate it. Check `log/latest_build/<pkg>/stdout_stderr.log` for details.

---

## udev Rules (Stable Device Symlinks)

Without udev rules, the LIDAR and ESP32 compete for `/dev/ttyUSB0` and `/dev/ttyUSB1` — their assignment can swap on every reboot or reconnect. These rules bind each device to its **physical USB port** (hub port path), not its enumeration order.

### Write the Rules File

```bash
sudo nano /etc/udev/rules.d/99-gatibot.rules
```

Paste the following (adjust `KERNELS` values if your hub topology differs):

```udev
# LIDAR — USB hub port 1-1.1
SUBSYSTEM=="tty", KERNELS=="1-1.1", SYMLINK+="gati_bot_lidar", MODE="0666"

# ESP32 (Mainboard) — USB hub port 1-1.2
SUBSYSTEM=="tty", KERNELS=="1-1.2", SYMLINK+="gati_bot_mb", MODE="0666"
```

### Reload and Trigger

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger

# Verify symlinks exist
ls -l /dev/gati_bot_*
# Expected:
# /dev/gati_bot_lidar -> ttyUSB0   (or USB1, depends on boot order)
# /dev/gati_bot_mb   -> ttyUSB1
```

---

## Firewall Configuration

Allow RDP access only from devices on the local network (`192.168.1.0/24`):

```bash
sudo ufw allow from 192.168.1.0/24 to any port 3389
sudo ufw reload
sudo ufw status         # Confirm rule is listed
```

> Adjust the subnet (`192.168.1.0/24`) to match your actual LAN if different.

---

## Device & Connection Layout
<img width="1349" height="976" alt="Screenshot from 2026-06-11 11-26-37" src="https://github.com/user-attachments/assets/afc87f3d-af25-42bb-a259-5ba188ae7103" />
<img width="1349" height="976" alt="Screenshot from 2026-06-11 11-26-43" src="https://github.com/user-attachments/assets/d8482b83-6859-474e-be5f-a96427907b68" />

### Power Connection

The robot is powered by a **14.8V LiPo battery** stepped down via a buck converter to supply the motor driver (MD52A) and logic components at 5V.

| Component | Connection |
|---|---|
| Battery (+) | Buck converter input (`VB+`) |
| Battery (−) | Buck converter input (`VB−`) |
| Buck converter output | Motor driver `M+` / `M−` and 5V rail (`MD.5V`) |
| Motor driver M1A/M1B | ESP32 GPIO D27, D14 (Right motor) |
| Motor driver M2A/M2B | ESP32 GPIO D25, D26 (Left motor) |
| Motor driver M1A/M1B | ESP32 GPIO D35, D32 (Right motor encoder) |
| Motor driver M2A/M2B | ESP32 GPIO D33, D34 (Left motor encoder) |

### USB Connection

All peripheral devices connect to the **Raspberry Pi** via USB:

| Device | Connection |
|---|---|
| ESP32 (Mainboard) | USB → Raspberry Pi |
| LIDAR | USB → Raspberry Pi |
| Power Bank | USB power → Raspberry Pi |

The Raspberry Pi also provides an **HDMI port** for display output and separate **power ports** for the LIDAR and ESP32 power supplies.

---

## Camera Setup (v4l2_camera)

### 1. Install Required Packages

```bash
sudo apt-get install ros-humble-v4l2-camera raspi-config ros-humble-image-transport-plugins v4l-utils
```

| Package | Purpose |
|---|---|
| `ros-humble-v4l2-camera` | Publishes camera output as a ROS2 topic |
| `raspi-config` | Configures camera device connections on Raspberry Pi |
| `ros-humble-image-transport-plugins` | Converts `image_raw` to compressed images for smoother transmission |
| `v4l-utils` | Utility for camera device inspection and connection |

---

### 2. Run raspi-config

The `v4l2_camera` package uses the **legacy camera driver**. You must enable it via `raspi-config`.

> ⚠️ **Warning:** Once the legacy driver is enabled, the `camera-ros` package will no longer detect the camera. To switch back, you must disable the legacy driver again (see Step 4).

```bash
sudo raspi-config
```

**Step 1 — Navigate to Interface Options:**

<img width="1086" height="329" alt="rpi_config1" src="https://github.com/user-attachments/assets/64793d3d-e266-4550-b23a-7ee35596c272" />


Select **3 Interface Options** → *Configure connections to peripherals*.

**Step 2 — Enable Legacy Camera:**

<img width="1086" height="329" alt="rpi_config2" src="https://github.com/user-attachments/assets/5ec02ec2-77d8-4233-918e-edfe216a3346" />


Select **I1 Legacy Camera** → choose **Enable** to activate legacy camera support (`bcm2835 MMAL` driver).

---

### 3. Enable Legacy Camera Stack in config.txt

Open the Raspberry Pi boot configuration file:

```bash
sudo nano /boot/firmware/config.txt
```

Add or modify the following lines:

```ini
# Disable libcamera auto detect
camera_auto_detect=0

# Enable legacy camera stack for bcm2835-v4l2
start_x=1
```

> **To revert to `camera-ros`:** Remove or comment out `camera_auto_detect=0` and `start_x=1` from this file.

---

### 4. Reboot the System

```bash
sudo reboot
```

---

### 5. Verify Camera Device

After rebooting, check that the camera is detected:

```bash
v4l2-ctl --list-devices
```

Expected output will show a device name like `mmal_service_16.1`. Note the `/dev/videoX` path listed under it (e.g. `/dev/video0`).

---

### 6. Run the Camera Node

```bash
ros2 run v4l2_camera v4l2_camera_node
```

By default the node publishes on `/image_raw`. To specify a device explicitly:

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
```

---

### 7. View Camera Input (Remote PC)

On the **Remote PC**, use `rqt_image_view` to verify the camera stream:

```bash
rqt_image_view
```

Select the `/image_raw` (or `/image_raw/compressed`) topic from the dropdown.

---

## Troubleshooting

**Symlinks not appearing after udev trigger**
```bash
# Check if the rule file has correct syntax
sudo udevadm test $(udevadm info -q path -n /dev/ttyUSB0)
```

**ESP32 / LIDAR device swap after reboot**
- Ensure both devices are plugged into the **same physical USB ports** every time.
- If using a USB hub, power cycle the hub after plugging in devices before booting.

**xrdp black screen on connect**
```bash
# Check if a .xsession file exists and is correct
echo "startxfce4" > ~/.xsession   # or your DE of choice
sudo systemctl restart xrdp
```

**Nav2 not launching / BT node errors**
```bash
# Ensure workspace is sourced before ROS2 commands
source /opt/ros/humble/setup.bash
source ~/gati_bot_ws/install/setup.bash
```

**Camera not detected after reboot**
- Confirm `camera_auto_detect=0` and `start_x=1` are present in `/boot/firmware/config.txt`.
- Run `v4l2-ctl --list-devices` to check if the device appears.
- If the device is missing, re-run `raspi-config` and confirm Legacy Camera is enabled.

---

*Maintained by Ayush Bhandurge — ayushjbhandurge@gmail.com*  
*Robot: GatiBot / Smorphi | Platform: ROS2 Humble | SVR Robotics Pvt Ltd, Pune*
