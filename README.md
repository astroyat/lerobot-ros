# 🤖 LeRobot ROS 2 Bridge

ROS 2 bridge for [LeRobot](https://github.com/huggingface/lerobot) — enabling seamless control of [LeRobot’s mobile base, arm, and sensors](https://github.com/astroyat/lerobot-ros/blob/main/media/lidar.jpg) within the ROS 2 ecosystem.

## 🧭 Overview

`lerobot_ros` connects LeRobot hardware to ROS 2. It integrates:
- **LIDAR** (`sllidar_ros2`)
- **Odometry** (`rf2o_laser_odometry`) 
- **SLAM** (`slam_toolbox`)
- **Navigation** (`nav2`)

## 🛠️ Installation

### 1. Prerequisite: ROS 2
Ensure you have ROS 2 installed (Humble or Jazzy).

### 2. Build the Package
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/astroyat/lerobot-ros.git
vcs import < lerobot-ros/lerobot_ros.repos

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 🚀 Usage

### 🟢 Bringup

Launch all required nodes with a single command:

#### 1. Mapping Mode (LIDAR + Odometry + TF + SLAM)
```bash
ros2 launch lerobot_ros bringup.launch.py mode:=slam
```
*(By default `serial_port:=/dev/ttyUSB1`. If your LiDAR is on another port, add `serial_port:=/dev/ttyUSB0`)*

#### 2. Navigation Mode (LIDAR + Odometry + TF + Nav2)
```bash
ros2 launch lerobot_ros bringup.launch.py mode:=nav
```

#### 3. Sensors & Odometry Only
```bash
ros2 launch lerobot_ros bringup.launch.py mode:=idle
```

### 🕹️ Teleoperation
To drive the robot manually via ROS 2:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Make sure `ros2operate.py` or `LeKiwiClient` is running on the client side to receive `/cmd_vel` and forward motor commands to LeKiwi.

## 🧩 Architecture

```text
+-----------------------------+
|        ROS 2 System         |
|-----------------------------|
| /cmd_vel   → Base Driver    |
| /odom      ← Odometry (rf2o)|
| /scan      ← LIDAR Driver   |
| /tf        ↔ Transform Tree |
| /map       ← SLAM Toolbox   |
+-----------------------------+
            │
            ↓
     LeRobot Hardware
```
