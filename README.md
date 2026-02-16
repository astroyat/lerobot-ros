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
vcs import < lerobot-ros/lerobot_ros.repos # (Optional: if we add a .repos file later)
# For now, manually clone dependencies if not installed via apt
git clone https://github.com/Slamtec/sllidar_ros2.git
git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 🚀 Usage

### 🟢 Bringup (LIDAR + Odom + TF + SLAM)
Launch everything with a single command:
```bash
ros2 launch lerobot_ros bringup.launch.py
```

This will:
1. Start the LIDAR driver.
2. Publish odometry using laser scans (`rf2o`).
3. Publish the static transform (`base_link` -> `laser`).
4. Start SLAM Toolbox for mapping and localization.

### 🕹️ Teleoperation
To control the robot, run the teleop node in a separate terminal:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 🗺️ Navigation (Nav2)
To use the Navigation stack (path planning):
*(Note: requires SLAM or a map to be running)*

```bash
ros2 launch nav2_bringup navigation_launch.py params_file:=$(ros2 pkg prefix lerobot_ros)/share/lerobot_ros/config/nav2.yaml
```

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
