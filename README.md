# 🤖 LeRobot ROS 2 Bridge

ROS 2 bridge for [LeRobot](https://github.com/huggingface/lerobot) — enabling seamless control of [LeRobot’s mobile base, arm, and sensors](https://github.com/astroyat/lerobot-ros/blob/main/media/lidar.jpg) within the ROS 2 ecosystem.

## 🧭 Overview

lerobot-ros connects LeRobot hardware to ROS 2.  
It provides a standard ROS 2 interface for teleoperation, odometry, and (soon) arm control.  
The package supports both Raspberry Pi 5 (on-board control) and desktop PCs (remote or simulation setups).

### ✨ Features

- ✅ Teleoperate LeRobot **mobile base** using `/cmd_vel`
- ✅ Stream and visualize **LIDAR** data
- ✅ Integrate **rf2o_laser_odometry** for SLAM or navigation
- 🚧 **Arm teleoperation** (in development)
- ⚙️ ROS 2 Humble (22.04) and Jazzy (24.04) supported

## 🧩 Architecture

```text
+-----------------------------+
|        ROS 2 System         |
|-----------------------------|
| /cmd_vel   → Base Driver    |
| /odom      ← Odometry       |
| /scan      ← LIDAR Driver   |
| /tf        ↔ Transform Tree |
| /arm/*     ↔ Arm (WIP)      |
+-----------------------------+
            │
            ↓
     LeRobot Hardware
 (Mobile Base + Arm + Sensors)
```

## 🛠️ Installation

### 🐧 Raspberry Pi 5 Setup
#### 1. Install <a href="https://ubuntu.com/download/raspberry-pi">Ubuntu Server 24.04.2 LTS (64-bit)</a>, setup wireless LAN and enable SSH in the rpi-imager, do not select Raspberry Pi OS as this requires running ROS 2 in docker.
#### 2. Install <a href="https://docs.ros.org/en/jazzy/Installation.html">ROS 2 Jazzy</a>.
#### 3. Install <a href="https://github.com/Slamtec/sllidar_ros2">SLAMTEC LIDAR ROS 2</a> or other LIDAR ROS 2 package depending on the LIDAR connected to the Raspberry Pi 5.
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ~/ros2_ws/
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```
#### 4. Modify the USB port (/dev/ttyUSBX) connected to the LIDAR in the sllidar_ros2 launch files (sllidar_a1_launch.py, view_sllidar_a1_launch.py).
#### 5. From the PC, run xhost +, ssh -X to the Raspberry Pi 5 and run the sllidar node to view the laser scan in <a href="media/scan.png">rviz2</a>.
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py
```
#### 6. Install LeRobot on Raspberry Pi 5.
#### 7. Fetch the <a href="https://github.com/astroyat/lerobot/tree/ros2-latest">ROS 2 changes</a>.
```bash
cd ~/lerobot
git checkout ros2-latest
```

### 💻 PC Setup
#### 1. Install <a href="https://docs.ros.org/en/humble/Installation.html">ROS 2 Humble</a> on Ubuntu 22.04.5 LTS on PC (other ROS 2 and Ubuntu do not work with LeRobot due to RCL Python dependency).
#### 2. Install <a href="https://github.com/MAPIRlab/rf2o_laser_odometry">rf2o_laser_odometry</a>.
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git
cd ~/ros2_ws/
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```
#### 3. Modify the odom_topic in the rf2o_laser_odometry launch file (rf2o_laser_odometry.launch.py) from odom_rf2o to odom.
#### 4. Install LeRobot on PC.
#### 5. Fetch the <a href="https://github.com/astroyat/lerobot/tree/ros2-latest">ROS 2 changes</a>.
```bash
cd ~/lerobot
git checkout ros2-latest
```

## 🚀 Usage
### 🕹️ Teleoperate the Mobile Base using ROS 2 /cmd_vel
#### 1. On the Raspberry Pi 5, run the LeRobot remote
```bash
cd ~/lerobot
python3 -m src.lerobot.robots.lekiwi.lekiwi_host
```
#### 2. On the PC, run the LeRobot teleop
```bash
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
cd ~/lerobot
python3 ~/lerobot/examples/lekiwi/ros2operate.py
```
#### 3. On the PC, run the ROS 2 teleop using keyboard
```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
#### 4. Alternatively, run the ROS 2 teleop using xbox joystick
```bash
source /opt/ros/humble/setup.bash
ros2 run joy joy_node &
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```

### 🕹️ Teleoperate the Mobile Base using ROS 2 NAV2 /cmd_vel
#### 1. On the Raspberry Pi 5, run the LeRobot remote
```bash
cd ~/lerobot
python3 -m src.lerobot.robots.lekiwi.lekiwi_host
```
#### 2. On the PC, run the LeRobot teleop
```bash
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
cd ~/lerobot
python3 ~/lerobot/examples/lekiwi/ros2operate.py
```
#### 3. On the Raspberry Pi 5, run the sllidar node in scan mode to publish the laser scan
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch sllidar_ros2 sllidar_a1_launch.py
```
#### 4. On the PC, confirm the scan topic is published
```bash
ros2 topic echo /scan --no-arr
```
#### 5. On the PC, run the rf2o_laser_odometry node to publish the odom
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
```
#### 6. Confirm the odom topic is published
```bash
ros2 topic echo /odom --no-arr
```
#### 7. On the PC, run the ROS 2 transform
```bash
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher -0.1 0 0.3 0 0 0 base_link laser
```
#### 8. On the PC, run the ROS 2 SLAM using <a href="nav2/slam.yaml">slam.yaml</a> to publish the map
```bash
source /opt/ros/humble/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false slam_params_file:=slam.yaml
```
#### 9. Confirm the map topic is published
```bash
ros2 topic echo /map --no-arr
```
#### 10. On the PC, run the ROS 2 NAV2 using <a href="nav2/nav2.yaml">nav2.yaml</a>
```bash
source /opt/ros/humble/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false params_file:=nav2.yaml
```
#### 11. On the PC, run rviz2 to view the <a href="media/map.png">map</a> and set the goal post, refer to various ROS 2 NAV2 tutorial.
```bash
rviz2
```
