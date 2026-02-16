# 🤖 LeRobot ROS 2 Bridge

ROS 2 bridge for [LeRobot](https://github.com/huggingface/lerobot) — enabling seamless control of [LeRobot’s mobile base, arm, and sensors](https://github.com/astroyat/lerobot-ros/blob/main/media/lidar.jpg) within the ROS 2 ecosystem.

## 🧭 Architecture Overview

```text
[ Workstation (PC: kabybot) ]                       [ LeKiwi Robot (pirobot.local: 10.0.0.71) ]
 ├── ros2operate.py                                  ├── robot.launch.py
 │    ├─ Subscribes to /cmd_vel (Twist)              │    ├─ LeKiwi Motor Host Daemon (ZMQ: 5555 / 5556)
 │    ├─ Streams Front & Wrist Feeds (OpenCV)        │    ├─ RPLiDAR A1 Driver (/scan on /dev/ttyUSB1)
 │    └─ Bridges Actions to LeKiwi Base via ZMQ      │    ├─ Static TF (base_link -> laser_frame)
 ├── bringup.launch.py (mode:=slam or mode:=nav)    │    └─ Laser Odometry (rf2o -> /odom)
 └── RViz2 (Map & Sensor Visualization)
```

`lerobot_ros` connects LeRobot hardware to ROS 2. It integrates:
- **LIDAR**: `sllidar_ros2` (RPLiDAR A1 on `/dev/ttyUSB1`)
- **Odometry**: `rf2o_laser_odometry`
- **SLAM**: `slam_toolbox`
- **Navigation**: `nav2`
- **Teleoperation**: Direct OpenCV keyboard, `teleop_twist_keyboard`, or Xbox gamepad (`teleop_twist_joy`)

### 🖨️ 3D Printed Hardware Mounts
The rear LIDAR mount files for LeKiwi are provided in [`stl/`](stl/):
- [`back_mount1.stl`](stl/back_mount1.stl)
- [`back_mount2.stl`](stl/back_mount2.stl)
- [`back_mount3.stl`](stl/back_mount3.stl)

See [media/back.jpg](media/back.jpg) and [media/lidar.jpg](media/lidar.jpg) for reference assembly photos.

---

## 🛠️ Installation & Setup

### 1. Operating System & Prerequisites
- **Robot (Raspberry Pi 5)**: Ubuntu Server 24.04 LTS (64-bit) with **ROS 2 Jazzy**.
- **Workstation / PC**: Ubuntu 24.04 LTS with **ROS 2 Jazzy**.
- **LeRobot (ROS 2 branch)**: Clone and checkout the `ros2-latest` branch containing ROS 2 teleoperation and motor driver support:
  ```bash
  cd ~/lerobot
  git checkout ros2-latest
  ```

#### 💡 Conda C++ Library Path Workaround
If running Python scripts within a Conda environment and encountering `libstdc++.so.6` or GLIBCXX symbol mismatch errors with ROS 2 Python packages:
```bash
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
```

### 2. Build the Workspace
Run this on both the robot and the workstation:

```bash
# Source ROS 2 system underlay
source /opt/ros/jazzy/setup.bash

# Create workspace and import dependencies
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/astroyat/lerobot-ros.git
vcs import < lerobot-ros/lerobot_ros.repos

# Build package
cd ~/ros2_ws
colcon build --symlink-install

# Source the workspace overlay
source install/setup.bash
```

---

## 🚀 Usage Guide

### 1. On LeKiwi Robot (`pirobot.local` / `10.0.0.71`)

SSH into the robot and launch all consolidated onboard services (Motor Host Daemon + RPLiDAR A1 + Static TF + Laser Odometry):

```bash
# 1. SSH into LeKiwi
ssh target@10.0.0.71

# 2. Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 3. Launch robot stack
ros2 launch lerobot_ros robot.launch.py serial_port:=/dev/ttyUSB1
```

*(Alternatively, run components individually: in terminal 1, run `cd ~/lerobot && ./examples/lekiwi/lekiwi_host.sh`; in terminal 2, run `source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch lerobot_ros lidar.launch.py serial_port:=/dev/ttyUSB1`).*

*(Optional: Inspect the LiDAR scan directly on the Pi via SSH X11 forwarding:)*
```bash
# On Workstation:
xhost +
ssh -X target@10.0.0.71

# On Pi:
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py
```

---

### 2. On Workstation (`kabybot`)

#### Step A: Teleoperation & Motor Bridge (`ros2operate.py`)
Run `ros2operate.py` to bridge ROS 2 `/cmd_vel` to LeKiwi's motors and view live camera streams:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

cd ~/lerobot
git checkout ros2-latest
python examples/lekiwi/ros2operate.py
```

##### Driving Controls:
- **Direct Keyboard in Camera Window**: Click either OpenCV camera window (`front` or `wrist`) and drive directly:
  - `W` / `S`: Forward / backward
  - `A` / `D`: Strafe left / right
  - `Z` / `X`: Turn counter-clockwise / clockwise
  - `R` / `F`: Increase / decrease speed level
- **ROS 2 Keyboard Node**: In a separate terminal:
  ```bash
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```
- **Xbox Gamepad / Joystick**:
  ```bash
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash
  ros2 run joy joy_node &
  ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
  ```

---

#### Step B: SLAM & Navigation Bringup
In a separate terminal on the workstation, launch SLAM mapping or autonomous navigation (listening to `/scan`, `/odom`, and TF broadcasted by `robot.launch.py`):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Option 1: SLAM Mapping Mode (SLAM Toolbox)
ros2 launch lerobot_ros bringup.launch.py mode:=slam

# Option 2: Autonomous Navigation Mode (Nav2 Stack)
ros2 launch lerobot_ros bringup.launch.py mode:=nav

# Option 3: Odometry & TF Only (if robot.launch.py was started without local odometry)
ros2 launch lerobot_ros bringup.launch.py mode:=idle launch_odom:=true launch_tf:=true
```

*(Note: If the RPLiDAR is connected directly to the workstation instead of the Pi, add `launch_lidar:=true` to automatically start local LiDAR, TF, and laser odometry).*

---

#### Step C: Visualization (RViz2)
Open RViz2 to visualize map, scans, robot pose, and dispatch Nav2 goals:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2
```

> [!IMPORTANT]
> In RViz2, set the `/map` display **Durability Policy** to **`Transient Local`** to properly display the SLAM occupancy grid map.

---

## 🔍 Topic Diagnostics & Verification

Verify inter-node and network topic communications:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Check LiDAR laser scan stream
ros2 topic echo /scan --no-arr

# Check 2D laser odometry
ros2 topic echo /odom --no-arr

# Check SLAM occupancy grid map
ros2 topic echo /map --no-arr

# Check velocity teleop commands
ros2 topic echo /cmd_vel
```
