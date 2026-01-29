# TIN3 Bot - Launch Guide

Multi-robot simulation for ROS 2 Humble and Gazebo Harmonic.

---

## Project Structure

```
tin3_bot/
├── urdf/                   # Robot description files
│   ├── robot.urdf.xacro    # Main entry point (includes all)
│   ├── chassis.xacro       # Chassis, tracks, wheels
│   ├── gimbal.xacro        # Pan-tilt gimbal
│   ├── camera.xacro        # RGB + IR cameras
│   ├── rifle.xacro         # Rifle holder
│   ├── sensors.xacro       # LiDAR, IMU, GPS
│   ├── macros.xacro        # Inertia calculation macros
│   └── gazebo.xacro        # Gazebo plugins
├── meshes/                 # 3D models (DAE format)
├── launch/                 # Launch files
│   ├── sim.launch.py       # Main simulation launcher
│   ├── spawn_robot.launch.py # Single robot spawner
│   └── view_robot.launch.py  # RViz viewer
├── config/                 # Configuration files
│   ├── ros_gz_bridge.yaml  # ROS-Gazebo topic bridge
│   └── ekf.yaml            # EKF sensor fusion
├── worlds/                 # Gazebo world files
└── .devcontainer/          # Docker setup 
```

---
## Quick Start

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select tin3_bot
source install/setup.bash

# Launch simulation
ros2 launch tin3_bot sim.launch.py

# View robot model only
ros2 launch tin3_bot view_robot.launch.py
```


## Launch Arguments

| Argument | Default | Options | Description |
|----------|---------|---------|-------------|
| `num_robots` | `1` | 1, 2, 4, 8... | Number of robots to spawn |
| `lidar_mode` | `full` | `full`, `half`, `low` | LiDAR resolution |
| `use_ekf` | `false` | `true`, `false` | Enable EKF sensor fusion |
| `world` | `empty_world.sdf` | any `.sdf` file | World file (in worlds/ folder) |

---

## LiDAR Modes

| Mode | Resolution (H×V) | Points/Scan | Update Rate | Use Case |
|------|------------------|-------------|-------------|----------|
| `full` | 192 × 144 | 27,648 | 10 Hz | Single robot |
| `half` | 96 × 72 | 6,912 | 10 Hz | 2-4 robots |
| `low` | 48 × 36 | 1,728 | 5 Hz | 8+ robots |

---

## Launch Examples

### Single Robot

```bash
# Basic (full LiDAR)
ros2 launch tin3_bot sim.launch.py

# With EKF
ros2 launch tin3_bot sim.launch.py use_ekf:=true

# Custom world
ros2 launch tin3_bot sim.launch.py world:=test_world.sdf
```

### Multi-Robot

```bash
# 2 robots
ros2 launch tin3_bot sim.launch.py num_robots:=2 lidar_mode:=half

# 4 robots
ros2 launch tin3_bot sim.launch.py num_robots:=4 lidar_mode:=half

# 8 robots
ros2 launch tin3_bot sim.launch.py num_robots:=8 lidar_mode:=low
```

### Full Combinations

```bash
# 4 robots + EKF + half LiDAR
ros2 launch tin3_bot sim.launch.py num_robots:=4 use_ekf:=true lidar_mode:=half

# 8 robots + custom world + low LiDAR
ros2 launch tin3_bot sim.launch.py num_robots:=8 world:=test_world.sdf lidar_mode:=low

# Everything
ros2 launch tin3_bot sim.launch.py num_robots:=4 use_ekf:=true lidar_mode:=half world:=test_world.sdf
```

---

## Sensors Overview

| Sensor | Topic (single) | Topic (multi) |
|--------|----------------|---------------|
| Odometry | `/odom` | `/robot_XX/odom` |
| IMU | `/imu/data` | `/robot_XX/imu/data` |
| GPS | `/gps/fix` | `/robot_XX/gps/fix` |
| LiDAR | `/scan/points` | `/robot_XX/scan/points` |
| RGB Camera | `/camera/image_raw` | `/robot_XX/camera/image_raw` |
| IR Camera | `/ir/image_raw` | `/robot_XX/ir/image_raw` |
| Joint States | `/joint_states` | `/robot_XX/joint_states` |