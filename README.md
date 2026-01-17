# TIN3 Bot Description

Multi-robot URDF/SDF system for ROS 2 Humble and Gazebo Harmonic.

## Project Status

**Week 1: Robot Model** - In Progress

## Package Structure

```
tin3_bot/
├── urdf/           # URDF/Xacro robot description files
├── meshes/
│   ├── visual/     # Visual meshes (DAE format)
│   └── collision/  # Simplified collision meshes (STL)
├── launch/         # Launch files
├── config/         # Configuration files (RViz, params)
├── worlds/         # Gazebo world files
├── scripts/        # Test and utility scripts
├── docs/           # Documentation
├── CMakeLists.txt
└── package.xml
```

## Quick Start

```bash
# Build
cd ~/ros2_ws
colcon build --packages-select tin3_bot
source install/setup.bash

# View robot in RViz (after URDF is complete)
ros2 launch tin3_bot view_robot.launch.py
```

## Robot Specifications

- **Type:** Track-based (skid steer)
- **Platform:** ROS 2 Humble + Gazebo Harmonic
- **Multi-robot:** 8-10 simultaneous robots

## Sensors

- 3D LiDAR
- IMU
- GPS/GNSS
- RGB Camera (on pan-tilt gimbal)
- IR Camera (on pan-tilt gimbal)

## Track Simulation Approach

```
┌────────────────────────────────────────────────┐
│  VISUAL LAYER (what you see)                   │
│  ┌──────────────────────────────────────────┐  │
│  │  Chassis mesh + Static track meshes      │  │
│  │  (tracks don't animate/rotate)           │  │
│  └──────────────────────────────────────────┘  │
│                                                │
│  PHYSICS LAYER (what drives motion)            │
│  ┌──────────────────────────────────────────┐  │
│  │  Hidden wheel cylinders (invisible)      │  │
│  │  ○ ○ ○         ○ ○ ○                     │  │
│  │  These rotate and handle collision       │  │
│  └──────────────────────────────────────────┘  │
└────────────────────────────────────────────────┘
```

## Week 1 Progress

- [x] Package structure created
- [ ] STEP file analyzed
- [ ] Meshes exported
- [ ] URDF structure created
- [ ] Collision geometry optimized
- [ ] Physics parameters set
- [ ] Differential drive working
- [ ] Launch files complete
- [ ] Milestone verified
