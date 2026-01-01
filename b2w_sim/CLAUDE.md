# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS 2 workspace for simulating the B2W (Bipedal-to-Wheeled) quadrupedal robot in Gazebo. The robot uses neural network-based locomotion control with ONNX Runtime for inference. Developed and tested on ROS 2 Jazzy.

## Build Commands

```bash
# Standard build (from workspace root)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build specific package
colcon build --packages-select <package_name> --symlink-install

# Clean build
rm -rf build install log
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace after building
source install/setup.bash
```

## Launch Commands

```bash
# Launch robot description only (URDF + robot_state_publisher)
ros2 launch b2w_description_ros2 load.launch.py

# Launch minimal Gazebo simulation
ros2 launch b2w_gazebo_ros2 gazebo.launch.py

# Launch full simulation stack (Gazebo + controllers)
ros2 launch b2w_gazebo_ros2 b2w_gazebo.launch.py

# Launch with RViz and mesh visualization
ros2 launch b2w_gazebo_ros2 b2w_gazebo.launch.py \
  enable_rviz:=true \
  enable_mesh_publisher:=true

# Launch with different environment profile
ros2 launch b2w_gazebo_ros2 b2w_gazebo.launch.py \
  controller_environment_profile:=crouched

# Launch with custom policy file
ros2 launch b2w_gazebo_ros2 b2w_gazebo.launch.py \
  controller_policy_path:=/absolute/path/to/policy.onnx

# Launch with different world
ros2 launch b2w_gazebo_ros2 gazebo.launch.py \
  world_file:=maze.world

# Launch with set_pose service bridge (for RViz panel)
ros2 launch b2w_gazebo_ros2 b2w_gazebo.launch.py \
  enable_set_pose_bridge:=true \
  enable_rviz:=true
```

## Testing

```bash
# Run linting tests (configured in CMakeLists.txt)
colcon test --packages-select b2w_controllers

# View test results
colcon test-result --verbose
```

Note: Copyright and cpplint checks are currently disabled in b2w_controllers/CMakeLists.txt.

## Architecture Overview

### Package Structure

- **b2w_description_ros2**: Robot URDF/Xacro models and 3D meshes (23 DAE files)
- **b2w_gazebo_ros2**: Gazebo launch files and ros_gz_bridge configuration
- **b2w_controllers**: C++ neural network-based high-level controller (ONNX Runtime)
- **b2w_panel_cpp**: RViz2 custom panel plugin for interactive robot control
- **b2w_sim_worlds**: Gazebo world files and environment mesh publisher

### System Data Flow

```
Gazebo Simulation
    ↓↑ (ros_gz_bridge - 46 topics/services)
ROS 2 Topics
    ├→ /joint_states (sensor_msgs/JointState)
    ├→ /dlio/odom_node/odom (nav_msgs/Odometry)
    ├→ /clock, /imu, /camera, /pointcloud
    └→ 16× joint command topics (std_msgs/Float64)
        ↓↑
b2w_controllers Node (C++)
    ├─ ONNX Policy Inference (50 Hz)
    │   ├─ Input: 60 observations (joint states, odometry, commands)
    │   ├─ Hidden: 256-dim LSTM cells
    │   └─ Output: 16 joint actions
    └─ Command Publishing (200 Hz)
        ├─ 12× position commands (hip/thigh/calf)
        └─ 4× velocity commands (foot joints)
```

### Robot Kinematics

The B2W robot has **16 DOF** across 4 legs (FL/FR/RL/RR):
- **Hip joint**: Lateral abduction/adduction
- **Thigh joint**: Forward/backward swing
- **Calf joint**: Knee flexion/extension
- **Foot joint**: Wheel rotation (velocity-controlled)

Joint ordering in arrays follows: `[FL, FR, RL, RR] × [hip, thigh, calf, foot]`

### Key Configuration Files

**b2w_controllers/config/b2w_controllers.yaml**
- Policy file path (default: `policy/policy_force_new.onnx`)
- 16 joint names and command topic mappings
- Default joint positions and scales
- Environment profiles: "default" and "crouched"
- Topic subscriptions: odometry, cmd_vel, joint_states

**b2w_gazebo_ros2/config/b2w_gz_bridge.yaml**
- 46 topic/service bridges between Gazebo and ROS 2
- Message type mappings (e.g., gz.msgs.Odometry → nav_msgs/Odometry)
- Camera/sensor bridges (depth, RGB, IMU, force/torque)

**b2w_description_ros2/xacro/**
- `robot.xacro`: Main robot assembly
- `leg.xacro`: Leg macro (instantiated 4 times)
- `const.xacro`: Physical constants and dimensions
- `gazebo.xacro`: Gazebo plugins (odometry, joint states, sensors)

### ONNX Runtime Integration

The b2w_controllers package uses a **bundled ONNX Runtime** library:
- Location: `b2w_controllers/third_party/onnxruntime/`
- Library: `lib64/libonnxruntime.so`
- Headers: `include/onnxruntime_cxx_api.h`
- Not system-installed, linked directly via CMakeLists.txt

When modifying the controller:
1. Neural network policies are in `b2w_controllers/src/policy/*.onnx`
2. Active policy: `policy_force_new.onnx` (configurable in YAML)
3. Conversion scripts available: `pt_to_onnx.py`, `new_onnx.py`
4. Policy input size: 60, hidden state: 256, output: 16 actions

### ros_gz_bridge Critical Points

The bridge configuration maps Gazebo topics to ROS 2:
- **Joint commands**: 16 separate Float64 topics (12 position + 4 velocity)
- **Odometry**: `/model/b2w/odometry` (Gazebo) → `/dlio/odom_node/odom` (ROS)
- **Joint states**: Aggregated from Gazebo into single `/joint_states` topic
- **Clock**: `/clock` must be bridged for `use_sim_time: true` to work

**Important**: All nodes must set `use_sim_time: true` for synchronized simulation time.

### Launch File Architecture

**b2w_gazebo.launch.py** orchestrates the full stack:
1. Includes `gazebo.launch.py` (starts Gazebo + spawns robot)
2. Launches `b2w_controllers` node (neural network controller)
3. Static TF: map → odom transform
4. Optional components (via arguments):
   - `enable_low_level_controller`: Low-level joint controller
   - `enable_mesh_publisher`: Environment visualization in RViz
   - `enable_set_pose_bridge`: Service for RViz panel robot repositioning
   - `enable_rviz`: RViz2 with demo configuration

### World Files

Located in `b2w_sim_worlds/worlds/`:
- `ISAACLAB_TRAIN.world` (default): Complex training environment
- `empty.world`: Flat ground plane
- `maze.world`: Maze navigation
- `playground.world`: Various obstacles
- `warehouse.world`: Indoor environment

Specify via: `ros2 launch b2w_gazebo_ros2 gazebo.launch.py world_file:=<name>.world`

## Development Patterns

### Modifying Robot Description

1. Edit Xacro files in `b2w_description_ros2/xacro/`
2. Rebuild package: `colcon build --packages-select b2w_description_ros2 --symlink-install`
3. URDFs are generated at build time from Xacro templates
4. With `--symlink-install`, Xacro changes are picked up without reinstall

### Changing Neural Network Policy

1. Place new `.onnx` file in `b2w_controllers/src/policy/`
2. Update `b2w_controllers.yaml`: `policy.relative_path: policy/<your_policy>.onnx`
3. OR use launch argument: `controller_policy_path:=/absolute/path/to/policy.onnx`
4. Policy must have: 60 inputs, 16 outputs, LSTM hidden state size 256

### Adding Gazebo-ROS Bridges

Edit `b2w_gazebo_ros2/config/b2w_gz_bridge.yaml`:
```yaml
- topic_name: /new_topic
  ros_type_name: std_msgs/msg/Float64
  gz_type_name: gz.msgs.Double
  direction: GZ_TO_ROS  # or ROS_TO_GZ or BIDIRECTIONAL
```

### Modifying Controller Behavior

Main file: `b2w_controllers/src/b2w_controllers.cpp`
Header: `b2w_controllers/include/b2w_controllers/b2w_controllers.hpp`

Key timing parameters:
- Inference loop: 50 Hz (20ms period) - `inference_timer_`
- Publishing loop: 200 Hz (5ms period) - `publish_timer_`

The controller maintains LSTM hidden states between inferences and applies joint position/velocity scaling before publishing commands.

### RViz Custom Panel Usage

The `b2w_panel_cpp` provides preset buttons (Maze, Pillar, Stairs, Pits) that call Gazebo's `/world/ISAACLAB_TRAIN_world/set_pose` service to reposition the robot. Requires `enable_set_pose_bridge:=true` in the launch file.

## Common Issues

**Simulation time not synchronized**: Ensure all nodes have `use_sim_time: true` in parameters and `/clock` topic is bridged in `b2w_gz_bridge.yaml`.

**ONNX Runtime errors**: The library is bundled in `third_party/onnxruntime/`. If missing, check that the directory exists after cloning. Do not install system ONNX Runtime.

**Joint commands not reaching Gazebo**: Verify all 16 joint command topics are listed in `b2w_gz_bridge.yaml` with correct direction `ROS_TO_GZ`.

**Robot falls through ground**: Check spawn height in `gazebo.launch.py` (default z=2.5m). Ensure Gazebo physics is not paused.

**World file not found**: World files are in `b2w_sim_worlds/worlds/`. Use filename only (e.g., `maze.world`), not full path, when passing to launch argument.
