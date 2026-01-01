# b2w_sim

This repository contains ROS 2 packages for simulating, controlling, and estimating the state of the b2w robot in Gazebo. It includes robot description files, controllers, state estimation, and launch files for running the full simulation stack.

## Quick Description

- **b2w_description_ros2**: Contains the URDF, meshes, and launch files for the b2w robot model.
- **b2w_gazebo_ros2**: Launch files and configuration for running the robot in Gazebo.
- **b2w_low_level_controller_gazebo**: Node for publishing low-level joint commands to Gazebo.
- **b2w_state_estimation_gazebo**: Node for estimating the robot's state from Gazebo simulation data.
- **b2w_controllers / b2w_controllers_python**: High-level and Python-based controllers for the robot.

All packages are licensed under the Apache 2.0 License.

## Getting Started

### 1. Clone the Repository

```bash
# Create a ROS 2 workspace if you don't have one
mkdir -p ~/b2w_ws/src
cd ~/b2w_ws/src

# Clone this repository
# (Replace <repo-url> with the actual URL)
git clone <repo-url> b2w_sim
```

### 2. Install Dependencies

Make sure you have ROS 2 installed (Developed on Jazzy). Install any additional dependencies as needed (see each package.xml for details).

**Install the ROS-Gazebo bridge:**  
Replace `<ros-distro>` with your ROS 2 distribution (e.g., `humble`, `iron`, `jazzy`):

```bash
sudo apt install ros-<ros-distro>-ros-gz
```

For example, on ROS Jazzy:

```bash
sudo apt install ros-jazzy-ros-gz
```

### 3. Build the Workspace

From the root of your workspace:

```bash
cd ~/b2w_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 4. Source the Workspace

```bash
source install/setup.bash
```

### 5. Launch the Simulation

Example: To launch the robot description and state publisher:

```bash
ros2 launch b2w_description_ros2 load.launch.py
```

To launch the full Gazebo simulation (edit as needed):

```bash
ros2 launch b2w_gazebo_ros2 b2w_gazebo.launch.py
```

---

