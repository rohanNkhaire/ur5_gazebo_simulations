# ur5 Gazebo Simulations
This repo contains files to simulate UR5 in Gazebo for pick and place task

![](media/picknplace_jazzy.gif)

# System Requirements #
- ROS2 Jazzy

# Dependencies #
- CycloneDDS RMW
```bash
sudo apt install ros-jazzy-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

# Installation #
```bash
# Clone the repo
git clone https://github.com/rohanNkhaire/ur5_pick_and_place.git -b jazzy

# Install dependencies
cd ur5_pick_and_place

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -y

# Source ROS2 Jazzy after rosdep install
source /opt/ros/jazzy/setup.bash

# Build the packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

# Gazebo export path #
### **Make sure that you export model and plugin path to use the models and the plugins of this package.** ###
```bash
# Gazebo models
export GZ_SIM_RESOURCE_PATH=<path to this repo>/src/ur5_pick_and_place/models

# Gazebo plugin
export GZ_SIM_SYSTEM_PLUGIN_PATH=<path to this repo>/build/GraspPlugin
```

# Usage #
```bash
# Launches Gazebo and Moveit2
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py

# Run the pick and place script
ros2 launch ur5_moveit_config picknpplace.launch.py executable_script:="picknplace_cube_low"

```
