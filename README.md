# ur5_pick_and_place
This repo contains files to simulate UR5 in Gazebo for pick and place task

![](media/picknplace.gif)

# System Requirements #
- ROS1 Noetic

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

# Source ROS2 Jazzy before rosdep install
source /opt/ros/jazzy/setup.bash

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

# Spawn the object in the gazebo world
# Open a new terminal
rosrun gazebo_ros spawn_model -file `rospack find ur_description`/urdf/objects/box.urdf -urdf -x -0.5 -y -0.5 -z 1 -model my_object

# Run the pick and place script
cd gazebo_ros_link_attacher/scripts
python3 picknplace.py
```
