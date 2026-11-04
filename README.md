# Autonomous Mobile Manipulator

This package targets ROS 2 Jazzy with Gazebo Harmonic. It simulates a differential-drive litter-collection robot with a 4-DOF arm, RGB-D camera, 2D lidar, onboard bin, SLAM Toolbox, Nav2, colour-based litter detection, frontier exploration, and a pick-and-deposit mission state machine.

## Install dependencies

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup ros-jazzy-nav2-simple-commander ros-jazzy-cv-bridge python3-opencv python3-numpy
```

## Build and run

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select autonomous_mobile_manipulator --symlink-install
source install/setup.bash
ros2 launch autonomous_mobile_manipulator mission.launch.py
```

For base, sensor, TF, SLAM, and Nav2 validation without the autonomous mission, launch `sim.launch.py`. The mission starts only after an occupancy map and the `map -> base_footprint` transform are available.

## Design notes

Gazebo detachable-joint plugins make object transfer deterministic: the controller publishes ROS `std_msgs/Empty` messages that the launch file bridges to Gazebo `gz.msgs.Empty` attach/detach topics. The detector publishes the selected object's model name on `/detected_litter_name`; this keeps the mission manager from guessing an object based on collection order.
