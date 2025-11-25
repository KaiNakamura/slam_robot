# SLAM Robot

An autonomous mapping robot that uses SLAM (Simultaneous Localization and Mapping)

## Installation

This repo has been tested on Ubuntu 24.04 LTS.

Install [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
Install [Nav2 packages](https://docs.nav2.org/getting_started/index.html#installation)
Install [slam_toolbox](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)

Also:

```bash
sudo apt install ros-jazzy-nav2-route
```

## Configuration Files

```bash
# Copy Nav2 parameters
cp /opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml config/

# Copy SLAM Toolbox parameters
cp /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml config/slam_toolbox_params.yaml
```

## Running

Build and launch the system:

```bash
colcon build --packages-select slam_robot
source install/setup.bash
ros2 launch slam_robot bringup.launch.py
```

## Shutting Down

If you need to manually shut down the system, use these commands:

```bash
# Kill ros2 launch processes
pkill -f "ros2 launch"

# Kill Gazebo processes (normal termination)
pkill -f "gz sim"

# If Gazebo doesn't respond, force kill
pkill -9 -f "gz sim"
pkill -9 -f "ruby.*gz"

# Kill RViz if running
pkill -f "rviz2"
```

Or use ros2 lifecycle commands to gracefully shut down Nav2 nodes:

```bash
# Shutdown Nav2 nodes
ros2 lifecycle set /controller_server shutdown
ros2 lifecycle set /planner_server shutdown
ros2 lifecycle set /recoveries_server shutdown
ros2 lifecycle set /bt_navigator shutdown
ros2 lifecycle set /waypoint_follower shutdown
ros2 lifecycle set /lifecycle_manager_navigation shutdown
```

## Docker

TODO: Adapt Docker container for ROS2 Jazzy
This repo has been tested on Ubuntu 24.04, but should work on any machine that can run the included Docker container.

It is recommened to install the following (or equivalent for your system):

- [Docker Engine](https://docs.docker.com/engine/install/ubuntu/#installation-methods)
- [Post-install steps](https://docs.docker.com/engine/install/linux-postinstall/) to run Docker as non-root user
