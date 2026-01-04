# SLAM Robot

An autonomous mapping robot using SLAM (Simultaneous Localization and Mapping)

## Installation

This repo has been tested on Ubuntu 24.04 LTS.

Install:

- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- [Nav2 packages](https://docs.nav2.org/getting_started/index.html#installation)
- [slam_toolbox](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html)

Also:

```sh
sudo apt install ros-jazzy-nav2-route
```

## Configuration Files

The files under `slam_robot/config` control some of the configuration settings necessary to run the demo. These files do not need to be modified, but the steps below show how they were created.

```sh
# Copy Nav2 parameters
cp /opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml config/

# Copy SLAM Toolbox parameters
cp /opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml config/slam_toolbox_params.yaml
```

After copying Nav2 parameters, ensure `enable_stamped_cmd_vel: true` is set in these sections of `nav2_params.yaml`:

- **`controller_server`** section - Required for controller to publish `TwistStamped` messages
- **`velocity_smoother`** section - Required to receive and forward `TwistStamped` messages
- **`collision_monitor`** section - Required for compatibility with Gazebo's `ros_gz_bridge`, which expects `TwistStamped` messages

## Running

Build and launch the system:

```sh
colcon build --packages-select slam_robot
source install/setup.sh
ros2 launch slam_robot bringup.launch.py
```

## Shutting Down

If you need to manually shut down the system, use these commands:

```sh
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

## ROS Structure

### robot (Gazebo / Turtlebot)

- Publishes
  - /scan
  - /odom
  - /tf - odom -> base_footprint -> base_link -> laser
- Subscribes
  - /cmd_vel

### slam_toolbox

- Subscribes (everything from robot)
  - /scan
  - /tf
  - /odom
- Publishes
  - /map
  - /tf - map -> odom

### Nav2

- Subscribes
  - /map - From slam_toolbox
  - /scan - For obstacle avoidance
  - /tf - For robot pose
- Publishes
  - /cmd_vel
  - /global_costmap/costmap - For RViz
  - /local_costmap/costmap - For RViz
- Actions
  - /navigate_to_pose

### frontier_server

- Subscribes
  - /map
  - /tf - For robot pose
- Publishes
  - /frontiers - Custom `FrontierList` msg

### frontier_explorer

- Subscribes
  - /frontiers
  - /tf - Via tf2 TransformListener
- Actions
  - /navigate_to_pose - Calls Nav2

### frontier_visualizer

- Subscribes
  - /frontiers
  - /map - For resolution
- Publishes
  - /frontier_centroids - MarkerArray
  - /frontier_cells - GridCells

## Docker

TODO: Adapt Docker container for ROS2 Jazzy

This repo has been tested on Ubuntu 24.04, but should work on any machine that can run the included Docker container.

It is recommened to install the following (or equivalent for your system):

- [Docker Engine](https://docs.docker.com/engine/install/ubuntu/#installation-methods)
- [Post-install steps](https://docs.docker.com/engine/install/linux-postinstall/) to run Docker as non-root user
