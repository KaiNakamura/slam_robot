from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Main bringup launch file for the slam_robot package.
    """
    # Find the turtlebot3_gazebo package
    turtlebot3_gazebo_pkg = FindPackageShare("turtlebot3_gazebo")

    # See available launch files in:
    # /opt/ros/{ROS_DISTRO}/share/turtlebot3_gazebo/launch
    launch_file_path = PathJoinSubstitution(
        [turtlebot3_gazebo_pkg, "launch", "turtlebot3_dqn_stage4.launch.py"]
    )

    return LaunchDescription(
        [IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file_path))]
    )
