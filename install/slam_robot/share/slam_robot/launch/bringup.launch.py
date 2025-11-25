from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Main bringup launch file for the slam_robot package.
    """
    # Find packages
    turtlebot3_gazebo_pkg = FindPackageShare("turtlebot3_gazebo")
    slam_toolbox_pkg = FindPackageShare("slam_toolbox")

    # Launch configuration
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # Gazebo with Turtlebot3 launch file
    gazebo_launch_file_path = PathJoinSubstitution(
        [turtlebot3_gazebo_pkg, "launch", "turtlebot3_dqn_stage4.launch.py"]
    )

    # slam_toolbox launch file
    slam_toolbox_launch_file_path = PathJoinSubstitution(
        [slam_toolbox_pkg, "launch", "online_async_launch.py"]
    )

    return LaunchDescription(
        [
            # Launch Gazebo with Turtlebot3
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch_file_path)
            ),
            # Launch slam_toolbox
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_toolbox_launch_file_path),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
        ]
    )
