from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    """
    Main bringup launch file for the slam_robot package.
    """
    # Find packages
    turtlebot3_gazebo_pkg = FindPackageShare("turtlebot3_gazebo")
    slam_toolbox_pkg = FindPackageShare("slam_toolbox")
    nav2_bringup_pkg = FindPackageShare("nav2_bringup")
    slam_robot_pkg = FindPackageShare("slam_robot")

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

    # Nav2 navigation launch file
    nav2_navigation_launch_file_path = PathJoinSubstitution(
        [nav2_bringup_pkg, "launch", "navigation_launch.py"]
    )

    # Custom RViz config
    rviz_config_path = PathJoinSubstitution([slam_robot_pkg, "rviz", "slam_robot.rviz"])

    # Frontier nodes with SetParameter + GroupAction pattern
    # This ensures use_sim_time is properly set before nodes start
    frontier_nodes_group = GroupAction(
        actions=[
            SetParameter("use_sim_time", use_sim_time),
            Node(
                package="slam_robot",
                executable="frontier_server",
                name="frontier_server",
                output="screen",
            ),
            Node(
                package="slam_robot",
                executable="frontier_explorer",
                name="frontier_explorer",
                output="screen",
            ),
            Node(
                package="slam_robot",
                executable="frontier_visualizer",
                name="frontier_visualizer",
                output="screen",
            ),
        ]
    )

    return LaunchDescription(
        [
            # Launch Gazebo with Turtlebot3
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch_file_path),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            # Launch slam_toolbox
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_toolbox_launch_file_path),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "slam_params_file": PathJoinSubstitution(
                        [slam_robot_pkg, "config", "slam_toolbox_params.yaml"]
                    ),
                }.items(),
            ),
            # Launch Nav2 navigation
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_navigation_launch_file_path),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": PathJoinSubstitution(
                        [slam_robot_pkg, "config", "nav2_params.yaml"]
                    ),
                }.items(),
            ),
            # Launch RViz with custom config
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_path],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            # Frontier exploration nodes (with SetParameter + GroupAction)
            frontier_nodes_group,
        ]
    )
