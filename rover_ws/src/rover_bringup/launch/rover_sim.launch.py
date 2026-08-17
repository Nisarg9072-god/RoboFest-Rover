"""
rover_sim.launch.py
===================
Launch file for Gazebo simulation mode.

Launches:
  - Gazebo with the competition world
  - robot_state_publisher (URDF)
  - All ROS 2 navigation / perception nodes (same as hardware)
  - RViz2 for visualisation

Status: PLANNED — URDF and Gazebo world must be created first.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Always true in simulation mode",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="competition_track.world",
        description="Gazebo world file name (must exist in rover_description/worlds/)",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    desc_pkg  = FindPackageShare("rover_description")
    bringup_pkg = FindPackageShare("rover_bringup")

    # Start Gazebo with the competition world
    gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            PathJoinSubstitution([desc_pkg, "worlds", LaunchConfiguration("world")]),
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    # Spawn the rover URDF into Gazebo
    spawn_rover = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "rover",
            "-x", "0.0", "-y", "0.0", "-z", "0.1",
        ],
        output="screen",
    )

    # robot_state_publisher
    robot_state_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([desc_pkg, "launch", "description.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # RViz2
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([bringup_pkg, "config", "rover.rviz"])],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            world_arg,
            LogInfo(msg="[rover_sim] Starting ROBOFEST Rover — SIMULATION mode"),
            gazebo,
            robot_state_pub,
            TimerAction(period=3.0, actions=[spawn_rover]),
            TimerAction(period=5.0, actions=[rviz2]),
        ]
    )
