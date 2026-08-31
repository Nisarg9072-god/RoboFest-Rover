"""
rover_hardware.launch.py
========================
Primary launch file for the ROBOFEST autonomous rover running on physical hardware.

Launches (in order):
  1. rover_description  — publishes URDF and static TF transforms
  2. rover_sensors      — all sensor driver nodes
  3. rover_control      — arduino_interface_node, motion_controller_node
  4. rover_localization — sensor_fusion_node (EKF), localization_node
  5. rover_slam         — slam_node (slam_toolbox)
  6. rover_navigation   — global_planner_node, local_planner_node
  7. rover_perception   — perception_node, obstacle_detection_node
  8. rover_behaviors    — behavior_manager_node, mission_manager_node,
                          parking_node, recovery_node
  9. rover_safety       — safety_controller_node

Status: PLANNED — not yet implemented
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch arguments ────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    total_rounds_arg = DeclareLaunchArgument(
        "total_rounds",
        default_value="7",
        description="Number of competition rounds (6 or 7)",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── Package share paths ─────────────────────────────────────────────────
    bringup_pkg   = FindPackageShare("rover_bringup")
    desc_pkg      = FindPackageShare("rover_description")
    sensors_pkg   = FindPackageShare("rover_sensors")
    control_pkg   = FindPackageShare("rover_control")
    loc_pkg       = FindPackageShare("rover_localization")
    slam_pkg      = FindPackageShare("rover_slam")
    nav_pkg       = FindPackageShare("rover_navigation")
    perc_pkg      = FindPackageShare("rover_perception")
    behav_pkg     = FindPackageShare("rover_behaviors")
    safety_pkg    = FindPackageShare("rover_safety")
    manip_pkg     = FindPackageShare("rover_manipulation")

    # ── Sub-launch includes ─────────────────────────────────────────────────
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([desc_pkg, "launch", "description.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sensors_pkg, "launch", "sensors.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([control_pkg, "launch", "control.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([loc_pkg, "launch", "localization.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_pkg, "launch", "slam.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav_pkg, "launch", "navigation.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([perc_pkg, "launch", "perception.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    behaviors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([behav_pkg, "launch", "behaviors.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "total_rounds": LaunchConfiguration("total_rounds"),
        }.items(),
    )

    safety_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([safety_pkg, "launch", "safety.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    manipulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([manip_pkg, "launch", "manipulation.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            total_rounds_arg,
            LogInfo(msg="[rover_bringup] Starting ROBOFEST Rover — HARDWARE mode"),
            description_launch,
            sensors_launch,
            # Allow sensors 2 s to initialise before control
            TimerAction(period=2.0, actions=[control_launch]),
            # Allow serial link to establish before EKF
            TimerAction(period=4.0, actions=[localization_launch]),
            TimerAction(period=5.0, actions=[slam_launch]),
            TimerAction(period=6.0, actions=[navigation_launch]),
            TimerAction(period=6.0, actions=[perception_launch]),
            # Safety must start before behaviours
            TimerAction(period=7.0, actions=[safety_launch]),
            TimerAction(period=7.5, actions=[manipulation_launch]),
            TimerAction(period=8.0, actions=[behaviors_launch]),
            LogInfo(msg="[rover_bringup] All subsystems launched. Awaiting SELF_CHECK."),
        ]
    )
