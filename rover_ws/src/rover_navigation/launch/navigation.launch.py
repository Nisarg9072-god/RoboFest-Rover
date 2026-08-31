import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_bringup')
    nav2_params_path = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    # Normally we would include nav2_bringup launch here.
    # For now, we will define the stubs for global and local planners.
    
    global_planner = Node(
        package='rover_navigation',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen'
    )
    
    local_planner = Node(
        package='rover_navigation',
        executable='local_planner_node',
        name='local_planner_node',
        output='screen'
    )

    return LaunchDescription([
        global_planner,
        local_planner
    ])
