from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='rover_manipulation', executable='arm_controller_node', name='arm_controller_node', output='screen'),
        Node(package='rover_manipulation', executable='gripper_controller_node', name='gripper_controller_node', output='screen'),
        Node(package='rover_manipulation', executable='manipulation_manager_node', name='manipulation_manager_node', output='screen')
    ])
