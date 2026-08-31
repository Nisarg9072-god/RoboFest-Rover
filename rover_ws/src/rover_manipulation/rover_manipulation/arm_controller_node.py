#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ArmControllerNode(Node):
    """
    Drives the 4 DOF robotic arm.
    Stub implementation pending exact hardware specs (e.g. PCA9685 vs Dynamixel).
    """
    def __init__(self):
        super().__init__('arm_controller_node')
        self.get_logger().info('Arm Controller Node started (Stub)')

    def move_to_pose(self, pose_name):
        self.get_logger().info(f'Moving arm to {pose_name}')

def main(args=None):
    rclpy.init(args=args)
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
