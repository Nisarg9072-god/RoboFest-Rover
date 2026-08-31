#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class GripperControllerNode(Node):
    """
    Controls the robotic gripper.
    Stub implementation.
    """
    def __init__(self):
        super().__init__('gripper_controller_node')
        self.get_logger().info('Gripper Controller Node started (Stub)')

    def set_gripper(self, closed: bool):
        state = 'CLOSED' if closed else 'OPEN'
        self.get_logger().info(f'Setting gripper to {state}')

def main(args=None):
    rclpy.init(args=args)
    node = GripperControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
