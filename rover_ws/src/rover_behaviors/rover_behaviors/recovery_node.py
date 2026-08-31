#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class RecoveryNode(Node):
    """
    Executes recovery behaviors (e.g. backup, spin) if navigation fails.
    """
    def __init__(self):
        super().__init__('recovery_node')
        self.get_logger().info('Recovery Node started')

def main(args=None):
    rclpy.init(args=args)
    node = RecoveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
