#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__('local_planner_node')
        self.get_logger().info('Local Planner Node (DWA) started - Stub for Nav2 integration')

def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
