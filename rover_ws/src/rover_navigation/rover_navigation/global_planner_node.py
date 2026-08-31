#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner_node')
        self.get_logger().info('Global Planner Node (A*) started - Stub for Nav2 integration')

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
