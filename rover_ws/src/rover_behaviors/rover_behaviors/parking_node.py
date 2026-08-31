#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class ParkingNode(Node):
    """
    Handles the precise autonomous parking state using ToF sensors.
    """
    def __init__(self):
        super().__init__('parking_node')
        self.get_logger().info('Parking Node started')

def main(args=None):
    rclpy.init(args=args)
    node = ParkingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
