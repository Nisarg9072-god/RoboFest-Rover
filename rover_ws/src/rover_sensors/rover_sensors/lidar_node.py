#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('lidar_node started (stub)')

    def timer_callback(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_link'
        self.publisher_.publish(msg)

        diag = DiagnosticArray()
        diag.header.stamp = msg.header.stamp
        status = DiagnosticStatus(name='lidar_node', message='Running (stub)', level=DiagnosticStatus.OK)
        diag.status.append(status)
        self.diag_pub.publish(diag)

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
