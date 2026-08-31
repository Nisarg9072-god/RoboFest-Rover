#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        self.pub_front = self.create_publisher(Range, '/ultrasonic/front', 10)
        self.pub_rear = self.create_publisher(Range, '/ultrasonic/rear', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('ultrasonic_node started (stub)')

    def timer_callback(self):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_front_link'
        self.pub_front.publish(msg)
        
        msg_rear = Range()
        msg_rear.header.stamp = msg.header.stamp
        msg_rear.header.frame_id = 'ultrasonic_rear_link'
        self.pub_rear.publish(msg_rear)

        diag = DiagnosticArray()
        diag.header.stamp = msg.header.stamp
        status = DiagnosticStatus(name='ultrasonic_node', message='Running (stub)', level=DiagnosticStatus.OK)
        diag.status.append(status)
        self.diag_pub.publish(diag)

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
