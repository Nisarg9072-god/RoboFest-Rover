#!/usr/bin/env python3
"""
safety_controller_node.py
=========================
Final velocity gate before the Arduino Mega.

Responsibilities:
  - Subscribe to /cmd_vel (from motion_controller or parking_node)
  - Monitor ToF, ultrasonic, IMU, battery, and Arduino diagnostics
  - Enforce velocity limits (max speed, max acceleration, max angular rate)
  - Trigger software emergency stop when sensor thresholds are breached
  - Publish /cmd_vel_safe (validated, limited velocity)
  - Publish /emergency_stop (bool)

Safety principle:
  Safety ALWAYS overrides navigation.
  If in doubt → STOP.
  The rover must fail SAFE, not fail MOVING.

Status: PLANNED — distance thresholds must be tuned on hardware.
"""

import rclpy
from rclpy.node import Node
import math
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, Imu, BatteryState
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


class SafetyControllerNode(Node):
    """
    Monitors all safety-critical sensors and enforces velocity limits.

    Subscribers:
        /cmd_vel              — geometry_msgs/Twist  (commanded velocity)
        /tof/front_left       — sensor_msgs/Range
        /tof/front_center     — sensor_msgs/Range
        /tof/front_right      — sensor_msgs/Range
        /ultrasonic/front     — sensor_msgs/Range
        /ultrasonic/rear      — sensor_msgs/Range
        /imu/data             — sensor_msgs/Imu
        /battery/status       — sensor_msgs/BatteryState

    Publishers:
        /cmd_vel_safe         — geometry_msgs/Twist
        /emergency_stop       — std_msgs/Bool
        /diagnostics          — diagnostic_msgs/DiagnosticArray
    """

    def __init__(self):
        super().__init__("safety_controller_node")

        # ── Parameters (loaded from safety_params.yaml) ───────────────────────
        self.declare_parameter("max_linear_velocity", 0.5)
        self.declare_parameter("max_angular_velocity", 1.0)
        self.declare_parameter("max_linear_acceleration", 0.3)
        self.declare_parameter("slow_linear_velocity", 0.15)
        self.declare_parameter("estop_distance_m", 0.15)
        self.declare_parameter("slow_zone_distance_m", 0.50)
        self.declare_parameter("warning_zone_distance_m", 1.00)
        self.declare_parameter("sensor_timeout_tof_s", 0.5)
        self.declare_parameter("sensor_timeout_ultrasonic_s", 1.0)
        self.declare_parameter("sensor_timeout_imu_s", 0.5)
        self.declare_parameter("cmd_vel_timeout_s", 0.3)
        self.declare_parameter("battery_low_v", 21.0)
        self.declare_parameter("battery_critical_v", 19.2)
        self.declare_parameter("max_roll_deg", 30.0)
        self.declare_parameter("max_pitch_deg", 30.0)

        self._max_lin    = self.get_parameter("max_linear_velocity").value
        self._max_ang    = self.get_parameter("max_angular_velocity").value
        self._max_acc    = self.get_parameter("max_linear_acceleration").value
        self._slow_lin   = self.get_parameter("slow_linear_velocity").value
        self._estop_d    = self.get_parameter("estop_distance_m").value
        self._slow_d     = self.get_parameter("slow_zone_distance_m").value
        self._warn_d     = self.get_parameter("warning_zone_distance_m").value
        self._batt_low   = self.get_parameter("battery_low_v").value
        self._batt_crit  = self.get_parameter("battery_critical_v").value
        self._max_roll   = math.radians(self.get_parameter("max_roll_deg").value)
        self._max_pitch  = math.radians(self.get_parameter("max_pitch_deg").value)

        # ── Sensor state ──────────────────────────────────────────────────────
        self._tof_left   = float("inf")
        self._tof_center = float("inf")
        self._tof_right  = float("inf")
        self._us_front   = float("inf")
        self._us_rear    = float("inf")
        self._roll       = 0.0
        self._pitch      = 0.0
        self._battery_v  = float("inf")

        # Watchdog timestamps
        self._last_cmd_time  = self.get_clock().now()
        self._last_tof_time  = self.get_clock().now()
        self._last_us_time   = self.get_clock().now()
        self._last_imu_time  = self.get_clock().now()

        # Previous velocity (for acceleration limiting)
        self._prev_linear  = 0.0
        self._prev_time    = self.get_clock().now()

        # Emergency stop state
        self._estop_active = False

        # ── Publishers ────────────────────────────────────────────────────────
        self._cmd_pub   = self.create_publisher(Twist,          "/cmd_vel_safe",   10)
        self._estop_pub = self.create_publisher(Bool,           "/emergency_stop", 10)
        self._diag_pub  = self.create_publisher(DiagnosticArray, "/diagnostics",   10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Twist,        "/cmd_vel",           self._cmd_cb,    10)
        self.create_subscription(Range,        "/tof/front_left",    self._tof_l_cb,  10)
        self.create_subscription(Range,        "/tof/front_center",  self._tof_c_cb,  10)
        self.create_subscription(Range,        "/tof/front_right",   self._tof_r_cb,  10)
        self.create_subscription(Range,        "/ultrasonic/front",  self._us_f_cb,   10)
        self.create_subscription(Range,        "/ultrasonic/rear",   self._us_r_cb,   10)
        self.create_subscription(Imu,          "/imu/data",          self._imu_cb,    10)
        self.create_subscription(BatteryState, "/battery/status",    self._batt_cb,   10)

        # ── Watchdog timer (runs at 20 Hz) ────────────────────────────────────
        self.create_timer(0.05, self._watchdog_check)

        # ── Diagnostics timer ─────────────────────────────────────────────────
        self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info("safety_controller_node started.")

    # ── Sensor callbacks ──────────────────────────────────────────────────────

    def _tof_l_cb(self, msg: Range):
        self._tof_left  = msg.range
        self._last_tof_time = self.get_clock().now()

    def _tof_c_cb(self, msg: Range):
        self._tof_center = msg.range
        self._last_tof_time = self.get_clock().now()

    def _tof_r_cb(self, msg: Range):
        self._tof_right = msg.range
        self._last_tof_time = self.get_clock().now()

    def _us_f_cb(self, msg: Range):
        self._us_front = msg.range
        self._last_us_time = self.get_clock().now()

    def _us_r_cb(self, msg: Range):
        self._us_rear = msg.range
        self._last_us_time = self.get_clock().now()

    def _imu_cb(self, msg: Imu):
        """Extract roll and pitch from quaternion."""
        q = msg.orientation
        # Roll (x-axis rotation)
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self._roll = math.atan2(sinr, cosr)
        # Pitch (y-axis rotation)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self._pitch = math.asin(sinp)
        self._last_imu_time = self.get_clock().now()

    def _batt_cb(self, msg: BatteryState):
        self._battery_v = msg.voltage

    # ── Main command callback ─────────────────────────────────────────────────

    def _cmd_cb(self, msg: Twist) -> None:
        """
        Receive /cmd_vel, apply all safety checks, publish /cmd_vel_safe.
        """
        self._last_cmd_time = self.get_clock().now()

        if self._estop_active:
            self._publish_zero()
            return

        # ── Determine minimum forward obstacle distance ────────────────────────
        min_front = min(self._tof_left, self._tof_center, self._tof_right, self._us_front)
        linear_x  = msg.linear.x
        angular_z = msg.angular.z

        # ── Estop zone ────────────────────────────────────────────────────────
        if linear_x > 0 and min_front < self._estop_d:
            self.get_logger().warn(
                f"ESTOP: front obstacle at {min_front:.2f} m < {self._estop_d:.2f} m"
            )
            self._trigger_estop()
            return

        # ── Slow zone: cap forward speed ──────────────────────────────────────
        if linear_x > 0 and min_front < self._slow_d:
            linear_x = min(linear_x, self._slow_lin)

        # ── Velocity caps ──────────────────────────────────────────────────────
        linear_x  = max(-self._max_lin,  min(self._max_lin,  linear_x))
        angular_z = max(-self._max_ang,  min(self._max_ang,  angular_z))

        # ── Acceleration limiting ──────────────────────────────────────────────
        now = self.get_clock().now()
        dt  = (now - self._prev_time).nanoseconds * 1e-9
        dt  = max(dt, 0.001)
        acc = (linear_x - self._prev_linear) / dt
        if abs(acc) > self._max_acc:
            linear_x = self._prev_linear + math.copysign(self._max_acc * dt, acc)
        self._prev_linear = linear_x
        self._prev_time   = now

        # ── Tilt check ────────────────────────────────────────────────────────
        if abs(self._roll) > self._max_roll or abs(self._pitch) > self._max_pitch:
            self.get_logger().error(
                f"Tilt exceeded: roll={math.degrees(self._roll):.1f}° "
                f"pitch={math.degrees(self._pitch):.1f}° — STOP"
            )
            self._trigger_estop()
            return

        # ── Battery critical ──────────────────────────────────────────────────
        if self._battery_v < self._batt_crit:
            self.get_logger().error(
                f"Battery CRITICAL: {self._battery_v:.2f} V — STOP"
            )
            self._trigger_estop()
            return

        if self._battery_v < self._batt_low:
            linear_x = min(linear_x, self._slow_lin)
            self.get_logger().warn(f"Battery LOW: {self._battery_v:.2f} V — reducing speed")

        # ── Publish validated command ─────────────────────────────────────────
        safe = Twist()
        safe.linear.x  = linear_x
        safe.angular.z = angular_z
        self._cmd_pub.publish(safe)

    # ── Watchdog ──────────────────────────────────────────────────────────────

    def _watchdog_check(self) -> None:
        """
        Check for command timeout. If /cmd_vel has not been received
        within cmd_vel_timeout_s, publish zero velocity.
        """
        cmd_timeout = self.get_parameter("cmd_vel_timeout_s").value
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if elapsed > cmd_timeout and not self._estop_active:
            self._publish_zero()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _trigger_estop(self) -> None:
        self._estop_active = True
        self._publish_zero()
        msg = Bool()
        msg.data = True
        self._estop_pub.publish(msg)

    def _publish_zero(self) -> None:
        self._cmd_pub.publish(Twist())  # all zeros

    def _publish_diagnostics(self) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        s = DiagnosticStatus()
        s.name = "safety_controller"
        s.hardware_id = "safety"
        if self._estop_active:
            s.level   = DiagnosticStatus.ERROR
            s.message = "EMERGENCY STOP ACTIVE"
        else:
            s.level   = DiagnosticStatus.OK
            s.message = "Nominal"
        s.values = [
            KeyValue(key="tof_min_m",   value=f"{min(self._tof_left, self._tof_center, self._tof_right):.3f}"),
            KeyValue(key="us_front_m",  value=f"{self._us_front:.3f}"),
            KeyValue(key="roll_deg",    value=f"{math.degrees(self._roll):.1f}"),
            KeyValue(key="pitch_deg",   value=f"{math.degrees(self._pitch):.1f}"),
            KeyValue(key="battery_v",   value=f"{self._battery_v:.2f}"),
            KeyValue(key="estop",       value=str(self._estop_active)),
        ]
        arr.status.append(s)
        self._diag_pub.publish(arr)


# Needed for KeyValue import
from diagnostic_msgs.msg import KeyValue


def main(args=None):
    rclpy.init(args=args)
    node = SafetyControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
