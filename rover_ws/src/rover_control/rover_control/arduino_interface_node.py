#!/usr/bin/env python3
"""
arduino_interface_node.py
=========================
Bridge between ROS 2 and the Arduino Mega 2560 over USB Serial.

Responsibilities:
  - Send structured velocity commands to the Arduino
  - Receive encoder odometry and status packets from the Arduino
  - Publish /odom (nav_msgs/Odometry) from encoder data
  - Publish /battery/status
  - Publish /diagnostics (Arduino health)
  - Implement software-side watchdog and heartbeat

Status: PLANNED — Serial protocol must match arduino_firmware exactly.

Communication protocol:
  RPi → Arduino:  <CMD_VEL,linear_x,angular_z,seq,checksum\\n>
  Arduino → RPi:  <ENC,vl,vm,vr,vrl,vrm,vrr,seq,checksum\\n>
                  <STATUS,batt_mv,estop,armed,wd_ok,seq\\n>
                  <FAULT,code,description\\n>
                  <PONG\\n>
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import serial
import serial.tools.list_ports
import threading
import time
import math

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from builtin_interfaces.msg import Time
import tf2_ros


class ArduinoInterfaceNode(Node):
    """
    Manages the USB serial link to the Arduino Mega 2560.

    Publishes:
        /odom              — nav_msgs/Odometry
        /battery/status    — sensor_msgs/BatteryState
        /diagnostics       — diagnostic_msgs/DiagnosticArray

    Subscribes:
        /cmd_vel_safe      — geometry_msgs/Twist

    Parameters (all TBD — set in rover_control/config/control_params.yaml):
        serial_port        — USB/UART port (e.g. /dev/ttyUSB0 or /dev/ttyACM0)
        baud_rate          — Serial baud rate (default: 115200)
        watchdog_timeout_s — No-command timeout before zero PWM (TBD)
        wheel_base         — Distance between left and right wheel centrelines (TBD m)
        wheel_radius       — Wheel radius (TBD m)
        encoder_ticks_per_rev — Encoder counts per wheel revolution (TBD)
    """

    def __init__(self):
        super().__init__("arduino_interface_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("watchdog_timeout_s", 0.5)
        self.declare_parameter("wheel_base", 0.30)          # TBD metres
        self.declare_parameter("wheel_radius", 0.05)        # TBD metres
        self.declare_parameter("encoder_ticks_per_rev", 1440)  # TBD
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        port      = self.get_parameter("serial_port").value
        baud      = self.get_parameter("baud_rate").value
        self._wb  = self.get_parameter("wheel_base").value
        self._wr  = self.get_parameter("wheel_radius").value
        self._tpr = self.get_parameter("encoder_ticks_per_rev").value
        self._odom_frame = self.get_parameter("odom_frame").value
        self._base_frame = self.get_parameter("base_frame").value

        # ── Odometry state ────────────────────────────────────────────────────
        self._x   = 0.0
        self._y   = 0.0
        self._yaw = 0.0
        self._seq = 0

        # ── Serial connection ─────────────────────────────────────────────────
        self._serial: serial.Serial | None = None
        self._serial_lock = threading.Lock()
        self._connect_serial(port, baud)

        # ── TF broadcaster ────────────────────────────────────────────────────
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Publishers ────────────────────────────────────────────────────────
        self._odom_pub   = self.create_publisher(Odometry,      "/odom",           10)
        self._batt_pub   = self.create_publisher(BatteryState,  "/battery/status", 10)
        self._diag_pub   = self.create_publisher(DiagnosticArray, "/diagnostics",  10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self._cmd_sub = self.create_subscription(
            Twist, "/cmd_vel_safe", self._cmd_vel_callback, 10
        )

        # ── Heartbeat timer (send PING every 0.5 s) ───────────────────────────
        self._heartbeat_timer = self.create_timer(0.5, self._send_heartbeat)

        # ── Diagnostics timer ─────────────────────────────────────────────────
        self._diag_timer = self.create_timer(1.0, self._publish_diagnostics)

        # ── Serial reader thread ──────────────────────────────────────────────
        self._running = True
        self._read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self._read_thread.start()

        self.get_logger().info("arduino_interface_node started. "
                               f"Port: {port}  Baud: {baud}")

    # ── Serial connection ─────────────────────────────────────────────────────

    def _connect_serial(self, port: str, baud: int) -> None:
        """Attempt to open the serial port. Logs error if unavailable."""
        try:
            self._serial = serial.Serial(port, baud, timeout=0.05)
            time.sleep(2.0)  # Allow Arduino to reset after serial open
            self.get_logger().info(f"Serial connected on {port} at {baud} baud.")
        except serial.SerialException as exc:
            self.get_logger().error(
                f"Cannot open serial port {port}: {exc}\n"
                "Verify Arduino is connected and /dev/ttyACM0 is correct.\n"
                "Check: ls /dev/ttyACM* or ls /dev/ttyUSB*"
            )
            self._serial = None

    # ── Command callback ──────────────────────────────────────────────────────

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """
        Receive /cmd_vel_safe and send CMD_VEL packet to Arduino.

        Packet format:  <CMD_VEL,linear_x,angular_z,seq,checksum\\n>
        """
        if self._serial is None or not self._serial.is_open:
            return

        self._seq = (self._seq + 1) % 65536
        payload = f"CMD_VEL,{msg.linear.x:.4f},{msg.angular.z:.4f},{self._seq}"
        chk = self._checksum(payload)
        packet = f"<{payload},{chk}>\n".encode("ascii")

        with self._serial_lock:
            try:
                self._serial.write(packet)
            except serial.SerialException as exc:
                self.get_logger().error(f"Serial write error: {exc}")

    # ── Serial read loop (background thread) ─────────────────────────────────

    def _serial_read_loop(self) -> None:
        """
        Continuously read lines from the Arduino and dispatch to parsers.
        Runs in a background daemon thread.
        """
        while self._running:
            if self._serial is None or not self._serial.is_open:
                time.sleep(0.1)
                continue
            try:
                with self._serial_lock:
                    raw = self._serial.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="ignore").strip()
                self._dispatch(line)
            except serial.SerialException as exc:
                self.get_logger().error(f"Serial read error: {exc}")
                time.sleep(0.1)

    def _dispatch(self, line: str) -> None:
        """Parse a single line received from the Arduino."""
        # Strip envelope characters < >
        if line.startswith("<") and line.endswith(">"):
            line = line[1:-1]

        if line.startswith("ENC,"):
            self._parse_encoder(line)
        elif line.startswith("STATUS,"):
            self._parse_status(line)
        elif line.startswith("FAULT,"):
            self.get_logger().warn(f"Arduino FAULT: {line}")
        elif line == "PONG":
            pass  # Heartbeat acknowledged
        else:
            self.get_logger().debug(f"Unknown Arduino line: {line}")

    # ── Encoder parser → /odom ────────────────────────────────────────────────

    def _parse_encoder(self, line: str) -> None:
        """
        Parse ENC packet and publish /odom.

        Expected format:
            ENC,vl_front,vl_mid,vl_rear,vr_front,vr_mid,vr_rear,seq,checksum

        vl_* and vr_* are wheel velocities in rad/s (TBD unit — match firmware).
        """
        parts = line.split(",")
        if len(parts) < 9:
            self.get_logger().warn(f"Malformed ENC packet: {line}")
            return

        try:
            vl = (float(parts[1]) + float(parts[2]) + float(parts[3])) / 3.0  # avg left rad/s
            vr = (float(parts[4]) + float(parts[5]) + float(parts[6])) / 3.0  # avg right rad/s
        except ValueError:
            self.get_logger().warn(f"ENC parse error: {line}")
            return

        # Convert wheel velocities → robot body velocity
        v_linear  = self._wr * (vl + vr) / 2.0
        v_angular = self._wr * (vr - vl) / self._wb

        # Integrate pose (simple Euler integration)
        # In production: use dt from actual timestamps
        dt = 0.02  # TBD — use actual time delta
        self._x   += v_linear * math.cos(self._yaw) * dt
        self._y   += v_linear * math.sin(self._yaw) * dt
        self._yaw += v_angular * dt

        # Publish Odometry message
        now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id  = self._base_frame

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        cy = math.cos(self._yaw * 0.5)
        sy = math.sin(self._yaw * 0.5)
        odom.pose.pose.orientation.w = cy
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy

        odom.twist.twist.linear.x  = v_linear
        odom.twist.twist.angular.z = v_angular

        # Covariance — TBD, set conservative placeholders
        odom.pose.covariance[0]  = 0.01   # x
        odom.pose.covariance[7]  = 0.01   # y
        odom.pose.covariance[35] = 0.01   # yaw
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.01

        self._odom_pub.publish(odom)

        # Broadcast odom → base_link TF
        tf = TransformStamped()
        tf.header.stamp    = now
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id  = self._base_frame
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = odom.pose.pose.orientation
        self._tf_broadcaster.sendTransform(tf)

    # ── Status parser ─────────────────────────────────────────────────────────

    def _parse_status(self, line: str) -> None:
        """
        Parse STATUS packet and publish /battery/status.

        Expected format:
            STATUS,battery_mv,estop_state,motor_armed,watchdog_ok,seq
        """
        parts = line.split(",")
        if len(parts) < 6:
            return
        try:
            batt_mv   = float(parts[1])
            estop     = parts[2] == "1"
            armed     = parts[3] == "1"
            wd_ok     = parts[4] == "1"
        except (ValueError, IndexError):
            return

        batt = BatteryState()
        batt.header.stamp = self.get_clock().now().to_msg()
        batt.voltage = batt_mv / 1000.0  # convert mV → V
        batt.present = True
        self._batt_pub.publish(batt)

        if estop:
            self.get_logger().warn("Arduino reports HARDWARE E-STOP active!")
        if not wd_ok:
            self.get_logger().error("Arduino watchdog NOT OK — serial may have been interrupted.")

    # ── Heartbeat ─────────────────────────────────────────────────────────────

    def _send_heartbeat(self) -> None:
        """Send PING to Arduino. Arduino must respond with PONG."""
        if self._serial is None or not self._serial.is_open:
            return
        with self._serial_lock:
            try:
                self._serial.write(b"<PING>\n")
            except serial.SerialException:
                pass

    # ── Diagnostics ───────────────────────────────────────────────────────────

    def _publish_diagnostics(self) -> None:
        """Publish Arduino interface health to /diagnostics."""
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = "arduino_interface"
        status.hardware_id = "Arduino Mega 2560"

        if self._serial is not None and self._serial.is_open:
            status.level   = DiagnosticStatus.OK
            status.message = "Serial connected"
        else:
            status.level   = DiagnosticStatus.ERROR
            status.message = "Serial NOT connected"

        arr.status.append(status)
        self._diag_pub.publish(arr)

    # ── Checksum ──────────────────────────────────────────────────────────────

    @staticmethod
    def _checksum(payload: str) -> int:
        """Simple XOR checksum over ASCII bytes."""
        chk = 0
        for c in payload:
            chk ^= ord(c)
        return chk

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._running = False
        if self._serial and self._serial.is_open:
            # Send DISARM before closing
            try:
                self._serial.write(b"<DISARM>\n")
            except Exception:
                pass
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
