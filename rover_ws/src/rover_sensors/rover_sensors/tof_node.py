#!/usr/bin/env python3
"""
tof_node.py
===========
Driver node for 3x VL53L1X Time-of-Flight sensors connected via I²C.

I²C address multiplexing:
  All VL53L1X sensors default to I²C address 0x29.
  To use three sensors on the same bus, each sensor's XSHUT pin
  is held LOW (disabled) while the others are programmed to a unique address.

  Procedure (implemented in _init_sensors()):
    1. Pull all XSHUT pins LOW (disable all sensors)
    2. Enable sensor #1 (pull XSHUT HIGH), assign address 0x29 (default)
    3. Enable sensor #2 (pull XSHUT HIGH), assign address 0x30
    4. Enable sensor #3 (pull XSHUT HIGH), assign address 0x31

  XSHUT GPIO pins: TBD — assign in parameters.

Dependencies:
  pip install vl53l1x
  or use the ST VL53L1X Python library

Status: PLANNED
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import time

# TBD: replace with correct VL53L1X library import for your platform
# Options:
#   from VL53L1X import VL53L1X       (ST official Python library)
#   import board, busio, adafruit_vl53l1x  (Adafruit CircuitPython)
# Uncomment the correct import after verifying the installed library:
# import VL53L1X  # placeholder

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


class ToFNode(Node):
    """
    Reads three VL53L1X ToF sensors via I²C and publishes Range messages.

    Publishers:
        /tof/front_left   — sensor_msgs/Range
        /tof/front_center — sensor_msgs/Range
        /tof/front_right  — sensor_msgs/Range
        /diagnostics      — diagnostic_msgs/DiagnosticArray

    Parameters:
        xshut_pin_left   (int) — GPIO pin for left sensor XSHUT (TBD)
        xshut_pin_center (int) — GPIO pin for center sensor XSHUT (TBD)
        xshut_pin_right  (int) — GPIO pin for right sensor XSHUT (TBD)
        i2c_addr_left    (int) — I²C address for left sensor (default: 0x29)
        i2c_addr_center  (int) — I²C address for center sensor (default: 0x30)
        i2c_addr_right   (int) — I²C address for right sensor (default: 0x31)
        publish_rate_hz  (float) — Sensor polling rate (TBD Hz)
        frame_id_left    (str) — TF frame ID for left sensor
        frame_id_center  (str) — TF frame ID for center sensor
        frame_id_right   (str) — TF frame ID for right sensor
    """

    SENSOR_LABELS = ["front_left", "front_center", "front_right"]
    TOPIC_NAMES   = [
        "/tof/front_left",
        "/tof/front_center",
        "/tof/front_right",
    ]

    def __init__(self):
        super().__init__("tof_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("xshut_pin_left",   17)   # TBD: BCM GPIO number
        self.declare_parameter("xshut_pin_center", 27)   # TBD
        self.declare_parameter("xshut_pin_right",  22)   # TBD
        self.declare_parameter("i2c_addr_left",    0x29)
        self.declare_parameter("i2c_addr_center",  0x30)
        self.declare_parameter("i2c_addr_right",   0x31)
        self.declare_parameter("publish_rate_hz",  20.0)  # TBD
        self.declare_parameter("frame_id_left",    "tof_front_left_link")
        self.declare_parameter("frame_id_center",  "tof_front_center_link")
        self.declare_parameter("frame_id_right",   "tof_front_right_link")

        self._xshut_pins = [
            self.get_parameter("xshut_pin_left").value,
            self.get_parameter("xshut_pin_center").value,
            self.get_parameter("xshut_pin_right").value,
        ]
        self._addrs = [
            self.get_parameter("i2c_addr_left").value,
            self.get_parameter("i2c_addr_center").value,
            self.get_parameter("i2c_addr_right").value,
        ]
        self._frame_ids = [
            self.get_parameter("frame_id_left").value,
            self.get_parameter("frame_id_center").value,
            self.get_parameter("frame_id_right").value,
        ]
        rate = self.get_parameter("publish_rate_hz").value

        # ── Publishers ────────────────────────────────────────────────────────
        self._pubs = [
            self.create_publisher(Range, topic, 10)
            for topic in self.TOPIC_NAMES
        ]
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        # ── Sensor initialisation ─────────────────────────────────────────────
        self._sensors = []  # Will hold sensor objects after _init_sensors()
        self._sensors_ok = False
        self._init_sensors()

        # ── Timer ─────────────────────────────────────────────────────────────
        self.create_timer(1.0 / rate, self._read_and_publish)

        self.get_logger().info(
            f"tof_node started. Sensors OK: {self._sensors_ok}"
        )

    def _init_sensors(self):
        """
        Initialise three VL53L1X sensors via XSHUT multiplexing.

        TBD: Replace placeholder implementation with actual VL53L1X library calls.
        """
        if not GPIO_AVAILABLE:
            self.get_logger().error(
                "RPi.GPIO not available. "
                "Running in stub mode — no real sensor data will be published."
            )
            self._sensors_ok = False
            return

        GPIO.setmode(GPIO.BCM)
        for pin in self._xshut_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)  # Disable all sensors
        time.sleep(0.01)

        # TBD: Enable sensors one-by-one and assign I²C addresses
        # Example (requires VL53L1X Python library):
        # for i, (pin, addr) in enumerate(zip(self._xshut_pins, self._addrs)):
        #     GPIO.output(pin, GPIO.HIGH)
        #     time.sleep(0.01)
        #     sensor = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=0x29)
        #     sensor.open()
        #     if addr != 0x29:
        #         sensor.change_address(addr)
        #     sensor.start_ranging(2)  # Mode 2: long range
        #     self._sensors.append(sensor)

        self.get_logger().warn(
            "VL53L1X sensor initialisation is a STUB. "
            "Implement _init_sensors() with the correct library."
        )
        self._sensors_ok = False  # Set True after real implementation

    def _read_and_publish(self):
        """Read distances and publish Range messages."""
        for i, (pub, label, frame_id) in enumerate(
            zip(self._pubs, self.SENSOR_LABELS, self._frame_ids)
        ):
            msg = Range()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            msg.radiation_type  = Range.INFRARED
            msg.field_of_view   = 0.436  # ~25° (VL53L1X typical — TBD verify)
            msg.min_range       = 0.04   # 4 cm — VL53L1X spec
            msg.max_range       = 4.00   # 4 m  — VL53L1X spec (long range mode)

            if self._sensors_ok and i < len(self._sensors):
                try:
                    # TBD: sensor.get_distance() returns distance in mm
                    # distance_mm = self._sensors[i].get_distance()
                    # msg.range = distance_mm / 1000.0
                    msg.range = float("inf")  # Stub
                except Exception as exc:
                    self.get_logger().warn(f"ToF {label} read error: {exc}")
                    msg.range = float("inf")
            else:
                msg.range = float("inf")  # No sensor

            pub.publish(msg)

    def destroy_node(self):
        # TBD: stop ranging on each sensor
        if GPIO_AVAILABLE:
            GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ToFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
