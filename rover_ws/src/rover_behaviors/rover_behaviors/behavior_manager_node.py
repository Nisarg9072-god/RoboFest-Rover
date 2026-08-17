#!/usr/bin/env python3
"""
behavior_manager_node.py
========================
Top-level behavior arbiter for the ROBOFEST rover.

Responsibilities:
  - Run the competition mission state machine
  - Decide which module has command authority at any time
  - Sequence rounds (1 through 7, configurable)
  - Trigger parking at mission end
  - Trigger recovery on failure
  - Monitor rover health and adjust behavior accordingly

State machine states:
  INITIALIZE → SELF_CHECK → READY →
  ROUND_RUNNING (NAV_TO_WAYPOINT ↔ OBSTACLE_HANDLING ↔ MARKER_DECISION) →
  ROUND_DONE → (repeat or) PARKING_SEQUENCE → MISSION_COMPLETE

Status: PLANNED — waypoints must be filled in mission.yaml after mapping.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from enum import Enum, auto
import yaml
import math
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from diagnostic_msgs.msg import DiagnosticArray
from vision_msgs.msg import Detection2DArray


class MissionState(Enum):
    INITIALIZE       = auto()
    SELF_CHECK       = auto()
    READY            = auto()
    ROUND_RUNNING    = auto()
    ROUND_DONE       = auto()
    PARKING_SEQUENCE = auto()
    MISSION_COMPLETE = auto()
    ABORT            = auto()


class RoundState(Enum):
    NAV_TO_WAYPOINT   = auto()
    OBSTACLE_HANDLING = auto()
    MARKER_DECISION   = auto()
    CHECKPOINT        = auto()
    ROUND_DONE        = auto()


class BehaviorManagerNode(Node):
    """
    Competition mission manager and behavior arbiter.

    Subscribers:
        /localization/pose  — geometry_msgs/PoseWithCovarianceStamped
        /camera/detections  — vision_msgs/Detection2DArray
        /emergency_stop     — std_msgs/Bool
        /diagnostics        — diagnostic_msgs/DiagnosticArray

    Publishers:
        /goal_pose          — geometry_msgs/PoseStamped
        /mission/state      — std_msgs/String  (human-readable state)
        /cmd_vel            — geometry_msgs/Twist (only during recovery/parking)
    """

    def __init__(self):
        super().__init__("behavior_manager_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("total_rounds", 7)
        self.declare_parameter("max_recovery_attempts", 3)
        self.declare_parameter("round_timeout_s", 300.0)
        self.declare_parameter("waypoint_tolerance_m", 0.25)
        self.declare_parameter("obstacle_wait_timeout_s", 10.0)

        self._total_rounds      = self.get_parameter("total_rounds").value
        self._max_recovery      = self.get_parameter("max_recovery_attempts").value
        self._round_timeout     = self.get_parameter("round_timeout_s").value
        self._wp_tol            = self.get_parameter("waypoint_tolerance_m").value
        self._obs_wait_timeout  = self.get_parameter("obstacle_wait_timeout_s").value

        # ── State ─────────────────────────────────────────────────────────────
        self._mission_state  = MissionState.INITIALIZE
        self._round_state    = RoundState.NAV_TO_WAYPOINT
        self._current_round  = 1
        self._current_wp_idx = 0
        self._recovery_count = 0
        self._estop_active   = False
        self._round_start_t  = None
        self._obs_detect_t   = None

        # Waypoints: loaded from mission.yaml
        self._waypoints: dict = {}

        # Current pose
        self._pose_x   = 0.0
        self._pose_y   = 0.0
        self._pose_yaw = 0.0

        # ── Publishers ────────────────────────────────────────────────────────
        self._goal_pub  = self.create_publisher(PoseStamped, "/goal_pose",     10)
        self._state_pub = self.create_publisher(String,      "/mission/state", 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(PoseStamped,     "/localization/pose", self._pose_cb,    10)
        self.create_subscription(Detection2DArray, "/camera/detections", self._detections_cb, 10)
        self.create_subscription(Bool,            "/emergency_stop",    self._estop_cb,   10)
        self.create_subscription(DiagnosticArray, "/diagnostics",       self._diag_cb,    10)

        # ── State machine timer (10 Hz) ───────────────────────────────────────
        self.create_timer(0.1, self._state_machine_tick)

        # ── Status publishing timer ───────────────────────────────────────────
        self.create_timer(1.0, self._publish_state)

        self.get_logger().info(
            f"behavior_manager_node started. "
            f"Total rounds: {self._total_rounds}"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pose_cb(self, msg: PoseStamped):
        self._pose_x = msg.pose.position.x
        self._pose_y = msg.pose.position.y
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._pose_yaw = math.atan2(siny, cosy)

    def _detections_cb(self, msg: Detection2DArray):
        """
        Process camera detections for marker-based decisions.
        TBD: implement marker decode and direction command.
        """
        for detection in msg.detections:
            # TBD: parse detection results_id to extract direction command
            pass

    def _estop_cb(self, msg: Bool):
        if msg.data:
            self._estop_active = True
            self.get_logger().error("EMERGENCY STOP received — halting mission.")

    def _diag_cb(self, msg: DiagnosticArray):
        """Monitor subsystem health from /diagnostics."""
        for status in msg.status:
            if status.level == 2:  # ERROR
                self.get_logger().warn(
                    f"Diagnostic ERROR from {status.name}: {status.message}"
                )

    # ── State machine ─────────────────────────────────────────────────────────

    def _state_machine_tick(self):
        """Called at 10 Hz. Drives the mission state machine."""

        if self._estop_active:
            self._mission_state = MissionState.ABORT
            return

        if self._mission_state == MissionState.INITIALIZE:
            self._handle_initialize()

        elif self._mission_state == MissionState.SELF_CHECK:
            self._handle_self_check()

        elif self._mission_state == MissionState.READY:
            self._handle_ready()

        elif self._mission_state == MissionState.ROUND_RUNNING:
            self._handle_round_running()

        elif self._mission_state == MissionState.ROUND_DONE:
            self._handle_round_done()

        elif self._mission_state == MissionState.PARKING_SEQUENCE:
            self._handle_parking()

        elif self._mission_state == MissionState.MISSION_COMPLETE:
            pass  # Done

        elif self._mission_state == MissionState.ABORT:
            self.get_logger().error("Mission ABORTED.")

    def _handle_initialize(self):
        """Load waypoints from parameter server and transition to SELF_CHECK."""
        self.get_logger().info("INITIALIZE — loading waypoints from parameters.")
        # TBD: Load waypoints from mission.yaml via parameter server
        # For now, create placeholder
        self._waypoints = {
            f"round_{r}": [{"x": 0.0, "y": 0.0, "yaw": 0.0, "label": "placeholder"}]
            for r in range(1, self._total_rounds + 1)
        }
        self._mission_state = MissionState.SELF_CHECK

    def _handle_self_check(self):
        """
        Verify critical subsystems are available.
        TBD: implement actual topic timeout checks.
        """
        self.get_logger().info("SELF_CHECK — verifying subsystems.")
        # TBD: check /scan, /imu/data, /odom, /battery/status within timeout
        # For now, pass through
        self.get_logger().info("SELF_CHECK passed (stub). Transitioning to READY.")
        self._mission_state = MissionState.READY

    def _handle_ready(self):
        """Waiting for start command. Transition to ROUND_RUNNING."""
        # TBD: implement start command (e.g. ROS service call or topic)
        # For now, auto-start after 2 s
        self.get_logger().info("READY — auto-starting mission (stub).")
        self._current_round  = 1
        self._current_wp_idx = 0
        self._round_start_t  = self.get_clock().now()
        self._mission_state  = MissionState.ROUND_RUNNING
        self._publish_next_goal()

    def _handle_round_running(self):
        """Run the inner waypoint navigation loop for one round."""
        # ── Round timeout ──────────────────────────────────────────────────
        elapsed = (
            self.get_clock().now() - self._round_start_t
        ).nanoseconds * 1e-9
        if elapsed > self._round_timeout:
            self.get_logger().warn(
                f"Round {self._current_round} TIMEOUT — triggering recovery."
            )
            self._mission_state = MissionState.ROUND_DONE
            return

        # ── Check if current waypoint reached ─────────────────────────────
        round_key = f"round_{self._current_round}"
        waypoints = self._waypoints.get(round_key, [])
        if not waypoints:
            self.get_logger().warn(f"No waypoints for {round_key}.")
            self._mission_state = MissionState.ROUND_DONE
            return

        wp = waypoints[self._current_wp_idx]
        dist = math.hypot(self._pose_x - wp["x"], self._pose_y - wp["y"])

        if dist < self._wp_tol:
            self.get_logger().info(
                f"Waypoint {self._current_wp_idx} reached "
                f"(label: {wp.get('label', 'N/A')})"
            )
            self._current_wp_idx += 1

            if self._current_wp_idx >= len(waypoints):
                self.get_logger().info(
                    f"Round {self._current_round} complete."
                )
                self._mission_state = MissionState.ROUND_DONE
            else:
                self._publish_next_goal()

    def _handle_round_done(self):
        """Transition to next round or parking."""
        if self._current_round < self._total_rounds:
            self._current_round  += 1
            self._current_wp_idx  = 0
            self._round_start_t   = self.get_clock().now()
            self._mission_state   = MissionState.ROUND_RUNNING
            self.get_logger().info(f"Starting round {self._current_round}.")
            self._publish_next_goal()
        else:
            self.get_logger().info(
                f"All {self._total_rounds} rounds complete. "
                "Transitioning to PARKING."
            )
            self._mission_state = MissionState.PARKING_SEQUENCE

    def _handle_parking(self):
        """
        Trigger parking node.
        TBD: implement via ROS 2 action or topic to parking_node.
        """
        self.get_logger().info("PARKING_SEQUENCE — activating parking_node (stub).")
        # TBD: send parking start command to parking_node
        self._mission_state = MissionState.MISSION_COMPLETE
        self.get_logger().info("MISSION COMPLETE.")

    # ── Goal publishing ───────────────────────────────────────────────────────

    def _publish_next_goal(self):
        """Publish the next waypoint as a PoseStamped to /goal_pose."""
        round_key = f"round_{self._current_round}"
        waypoints = self._waypoints.get(round_key, [])
        if self._current_wp_idx >= len(waypoints):
            return

        wp = waypoints[self._current_wp_idx]
        goal = PoseStamped()
        goal.header.stamp    = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = wp["x"]
        goal.pose.position.y = wp["y"]

        # Yaw → quaternion
        cy = math.cos(wp["yaw"] * 0.5)
        sy = math.sin(wp["yaw"] * 0.5)
        goal.pose.orientation.w = cy
        goal.pose.orientation.z = sy

        self._goal_pub.publish(goal)
        self.get_logger().info(
            f"Round {self._current_round} → Waypoint {self._current_wp_idx}: "
            f"({wp['x']:.2f}, {wp['y']:.2f}, yaw={math.degrees(wp['yaw']):.1f}°) "
            f"label={wp.get('label', 'N/A')}"
        )

    # ── State publishing ──────────────────────────────────────────────────────

    def _publish_state(self):
        msg = String()
        msg.data = (
            f"mission={self._mission_state.name} "
            f"round={self._current_round}/{self._total_rounds} "
            f"wp={self._current_wp_idx}"
        )
        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
