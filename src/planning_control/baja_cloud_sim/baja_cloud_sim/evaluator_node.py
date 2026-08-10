"""Closed-loop metric logger and RViz status visualization.

Supports two modes:
  - use_scenario=true  (simulation): loads scenario JSON for centerline/obstacles/vehicle
  - use_scenario=false (real car):    subscribes to /reference_centerline and
                                      /obstacle_markers for live centerline/obstacles

In real-car mode, /metrics/eps_actual_steer is unavailable (actuator_adapter
not running). Instead, /vehicle_status (from can_bridge) provides the actual
steering angle.
"""

from __future__ import annotations

import csv
import json
import math
from datetime import datetime
from pathlib import Path

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from .core import (
    nearest_index,
    point_to_oriented_box_clearance,
    polyline_distance,
    quaternion_to_yaw,
    signed_lateral,
)


class EvaluatorNode(Node):
    def __init__(self) -> None:
        super().__init__("evaluator_node")
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("results_dir", "results")
        self.declare_parameter("use_scenario", True)
        self.declare_parameter("vehicle_length", 3.0)
        self.declare_parameter("vehicle_width", 1.5)
        self.declare_parameter("vehicle_status_topic", "/vehicle_status")

        use_scenario = bool(self.get_parameter("use_scenario").value)
        results_dir = Path(str(self.get_parameter("results_dir").value))
        results_dir.mkdir(parents=True, exist_ok=True)
        self.output_path = results_dir / f"tracking_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self._csv_handle = self.output_path.open("w", encoding="utf-8", newline="")
        self.writer = csv.writer(self._csv_handle)
        self.writer.writerow([
            "time_s", "x", "y", "yaw_rad", "speed_mps", "command_speed_mps",
            "steering_rad", "actual_steer_rad", "tracking_error_m",
            "center_error_m", "minimum_clearance_m", "planning_ms",
            "planner_status", "collision_count", "progress_percent",
        ])

        # State
        self.position = None
        self.yaw = 0.0
        self.speed = 0.0
        self.command_speed = 0.0
        self.steering = 0.0
        self.actual_steering = 0.0
        self.planned_path = []
        self.planning_ms = 0.0
        self.planner_status = "WAITING"
        self.collision_active = False
        self.collision_count = 0
        self.center_index = 0
        self.actual_path = PathMessage()
        self.actual_path.header.frame_id = "map"
        self.started = self.get_clock().now()

        # Mode-specific data sources
        self.use_scenario = use_scenario
        self.centerline: list = []
        self.obstacles: list = []
        self.vehicle_length = float(self.get_parameter("vehicle_length").value)
        self.vehicle_width = float(self.get_parameter("vehicle_width").value)
        self.track_length = 1.0  # Avoid div-by-zero; updated when centerline is set

        if use_scenario:
            scenario_path = Path(str(self.get_parameter("scenario_file").value))
            if not scenario_path or not scenario_path.is_file():
                raise RuntimeError(
                    f"use_scenario=true but scenario_file not found: {scenario_path}"
                )
            self.scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
            self.centerline = self.scenario["centerline"]
            self.obstacles = self.scenario["obstacles"]
            self.vehicle_length = float(self.scenario["vehicle"]["length"])
            self.vehicle_width = float(self.scenario["vehicle"]["width"])
            self.track_length = float(self.scenario.get("length", 1.0))
            self.get_logger().info(
                f"Evaluator (scenario mode): {len(self.centerline)} centerline pts, "
                f"{len(self.obstacles)} obstacles, track={self.track_length:.1f} m"
            )
        else:
            # Real-car mode: subscribe to live centerline and obstacles
            latched = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                PathMessage, "/reference_centerline",
                self._centerline_callback_eval, latched
            )
            self.create_subscription(
                MarkerArray, "/obstacle_markers",
                self._obstacle_callback_eval, 10
            )
            self.get_logger().info(
                "Evaluator (real-car mode): subscribing to /reference_centerline "
                "and /obstacle_markers for live centerline/obstacles"
            )

        # Publishers
        self.path_pub = self.create_publisher(PathMessage, "/actual_path", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/simulation/metrics", 10)

        # Common subscriptions
        self.create_subscription(Odometry, "/ground_truth/odom", self._odom_callback, 20)
        self.create_subscription(PathMessage, "/planned_path", self._path_callback, 10)
        self.create_subscription(AckermannDriveStamped, "/cmd_control", self._command_callback, 20)
        self.create_subscription(Float32, "/metrics/planning_ms", self._planning_callback, 10)
        self.create_subscription(String, "/planner/status", self._status_callback, 10)

        # Steering feedback: simulation uses /metrics/eps_actual_steer (Float32 from
        # actuator_adapter), real car uses /vehicle_status (AckermannDriveStamped
        # from can_bridge). Subscribe to both; whichever is publishing will update.
        self.create_subscription(Float32, "/metrics/eps_actual_steer", self._eps_steer_callback, 10)
        vehicle_status_topic = str(self.get_parameter("vehicle_status_topic").value)
        self.create_subscription(
            AckermannDriveStamped, vehicle_status_topic,
            self._vehicle_status_callback, 20
        )

        self.create_timer(0.10, self._evaluate)
        self.get_logger().info(f"Metrics CSV: {self.output_path}")

    # ── Real-car mode callbacks ──
    def _centerline_callback_eval(self, message: PathMessage) -> None:
        """Build centerline list from /reference_centerline Path message."""
        pts = []
        cumulative = 0.0
        prev = None
        for pose in message.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            q = pose.pose.orientation
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            if prev is not None:
                cumulative += math.hypot(x - prev[0], y - prev[1])
            pts.append({"x": x, "y": y, "yaw": yaw, "s": cumulative, "half_width": 4.0})
            prev = (x, y)
        self.centerline = pts
        self.track_length = cumulative if cumulative > 0 else 1.0
        self.get_logger().info(
            f"Evaluator received centerline: {len(pts)} pts, length={cumulative:.1f} m"
        )

    def _obstacle_callback_eval(self, message: MarkerArray) -> None:
        """Build obstacle list from /obstacle_markers (ns='tall' only)."""
        obstacles = []
        for marker in message.markers:
            if marker.ns != "tall":
                continue
            q = marker.pose.orientation
            relative_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            obstacles.append({
                "x": marker.pose.position.x,
                "y": marker.pose.position.y,
                "yaw": relative_yaw,
                "length": marker.scale.x,
                "width": marker.scale.y,
            })
        self.obstacles = obstacles

    # ── Common callbacks ──
    def _odom_callback(self, message: Odometry) -> None:
        self.position = (message.pose.pose.position.x, message.pose.pose.position.y)
        q = message.pose.pose.orientation
        self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.speed = math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)

    def _path_callback(self, message: PathMessage) -> None:
        self.planned_path = [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]

    def _command_callback(self, message: AckermannDriveStamped) -> None:
        self.command_speed = message.drive.speed
        self.steering = message.drive.steering_angle

    def _eps_steer_callback(self, message: Float32) -> None:
        """Simulation: EPS actual steering from actuator_adapter."""
        self.actual_steering = float(message.data)

    def _vehicle_status_callback(self, message: AckermannDriveStamped) -> None:
        """Real car: actual steering from can_bridge /vehicle_status."""
        self.actual_steering = float(message.drive.steering_angle)

    def _planning_callback(self, message: Float32) -> None:
        self.planning_ms = float(message.data)

    def _status_callback(self, message: String) -> None:
        self.planner_status = message.data

    def _evaluate(self) -> None:
        if self.position is None:
            return
        if not self.centerline:
            # No centerline yet (real-car mode, waiting for /reference_centerline)
            return

        self.center_index = nearest_index(
            self.centerline, self.position[0], self.position[1],
            max(0, self.center_index - 3),
        )
        reference = self.centerline[self.center_index]
        center_error = abs(signed_lateral(self.position, reference))
        tracking_error = polyline_distance(self.position, self.planned_path) if self.planned_path else 0.0

        # Clearance: use half_width if available, else default
        half_width = reference.get("half_width", 4.0)
        boundary_clearance = half_width - center_error - self.vehicle_width * 0.5
        minimum_clearance = boundary_clearance
        collision = boundary_clearance < 0.0

        for obstacle in self.obstacles:
            clearance = point_to_oriented_box_clearance(
                self.position, obstacle,
                self.vehicle_length * 0.5,
                self.vehicle_width * 0.5,
            )
            minimum_clearance = min(minimum_clearance, clearance)
            collision = collision or clearance <= 0.0

        if collision and not self.collision_active:
            self.collision_count += 1
        self.collision_active = collision

        progress = 100.0 * reference["s"] / max(self.track_length, 1.0)
        elapsed = (self.get_clock().now() - self.started).nanoseconds / 1e9
        self.writer.writerow([
            f"{elapsed:.3f}", f"{self.position[0]:.5f}", f"{self.position[1]:.5f}",
            f"{self.yaw:.5f}", f"{self.speed:.4f}", f"{self.command_speed:.4f}",
            f"{self.steering:.5f}", f"{self.actual_steering:.5f}",
            f"{tracking_error:.4f}", f"{center_error:.4f}",
            f"{minimum_clearance:.4f}", f"{self.planning_ms:.3f}",
            self.planner_status, self.collision_count, f"{progress:.2f}",
        ])
        self._csv_handle.flush()

        pose = self._pose_stamped()
        self.actual_path.header.stamp = pose.header.stamp
        self.actual_path.poses.append(pose)
        if len(self.actual_path.poses) > 3000:
            self.actual_path.poses = self.actual_path.poses[-2500:]
        self.path_pub.publish(self.actual_path)
        self._publish_metrics(tracking_error, center_error, minimum_clearance, progress)

    def _pose_stamped(self) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = self.position
        pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        return pose

    def _publish_metrics(self, track: float, center: float, clearance: float, progress: float) -> None:
        array = MarkerArray()
        text = Marker()
        text.header.frame_id = "base_link"
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns, text.id, text.type, text.action = "closed_loop_metrics", 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose.position.x, text.pose.position.y, text.pose.position.z = -1.0, 0.0, 2.55
        text.scale.z = 0.28
        text.color.r, text.color.g, text.color.b, text.color.a = 0.95, 0.95, 1.0, 1.0
        text.text = (
            f"v={self.speed:.2f} m/s  steer={math.degrees(self.steering):+.1f} deg  "
            f"track={track:.2f} m  center={center:.2f} m  "
            f"clearance={clearance:.2f} m  progress={progress:.0f}%  "
            f"collisions={self.collision_count}"
        )
        array.markers.append(text)
        self.marker_pub.publish(array)

    def destroy_node(self) -> bool:
        if not self._csv_handle.closed:
            self._csv_handle.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
