"""Closed-loop metric logger and RViz status visualization."""

from __future__ import annotations

import csv
import json
import math
from datetime import datetime
from pathlib import Path

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.node import Node
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from .core import nearest_index, point_to_oriented_box_clearance, polyline_distance, quaternion_to_yaw, signed_lateral


class EvaluatorNode(Node):
    def __init__(self) -> None:
        super().__init__("evaluator_node")
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("results_dir", "results")
        scenario_path = Path(str(self.get_parameter("scenario_file").value))
        self.scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
        results_dir = Path(str(self.get_parameter("results_dir").value))
        results_dir.mkdir(parents=True, exist_ok=True)
        self.output_path = results_dir / f"tracking_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        # Node.handle is a read-only rclpy property; keep the CSV resource under
        # an application-specific name.
        self._csv_handle = self.output_path.open("w", encoding="utf-8", newline="")
        self.writer = csv.writer(self._csv_handle)
        self.writer.writerow(["time_s", "x", "y", "yaw_rad", "speed_mps", "command_speed_mps", "steering_rad", "tracking_error_m", "center_error_m", "minimum_clearance_m", "planning_ms", "planner_status", "collision_count", "progress_percent"])
        self.position = None
        self.yaw = 0.0
        self.speed = 0.0
        self.command_speed = 0.0
        self.steering = 0.0
        self.planned_path = []
        self.planning_ms = 0.0
        self.planner_status = "WAITING"
        self.collision_active = False
        self.collision_count = 0
        self.center_index = 0
        self.actual_path = PathMessage()
        self.actual_path.header.frame_id = "map"
        self.started = self.get_clock().now()

        self.path_pub = self.create_publisher(PathMessage, "/actual_path", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/simulation/metrics", 10)
        self.create_subscription(Odometry, "/ground_truth/odom", self._odom_callback, 20)
        self.create_subscription(PathMessage, "/planned_path", self._path_callback, 10)
        self.create_subscription(AckermannDriveStamped, "/cmd_control", self._command_callback, 20)
        self.create_subscription(Float32, "/metrics/planning_ms", self._planning_callback, 10)
        self.create_subscription(String, "/planner/status", self._status_callback, 10)
        self.create_timer(0.10, self._evaluate)
        self.get_logger().info(f"Metrics CSV: {self.output_path}")

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

    def _planning_callback(self, message: Float32) -> None:
        self.planning_ms = float(message.data)

    def _status_callback(self, message: String) -> None:
        self.planner_status = message.data

    def _evaluate(self) -> None:
        if self.position is None:
            return
        centerline = self.scenario["centerline"]
        self.center_index = nearest_index(centerline, self.position[0], self.position[1], max(0, self.center_index - 3))
        reference = centerline[self.center_index]
        center_error = abs(signed_lateral(self.position, reference))
        tracking_error = polyline_distance(self.position, self.planned_path)
        boundary_clearance = reference["half_width"] - center_error - self.scenario["vehicle"]["width"] * 0.5
        minimum_clearance = boundary_clearance
        collision = boundary_clearance < 0.0
        for obstacle in self.scenario["obstacles"]:
            clearance = point_to_oriented_box_clearance(
                self.position, obstacle,
                self.scenario["vehicle"]["length"] * 0.5,
                self.scenario["vehicle"]["width"] * 0.5,
            )
            minimum_clearance = min(minimum_clearance, clearance)
            collision = collision or clearance <= 0.0
        if collision and not self.collision_active:
            self.collision_count += 1
        self.collision_active = collision
        progress = 100.0 * reference["s"] / max(self.scenario["length"], 1.0)
        elapsed = (self.get_clock().now() - self.started).nanoseconds / 1e9
        self.writer.writerow([f"{elapsed:.3f}", f"{self.position[0]:.5f}", f"{self.position[1]:.5f}", f"{self.yaw:.5f}", f"{self.speed:.4f}", f"{self.command_speed:.4f}", f"{self.steering:.5f}", f"{tracking_error:.4f}", f"{center_error:.4f}", f"{minimum_clearance:.4f}", f"{self.planning_ms:.3f}", self.planner_status, self.collision_count, f"{progress:.2f}"])
        self._csv_handle.flush()

        pose = self._pose_stamped()
        self.actual_path.header.stamp = pose.header.stamp
        self.actual_path.poses.append(pose)
        if len(self.actual_path.poses) > 3000:
            self.actual_path.poses = self.actual_path.poses[-2500:]
        self.path_pub.publish(self.actual_path)
        self._publish_metrics(tracking_error, center_error, minimum_clearance, progress)

    def _pose_stamped(self):
        from geometry_msgs.msg import PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = self.position
        pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        return pose

    def _publish_metrics(self, track, center, clearance, progress) -> None:
        array = MarkerArray()
        text = Marker()
        text.header.frame_id = "base_link"
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns, text.id, text.type, text.action = "closed_loop_metrics", 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose.position.x, text.pose.position.y, text.pose.position.z = -1.0, 0.0, 2.55
        text.scale.z = 0.28
        text.color.r, text.color.g, text.color.b, text.color.a = 0.95, 0.95, 1.0, 1.0
        text.text = f"v={self.speed:.2f} m/s  steer={math.degrees(self.steering):+.1f} deg  track={track:.2f} m  center={center:.2f} m  clearance={clearance:.2f} m  progress={progress:.0f}%  collisions={self.collision_count}"
        array.markers.append(text)
        self.marker_pub.publish(array)

    def destroy_node(self):
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
