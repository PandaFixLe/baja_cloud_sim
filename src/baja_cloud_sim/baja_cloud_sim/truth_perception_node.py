"""Publish ideal GPS, yaw, road boundaries and obstacle boxes from Gazebo truth."""

from __future__ import annotations

import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .core import quaternion_to_yaw, world_to_base, wrap_angle, yaw_to_quaternion


class TruthPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("truth_perception_node")
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("perception_forward", 34.0)
        self.declare_parameter("perception_backward", 6.0)
        scenario_file = self.get_parameter("scenario_file").get_parameter_value().string_value
        if not scenario_file:
            raise RuntimeError("scenario_file parameter is required")
        self.scenario = json.loads(Path(scenario_file).read_text(encoding="utf-8"))
        self.forward = float(self.get_parameter("perception_forward").value)
        self.backward = float(self.get_parameter("perception_backward").value)
        self.pose = None
        self.yaw = 0.0
        self.tf_broadcaster = TransformBroadcaster(self)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.yaw_pub = self.create_publisher(Float32, "/imu/yaw", 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, "/obstacle_markers", 10)
        self.boundary_pub = self.create_publisher(MarkerArray, "/road_boundary_markers", 10)
        self.centerline_pub = self.create_publisher(PathMessage, "/reference_centerline", latched)
        self.create_subscription(Odometry, "/ground_truth/odom", self._odom_callback, 20)
        self.create_timer(0.05, self._publish_truth)
        self.create_timer(1.0, self._publish_centerline)
        self._publish_centerline()
        self.get_logger().info(
            f"Truth perception ready: {len(self.scenario['obstacles'])} boxes, "
            f"{len(self.scenario['centerline'])} centerline samples"
        )

    def _odom_callback(self, message: Odometry) -> None:
        self.pose = message.pose.pose.position
        q = message.pose.pose.orientation
        self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def _publish_centerline(self) -> None:
        message = PathMessage()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        for item in self.scenario["centerline"]:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = item["x"]
            pose.pose.position.y = item["y"]
            pose.pose.position.z = 0.08
            qx, qy, qz, qw = yaw_to_quaternion(item["yaw"])
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            message.poses.append(pose)
        self.centerline_pub.publish(message)

    def _boundary_marker(self, points, namespace: str, marker_id: int, color) -> Marker:
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.09
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.lifetime.nanosec = 180_000_000
        for item in points:
            local = world_to_base((item["x"], item["y"]), (self.pose.x, self.pose.y), self.yaw)
            if -self.backward <= local[0] <= self.forward and abs(local[1]) <= 12.0:
                point = Point(x=local[0], y=local[1], z=0.08)
                marker.points.append(point)
        return marker

    def _publish_truth(self) -> None:
        if self.pose is None:
            return
        now = self.get_clock().now().to_msg()
        origin = self.scenario["gps_origin"]
        latitude = origin["latitude"] + self.pose.y / 111320.0
        longitude = origin["longitude"] + self.pose.x / (
            111320.0 * math.cos(math.radians(origin["latitude"]))
        )
        gps = NavSatFix()
        gps.header.stamp = now
        gps.header.frame_id = "gps_link"
        gps.latitude = latitude
        gps.longitude = longitude
        gps.altitude = origin["altitude"] + self.pose.z
        gps.status.status = NavSatStatus.STATUS_GBAS_FIX
        gps.status.service = NavSatStatus.SERVICE_GPS
        gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        gps.position_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.02]
        self.gps_pub.publish(gps)

        navigation_yaw = wrap_angle(math.pi * 0.5 - self.yaw)
        self.yaw_pub.publish(Float32(data=float(navigation_yaw)))

        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.pose.x
        transform.transform.translation.y = self.pose.y
        transform.transform.translation.z = self.pose.z
        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

        boundary_array = MarkerArray()
        boundary_array.markers.append(
            self._boundary_marker(self.scenario["left_boundary"], "road_left", 0, (0.10, 0.85, 1.0, 1.0))
        )
        boundary_array.markers.append(
            self._boundary_marker(self.scenario["right_boundary"], "road_right", 1, (0.10, 0.85, 1.0, 1.0))
        )
        self.boundary_pub.publish(boundary_array)

        obstacle_array = MarkerArray()
        for obstacle in self.scenario["obstacles"]:
            local = world_to_base((obstacle["x"], obstacle["y"]), (self.pose.x, self.pose.y), self.yaw)
            if not (-self.backward <= local[0] <= self.forward and abs(local[1]) <= 12.0):
                continue
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = now
            marker.ns = "truth_obstacles"
            marker.id = int(obstacle["id"])
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = local[0]
            marker.pose.position.y = local[1]
            marker.pose.position.z = obstacle["height"] * 0.5
            relative_yaw = wrap_angle(obstacle["yaw"] - self.yaw)
            _, _, marker.pose.orientation.z, marker.pose.orientation.w = yaw_to_quaternion(relative_yaw)
            marker.scale.x = obstacle["length"]
            marker.scale.y = obstacle["width"]
            marker.scale.z = obstacle["height"]
            marker.color.r = 1.0
            marker.color.g = 0.18
            marker.color.b = 0.08
            marker.color.a = 0.72
            marker.lifetime.nanosec = 180_000_000
            obstacle_array.markers.append(marker)
        self.obstacle_pub.publish(obstacle_array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TruthPerceptionNode()
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
