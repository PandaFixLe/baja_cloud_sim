"""Online-path version of the existing lookahead / heading controller."""

from __future__ import annotations

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String

from .core import ControllerConfig, legacy_path_control


class PathFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("path_follower_node")
        for name, default in (
            ("origin_latitude", 30.0), ("origin_longitude", 114.0),
            ("target_speed", 2.5), ("lookahead_distance", 3.0),
            ("kp_heading", 1.2), ("max_steering_angle", 35.0),
        ):
            self.declare_parameter(name, default)
        self.origin_lat = float(self.get_parameter("origin_latitude").value)
        self.origin_lon = float(self.get_parameter("origin_longitude").value)
        self.config = ControllerConfig(
            target_speed=float(self.get_parameter("target_speed").value),
            lookahead_distance=float(self.get_parameter("lookahead_distance").value),
            heading_gain=float(self.get_parameter("kp_heading").value),
            max_steering_deg=float(self.get_parameter("max_steering_angle").value),
        )
        self.position = None
        self.yaw_navigation = 0.0
        self.path = []
        self.planner_feasible = False
        self.last_path_time = None

        self.command_pub = self.create_publisher(AckermannDriveStamped, "/cmd_control", 10)
        self.lookahead_pub = self.create_publisher(PointStamped, "/lookahead_point", 10)
        self.create_subscription(NavSatFix, "/gps/fix", self._gps_callback, 20)
        self.create_subscription(Float32, "/imu/yaw", self._yaw_callback, 20)
        self.create_subscription(PathMessage, "/planned_path", self._path_callback, 10)
        self.create_subscription(String, "/planner/status", self._status_callback, 10)
        self.create_timer(0.05, self._control)
        self.get_logger().info("Legacy lookahead control enabled with online /planned_path input")

    def _gps_callback(self, message: NavSatFix) -> None:
        x = (message.longitude - self.origin_lon) * 111320.0 * math.cos(math.radians(self.origin_lat))
        y = (message.latitude - self.origin_lat) * 111320.0
        self.position = (x, y)

    def _yaw_callback(self, message: Float32) -> None:
        self.yaw_navigation = float(message.data)

    def _path_callback(self, message: PathMessage) -> None:
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]
        self.last_path_time = self.get_clock().now()

    def _status_callback(self, message: String) -> None:
        self.planner_feasible = message.data == "FEASIBLE"

    def _publish_stop(self) -> None:
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.drive.speed = 0.0
        message.drive.steering_angle = 0.0
        self.command_pub.publish(message)

    def _control(self) -> None:
        if self.position is None or not self.planner_feasible or len(self.path) < 2:
            self._publish_stop()
            return
        if self.last_path_time is None or (self.get_clock().now() - self.last_path_time).nanoseconds > 500_000_000:
            self._publish_stop()
            return
        command = legacy_path_control(self.position, self.yaw_navigation, self.path, self.config)
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.drive.speed = float(command["speed"])
        message.drive.steering_angle = float(command["steering"])
        self.command_pub.publish(message)

        lookahead = PointStamped()
        lookahead.header.stamp = message.header.stamp
        lookahead.header.frame_id = "map"
        lookahead.point.x = command["target_x"]
        lookahead.point.y = command["target_y"]
        lookahead.point.z = 0.18
        self.lookahead_pub.publish(lookahead)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathFollowerNode()
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
