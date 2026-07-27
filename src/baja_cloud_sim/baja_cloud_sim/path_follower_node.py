"""Online-path version of the existing lookahead / heading controller."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String

from .core import (
    ControllerConfig,
    StanleyControllerConfig,
    StanleyState,
    legacy_path_control,
    stanley_path_control,
    lqr_path_control,
)
from .trajectory_smoother import TrajectorySmoother, TrajectoryTable
from .lqr_controller import LQRController, LQRConfig


class PathFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("path_follower_node")
        for name, default in (
            ("origin_latitude", 30.0), ("origin_longitude", 114.0),
            ("target_speed", 2.5), ("lookahead_distance", 3.0),
            ("kp_heading", 1.2), ("max_steering_angle", 35.0),
            # v1.1 Stanley controller additions
            ("controller_mode", "stanley"),
            ("kd_heading", 0.3),
            ("k_stanley", 0.8),
            ("k_cte_dot", 0.15),
            ("k_yaw_rate", 0.5),
            ("steering_alpha", 0.6),
            ("max_steer_rate_deg", 8.0),
            ("adaptive_steering", True),
            ("adaptive_speed", True),
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
        # Note: v1.0 path_follower_node already consumes online /planned_path;
        # no CSV-loading, 4WS, or virtual_target state-machine remains here.
        mode = str(self.get_parameter("controller_mode").value).strip().lower()
        if mode not in ("legacy", "stanley", "lqr"):
            self.get_logger().warning(
                f"Unknown controller_mode '{mode}', falling back to 'stanley'"
            )
            mode = "stanley"
        self.controller_mode = mode
        if mode in ("stanley", "lqr"):
            self.stanley_config = StanleyControllerConfig(
                target_speed=self.config.target_speed,
                lookahead_distance=self.config.lookahead_distance,
                heading_gain=self.config.heading_gain,
                max_steering_deg=self.config.max_steering_deg,
                kd_heading=float(self.get_parameter("kd_heading").value),
                k_stanley=float(self.get_parameter("k_stanley").value),
                k_cte_dot=float(self.get_parameter("k_cte_dot").value),
                k_yaw_rate=float(self.get_parameter("k_yaw_rate").value),
                steering_alpha=float(self.get_parameter("steering_alpha").value),
                max_steer_rate_deg=float(self.get_parameter("max_steer_rate_deg").value),
                adaptive_steering=bool(self.get_parameter("adaptive_steering").value),
                adaptive_speed=bool(self.get_parameter("adaptive_speed").value),
            )
            self.stanley_state = StanleyState()
        else:
            self.stanley_config = None
            self.stanley_state = None
        self.position = None
        self.yaw_navigation = 0.0
        self.path = []
        self.planner_feasible = False
        self.last_path_time = None

        self.command_pub = self.create_publisher(AckermannDriveStamped, "/cmd_control", 10)
        self.lookahead_pub = self.create_publisher(PointStamped, "/lookahead_point", 10)

        # LQR + Bézier trajectory smoother (used when controller_mode == "lqr")
        self.trajectory_table: Optional[TrajectoryTable] = None
        self.lqr_ctrl: Optional[LQRController] = None
        if mode == "lqr":
            self.smoother = TrajectorySmoother(wheelbase=1.43)
            self.smoothed_pub = self.create_publisher(PathMessage, "/smoothed_path", 10)
            self.get_logger().info(
                "path_follower: mode=lqr (Bézier feed-forward + LQR feedback)"
            )
        else:
            self.smoother = None
            self.smoothed_pub = None

        self.create_subscription(NavSatFix, "/gps/fix", self._gps_callback, 20)
        self.create_subscription(Float32, "/imu/yaw", self._yaw_callback, 20)
        self.create_subscription(PathMessage, "/planned_path", self._path_callback, 10)
        self.create_subscription(String, "/planner/status", self._status_callback, 10)
        self.create_timer(0.05, self._control)
        if self.controller_mode == "stanley":
            self.get_logger().info(
                "path_follower: mode=stanley (Stanley + PD + dual-damping + low-pass + rate-limit)"
            )
        else:
            self.get_logger().info(
                "path_follower: mode=legacy (fallback)"
            )

    def _gps_callback(self, message: NavSatFix) -> None:
        x = (message.longitude - self.origin_lon) * 111320.0 * math.cos(math.radians(self.origin_lat))
        y = (message.latitude - self.origin_lat) * 111320.0
        self.position = (x, y)

    def _yaw_callback(self, message: Float32) -> None:
        self.yaw_navigation = float(message.data)

    def _path_callback(self, message: PathMessage) -> None:
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]
        self.last_path_time = self.get_clock().now()

        # LQR mode: regenerate trajectory table on each new path
        if self.controller_mode == "lqr" and self.smoother is not None:
            target_speed = float(self.get_parameter("target_speed").value)
            self.trajectory_table = self.smoother.generate(
                self.path, target_speed=target_speed,
            )
            if self.lqr_ctrl is None and self.trajectory_table is not None:
                self.lqr_ctrl = LQRController(LQRConfig())
            elif self.trajectory_table is None:
                self.get_logger().warn(
                    "Trajectory smoother returned None, falling back to Stanley",
                    throttle_duration_sec=2.0,
                )
            # Publish smoothed path for RViz visualisation
            if self.smoothed_pub is not None and self.trajectory_table is not None:
                self._publish_smoothed_path()

    def _status_callback(self, message: String) -> None:
        self.planner_feasible = message.data == "FEASIBLE"

    def _publish_smoothed_path(self) -> None:
        """Publish the Bézier-smoothed trajectory as a nav_msgs/Path for RViz."""
        if self.trajectory_table is None or self.smoothed_pub is None:
            return
        msg = PathMessage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        from geometry_msgs.msg import PoseStamped
        for pt in self.trajectory_table.points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = pt.x
            ps.pose.position.y = pt.y
            ps.pose.position.z = 0.12
            # Encode yaw in orientation quaternion
            half_yaw = pt.yaw * 0.5
            ps.pose.orientation.z = math.sin(half_yaw)
            ps.pose.orientation.w = math.cos(half_yaw)
            msg.poses.append(ps)
        self.smoothed_pub.publish(msg)

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
        if self.last_path_time is None or (self.get_clock().now() - self.last_path_time).nanoseconds > 400_000_000:
            self._publish_stop()
            return

        if self.controller_mode == "legacy":
            command = legacy_path_control(self.position, self.yaw_navigation, self.path, self.config)
            speed = float(command["speed"])
            steering = float(command["steering"])
            target_x, target_y = float(command["target_x"]), float(command["target_y"])
        elif self.controller_mode == "lqr":
            command = lqr_path_control(
                self.position, self.yaw_navigation, self.path,
                self.stanley_config, self.stanley_state, dt=0.05,
                trajectory_table=self.trajectory_table,
                lqr_ctrl=self.lqr_ctrl,
            )
            self.stanley_state = command["state"]
            speed = float(command["speed"])
            steering = float(command["steering"])
            target_x, target_y = float(command["target_x"]), float(command["target_y"])
        else:
            command = stanley_path_control(
                self.position, self.yaw_navigation, self.path,
                self.stanley_config, self.stanley_state, dt=0.05,
            )
            self.stanley_state = command["state"]
            speed = float(command["speed"])
            steering = float(command["steering"])
            target_x, target_y = float(command["target_x"]), float(command["target_y"])

        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.drive.speed = speed
        message.drive.steering_angle = steering
        self.command_pub.publish(message)

        lookahead = PointStamped()
        lookahead.header.stamp = message.header.stamp
        lookahead.header.frame_id = "map"
        lookahead.point.x = target_x
        lookahead.point.y = target_y
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
