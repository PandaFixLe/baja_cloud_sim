"""Adapt existing AckermannDriveStamped commands to Gazebo cmd_vel."""

from __future__ import annotations

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .core import smooth_velocity


class ActuatorAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("actuator_adapter_node")
        self.declare_parameter("wheelbase", 1.43)
        self.declare_parameter("command_timeout", 0.35)
        # Platform-specific topic remaps (phase 0 of real-car port):
        # in simulation this points at the Gazebo vehicle; on the Orin real car
        # it points at the chassis driver (e.g. /chassis/cmd).
        self.declare_parameter("cmd_vel_topic", "/model/baja_vehicle/cmd_vel")
        # Input odometry used only to echo vehicle status. In simulation this is
        # the Gazebo ground-truth odometry; on the real car it would be the
        # fused localization odometry instead.
        self.declare_parameter("odom_topic", "/ground_truth/odom")
        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.timeout = float(self.get_parameter("command_timeout").value)
        self.last_command = None
        self.last_stamp = None
        self._current_speed = 0.0
        self._prev_accel = 0.0
        self.command_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10)
        self.status_pub = self.create_publisher(AckermannDriveStamped, "/vehicle_status", 10)
        self.create_subscription(AckermannDriveStamped, "/cmd_control", self._command_callback, 20)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self._odom_callback, 20)
        self.create_timer(0.05, self._publish)

    def _command_callback(self, message: AckermannDriveStamped) -> None:
        self.last_command = message
        self.last_stamp = self.get_clock().now()

    def _odom_callback(self, message: Odometry) -> None:
        status = AckermannDriveStamped()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = "base_link"
        status.drive.speed = math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
        status.drive.steering_angle = self.last_command.drive.steering_angle if self.last_command else 0.0
        self.status_pub.publish(status)

    def _publish(self) -> None:
        output = Twist()
        if self.last_command is not None and self.last_stamp is not None:
            age = (self.get_clock().now() - self.last_stamp).nanoseconds / 1e9
            if age <= self.timeout:
                target_speed = float(self.last_command.drive.speed)
                steering = float(self.last_command.drive.steering_angle)
                # accel + jerk smoothing (Phase 0.6)
                smoothed, self._prev_accel = smooth_velocity(
                    target_speed, self._current_speed, 2.0, -2.5, 0.05,
                    prev_accel=self._prev_accel, max_jerk=4.0,
                )
                self._current_speed = smoothed
                output.linear.x = smoothed
                output.angular.z = smoothed * math.tan(steering) / self.wheelbase if abs(smoothed) > 0.01 else 0.0
        else:
            self._current_speed = 0.0
            self._prev_accel = 0.0
        self.command_pub.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ActuatorAdapterNode()
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
