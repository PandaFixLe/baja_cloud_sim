#!/usr/bin/env python3
"""Log commanded steering against the angle the steering joints actually reach.

The controller can only be tuned against a plant whose actuator authority is
known.  This probe pairs /cmd_control (what the controller asked for) with
/joint_states (what the front steering joints delivered) so the ratio between
them can be measured directly instead of inferred from yaw rate.
"""

from __future__ import annotations

import csv
import math
import sys

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SteeringProbe(Node):
    def __init__(self, out_path: str) -> None:
        super().__init__("steering_probe")
        self._cmd_steer = 0.0
        self._cmd_speed = 0.0
        self._speed = 0.0
        self._yaw_rate = 0.0
        self._t0 = None
        self._file = open(out_path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(
            ["t", "cmd_steer_rad", "left_joint_rad", "right_joint_rad",
             "avg_joint_rad", "speed_mps", "yaw_rate_rps"]
        )
        self.create_subscription(AckermannDriveStamped, "/cmd_control", self._on_cmd, 20)
        self.create_subscription(Odometry, "/ground_truth/odom", self._on_odom, 20)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 20)

    def _on_cmd(self, msg: AckermannDriveStamped) -> None:
        self._cmd_steer = float(msg.drive.steering_angle)
        self._cmd_speed = float(msg.drive.speed)

    def _on_odom(self, msg: Odometry) -> None:
        self._speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self._yaw_rate = msg.twist.twist.angular.z

    def _on_joints(self, msg: JointState) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if self._t0 is None:
            self._t0 = now
        left = right = None
        for name, pos in zip(msg.name, msg.position):
            if name == "front_left_steering_joint":
                left = pos
            elif name == "front_right_steering_joint":
                right = pos
        if left is None or right is None:
            return
        self._writer.writerow([
            f"{now - self._t0:.3f}", f"{self._cmd_steer:.5f}",
            f"{left:.5f}", f"{right:.5f}", f"{0.5 * (left + right):.5f}",
            f"{self._speed:.4f}", f"{self._yaw_rate:.5f}",
        ])

    def destroy_node(self) -> bool:
        self._file.close()
        return super().destroy_node()


def main() -> None:
    out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/steering_probe.csv"
    rclpy.init()
    node = SteeringProbe(out)
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
