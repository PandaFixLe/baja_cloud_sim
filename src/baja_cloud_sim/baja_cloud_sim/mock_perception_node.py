"""Mock perception node that emulates the perception-group's output stream.

Purpose
-------
This node is NOT used in simulation or on the real car. It is a standalone
debug/integration helper that publishes the *exact* message contract the
planning & control core expects from the perception group, so we can verify
the algorithm nodes receive and parse the data correctly *before* the real
perception package arrives.

Contract being emulated (see params.yaml / docs):
  /road_boundary_markers  : MarkerArray, frame_id=base_link, LINE_STRIP
                            ns="road_left" | "road_right"
  /obstacle_markers       : MarkerArray, frame_id=base_link, CUBE
                            ns="tall"        -> high obstacle (lateral avoid)
                            ns="flat_ground" -> special ground (straight pass,
                                                longitudinal derate)
                            markers without a valid ns are ignored by the core.

Unlike truth_perception_node this node does NOT read Gazebo ground-truth
odometry; it fabricates a static scene in the base_link frame and streams it
at a fixed rate, which is enough to test subscription/callback wiring.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from .core import yaw_to_quaternion


class MockPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("mock_perception_node")
        # ── 话题可参数化（与实车对接时改这里即可） ──
        self.declare_parameter("boundary_topic", "/road_boundary_markers")
        self.declare_parameter("obstacle_topic", "/obstacle_markers")
        self.declare_parameter("publish_rate_hz", 20.0)
        # 相对车身(base_link)的静态场景描述（米）
        self.declare_parameter("road_half_width", 3.0)      # 左右边界对称半宽
        self.declare_parameter("road_forward", 34.0)        # 边界向前可见距离
        self.declare_parameter("road_backward", 6.0)        # 边界向后可见距离

        boundary_topic = self.get_parameter("boundary_topic").value
        obstacle_topic = self.get_parameter("obstacle_topic").value
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.half_width = float(self.get_parameter("road_half_width").value)
        self.forward = float(self.get_parameter("road_forward").value)
        self.backward = float(self.get_parameter("road_backward").value)

        # 一对示例障碍物：一个高障碍(tall)，一个特殊地面(flat_ground)
        # (x_forward, y_lateral, length, width, height, ns)
        self._obstacles = [
            # 正前方 12m 处的高障碍，偏右 1.2m，应当触发横向避让
            {"x": 12.0, "y": 1.2, "length": 1.0, "width": 0.8, "height": 1.2, "ns": "tall"},
            # 正前方 20m 处的特殊地面(减速带/土坡)，直线通过+平滑降速
            {"x": 20.0, "y": 0.0, "length": 3.0, "width": 1.8, "height": 0.08, "ns": "flat_ground"},
        ]

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._boundary_pub = self.create_publisher(MarkerArray, boundary_topic, qos)
        self._obstacle_pub = self.create_publisher(MarkerArray, obstacle_topic, qos)
        self._timer = self.create_timer(1.0 / max(rate, 1.0), self._publish)
        self.get_logger().info(
            f"Mock perception streaming on '{boundary_topic}' & '{obstacle_topic}' "
            f"({len(self._obstacles)} obstacles: "
            + ", ".join(o["ns"] for o in self._obstacles)
            + ")"
        )

    def _make_boundary(self) -> MarkerArray:
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for ns, sign in (("road_left", 1.0), ("road_right", -1.0)):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = stamp
            marker.ns = ns
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.09
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.10, 0.85, 1.0, 1.0)
            marker.lifetime.nanosec = 200_000_000
            # 沿纵向均匀采点，横向恒定半宽（模拟直道两侧边界）
            n = 21
            for i in range(n):
                fx = self.backward + (self.forward + self.backward) * i / (n - 1)
                marker.points.append(Point(x=fx, y=sign * self.half_width, z=0.08))
            array.markers.append(marker)
        return array

    def _make_obstacles(self) -> MarkerArray:
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for idx, ob in enumerate(self._obstacles):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = stamp
            marker.ns = ob["ns"]  # 关键：类型通过 ns 区分
            marker.id = idx
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = ob["x"]
            marker.pose.position.y = ob["y"]
            marker.pose.position.z = ob["height"] * 0.5
            # 面向车身（无相对偏航）
            _, _, marker.pose.orientation.z, marker.pose.orientation.w = yaw_to_quaternion(0.0)
            marker.scale.x = ob["length"]
            marker.scale.y = ob["width"]
            marker.scale.z = ob["height"]
            if ob["ns"] == "tall":
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.18, 0.08, 0.72)
            else:  # flat_ground
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.95, 0.78, 0.12, 0.85)
            marker.lifetime.nanosec = 200_000_000
            array.markers.append(marker)
        return array

    def _publish(self) -> None:
        self._boundary_pub.publish(self._make_boundary())
        self._obstacle_pub.publish(self._make_obstacles())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockPerceptionNode()
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
