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
        # 障碍物宽度兜底（算法核心不要求传 scale.y，缺失时用此值）
        self.declare_parameter("obstacle_default_half_width", 0.9)

        boundary_topic = self.get_parameter("boundary_topic").value
        obstacle_topic = self.get_parameter("obstacle_topic").value
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.half_width = float(self.get_parameter("road_half_width").value)
        self.forward = float(self.get_parameter("road_forward").value)
        self.backward = float(self.get_parameter("road_backward").value)
        self.obstacle_half_width = float(self.get_parameter("obstacle_default_half_width").value)

        # 示例障碍物：一个高障碍(tall)，一个特殊地面(flat_ground)
        # 最小契约：仅需 x/y(相对车身坐标) + length(前向长度) + ns(类型)
        # 宽度/高度由算法核心兜底或仅用于 RViz 显示，无需感知组提供
        self._obstacles = [
            # 正前方 12m 处的高障碍，偏右 1.2m，应当触发横向避让
            {"x": 12.0, "y": 1.2, "length": 1.0, "ns": "tall"},
            # 正前方 20m 处的特殊地面(减速带/土坡)，直线通过+平滑降速
            {"x": 20.0, "y": 0.0, "length": 3.0, "ns": "flat_ground"},
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
            # 盒子显示高度：tall 用长度近似，flat_ground 给极小显示高度
            # （scale.z 算法不用，仅 RViz 可视化；高度非必需字段）
            show_z = ob["length"] * 0.6 if ob["ns"] == "tall" else 0.05
            marker.pose.position.z = show_z * 0.5
            # 面向车身（无相对偏航）
            _, _, marker.pose.orientation.z, marker.pose.orientation.w = yaw_to_quaternion(0.0)
            # 最小契约：仅 scale.x (前向长度) 为算法必需；scale.y 用兜底宽度
            marker.scale.x = ob["length"]
            marker.scale.y = 2.0 * self.obstacle_half_width
            marker.scale.z = show_z
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
