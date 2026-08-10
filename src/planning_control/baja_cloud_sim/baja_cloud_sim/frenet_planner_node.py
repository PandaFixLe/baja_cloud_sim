"""Local Frenet-lattice planner using road-line and obstacle-box truth messages."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from .core import (
    PlannerConfig,
    base_to_world,
    gps_to_local,
    nav_to_world_yaw,
    nearest_index,
    plan_frenet_path,
    quaternion_to_yaw,
    signed_lateral,
    wrap_angle,
    yaw_to_quaternion,
)


class FrenetPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__("frenet_planner_node")
        for name, default in (
            ("origin_latitude", 30.0), ("origin_longitude", 114.0),
            ("horizon_m", 30.0), ("center_weight", 1.0),
            ("clearance_weight", 12.0), ("desired_clearance", 1.2),
            ("vehicle_length", 3.0), ("vehicle_width", 1.5),
            ("default_half_width", 4.0),
            ("min_half_width", 1.5),
            ("use_obstacle", True),
        ):
            self.declare_parameter(name, default)
        self.origin_lat = float(self.get_parameter("origin_latitude").value)
        self.origin_lon = float(self.get_parameter("origin_longitude").value)
        self.default_half_width = float(self.get_parameter("default_half_width").value)
        self.min_half_width = float(self.get_parameter("min_half_width").value)
        self.use_obstacle = bool(self.get_parameter("use_obstacle").value)
        self.config = PlannerConfig(
            horizon_m=float(self.get_parameter("horizon_m").value),
            center_weight=float(self.get_parameter("center_weight").value),
            clearance_weight=float(self.get_parameter("clearance_weight").value),
            desired_clearance=float(self.get_parameter("desired_clearance").value),
            vehicle_length=float(self.get_parameter("vehicle_length").value),
            vehicle_width=float(self.get_parameter("vehicle_width").value),
        )
        # Detections within this base_link radius of the sensor origin are
        # treated as spurious self/clutter (e.g. the radar reporting the vehicle
        # chassis or ground clang at startup) and ignored — the planner cannot
        # avoid something it is already on top of, and these produce phantom
        # inflated boxes near the spawn point.
        self.declare_parameter("min_obstacle_range", 0.5)
        self.min_obstacle_range = float(self.get_parameter("min_obstacle_range").value)
        self.centerline = []
        self.position = None
        self.yaw_navigation = 0.0
        self.yaw_world = 0.0
        self.left_world = []
        self.right_world = []
        self.obstacles = []
        self.last_nearest = 0

        self.path_pub = self.create_publisher(PathMessage, "/planned_path", 10)
        self.status_pub = self.create_publisher(String, "/planner/status", 10)
        self.debug_pub = self.create_publisher(MarkerArray, "/planning_debug", 10)
        self.plan_time_pub = self.create_publisher(Float32, "/metrics/planning_ms", 10)
        self.clearance_pub = self.create_publisher(Float32, "/metrics/planned_clearance", 10)
        self.create_subscription(PathMessage, "/reference_centerline", self._centerline_callback, 10)
        self.create_subscription(NavSatFix, "/gps/fix", self._gps_callback, 20)
        self.create_subscription(Float32, "/imu/yaw", self._yaw_callback, 20)
        self.create_subscription(MarkerArray, "/road_boundary_markers", self._boundary_callback, 10)
        self.create_subscription(MarkerArray, "/obstacle_markers", self._obstacle_callback, 10)
        self.create_timer(0.10, self._plan)

    def _centerline_callback(self, message: PathMessage) -> None:
        points = []
        cumulative = 0.0
        previous = None
        for pose in message.poses:
            q = pose.pose.orientation
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            x, y = pose.pose.position.x, pose.pose.position.y
            if previous is not None:
                cumulative += math.hypot(x - previous[0], y - previous[1])
            points.append({"x": x, "y": y, "yaw": yaw, "s": cumulative, "half_width": self.default_half_width})
            previous = (x, y)
        self.centerline = points

    def _gps_callback(self, message: NavSatFix) -> None:
        self.position = gps_to_local(
            message.latitude, message.longitude, self.origin_lat, self.origin_lon
        )

    def _yaw_callback(self, message: Float32) -> None:
        self.yaw_navigation = float(message.data)
        self.yaw_world = nav_to_world_yaw(self.yaw_navigation)

    def _boundary_callback(self, message: MarkerArray) -> None:
        if self.position is None:
            return
        for marker in message.markers:
            points = [base_to_world((point.x, point.y), self.position, self.yaw_world) for point in marker.points]
            if marker.ns == "road_left":
                self.left_world = points
            elif marker.ns == "road_right":
                self.right_world = points

    def _obstacle_callback(self, message: MarkerArray) -> None:
        if not self.use_obstacle:
            self.obstacles = []
            return
        if self.position is None:
            return
        obstacles = []
        for marker in message.markers:
            # Only "tall" obstacles participate in the Frenet lateral corridor.
            # "flat_ground" (special terrain) is handled by path_follower's
            # longitudinal derating and must NOT trigger lateral avoidance.
            # Markers without a recognised ns are ignored (no silent default).
            if marker.ns != "tall":
                continue
            # Ignore spurious self/clutter detections at the sensor origin.
            if math.hypot(marker.pose.position.x, marker.pose.position.y) < self.min_obstacle_range:
                continue
            x, y = base_to_world((marker.pose.position.x, marker.pose.position.y), self.position, self.yaw_world)
            q = marker.pose.orientation
            relative_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            obstacles.append({
                "id": marker.id, "x": x, "y": y,
                "yaw": wrap_angle(self.yaw_world + relative_yaw),
                "length": marker.scale.x, "width": marker.scale.y, "height": marker.scale.z,
            })
        self.obstacles = obstacles

    @staticmethod
    def _closest_lateral(reference, boundary):
        if not boundary:
            return None
        closest = min(boundary, key=lambda point: (point[0] - reference["x"]) ** 2 + (point[1] - reference["y"]) ** 2)
        return signed_lateral(closest, reference)

    def _nearest_wrap(self, last: int) -> int:
        """Wrap-aware nearest centreline index (follows the car around a loop).

        Searches the *entire* loop for the geometrically nearest centreline
        point.  A small forward-biased window (e.g. -6..+70) let the index
        drift steadily *ahead* of the vehicle: each frame it would pick a
        point slightly further forward, and after the seam the published
        ``/planned_path`` would stretch from the car all the way back to the
        stale index — the long, infinitely-extended line seen in circle mode.
        Locking onto the true nearest point keeps the path anchored to the
        vehicle at every frame.
        """
        n = len(self.centerline)
        if n == 0:
            return 0
        best_i = last % n
        best_d2 = float("inf")
        for k in range(n):
            i = (last + k) % n
            dx = self.centerline[i]["x"] - self.position[0]
            dy = self.centerline[i]["y"] - self.position[1]
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2, best_i = d2, i
        return best_i

    def _plan(self) -> None:
        if self.position is None or len(self.centerline) < 3:
            return
        n = len(self.centerline)
        loop = math.hypot(self.centerline[0]["x"] - self.centerline[-1]["x"],
                          self.centerline[0]["y"] - self.centerline[-1]["y"]) < 3.0
        if loop:
            self.last_nearest = self._nearest_wrap(self.last_nearest)
        else:
            self.last_nearest = nearest_index(self.centerline, self.position[0], self.position[1], max(0, self.last_nearest - 4))
        left_limits = [point["half_width"] for point in self.centerline]
        right_limits = [-point["half_width"] for point in self.centerline]
        horizon_samples = int(self.config.horizon_m / 0.5) + 8
        if loop:
            idx_window = [(self.last_nearest + k) % n for k in range(horizon_samples)]
        else:
            idx_window = list(range(self.last_nearest, min(n, self.last_nearest + horizon_samples)))
        # P2: 走廊按车位置自适应扩张 — 车已偏离中心线时, 几何中心走廊会逼车"急切回中",
        # 但反馈已饱和+EPS跟不上时根本切不回, 反而因走廊窄报 infeasible 或贴边触发
        # EMERGENCY. 这里把车所在侧的走廊往外扩 offset_margin, 给车"就地缓缓回正"空间.
        vehicle_lateral = signed_lateral(self.position, self.centerline[self.last_nearest])
        offset_margin = 0.5  # m, 车偏离>阈值时单侧扩宽
        offset_threshold = 0.5  # m, 触发阈值
        if abs(vehicle_lateral) > offset_threshold:
            if vehicle_lateral > 0:
                # 车在中心线左侧 → 左走廊往外扩
                for index in idx_window:
                    left_limits[index] += offset_margin
            else:
                # 车在中心线右侧 → 右走廊往外扩(更负)
                for index in idx_window:
                    right_limits[index] -= offset_margin
        for index in idx_window:
            # Compute left/right limits independently — ns labels guarantee
            # which boundary is which, no max/min cross-mixing needed.
            left = self._closest_lateral(self.centerline[index], self.left_world)
            right = self._closest_lateral(self.centerline[index], self.right_world)
            if left is not None:
                left_limits[index] = max(left_limits[index], left)
            if right is not None:
                right_limits[index] = min(right_limits[index], right)
            # 无条件兜底：真实边界缺失或退化(半宽 < min_half_width)时，
            # 用中心线点自带 half_width(闭环中段 3.75m/直线 4.0m, fallback default_half_width)
            # 沿法向展开 ±half_width 作为保底走廊。仿真实车统一, 不依赖任何外部边界消息源。
            ref_hw = self.centerline[index].get("half_width", self.default_half_width)
            if ref_hw < self.min_half_width:
                ref_hw = self.default_half_width
            if left is None or left_limits[index] < self.min_half_width:
                left_limits[index] = ref_hw
            if right is None or -right_limits[index] < self.min_half_width:
                right_limits[index] = -ref_hw
        result = plan_frenet_path(
            self.centerline, self.last_nearest, self.position,
            left_limits, right_limits, self.obstacles, self.config,
            closed_loop=loop,
        )
        self.status_pub.publish(String(data="FEASIBLE" if result.feasible else f"INFEASIBLE: {result.reason}"))
        self.plan_time_pub.publish(Float32(data=float(result.planning_ms)))
        self.clearance_pub.publish(Float32(data=float(result.min_clearance)))

        message = PathMessage()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        for index, point in enumerate(result.path):
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point[0], point[1], 0.12
            if index + 1 < len(result.path):
                yaw = math.atan2(result.path[index + 1][1] - point[1], result.path[index + 1][0] - point[0])
            else:
                yaw = self.centerline[min(self.last_nearest + index * 2, len(self.centerline) - 1)]["yaw"]
            _, _, pose.pose.orientation.z, pose.pose.orientation.w = yaw_to_quaternion(yaw)
            message.poses.append(pose)
        self.path_pub.publish(message)
        self._publish_debug(result)

    def _publish_debug(self, result) -> None:
        array = MarkerArray()
        # Transparent red "inflated_obstacles" box visualises the planner's
        # internal safety margin for every *tall* obstacle. Road-edge tires are
        # pure Gazebo cylinders (not published on /obstacle_markers) and never
        # enter self.obstacles, so they are intentionally excluded from this view —
        # tires exist only for the perception group's landmark detection, not for
        # avoidance, and must not clutter the avoidance debug overlay.
        for index, obstacle in enumerate(self.obstacles):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "inflated_obstacles"
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = obstacle["x"]
            marker.pose.position.y = obstacle["y"]
            marker.pose.position.z = 0.06
            _, _, marker.pose.orientation.z, marker.pose.orientation.w = yaw_to_quaternion(obstacle["yaw"])
            marker.scale.x = obstacle["length"] + self.config.vehicle_length + 2 * self.config.safety_margin
            marker.scale.y = obstacle["width"] + self.config.vehicle_width + 2 * self.config.safety_margin
            marker.scale.z = 0.05
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.1, 0.05, 0.18
            marker.lifetime.nanosec = 180_000_000
            array.markers.append(marker)
        text = Marker()
        text.header.frame_id = "base_link"
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns, text.id, text.type, text.action = "planner_status", 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose.position.x, text.pose.position.y, text.pose.position.z = 0.0, 0.0, 2.0
        text.scale.z = 0.34
        text.color.r, text.color.g, text.color.b, text.color.a = (0.2, 1.0, 0.4, 1.0) if result.feasible else (1.0, 0.15, 0.1, 1.0)
        text.text = f"Planner: {'OK' if result.feasible else 'STOP'} | {result.planning_ms:.1f} ms | clearance {result.min_clearance:.2f} m"
        text.lifetime.nanosec = 180_000_000
        array.markers.append(text)
        self.debug_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrenetPlannerNode()
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
