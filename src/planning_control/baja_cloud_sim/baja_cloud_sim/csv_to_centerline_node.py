"""Load a recorded CSV waypoint file and publish it as /reference_centerline.

This node bridges the real-car path recording workflow with the Frenet planner:
  1. path_recorder drives the track once → recorded_path.csv (lat, lon, yaw, alt)
  2. csv_to_centerline loads the CSV → publishes /reference_centerline (Path msg)
  3. frenet_planner receives /reference_centerline → online Frenet planning
  4. path_follower receives /planned_path → LQR tracking control

The CSV first point is used as the GPS origin (origin_latitude/longitude).
All waypoints are projected to local XY using gps_to_local (same algorithm as
path_follower and frenet_planner, ensuring coordinate consistency).
"""

from __future__ import annotations

import csv
import math
import os
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .core import gps_to_local, yaw_to_quaternion


class CsvToCenterlineNode(Node):
    def __init__(self) -> None:
        super().__init__("csv_to_centerline")

        # Parameters
        self.declare_parameter("csv_file", "")
        self.declare_parameter("origin_latitude", 0.0)
        self.declare_parameter("origin_longitude", 0.0)
        self.declare_parameter("auto_origin", True)
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("frame_id", "map")

        csv_file = str(self.get_parameter("csv_file").value)
        self.auto_origin = bool(self.get_parameter("auto_origin").value)
        self.origin_lat = float(self.get_parameter("origin_latitude").value)
        self.origin_lon = float(self.get_parameter("origin_longitude").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        publish_rate = float(self.get_parameter("publish_rate").value)

        if not csv_file:
            self.get_logger().error("csv_file parameter is required")
            raise RuntimeError("csv_file parameter is required")

        # Resolve CSV path (search common locations)
        self.csv_path = self._resolve_csv_path(csv_file)
        if not self.csv_path:
            self.get_logger().error(f"CSV file not found: {csv_file}")
            raise RuntimeError(f"CSV file not found: {csv_file}")

        # Load waypoints
        self.waypoints = self._load_csv()
        if not self.waypoints:
            self.get_logger().error("No waypoints loaded from CSV")
            raise RuntimeError("No waypoints loaded from CSV")

        # Auto-set origin from first waypoint if requested
        if self.auto_origin and self.origin_lat == 0.0 and self.origin_lon == 0.0:
            self.origin_lat = self.waypoints[0]["lat"]
            self.origin_lon = self.waypoints[0]["lon"]
            self.get_logger().info(
                f"Auto-set GPS origin from first waypoint: "
                f"({self.origin_lat:.7f}, {self.origin_lon:.7f})"
            )

        # Latched QoS so late-joining subscribers receive the centerline
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.centerline_pub = self.create_publisher(
            PathMessage, "/reference_centerline", latched
        )

        # Also publish GPS origin so other nodes (path_follower, frenet_planner)
        # can pick it up if they need it. Published once as a latched parameter-like topic.
        # NOTE: path_follower and frenet_planner read origin from their own params,
        # so the launch file must pass the same origin_latitude/longitude to all nodes.
        # This is just for logging/verification.
        self.get_logger().info(
            f"CSV centerline ready: {len(self.waypoints)} waypoints, "
            f"origin=({self.origin_lat:.7f}, {self.origin_lon:.7f})"
        )

        # Publish immediately (latched) and periodically
        self._publish_centerline()
        if publish_rate > 0:
            self.create_timer(1.0 / publish_rate, self._publish_centerline)

    def _resolve_csv_path(self, csv_file: str) -> str | None:
        """Search common locations for the CSV file."""
        candidates = [
            csv_file,
            os.path.expanduser(csv_file),
            os.path.join(os.getcwd(), csv_file),
        ]
        # Also search relative to the package share directory
        try:
            from ament_index_python.packages import get_package_share_directory

            share_dir = get_package_share_directory("baja_cloud_sim")
            candidates.append(os.path.join(share_dir, csv_file))
            candidates.append(os.path.join(share_dir, "path", os.path.basename(csv_file)))
        except Exception:
            pass

        for candidate in candidates:
            if os.path.isfile(candidate):
                return candidate
        return None

    def _load_csv(self) -> list:
        """Load CSV waypoints.

        Expected columns: latitude, longitude, yaw_rad, yaw_deg, altitude
        (as produced by path_recorder_node). Falls back to lat, lon only if
        yaw is missing.
        """
        waypoints = []
        with open(self.csv_path, "r", encoding="utf-8") as f:
            reader = csv.reader(f)
            header = next(reader, None)

            for row in reader:
                if len(row) < 2:
                    continue
                try:
                    lat = float(row[0])
                    lon = float(row[1])
                    yaw_rad = float(row[2]) if len(row) >= 3 and row[2] else 0.0
                    alt = float(row[4]) if len(row) >= 5 and row[4] else 0.0
                except (ValueError, IndexError):
                    continue
                waypoints.append({"lat": lat, "lon": lon, "yaw": yaw_rad, "alt": alt})

        self.get_logger().info(f"Loaded {len(waypoints)} waypoints from {self.csv_path}")
        return waypoints

    def _publish_centerline(self) -> None:
        """Publish /reference_centerline as a Path message in local XY frame."""
        message = PathMessage()
        message.header.frame_id = self.frame_id
        message.header.stamp = self.get_clock().now().to_msg()

        for wp in self.waypoints:
            x, y = gps_to_local(
                wp["lat"], wp["lon"], self.origin_lat, self.origin_lon
            )
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = wp["alt"]
            # Use recorded yaw if available, else compute from segment direction
            yaw = wp["yaw"]
            _, _, pose.pose.orientation.z, pose.pose.orientation.w = yaw_to_quaternion(yaw)
            message.poses.append(pose)

        self.centerline_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CsvToCenterlineNode()
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
