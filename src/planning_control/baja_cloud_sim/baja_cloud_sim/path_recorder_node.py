"""Real-car path recorder: records GPS+IMU waypoints to CSV for later Frenet planning.

Subscribes to the real-car localization topics (remapped in launch):
  - /chcnav/devpvt  (NavSatFix)  → GPS position
  - /imu_yaw        (Float32, degrees) → heading

Outputs a CSV file with columns:
  latitude, longitude, yaw_rad, yaw_deg, altitude

The CSV is consumed by csv_to_centerline_node to build /reference_centerline
for the Frenet planner on the next run.
"""

from __future__ import annotations

import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32


class PathRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__("path_recorder")

        # Parameters
        self.declare_parameter("output_file", "recorded_path.csv")
        self.declare_parameter("min_point_distance", 0.5)
        self.declare_parameter("auto_start", False)

        output_file = str(self.get_parameter("output_file").value)
        self.min_distance = float(self.get_parameter("min_point_distance").value)
        self.auto_start = bool(self.get_parameter("auto_start").value)

        # State
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_yaw = 0.0  # radians
        self.current_alt = 0.0
        self.recording = False
        self.point_count = 0
        self.last_lat = 0.0
        self.last_lon = 0.0
        self.gps_ready = False
        self.start_time = 0.0

        # Resolve output path
        self.save_file = os.path.abspath(output_file)
        os.makedirs(os.path.dirname(self.save_file) or ".", exist_ok=True)

        # Subscriptions (topic names are remapped in launch files)
        self.create_subscription(NavSatFix, "/gps/fix", self._gps_callback, 10)
        self.create_subscription(Float32, "/imu/yaw", self._imu_callback, 10)

        # Status timer
        self.create_timer(2.0, self._print_status)

        self.get_logger().info("=" * 60)
        self.get_logger().info("Path Recorder ready")
        self.get_logger().info(f"  Output: {self.save_file}")
        self.get_logger().info(f"  Min point distance: {self.min_distance} m")
        self.get_logger().info("=" * 60)

        if self.auto_start:
            self._wait_for_gps_and_start()
        else:
            self.create_timer(0.5, self._wait_for_gps_timer)

    def _wait_for_gps_timer(self) -> None:
        """Wait for GPS in non-auto mode, then prompt for Enter."""
        if not self.gps_ready:
            return
        # GPS ready, remove this timer and wait for user input
        for timer in self._timers_to_cancel():
            timer.cancel()
        self.get_logger().info(
            f"GPS ready: ({self.current_lat:.7f}, {self.current_lon:.7f})  "
            f"yaw={math.degrees(self.current_yaw):.1f} deg"
        )
        try:
            input("\nPress Enter to start recording...")
        except (EOFError, KeyboardInterrupt):
            pass
        self._start_recording()

    def _timers_to_cancel(self):
        """Return timers created by this node (workaround for attribute access)."""
        return [t for t in getattr(self, "_timers", [])]

    def _wait_for_gps_and_start(self) -> None:
        """Auto-start mode: wait for GPS then start immediately."""
        count = 0
        while rclpy.ok() and not self.gps_ready:
            rclpy.spin_once(self, timeout_sec=0.5)
            count += 1
            if count % 4 == 0:
                self.get_logger().info(f"Waiting for GPS... ({count} frames received)")
        if self.gps_ready:
            self._start_recording()

    def _gps_callback(self, msg: NavSatFix) -> None:
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude

        if abs(self.current_lat) < 1e-6:
            return

        if not self.gps_ready:
            self.gps_ready = True
            self.get_logger().info(
                f"GPS acquired: ({self.current_lat:.7f}, {self.current_lon:.7f})"
            )

        if not self.recording or self.last_lat == 0.0:
            return

        # Distance from last recorded point
        dlat = (self.current_lat - self.last_lat) * 111320
        dlon = (self.current_lon - self.last_lon) * 111320 * math.cos(
            math.radians(self.current_lat)
        )
        dist = math.hypot(dlat, dlon)

        if dist >= self.min_distance:
            self._write_point()
            self.last_lat = self.current_lat
            self.last_lon = self.current_lon

    def _imu_callback(self, msg: Float32) -> None:
        """Yaw callback: expects degrees (from chcnav /imu_yaw)."""
        yaw_deg = float(msg.data) % 360.0
        self.current_yaw = math.radians(yaw_deg)

    def _start_recording(self) -> None:
        self.file = open(self.save_file, "w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(
            ["latitude", "longitude", "yaw_rad", "yaw_deg", "altitude"]
        )

        self.last_lat = self.current_lat
        self.last_lon = self.current_lon
        self._write_point()
        self.point_count = 1
        self.recording = True
        self.start_time = time.time()

        self.get_logger().info("=" * 60)
        self.get_logger().info("Recording started")
        self.get_logger().info(
            f"  Start: ({self.current_lat:.7f}, {self.current_lon:.7f})  "
            f"yaw={math.degrees(self.current_yaw):.1f} deg"
        )
        self.get_logger().info("  Drive the vehicle along the desired path.")
        self.get_logger().info("  Press Ctrl+C to stop recording.")
        self.get_logger().info("=" * 60)

    def _write_point(self) -> None:
        self.writer.writerow(
            [
                f"{self.current_lat:.7f}",
                f"{self.current_lon:.7f}",
                f"{self.current_yaw:.6f}",
                f"{math.degrees(self.current_yaw):.2f}",
                f"{self.current_alt:.2f}",
            ]
        )
        self.point_count += 1

    def _print_status(self) -> None:
        if not self.recording:
            if not self.gps_ready:
                self.get_logger().info("Waiting for GPS data...")
            return
        elapsed = time.time() - self.start_time
        distance = (self.point_count - 1) * self.min_distance
        speed = distance / elapsed if elapsed > 0 else 0.0
        self.get_logger().info(
            f"Recorded {self.point_count} points | "
            f"distance: {distance:.1f} m | "
            f"speed: {speed:.2f} m/s | "
            f"yaw: {math.degrees(self.current_yaw):.1f} deg"
        )

    def destroy_node(self) -> bool:
        if hasattr(self, "file") and hasattr(self.file, "closed") and not self.file.closed:
            self.file.close()
            elapsed = time.time() - getattr(self, "start_time", time.time())
            distance = (self.point_count - 1) * self.min_distance if hasattr(self, "point_count") else 0
            self.get_logger().info("=" * 60)
            self.get_logger().info("Recording complete!")
            self.get_logger().info(f"  Points: {getattr(self, 'point_count', 0)}")
            self.get_logger().info(f"  Distance: {distance:.1f} m")
            if elapsed > 0:
                self.get_logger().info(f"  Duration: {elapsed:.1f} s")
                self.get_logger().info(f"  Avg speed: {distance / elapsed:.2f} m/s")
            self.get_logger().info(f"  Saved: {self.save_file}")
            self.get_logger().info("=" * 60)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\nRecording interrupted by user.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
