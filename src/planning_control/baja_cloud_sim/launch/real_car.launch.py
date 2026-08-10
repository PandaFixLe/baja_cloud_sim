"""Real-car autonomous launch: LQR path following + Frenet planning + CAN bridge.

Starts the full real-car autonomous stack:
  Hardware: chcnav (GNSS+IMU), lslidar (LiDAR)
  Perception: lidar3d_bringup (obstacles + road boundaries)
  Planning: csv_to_centerline → frenet_planner
  Control: path_follower → can_bridge → VCU
  Logging: evaluator (real-car mode)

Topic remaps connect real-car hardware topics to the same names used in
simulation (/gps/fix, /imu/yaw, /ground_truth/odom) so core nodes are unchanged.

Usage:
  ros2 launch baja_cloud_sim real_car.launch.py csv_file:=/path/to/recorded_path.csv
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("baja_cloud_sim"))
    params = str(share / "config" / "real_car_params.yaml")
    rviz = str(share / "config" / "real_car.rviz")

    csv_file = LaunchConfiguration("csv_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_perception = LaunchConfiguration("use_perception")

    # Common remaps: real-car hardware topics → simulation-standard names
    hw_remappings = [
        ("/chcnav/devpvt", "/gps/fix"),
        ("/imu_yaw", "/imu/yaw"),
        ("/chcnav/odom", "/ground_truth/odom"),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("csv_file", default_value="recorded_path.csv",
                              description="Recorded CSV path file for Frenet centerline"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_perception", default_value="true",
                              description="Start LiDAR perception chain (lidar3d_bringup)"),

        # ── Hardware: CHCNAV combined navigation (GNSS + IMU on vcan2) ──
        Node(
            package="chcnav",
            executable="chcnav_full_node",
            name="chcnav_full_node",
            output="screen",
        ),

        # ── Hardware: LSLiDAR driver (CX series, namespace=cx) ──
        Node(
            package="lslidar_driver",
            executable="lslidar_driver_node",
            name="lslidar_driver_node",
            namespace="cx",
            output="screen",
        ),

        # ── Static TF: base_link → laser_link (LiDAR mounting) ──
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="laser_link_tf",
            arguments=["-0.5", "0", "1.05", "0", "0", "0", "base_link", "laser_link"],
        ),

        # ── Perception: LiDAR 3D perception (patchwork++ + clustering) ──
        # Input point cloud remapped from /cx/lslidar_point_cloud
        Node(
            package="lidar3d_bringup",
            executable="lidar_sim",
            name="lidar3d_perception",
            parameters=[{"use_sim_time": False}],
            remappings=[("/lidar/points", "/cx/lslidar_point_cloud")],
            condition=IfCondition(use_perception),
            output="screen",
        ),

        # ── Path centerline from recorded CSV ──
        Node(
            package="baja_cloud_sim",
            executable="csv_to_centerline",
            name="csv_to_centerline_node",
            parameters=[params, {"csv_file": csv_file}],
            output="screen",
        ),

        # ── Frenet online planner ──
        Node(
            package="baja_cloud_sim",
            executable="frenet_planner",
            name="frenet_planner_node",
            parameters=[params],
            remappings=hw_remappings,
            output="screen",
        ),

        # ── LQR path follower (control) ──
        Node(
            package="baja_cloud_sim",
            executable="path_follower",
            name="path_follower_node",
            parameters=[params],
            remappings=hw_remappings,
            output="screen",
        ),

        # ── CAN bridge: /cmd_control → VCU ──
        Node(
            package="car_autonomous_pkg",
            executable="can_bridge_node",
            name="can_bridge_node",
            parameters=[params],
            output="screen",
        ),

        # ── Evaluator (real-car mode) ──
        Node(
            package="baja_cloud_sim",
            executable="evaluator",
            name="evaluator_node",
            parameters=[params],
            remappings=hw_remappings,
            output="screen",
        ),

        # ── RViz ──
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
    ])
