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

import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _yaml_bool(params_path, node, key, fallback):
    """从 yaml 的 node.ros__parameters[key] 读取 bool 默认；缺失则 fallback。"""
    try:
        with open(params_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        val = data.get(node, {}).get("ros__parameters", {}).get(key)
        if isinstance(val, bool):
            return "true" if val else "false"
    except FileNotFoundError:
        pass
    return "true" if fallback else "false"


def generate_launch_description():
    share = Path(get_package_share_directory("baja_cloud_sim"))
    params = str(share / "config" / "real_car_params.yaml")
    rviz = str(share / "config" / "real_car.rviz")

    csv_file = LaunchConfiguration("csv_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_boundary = LaunchConfiguration("use_boundary")
    use_obstacle = LaunchConfiguration("use_obstacle")

    # use_boundary / use_obstacle 默认值从 yaml(real_car_params.yaml) 读取，
    # 命令行 --use-boundary/--use-obstacle 仍可临时覆盖。
    default_boundary = _yaml_bool(params, "frenet_planner_node", "use_boundary", True)
    default_obstacle = _yaml_bool(params, "frenet_planner_node", "use_obstacle", True)

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
        DeclareLaunchArgument("use_boundary", default_value=default_boundary,
                              description="true: road_analyzer 发布真实车道线(/road_boundary_markers)。"
                                          "false: Frenet 改用中心线 ±half_width 兜底走廊。"
                                          "默认取自 real_car_params.yaml frenet_planner_node.use_boundary。"),
        DeclareLaunchArgument("use_obstacle", default_value=default_obstacle,
                              description="true: 启动障碍检测 → /obstacle_markers。"
                                          "false: 关闭障碍检测, frenet_planner 不消费障碍消息。"
                                          "默认取自 real_car_params.yaml frenet_planner_node.use_obstacle。"),

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

        # ── Perception: LiDAR 3D perception (patchwork++ + road_analyzer + obstacle) ──
        # cloud_topic 直接从实车 lslidar 驱动读取(/cx/lslidar_point_cloud)。
        # use_boundary / use_obstacle 独立控制车道线与障碍。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(get_package_share_directory("lidar3d_bringup")) / "launch" / "lidar_sim.launch.py")),
            launch_arguments={
                "use_rviz": "false",
                "cloud_topic": "/cx/lslidar_point_cloud",
                "target_frame": "base_link",
                "use_boundary": use_boundary,
                "use_obstacle": use_obstacle,
            }.items(),
            condition=IfCondition(
                PythonExpression(["'", use_boundary, "' == 'true' or '", use_obstacle, "' == 'true'"])),
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
