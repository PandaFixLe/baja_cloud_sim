"""Real-car remote teleop launch: UDP remote control + CAN bridge.

Starts the real-car teleop (视驾) stack:
  Hardware: chcnav (GNSS+IMU), lslidar (LiDAR)
  Perception: lidar3d_bringup (optional, for situational awareness)
  Control: remote_control (UDP) → can_bridge → VCU
  Logging: evaluator (real-car mode, no planned_path)

remote_control and path_follower both publish /cmd_control but are mutually
exclusive — only one control source runs at a time. This launch uses
remote_control; use real_car.launch.py for autonomous mode.

Usage:
  ros2 launch baja_cloud_sim real_car_remote.launch.py
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

    use_rviz = LaunchConfiguration("use_rviz")
    use_boundary = LaunchConfiguration("use_boundary")
    use_obstacle = LaunchConfiguration("use_obstacle")

    # use_boundary / use_obstacle 默认值从 yaml(real_car_params.yaml) 读取，
    # 命令行 --use-boundary/--use-obstacle 仍可临时覆盖。
    default_boundary = _yaml_bool(params, "frenet_planner_node", "use_boundary", True)
    default_obstacle = _yaml_bool(params, "frenet_planner_node", "use_obstacle", True)

    hw_remappings = [
        ("/chcnav/devpvt", "/gps/fix"),
        ("/imu_yaw", "/imu/yaw"),
        ("/chcnav/odom", "/ground_truth/odom"),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_boundary", default_value=default_boundary,
                              description="true: road_analyzer 发布真实车道线(/road_boundary_markers)。"
                                          "false: Frenet 改用中心线 ±half_width 兜底走廊。"
                                          "默认取自 real_car_params.yaml frenet_planner_node.use_boundary。"),
        DeclareLaunchArgument("use_obstacle", default_value=default_obstacle,
                              description="true: 启动障碍检测 → /obstacle_markers。"
                                          "false: 关闭障碍检测(视驾模式下一般关闭)。"
                                          "默认取自 real_car_params.yaml frenet_planner_node.use_obstacle。"),

        # ── Hardware: CHCNAV ──
        Node(
            package="chcnav",
            executable="chcnav_full_node",
            name="chcnav_full_node",
            output="screen",
        ),

        # ── Hardware: LSLiDAR ──
        Node(
            package="lslidar_driver",
            executable="lslidar_driver_node",
            name="lslidar_driver_node",
            namespace="cx",
            output="screen",
        ),

        # ── Static TF ──
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="laser_link_tf",
            arguments=["-0.5", "0", "1.05", "0", "0", "0", "base_link", "laser_link"],
        ),

        # ── Perception (optional, 视驾模式下一般只需障碍感知) ──
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

        # ── Remote control (UDP → /cmd_control) ──
        Node(
            package="baja_cloud_sim",
            executable="remote_control",
            name="remote_control_node",
            parameters=[params],
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

        # ── Evaluator (real-car mode, tracks teleop metrics) ──
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
