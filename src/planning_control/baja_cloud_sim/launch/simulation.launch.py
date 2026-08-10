from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
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
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    world = LaunchConfiguration("world_file")
    scenario = LaunchConfiguration("scenario_file")
    results = LaunchConfiguration("results_dir")
    use_rviz = LaunchConfiguration("use_rviz")
    use_gz_gui = LaunchConfiguration("use_gz_gui")
    use_video = LaunchConfiguration("use_video")
    video_path = LaunchConfiguration("video_path")
    finish_mode = LaunchConfiguration("finish_mode")
    use_boundary = LaunchConfiguration("use_boundary")
    use_obstacle = LaunchConfiguration("use_obstacle")
    params = str(share / "config" / "params.yaml")
    bridge = str(share / "config" / "bridge.yaml")
    rviz = str(share / "config" / "simulation.rviz")
    robot_description = (share / "urdf" / "baja_vehicle.urdf").read_text(encoding="utf-8")

    # use_boundary / use_obstacle 默认值从 yaml(params.yaml) 读取，
    # 命令行 --use-boundary/--use-obstacle 仍可临时覆盖。
    default_boundary = _yaml_bool(params, "frenet_planner_node", "use_boundary", True)
    default_obstacle = _yaml_bool(params, "frenet_planner_node", "use_obstacle", True)

    return LaunchDescription([
        DeclareLaunchArgument("world_file"),
        DeclareLaunchArgument("scenario_file"),
        DeclareLaunchArgument("results_dir", default_value="results"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_gz_gui", default_value="true"),
        DeclareLaunchArgument("use_video", default_value="true"),
        DeclareLaunchArgument("video_path", default_value="results/gazebo.mp4"),
        DeclareLaunchArgument("finish_mode", default_value="none",
                              description="Finish/终点逻辑: none | line | circle | time"),
    DeclareLaunchArgument("use_boundary", default_value=default_boundary,
                          description="true: 启动 road_analyzer 发布真实车道线(/road_boundary_markers)。"
                                      "false: Frenet 改用中心线 ±half_width 兜底走廊(不依赖车道线检测)。"
                                      "默认取自 params.yaml frenet_planner_node.use_boundary。"),
    DeclareLaunchArgument("use_obstacle", default_value=default_obstacle,
                          description="true: 启动障碍检测链路(gz_pcl_bridge + lidar_sim 障碍部分)，"
                                      "发布 /obstacle_markers。false: 关闭障碍检测，frenet_plceiver 不消费障碍消息。"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(ros_gz_share / "launch" / "gz_sim.launch.py")),
            launch_arguments={"gz_args": ["-r -v 3 --render-engine ogre --render-engine-gui ogre ", world]}.items(),
            condition=IfCondition(use_gz_gui),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(ros_gz_share / "launch" / "gz_sim.launch.py")),
            launch_arguments={"gz_args": ["-r -s -v 3 --headless-rendering --render-engine ogre ", world]}.items(),
            condition=UnlessCondition(use_gz_gui),
        ),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="ros_gz_bridge", parameters=[{"config_file": bridge, "use_sim_time": True}], output="screen"),
        # Workaround for ros_gz_bridge corrupting gz.msgs.PointCloudPacked ->
        # sensor_msgs/msg/PointCloud2 (all points -inf). This node subscribes to
        # the raw Gazebo topic and republishes a correct ROS PointCloud2.
        # 仅在开启任意感知(use_boundary 或 use_obstacle)时启动，它依赖 LiDAR 点云话题。
        Node(package="baja_cloud_sim", executable="gz_pcl_bridge", name="gz_pcl_bridge", parameters=[{"use_sim_time": True}], output="screen", condition=IfCondition(PythonExpression(["'", use_boundary, "' == 'true' or '", use_obstacle, "' == 'true'"]))),
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher", parameters=[{"robot_description": robot_description, "use_sim_time": True}], output="screen"),
        Node(package="baja_cloud_sim", executable="truth_perception", name="truth_perception_node", parameters=[params, {"scenario_file": scenario, "use_sim_time": True}], output="screen"),
        Node(package="baja_cloud_sim", executable="frenet_planner", name="frenet_planner_node", parameters=[params, {"use_sim_time": True}], output="screen"),
        Node(package="baja_cloud_sim", executable="path_follower", name="path_follower_node", parameters=[params, {"finish_mode": finish_mode, "use_sim_time": True}], output="screen"),
        Node(package="baja_cloud_sim", executable="actuator_adapter", name="actuator_adapter_node", parameters=[params, {"use_sim_time": True}], output="screen"),
        Node(package="baja_cloud_sim", executable="evaluator", name="evaluator_node", parameters=[params, {"scenario_file": scenario, "results_dir": results, "use_sim_time": True}], output="screen"),
        Node(package="baja_cloud_sim", executable="video_recorder", name="video_recorder_node", parameters=[{"video_path": video_path, "use_sim_time": True}], condition=IfCondition(use_video), output="screen"),
        Node(package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", rviz], parameters=[{"use_sim_time": True}], condition=IfCondition(use_rviz), output="screen"),
        # ── 感知链路开关 ──
        # use_boundary / use_obstacle 两个独立开关:
        #   use_boundary=true (默认): road_analyzer 发布真实车道线 → /road_boundary_markers
        #   use_boundary=false:        Frenet 改用中心线 ±half_width 兜底走廊(不依赖车道线检测)
        #   use_obstacle=true (默认):  启动障碍检测 → /obstacle_markers
        #   use_obstacle=false:        关闭障碍检测, frenet_planner 不消费障碍消息
        # 两者可在 yaml(params.yaml) 或命令行 --use-boundary/--use-obstacle 便捷调整。
        # truth_perception 不再发布车道线(已移除真值边界兜底); 感知组整体在
        # use_boundary 或 use_obstacle 任一为真时启动。
        # Perception group pipeline (lidar3d_bringup + patchwork++ + lidar3d_perception_cpp).
        # Provides /obstacle_markers (obstacle_adapter, 受 use_obstacle 控制) and
        # /road_boundary_markers (road_analyzer, 受 use_boundary 控制 remap)。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory("lidar3d_bringup")) / "launch" / "lidar_sim.launch.py")),
            launch_arguments={"use_rviz": "false", "use_boundary": use_boundary, "use_obstacle": use_obstacle}.items(),
            condition=IfCondition(PythonExpression(["'", use_boundary, "' == 'true' or '", use_obstacle, "' == 'true'"])),
        ),
    ])
