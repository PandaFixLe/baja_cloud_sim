from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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
    use_perception = LaunchConfiguration("use_perception")
    params = str(share / "config" / "params.yaml")
    bridge = str(share / "config" / "bridge.yaml")
    rviz = str(share / "config" / "simulation.rviz")
    robot_description = (share / "urdf" / "baja_vehicle.urdf").read_text(encoding="utf-8")

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
    DeclareLaunchArgument("use_perception", default_value="true",
                          description="true: 启动 LiDAR 感知链路(lidar_sim + gz_pcl_bridge)。"
                                      "false: 关闭感知，仅用 truth_perception 发的车道线真值"
                                      "跑规划控制核心(单独验证算法用)。"),
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
        # 仅在开启感知(use_perception=true)时启动，它依赖 LiDAR 点云话题。
        Node(package="baja_cloud_sim", executable="gz_pcl_bridge", name="gz_pcl_bridge", parameters=[{"use_sim_time": True}], output="screen", condition=IfCondition(use_perception)),
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher", parameters=[{"robot_description": robot_description, "use_sim_time": True}], output="screen"),
        Node(package="baja_cloud_sim", executable="truth_perception", name="truth_perception_node", parameters=[params, {"scenario_file": scenario, "publish_ground_truth_boundary": PythonExpression(["'", use_perception, "' == 'false'"]), "use_sim_time": True}], output="screen"),
        Node(package="baja_cloud_sim", executable="frenet_planner", name="frenet_planner_node", parameters=[params, {"use_sim_time": True}], output="screen"),
        Node(package="baja_cloud_sim", executable="path_follower", name="path_follower_node", parameters=[params, {"finish_mode": finish_mode, "use_sim_time": True}], output="screen"),
        Node(package="baja_cloud_sim", executable="actuator_adapter", name="actuator_adapter_node", parameters=[params, {"use_sim_time": True}], output="screen"),
        Node(package="baja_cloud_sim", executable="evaluator", name="evaluator_node", parameters=[params, {"scenario_file": scenario, "results_dir": results, "use_sim_time": True}], output="screen"),
        Node(package="baja_cloud_sim", executable="video_recorder", name="video_recorder_node", parameters=[{"video_path": video_path, "use_sim_time": True}], condition=IfCondition(use_video), output="screen"),
        Node(package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", rviz], parameters=[{"use_sim_time": True}], condition=IfCondition(use_rviz), output="screen"),
        # ── 感知链路开关 ──
        # use_perception=true(默认): 启动完整 LiDAR 感知组(lidar_sim)，车道线由
        #   road_analyzer 发布，truth_perception 的 publish_ground_truth_boundary=false(避免双发布冲突)。
        # use_perception=false: 关闭 LiDAR 感知(gz_pcl_bridge + lidar_sim 不启动)，
        #   truth_perception 的 publish_ground_truth_boundary 自动=true，由 truth_perception
        #   直接发车道线真值，让规划控制核心在无感知下也能跑(单独验证算法用)。
        # 注意: 之前用 SetLaunchConfiguration 覆盖存在时序/作用域 bug，改用 PythonExpression
        # 在 parameters 内直接按 use_perception 求值(执行阶段才解析，可靠)。
        # Perception group pipeline (lidar3d_bringup + patchwork++ + lidar3d_perception_cpp).
        # Provides /obstacle_markers (obstacle_adapter) and /road_boundary_markers
        # (road_analyzer) to the planning & control core. In lidar mode the
        # perception group's road_analyzer is the SOLE publisher of
        # /road_boundary_markers (remapped from /lidar/road_boundary_markers).
        # truth_perception_node no longer publishes ground-truth boundaries by
        # default (publish_ground_truth_boundary=false) to avoid a second
        # publisher conflicting on the same topic.
        # Road boundaries (/road_boundary_markers) come from truth_perception's ground-truth
        # scenario data, NOT from road_analyzer — the perception group's lane-edge detection
        # is too unreliable on sparse 16-line simulated LiDAR.  perception_mode=truth prevents
        # road_analyzer from remapping /lidar/road_boundary_markers → /road_boundary_markers.
        # The perception group's own 2D rviz is disabled to avoid clashing with the main rviz2.
        # Requires a gpu_lidar sensor in the Gazebo world SDF publishing /lidar/points.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory("lidar3d_bringup")) / "launch" / "lidar_sim.launch.py")),
            launch_arguments={"use_rviz": "false", "perception_mode": "lidar"}.items(),
            condition=IfCondition(use_perception),
        ),
    ])
