"""Launch the Baja simulator with UDP remote control instead of path following."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    listen_address = LaunchConfiguration("listen_address")
    listen_port = LaunchConfiguration("listen_port")

    params = str(share / "config" / "params.yaml")
    bridge = str(share / "config" / "bridge.yaml")
    rviz = str(share / "config" / "simulation.rviz")
    robot_description = (share / "urdf" / "baja_vehicle.urdf").read_text(
        encoding="utf-8"
    )

    return LaunchDescription([
        DeclareLaunchArgument("world_file"),
        DeclareLaunchArgument("scenario_file"),
        DeclareLaunchArgument("results_dir", default_value="results/remote"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_gz_gui", default_value="true"),
        DeclareLaunchArgument("use_video", default_value="false"),
        DeclareLaunchArgument("listen_address", default_value="127.0.0.1"),
        DeclareLaunchArgument("listen_port", default_value="5005"),
        DeclareLaunchArgument(
            "video_path", default_value="results/remote/gazebo.mp4"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(ros_gz_share / "launch" / "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": ["-r -v 3 --render-engine-gui ogre ", world]
            }.items(),
            condition=IfCondition(use_gz_gui),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(ros_gz_share / "launch" / "gz_sim.launch.py")
            ),
            launch_arguments={"gz_args": ["-r -s -v 3 ", world]}.items(),
            condition=UnlessCondition(use_gz_gui),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            parameters=[{"config_file": bridge, "use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": True,
            }],
            output="screen",
        ),
        Node(
            package="baja_cloud_sim",
            executable="truth_perception",
            name="truth_perception_node",
            parameters=[params, {"scenario_file": scenario}],
            output="screen",
        ),
        Node(
            package="baja_cloud_sim",
            executable="remote_control",
            name="remote_control_node",
            parameters=[params, {
                "listen_address": listen_address,
                "listen_port": ParameterValue(listen_port, value_type=int),
            }],
            output="screen",
        ),
        Node(
            package="baja_cloud_sim",
            executable="actuator_adapter",
            name="actuator_adapter_node",
            parameters=[params],
            output="screen",
        ),
        Node(
            package="baja_cloud_sim",
            executable="evaluator",
            name="evaluator_node",
            parameters=[params, {
                "scenario_file": scenario,
                "results_dir": results,
            }],
            output="screen",
        ),
        Node(
            package="baja_cloud_sim",
            executable="video_recorder",
            name="video_recorder_node",
            parameters=[{"video_path": video_path, "use_sim_time": True}],
            condition=IfCondition(use_video),
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(use_rviz),
            output="screen",
        ),
    ])
