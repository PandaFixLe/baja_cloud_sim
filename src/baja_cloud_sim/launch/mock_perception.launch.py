"""Standalone integration check: algorithm core fed by mock perception only.

This launch does NOT start Gazebo, the bridge, or truth_perception. It runs
the planning/control core together with `mock_perception_node`, which streams
the perception-group message contract (road boundary + obstacles with
ns="tall"/"flat_ground"). It is used to verify the core receives and parses
the perception stream correctly before the real perception package lands.

Note: with no real odometry the planner/follower have no vehicle state, so
this launch is meant for inspecting topic wiring (ros2 topic echo / RViz),
not for a driving run. Use it alongside the simulation launch once the real
perception package is merged.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("baja_cloud_sim"))
    params = str(share / "config" / "params.yaml")
    rviz = str(share / "config" / "simulation.rviz")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        Node(package="baja_cloud_sim", executable="mock_perception", name="mock_perception_node", parameters=[params], output="screen"),
        Node(package="baja_cloud_sim", executable="frenet_planner", name="frenet_planner_node", parameters=[params], output="screen"),
        Node(package="baja_cloud_sim", executable="path_follower", name="path_follower_node", parameters=[params], output="screen"),
        Node(package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", rviz], parameters=[{"use_sim_time": False}], condition=IfCondition(use_rviz), output="screen"),
    ])
