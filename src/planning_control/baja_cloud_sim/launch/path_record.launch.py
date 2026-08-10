"""Path recording launch: record GPS+IMU waypoints to CSV for later planning.

Starts only the hardware localization + path_recorder. No control, no planning.
Drive the vehicle manually (RC or teleop) along the desired racing line; the
recorder saves waypoints at regular intervals for csv_to_centerline_node.

Usage:
  ros2 launch baja_cloud_sim path_record.launch.py output_file:=/path/to/recorded_path.csv
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output_file = LaunchConfiguration("output_file")
    auto_start = LaunchConfiguration("auto_start")

    hw_remappings = [
        ("/chcnav/devpvt", "/gps/fix"),
        ("/imu_yaw", "/imu/yaw"),
    ]

    return LaunchDescription([
        DeclareLaunchArgument("output_file", default_value="recorded_path.csv",
                              description="Output CSV file path"),
        DeclareLaunchArgument("auto_start", default_value="false",
                              description="Auto-start recording when GPS is ready (skip Enter prompt)"),

        # ── Hardware: CHCNAV ──
        Node(
            package="chcnav",
            executable="chcnav_full_node",
            name="chcnav_full_node",
            output="screen",
        ),

        # ── Path recorder ──
        Node(
            package="baja_cloud_sim",
            executable="path_recorder",
            name="path_recorder",
            parameters=[{
                "output_file": output_file,
                "min_point_distance": 0.5,
                "auto_start": auto_start,
            }],
            remappings=hw_remappings,
            output="screen",
        ),
    ])
