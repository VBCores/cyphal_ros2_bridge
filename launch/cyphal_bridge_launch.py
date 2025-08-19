import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share_dir = Path(get_package_share_directory("cyphal_ros2_bridge")).resolve()
    examples_dir = share_dir / "examples"

    declare_config_file = DeclareLaunchArgument(
        "config_file",
        default_value=str(examples_dir / "test_node.json"),
        description="Path to Cyphal-ROS2 bridge config file",
    )

    cyphal_node = Node(
        package="cyphal_ros2_bridge",
        executable="cyphal_bridge",
        name="cyphal_bridge",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("config_file")}],
    )

    return LaunchDescription([declare_config_file, cyphal_node])
