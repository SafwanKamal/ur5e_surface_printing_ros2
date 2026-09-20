import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("syringe_controller"),
        "config",
        "syringe.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=config,
            ),
            Node(
                package="syringe_controller",
                executable="syringe_serial_node",
                name="syringe_serial_node",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
