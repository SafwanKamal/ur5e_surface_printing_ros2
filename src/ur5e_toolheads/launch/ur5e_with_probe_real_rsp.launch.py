from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    standard_rsp_launch = PathJoinSubstitution(
        [
            FindPackageShare("ur_robot_driver"),
            "launch",
            "ur_rsp.launch.py",
        ]
    )

    custom_description_file = PathJoinSubstitution(
        [
            FindPackageShare("ur5e_toolheads"),
            "urdf",
            "ur5e_with_probe_real.urdf.xacro",
        ]
    )

    custom_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(standard_rsp_launch),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "reverse_ip": LaunchConfiguration("reverse_ip"),
            "description_file": custom_description_file,
        }.items(),
    )

    return LaunchDescription([custom_rsp])