from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    reverse_ip = LaunchConfiguration("reverse_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")

    robot_ip_argument = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.1.102",
        description="IP address of the physical UR5e",
    )

    reverse_ip_argument = DeclareLaunchArgument(
        "reverse_ip",
        default_value="192.168.1.100",
        description="IP address of the Ubuntu ROS computer",
    )

    launch_rviz_argument = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Start the custom MoveIt RViz configuration",
    )

    driver_launch = PathJoinSubstitution(
        [
            FindPackageShare("ur_robot_driver"),
            "launch",
            "ur_control.launch.py",
        ]
    )

    custom_description_launch = PathJoinSubstitution(
        [
            FindPackageShare("ur5e_toolheads"),
            "launch",
            "ur5e_with_probe_real_rsp.launch.py",
        ]
    )

    physical_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(driver_launch),
        launch_arguments={
            "ur_type": "ur5e",
            "robot_ip": robot_ip,
            "reverse_ip": reverse_ip,

            "description_launchfile": custom_description_launch,

            "use_mock_hardware": "false",
            "headless_mode": "false",

            "initial_joint_controller":
                "scaled_joint_trajectory_controller",
            "activate_joint_controller": "true",

            # Use the custom MoveIt RViz instance below.
            "launch_rviz": "false",
        }.items(),
    )

    move_group_launch = PathJoinSubstitution(
        [
            FindPackageShare("ur5e_probe_moveit_config"),
            "launch",
            "move_group_real.launch.py",
        ]
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch)
    )

    moveit_rviz_launch = PathJoinSubstitution(
        [
            FindPackageShare("ur5e_probe_moveit_config"),
            "launch",
            "moveit_rviz.launch.py",
        ]
    )

    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_rviz_launch),
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(
        [
            robot_ip_argument,
            reverse_ip_argument,
            launch_rviz_argument,
            physical_driver,
            move_group,
            moveit_rviz,
        ]
    )