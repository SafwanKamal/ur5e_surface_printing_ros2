from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    robot_ip=LaunchConfiguration('robot_ip')
    reverse_ip=LaunchConfiguration('reverse_ip')
    calibration=LaunchConfiguration('kinematics_params_file')
    description=str(Path(get_package_share_directory('ur5e_toolheads')) /
                    'urdf' / 'ur5e_with_probe_real.urdf.xacro')
    moveit_config=(
        MoveItConfigsBuilder('ur5e_with_probe_mock',package_name='ur5e_probe_moveit_config')
        .robot_description(file_path=description,mappings={
            'robot_ip':robot_ip,
            'reverse_ip':reverse_ip,
            'kinematics_params':calibration,
            'use_mock_hardware':'false',
        })
        .robot_description_semantic(file_path='config/ur5e_with_probe_mock.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers_real.yaml')
        .to_moveit_configs())

    # The physical UR scaled controller stretches wall-clock execution when the
    # pendant speed slider is below 100%.  MoveIt's default 1.2 multiplier can
    # therefore abort a valid slow trajectory long before the controller is
    # finished.  The surface-printing executor still applies its independent,
    # speed-aware deadline and speed-change watchdog.
    moveit_config.trajectory_execution[
        'trajectory_execution.allowed_execution_duration_scaling'
    ]=10.0

    generated=generate_move_group_launch(moveit_config)
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip'),
        DeclareLaunchArgument('reverse_ip',default_value='0.0.0.0'),
        DeclareLaunchArgument('kinematics_params_file'),
        *generated.entities,
    ])
