from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "ur5e_with_probe_mock",
            package_name="ur5e_probe_moveit_config",
        )
        .robot_description(
            file_path="config/ur5e_with_probe_mock.urdf.xacro"
        )
        .robot_description_semantic(
            file_path="config/ur5e_with_probe_mock.srdf"
        )
        .trajectory_execution(
            file_path="config/moveit_controllers_real.yaml"
        )
        .to_moveit_configs()
    )

    return generate_move_group_launch(moveit_config)