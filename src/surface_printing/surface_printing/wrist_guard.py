"""Shared Wrist 3 bounds; collision checking handles tool/arm clearance."""
import math
from pathlib import Path


def load_bounds(*, simulation=True):
    del simulation  # The same finite, collision-checked range is used on hardware.
    import yaml
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory('ur5e_probe_moveit_config'))/'config/joint_limits.yaml'
    entry = yaml.safe_load(path.read_text())['joint_limits']['wrist_3_joint']
    if entry.get('has_position_limits') is not True:
        raise ValueError('Wrist 3 position limits must be enabled')
    low, high = float(entry['min_position']), float(entry['max_position'])
    if not all(math.isfinite(x) for x in (low, high)) or low >= high or low < -math.pi or high > math.pi:
        raise ValueError('Demo Wrist 3 range must be finite, ordered and within +/-180 degrees')
    return low, high


def check_value(value, bounds):
    low, high = bounds
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'Wrist 3 angle {value} rad is outside [{low}, {high}]. Reposition in simulation; do not re-center limits on an arbitrary pose.')


def check_state(state, bounds):
    if len(state.name) != len(state.position) or state.name.count('wrist_3_joint') != 1:
        raise ValueError('Missing/ambiguous Wrist 3 feedback')
    check_value(state.position[state.name.index('wrist_3_joint')], bounds)


def check_trajectory(trajectory, bounds):
    jt = trajectory.joint_trajectory
    if jt.joint_names.count('wrist_3_joint') != 1:
        raise ValueError('Trajectory must contain Wrist 3 exactly once')
    i = jt.joint_names.index('wrist_3_joint')
    for point in jt.points:
        if len(point.positions) != len(jt.joint_names):
            raise ValueError('Invalid trajectory positions')
        check_value(point.positions[i], bounds)
