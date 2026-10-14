"""Capture the manually placed tip and create a fixed-orientation tube CSV.

This command never moves the robot or starts the pump.
"""
import csv
import math
from pathlib import Path
import rclpy
from moveit_msgs.srv import GetPositionFK
from .common import WorkNode
from .demo_safety import validate_tube_tool, live_model
from .tube_geometry import tube_points, normalized_orientation


def run(node):
    diameter = float(node.parameter('diameter_mm', 10.0))
    height = float(node.parameter('height_mm', 0.0))
    pitch = float(node.parameter('pitch_mm', 0.5))
    step = float(node.parameter('step_mm', 0.25))
    output = Path(node.parameter('output_path', '/tmp/tube_10mm.csv'))
    simulation = bool(node.parameter('simulation', False))
    # Validate sizing before querying any live state.
    tube_points((0., 0., 0.), diameter, height, pitch, step)
    xml, _ = live_model(node)
    validate_tube_tool(xml, simulation=simulation)
    client = node.create_client(GetPositionFK, '/compute_fk')
    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError('MoveIt /compute_fk unavailable')
    request = GetPositionFK.Request()
    request.header.frame_id = 'world'
    request.fk_link_names = ['probe_tcp']
    joints = node.stationary_joints()
    required = {f'{name}_joint' for name in
                ('shoulder_pan','shoulder_lift','elbow','wrist_1','wrist_2','wrist_3')}
    if (not required.issubset(joints.name) or
            not all(math.isfinite(v) for v in joints.position)):
        raise RuntimeError('Missing or invalid arm joints in live state')
    stamp = joints.header.stamp.sec * 1000000000 + joints.header.stamp.nanosec
    age = (node.get_clock().now().nanoseconds - stamp) / 1e9
    if stamp <= 0 or not -0.1 <= age <= 0.5:
        raise RuntimeError('Joint state timestamp is stale or incompatible with the node clock')
    request.robot_state.joint_state = joints
    request.robot_state.is_diff = True
    result = node.wait(client.call_async(request))
    if result.error_code.val != 1 or len(result.pose_stamped) != 1:
        raise RuntimeError('FK failed for probe_tcp in world')
    if result.pose_stamped[0].header.frame_id != 'world':
        raise RuntimeError('FK returned an unexpected coordinate frame')
    pose = result.pose_stamped[0].pose
    start = tuple(getattr(pose.position, a) for a in 'xyz')
    q = normalized_orientation(tuple(getattr(pose.orientation, a) for a in 'xyzw'))
    # TCP +Z points out of the nozzle. A vertical tube requires it downward.
    zz = 1 - 2 * (q[0]**2 + q[1]**2)
    tilt = math.degrees(math.acos(max(-1., min(1., -zz))))
    if tilt > 15:
        raise RuntimeError(f'Nozzle is {tilt:.1f} degrees from downward. '
                           'Manually orient it downward, reposition the tip, and recapture.')
    points = tube_points(start, diameter, height, pitch, step)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['line_id', 'point_index', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        for i, point in enumerate(points):
            writer.writerow([0, i, *point, *q])
    node.get_logger().info(
        f'PATH ONLY: {output}; {len(points)} points; nominal diameter={diameter} mm; '
        f'height={height} mm; pitch={pitch} mm. Start={start}; fixed quaternion={q}. '
        'Tube center lies diameter/2 toward world -X from the captured tip. '
        'CSV coordinates are world meters for plan_surface_path; no object transform. '
        'No robot/pump motion commanded.')


def main(args=None):
    rclpy.init(args=args)
    node = WorkNode('make_tube_path')
    try:
        run(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

