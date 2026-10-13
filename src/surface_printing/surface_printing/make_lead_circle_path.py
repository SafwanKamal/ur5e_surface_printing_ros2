"""Capture the current nozzle pose and generate a lead-in plus circle CSV.

This command generates a path only; it never moves the robot or starts extrusion.
"""
import csv
from pathlib import Path
import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from .common import WorkNode
from .lead_circle_geometry import lead_circle_offsets


def run(node):
    lead = float(node.parameter('lead_in_mm', 10.0))
    inner = float(node.parameter('inner_diameter_mm', 2.0))
    bead = float(node.parameter('bead_width_mm', 0.5))
    spacing = float(node.parameter('spacing_mm', 0.1))
    direction = float(node.parameter('direction', -1.0))
    passes = node.parameter('passes', 1)
    frame = node.parameter('frame_id', 'world')
    link = node.parameter('link_name', 'probe_tcp')
    output = Path(node.parameter('output_path', '/tmp/line_circle.csv'))
    if not frame or not link:
        raise ValueError('frame_id and link_name must be nonempty')
    offsets = lead_circle_offsets(lead, inner, bead, spacing, direction, passes)
    client = node.create_client(GetPositionFK, '/compute_fk')
    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError('MoveIt /compute_fk unavailable')
    request = GetPositionFK.Request()
    request.header.frame_id = frame
    request.fk_link_names = [link]
    request.robot_state = RobotState()
    request.robot_state.joint_state = node.fresh_joints()
    request.robot_state.is_diff = True
    result = node.wait(client.call_async(request))
    if result.error_code.val != 1 or len(result.pose_stamped) != 1:
        raise RuntimeError('FK failed; check frame and calibrated nozzle link')
    if result.pose_stamped[0].header.frame_id != frame:
        raise RuntimeError('FK returned an unexpected frame')
    pose = result.pose_stamped[0].pose
    base = [getattr(pose.position, axis) for axis in 'xyz']
    quaternion = [getattr(pose.orientation, axis) for axis in 'xyzw']
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['line_id', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        for x, y in offsets:
            writer.writerow([0, base[0] + x, base[1] + y, base[2], *quaternion])
    node.get_logger().info(
        f'PATH ONLY: current nozzle start={base}; lead-in={lead:g} mm; '
        f'target inner diameter={inner:g} mm; bead width={bead:g} mm; '
        f'path diameter={inner + bead:g} mm; {passes} circle pass(es); '
        f'{len(offsets)} points; saved {output}. Fixed height/orientation; '
        'inner diameter is approximate and depends on actual deposition.')


def main(args=None):
    rclpy.init(args=args)
    node = WorkNode('make_lead_circle_path')
    try:
        run(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
