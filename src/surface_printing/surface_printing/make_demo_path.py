"""Capture current calibrated nozzle pose and write a small planar demo CSV."""
import csv
from pathlib import Path
import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from .common import WorkNode
from .demo_geometry import demo_offsets


def run(node):
    shape = node.parameter('shape', 'line')
    size = float(node.parameter('size_mm', 30.0))
    plane = node.parameter('plane', 'xy')
    frame = node.parameter('frame_id', 'world')
    link = node.parameter('link_name', 'probe_tcp')
    output = Path(node.parameter('output_path', '/tmp/demo_path.csv'))
    sign = float(node.parameter('direction', 1.0))
    if plane not in ('xy', 'xz', 'yz') or sign not in (-1., 1.):
        raise ValueError('plane must be xy/xz/yz; direction must be +/-1')
    offsets = demo_offsets(shape, size)
    fk = node.create_client(GetPositionFK, '/compute_fk')
    if not fk.wait_for_service(timeout_sec=5):
        raise RuntimeError('MoveIt /compute_fk unavailable')
    req = GetPositionFK.Request()
    req.header.frame_id = frame
    req.fk_link_names = [link]
    req.robot_state = RobotState()
    req.robot_state.joint_state = node.fresh_joints()
    req.robot_state.is_diff = True
    result = node.wait(fk.call_async(req))
    if result.error_code.val != 1 or len(result.pose_stamped) != 1:
        raise RuntimeError('FK failed; check frame and calibrated nozzle link')
    pose = result.pose_stamped[0].pose
    base = [getattr(pose.position, a) for a in 'xyz']
    q = [getattr(pose.orientation, a) for a in 'xyzw']
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['line_id', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        for u, v in offsets:
            p = base.copy()
            p['xyz'.index(plane[0])] += sign*u
            p['xyz'.index(plane[1])] += sign*v
            writer.writerow([0, *p, *q])
    node.get_logger().info(f'PATH ONLY: {shape}, {size:g} mm in {frame} {plane}; {output}. '
                           'This captures the present pose; it does not locate a surface.')


def main(args=None):
    rclpy.init(args=args)
    node = WorkNode('make_demo_path')
    try:
        run(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
