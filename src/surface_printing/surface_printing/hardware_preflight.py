"""Read-only physical UR preflight. This node never commands robot motion."""
import time

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

from .common import WorkNode, speed_scaling_fraction
from .demo_safety import (calibrated_tool, live_model,
                          require_controller_mode, require_real_model)
from .wrist_guard import check_state, load_bounds


class PreflightNode(WorkNode):
    def __init__(self):
        super().__init__('hardware_preflight')
        self.scale=None
        self.scale_received=0.0
        self.create_subscription(Float64,
            '/speed_scaling_state_broadcaster/speed_scaling',self._scale,
            qos_profile_sensor_data)

    def _scale(self,message):
        self.scale=message.data
        self.scale_received=time.monotonic()


def run(node):
    xml,_=live_model(node)
    require_real_model(xml)
    calibrated_tool(xml)
    require_controller_mode(node,simulation=False)
    state=node.fresh_joints()
    check_state(state,load_bounds(simulation=False))
    client=node.create_client(GetStateValidity,'/check_state_validity')
    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError('MoveIt collision validation service unavailable')
    request=GetStateValidity.Request()
    request.group_name='ur_manipulator'
    request.robot_state=RobotState()
    request.robot_state.is_diff=True
    request.robot_state.joint_state=state
    result=node.wait(client.call_async(request),5)
    if not result.valid:
        pairs=', '.join(f'{c.contact_body_1}/{c.contact_body_2}' for c in result.contacts)
        raise RuntimeError('Current physical start state is in collision: '+(pairs or 'contact details unavailable'))
    deadline=time.monotonic()+5.0
    while node.scale is None or time.monotonic()-node.scale_received>0.5:
        if time.monotonic()>deadline:
            raise RuntimeError('No fresh UR speed-scaling feedback')
        rclpy.spin_once(node,timeout_sec=0.05)
    scale_fraction=speed_scaling_fraction(node.scale)
    node.get_logger().info(
        f'PREFLIGHT PASSED: real UR model, calibrated tool, real controller, '
        f'fresh joints, collision-free state, speed scaling={scale_fraction*100:.1f}%')
    node.get_logger().info('No robot or pump motion was commanded')


def main(args=None):
    rclpy.init(args=args)
    node=PreflightNode()
    try:
        run(node)
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
