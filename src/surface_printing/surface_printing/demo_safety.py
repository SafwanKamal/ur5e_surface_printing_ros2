"""Read the live MoveIt model and validate saved trajectories against its scene."""
import hashlib
import math
import xml.etree.ElementTree as ET
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import GetParameters
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity


def live_model(node):
    client=node.create_client(GetParameters, '/move_group/get_parameters')
    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError('MoveIt parameter service unavailable')
    req=GetParameters.Request()
    req.names=['robot_description','robot_description_semantic']
    response=node.wait(client.call_async(req))
    if len(response.values)!=2 or any(not v.string_value for v in response.values):
        raise RuntimeError('Cannot read live MoveIt robot model')
    xml=response.values[0].string_value
    digest=hashlib.sha256('\n'.join(v.string_value for v in response.values).encode()).hexdigest()
    return xml, digest


def calibrated_tool(xml):
    path=Path(get_package_share_directory('ur5e_toolheads'))/'config/printing_tool.yaml'
    config=yaml.safe_load(path.read_text())
    if config.get('enabled') is not True or config.get('calibrated') is not True:
        raise RuntimeError('Enable and calibrate printing_tool.yaml, rebuild, then restart robot/MoveIt')
    root=ET.fromstring(xml)
    if root.find("./link[@name='pump_sleeve_envelope']") is not None:
        raise RuntimeError('Coarse collision envelopes are simulation-only; restore verified geometry for physical execution')
    mesh=root.find("./link[@name='probe_tool_link']/collision/geometry/mesh")
    if mesh is None or not mesh.get('filename','').endswith('/Full_UR_Syringe_Pump.stl'):
        raise RuntimeError('Live MoveIt model does not contain the full syringe pump collision mesh')
    actual_scale=[float(v) for v in mesh.get('scale','1 1 1').split()]
    expected_scale=config['mesh_scale']
    if len(actual_scale)!=3 or len(expected_scale)!=3 or any(
        not math.isfinite(a) or not math.isfinite(b) or b<=0 or abs(a-b)>1e-9
        for a,b in zip(actual_scale,expected_scale)):
        raise RuntimeError('Live pump mesh scale differs from calibration')
    checks=[("./joint[@name='probe_tool_to_tcp']/origin",'tcp'),
            ("./joint[@name='flange_to_probe_tool']/origin",'mount'),
            ("./link[@name='probe_tool_link']/collision/origin",'mesh')]
    for query, prefix in checks:
        origin=root.find(query)
        if origin is None: raise RuntimeError('Missing tool transform')
        for key in ('xyz','rpy'):
            actual=[float(v) for v in origin.get(key,'0 0 0').split()]
            expected=config[prefix+'_'+key]
            if len(actual)!=3 or len(expected)!=3 or any(
                not math.isfinite(a) or not math.isfinite(float(b)) or abs(a-b)>1e-7 for a,b in zip(actual,expected)):
                raise RuntimeError('Live tool transform differs from configuration; restart all launches')


def check_scene(node, trajectory, group):
    client=node.create_client(GetStateValidity, '/check_state_validity')
    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError('MoveIt collision validation service unavailable')
    jt=trajectory.joint_trajectory
    for index, point in enumerate(jt.points):
        req=GetStateValidity.Request(); req.group_name=group
        req.robot_state=RobotState(); req.robot_state.is_diff=True
        req.robot_state.joint_state.name=jt.joint_names
        req.robot_state.joint_state.position=point.positions
        if not node.wait(client.call_async(req), 3).valid:
            raise RuntimeError(f'Current planning scene rejects trajectory point {index}')


def require_mock_model(xml):
    plugins=[p.text for p in ET.fromstring(xml).findall('./ros2_control/hardware/plugin')]
    if not plugins or any(p != 'mock_components/GenericSystem' for p in plugins):
        raise RuntimeError('simulation:=true requires a live mock_components/GenericSystem model')
