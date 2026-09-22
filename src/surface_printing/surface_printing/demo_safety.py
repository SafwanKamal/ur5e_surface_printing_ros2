"""Read the live MoveIt model and validate saved trajectories against its scene."""
import hashlib
import math
import xml.etree.ElementTree as ET
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.srv import GetParameters
from controller_manager_msgs.srv import ListControllers
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
    expected_scale=[float(v) for v in config['mesh_scale']]
    if len(expected_scale)!=3 or any(not math.isfinite(v) or v<=0 for v in expected_scale):
        raise RuntimeError('Invalid configured pump mesh scale')
    checks=[("./joint[@name='probe_tool_to_tcp']/origin",'tcp'),
            ("./joint[@name='flange_to_probe_tool']/origin",'mount')]
    for query, prefix in checks:
        origin=root.find(query)
        if origin is None: raise RuntimeError('Missing tool transform')
        for key in ('xyz','rpy'):
            actual=[float(v) for v in origin.get(key,'0 0 0').split()]
            expected=config[prefix+'_'+key]
            if len(actual)!=3 or len(expected)!=3 or any(
                not math.isfinite(a) or not math.isfinite(float(b)) or abs(a-b)>1e-7 for a,b in zip(actual,expected)):
                raise RuntimeError('Live tool transform differs from configuration; restart all launches')

    needle_length=float(config.get('needle_length_mm',0.0))
    needle_radius=float(config.get('needle_radius_mm',1.0))
    needle_tip_clearance=float(config.get('needle_collision_tip_clearance_mm',2.0))
    if (not all(math.isfinite(v) for v in (needle_length,needle_radius,needle_tip_clearance)) or
            needle_length<0 or needle_length>200 or needle_radius<=0 or
            needle_tip_clearance<0 or needle_tip_clearance>=needle_length and needle_length>0):
        raise RuntimeError('Invalid configured needle collision geometry')
    envelope_names={
        'pump_sleeve_envelope','pump_body_envelope','pump_tip_stem_envelope',
        'pump_tip_collar_envelope','pump_nozzle_envelope','pump_outlet_envelope'}
    if needle_length>0: envelope_names.add('pump_needle_envelope')
    live_envelopes={link.get('name') for link in root.findall('./link')
                    if (link.get('name') or '').startswith('pump_') and
                    (link.get('name') or '').endswith('_envelope')}
    if live_envelopes:
        if config.get('coarse_collision') is not True:
            raise RuntimeError('Live coarse collision model is not enabled in printing_tool.yaml')
        if live_envelopes != envelope_names:
            raise RuntimeError('Live coarse collision model is incomplete or unexpected')
        if root.find("./link[@name='probe_tool_link']/collision") is not None:
            raise RuntimeError('Live model contains both full-mesh and coarse tool collisions')
        specs={
            'pump_sleeve_envelope':('cylinder',(0,1079.4,205.4),(54.0,62.0)),
            'pump_body_envelope':('box',(89.625,1081.41265,173.7),(71.25,43.0251,203.6)),
            'pump_tip_stem_envelope':('box',(87.65,1083.25,293.6),(3.9,12.3,34.6)),
            'pump_tip_collar_envelope':('cylinder',(96.74983,1081.4002,313.05),(11.0,4.5)),
            'pump_nozzle_envelope':('cylinder',(96.74983,1081.4002,319.25),(5.0,8.5)),
            'pump_outlet_envelope':('cylinder',(96.74983,1081.4002,324.55),(2.2,3.0)),
        }
        if needle_length>0:
            collision_length=(needle_length-needle_tip_clearance)/1000.0
            outlet=config['syringe_outlet_cad_mm']
            specs['pump_needle_envelope']=(
                'cylinder',
                (float(outlet[0]),float(outlet[1]),
                 float(outlet[2])+collision_length/(2.0*expected_scale[2])),
                ((needle_radius/1000.0)/max(expected_scale[:2]),
                 collision_length/expected_scale[2]))
        for name,(kind,cad_xyz,dims) in specs.items():
            collision=root.find(f"./link[@name='{name}']/collision")
            if collision is None: raise RuntimeError(f'Missing collision for {name}')
            _check_values(collision.find('origin'),'xyz',
                          [cad_xyz[i]*expected_scale[i] for i in range(3)],1e-7)
            geometry=collision.find(f'geometry/{kind}')
            if geometry is None: raise RuntimeError(f'Wrong collision primitive for {name}')
            if kind=='box':
                _check_values(geometry,'size',[dims[i]*expected_scale[i] for i in range(3)],1e-7)
            else:
                _check_values(geometry,'radius',[dims[0]*max(expected_scale[:2])],1e-7)
                _check_values(geometry,'length',[dims[1]*expected_scale[2]],1e-7)
        joint_children=set()
        for joint in root.findall('./joint'):
            child=joint.find('child')
            if child is not None and child.get('link') in envelope_names:
                joint_children.add(child.get('link'))
                if joint.get('type')!='fixed': raise RuntimeError('Tool envelope joint is not fixed')
                parent=joint.find('parent')
                if parent is None or parent.get('link')!='probe_tool_link':
                    raise RuntimeError('Tool envelope is not fixed to probe_tool_link')
                _check_values(joint.find('origin'),'xyz',config['mesh_xyz'],1e-7)
                _check_values(joint.find('origin'),'rpy',config['mesh_rpy'],1e-7)
        if joint_children != envelope_names:
            raise RuntimeError('Live coarse collision joints are incomplete')
        return

    mesh=root.find("./link[@name='probe_tool_link']/collision/geometry/mesh")
    if mesh is None or not mesh.get('filename','').endswith('/Full_UR_Syringe_Pump.stl'):
        raise RuntimeError('Live MoveIt model contains neither the verified mesh nor coarse pump collision model')
    _check_values(mesh,'scale',expected_scale,1e-9)
    collision=root.find("./link[@name='probe_tool_link']/collision")
    _check_values(collision.find('origin') if collision is not None else None,
                  'xyz',config['mesh_xyz'],1e-7)
    _check_values(collision.find('origin') if collision is not None else None,
                  'rpy',config['mesh_rpy'],1e-7)


def _check_values(element, attribute, expected, tolerance):
    if element is None: raise RuntimeError(f'Missing tool collision {attribute}')
    actual=[float(v) for v in element.get(attribute,'').split()]
    expected=[float(v) for v in expected]
    if len(actual)!=len(expected) or any(
        not math.isfinite(a) or not math.isfinite(b) or abs(a-b)>tolerance
        for a,b in zip(actual,expected)):
        raise RuntimeError(f'Live tool collision {attribute} differs from configuration')


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


def require_real_model(xml):
    plugins=[(p.text or '').strip() for p in ET.fromstring(xml).findall('./ros2_control/hardware/plugin')]
    if not plugins or any(p=='mock_components/GenericSystem' for p in plugins):
        raise RuntimeError('Physical execution requires the live UR driver robot model, not mock hardware')
    if not any('ur_robot_driver/' in p for p in plugins):
        raise RuntimeError('Live robot model does not contain the UR robot driver hardware plugin')


def require_controller_mode(node, *, simulation):
    client=node.create_client(ListControllers,'/controller_manager/list_controllers')
    if not client.wait_for_service(timeout_sec=5):
        raise RuntimeError('Controller manager is unavailable')
    response=node.wait(client.call_async(ListControllers.Request()),5)
    active={controller.name for controller in response.controller if controller.state=='active'}
    mock='ur_manipulator_controller'
    real='scaled_joint_trajectory_controller'
    if simulation:
        if mock not in active or real in active:
            raise RuntimeError('Simulation requires only the active mock trajectory controller')
    elif real not in active or mock in active:
        raise RuntimeError('Physical execution requires only the active scaled UR trajectory controller')
