"""Bounded demonstration with leased frequency or calibrated-flow extrusion."""
import math
import signal
import time
from pathlib import Path
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from controller_manager_msgs.srv import SwitchController, ListControllers
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotTrajectory
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.set_message import set_message_fields
from std_msgs.msg import Empty, Float64
from std_srvs.srv import Trigger
from syringe_interfaces.srv import SetFlow, SetExtrusion
from std_msgs.msg import String
from .common import WorkNode, validate_trajectory, speed_scaling_fraction
from .demo_geometry import validate_budget, validate_pulse_budget
from .tcp_speed import verify as verify_tcp_speed, TCPModel
from .constant_speed import verify_constant
from .cartesian_timing import configured_limits
from .speed_tracking import SpeedTracker
from .wrist_guard import load_bounds, check_state, check_trajectory
from .demo_safety import (live_model, calibrated_tool, check_scene,
                          require_controller_mode, require_mock_model,
                          require_real_model)


class PrintNode(WorkNode):
    def __init__(self):
        super().__init__('execute_surface_print')
        self.extruding=False; self.monitoring=False; self.cleaning=False
        self.error=''; self.health=''; self.last_health=0.; self.last_ping=0.
        self.started=0.; self.flow_started=0.; self.last_motion=0.
        self.motion_reference=None; self.motion_seen=False; self.joint_names=[]
        self.scale=None; self.scale_received=0.; self.baseline_scale=None
        self.simulation=False; self.robot_goal=None; self.goal_future=None
        self.abort_pending_goal=False; self.deadline=math.inf; self.max_volume=1.
        self.effective_flow=0.; self.result_future=None
        self.frequency_mode=False; self.step_rate=0.; self.max_pulses=0
        self.extrusion_direction=-1
        self.speed_tracker=None
        self.last_speed_log=0.
        self.keepalive=self.create_publisher(Empty,'/syringe/flow_keepalive',10)
        self.status_sub=self.create_subscription(String,'/syringe/status',self.status,10)
        self.scale_sub=self.create_subscription(Float64,
            '/speed_scaling_state_broadcaster/speed_scaling',self.scaling,qos_profile_sensor_data)
        self.flow=self.create_client(SetFlow,'/syringe/set_flow')
        self.frequency=self.create_client(SetExtrusion,'/syringe/set_extrusion')
        self.stop=self.create_client(Trigger,'/syringe/stop')
        self.abort_srv=self.create_service(Trigger,'/surface_printing/stop',self.abort)
        self.robot=ActionClient(self,ExecuteTrajectory,'/execute_trajectory')
        self.controller_cancel=self.create_client(
            CancelGoal,'/scaled_joint_trajectory_controller/follow_joint_trajectory/_action/cancel_goal')
        self.controller_switch=self.create_client(
            SwitchController,'/controller_manager/switch_controller')
        self.controller_list=self.create_client(
            ListControllers,'/controller_manager/list_controllers')

    def abort(self, request, response):
        self.error='Operator requested stop'
        response.success=True; response.message='Abort requested; inspect completion and physical motion'
        return response

    def scaling(self, msg):
        self.scale=msg.data; self.scale_received=time.monotonic()

    def status(self, msg):
        if msg.data.startswith('STATE '):
            self.health=msg.data[6:]; self.last_health=time.monotonic()
        if self.extruding and msg.data.split()[:1] in (['FAULT'],['ERROR'],['LEASE_EXPIRED']):
            self.error=msg.data

    def tick(self):
        if self.cleaning: return
        if self.error: raise RuntimeError(self.error)
        now=time.monotonic()
        if self.monitoring:
            if now>self.deadline: raise RuntimeError('Maximum execution time reached')
            if now-self.joints_received>.5: raise RuntimeError('Robot joint feedback stale')
            if not self.simulation:
                if now-self.scale_received>.5: raise RuntimeError('Robot speed-scaling feedback stale')
                current_scale=speed_scaling_fraction(self.scale)
                if abs(current_scale-self.baseline_scale)>0.02:
                    raise RuntimeError('Robot paused/stopped or speed scaling changed; aborting print')
            current=dict(zip(self.joints.name,self.joints.position))
            if any(n not in current or not math.isfinite(current[n]) for n in self.joint_names):
                raise RuntimeError('Invalid robot feedback')
            if hasattr(self, 'wrist_bounds'): check_state(self.joints, self.wrist_bounds)
            if getattr(self, 'speed_tracker', None) is not None:
                self.speed_tracker.update(self.joints)
                if self.motion_seen and now-getattr(self,'last_speed_log',0.)>=5.:
                    self.last_speed_log=now
                    measured=self.speed_tracker.report()
                    self.get_logger().info(
                        f"Measured tip {self.speed_tracker.last_speed:.3f} mm/s; "
                        f"target-band distance so far {100*measured['measured_coverage']:.1f}%")
            positions=[current[n] for n in self.joint_names]
            if self.motion_reference is None: self.motion_reference=positions
            if max(abs(a-b) for a,b in zip(positions,self.motion_reference))>0.00005:
                self.motion_seen=True; self.last_motion=now; self.motion_reference=positions
            if now-self.last_motion>1.0:
                raise RuntimeError('No measured robot progress for 1 second')
        if self.extruding:
            if now-self.flow_started>0.75 and self.health!='MOVING':
                raise RuntimeError('Syringe is not moving: '+self.health)
            if now-max(self.last_health,self.flow_started)>1.0:
                raise RuntimeError('Syringe status stale')
            if self.frequency_mode and self.max_pulses > 0 and (now-self.flow_started)*self.step_rate >= self.max_pulses:
                raise RuntimeError('Extrusion pulse budget reached')
            if not self.frequency_mode and (now-self.flow_started)*self.effective_flow/60 >= self.max_volume:
                raise RuntimeError('Extrusion volume budget reached')
            if now-self.last_ping>=.2:
                self.keepalive.publish(Empty()); self.last_ping=now

    def late_goal(self, future):
        if self.abort_pending_goal:
            handle=future.result()
            if handle is not None and handle.accepted: handle.cancel_goal_async()

    def cleanup(self):
        self.cleaning=True; self.extruding=False; self.monitoring=False
        self.abort_pending_goal=True
        motion_possible=self.goal_future is not None or self.robot_goal is not None
        pending=[]
        # Send independent stop requests before waiting on any one service.
        if motion_possible:
            try:
                if self.robot_goal is not None and self.robot_goal.accepted:
                    pending.append((self.robot_goal.cancel_goal_async(),'MoveIt cancel'))
            except Exception as e:
                self.get_logger().error(f'MoveIt cancel dispatch failed: {e}')
            if not self.simulation:
                try:
                    if self.controller_cancel.service_is_ready():
                        pending.append((self.controller_cancel.call_async(CancelGoal.Request()),
                                        'Controller goal cancel'))
                except Exception as e:
                    self.get_logger().error(f'Controller cancel dispatch failed: {e}')
            try:
                if self.controller_switch.service_is_ready():
                    request=SwitchController.Request()
                    request.deactivate_controllers=[
                        'ur_manipulator_controller' if self.simulation
                        else 'scaled_joint_trajectory_controller']
                    request.strictness=2
                    request.timeout.sec=3
                    pending.append((self.controller_switch.call_async(request),
                                    'Controller deactivate'))
            except Exception as e:
                self.get_logger().error(f'Controller stop dispatch failed: {e}')
        try:
            if self.stop.service_is_ready():
                pending.append((self.stop.call_async(Trigger.Request()),'Syringe STOP'))
        except Exception as e:
            self.get_logger().error(f'Syringe stop dispatch failed: {e}')
        for future,label in pending:
            try:
                response=self.wait(future,3)
                if label=='Syringe STOP' and not response.success:
                    raise RuntimeError(response.message)
                if label=='Controller deactivate' and not response.ok:
                    self.get_logger().warning('Deactivation rejected; checking actual controller state')
            except Exception as e:
                self.get_logger().error(f'{label} unconfirmed: {e}')
        if motion_possible:
            controller='ur_manipulator_controller' if self.simulation else 'scaled_joint_trajectory_controller'
            try:
                if not self.controller_list.service_is_ready():
                    raise RuntimeError('Controller status service unavailable')
                states=self.wait(self.controller_list.call_async(ListControllers.Request()),3)
                if any(c.name==controller and c.state=='active' for c in states.controller):
                    raise RuntimeError('Motion controller is still active')
                self.stationary_joints(timeout=3.0)
                self.get_logger().info(
                    'ABORT STOP VERIFIED: motion controller inactive and measured joints stationary. '
                    'Regenerate/replan; reactivate controller after checking robot state.')
            except Exception as e:
                self.get_logger().error(
                    f'ROBOT STOP NOT VERIFIED: {e}. Use the physical pendant stop; '
                    'do not regenerate or execute another path while the robot is moving.')
        if self.goal_future is not None and not self.goal_future.done():
            try: self.wait(self.goal_future,3)
            except Exception: self.get_logger().error('Goal acceptance unknown; use physical robot stop')


def start_matches(node, trajectory, tolerance):
    msg=node.stationary_joints(); current=dict(zip(msg.name,msg.position))
    for name,p in zip(trajectory.joint_trajectory.joint_names,
                      trajectory.joint_trajectory.points[0].positions):
        if name not in current or not math.isfinite(current[name]) or abs(current[name]-p)>tolerance:
            raise RuntimeError(f'Robot is not at saved start ({name}); replan from current pose')


def extrusion_request(node, flow):
    if node.frequency_mode:
        req=SetExtrusion.Request()
        req.enabled=True; req.direction=node.extrusion_direction; req.step_rate_hz=node.step_rate
        return node.frequency, req
    req=SetFlow.Request(); req.enabled=True; req.flow_ml_per_min=flow; req.retract=False
    return node.flow, req


def run(node):
    path=node.parameter('trajectory_path','/tmp/surface_line_plan.yaml')
    execute=bool(node.parameter('execute',False)); extrude=bool(node.parameter('extrude',False))
    node.simulation=bool(node.parameter('simulation',False))
    confirmed=bool(node.parameter('hardware_confirmed',False))
    node.step_rate=float(node.parameter('step_rate_hz',200.0))
    node.frequency_mode=node.step_rate != 0.0
    node.max_pulses=node.parameter('max_pulses',0)
    node.extrusion_direction=node.parameter('extrusion_direction',-1)
    if node.extrusion_direction not in (-1,1):
        raise ValueError('extrusion_direction must be -1 or +1')
    flow=None if node.frequency_mode else float(node.parameter('flow_ml_per_min',1.0))
    tolerance=float(node.parameter('start_tolerance_rad',0.01))
    max_seconds=float(node.parameter('max_duration_sec',60.0))
    node.max_volume=math.inf if node.frequency_mode else float(node.parameter('max_volume_ml',1.0))
    def budget(seconds, effective_flow=flow):
        if node.frequency_mode:
            validate_pulse_budget(seconds,node.step_rate,max_seconds,node.max_pulses)
        else:
            validate_budget(seconds,effective_flow,max_seconds,node.max_volume)
    if not math.isfinite(tolerance) or not 0<tolerance<=.03:
        raise ValueError('start_tolerance_rad must be (0,0.03]')
    if node.simulation and extrude: raise ValueError('Simulation forbids extrusion')
    data=yaml.safe_load(Path(path).read_text())
    if not isinstance(data,dict) or data.get('schema')!='surface_print_plan_v2':
        raise ValueError('Create a fresh v2 plan with plan_surface_path')
    age=time.time()-float(data['created_unix_sec'])
    if not math.isfinite(age) or not 0<=age<=300:
        raise ValueError('Plan is older than 5 minutes or has an invalid timestamp; replan')
    trajectory=RobotTrajectory(); set_message_fields(trajectory,data['trajectory'])
    duration=validate_trajectory(trajectory)
    node.wrist_bounds=load_bounds(simulation=node.simulation)
    check_state(node.fresh_joints(), node.wrist_bounds)
    check_trajectory(trajectory, node.wrist_bounds)
    xml,model_hash=live_model(node)
    if data.get('link_name') not in ('probe_tcp','nozzle_tcp'):
        raise ValueError('Demo execution requires the calibrated nozzle link')
    if model_hash!=data.get('model_sha256'): raise RuntimeError('Robot model changed; replan')
    node.speed_tracker=None
    if 'average_cartesian_speed_mm_s' in data:
        limits=configured_limits(xml,trajectory.joint_trajectory.joint_names,
                                 data['velocity_scale'],data['acceleration_scale'])
        limits={name:(min(v,.3),a) for name,(v,a) in limits.items()}
        speed_report=verify_constant(trajectory,xml,data['link_name'],
                                     data['max_cartesian_speed_mm_s'],
                                     data['average_cartesian_speed_mm_s'],
                                     node.wrist_bounds,limits)
        node.get_logger().info(
            f"Verified target-speed coverage: {100*speed_report['coverage_fraction']:.2f}% "
            f"of path within +/-5% of {data['average_cartesian_speed_mm_s']:.3f} mm/s")
        node.speed_tracker=SpeedTracker(
            TCPModel(xml,data['link_name'],list(trajectory.joint_trajectory.joint_names)),
            trajectory.joint_trajectory.joint_names,data['max_cartesian_speed_mm_s'],
            data['average_cartesian_speed_mm_s'],speed_report['path_length_mm'])
    else:
        verify_tcp_speed(trajectory, xml, data['link_name'],
                         data['cartesian_speed_mm_s'], node.wrist_bounds)
    if node.simulation:
        require_mock_model(xml)
    else:
        require_real_model(xml)
        calibrated_tool(xml)
    require_controller_mode(node,simulation=node.simulation)
    budget(duration)
    check_scene(node,trajectory,data.get('group_name','ur_manipulator'))
    start_matches(node,trajectory,tolerance)
    setting=(f'frequency={node.step_rate:g} Hz, pulse budget={node.max_pulses}; volume uncalibrated'
             if node.frequency_mode else f'flow={flow:g} mL/min')
    node.get_logger().info(f'Validated {duration:.2f}s plan, {setting}; '
                           f'execute={execute}, extrude={extrude}')
    if not execute:
        node.get_logger().info('CHECK ONLY: no robot or pump motion requested'); return
    if not node.simulation and not confirmed:
        raise RuntimeError('Set hardware_confirmed:=true only after completing the demo checklist')
    if not node.robot.wait_for_server(timeout_sec=5): raise RuntimeError('MoveIt execution unavailable')
    if not node.simulation:
        deadline=time.monotonic()+3
        while node.scale is None or time.monotonic()-node.scale_received>.5:
            if time.monotonic()>deadline: raise RuntimeError('No fresh UR speed scaling factor')
            rclpy.spin_once(node,timeout_sec=.02)
        node.baseline_scale=speed_scaling_fraction(node.scale)
        if node.speed_tracker is not None and abs(node.baseline_scale-1.)>.01:
            raise RuntimeError('Target-speed execution requires effective UR speed scaling '
                               'at 100%; current factor='+str(node.baseline_scale))
    actual_estimate=duration/(1. if node.simulation else node.baseline_scale)
    # Reserve stop latency in the pulse/volume budget (lease + firmware timeout).
    budget(actual_estimate+3.)
    if not node.simulation:
        # Even motion-only execution establishes that the pump is stopped.
        if not node.stop.wait_for_service(timeout_sec=5): raise RuntimeError('Syringe STOP unavailable')
        stopped=node.wait(node.stop.call_async(Trigger.Request()),3)
        if not stopped.success: raise RuntimeError('Could not establish stopped pump')
        if 'fault' in getattr(stopped, 'message', '').lower():
            raise RuntimeError('Syringe is faulted before robot motion: '+stopped.message)
        if extrude:
            extrusion_client,_=extrusion_request(node,flow)
            if not extrusion_client.wait_for_service(timeout_sec=5):
                raise RuntimeError('Syringe extrusion service unavailable')
    start_matches(node,trajectory,tolerance)
    node.joint_names=list(trajectory.joint_trajectory.joint_names)
    node.started=node.last_motion=time.monotonic()
    node.deadline=node.started+min(max_seconds,actual_estimate+5.)
    node.monitoring=True
    goal=ExecuteTrajectory.Goal(); goal.trajectory=trajectory
    node.goal_future=node.robot.send_goal_async(goal); node.goal_future.add_done_callback(node.late_goal)
    node.robot_goal=node.wait(node.goal_future,5)
    if not node.robot_goal.accepted: raise RuntimeError('MoveIt rejected trajectory')
    node.result_future=node.robot_goal.get_result_async()
    if extrude:
        # Action acceptance alone does not prove movement. Await measured progress.
        while not node.motion_seen:
            rclpy.spin_once(node,timeout_sec=.02); node.tick()
            if node.result_future.done(): raise RuntimeError('Robot ended before extrusion could start')
        extrusion_client,req=extrusion_request(node,flow)
        # Include service latency in the conservative extrusion budget.
        flow_requested=time.monotonic()
        response=node.wait(extrusion_client.call_async(req),3)
        if not response.success: raise RuntimeError(response.message)
        if not node.frequency_mode:
            node.effective_flow=response.effective_flow_ml_per_min
        node.extruding=True; node.flow_started=flow_requested; node.last_ping=0
        budget(actual_estimate+3.,node.effective_flow)
        node.tick()
        if node.frequency_mode:
            node.get_logger().info(f'Extruding at target {node.step_rate:g} Hz, direction '
                                   f'{node.extrusion_direction}; firmware ramps; volume uncalibrated')
        else:
            node.get_logger().info(f'Extruding at {node.effective_flow:.4f} mL/min '
                                   '(pulse estimate; no material-flow sensor)')
    response=node.wait(node.result_future,max_seconds)
    node.extruding=False; node.monitoring=False
    if not node.simulation:
        stopped=node.wait(node.stop.call_async(Trigger.Request()),3)
        if not stopped.success: raise RuntimeError('Syringe STOP unconfirmed')
    if response.status!=GoalStatus.STATUS_SUCCEEDED or response.result.error_code.val!=1:
        raise RuntimeError(f'Robot failed: status={response.status}, code={response.result.error_code.val}')
    node.robot_goal=None
    if node.speed_tracker is not None:
        measured=node.speed_tracker.report()
        node.get_logger().info(
            f"Measured TCP speed: average {measured['measured_average_mm_s']:.3f} mm/s; "
            f"peak {measured['measured_peak_mm_s']:.3f} mm/s; "
            f"target-band distance {100*measured['measured_coverage']:.2f}%; "
            f"{measured['measurement_windows']} measurement windows")
        if measured['measurement_windows']==0 or measured['measured_coverage']<.90:
            raise RuntimeError('Execution finished but measured target-speed coverage was below 90%')
    node.get_logger().info('Demo completed; pump stopped. No automatic retract or travel.')


def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum,frame): raise KeyboardInterrupt()
    signal.signal(signal.SIGINT,interrupt); signal.signal(signal.SIGTERM,interrupt)
    node=PrintNode(); code=0
    try: run(node)
    except (Exception,KeyboardInterrupt) as e:
        code=1; node.get_logger().error(f'Demo aborted: {e}')
        # Ignore further terminal interrupts during bounded cleanup.
        signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,signal.SIG_IGN)
        node.cleanup()
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    if code: raise SystemExit(code)

