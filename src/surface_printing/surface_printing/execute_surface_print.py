"""Bounded demonstration with motion monitoring and leased constant flow."""
import math
import signal
import time
from pathlib import Path
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotTrajectory
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.set_message import set_message_fields
from std_msgs.msg import Empty, Float64
from std_srvs.srv import Trigger
from syringe_interfaces.srv import SetFlow
from std_msgs.msg import String
from .common import WorkNode, validate_trajectory
from .demo_geometry import validate_budget
from .tcp_speed import verify as verify_tcp_speed
from .wrist_guard import load_bounds, check_state, check_trajectory
from .demo_safety import live_model, calibrated_tool, check_scene, require_mock_model


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
        self.keepalive=self.create_publisher(Empty,'/syringe/flow_keepalive',10)
        self.status_sub=self.create_subscription(String,'/syringe/status',self.status,10)
        self.scale_sub=self.create_subscription(Float64,
            '/speed_scaling_state_broadcaster/speed_scaling',self.scaling,qos_profile_sensor_data)
        self.flow=self.create_client(SetFlow,'/syringe/set_flow')
        self.stop=self.create_client(Trigger,'/syringe/stop')
        self.abort_srv=self.create_service(Trigger,'/surface_printing/stop',self.abort)
        self.robot=ActionClient(self,ExecuteTrajectory,'/execute_trajectory')

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
                if not math.isfinite(self.scale) or self.scale<=0 or abs(self.scale-self.baseline_scale)>2.:
                    raise RuntimeError('Robot paused/stopped or speed scaling changed; aborting print')
            current=dict(zip(self.joints.name,self.joints.position))
            if any(n not in current or not math.isfinite(current[n]) for n in self.joint_names):
                raise RuntimeError('Invalid robot feedback')
            if hasattr(self, 'wrist_bounds'): check_state(self.joints, self.wrist_bounds)
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
            if (now-self.flow_started)*self.effective_flow/60 >= self.max_volume:
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
        # Dispatch both immediately; do not wait on USB before requesting robot cancel.
        cancel=None; stop=None
        try:
            if self.robot_goal is not None and self.robot_goal.accepted:
                cancel=self.robot_goal.cancel_goal_async()
        except Exception as e: self.get_logger().error(f'Robot cancel request failed: {e}')
        try:
            if self.stop.service_is_ready(): stop=self.stop.call_async(Trigger.Request())
        except Exception as e: self.get_logger().error(f'Syringe stop request failed: {e}')
        for future,label in ((stop,'Syringe STOP'),(cancel,'Robot cancel')):
            if future is None: continue
            try:
                response=self.wait(future,3)
                if label=='Syringe STOP' and not response.success:
                    raise RuntimeError(response.message)
                if label=='Robot cancel' and not response.goals_canceling:
                    self.get_logger().warning('Cancel not accepted; robot may already have finished')
            except Exception as e: self.get_logger().error(f'{label} unconfirmed: {e}; use physical stop')
        # Allow a delayed goal acceptance callback to cancel before shutting down.
        if self.goal_future is not None and not self.goal_future.done():
            try: self.wait(self.goal_future,3)
            except Exception: self.get_logger().error('Goal acceptance unknown; use physical robot stop')
        if self.result_future is not None and not self.result_future.done():
            try: self.wait(self.result_future,3)
            except Exception: self.get_logger().error('Robot termination unconfirmed; use physical robot stop')


def start_matches(node, trajectory, tolerance):
    msg=node.fresh_joints(); current=dict(zip(msg.name,msg.position))
    for name,p in zip(trajectory.joint_trajectory.joint_names,
                      trajectory.joint_trajectory.points[0].positions):
        if name not in current or not math.isfinite(current[name]) or abs(current[name]-p)>tolerance:
            raise RuntimeError(f'Robot is not at saved start ({name}); replan from current pose')


def run(node):
    path=node.parameter('trajectory_path','/tmp/surface_line_plan.yaml')
    execute=bool(node.parameter('execute',False)); extrude=bool(node.parameter('extrude',False))
    node.simulation=bool(node.parameter('simulation',False))
    confirmed=bool(node.parameter('hardware_confirmed',False))
    flow=float(node.parameter('flow_ml_per_min',1.0))
    tolerance=float(node.parameter('start_tolerance_rad',0.01))
    max_seconds=float(node.parameter('max_duration_sec',60.0))
    node.max_volume=float(node.parameter('max_volume_ml',1.0))
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
    verify_tcp_speed(trajectory, xml, data['link_name'],
                     data['cartesian_speed_mm_s'], node.wrist_bounds)
    if node.simulation: require_mock_model(xml)
    else: calibrated_tool(xml)
    validate_budget(duration,flow,max_seconds,node.max_volume)
    check_scene(node,trajectory,data.get('group_name','ur_manipulator'))
    start_matches(node,trajectory,tolerance)
    node.get_logger().info(f'Validated {duration:.2f}s plan, flow={flow:g} mL/min; '
                           f'execute={execute}, extrude={extrude}')
    if not execute:
        node.get_logger().info('CHECK ONLY: no robot or pump motion requested'); return
    if not node.simulation and not confirmed:
        raise RuntimeError('Set hardware_confirmed:=true only after completing the demo checklist')
    if not node.robot.wait_for_server(timeout_sec=5): raise RuntimeError('MoveIt execution unavailable')
    if not node.simulation:
        deadline=time.monotonic()+3
        while node.scale is None or time.monotonic()-node.scale_received>.5:
            if time.monotonic()>deadline: raise RuntimeError('No fresh UR speed scaling (percent)')
            rclpy.spin_once(node,timeout_sec=.02)
        if not math.isfinite(node.scale) or not 1<=node.scale<=100:
            raise RuntimeError('UR speed scaling must be 1..100 percent')
        node.baseline_scale=node.scale
    actual_estimate=duration/(1. if node.simulation else node.baseline_scale/100.)
    # Reserve stop latency in the volume budget (lease + firmware timeout).
    validate_budget(actual_estimate+3.,flow,max_seconds,node.max_volume)
    if not node.simulation:
        # Even motion-only execution establishes that the pump is stopped.
        if not node.stop.wait_for_service(timeout_sec=5): raise RuntimeError('Syringe STOP unavailable')
        stopped=node.wait(node.stop.call_async(Trigger.Request()),3)
        if not stopped.success: raise RuntimeError('Could not establish stopped pump')
        if extrude and not node.flow.wait_for_service(timeout_sec=5):
            raise RuntimeError('Syringe flow service unavailable')
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
        req=SetFlow.Request(); req.enabled=True; req.flow_ml_per_min=flow; req.retract=False
        # Include service latency in the estimated extrusion volume.
        flow_requested=time.monotonic()
        response=node.wait(node.flow.call_async(req),3)
        if not response.success: raise RuntimeError(response.message)
        node.effective_flow=response.effective_flow_ml_per_min
        node.extruding=True; node.flow_started=flow_requested; node.last_ping=0
        validate_budget(actual_estimate+3.,node.effective_flow,max_seconds,node.max_volume)
        node.tick()
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

