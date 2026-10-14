"""Abort tests reproduce an unresponsive MoveIt cancellation service."""
import ast
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock
ROOT=Path(__file__).resolve().parents[1]
SRC=ROOT/'src/surface_printing/surface_printing'


def extract(filename, classname, namespace):
    tree=ast.parse((SRC/filename).read_text())
    tree.body=[n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==classname]
    tree.body[0].bases=[]
    tree.body[0].body=[n for n in tree.body[0].body
                       if not isinstance(n,ast.FunctionDef) or n.name!='__init__']
    exec(compile(tree,str(SRC/filename),'exec'),namespace)
    return namespace[classname]


PrintNode=extract('execute_surface_print.py','PrintNode',dict(
    time=time,math=math,CancelGoal=S(Request=S),
    SwitchController=S(Request=lambda:S(timeout=S(sec=0))),
    ListControllers=S(Request=S),Trigger=S(Request=S)))


class AbortStopTests(unittest.TestCase):
    def node(self,moving=True):
        n=PrintNode();n.simulation=False;n.logger=Mock();n.get_logger=lambda:n.logger
        n.goal_future=Mock() if moving else None
        n.robot_goal=S(accepted=True,cancel_goal_async=lambda:'moveit') if moving else None
        n.controller_cancel=Mock();n.controller_switch=Mock();n.controller_list=Mock();n.stop=Mock()
        n.controller_cancel.call_async.return_value='direct'
        n.controller_switch.call_async.return_value='deactivate'
        n.controller_list.call_async.return_value='list'
        n.stop.call_async.return_value='syringe'
        n.stationary_joints=Mock()
        n.events=[]
        def wait(future,timeout):
            n.events.append(future)
            if future=='moveit':raise TimeoutError('MoveIt request timed out')
            if future=='deactivate':return S(ok=True)
            if future=='list':return S(controller=[S(name='scaled_joint_trajectory_controller',state='inactive')])
            if future=='syringe':return S(success=True)
            return S()
        n.wait=wait
        return n

    def test_moveit_cancel_timeout_does_not_prevent_controller_stop(self):
        n=self.node();n.cleanup()
        self.assertTrue(n.abort_pending_goal)
        n.controller_cancel.call_async.assert_called_once()
        request=n.controller_switch.call_async.call_args.args[0]
        self.assertEqual(request.deactivate_controllers,['scaled_joint_trajectory_controller'])
        self.assertEqual(request.strictness,2)
        n.stop.call_async.assert_called_once()
        n.stationary_joints.assert_called_once()
        self.assertTrue(any('ABORT STOP VERIFIED' in str(c)
                            for c in n.logger.info.call_args_list))

    def test_motion_not_stationary_remains_unconfirmed(self):
        n=self.node();n.stationary_joints.side_effect=RuntimeError('still moving')
        n.cleanup()
        self.assertTrue(any('ROBOT STOP NOT VERIFIED' in str(c)
                            for c in n.logger.error.call_args_list))
        self.assertFalse(any('ABORT STOP VERIFIED' in str(c)
                             for c in n.logger.info.call_args_list))

    def test_no_motion_submitted_does_not_deactivate_controller(self):
        n=self.node(moving=False);n.cleanup()
        n.controller_switch.call_async.assert_not_called()
        n.controller_cancel.call_async.assert_not_called()
        n.stop.call_async.assert_called_once()


class StationaryCaptureTests(unittest.TestCase):
    def node(self,rate=0.):
        clock=S(t=0.)
        def spin(node,timeout_sec):clock.t+=timeout_sec
        cls=extract('common.py','WorkNode',dict(
            math=math,time=S(monotonic=lambda:clock.t),
            rclpy=S(spin_once=spin)))
        node=cls()
        def fresh():
            total=100+clock.t
            return S(name=['joint'],position=[rate*clock.t],
                     header=S(stamp=S(sec=int(total),nanosec=int((total-int(total))*1e9))))
        node.fresh_joints=fresh
        return node,clock

    def test_stationary_feedback_passes_after_observation(self):
        n,c=self.node();n.stationary_joints(timeout=1)
        self.assertGreaterEqual(c.t,.5)

    def test_moving_robot_capture_rejected(self):
        n,c=self.node(rate=.001)
        with self.assertRaisesRegex(RuntimeError,'still moving'):
            n.stationary_joints(timeout=.8)

    def test_duplicate_feedback_does_not_prove_stationary(self):
        n,c=self.node()
        msg=n.fresh_joints();n.fresh_joints=lambda:msg
        with self.assertRaisesRegex(RuntimeError,'unconfirmed'):
            n.stationary_joints(timeout=.8)


if __name__=='__main__':unittest.main()

