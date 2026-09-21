"""Exercise production geometry and monitor logic without a ROS installation.

AST loading removes ROS imports/constructors only; tested methods are unchanged.
This is not a ROS transport or hardware integration test.
"""
import ast
import math
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / 'src/surface_printing/surface_printing'
sys.path.insert(0, str(SRC.parent))
from surface_printing.demo_geometry import demo_offsets, validate_budget
from surface_printing.convert_surface_csv import multiply


def extract(filename, names, namespace):
    tree = ast.parse((SRC / filename).read_text())
    tree.body = [n for n in tree.body if isinstance(n, (ast.ClassDef, ast.FunctionDef)) and n.name in names]
    for n in tree.body:
        if isinstance(n, ast.ClassDef):
            n.bases = []
            n.body = [f for f in n.body if not isinstance(f, ast.FunctionDef) or f.name != '__init__']
    exec(compile(tree, str(SRC / filename), 'exec'), namespace)
    return namespace


monitor = extract('execute_surface_print.py', ['PrintNode'],
                  {'math': math, 'time': time, 'Empty': lambda: None})['PrintNode']
validate = extract('common.py', ['validate_trajectory'], {'math': math})['validate_trajectory']


class DemoSafety(unittest.TestCase):
    def node(self):
        n = monitor()
        now = time.monotonic()
        n.cleaning = False; n.error = ''; n.monitoring = True; n.extruding = False
        n.deadline = now + 30; n.joints_received = now; n.simulation = False
        n.scale_received = now; n.scale = n.baseline_scale = .5
        n.joint_names = ['a']; n.joints = NS(name=['a'], position=[0.01])
        n.motion_reference = [0.]; n.motion_seen = False; n.last_motion = now
        n.flow_started = now; n.last_health = now; n.health = 'MOVING'
        n.max_volume = 1.; n.effective_flow = 1.; n.last_ping = 0.
        n.keepalive = Mock()
        return n

    def test_measured_motion(self):
        n = self.node(); n.tick(); self.assertTrue(n.motion_seen)

    def test_stale_joints(self):
        n = self.node(); n.joints_received -= 1
        with self.assertRaisesRegex(RuntimeError, 'joint feedback stale'): n.tick()

    def test_stale_scaling(self):
        n = self.node(); n.scale_received -= 1
        with self.assertRaisesRegex(RuntimeError, 'speed-scaling feedback stale'): n.tick()

    def test_paused_and_changed_scaling(self):
        for scale in [0, float('nan'), .25, 1.01]:
            n = self.node(); n.scale = scale
            with self.assertRaises(RuntimeError): n.tick()

    def test_missing_and_invalid_joint(self):
        for names, positions in [([], []), (['a'], [float('nan')])]:
            n = self.node(); n.joints = NS(name=names, position=positions)
            with self.assertRaises(RuntimeError): n.tick()

    def test_stall(self):
        n = self.node(); n.motion_reference = [.01]; n.last_motion -= 2
        with self.assertRaisesRegex(RuntimeError, 'No measured'): n.tick()

    def test_time_limit(self):
        n = self.node(); n.deadline = 0
        with self.assertRaisesRegex(RuntimeError, 'Maximum execution'): n.tick()

    def test_volume_limit(self):
        n = self.node(); n.extruding = True; n.flow_started -= 10; n.max_volume = .01
        with self.assertRaisesRegex(RuntimeError, 'volume budget'): n.tick()

    def test_dead_syringe(self):
        n = self.node(); n.extruding = True; n.flow_started -= 2; n.health = 'IDLE'
        with self.assertRaisesRegex(RuntimeError, 'not moving'): n.tick()

    def test_stale_syringe(self):
        n = self.node(); n.extruding = True; n.flow_started -= 2; n.last_health -= 2
        with self.assertRaisesRegex(RuntimeError, 'Syringe status stale'): n.tick()

    def test_fault_and_keepalive(self):
        n = self.node(); n.extruding = True; n.tick()
        n.keepalive.publish.assert_called_once()
        n.status(NS(data='FAULT WATCHDOG'))
        with self.assertRaisesRegex(RuntimeError, 'WATCHDOG'): n.tick()

    def test_late_acceptance_is_cancelled(self):
        n = self.node(); n.abort_pending_goal = True
        handle = Mock(accepted=True); future = Mock(); future.result.return_value = handle
        n.late_goal(future); handle.cancel_goal_async.assert_called_once()

    def test_shapes(self):
        for shape in ['line', 'circle', 'square']:
            p = demo_offsets(shape, 30)
            self.assertEqual(p[0], (0., 0.))
            self.assertLessEqual(max(math.dist(a,b) for a,b in zip(p,p[1:])), .00100001)
            if shape != 'line': self.assertLess(math.dist(p[0], p[-1]), 1e-10)

    def test_budget_rejects_invalid_and_excessive(self):
        validate_budget(20, 1, 60, 1)
        for args in [(20, float('nan'),60,1), (70,1,60,2), (60,5,60,1), (10,6,60,2)]:
            with self.assertRaises(ValueError): validate_budget(*args)

    def test_quaternion_composition(self):
        q = (0, 0, math.sin(math.pi/4), math.cos(math.pi/4))
        rotated = multiply(multiply(q, (1,0,0,0)), (0,0,-q[2],q[3]))
        self.assertAlmostEqual(rotated[0], 0); self.assertAlmostEqual(rotated[1], 1)

    def test_trajectory_rejections(self):
        def point(x, t):
            return NS(positions=[x], velocities=[], accelerations=[], effort=[],
                      time_from_start=NS(sec=t, nanosec=0))
        a, b = point(0,0), point(.1,1)
        trajectory = NS(joint_trajectory=NS(joint_names=['a'], points=[a,b]),
                        multi_dof_joint_trajectory=NS(points=[]))
        self.assertEqual(validate(trajectory), 1)
        for x,t in [(float('nan'),1),(.4,1),(.1,0)]:
            trajectory.joint_trajectory.points[1] = point(x,t)
            with self.assertRaises(ValueError): validate(trajectory)


if __name__ == '__main__': unittest.main()
