"""Verify the startup pose's nozzle direction using nominal Jazzy UR5e FK.

Joint origins: UniversalRobots/Universal_Robots_ROS2_Description, jazzy,
config/ur5e/default_kinematics.yaml. Base/flange transforms: urdf/ur_macro.xacro.
This is an orientation/height regression test, not a collision or physical test.
"""
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
         'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    t = np.eye(4)
    t[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                 [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                 [-sp, cp*sr, cp*cr]]
    t[:3, 3] = xyz
    return t


def nominal_fk(joints, tool):
    half = math.pi/2
    # Upstream rounded roll values agree with these nominal angles to 1e-9.
    origins = [((0, 0, .1625), (0, 0, 0)), ((0, 0, 0), (half, 0, 0)),
               ((-.425, 0, 0), (0, 0, 0)), ((-.3922, 0, .1333), (0, 0, 0)),
               ((0, -.0997, 0), (half, 0, 0)), ((0, .0996, 0), (half, math.pi, math.pi))]
    result = transform(rpy=(0, 0, math.pi))
    for name, (xyz, rpy) in zip(NAMES, origins):
        result = result @ transform(xyz, rpy) @ transform(rpy=(0, 0, joints[name]))
    result = result @ transform(rpy=(0, -half, -half))  # wrist_3 -> flange
    return result @ transform(tool['mount_xyz'], tool['mount_rpy']) @ transform(tool['tcp_xyz'], tool['tcp_rpy'])


class MockStartPose(unittest.TestCase):
    def setUp(self):
        self.joints = yaml.safe_load((ROOT/'src/ur5e_probe_moveit_config/config/initial_positions.yaml').read_text())['initial_positions']
        self.tool = yaml.safe_load((ROOT/'src/ur5e_toolheads/config/printing_tool.yaml').read_text())

    def test_committed_tool_points_down_with_positive_tip_clearance(self):
        tcp = nominal_fk(self.joints, self.tool)
        np.testing.assert_allclose(tcp[:3, 2], [0, 0, -1], atol=1e-8)
        self.assertGreater(tcp[2, 3], .15)
        self.assertLess(tcp[2, 3], .30)
        self.assertGreater(math.hypot(*tcp[:2, 3]), .4)

    def test_sideways_zero_pose_is_not_the_startup_pose(self):
        sideways = nominal_fk(dict.fromkeys(NAMES, 0.0), self.tool)
        self.assertLess(abs(sideways[2, 2]), 1e-8)
        self.assertEqual(self.joints['wrist_3_joint'], 0.0)

    def test_alternate_mock_entry_point_uses_the_same_joints(self):
        xml = ET.parse(ROOT/'src/ur5e_toolheads/urdf/mock_ros2_control.xacro')
        for joint in xml.findall('.//joint'):
            initial = joint.find("state_interface[@name='position']/param[@name='initial_value']")
            self.assertAlmostEqual(float(initial.text), self.joints[joint.attrib['name']], places=12)
