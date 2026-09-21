import copy
import math
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/surface_printing'))
from surface_printing.tcp_speed import peak_speed, retime, verify

XML = '''<robot name="test"><link name="base"/><link name="arm"/><link name="tip"/>
<joint name="j" type="revolute"><parent link="base"/><child link="arm"/><axis xyz="0 0 1"/></joint>
<joint name="offset" type="fixed"><parent link="arm"/><child link="tip"/><origin xyz="1 0 0"/></joint></robot>'''

def trajectory():
    def point(q, sec):
        return S(positions=[q], velocities=[0.], accelerations=[0.],
                 time_from_start=S(sec=sec, nanosec=0))
    return S(joint_trajectory=S(joint_names=['j'], points=[point(0,0),point(.01,1)]))

class TCPSpeedTests(unittest.TestCase):
    def test_interior_speed_and_consistent_retiming(self):
        t=trajectory(); before=copy.deepcopy(t)
        self.assertAlmostEqual(peak_speed(t,XML,'tip')[0],.01875)
        with self.assertRaises(ValueError): verify(t,XML,'tip',2)
        report=retime(t,XML,'tip',2)
        verify(t,XML,'tip',2)
        self.assertAlmostEqual(report['after_peak_mm_s'],1.9,places=6)
        self.assertEqual(t.joint_trajectory.points[1].positions,before.joint_trajectory.points[1].positions)
        self.assertGreater(t.joint_trajectory.points[1].time_from_start.sec,1)

    def test_invalid_limits_and_times(self):
        for limit in [0,-1,float('nan'),float('inf')]:
            with self.assertRaises(ValueError): retime(trajectory(),XML,'tip',limit)
        t=trajectory();t.joint_trajectory.points[1].time_from_start.sec=0
        with self.assertRaises(ValueError): peak_speed(t,XML,'tip')

    def test_missing_joint_and_nonfinite(self):
        t=trajectory();t.joint_trajectory.joint_names=['other']
        with self.assertRaises(ValueError): peak_speed(t,XML,'tip')
        t=trajectory();t.joint_trajectory.points[1].positions=[float('nan')]
        with self.assertRaises(ValueError): peak_speed(t,XML,'tip')

    def test_no_speedup(self):
        t=trajectory();t.joint_trajectory.points[1].time_from_start.sec=100
        self.assertEqual(retime(t,XML,'tip',2)['time_scale'],1)
