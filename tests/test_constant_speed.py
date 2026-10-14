import copy
import math
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/surface_printing'))
from surface_printing.constant_speed import retime_constant,verify_constant
from surface_printing.tcp_speed import TCPModel

XML='<robot name="xyz"><link name="base"/><link name="x"/><link name="y"/><link name="tip"/>'+''.join(
    f'<joint name="{n}" type="prismatic"><parent link="{a}"/><child link="{b}"/><axis xyz="{axis}"/></joint>'
    for n,a,b,axis in [('jx','base','x','1 0 0'),('jy','x','y','0 1 0'),('jz','y','tip','0 0 1')])+'</robot>'
LIMITS={n:(.03,.03) for n in ('jx','jy','jz')}

def trajectory(points):
    return S(joint_trajectory=S(joint_names=['jx','jy','jz'],points=[
        S(positions=list(q),velocities=[0.]*3,accelerations=[0.]*3,effort=[],
          time_from_start=S(sec=i,nanosec=0)) for i,q in enumerate(points)]))


class ConstantSpeedTests(unittest.TestCase):
    def test_straight_line_has_constant_interior_and_rest_ends(self):
        points=[(x,0,0) for x in np.linspace(0,.03,301)]
        t=trajectory(points)
        r=retime_constant(t,XML,'tip',1.2,1.,LIMITS)
        self.assertGreater(r['coverage_fraction'],.98)
        self.assertLessEqual(r['after_peak_mm_s'],1.2)
        self.assertEqual(t.joint_trajectory.points[0].velocities,[0.,0.,0.])
        self.assertEqual(t.joint_trajectory.points[-1].velocities,[0.,0.,0.])
        self.assertTrue(np.allclose([p.positions for p in t.joint_trajectory.points],points))
        self.assertAlmostEqual(r['overall_average_mm_s'],1.,delta=.05)

    def test_helix_preserves_geometry_and_meets_coverage(self):
        theta=np.linspace(0,6*math.pi,301)
        t=trajectory([(.005*math.cos(a),.005*math.sin(a),.003*a/(6*math.pi)) for a in theta])
        before=[p.positions[:] for p in t.joint_trajectory.points]
        r=retime_constant(t,XML,'tip',1.1,1.,LIMITS)
        self.assertGreaterEqual(r['coverage_fraction'],.90)
        self.assertEqual(before,[p.positions for p in t.joint_trajectory.points])
        self.assertLessEqual(r['after_peak_mm_s'],1.1)

    def test_equal_max_and_target(self):
        t=trajectory([(x,0,0) for x in np.linspace(0,.03,301)])
        r=retime_constant(t,XML,'tip',1.,1.,LIMITS)
        self.assertGreaterEqual(r['coverage_fraction'],.90)
        self.assertLessEqual(r['after_peak_mm_s'],1.)

    def test_short_corner_heavy_path_rejected(self):
        points=[(x,0,0) for x in np.linspace(0,.0002,3)]
        points += [(.0002,y,0) for y in np.linspace(0,.0002,3)[1:]]
        t=trajectory(points)
        with self.assertRaisesRegex(ValueError,'coverage'):
            retime_constant(t,XML,'tip',1.,1.,{n:(.001,.00001) for n in LIMITS})

    def test_joint_limit_infeasible_rejected(self):
        t=trajectory([(x,0,0) for x in np.linspace(0,.03,301)])
        with self.assertRaisesRegex(ValueError,'coverage'):
            retime_constant(t,XML,'tip',1.2,1.,{n:(.0005,.03) for n in LIMITS})

    def test_bad_targets_and_tampered_timing(self):
        for maximum,target in [(1,2),(0,1),(1,float('nan'))]:
            with self.assertRaises(ValueError):
                retime_constant(trajectory([(0,0,0),(.01,0,0),(.02,0,0)]),XML,'tip',
                                maximum,target,LIMITS)
        t=trajectory([(x,0,0) for x in np.linspace(0,.03,301)])
        retime_constant(t,XML,'tip',1.2,1,LIMITS)
        for p in t.joint_trajectory.points:
            p.time_from_start.sec*=2;p.time_from_start.nanosec*=2
            if p.time_from_start.nanosec>=1000000000:
                p.time_from_start.sec+=1;p.time_from_start.nanosec-=1000000000
            p.velocities=[v/2 for v in p.velocities]
        with self.assertRaisesRegex(ValueError,'coverage'):
            verify_constant(t,XML,'tip',1.2,1,limits=LIMITS)

    def test_revolute_chain_and_vectorized_speed(self):
        xml='<robot name="r"><link name="base"/><link name="arm"/><link name="tip"/>'+ \
            '<joint name="j" type="revolute"><parent link="base"/><child link="arm"/>'+ \
            '<axis xyz="0 0 1"/></joint><joint name="o" type="fixed">'+ \
            '<parent link="arm"/><child link="tip"/><origin xyz=".5 0 0"/></joint></robot>'
        model=TCPModel(xml,'tip',['j'])
        q=np.array([[0.],[.1],[.2]]);dq=np.array([[.01],[.02],[.03]])
        self.assertTrue(np.allclose(model.speeds(q,dq),
                                   [model.speed(a,b) for a,b in zip(q,dq)]))
        points=[S(positions=[float(x)],velocities=[0.],accelerations=[],effort=[],
                  time_from_start=S(sec=i,nanosec=0))
                for i,x in enumerate(np.linspace(0,.06,301))]
        t=S(joint_trajectory=S(joint_names=['j'],points=points))
        r=retime_constant(t,xml,'tip',1.2,1.,{'j':(.03,.03)})
        self.assertGreater(r['coverage_fraction'],.98)

    def test_nonuniform_samples(self):
        x=np.concatenate((np.linspace(0,.01,201),np.linspace(.0102,.03,100)))
        r=retime_constant(trajectory([(v,0,0) for v in x]),XML,'tip',1.2,1.,LIMITS)
        self.assertGreaterEqual(r['coverage_fraction'],.90)

    def test_fk_position(self):
        self.assertTrue(np.allclose(TCPModel(XML,'tip',['jx','jy','jz']).position([.1,.2,.3]),
                                    [.1,.2,.3]))


if __name__=='__main__':unittest.main()
