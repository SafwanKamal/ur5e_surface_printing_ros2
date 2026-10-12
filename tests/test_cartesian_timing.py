"""Reversal stitching must preserve motion geometry and strictly increasing time."""
import copy
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/surface_printing'))
from surface_printing.cartesian_timing import (
    strictly_timed, split_legs, join_legs, remove_identical_time_duplicates)
from surface_printing.tcp_speed import retime, verify
from test_tcp_speed import XML


def leg(a,b):
    def point(q,sec):
        return S(positions=[q],velocities=[0.],accelerations=[0.],effort=[],
                 time_from_start=S(sec=sec,nanosec=0))
    return S(joint_trajectory=S(joint_names=['j'],points=[point(a,0),point(b,1)]),
             multi_dof_joint_trajectory=S(points=[]))


class CartesianTiming(unittest.TestCase):
    def test_three_reversals_keep_geometry_and_validate_speed(self):
        parts=[leg(0,.01),leg(.01,0),leg(0,.01)]
        original=copy.deepcopy(parts)
        joined=join_legs(parts)
        self.assertEqual([p.positions[0] for p in joined.joint_trajectory.points], [0,.01,0,.01])
        self.assertEqual([p.time_from_start.sec for p in joined.joint_trajectory.points],[0,1,2,3])
        self.assertTrue(strictly_timed(joined))
        retime(joined,XML,'tip',2)
        verify(joined,XML,'tip',2)
        self.assertEqual(parts[1].joint_trajectory.points[0].time_from_start.sec,
                         original[1].joint_trajectory.points[0].time_from_start.sec)

    def test_sharp_turn_splitting(self):
        poses=[S(position=S(x=x,y=0,z=0)) for x in [0,.005,.01,.005,0,.005,.01]]
        legs=split_legs(poses)
        self.assertEqual(len(legs),3)
        self.assertIs(legs[0][-1],legs[1][0])
        self.assertIs(legs[1][-1],legs[2][0])

    def test_invalid_timing_and_seams_rejected(self):
        bad=leg(0,.01);bad.joint_trajectory.points[1].time_from_start.sec=0
        with self.assertRaises(ValueError): join_legs([bad])
        with self.assertRaises(ValueError): join_legs([leg(0,.01),leg(.02,0)])
        moving=leg(.01,0);moving.joint_trajectory.points[0].velocities=[.1]
        with self.assertRaises(ValueError): join_legs([leg(0,.01),moving])

    def test_only_identical_zero_time_states_are_removed(self):
        t=leg(0,.01)
        t.joint_trajectory.points.insert(1,copy.deepcopy(t.joint_trajectory.points[0]))
        self.assertEqual(remove_identical_time_duplicates(t),1)
        self.assertTrue(strictly_timed(t))
        bad=leg(0,.01);bad.joint_trajectory.points[1].time_from_start.sec=0
        self.assertEqual(remove_identical_time_duplicates(bad),0)
        self.assertFalse(strictly_timed(bad))
