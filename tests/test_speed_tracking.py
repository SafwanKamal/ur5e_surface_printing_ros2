import math
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/surface_printing'))
from surface_printing.speed_tracking import SpeedTracker


class LinearModel:
    def position(self,q):
        import numpy as np
        return np.array([q[0],0.,0.])


def joints(t,x):
    sec=int(t)
    return S(name=['j'],position=[x],header=S(stamp=S(sec=sec,nanosec=round((t-sec)*1e9))))


class SpeedTrackingTests(unittest.TestCase):
    def test_constant_actual_motion_meets_target(self):
        tracker=SpeedTracker(LinearModel(),['j'],1.2,1.,30.)
        for i in range(301):
            tracker.update(joints(100+i*.1,i*.0001))
        r=tracker.report()
        self.assertAlmostEqual(r['measured_average_mm_s'],1.)
        self.assertAlmostEqual(r['measured_coverage'],1.)
        self.assertAlmostEqual(r['measured_distance_mm'],30.)

    def test_slow_motion_aborts_when_90_percent_impossible(self):
        tracker=SpeedTracker(LinearModel(),['j'],1.2,1.,30.)
        with self.assertRaisesRegex(RuntimeError,'90%'):
            for i in range(100):
                tracker.update(joints(100+i*.1,i*.00005))

    def test_overspeed_detected(self):
        tracker=SpeedTracker(LinearModel(),['j'],1.2,1.,30.)
        tracker.update(joints(100,0))
        with self.assertRaisesRegex(RuntimeError,'exceeds'):
            tracker.update(joints(100.2,.0004))

    def test_duplicate_samples_and_clock_reversal(self):
        tracker=SpeedTracker(LinearModel(),['j'],1.2,1.,30.)
        tracker.update(joints(100,0));tracker.update(joints(100,0))
        self.assertEqual(tracker.windows,0)
        with self.assertRaisesRegex(RuntimeError,'backward'):
            tracker.update(joints(99,0))

    def test_stationary_time_does_not_count_as_path_coverage(self):
        tracker=SpeedTracker(LinearModel(),['j'],1.2,1.,30.)
        tracker.update(joints(100,0));tracker.update(joints(101,0))
        tracker.update(joints(101.2,.0002))
        self.assertAlmostEqual(tracker.report()['measured_coverage'],1.)


if __name__=='__main__':unittest.main()
