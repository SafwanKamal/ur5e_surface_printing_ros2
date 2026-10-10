"""Ramp timing and demo admission checks without ROS dependencies."""
import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/syringe_controller'))
from syringe_controller.motion_profile import move_duration


class MotionProfile(unittest.TestCase):
    def test_low_rate_and_single_pulse(self):
        self.assertAlmostEqual(move_duration(-10, 50), .001 + 9 / 50 + .00001)
        self.assertAlmostEqual(move_duration(1, 5000), .00101)

    def test_high_target_short_move_cannot_reach_plateau(self):
        self.assertEqual(move_duration(25, 1000), move_duration(25, 5000))
        self.assertGreater(move_duration(3000, 5000), 3000 / 5000)
        self.assertEqual(move_duration(3000, 5000), move_duration(-3000, 5000))

    def test_demo_duration_boundaries(self):
        self.assertLess(move_duration(3000, 30), 100)
        self.assertGreater(move_duration(3000, 29), 100)
        self.assertLess(move_duration(3000, 5000), 100)

    def test_invalid(self):
        for steps, rate in [(0, 100), (3001, 100), (1, 0), (1, 5001), (1, math.nan)]:
            with self.assertRaises(ValueError):
                move_duration(steps, rate)
