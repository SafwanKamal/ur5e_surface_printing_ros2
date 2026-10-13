"""Verify lead-in anchoring, bead compensation, spacing and repeated closure."""
import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/surface_printing'))
from surface_printing.lead_circle_geometry import lead_circle_offsets


class LeadCircleGeometry(unittest.TestCase):
    def test_compensated_circle_and_lead(self):
        points = lead_circle_offsets(10, 2, .5)
        self.assertEqual(points[0], (0., 0.))
        self.assertEqual(points[100], (-.010, 0.))
        self.assertEqual(points[-1], (-.010, 0.))
        center, radius = (-.01125, 0.), .00125
        for p in points[100:]:
            self.assertAlmostEqual(math.dist(p, center), radius)
        self.assertAlmostEqual(min(p[0] for p in points), -.0125, delta=2e-6)
        self.assertLessEqual(max(math.dist(a,b) for a,b in zip(points,points[1:])), .000100001)

    def test_positive_direction_and_repeats(self):
        points = lead_circle_offsets(10, 2, .5, direction=1., passes=3)
        self.assertEqual(points[-1], (.010, 0.))
        self.assertAlmostEqual(max(p[0] for p in points), .0125, delta=2e-6)
        self.assertEqual(points.count((.010, 0.)), 4)

    def test_zero_lead(self):
        points = lead_circle_offsets(0, 2, 0)
        self.assertEqual(points[0], (0., 0.))
        self.assertEqual(points[-1], points[0])
        self.assertTrue(all(a != b for a,b in zip(points,points[1:])))

    def test_invalid_inputs(self):
        for values in [(-1,2,.5), (101,2,.5), (10,0,.5),
                       (10,2,-.5), (10,100,.5), (10,float('nan'),.5)]:
            with self.assertRaises(ValueError):
                lead_circle_offsets(*values)
        for kwargs in [dict(direction=0), dict(spacing_mm=0),
                       dict(passes=0), dict(passes=1.5)]:
            with self.assertRaises(ValueError):
                lead_circle_offsets(10,2,.5,**kwargs)


if __name__ == '__main__':
    unittest.main()
