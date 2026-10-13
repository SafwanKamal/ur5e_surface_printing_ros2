import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/surface_printing'))
from surface_printing.lead_helix_geometry import lead_helix_points
from surface_printing.tube_geometry import tube_points


class LeadHelixTests(unittest.TestCase):
    def test_start_lead_radius_height_and_spacing(self):
        start = (.4, .2, .01)
        p = lead_helix_points(start, 10, 2, .5, 10, .5, .1)
        self.assertEqual(p[0], start)
        self.assertEqual(p[100], (.39, .2, .01))
        self.assertAlmostEqual(p[-1][0], .39)
        self.assertAlmostEqual(p[-1][1], .2)
        self.assertAlmostEqual(p[-1][2], .02)
        for x,y,z in p[100:]:
            self.assertAlmostEqual(math.hypot(x-.38875,y-.2),.00125)
        self.assertTrue(all(a[2]<b[2] for a,b in zip(p[100:],p[101:])))
        self.assertTrue(all(a!=b for a,b in zip(p,p[1:])))
        self.assertLessEqual(max(math.dist(a,b) for a,b in zip(p,p[1:])),.000100001)

    def test_zero_lead_and_no_compensation(self):
        self.assertEqual(lead_helix_points((0,0,0),0,2,0,1,.5,.1),
                         tube_points((0,0,0),2,1,.5,.1))

    def test_partial_turn_height(self):
        p=lead_helix_points((0,0,0),5,2,.5,1.2,.5,.1)
        self.assertAlmostEqual(p[-1][2],.0012)

    def test_invalid(self):
        for dims in [(-1,2,.5,10,.5,.1),(101,2,.5,10,.5,.1),
                     (10,0,.5,10,.5,.1),(10,2,-.5,10,.5,.1),
                     (10,2,.5,0,.5,.1),(10,2,.5,10,0,.1),
                     (10,2,.5,10,.5,0),(10,2,float('nan'),10,.5,.1)]:
            with self.assertRaises(ValueError):
                lead_helix_points((0,0,0),*dims)


if __name__ == '__main__':
    unittest.main()
