import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/surface_printing'))
from surface_printing.column_tube_geometry import column_tube_path


class ColumnTubeTests(unittest.TestCase):
    def test_default_dimensions_spacing_and_seam(self):
        start=(.5,.2,.01)
        rec,meta=column_tube_path(start)
        self.assertEqual(meta['column_count'],84)
        self.assertLessEqual(meta['center_spacing_mm'],.4)
        self.assertEqual(rec[0][0],start)
        self.assertEqual(rec[100][0],(.49,.2,.01))
        self.assertEqual(rec[-1][0],(.49,.2,.01))
        for (x,y,z),phase,index in rec[101:]:
            self.assertAlmostEqual(math.hypot(x-.48475,y-.2),.00525)
            self.assertGreaterEqual(z,.01-1e-12)
            self.assertLessEqual(z,.013+1e-12)
        self.assertLessEqual(max(math.dist(a[0],b[0]) for a,b in zip(rec,rec[1:])),.000100001)
        self.assertTrue(all(math.dist(a[0],b[0])>1e-12 for a,b in zip(rec,rec[1:])))
        self.assertAlmostEqual(meta['nominal_path_length_mm'],10+84*3+math.pi*10.5)

    def test_columns_are_vertical_and_alternate(self):
        rec,meta=column_tube_path((0,0,0),lead_in_mm=0,column_count=8,height_mm=1)
        for i in range(8):
            points=[p for p,phase,index in rec if phase=='column' and index==i]
            self.assertEqual(len(points),10)
            self.assertTrue(all(abs(p[0]-points[0][0])<1e-12 and
                                abs(p[1]-points[0][1])<1e-12 for p in points))
            for a,b in zip(points,points[1:]):
                self.assertGreater((b[2]-a[2])*(1 if i%2==0 else -1),0)
            self.assertAlmostEqual(points[-1][2],.001 if i%2==0 else 0)
            connector=[p for p,phase,index in rec if phase=='connector' and index==i]
            self.assertTrue(all(p[2]==points[-1][2] for p in connector))

    def test_more_overlap_increases_columns(self):
        _,a=column_tube_path((0,0,0),overlap_fraction=0)
        _,b=column_tube_path((0,0,0),overlap_fraction=.3)
        self.assertGreater(b['column_count'],a['column_count'])

    def test_invalid(self):
        for kwargs in [dict(lead_in_mm=-1),dict(inner_diameter_mm=0),
                       dict(bead_width_mm=0),dict(height_mm=0),
                       dict(overlap_fraction=1),dict(overlap_fraction=-1),
                       dict(column_count=7),dict(column_count=3),
                       dict(column_count=True),dict(column_count=8.5),
                       dict(step_mm=0),dict(height_mm=float('nan'))]:
            with self.assertRaises(ValueError):
                column_tube_path((0,0,0),**kwargs)


if __name__=='__main__':unittest.main()
