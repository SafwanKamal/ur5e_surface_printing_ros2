import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/surface_printing'))
sys.path.insert(0, str(ROOT/'src/ur5e_toolheads'))
from surface_printing.tube_geometry import tube_points, normalized_orientation
from ur5e_toolheads.set_axial_nozzle_tcp import axial_tcp


def rotation(rpy):
    r,p,y = rpy
    cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return ((cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),
            (sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),(-sp,cp*sr,cp*cr))


class TubeTests(unittest.TestCase):
    def test_start_radius_height_and_step(self):
        start = (.4,.2,.01)
        points = tube_points(start,10,5,.5,.25)
        self.assertEqual(points[0],start)
        self.assertAlmostEqual(points[-1][2],.015)
        for p in points:
            self.assertAlmostEqual(math.hypot(p[0]-.395,p[1]-.2),.005)
        self.assertLessEqual(max(math.dist(a,b) for a,b in zip(points,points[1:])),.000250001)
        self.assertTrue(all(a[2]<b[2] for a,b in zip(points,points[1:])))

    def test_invalid(self):
        for dims in [(10,0,.5,.25),(10,5,0,.25),(10,5,.5,0),
                     (float('nan'),5,.5,.25),(10,5,6,.25),(100,100,.00001,.001)]:
            with self.assertRaises(ValueError): tube_points((0,0,0),*dims)

    def test_quaternion(self):
        self.assertEqual(normalized_orientation((2.,0.,0.,0.)),(1.,0.,0.,0.))
        with self.assertRaises(ValueError): normalized_orientation((0.,0.,0.,0.))

    def test_flange_composition(self):
        for mount in [(0,0,0),(0,math.pi/2,0),(0,-math.pi/2,0),(.2,.6,-.9)]:
            config=dict(mount_xyz=[.02,-.003,.01],mount_rpy=mount)
            xyz,rpy=axial_tcp(config,160)
            a,b=rotation(mount),rotation(rpy)
            actual=[config['mount_xyz'][i]+sum(a[i][j]*xyz[j] for j in range(3)) for i in range(3)]
            for got,expected in zip(actual,[.160,0,0]): self.assertAlmostEqual(got,expected)
            expected_rotation=((0,0,1),(1,0,0),(0,1,0))
            for i in range(3):
                for j in range(3):
                    self.assertAlmostEqual(sum(a[i][k]*b[k][j] for k in range(3)),expected_rotation[i][j])

    def test_bad_axial_length(self):
        for length in [0,-1,301,float('nan')]:
            with self.assertRaises(ValueError): axial_tcp(dict(mount_xyz=[0]*3,mount_rpy=[0]*3),length)


if __name__=='__main__': unittest.main()
