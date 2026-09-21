import sys
from pathlib import Path
import unittest
import math
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/ur5e_toolheads'))
from ur5e_toolheads.set_needle_tcp import calculate_tcp

class NeedleTCP(unittest.TestCase):
    def config(self):
        return dict(syringe_outlet_cad_mm=[96.74983,1081.40020,325.88336],
                    mesh_scale=[.001]*3,mesh_xyz=[0,-1.0794,-.227868],mesh_rpy=[0,0,0])
    def test_outlet(self):
        xyz,_=calculate_tcp(self.config(),0)
        for a,b in zip(xyz,[.09674983,.00200020,.09801536]): self.assertAlmostEqual(a,b)
    def test_rotated_axis(self):
        c=self.config();c['mesh_rpy']=[0,math.pi/2,0]
        base,_=calculate_tcp(c,0);tip,_=calculate_tcp(c,25)
        for a,b in zip([tip[i]-base[i] for i in range(3)],[.025,0,0]):self.assertAlmostEqual(a,b)
    def test_invalid(self):
        for n in [-1,201,float('nan'),float('inf')]:
            with self.assertRaises(ValueError):calculate_tcp(self.config(),n)
