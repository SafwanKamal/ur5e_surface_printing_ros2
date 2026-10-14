"""Repeated paths preserve endpoints and exact length, without connector moves."""
import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/surface_printing'))
from surface_printing.demo_geometry import demo_offsets, repeat_offsets, rotate_offsets


class RepeatabilityGeometry(unittest.TestCase):
    def test_diagonal_line_preserves_length_and_start(self):
        for angle in [45, -45, 90, 180]:
            original=repeat_offsets(demo_offsets('line',10,.25),3)
            rotated=rotate_offsets(original,angle)
            self.assertEqual(rotated[0],(0.,0.))
            self.assertAlmostEqual(sum(math.dist(a,b) for a,b in zip(rotated,rotated[1:])),.03)
        end=rotate_offsets(demo_offsets('line',10),45)[-1]
        self.assertAlmostEqual(end[0],.01/math.sqrt(2))
        self.assertAlmostEqual(end[1],.01/math.sqrt(2))
        self.assertEqual(rotate_offsets(original,0),original)

    def test_rotation_rejects_nonfinite_angle(self):
        for angle in [float('nan'),float('inf')]:
            with self.assertRaises(ValueError): rotate_offsets([(0,0),(1,0)],angle)

    def test_three_line_traversals(self):
        for mm in [10, 5, 2]:
            one = demo_offsets('line', mm, .25)
            path = repeat_offsets(one, 3)
            self.assertEqual(len(path), 3 * (len(one) - 1) + 1)
            self.assertEqual(path[0], one[0])
            self.assertEqual(path[-1], one[-1])
            self.assertEqual(path[len(one)-1], one[-1])
            self.assertEqual(path[2*(len(one)-1)], one[0])
            self.assertAlmostEqual(sum(math.dist(a,b) for a,b in zip(path,path[1:])),
                                   mm * 3 / 1000)

    def test_closed_shapes(self):
        for shape in ['square', 'circle']:
            one = demo_offsets(shape, 10, .25)
            path = repeat_offsets(one, 3)
            self.assertEqual(path[0], path[-1])
            self.assertEqual(len(path), 3*(len(one)-1)+1)
        square = demo_offsets('square', 10)
        self.assertAlmostEqual(sum(math.dist(a,b) for a,b in zip(square,square[1:])), .04)

    def test_invalid_passes_and_single_pass(self):
        one = demo_offsets('line', 2)
        self.assertEqual(repeat_offsets(one, 1), one)
        for count in [0, -1, 1.5, True, 101]:
            with self.assertRaises(ValueError): repeat_offsets(one, count)
