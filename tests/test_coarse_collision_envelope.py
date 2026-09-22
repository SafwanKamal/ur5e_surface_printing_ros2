import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
XACRO = (
    ROOT / "src/ur5e_toolheads/urdf/probe_tool.xacro"
).read_text()
VALIDATOR = (
    ROOT / "src/surface_printing/surface_printing/demo_safety.py"
).read_text()


class CoarseCollisionEnvelopeTests(unittest.TestCase):
    def test_body_starts_at_sleeve_boundary(self):
        origin = re.search(
            r'<origin xyz="\$\{([0-9.]+)\*pump\[\'mesh_scale\'\]\[0\]\} '
            r'\$\{1081\.41265\*pump\[\'mesh_scale\'\]\[1\]\} '
            r'\$\{173\.7\*pump\[\'mesh_scale\'\]\[2\]\}"/>',
            XACRO,
        )
        size = re.search(
            r'<geometry><box size="\$\{([0-9.]+)\*pump\[\'mesh_scale\'\]\[0\]\} '
            r'\$\{43\.0251\*pump\[\'mesh_scale\'\]\[1\]\} '
            r'\$\{203\.6\*pump\[\'mesh_scale\'\]\[2\]\}"/></geometry>',
            XACRO,
        )
        self.assertIsNotNone(origin)
        self.assertIsNotNone(size)
        center = float(origin.group(1))
        width = float(size.group(1))
        self.assertAlmostEqual(center - width / 2.0, 54.0)
        self.assertAlmostEqual(center + width / 2.0, 125.25)

    def test_sleeve_meets_body_boundary(self):
        self.assertIn(
            'radius="${54.0*max(pump[\'mesh_scale\'][0],pump[\'mesh_scale\'][1])}"',
            XACRO,
        )

    def test_preflight_matches_corrected_envelopes(self):
        self.assertIn(
            "'pump_sleeve_envelope':('cylinder',(0,1079.4,205.4),(54.0,62.0))",
            VALIDATOR,
        )
        self.assertIn(
            "'pump_body_envelope':('box',(89.625,1081.41265,173.7),"
            "(71.25,43.0251,203.6))",
            VALIDATOR,
        )

    def test_tcp_and_mount_transforms_remain_config_driven(self):
        self.assertIn("pump['mount_rpy']", XACRO)
        self.assertIn("pump['tcp_xyz']", XACRO)
        self.assertIn("pump['tcp_rpy']", XACRO)

    def test_thin_needle_is_collision_checked_and_tip_is_clear(self):
        self.assertIn('name="pump_needle_envelope"', XACRO)
        self.assertIn("pump.get('needle_radius_mm',1.0)/1000.0", XACRO)
        self.assertIn("needle_collision_tip_clearance_mm", XACRO)
        self.assertIn("envelope_names.add('pump_needle_envelope')", VALIDATOR)


if __name__ == "__main__":
    unittest.main()
