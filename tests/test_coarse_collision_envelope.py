import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
XACRO = (
    ROOT / "src/ur5e_toolheads/urdf/probe_tool.xacro"
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
        self.assertAlmostEqual(center - width / 2.0, 53.0)
        self.assertAlmostEqual(center + width / 2.0, 125.25)

    def test_tcp_and_mount_transforms_remain_config_driven(self):
        self.assertIn("pump['mount_rpy']", XACRO)
        self.assertIn("pump['tcp_xyz']", XACRO)
        self.assertIn("pump['tcp_rpy']", XACRO)


if __name__ == "__main__":
    unittest.main()
