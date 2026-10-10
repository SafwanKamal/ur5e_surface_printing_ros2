"""Test simulation/physical model gates without a ROS installation."""
import ast
from pathlib import Path
import unittest
from unittest.mock import Mock
import xml.etree.ElementTree as ET


SOURCE = Path(__file__).resolve().parents[1] / 'src/surface_printing/surface_printing/demo_safety.py'


def load_functions(names, namespace):
    tree = ast.parse(SOURCE.read_text())
    tree.body = [item for item in tree.body
                 if isinstance(item, ast.FunctionDef) and item.name in names]
    exec(compile(tree, str(SOURCE), 'exec'), namespace)
    return namespace


def model(plugin):
    return f'<robot><ros2_control><hardware><plugin>{plugin}</plugin></hardware></ros2_control></robot>'


class TubeSimulationTests(unittest.TestCase):
    def setUp(self):
        self.geometry = Mock()
        self.ns = load_functions(
            {'validate_tube_tool', 'require_mock_model', 'require_real_model'},
            {'ET': ET, 'calibrated_tool': self.geometry})

    def test_mock_still_validates_geometry(self):
        xml = model('mock_components/GenericSystem')
        self.ns['validate_tube_tool'](xml, simulation=True)
        self.geometry.assert_called_once_with(xml, require_calibration=False)

    def test_simulation_cannot_bypass_physical_calibration(self):
        with self.assertRaises(RuntimeError):
            self.ns['validate_tube_tool'](model('ur_robot_driver/URPositionHardwareInterface'), simulation=True)
        self.geometry.assert_not_called()

    def test_default_physical_mode_requires_calibration(self):
        xml = model('ur_robot_driver/URPositionHardwareInterface')
        self.geometry.side_effect = RuntimeError('not calibrated')
        with self.assertRaisesRegex(RuntimeError, 'not calibrated'):
            self.ns['validate_tube_tool'](xml)
        self.geometry.assert_called_once_with(xml)

    def test_physical_mode_rejects_mock(self):
        with self.assertRaises(RuntimeError):
            self.ns['validate_tube_tool'](model('mock_components/GenericSystem'))
        self.geometry.assert_not_called()

    def test_simulation_propagates_invalid_geometry(self):
        self.geometry.side_effect = RuntimeError('incorrect TCP geometry')
        with self.assertRaisesRegex(RuntimeError, 'incorrect TCP geometry'):
            self.ns['validate_tube_tool'](model('mock_components/GenericSystem'), simulation=True)


if __name__ == '__main__':
    unittest.main()
