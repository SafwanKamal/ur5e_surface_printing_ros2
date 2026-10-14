"""Test launch transformation without requiring ROS processes or hardware."""
import ast
from pathlib import Path
import unittest
ROOT=Path(__file__).resolve().parents[1]
FILE=ROOT/'src/ur5e_probe_moveit_config/launch/ur_control_independent.launch.py'
tree=ast.parse(FILE.read_text())
tree.body=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='separate_initial_controller']
namespace={'ast':ast}
exec(compile(tree,str(FILE),'exec'),namespace)
transform=namespace['separate_initial_controller']

SOURCE="""
def launch_setup(context):
    controllers_active = ['joint_state_broadcaster', 'gravity_update_controller']
    controllers_inactive = ['scaled_joint_trajectory_controller', 'twist_controller']
    if activate_joint_controller.perform(context) == 'true':
        controllers_active.append(initial_joint_controller.perform(context))
        controllers_inactive.remove(initial_joint_controller.perform(context))
    controller_spawners = [
        controller_spawner(controllers_active),
        controller_spawner(controllers_inactive, active=False),
    ]
    return controller_spawners
"""

class Argument:
    def __init__(self,value):self.value=value
    def perform(self,context):return self.value


class IndependentSpawnerTests(unittest.TestCase):
    def launch(self,active):
        calls=[]
        def spawner(names,active=True):
            result=(list(names),active);calls.append(result);return result
        env=dict(initial_joint_controller=Argument('scaled_joint_trajectory_controller'),
                 activate_joint_controller=Argument('true' if active else 'false'),
                 controller_spawner=spawner)
        exec(compile(transform(SOURCE,'vendor'),'vendor','exec'),env)
        return env['launch_setup'](None)

    def test_active_motion_is_independent_of_optional_groups(self):
        result=self.launch(True)
        self.assertEqual(result[0],(['scaled_joint_trajectory_controller'],True))
        self.assertEqual(result[1],(['joint_state_broadcaster','gravity_update_controller'],True))
        self.assertEqual(result[2],(['twist_controller'],False))
        self.assertEqual(sum(names.count('scaled_joint_trajectory_controller')
                             for names,active in result),1)

    def test_inactive_setting_preserved(self):
        result=self.launch(False)
        self.assertEqual(result[0],(['scaled_joint_trajectory_controller'],False))

    def test_incompatible_vendor_launch_fails_explicitly(self):
        with self.assertRaisesRegex(RuntimeError,'identify'):
            transform('def launch_setup(context): return []','vendor')
        with self.assertRaisesRegex(RuntimeError,'layout'):
            transform(SOURCE.replace('controller_spawner(controllers_active)',
                                     'other_spawner(controllers_active)'),'vendor')


if __name__=='__main__': unittest.main()
