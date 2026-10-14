"""Isolate the initial motion-controller spawner from optional controller groups.

Use the installed vendor launch and its parameters. Do not modify /opt/ros files
or disable the UR controller stopper. The source transformation is deliberately
restricted to the two grouped controller-spawner calls in launch_setup.
"""
import ast
from pathlib import Path
from types import ModuleType

from ament_index_python.packages import get_package_share_directory


def separate_initial_controller(source, filename):
    tree=ast.parse(source,filename)
    patched=0
    for function in tree.body:
        if not isinstance(function,ast.FunctionDef) or function.name!='launch_setup':
            continue
        body=[]
        for statement in function.body:
            is_spawners=(isinstance(statement,ast.Assign) and
                         any(isinstance(t,ast.Name) and t.id=='controller_spawners'
                             for t in statement.targets))
            if not is_spawners:
                body.append(statement);continue
            if (not isinstance(statement.value,ast.List) or len(statement.value.elts)!=2
                or any(not isinstance(e,ast.Call) or not isinstance(e.func,ast.Name)
                       or e.func.id!='controller_spawner' for e in statement.value.elts)):
                raise RuntimeError('Unsupported UR driver spawner layout; inspect installed launch')
            prep=ast.parse(
                'selected = initial_joint_controller.perform(context)\n'
                'controllers_active = [c for c in controllers_active if c != selected]\n'
                'controllers_inactive = [c for c in controllers_inactive if c != selected]\n').body
            independent=ast.parse(
                "controller_spawner([selected], "
                "active=activate_joint_controller.perform(context) == 'true')").body[0].value
            statement.value.elts.insert(0,independent)
            body.extend(prep);body.append(statement);patched+=1
        function.body=body
    if patched!=1:
        raise RuntimeError('Could not identify UR driver grouped spawners; no launch started')
    return ast.fix_missing_locations(tree)


def generate_launch_description():
    filename=Path(get_package_share_directory('ur_robot_driver'))/'launch/ur_control.launch.py'
    source=filename.read_text()
    module=ModuleType('ur_driver_independent_initial_controller')
    module.__file__=str(filename)
    exec(compile(separate_initial_controller(source,str(filename)),str(filename),'exec'),
         module.__dict__)
    return module.generate_launch_description()
