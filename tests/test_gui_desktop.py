"""Optional desktop smoke with fake ROS argv endpoints; never connects hardware."""
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/surface_printing_gui'))


@unittest.skipUnless(os.environ.get('DISPLAY'), 'Requires a desktop/virtual X display')
class DesktopSmoke(unittest.TestCase):
    def test_tabs_plan_gate_changed_file_and_original_domain_stop(self):
        import tkinter as tk
        from surface_printing_gui.app import PrintingApp
        with tempfile.TemporaryDirectory() as folder:
            d=Path(folder)
            (d/'install').mkdir()
            (d/'bin').mkdir()
            endpoint=d/'bin/ros2'
            endpoint.write_text('''#!/usr/bin/env python3
import os,pathlib,sys,time
print('Mock desktop fixture domain='+os.environ['ROS_DOMAIN_ID'],flush=True)
if 'plan_surface_path' in sys.argv:
 for arg in sys.argv:
  if arg.startswith('output_path:='):
   pathlib.Path(arg.split(':=',1)[1]).write_text('fake plan')
if 'execute:=true' in sys.argv:
 time.sleep(1)
''')
            endpoint.chmod(0o755)
            environment={'PATH':str(d/'bin')+os.pathsep+os.environ['PATH'],
                         'AMENT_PREFIX_PATH':str(d/'install')}
            with patch.dict(os.environ,environment):
                root=tk.Tk()
                self.addCleanup(root.destroy)
                app=PrintingApp(root,str(d),str(d))
                csv=d/'world.csv'
                csv.write_text('test CSV fixture')
                app.variables['world_csv'].set(str(csv))
                app.variables['trajectory'].set(str(d/'plan.yaml'))
                app.mock_isolated.set(True)
                self.assertEqual(str(app.execute_button.cget('state')),'disabled')
                app.plan_path()
                self.drain(root,app)
                app.check_plan()
                self.drain(root,app)
                app.confirmed.set(True)
                root.update()
                self.assertTrue(app.gate.ready(app.signature()))
                self.assertEqual(str(app.execute_button.cget('state')),'normal')
                for i in range(6):
                    app.notebook.select(i)
                    root.update()
                csv.write_text('modified outside the GUI')
                with self.assertRaises(ValueError):app.execute_plan()
                app.plan_path()
                self.drain(root,app)
                app.check_plan()
                self.drain(root,app)
                app.confirmed.set(True)
                app.execute_plan()
                app.variables['mode'].set('physical')
                app.request_stop()
                self.drain(root,app)
                self.assertFalse(app.gate.ready(app.signature()))
                stop_lines=[line for line in app.logs.get('1.0','end').splitlines()
                            if 'STOP request' in line and 'service call' in line]
                self.assertEqual(len(stop_lines),2)
                self.assertTrue(all('domain 78, mock' in line for line in stop_lines))

    def drain(self,root,app):
        deadline=time.monotonic()+8
        while app.processes.jobs and time.monotonic()<deadline:
            root.update()
            time.sleep(.01)
        self.assertFalse(app.processes.jobs,'Fake command did not exit')
        root.update()
