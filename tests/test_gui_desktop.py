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
                self.addCleanup(self.destroy_root, root)
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

    def test_mock_window_live_trace_and_session_isolation(self):
        import tkinter as tk
        from surface_printing_gui.app import PrintingApp
        with tempfile.TemporaryDirectory() as folder:
            d=Path(folder)
            (d/'install').mkdir()
            (d/'bin').mkdir()
            endpoint=d/'bin/ros2'
            endpoint.write_text('''#!/usr/bin/env python3
import json,os,pathlib,sys,time
assert os.environ['ROS_DOMAIN_ID']=='78', 'Mock command escaped domain 78'
if 'mock_monitor' in sys.argv:
 i=0
 while True:
  print('MOCK_STATE '+json.dumps({'joints':{'elbow_joint':i*.001},
        'tcp':[.2+i*.00001,.1,.3], 'joint_age':.01, 'tf_age':.01}),flush=True)
  i+=1
  time.sleep(.05)
elif 'plan_surface_path' in sys.argv:
 for arg in sys.argv:
  if arg.startswith('output_path:='):
   pathlib.Path(arg.split(':=',1)[1]).write_text('duration_sec: 0.4')
elif 'execute:=true' in sys.argv:
 assert 'simulation:=true' in sys.argv
 assert 'extrude:=false' in sys.argv
 time.sleep(.5)
''')
            endpoint.chmod(0o755)
            environment={'PATH':str(d/'bin')+os.pathsep+os.environ['PATH'],
                         'AMENT_PREFIX_PATH':str(d/'install')}
            with patch.dict(os.environ,environment):
                root=tk.Tk()
                self.addCleanup(self.destroy_root, root)
                app=PrintingApp(root,str(d),str(d))
                csv=d/'world.csv'
                csv.write_text('line_id,point_index,x,y,z\n0,0,.2,.1,.3\n0,1,.21,.1,.3\n')
                app.variables['world_csv'].set(str(csv))
                app.variables['trajectory'].set(str(d/'plan.yaml'))
                app.mock_isolated.set(True)
                app.open_mock_window()
                window=app.mock_window
                window.connect()
                self.wait_for(root,lambda:window.feedback.fresh())
                app.plan_path()
                self.wait_for(root,lambda:app.task is None)
                app.check_plan()
                self.wait_for(root,lambda:app.task is None)
                app.confirmed.set(True)
                self.wait_for(root,lambda:str(window.execute.cget('state'))=='normal')
                window.action(window.rehearse)
                self.wait_for(root,lambda:app.task is None)
                self.assertTrue(window.observed)
                self.assertGreater(len(window.feedback.trace),1)
                self.assertIn('MOCK REHEARSAL COMPLETED',window.message.get())
                self.assertFalse(app.gate.ready(app.signature()))
                app.variables['mode'].set('physical')
                root.update()
                with self.assertRaises(ValueError):window.action(window.rehearse)
                window.stop()
                self.wait_for(root,lambda:not any(n.startswith('STOP request') for n in app.processes.jobs))
                stop_lines=[line for line in app.logs.get('1.0','end').splitlines()
                            if 'STOP request' in line and 'service call' in line]
                self.assertEqual(len(stop_lines),2)
                self.assertTrue(all('domain 78, mock' in line for line in stop_lines))
                window.close()
                self.drain(root,app)
                self.assertIsNone(app.mock_window)

    def destroy_root(self,root):
        for callback in root.tk.call('after','info'):
            root.after_cancel(callback)
        root.destroy()

    def wait_for(self,root,predicate):
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            root.update()
            if predicate():return
            time.sleep(.01)
        self.fail('Desktop condition did not become true')

    def drain(self,root,app):
        deadline=time.monotonic()+8
        while app.processes.jobs and time.monotonic()<deadline:
            root.update()
            time.sleep(.01)
        self.assertFalse(app.processes.jobs,'Fake command did not exit')
        root.update()
