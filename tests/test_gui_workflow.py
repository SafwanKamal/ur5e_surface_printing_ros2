"""GUI orchestration contracts without a ROS installation or display server."""
from pathlib import Path
import queue
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/surface_printing_gui'))
from surface_printing_gui.commands import Session, PlanGate, capture, dispense, execution, convert
from surface_printing_gui.processes import ProcessManager


class GUIWorkflow(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.root = Path(self.folder.name)
        self.mock = Session('mock', 78, str(self.root))
        self.physical = Session('physical', 77, str(self.root))
        self.plan = self.root / 'plan.yaml'
        self.plan.write_text('test plan fixture')

    def test_default_mock_cannot_extrude_or_dispense(self):
        with self.assertRaises(ValueError):
            execution(self.mock, self.plan, True, True, True, 1, 60, 1)
        with self.assertRaises(ValueError):
            dispense(self.mock, 0.1, 1, True)
        argv = execution(self.mock, self.plan, True, False, False, 1, 60, 1)
        self.assertIn('simulation:=true', argv)
        self.assertIn('extrude:=false', argv)
        self.assertIn('hardware_confirmed:=false', argv)

    def test_physical_confirmation_and_budget_limits(self):
        with self.assertRaises(ValueError):
            execution(self.physical, self.plan, True, False, False, 1, 60, 1)
        for flow, seconds, volume in [(1, 300, 1), (6, 60, 1), (1, 60, 6), ('nan', 60, 1)]:
            with self.assertRaises(ValueError):
                execution(self.physical, self.plan, True, True, True, flow, seconds, volume)
        with self.assertRaises(ValueError):
            dispense(self.physical, 0.1, 1, False)

    def test_tube_has_explicit_mode_and_rejects_impossible_sizing(self):
        args = (self.mock, 'tube', 10, 'xy', -1, self.root/'tube.csv', 1, 0.5, 0.25)
        argv = capture(*args)
        self.assertIn('simulation:=true', argv)
        with self.assertRaises(ValueError):
            capture(*args[:-2], 2, 0.25)
        with self.assertRaises(ValueError):
            capture(*args[:-1], 2)

    def test_literal_path_argument_is_not_a_shell_command(self):
        source = self.root / 'object $(touch unexpected).csv'
        source.write_text('test')
        argv = convert(source, self.root/'world.csv', [0, 0, 0], [0, 0, 0])
        self.assertIn(str(source), argv)
        manager = ProcessManager()
        manager.start('literal', [sys.executable, '-c', 'import sys; print(sys.argv[1])', str(source)], self.mock)
        log, code = self.await_exit(manager, 'literal')
        self.assertEqual(code, 0)
        self.assertIn(str(source), log)
        self.assertFalse((self.root/'unexpected').exists())

    def test_failed_checks_and_changed_jobs_relock_execution(self):
        gate = PlanGate()
        old = ('domain78', 'csv-hash', 'plan-hash')
        new = ('domain77', 'csv-hash', 'plan-hash')
        gate.check_finished(old, True)
        self.assertFalse(gate.ready(old))
        gate.plan_finished(old, True)
        gate.check_finished(old, False)
        self.assertFalse(gate.ready(old))
        gate.check_finished(old, True)
        self.assertTrue(gate.ready(old))
        self.assertFalse(gate.ready(new))
        gate.invalidate()
        self.assertFalse(gate.ready(old))
        gate.plan_finished(old, False)
        self.assertFalse(gate.ready(old))

    def test_process_domain_logs_exclusive_name_and_interrupt(self):
        manager = ProcessManager()
        code = ('import os,time; print(os.environ["ROS_DOMAIN_ID"],flush=True); '
                'print("READY",flush=True); time.sleep(30)')
        manager.start('worker', [sys.executable, '-c', code], self.physical)
        with self.assertRaises(ValueError):
            manager.start('worker', [sys.executable, '-c', 'pass'], self.mock)
        lines=[]
        deadline=time.monotonic()+5
        try:
            while 'READY' not in lines and time.monotonic()<deadline:
                event,name,value=manager.events.get(timeout=1)
                if event=='log':lines.append(value)
            self.assertIn('77',lines)
            self.assertIn('READY',lines)
        finally:
            manager.interrupt('worker')
        _, code = self.await_exit(manager, 'worker')
        self.assertNotEqual(code, 0)
        self.assertIsNotNone(manager.reap('worker'))
        self.assertFalse(manager.jobs)

    def await_exit(self, manager, name):
        lines=[]
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            try:event,_,value=manager.events.get(timeout=0.2)
            except queue.Empty:continue
            if event=='exit':return '\n'.join(lines),value
            lines.append(value)
        manager.interrupt(name)
        self.fail('Subprocess did not exit in time')


if __name__ == '__main__':
    unittest.main()
