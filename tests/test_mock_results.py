"""Mock visualisation must reflect telemetry, not animated command estimates."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/surface_printing_gui'))
from surface_printing_gui.mock_monitor import PREFIX, main
from surface_printing_gui.mock_results import MockFeedback, path_length_mm, read_path, rehearsal_blockers
from unittest.mock import patch
from surface_printing_gui.commands import PlanGate


class MockResults(unittest.TestCase):
    def packet(self, **updates):
        data = {'joints': {'elbow_joint': 0.1}, 'tcp': [0.2, 0.1, 0.3],
                'joint_age': 0.01, 'tf_age': 0.02}
        data.update(updates)
        return PREFIX + json.dumps(data)

    def test_actual_feedback_trace_and_freshness(self):
        feedback = MockFeedback()
        feedback.accept(self.packet(), now=10)
        self.assertTrue(feedback.fresh(now=10))
        self.assertFalse(feedback.trace)  # idle samples are not rehearsal results
        feedback.start()
        feedback.accept(self.packet(), now=10.1)
        feedback.accept(self.packet(tcp=[0.201, 0.1, 0.3]), now=10.2)
        self.assertEqual(feedback.samples, 2)
        self.assertEqual(len(feedback.trace), 2)
        self.assertFalse(feedback.fresh(now=12))
        feedback.accept(self.packet(joint_age=2), now=12)
        self.assertFalse(feedback.fresh(now=12))
        self.assertEqual(feedback.samples, 2)
        feedback.accept(self.packet(tf_age=2), now=13)
        self.assertFalse(feedback.fresh(now=13))

    def test_bad_or_missing_feedback_is_not_live(self):
        feedback = MockFeedback()
        self.assertFalse(feedback.accept('ROS log line'))
        for changes in ({'tcp': [float('nan'), 0, 0]}, {'tcp': [1]},
                        {'joints': []}, {'tf_age': -1}, {'joint_age': 'fresh'}):
            with self.assertRaises(ValueError):feedback.accept(self.packet(**changes))
        feedback.accept(self.packet(tcp=None), now=10)
        self.assertFalse(feedback.fresh(now=10))

    def test_csv_selection_order_units_and_duplicate_rejection(self):
        with tempfile.TemporaryDirectory() as folder:
            filename=Path(folder)/'path.csv'
            filename.write_text('line_id,point_index,x,y,z\n0,1,0.01,0,0\n1,0,9,9,9\n0,0,0,0,0\n')
            points=read_path(filename,0)
            self.assertEqual(points, [(0.,0.,0.),(.01,0.,0.)])
            self.assertAlmostEqual(path_length_mm(points),10)
            with self.assertRaises(ValueError):read_path(filename,1)
            filename.write_text('line_id,point_index,x,y,z\n0,0,0,0,0\n0,0,0.01,0,0\n')
            with self.assertRaises(ValueError):read_path(filename,0)

    def test_monitor_refuses_other_domains_before_importing_ros(self):
        for domain in ('77', '0', ''):
            with patch.dict('os.environ', {'ROS_DOMAIN_ID': domain}):
                with self.assertRaises(SystemExit):main()

    def test_lock_explains_failed_check_and_missing_feedback(self):
        feedback=MockFeedback()
        gate=PlanGate()
        signature=('job','plan')
        gate.plan_finished(signature,True)
        kwargs=dict(same_session=True,busy=False,isolated=True,confirmed=True,
                    feedback=feedback,gate=gate,signature=signature)
        reasons=rehearsal_blockers(**kwargs)
        self.assertTrue(any('check has not passed' in reason for reason in reasons))
        self.assertTrue(any('no monitor samples' in reason for reason in reasons))
        gate.check_finished(signature,True)
        feedback.accept(self.packet())
        self.assertEqual(rehearsal_blockers(**kwargs),[])
        feedback.accept(self.packet(tf_age=2))
        self.assertTrue(any('TCP transform stale' in reason for reason in rehearsal_blockers(**kwargs)))
        kwargs['same_session']=False
        self.assertTrue(any('domain 78' in reason for reason in rehearsal_blockers(**kwargs)))
        kwargs['signature']=('changed','plan')
        self.assertTrue(any('current files/settings' in reason for reason in rehearsal_blockers(**kwargs)))
