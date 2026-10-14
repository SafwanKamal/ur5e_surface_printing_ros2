"""Run the actual generator body with mocked FK and model services."""
import ast
import csv
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/surface_printing'))
from surface_printing.column_tube_geometry import column_tube_path
from surface_printing.tube_geometry import normalized_orientation
SRC=Path(__file__).resolve().parents[1]/'src/surface_printing/surface_printing'


class ColumnGeneratorTests(unittest.TestCase):
    def test_capture_and_fixed_orientation(self):
        tree=ast.parse((SRC/'make_column_tube_path.py').read_text())
        tree.body=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='run']
        validate_tool=Mock()
        namespace=dict(csv=csv,math=math,Path=Path,column_tube_path=column_tube_path,
                       normalized_orientation=normalized_orientation,
                       live_model=lambda node:('physical_model','hash'),
                       validate_tube_tool=validate_tool,
                       GetPositionFK=NS(Request=lambda:NS(header=NS(),robot_state=NS())))
        exec(compile(tree,'generator','exec'),namespace)
        names=[name+'_joint' for name in
               ('shoulder_pan','shoulder_lift','elbow','wrist_1','wrist_2','wrist_3')]
        joints=NS(name=names,position=[0]*6,header=NS(stamp=NS(sec=100,nanosec=0)))
        with tempfile.TemporaryDirectory() as d:
            out=Path(d)/'path.csv'
            params=dict(output_path=str(out),column_count=8)
            client=Mock()
            client.wait_for_service.return_value=True
            pose=NS(position=NS(x=.5,y=.2,z=.01),orientation=NS(x=1.,y=0.,z=0.,w=0.))
            result=NS(error_code=NS(val=1),pose_stamped=[NS(header=NS(frame_id='world'),pose=pose)])
            node=NS(parameter=lambda k,default:params.get(k,default),
                    create_client=lambda *args:client,fresh_joints=lambda:joints,
                    get_clock=lambda:NS(now=lambda:NS(nanoseconds=100000000000)),
                    wait=lambda future:result,get_logger=lambda:Mock())
            namespace['run'](node)
            validate_tool.assert_called_once_with('physical_model',simulation=False)
            request=client.call_async.call_args.args[0]
            self.assertEqual(request.header.frame_id,'world')
            self.assertEqual(request.fk_link_names,['probe_tcp'])
            with out.open() as f: rows=list(csv.DictReader(f))
            self.assertEqual(tuple(float(rows[0][a]) for a in 'xyz'),(.5,.2,.01))
            self.assertTrue(all(row['line_id']=='0' for row in rows))
            self.assertTrue(all([row[k] for k in ('qx','qy','qz','qw')]==['1.0','0.0','0.0','0.0']
                                for row in rows))
            out.unlink()
            pose.orientation=NS(x=0.,y=0.,z=0.,w=1.)
            with self.assertRaisesRegex(RuntimeError,'downward'):
                namespace['run'](node)
            self.assertFalse(out.exists())


if __name__=='__main__': unittest.main()

