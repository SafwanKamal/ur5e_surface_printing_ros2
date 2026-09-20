"""Fault injection tests; run with python -m unittest discover -s tests."""
import queue
import sys
import threading
import time
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/syringe_controller'))
from syringe_controller.serial_backend import SerialBackend
from syringe_controller.calibration import Calibration


class Port:
    def __init__(self):
        self.rx=queue.Queue(); self.commands=[]; self.reply=True; self.drop_start=False
    def write(self,data):
        cmd=data.decode().strip(); self.commands.append(cmd)
        if self.reply:
            if cmd=='PING': self.rx.put(b'PONG\n')
            elif not(self.drop_start and cmd.startswith('START')):
                self.rx.put(('ACK '+cmd.split()[0]+'\n').encode())
        return len(data)
    def read(self,n):
        try: return self.rx.get(timeout=.01)
        except queue.Empty: return b''
    def close(self): pass


class SerialSafety(unittest.TestCase):
    def setUp(self):
        self.port=Port(); self.backend=SerialBackend(self.port,ack_timeout=.08,heartbeat_period=.02)
    def tearDown(self): self.backend.close()
    def test_exclusive_reservation(self):
        self.assertTrue(self.backend.reserve('first'))
        self.assertFalse(self.backend.reserve('second'))
        self.assertFalse(self.backend.start('START -1 9','second'))
    def test_stop_prevents_delayed_start(self):
        self.assertTrue(self.backend.reserve('action'))
        self.assertTrue(self.backend.stop())
        self.assertFalse(self.backend.start('MOVE -10 9','action'))
    def test_lost_start_ack_stops_and_latches(self):
        self.port.drop_start=True; self.backend.reserve('action')
        self.assertFalse(self.backend.start('START -1 9','action'))
        self.assertIn('STOP',self.port.commands)
        self.assertTrue(self.backend.snapshot()['fault'])
    def test_expired_flow_lease_stops(self):
        owner=('flow',object()); self.backend.reserve(owner,lease_sec=.04)
        self.assertTrue(self.backend.start('START -1 9',owner))
        time.sleep(.12)
        self.assertFalse(self.backend.snapshot()['moving'])
        self.assertIn('STOP',self.port.commands)
    def test_firmware_reset_latches(self):
        self.backend._line('READY SYRINGE_DEMO_V1')
        self.assertFalse(self.backend.reserve('action'))
    def test_feedback_loss_stops(self):
        self.backend.reserve('action'); self.backend.start('START -1 9','action')
        self.port.reply=False; self.backend.last_rx=time.monotonic()-2
        time.sleep(.14)
        self.assertIn('STOP',self.port.commands)
        self.assertTrue(self.backend.snapshot()['fault'])
    def test_calibration_rounding_and_direction(self):
        cal=Calibration()
        self.assertEqual(cal.volume(2),-1111)
        self.assertEqual(cal.flow(1)[0],9)
        for bad in (0,-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError): cal.flow(bad)
