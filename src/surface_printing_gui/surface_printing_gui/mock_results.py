"""ROS-independent path inspection and mock telemetry state."""
import csv
import json
import math
from pathlib import Path
import time

from .mock_monitor import PREFIX


def read_path(filename, line_id):
    points = []
    with Path(filename).expanduser().open(newline='') as handle:
        for row in csv.DictReader(handle):
            if int(row['line_id']) != int(line_id):
                continue
            point = tuple(float(row[k]) for k in ('x', 'y', 'z'))
            if not all(math.isfinite(v) for v in point):
                raise ValueError('Path coordinates must be finite')
            points.append((int(row['point_index']), point))
    points.sort(key=lambda item: item[0])
    if len(points) < 2:
        raise ValueError('Selected CSV line needs at least two points')
    if len({index for index, _ in points}) != len(points):
        raise ValueError('Duplicate point index')
    return [point for _, point in points]


def path_length_mm(points):
    return sum(math.dist(a, b) for a, b in zip(points, points[1:])) * 1000


class MockFeedback:
    def __init__(self):
        self.received = None
        self.data = {}
        self.trace = []
        self.recording = False
        self.samples = 0

    def accept(self, line, now=None):
        if not line.startswith(PREFIX):
            return False
        data = json.loads(line[len(PREFIX):])
        if not isinstance(data, dict) or not isinstance(data.get('joints'), dict):
            raise ValueError('Malformed mock telemetry')
        for value in data['joints'].values():
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('Invalid joint telemetry')
        tcp = data.get('tcp')
        if tcp is not None and (not isinstance(tcp, list) or len(tcp) != 3 or
                                not all(isinstance(v, (int, float)) and math.isfinite(v) for v in tcp)):
            raise ValueError('Invalid TCP telemetry')
        for key in ('joint_age', 'tf_age'):
            value = data.get(key)
            if value is not None and (not isinstance(value, (int, float)) or
                                      not math.isfinite(value) or value < 0):
                raise ValueError('Invalid telemetry age')
        self.received = time.monotonic() if now is None else now
        self.data = data
        if self.recording and self.fresh(self.received):
            self.samples += 1
            if not self.trace or math.dist(tcp, self.trace[-1]) > 0.00001:
                self.trace.append(tuple(tcp))
                self.trace = self.trace[-5000:]
        return True

    def fresh(self, now=None):
        return not self.blockers(now)

    def blockers(self, now=None):
        now = time.monotonic() if now is None else now
        if self.received is None:
            return ['Connect live mock state: no monitor samples received']
        reasons = []
        if now - self.received > 1.0:
            reasons.append('Live monitor stopped or its output is stale')
        if not self.data.get('joints'):
            reasons.append('No /joint_states received')
        age = self.data.get('joint_age')
        if age is None:
            reasons.append('Joint timestamp missing or incompatible with ROS clock')
        elif age > .5:
            reasons.append(f'Joint feedback stale ({age:.2f}s)')
        if self.data.get('tcp') is None:
            reasons.append('No world -> probe_tcp transform available')
        age = self.data.get('tf_age')
        if age is None:
            reasons.append('TCP transform timestamp unavailable')
        elif age > .5:
            reasons.append(f'TCP transform stale ({age:.2f}s)')
        return reasons

    def start(self):
        self.trace = []
        self.samples = 0
        self.recording = True


def rehearsal_blockers(*, same_session, busy, isolated, confirmed, feedback, gate, signature):
    reasons = []
    if not same_session:reasons.append('Restore the original mock/domain 78 session')
    if busy:reasons.append('Wait for the current workflow step to finish')
    if not isolated:reasons.append('Confirm physical drivers stopped and pump power off')
    if signature is None or signature != gate.planned:
        reasons.append('Create a successful plan for the current files/settings')
    elif not gate.ready(signature):
        reasons.append('Saved-plan check has not passed; inspect the check output')
    if not confirmed:reasons.append('Tick the path/RViz review checkbox')
    reasons.extend(feedback.blockers())
    return reasons
