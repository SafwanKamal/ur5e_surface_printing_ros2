"""Measured TCP speed from timestamped joint-state positions.

Finite-difference windows are measurement estimates, not continuous-time proof.
"""
import math
import numpy as np
from .constant_speed import TOLERANCE, REFERENCE_COVERAGE


class SpeedTracker:
    def __init__(self,model,names,maximum,target,planned_length_mm,window_sec=.25):
        self.model=model; self.names=list(names)
        self.maximum=maximum; self.target=target
        self.planned_length=planned_length_mm; self.window=window_sec
        self.anchor=None; self.last_stamp=None
        self.length=0.; self.matched=0.; self.peak=0.; self.elapsed=0.
        self.last_speed=0.; self.windows=0

    def update(self,joints):
        current=dict(zip(joints.name,joints.position))
        if any(n not in current or not math.isfinite(current[n]) for n in self.names):
            raise RuntimeError('Invalid joints for measured TCP speed')
        stamp=joints.header.stamp.sec+joints.header.stamp.nanosec*1e-9
        if not math.isfinite(stamp) or stamp<=0:
            raise RuntimeError('Invalid joint timestamp for measured TCP speed')
        if self.last_stamp is not None:
            if stamp<self.last_stamp:
                raise RuntimeError('Joint clock moved backward during speed measurement')
            if stamp==self.last_stamp: return
        self.last_stamp=stamp
        position=self.model.position([current[n] for n in self.names])
        if self.anchor is None:
            self.anchor=(stamp,position);return
        t0,p0=self.anchor
        dt=stamp-t0
        if dt<self.window:return
        distance=float(np.linalg.norm(position-p0))*1000.
        self.anchor=(stamp,position)
        speed=distance/dt
        self.length+=distance;self.elapsed+=dt;self.windows+=1
        self.last_speed=speed;self.peak=max(self.peak,speed)
        if self.target*(1-TOLERANCE)<=speed<=self.target*(1+TOLERANCE):
            self.matched+=distance
        # Measured speed is telemetry only; never abort on speed or coverage.

    def report(self):
        return dict(measured_distance_mm=self.length,
                    measured_peak_mm_s=self.peak,
                    measured_average_mm_s=self.length/self.elapsed if self.elapsed else 0.,
                    measured_coverage=self.matched/self.length if self.length else 0.,
                    measurement_windows=self.windows,
                    measured_max_exceeded=self.peak>self.maximum*(1+TOLERANCE),
                    measured_target_met=(self.matched/self.length>=REFERENCE_COVERAGE)
                    if self.length else False)

