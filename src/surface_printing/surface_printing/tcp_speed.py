"""Sample spline TCP speeds from live URDF; uniformly slow trajectories only.

Assumes joint_trajectory_controller spline interpolation (linear/cubic/quintic
according to populated derivatives). Samples are not a continuous-time proof.
"""
import math
import xml.etree.ElementTree as ET
import numpy as np


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid URDF joint axis')
    x, y, z = axis / norm
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*k + (1-math.cos(angle))*(k@k)


class TCPModel:
    def __init__(self, xml, tip, names):
        root = ET.fromstring(xml)
        if root.find(f"link[@name='{tip}']") is None:
            raise ValueError('TCP link missing from live URDF')
        parents = {j.find('child').get('link'): j for j in root.findall('joint')}
        self.chain = []
        seen = set()
        while tip in parents:
            if tip in seen: raise ValueError('Cyclic URDF')
            seen.add(tip)
            j = parents[tip]
            kind = j.get('type')
            if kind not in ('fixed', 'revolute', 'continuous', 'prismatic'):
                raise ValueError('Unsupported joint type for TCP speed check')
            origin = j.find('origin')
            xyz = np.zeros(3); rpy = [0., 0., 0.]
            if origin is not None:
                xyz = np.array(list(map(float, origin.get('xyz','0 0 0').split())))
                rpy = list(map(float, origin.get('rpy','0 0 0').split()))
            rot = rotation([0,0,1],rpy[2])@rotation([0,1,0],rpy[1])@rotation([1,0,0],rpy[0])
            index = None; axis = np.array([1.,0.,0.])
            if kind != 'fixed':
                if j.find('mimic') is not None or j.get('name') not in names:
                    raise ValueError('Unsupported mimic or missing trajectory joint')
                index = names.index(j.get('name'))
                if j.find('axis') is not None:
                    axis = np.array(list(map(float,j.find('axis').get('xyz').split())))
                axis = axis / np.linalg.norm(axis)
            self.chain.append((xyz,rot,kind,index,axis))
            tip = j.find('parent').get('link')
        self.chain.reverse()

    def speed(self, q, dq):
        pos = np.zeros(3); rot = np.eye(3); axes = []
        for xyz, origin_rot, kind, index, axis in self.chain:
            pos = pos + rot@xyz; rot = rot@origin_rot
            if index is not None:
                world_axis = rot@axis
                axes.append((pos.copy(), world_axis, kind, index))
                if kind == 'prismatic': pos = pos + world_axis*q[index]
                else: rot = rot@rotation(axis,q[index])
        velocity = np.zeros(3)
        for center, axis, kind, index in axes:
            velocity += dq[index]*(axis if kind=='prismatic' else np.cross(axis,pos-center))
        return float(np.linalg.norm(velocity))


def seconds(point):
    return point.time_from_start.sec + point.time_from_start.nanosec*1e-9


def coefficients(a, b, dt):
    q0, q1 = np.asarray(a.positions), np.asarray(b.positions)
    n = len(q0)
    if len(q1)!=n: raise ValueError('Inconsistent positions')
    v = len(a.velocities)==n and len(b.velocities)==n
    acc = len(a.accelerations)==n and len(b.accelerations)==n
    if acc and not v: raise ValueError('Acceleration requires velocity')
    if any(len(x) not in (0,n) for x in (a.velocities,b.velocities,a.accelerations,b.accelerations)):
        raise ValueError('Invalid derivative sizes')
    if bool(a.velocities)!=bool(b.velocities) or bool(a.accelerations)!=bool(b.accelerations):
        raise ValueError('Inconsistent derivative fields')
    if not v: c = np.array([q0, q1-q0])
    elif not acc:
        v0=np.asarray(a.velocities)*dt; v1=np.asarray(b.velocities)*dt; delta=q1-q0
        c=np.array([q0,v0,3*delta-2*v0-v1,-2*delta+v0+v1])
    else:
        v0=np.asarray(a.velocities)*dt; v1=np.asarray(b.velocities)*dt
        a0=np.asarray(a.accelerations)*dt*dt; a1=np.asarray(b.accelerations)*dt*dt
        delta=q1-q0
        c=np.array([q0,v0,a0/2,10*delta-6*v0-4*v1-1.5*a0+.5*a1,
                    -15*delta+8*v0+7*v1+1.5*a0-a1,6*delta-3*v0-3*v1-.5*a0+.5*a1])
    if not np.isfinite(c).all(): raise ValueError('Nonfinite trajectory')
    return c


def peak_speed(trajectory, xml, tip, wrist_bounds=None):
    jt=trajectory.joint_trajectory
    if len(jt.points)<2 or len(set(jt.joint_names))!=len(jt.joint_names):
        raise ValueError('Invalid trajectory for speed validation')
    model=TCPModel(xml,tip,list(jt.joint_names)); peak=0.; count=0
    wi=jt.joint_names.index('wrist_3_joint') if wrist_bounds is not None else None
    for a,b in zip(jt.points,jt.points[1:]):
        dt=seconds(b)-seconds(a)
        if not math.isfinite(dt) or dt<=0: raise ValueError('Non-increasing trajectory time')
        c=coefficients(a,b,dt)
        if c.shape[1]!=len(jt.joint_names): raise ValueError('Joint count mismatch')
        # Fixed normalized sampling means uniform retiming preserves validation points.
        for u in np.linspace(0,1,101):
            q=sum(c[k]*u**k for k in range(len(c)))
            dq=sum(k*c[k]*u**(k-1) for k in range(1,len(c)))/dt
            if wi is not None and not wrist_bounds[0]-1e-10<=q[wi]<=wrist_bounds[1]+1e-10:
                raise ValueError('Interpolated Wrist 3 exceeds allowed bounds')
            speed=model.speed(q,dq)
            if not math.isfinite(speed): raise ValueError('Nonfinite TCP speed')
            peak=max(peak,speed); count+=1
    return peak,count


def retime(trajectory, xml, tip, limit_mm_s, wrist_bounds=None):
    limit=float(limit_mm_s)/1000
    if not math.isfinite(limit) or not 0.0001<=limit<=0.01:
        raise ValueError('TCP limit must be 0.1..10 mm/s')
    peak,count=peak_speed(trajectory,xml,tip,wrist_bounds)
    factor=max(1.,peak/(limit*.95))
    for point in trajectory.joint_trajectory.points:
        ns=round(seconds(point)*factor*1e9)
        point.time_from_start.sec,point.time_from_start.nanosec=divmod(ns,1000000000)
        point.velocities=[v/factor for v in point.velocities]
        point.accelerations=[a/(factor*factor) for a in point.accelerations]
    after,_=peak_speed(trajectory,xml,tip,wrist_bounds)
    if after>limit*(1+1e-6): raise ValueError('Retimed speed check failed')
    return dict(method='urdf_spline_sampling_v1',samples=count,time_scale=factor,
                before_peak_mm_s=peak*1000,after_peak_mm_s=after*1000)


def verify(trajectory,xml,tip,limit_mm_s,wrist_bounds=None):
    limit=float(limit_mm_s)
    if not math.isfinite(limit) or not .1<=limit<=10: raise ValueError('Invalid saved speed limit')
    peak,_=peak_speed(trajectory,xml,tip,wrist_bounds)
    if peak*1000>limit*(1+1e-6):
        raise ValueError(f'TCP speed {peak*1000:.3f} mm/s exceeds {limit}; replan with speed retiming')
