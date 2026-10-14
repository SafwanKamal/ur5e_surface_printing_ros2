"""Locally retime joint splines for a steady Cartesian deposition speed.

Coverage is measured by traveled TCP distance, not waypoint count or elapsed time.
Uses cubic interpolation and preserves scaled joint velocity/acceleration limits.
"""
import math
import numpy as np
from .tcp_speed import TCPModel, coefficients, seconds

TOLERANCE = .05
REFERENCE_COVERAGE = .90


def segment_report(a, b, model, names, limits=None, wrist_bounds=None, samples=101):
    dt = seconds(b) - seconds(a)
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError('Non-increasing trajectory time')
    c = coefficients(a,b,dt)
    if len(c) != 4:
        raise ValueError('Constant-speed timing requires cubic joint interpolation')
    factor = 1.
    if limits is not None:
        for j,name in enumerate(names):
            vmax,amax = limits[name]
            if not all(math.isfinite(v) and v>0 for v in (vmax,amax)):
                raise ValueError('Invalid joint limits')
            candidates = [0.,1.]
            if c[3,j] and 0 < -c[2,j]/(3*c[3,j]) < 1:
                candidates.append(-c[2,j]/(3*c[3,j]))
            v = max(abs((c[1,j]+2*c[2,j]*u+3*c[3,j]*u*u)/dt)
                    for u in candidates)
            acc = max(abs(2*c[2,j]/dt**2),
                      abs((2*c[2,j]+6*c[3,j])/dt**2))
            factor = max(factor,v/vmax,math.sqrt(acc/amax))
    u=np.linspace(0,1,samples)
    q=np.stack([u**k for k in range(4)],axis=1)@c
    dq=np.stack([np.zeros_like(u),np.ones_like(u),2*u,3*u*u],axis=1)@c/dt
    if wrist_bounds is not None:
        wi=names.index('wrist_3_joint')
        if np.any(q[:,wi]<wrist_bounds[0]-1e-10) or np.any(q[:,wi]>wrist_bounds[1]+1e-10):
            raise ValueError('Interpolated Wrist 3 exceeds allowed bounds')
    speeds=model.speeds(q,dq)*1000.
    if not np.isfinite(speeds).all(): raise ValueError('Nonfinite TCP speed')
    mids=(speeds[:-1]+speeds[1:])/2
    weights=mids*dt/(samples-1)
    return float(max(speeds)),mids,weights,factor


def validate_targets(maximum, average):
    if not all(math.isfinite(v) and .1<=v<=10 for v in (maximum,average)):
        raise ValueError('Maximum and average Cartesian speeds must be 0.1..10 mm/s')
    if average > maximum:
        raise ValueError('average_cartesian_speed_mm_s cannot exceed max_cartesian_speed_mm_s')


def verify_constant(trajectory,xml,tip,maximum,average,wrist_bounds=None,limits=None):
    validate_targets(maximum,average)
    jt=trajectory.joint_trajectory
    if len(jt.points)<2: raise ValueError('Too few trajectory points')
    names=list(jt.joint_names)
    model=TCPModel(xml,tip,names)
    peak=length=matched=weighted_speed=0.
    for a,b in zip(jt.points,jt.points[1:]):
        p,v,w,factor=segment_report(a,b,model,names,limits,wrist_bounds)
        peak=max(peak,p); length+=float(sum(w))
        matched+=float(sum(w[(v>=average*(1-TOLERANCE)) & (v<=average*(1+TOLERANCE))]))
        weighted_speed+=float(sum(v*w))
        if factor>1+1e-6: raise ValueError('Constant-speed plan exceeds scaled joint limits')
    if not length>0: raise ValueError('No TCP travel')
    duration=seconds(jt.points[-1])-seconds(jt.points[0])
    report=dict(method='local_cartesian_cubic_v1',after_peak_mm_s=peak,
                target_mm_s=average,max_mm_s=maximum,tolerance_fraction=TOLERANCE,
                coverage_reference_fraction=REFERENCE_COVERAGE,coverage_enforced=False,
                coverage_fraction=matched/length,
                path_length_mm=length,overall_average_mm_s=length/duration,
                distance_weighted_speed_mm_s=weighted_speed/length)
    if peak>maximum*(1+1e-6):
        raise ValueError(f'Sampled TCP peak {peak:.4f} exceeds maximum {maximum:.4f}')
    return report


def retime_constant(trajectory,xml,tip,maximum,average,limits,wrist_bounds=None):
    validate_targets(maximum,average)
    jt=trajectory.joint_trajectory
    names=list(jt.joint_names)
    if len(set(names))!=len(names): raise ValueError('Duplicate joints')
    model=TCPModel(xml,tip,names)
    kept=[]
    for point in jt.points:
        if len(point.positions)!=len(names) or not np.isfinite(point.positions).all():
            raise ValueError('Invalid trajectory positions')
        if kept and np.max(np.abs(np.array(point.positions)-kept[-1].positions))<1e-12:
            continue
        kept.append(point)
    jt.points=kept
    if len(kept)<3:
        raise ValueError('Use a finely sampled path for constant-speed retiming')
    q=np.array([p.positions for p in kept])
    xyz=np.array([model.position(p) for p in q])
    ds=np.linalg.norm(np.diff(xyz,axis=0),axis=1)*1000
    if np.any(ds<1e-9): raise ValueError('Zero TCP displacement with changing joints')
    dt=ds/average
    stops=np.zeros(len(kept),dtype=bool); stops[0]=stops[-1]=True
    vectors=np.diff(xyz,axis=0)
    for i in range(1,len(kept)-1):
        cos=float(np.dot(vectors[i-1],vectors[i])/
                  (np.linalg.norm(vectors[i-1])*np.linalg.norm(vectors[i])))
        if cos<.5: stops[i]=True

    def rebuild():
        slopes=np.diff(q,axis=0)/dt[:,None]
        velocities=np.zeros_like(q)
        for i in range(1,len(kept)-1):
            if stops[i]: continue
            left,right=slopes[i-1],slopes[i]
            mask=left*right>0
            w1,w2=2*dt[i]+dt[i-1],dt[i]+2*dt[i-1]
            velocities[i,mask]=(w1+w2)/(w1/left[mask]+w2/right[mask])
        times=np.concatenate(([0.],np.cumsum(dt)))
        for p,t,v in zip(kept,times,velocities):
            p.time_from_start.sec,p.time_from_start.nanosec=divmod(round(t*1e9),1000000000)
            p.velocities=v.tolist(); p.accelerations=[]; p.effort=[]

    # Slow only offending intervals, rather than stretching every interval.
    for iteration in range(80):
        rebuild()
        factors=[]
        for a,b in zip(kept,kept[1:]):
            peak,_,_,joint_factor=segment_report(a,b,model,names,limits,wrist_bounds,samples=21)
            factor=max(joint_factor,peak/(maximum*.999))
            factors.append(factor*1.002 if factor>1+1e-7 else 1.)
        if max(factors)<=1:
            # Repeat with the same dense sampling used by execution validation.
            dense=[]
            for a,b in zip(kept,kept[1:]):
                peak,_,_,jf=segment_report(a,b,model,names,limits,wrist_bounds)
                f=max(jf,peak/(maximum*.999))
                dense.append(f*1.002 if f>1+1e-7 else 1.)
            if max(dense)<=1:
                report=verify_constant(trajectory,xml,tip,maximum,average,wrist_bounds,limits)
                report['retiming_iterations']=iteration+1
                report['stop_knots']=int(sum(stops))
                return report
            factors=dense
        dt*=np.array(factors)
    raise ValueError('Local Cartesian timing did not converge; no plan saved')
