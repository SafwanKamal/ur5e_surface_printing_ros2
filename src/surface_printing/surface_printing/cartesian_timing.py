"""Join independently timed Cartesian legs at stationary sharp turns."""
import copy
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def configured_limits(xml, names, velocity_scale, acceleration_scale):
    """Scaled limits from live URDF and this workspace's MoveIt configuration."""
    import yaml
    from ament_index_python.packages import get_package_share_directory
    config = yaml.safe_load((Path(get_package_share_directory('ur5e_probe_moveit_config')) /
                            'config/joint_limits.yaml').read_text())['joint_limits']
    root = ET.fromstring(xml)
    limits = {}
    for name in names:
        joint = root.find(f"joint[@name='{name}']/limit")
        entry = config[name]
        if joint is None or not entry.get('has_acceleration_limits'):
            raise ValueError(f'Missing velocity/acceleration limit for {name}')
        velocity = float(joint.get('velocity'))
        if entry.get('has_velocity_limits'):
            velocity = min(velocity, float(entry['max_velocity']))
        limits[name] = (velocity*velocity_scale,
                        float(entry['max_acceleration'])*acceleration_scale)
    return limits


def joint_peaks(trajectory):
    """Exact velocity/acceleration extrema for each cubic Hermite segment."""
    names = trajectory.joint_trajectory.joint_names
    peaks = {name: [0., 0.] for name in names}
    for a,b in zip(trajectory.joint_trajectory.points, trajectory.joint_trajectory.points[1:]):
        dt = (nanoseconds(b)-nanoseconds(a))/1e9
        if dt <= 0:
            raise ValueError('Non-increasing leg time')
        for j,name in enumerate(names):
            delta = b.positions[j]-a.positions[j]
            v0,v1 = a.velocities[j]*dt,b.velocities[j]*dt
            c2,c3 = 3*delta-2*v0-v1, -2*delta+v0+v1
            samples = [0.,1.]
            if c3 and 0 < -c2/(3*c3) < 1:
                samples.append(-c2/(3*c3))
            peaks[name][0] = max(peaks[name][0], *(abs((v0+2*c2*u+3*c3*u*u)/dt) for u in samples))
            peaks[name][1] = max(peaks[name][1], abs(2*c2/dt**2), abs((2*c2+6*c3)/dt**2))
    return peaks


def stop_ended_timing(trajectory, limits):
    """Rest endpoints and monotone cubic joint interpolation; slow to limits.

    Interior velocities use weighted harmonic slopes, avoiding joint-position
    overshoot between waypoints. Empty acceleration fields select cubic spline
    interpolation; actual cubic acceleration is bounded analytically below.
    """
    if not strictly_timed(trajectory):
        raise ValueError('Cannot retime a leg with non-increasing times')
    points = trajectory.joint_trajectory.points
    names = trajectory.joint_trajectory.joint_names
    n = len(names)
    if any(len(p.positions)!=n or not all(math.isfinite(q) for q in p.positions) for p in points):
        raise ValueError('Invalid leg positions')
    intervals = [(nanoseconds(b)-nanoseconds(a))/1e9 for a,b in zip(points,points[1:])]
    slopes = [[(b.positions[j]-a.positions[j])/dt for j in range(n)]
              for a,b,dt in zip(points,points[1:],intervals)]
    for p in points:
        p.velocities = [0.] * n
        p.accelerations = []
    for i in range(1,len(points)-1):
        before,after = intervals[i-1],intervals[i]
        w1,w2 = 2*after+before,after+2*before
        for j in range(n):
            left,right = slopes[i-1][j],slopes[i][j]
            if left*right>0:
                points[i].velocities[j] = (w1+w2)/(w1/left+w2/right)
    factor = 1.
    for name,(velocity,acceleration) in joint_peaks(trajectory).items():
        vmax,amax = limits[name]
        if not all(math.isfinite(v) and v>0 for v in (vmax,amax)):
            raise ValueError(f'Invalid scaled limits for {name}')
        factor = max(factor, velocity/vmax, math.sqrt(acceleration/amax))
    factor *= 1.01
    for p in points:
        p.time_from_start.sec,p.time_from_start.nanosec = divmod(round(nanoseconds(p)*factor),1000000000)
        p.velocities = [v/factor for v in p.velocities]
    if not strictly_timed(trajectory):
        raise ValueError('Rest-endpoint retiming produced invalid time')
    for name,(velocity,acceleration) in joint_peaks(trajectory).items():
        vmax,amax = limits[name]
        if velocity>vmax*(1+1e-6) or acceleration>amax*(1+1e-6):
            raise ValueError(f'Retimed joint limit exceeded: {name}')
    return factor


def nanoseconds(point):
    return point.time_from_start.sec * 1000000000 + point.time_from_start.nanosec


def strictly_timed(trajectory):
    points = trajectory.joint_trajectory.points
    return len(points) >= 2 and nanoseconds(points[0]) >= 0 and all(
        nanoseconds(b) > nanoseconds(a) for a, b in zip(points, points[1:]))


def remove_identical_time_duplicates(trajectory):
    """Remove only identical states at equal times, never retimestamp motion."""
    points = trajectory.joint_trajectory.points
    kept = []
    for point in points:
        if kept and nanoseconds(point) == nanoseconds(kept[-1]) and all(
                list(getattr(point, field)) == list(getattr(kept[-1], field))
                for field in ('positions', 'velocities', 'accelerations', 'effort')):
            continue
        kept.append(point)
    trajectory.joint_trajectory.points = kept
    return len(points)-len(kept)


def split_legs(poses):
    """Sharp turns >=60 degrees become separate legs; circles stay continuous."""
    cuts = [0]
    for i in range(1, len(poses)-1):
        a = [getattr(poses[i].position, axis)-getattr(poses[i-1].position, axis) for axis in 'xyz']
        b = [getattr(poses[i+1].position, axis)-getattr(poses[i].position, axis) for axis in 'xyz']
        lengths = math.sqrt(sum(x*x for x in a)*sum(x*x for x in b))
        if lengths > 1e-20 and sum(x*y for x,y in zip(a,b))/lengths < .5:
            cuts.append(i)
    cuts.append(len(poses)-1)
    return [poses[a:b+1] for a,b in zip(cuts,cuts[1:])]


def join_legs(trajectories):
    """Preserve joint positions and durations; stop at each shared endpoint."""
    if not trajectories:
        raise ValueError('No Cartesian legs')
    result = copy.deepcopy(trajectories[0])
    names = list(result.joint_trajectory.joint_names)
    if any(t.multi_dof_joint_trajectory.points for t in trajectories):
        raise ValueError('Multi-DOF leg stitching is unsupported')
    result.joint_trajectory.points = []
    for trajectory in trajectories:
        if list(trajectory.joint_trajectory.joint_names) != names or not strictly_timed(trajectory):
            raise ValueError('Cartesian leg has invalid names or non-increasing time')
        points = copy.deepcopy(trajectory.joint_trajectory.points)
        for endpoint in (points[0], points[-1]):
            if len(endpoint.velocities) != len(names) or any(
                    not math.isfinite(v) or abs(v)>1e-7 for v in endpoint.velocities):
                raise ValueError('Cartesian leg endpoint must be stationary')
            endpoint.velocities = [0.] * len(names)
            if endpoint.accelerations:
                if len(endpoint.accelerations) != len(names):
                    raise ValueError('Invalid endpoint accelerations')
                endpoint.accelerations = [0.] * len(names)
        first_time = nanoseconds(points[0])
        if result.joint_trajectory.points:
            last = result.joint_trajectory.points[-1]
            if len(last.positions) != len(points[0].positions) or any(
                    not math.isfinite(a) or not math.isfinite(b) or abs(a-b)>1e-7
                    for a,b in zip(last.positions,points[0].positions)):
                raise ValueError('Cartesian legs do not share the same joint endpoint')
            shift = nanoseconds(last)-first_time
            points = points[1:]
        else:
            shift = -first_time
        for point in points:
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(
                nanoseconds(point)+shift, 1000000000)
        result.joint_trajectory.points.extend(points)
    if not strictly_timed(result):
        raise ValueError('Joined Cartesian trajectory has invalid timing')
    return result
