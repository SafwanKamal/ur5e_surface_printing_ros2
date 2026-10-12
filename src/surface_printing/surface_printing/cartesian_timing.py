"""Join independently timed Cartesian legs at stationary sharp turns."""
import copy
import math


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
