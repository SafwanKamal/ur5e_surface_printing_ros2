"""Pure geometry and limits, usable without ROS."""
import math


def demo_offsets(shape, size_mm, spacing_mm=1.0):
    if not math.isfinite(size_mm) or not 0.1 <= size_mm <= 100:
        raise ValueError('size_mm must be 0.1..100')
    if not math.isfinite(spacing_mm) or not 0.1 <= spacing_mm <= 2:
        raise ValueError('spacing_mm must be 0.1..2')
    size = size_mm / 1000.0
    if shape == 'circle':
        n = max(24, math.ceil(math.pi * size_mm / spacing_mm))
        # Current nozzle pose is the leftmost point, not the center.
        return [(size/2*(1-math.cos(2*math.pi*i/n)),
                 size/2*math.sin(2*math.pi*i/n)) for i in range(n+1)]
    if shape == 'line':
        corners = [(0., 0.), (size, 0.)]
    elif shape == 'square':
        corners = [(0., 0.), (size, 0.), (size, size), (0., size), (0., 0.)]
    else:
        raise ValueError('shape must be line, square or circle')
    out = [corners[0]]
    for a, b in zip(corners, corners[1:]):
        n = math.ceil(math.dist(a, b)*1000/spacing_mm)
        out.extend([(a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n)
                    for i in range(1, n+1)])
    return out


def repeat_offsets(offsets, passes):
    """Repeat a closed contour, or alternate direction over an open segment."""
    if isinstance(passes, bool) or not isinstance(passes, int) or not 1 <= passes <= 100:
        raise ValueError('passes must be an integer in 1..100')
    if len(offsets) < 2:
        raise ValueError('Repeated path needs at least two points')
    points = list(offsets)
    closed = math.dist(points[0], points[-1]) <= 1e-12
    if closed:
        points[-1] = points[0]
    result = list(points)
    for index in range(1, passes):
        traversal = points if closed or index % 2 == 0 else list(reversed(points))
        result.extend(traversal[1:])
    return result


def validate_budget(duration, flow, max_seconds, max_volume):
    if not all(math.isfinite(v) and v > 0 for v in (duration, flow, max_seconds, max_volume)):
        raise ValueError('Duration, flow and limits must be finite and positive')
    if not 0 < flow <= 5 or not max_seconds <= 100 or not max_volume <= 5:
        raise ValueError('Demo limits: <=5 mL/min, <=100 seconds, <=5 mL')
    if duration > max_seconds or flow * duration / 60 > max_volume:
        raise ValueError('Trajectory exceeds configured time or volume budget')


def validate_pulse_budget(duration, rate_hz, max_seconds, max_pulses):
    """Conservative pulse bound: assumes full target rate throughout the run."""
    if not all(math.isfinite(v) and v > 0 for v in (duration, rate_hz, max_seconds)):
        raise ValueError('Duration, frequency and pulse limits must be finite and positive')
    if rate_hz != int(rate_hz) or not 1 <= rate_hz <= 5000:
        raise ValueError('step_rate_hz must be a whole number in 1..5000')
    if not math.isfinite(max_pulses) or max_pulses < 0 or max_pulses != int(max_pulses):
        raise ValueError('Pulse budget must be a nonnegative integer; 0 disables the count cap')
    if max_seconds > 100:
        raise ValueError('Run-time limit: <=100 seconds')
    if duration > max_seconds or (max_pulses > 0 and math.ceil(duration * rate_hz) > max_pulses):
        raise ValueError('Trajectory exceeds configured time or pulse budget')
