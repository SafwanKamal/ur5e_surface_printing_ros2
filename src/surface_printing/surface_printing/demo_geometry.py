"""Pure geometry and limits, usable without ROS."""
import math


def demo_offsets(shape, size_mm, spacing_mm=1.0):
    if not math.isfinite(size_mm) or not 5 <= size_mm <= 100:
        raise ValueError('size_mm must be 5..100')
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


def validate_budget(duration, flow, max_seconds, max_volume):
    if not all(math.isfinite(v) and v > 0 for v in (duration, flow, max_seconds, max_volume)):
        raise ValueError('Duration, flow and limits must be finite and positive')
    if not 0 < flow <= 5 or not max_seconds <= 100 or not max_volume <= 5:
        raise ValueError('Demo limits: <=5 mL/min, <=100 seconds, <=5 mL')
    if duration > max_seconds or flow * duration / 60 > max_volume:
        raise ValueError('Trajectory exceeds configured time or volume budget')
