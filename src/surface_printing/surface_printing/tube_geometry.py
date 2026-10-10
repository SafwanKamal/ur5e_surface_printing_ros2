"""Pure geometry for a nominal-diameter, fixed-orientation helical tube path."""
import math


def tube_points(start, diameter_mm, height_mm, pitch_mm, step_mm):
    """Start is on the +X rim; center is start minus radius along world X.

    Diameter is the TCP centerline diameter, not a guaranteed finished lumen.
    Height is exact; the last revolution can be partial.
    """
    values = [*start, diameter_mm, height_mm, pitch_mm, step_mm]
    if len(start) != 3 or not all(math.isfinite(float(v)) for v in values):
        raise ValueError('Tube inputs must be finite')
    if not (0 < diameter_mm <= 100 and 0 < height_mm <= 100
            and 0 < pitch_mm <= height_mm and 0 < step_mm <= 1):
        raise ValueError('Require diameter/height in (0,100] mm, '
                         'pitch in (0,height] mm, and step in (0,1] mm')
    radius = diameter_mm / 2000.0
    angle = 2 * math.pi * height_mm / pitch_mm
    length_mm = math.hypot(math.pi * diameter_mm * height_mm / pitch_mm, height_mm)
    count = max(1, math.ceil(length_mm / step_mm))
    if count > 100000:
        raise ValueError('Tube path exceeds 100000 intervals; increase pitch or step')
    points = []
    for i in range(count + 1):
        fraction = i / count
        theta = angle * fraction
        points.append((start[0] + radius * (math.cos(theta) - 1),
                       start[1] + radius * math.sin(theta),
                       start[2] + height_mm * fraction / 1000.0))
    points[0] = tuple(start)
    return points


def normalized_orientation(values):
    if len(values) != 4 or not all(math.isfinite(v) for v in values):
        raise ValueError('Invalid captured orientation')
    norm = math.sqrt(sum(v*v for v in values))
    if norm < 1e-9:
        raise ValueError('Zero quaternion')
    return tuple(v / norm for v in values)
