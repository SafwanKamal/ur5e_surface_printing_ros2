"""Geometry for a straight lead-in followed by a compensated planar circle."""
import math
from .demo_geometry import demo_offsets, repeat_offsets


def lead_circle_offsets(lead_in_mm, inner_diameter_mm, bead_width_mm,
                        spacing_mm=0.1, direction=-1.0, passes=1):
    """Return XY offsets in meters, beginning at the captured nozzle pose.

    Inner diameter compensation assumes a symmetric bead of measured width.
    The line and circle extend along direction * X, at a fixed height.
    """
    values = (lead_in_mm, inner_diameter_mm, bead_width_mm, spacing_mm, direction)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Geometry parameters must be finite')
    if not 0 <= lead_in_mm <= 100:
        raise ValueError('lead_in_mm must be 0..100')
    if inner_diameter_mm <= 0 or bead_width_mm < 0:
        raise ValueError('inner_diameter_mm must be positive; bead_width_mm nonnegative')
    if direction not in (-1.0, 1.0):
        raise ValueError('direction must be -1 or +1')
    diameter = inner_diameter_mm + bead_width_mm
    circle = repeat_offsets(demo_offsets('circle', diameter, spacing_mm), passes)
    length = lead_in_mm / 1000.0
    count = max(1, math.ceil(lead_in_mm / spacing_mm))
    lead = [(direction * length * i / count, 0.0) for i in range(count + 1)]
    if length == 0:
        lead = [(0.0, 0.0)]
    return lead + [(direction * (length + x), direction * y) for x, y in circle[1:]]
