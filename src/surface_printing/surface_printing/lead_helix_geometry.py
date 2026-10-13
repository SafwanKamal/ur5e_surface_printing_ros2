"""Continuous straight lead-in plus rising helix, in world meters."""
import math
from .tube_geometry import tube_points


def lead_helix_points(start, lead_in_mm, inner_diameter_mm, bead_width_mm,
                      height_mm, pitch_mm, step_mm):
    """Compensation assumes symmetric measured bead width about the centerline."""
    values = (lead_in_mm, inner_diameter_mm, bead_width_mm)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Lead-in, inner diameter and bead width must be finite')
    if not 0 <= lead_in_mm <= 100:
        raise ValueError('lead_in_mm must be 0..100')
    if inner_diameter_mm <= 0 or bead_width_mm < 0:
        raise ValueError('inner_diameter_mm must be positive; bead_width_mm nonnegative')
    # Validate start and all helix dimensions before generating the lead-in.
    tube_points(start, inner_diameter_mm + bead_width_mm, height_mm, pitch_mm, step_mm)
    length = lead_in_mm / 1000.0
    helix_start = (start[0] - length, start[1], start[2])
    helix = tube_points(helix_start, inner_diameter_mm + bead_width_mm,
                        height_mm, pitch_mm, step_mm)
    count = max(1, math.ceil(lead_in_mm / step_mm))
    lead = [(start[0] - length * i / count, start[1], start[2])
            for i in range(count + 1)]
    if length == 0:
        lead = [tuple(start)]
    if len(lead) + len(helix) - 1 > 100001:
        raise ValueError('Combined path exceeds 100000 intervals; increase step or pitch')
    return lead + helix[1:]
