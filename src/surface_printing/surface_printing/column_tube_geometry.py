"""Vertical serpentine columns around a cylindrical wall."""
import math


def column_tube_path(start, lead_in_mm=10., inner_diameter_mm=10.,
                     bead_width_mm=.5, height_mm=3., overlap_fraction=.2,
                     column_count=0, step_mm=.1):
    """Return (point, phase, column_index) records and geometry metadata.

    Lead-in is world -X; the circle starts at its +X rim.
    Even column count permits a bottom-to-bottom continuous serpentine.
    """
    values = (*start, lead_in_mm, inner_diameter_mm, bead_width_mm,
              height_mm, overlap_fraction, step_mm)
    if len(start) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError('Geometry inputs must be finite')
    diameter = inner_diameter_mm + bead_width_mm
    if not (0 <= lead_in_mm <= 100 and inner_diameter_mm > 0
            and bead_width_mm > 0 and diameter <= 100
            and 0 < height_mm <= 100 and 0 < step_mm <= 1
            and 0 <= overlap_fraction < 1):
        raise ValueError('Require positive diameter/bead/height/step; '
                         'lead 0..100 mm, path diameter/height <=100 mm, '
                         'step <=1 mm, overlap in [0,1)')
    if isinstance(column_count, bool) or not isinstance(column_count, int):
        raise ValueError('column_count must be an integer')
    target_spacing = bead_width_mm * (1 - overlap_fraction)
    if column_count == 0:
        # Adjacent centerline chord length <= requested bead overlap spacing.
        n = max(4, math.ceil(math.pi / math.asin(min(1., target_spacing / diameter))))
        n += n % 2
    elif column_count >= 4 and column_count % 2 == 0:
        n = column_count
    else:
        raise ValueError('column_count must be 0 (automatic) or an even integer >=4')
    lead_n = max(1, math.ceil(lead_in_mm / step_mm))
    vertical_n = max(1, math.ceil(height_mm / step_mm))
    arc_length_mm = math.pi * diameter / n
    arc_n = max(1, math.ceil(arc_length_mm / step_mm))
    point_count = (lead_n + 1 if lead_in_mm else 1) + n * (vertical_n + arc_n)
    if point_count > 100001:
        raise ValueError('Path exceeds 100000 intervals; increase step or reduce column count')
    lead = lead_in_mm / 1000.
    height = height_mm / 1000.
    radius = diameter / 2000.
    rim = (start[0] - lead, start[1], start[2])
    center_x = rim[0] - radius
    def ring(theta, z):
        return (center_x + radius * math.cos(theta),
                rim[1] + radius * math.sin(theta), z)
    records = [(tuple(start), 'lead_in', -1)]
    if lead:
        records += [((start[0] - lead * j / lead_n, start[1], start[2]),
                     'lead_in', -1) for j in range(1, lead_n + 1)]
    for i in range(n):
        theta = 2 * math.pi * i / n
        up = i % 2 == 0
        for j in range(1, vertical_n + 1):
            fraction = j / vertical_n if up else 1 - j / vertical_n
            records.append((ring(theta, rim[2] + height * fraction), 'column', i))
        z = rim[2] + height if up else rim[2]
        for j in range(1, arc_n + 1):
            angle = theta + 2 * math.pi / n * j / arc_n
            records.append((ring(angle, z), 'connector', i))
    # Exact bottom seam closure after the final short connector.
    records[-1] = (rim, 'connector', n - 1)
    return records, dict(column_count=n, path_diameter_mm=diameter,
        center_spacing_mm=diameter * math.sin(math.pi / n),
        arc_spacing_mm=arc_length_mm,
        nominal_path_length_mm=lead_in_mm + n * height_mm + math.pi * diameter)
