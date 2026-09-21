"""Set a CAD-derived TCP in source YAML; rebuild/restart before using it."""
import argparse
import math
from pathlib import Path
import os
import tempfile
import yaml


def calculate_tcp(config, length_mm):
    if not math.isfinite(length_mm) or not 0 <= length_mm <= 200:
        raise ValueError('Needle extension must be finite and between 0 and 200 mm')
    def vector(key):
        values = [float(x) for x in config[key]]
        if len(values) != 3 or not all(math.isfinite(x) for x in values):
            raise ValueError(f'{key} must contain three finite numbers')
        return values
    outlet = vector('syringe_outlet_cad_mm')
    scale = vector('mesh_scale')
    if any(x <= 0 for x in scale):
        raise ValueError('Mesh scale must be positive')
    offset = vector('mesh_xyz')
    roll, pitch, yaw = vector('mesh_rpy')
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
                (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
                (-sp, cp*sr, cp*cr))
    # Length is physical mm, independent of mesh scaling. CAD outlet axis is +Z.
    point = [outlet[i]*scale[i] for i in range(3)]
    point[2] += length_mm / 1000.0
    xyz = [offset[i] + sum(rotation[i][j]*point[j] for j in range(3))
           for i in range(3)]
    return xyz, [roll, pitch, yaw]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True,
                        help='Source printing_tool.yaml (not an installed copy)')
    parser.add_argument('--needle-length-mm', type=float, required=True,
                        help='Outlet-to-tip extension, not total needle/hub length')
    parser.add_argument('--write', action='store_true', help='Apply; otherwise preview only')
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    xyz, rpy = calculate_tcp(config, args.needle_length_mm)
    print(f'tcp_xyz (m): {xyz}\ntcp_rpy (rad): {rpy}')
    if not args.write:
        print('Preview only. Add --write to update source configuration.')
        return
    config.update(tcp_xyz=xyz, tcp_rpy=rpy, needle_length_mm=args.needle_length_mm,
                  calibrated=False)
    # Atomic replacement avoids leaving a partial YAML on interruption.
    target = args.config.resolve()
    fd, name = tempfile.mkstemp(prefix=target.name+'.', dir=target.parent)
    try:
        with os.fdopen(fd, 'w') as handle:
            yaml.safe_dump(config, handle, sort_keys=False)
        os.replace(name, target)
    finally:
        if os.path.exists(name):
            os.unlink(name)
    print('Updated; calibrated=false. Verify tool alignment, rebuild and restart all '
          'robot/MoveIt launches. Generate a fresh path and plan. No live TF was changed.')


if __name__ == '__main__':
    main()
