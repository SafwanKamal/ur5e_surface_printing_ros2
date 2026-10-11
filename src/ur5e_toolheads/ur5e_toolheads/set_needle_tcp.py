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


def needle_for_flange_distance(config, distance_mm):
    """Restore the CAD lateral offset and set axial distance from the flange."""
    if not math.isfinite(distance_mm) or not 0 < distance_mm <= 300:
        raise ValueError('Flange axial distance must be in (0,300] mm')
    base, _ = calculate_tcp(config, 0)
    tip, _ = calculate_tcp(config, 1)
    # UR flange +X is the physical tool axis. Transform the CAD outlet/axis
    # through the existing mount; do not erase its lateral displacement.
    mount = dict(mesh_scale=[1., 1., 1.], mesh_xyz=config['mount_xyz'],
                 mesh_rpy=config['mount_rpy'], syringe_outlet_cad_mm=base)
    flange_base, _ = calculate_tcp(mount, 0)
    mount['mesh_xyz'] = [0., 0., 0.]
    mount['syringe_outlet_cad_mm'] = [(tip[i]-base[i])*1000 for i in range(3)]
    axis, _ = calculate_tcp(mount, 0)
    if abs(abs(axis[0])-1) > 1e-6 or math.hypot(axis[1], axis[2]) > 1e-6:
        raise ValueError('CAD nozzle axis is not parallel to the flange tool axis; '
                         'cannot preserve sideways offset by changing needle length')
    length = (distance_mm/1000 - flange_base[0])/axis[0]*1000
    calculate_tcp(config, length)  # Validate representable physical extension.
    return length


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True,
                        help='Source printing_tool.yaml (not an installed copy)')
    length_args = parser.add_mutually_exclusive_group(required=True)
    length_args.add_argument('--needle-length-mm', type=float,
                        help='Outlet-to-tip extension, not total needle/hub length')
    length_args.add_argument('--flange-distance-mm', type=float,
                             help='Axial flange-to-tip distance; restore CAD sideways offset')
    parser.add_argument('--write', action='store_true', help='Apply; otherwise preview only')
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    length = (needle_for_flange_distance(config, args.flange_distance_mm)
              if args.flange_distance_mm is not None else args.needle_length_mm)
    xyz, rpy = calculate_tcp(config, length)
    print(f'tcp_xyz (m): {xyz}\ntcp_rpy (rad): {rpy}')
    if not args.write:
        print('Preview only. Add --write to update source configuration.')
        return
    config.update(tcp_xyz=xyz, tcp_rpy=rpy, needle_length_mm=length,
                  calibrated=False)
    # An earlier centered axial-nozzle envelope no longer represents this tool.
    config.pop('axial_nozzle_length_mm', None)
    config.pop('axial_nozzle_radius_mm', None)
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
