"""Set total flange-face-to-tip length along UR tool0 +Z (ROS flange +X)."""
import argparse
import math
import os
import tempfile
from pathlib import Path
import yaml
from .set_needle_tcp import calculate_tcp


def axial_tcp(config, length_mm):
    if not math.isfinite(length_mm) or not 0 < length_mm <= 300:
        raise ValueError('Flange-to-tip length must be in (0,300] mm')
    # Reuse CAD transform math to get the mount rotation matrix columns.
    mount = dict(mesh_scale=[1., 1., 1.], mesh_xyz=[0., 0., 0.],
                 mesh_rpy=config['mount_rpy'])
    columns = []
    for i in range(3):
        mount['syringe_outlet_cad_mm'] = [float(j == i) for j in range(3)]
        columns.append(calculate_tcp(mount, 0)[0])
    offset = [float(v) for v in config['mount_xyz']]
    if len(offset) != 3 or not all(math.isfinite(v) for v in offset):
        raise ValueError('Invalid mount_xyz')
    delta = [length_mm/1000. - offset[0], -offset[1], -offset[2]]
    xyz = [sum(columns[i][j]*delta[j] for j in range(3)) for i in range(3)]
    # UR's flange->tool0 rotation is RPY(pi/2,0,pi/2), not identity.
    # Target orientation keeps TCP +Z pointing physically out of the nozzle.
    tool0_rotation = ((0.,0.,1.),(1.,0.,0.),(0.,1.,0.))
    r = [[sum(columns[i][k]*tool0_rotation[k][j] for k in range(3))
          for j in range(3)] for i in range(3)]
    pitch = math.asin(max(-1., min(1., -r[2][0])))
    if abs(math.cos(pitch)) > 1e-8:
        roll, yaw = math.atan2(r[2][1], r[2][2]), math.atan2(r[1][0], r[0][0])
    else:
        roll, yaw = math.atan2(-r[1][2], r[1][1]), 0.
    return xyz, [roll, pitch, yaw]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--length-mm', type=float, default=160.)
    parser.add_argument('--radius-mm', type=float, required=True,
                        help='Measured maximum outer radius of the axial nozzle')
    parser.add_argument('--write', action='store_true')
    args = parser.parse_args()
    if not math.isfinite(args.radius_mm) or not 0 < args.radius_mm <= 20:
        parser.error('radius-mm must be in (0,20]')
    config = yaml.safe_load(args.config.read_text())
    xyz, rpy = axial_tcp(config, args.length_mm)
    print(f'tool0 -> probe_tcp: xyz=[0,0,{args.length_mm/1000}] rpy=[0,0,0]')
    print(f'probe_tool_link -> probe_tcp: xyz={xyz} rpy={rpy}')
    if not args.write:
        print('Preview only; add --write to apply.')
        return
    config.update(enabled=True, calibrated=False, tcp_xyz=xyz, tcp_rpy=rpy,
                  needle_length_mm=0., axial_nozzle_length_mm=args.length_mm,
                  axial_nozzle_radius_mm=args.radius_mm)
    # Do not relabel old CAD pump geometry or silently bypass calibration.
    target = args.config.resolve()
    fd, temporary = tempfile.mkstemp(prefix=target.name+'.', dir=target.parent)
    try:
        with os.fdopen(fd, 'w') as handle:
            yaml.safe_dump(config, handle, sort_keys=False)
        os.replace(temporary, target)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
    print('Updated source YAML; calibrated=false. Rebuild/restart, verify actual '
          'tip and pump collisions, then mark calibrated=true. No motion commanded.')


if __name__ == '__main__':
    main()
