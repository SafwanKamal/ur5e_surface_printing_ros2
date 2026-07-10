#!/usr/bin/env python3

import csv
import math
from pathlib import Path


# ============================================================
# Sphere / workpiece geometry
# ============================================================

# Sphere center in MoveIt's "world" frame.
CENTER_X = 0.55
CENTER_Y = 0.00
CENTER_Z = 0.25

# 150 mm diameter sphere.
SPHERE_RADIUS = 0.075

# TCP offset outward from sphere surface.
# The manually validated RViz pose was roughly 15 mm outside.
STANDOFF = 0.015

# Latitude of the ring.
#
# 0 deg   = top of sphere
# 90 deg  = equator
#
# This value was estimated from the known-good RViz pose.
PHI = math.radians(70.4)

# Number of segments around the full ring.
WAYPOINT_COUNT = 72

# Generate multiple versions of the same ring with different starts.
START_SAMPLE_COUNT = 10

OUTPUT_DIRECTORY = Path.home() / "ur5e_ws" / "paths"


# ============================================================
# Vector helpers
# ============================================================

def normalize(vector):
    """Return a unit-length 3D vector."""
    magnitude = math.sqrt(sum(value * value for value in vector))

    if magnitude < 1e-12:
        raise ValueError("Cannot normalize a zero-length vector.")

    return tuple(value / magnitude for value in vector)


def dot(a, b):
    """3D dot product."""
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    """3D cross product."""
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


# ============================================================
# Quaternion helpers
# ============================================================

def quaternion_from_rotation_matrix(matrix):
    """
    Convert a 3x3 rotation matrix to ROS quaternion order:

        (qx, qy, qz, qw)
    """
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]

    trace = m00 + m11 + m22

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (m21 - m12) / scale
        qy = (m02 - m20) / scale
        qz = (m10 - m01) / scale

    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / scale
        qx = 0.25 * scale
        qy = (m01 + m10) / scale
        qz = (m02 + m20) / scale

    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / scale
        qx = (m01 + m10) / scale
        qy = 0.25 * scale
        qz = (m12 + m21) / scale

    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / scale
        qx = (m02 + m20) / scale
        qy = (m12 + m21) / scale
        qz = 0.25 * scale

    return qx, qy, qz, qw


def normalize_quaternion(quaternion):
    """Normalize quaternion values."""
    x, y, z, w = quaternion

    magnitude = math.sqrt(
        x * x +
        y * y +
        z * z +
        w * w
    )

    if magnitude < 1e-12:
        raise ValueError("Cannot normalize a zero-length quaternion.")

    return (
        x / magnitude,
        y / magnitude,
        z / magnitude,
        w / magnitude,
    )


# ============================================================
# Orientation construction
# ============================================================

def make_orientation(path_tangent, desired_z_axis):
    """
    Construct a right-handed TCP orientation.

    Frame convention:

        TCP local +Z:
            points in desired_z_axis direction.

        TCP local +X:
            follows the surface-ring tangent as closely as possible.

        TCP local +Y:
            completes a right-handed coordinate frame.

    For this project, desired_z_axis is the inward normal:
    TCP -> sphere center.
    """
    z_axis = normalize(desired_z_axis)

    # Project tangent onto plane perpendicular to tool Z.
    tangent_projection = dot(path_tangent, z_axis)

    x_axis = (
        path_tangent[0] - tangent_projection * z_axis[0],
        path_tangent[1] - tangent_projection * z_axis[1],
        path_tangent[2] - tangent_projection * z_axis[2],
    )
    x_axis = normalize(x_axis)

    # Complete right-handed basis.
    y_axis = normalize(cross(z_axis, x_axis))

    # Recompute X to ensure strict orthogonality.
    x_axis = normalize(cross(y_axis, z_axis))

    rotation_matrix = [
        [x_axis[0], y_axis[0], z_axis[0]],
        [x_axis[1], y_axis[1], z_axis[1]],
        [x_axis[2], y_axis[2], z_axis[2]],
    ]

    return quaternion_from_rotation_matrix(rotation_matrix)


# ============================================================
# Ring generation
# ============================================================

def create_ring_rows(start_theta):
    """
    Create one complete closed ring.

    start_theta changes only the first point of the ring.
    The ring geometry itself remains identical.
    """
    rows = []

    for index in range(WAYPOINT_COUNT + 1):
        theta = start_theta + (
            2.0 * math.pi * index / WAYPOINT_COUNT
        )

        # Outward normal: sphere center -> surface point.
        outward_normal = (
            math.sin(PHI) * math.cos(theta),
            math.sin(PHI) * math.sin(theta),
            math.cos(PHI),
        )

        # Point on physical sphere surface.
        surface_point = (
            CENTER_X + SPHERE_RADIUS * outward_normal[0],
            CENTER_Y + SPHERE_RADIUS * outward_normal[1],
            CENTER_Z + SPHERE_RADIUS * outward_normal[2],
        )

        # TCP sits outside the sphere by STANDOFF.
        tcp_position = (
            surface_point[0] + STANDOFF * outward_normal[0],
            surface_point[1] + STANDOFF * outward_normal[1],
            surface_point[2] + STANDOFF * outward_normal[2],
        )

        # Tangent around the latitude ring.
        tangent = (
            -math.sin(PHI) * math.sin(theta),
            math.sin(PHI) * math.cos(theta),
            0.0,
        )

        # IMPORTANT:
        # Tool local +Z should point inward from the TCP
        # toward the sphere center / surface.
        inward_normal = (
            -outward_normal[0],
            -outward_normal[1],
            -outward_normal[2],
        )

        # No local-Y flip here.
        #
        # The previous local-Y flip reversed +Z and made the
        # generated tool orientation point outward from the sphere.
        final_quaternion = make_orientation(
            tangent,
            inward_normal,
        )

        qx, qy, qz, qw = normalize_quaternion(final_quaternion)

        rows.append({
            "x": tcp_position[0],
            "y": tcp_position[1],
            "z": tcp_position[2],
            "qx": qx,
            "qy": qy,
            "qz": qz,
            "qw": qw,
            "nx": outward_normal[0],
            "ny": outward_normal[1],
            "nz": outward_normal[2],
        })

    return rows


def write_csv(output_path, rows):
    """Write one ring to CSV."""
    with output_path.open("w", newline="") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "x",
                "y",
                "z",
                "qx",
                "qy",
                "qz",
                "qw",
                "nx",
                "ny",
                "nz",
            ],
        )

        writer.writeheader()
        writer.writerows(rows)


def main():
    OUTPUT_DIRECTORY.mkdir(parents=True, exist_ok=True)

    # Start 00 is deliberately close to the manually valid RViz pose,
    # whose radial angle was approximately -80 degrees.
    base_start_theta = math.radians(-80.0)

    for sample_index in range(START_SAMPLE_COUNT):
        start_theta = (
            base_start_theta +
            2.0 * math.pi * sample_index / START_SAMPLE_COUNT
        )

        rows = create_ring_rows(start_theta)

        output_path = (
            OUTPUT_DIRECTORY /
            f"hemisphere_ring_start_{sample_index:02d}.csv"
        )

        write_csv(output_path, rows)

        first = rows[0]

        print(
            f"Start {sample_index:02d} | "
            f"theta={math.degrees(start_theta):7.2f} deg | "
            f"TCP=({first['x']:.4f}, "
            f"{first['y']:.4f}, "
            f"{first['z']:.4f}) | "
            f"{output_path.name}"
        )

    print()
    print(f"Generated {START_SAMPLE_COUNT} ring CSV files.")
    print(f"Output directory: {OUTPUT_DIRECTORY}")
    print(f"Sphere center: ({CENTER_X}, {CENTER_Y}, {CENTER_Z})")
    print(f"Sphere radius: {SPHERE_RADIUS:.3f} m")
    print(f"TCP standoff: {STANDOFF:.3f} m")
    print(f"PHI: {math.degrees(PHI):.2f} deg")
    print("Tool +Z direction: inward toward sphere center")


if __name__ == "__main__":
    main()