"""Plan one localized CSV raster line without executing it."""

import csv
import json
import math
import time

from pathlib import Path

import rclpy
import yaml

from geometry_msgs.msg import Pose

from moveit_msgs.msg import (
    DisplayTrajectory,
    RobotState,
    JointConstraint,
)
from moveit_msgs.srv import (
    GetCartesianPath,
    GetPositionFK,
)

from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
)

from rosidl_runtime_py.convert import message_to_ordereddict

from .common import WorkNode, validate_trajectory
from .demo_safety import live_model
from .wrist_guard import load_bounds, check_state, check_trajectory


def read_waypoints(path, line_id):
    poses = []

    with open(path, newline="") as stream:
        rows = csv.DictReader(stream)

        required = {
            "x",
            "y",
            "z",
            "qx",
            "qy",
            "qz",
            "qw",
        }

        if not required.issubset(rows.fieldnames or []):
            raise ValueError(
                "CSV requires x,y,z,qx,qy,qz,qw "
                "in meters; optional line_id"
            )

        for row in rows:
            if (
                "line_id" in row
                and int(row["line_id"]) != line_id
            ):
                continue

            values = [
                float(row[key])
                for key in (
                    "x",
                    "y",
                    "z",
                    "qx",
                    "qy",
                    "qz",
                    "qw",
                )
            ]

            if not all(math.isfinite(v) for v in values):
                raise ValueError("Nonfinite CSV pose")

            norm = math.sqrt(
                sum(v * v for v in values[3:])
            )

            if abs(norm - 1.0) > 0.01:
                raise ValueError(
                    "CSV quaternions must have unit length"
                )

            pose = Pose()

            (
                pose.position.x,
                pose.position.y,
                pose.position.z,
            ) = values[:3]

            quaternion = [
                value / norm
                for value in values[3:]
            ]

            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ) = quaternion

            poses.append(pose)

    if len(poses) < 2:
        raise ValueError(
            "Selected line needs at least two poses"
        )

    return poses


def run(node):
    csv_path = node.parameter("csv_path", "")
    frame = node.parameter("frame_id", "")
    link = node.parameter("link_name", "")

    group = node.parameter(
        "group_name",
        "ur_manipulator",
    )

    line_id = int(node.parameter("line_id", 0))

    output = node.parameter(
        "output_path",
        "/tmp/surface_line_plan.yaml",
    )

    scale = float(
        node.parameter("velocity_scale", 0.03)
    )

    acceleration = float(
        node.parameter("acceleration_scale", 0.03)
    )

    step = float(
        node.parameter("eef_step", 0.001)
    )

    if not frame or not link or not csv_path:
        raise ValueError(
            "Set csv_path, frame_id, and calibrated "
            "nozzle link_name explicitly"
        )

    if (
        not 0 < scale <= 1
        or not 0 < acceleration <= 1
        or not 0 < step <= 0.01
    ):
        raise ValueError(
            "Invalid scaling or eef_step "
            "(must be <= 0.01 m)"
        )

    speed_mm_s = float(node.parameter("cartesian_speed_mm_s", 2.0))
    if not math.isfinite(speed_mm_s) or not 0.1 <= speed_mm_s <= 10:
        raise ValueError("Demo cartesian_speed_mm_s must be 0.1..10")
    _, model_hash = live_model(node)
    poses = read_waypoints(
        csv_path,
        line_id,
    )

    state = RobotState()
    state.joint_state = node.fresh_joints()
    state.is_diff = True
    wrist_bounds = load_bounds()
    check_state(state.joint_state, wrist_bounds)

    fk = node.create_client(
        GetPositionFK,
        "/compute_fk",
    )

    planner = node.create_client(
        GetCartesianPath,
        "/compute_cartesian_path",
    )

    if (
        not fk.wait_for_service(timeout_sec=5.0)
        or not planner.wait_for_service(timeout_sec=5.0)
    ):
        raise RuntimeError(
            "MoveIt FK/Cartesian services are unavailable"
        )

    request = GetPositionFK.Request()
    request.header.frame_id = frame
    request.fk_link_names = [link]
    request.robot_state = state

    response = node.wait(
        fk.call_async(request)
    )

    if (
        response.error_code.val != 1
        or len(response.pose_stamped) != 1
    ):
        raise RuntimeError(
            "FK failed; check reference frame and nozzle link"
        )

    current = response.pose_stamped[0].pose

    distance = math.sqrt(
        sum(
            (
                getattr(current.position, axis)
                - getattr(poses[0].position, axis)
            ) ** 2
            for axis in ("x", "y", "z")
        )
    )

    dot = abs(
        sum(
            getattr(current.orientation, axis)
            * getattr(poses[0].orientation, axis)
            for axis in ("x", "y", "z", "w")
        )
    )

    angle = 2 * math.acos(min(1.0, dot))

    if distance > 0.002 or angle > math.radians(5):
        raise RuntimeError(
            f"First position is {distance * 1000:.2f} mm / "
            f"{math.degrees(angle):.2f} deg from nozzle. "
            "Move to the line start with extrusion OFF first."
        )

    request = GetCartesianPath.Request()

    request.header.frame_id = frame
    request.start_state = state

    request.group_name = group
    request.link_name = link
    request.waypoints = poses

    request.max_step = step
    if not hasattr(request, "max_cartesian_speed"):
        raise RuntimeError("Installed moveit_msgs lacks Cartesian speed limiting; update Jazzy MoveIt")
    request.cartesian_speed_limited_link = link
    request.max_cartesian_speed = speed_mm_s / 1000.0
    request.avoid_collisions = True
    low, high = wrist_bounds
    wrist = JointConstraint()
    wrist.joint_name = 'wrist_3_joint'
    wrist.position = (low + high) / 2
    wrist.tolerance_above = wrist.tolerance_below = (high - low) / 2
    wrist.weight = 1.0
    request.path_constraints.joint_constraints = [wrist]

    request.revolute_jump_threshold = 0.25

    request.max_velocity_scaling_factor = scale
    request.max_acceleration_scaling_factor = acceleration

    response = node.wait(
        planner.call_async(request),
        60.0,
    )

    if (
        response.error_code.val != 1
        or response.fraction < 0.999999
    ):
        raise RuntimeError(
            "Incomplete/failed Cartesian path: "
            f"fraction={response.fraction:.6f}, "
            f"code={response.error_code.val}; no plan saved"
        )

    check_trajectory(response.solution, wrist_bounds)
    duration = validate_trajectory(
        response.solution
    )

    points = response.solution.joint_trajectory.points

    for previous, following in zip(
        points,
        points[1:],
    ):
        maximum_jump = max(
            abs(a - b)
            for a, b in zip(
                previous.positions,
                following.positions,
            )
        )

        if maximum_jump > 0.25:
            raise RuntimeError(
                "Joint discontinuity > 0.25 rad "
                "in Cartesian plan"
            )

    def plain(message):
        return json.loads(
            json.dumps(
                message_to_ordereddict(message)
            )
        )

    data = {
        "schema": "surface_print_plan_v2",
        "model_sha256": model_hash,
        "group_name": group,
        "cartesian_speed_mm_s": speed_mm_s,
        "source_csv": str(Path(csv_path).resolve()),
        "frame_id": frame,
        "link_name": link,
        "line_id": line_id,
        "created_unix_sec": time.time(),
        "duration_sec": duration,
        "trajectory": plain(response.solution),
        "start_state": plain(response.start_state),
    }

    Path(output).parent.mkdir(parents=True, exist_ok=True)
    Path(output).write_text(
        yaml.safe_dump(
            data,
            sort_keys=False,
        )
    )

    publisher = node.create_publisher(
        DisplayTrajectory,
        "/display_planned_path",
        QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )

    display = DisplayTrajectory()
    display.trajectory_start = response.start_state
    display.trajectory = [response.solution]

    for _ in range(10):
        publisher.publish(display)

        rclpy.spin_once(
            node,
            timeout_sec=0.1,
        )

    node.get_logger().info(
        f"PLAN ONLY: {duration:.2f} s, "
        f"{len(poses)} poses; saved {output}"
    )


def main(args=None):
    rclpy.init(args=args)

    node = WorkNode("plan_surface_path")

    try:
        run(node)

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()