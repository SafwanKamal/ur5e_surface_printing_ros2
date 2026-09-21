import math
import time

import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState


class WorkNode(Node):
    def __init__(self, name):
        super().__init__(name)

        self.joints = None
        self.joints_received = 0.0

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self._joints,
            qos_profile_sensor_data,
        )

    def _joints(self, message):
        self.joints = message
        self.joints_received = time.monotonic()

    def parameter(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def tick(self):
        pass

    def wait(self, future, timeout=10.0):
        deadline = time.monotonic() + timeout

        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError("ROS request timed out")

            self.tick()

            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

        if not future.done():
            raise RuntimeError(
                "ROS shutdown while waiting"
            )

        return future.result()

    def fresh_joints(self):
        deadline = time.monotonic() + 5.0

        while (
            self.joints is None
            or time.monotonic() - self.joints_received > 0.5
        ):
            if time.monotonic() > deadline:
                raise RuntimeError(
                    "No fresh /joint_states"
                )

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

        if len(self.joints.name) != len(self.joints.position):
            raise RuntimeError("Invalid joint state")

        return self.joints


def validate_trajectory(trajectory):
    joint_trajectory = trajectory.joint_trajectory
    names = joint_trajectory.joint_names

    if not names or len(set(names)) != len(names):
        raise ValueError(
            "Invalid trajectory joint names"
        )

    if (
        len(joint_trajectory.points) < 2
        or trajectory.multi_dof_joint_trajectory.points
    ):
        raise ValueError(
            "Expected a timed joint trajectory "
            "with at least two points"
        )

    previous_time = -1.0
    previous_point = None

    for point in joint_trajectory.points:
        if len(point.positions) != len(names):
            raise ValueError(
                "Trajectory position count mismatch"
            )

        for values in (
            point.positions,
            point.velocities,
            point.accelerations,
            point.effort,
        ):
            if values and len(values) != len(names):
                raise ValueError(
                    "Trajectory field count mismatch"
                )

            if not all(math.isfinite(v) for v in values):
                raise ValueError(
                    "Nonfinite trajectory data"
                )

        if point.time_from_start.sec < 0 or not 0 <= point.time_from_start.nanosec < 1000000000:
            raise ValueError("Invalid ROS duration")
        duration = (
            point.time_from_start.sec
            + point.time_from_start.nanosec * 1e-9
        )

        if duration < 0 or duration <= previous_time:
            raise ValueError(
                "Trajectory timestamps must increase strictly"
            )

        if previous_point is not None:
            dt = duration - previous_time
            if any(abs(a-b) > 0.25 or abs(a-b)/dt > 0.3
                   for a,b in zip(point.positions,previous_point.positions)):
                raise ValueError("Demo joint jump/speed limit exceeded (0.25 rad / 0.3 rad/s)")
        if any(abs(v) > 0.3 for v in point.velocities):
            raise ValueError("Demo joint velocity exceeds 0.3 rad/s")
        previous_point = point
        previous_time = duration

    if previous_time <= 0:
        raise ValueError(
            "Trajectory is not time-parameterized"
        )

    return previous_time