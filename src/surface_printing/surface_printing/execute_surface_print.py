"""Execute a reviewed trajectory with leased constant-flow extrusion."""

import math
import signal
import time

from pathlib import Path

import rclpy
import yaml

from action_msgs.msg import GoalStatus

from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotTrajectory

from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions

from rosidl_runtime_py.set_message import set_message_fields

from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

from syringe_interfaces.srv import SetFlow

from .common import WorkNode, validate_trajectory


class PrintNode(WorkNode):
    def __init__(self):
        super().__init__("execute_surface_print")

        self.extruding = False

        self.started = 0.0
        self.last_health = 0.0
        self.health = ""
        self.error = ""

        self.last_ping = 0.0

        self.keepalive = self.create_publisher(
            Empty,
            "/syringe/flow_keepalive",
            10,
        )

        self.status_sub = self.create_subscription(
            String,
            "/syringe/status",
            self.status,
            10,
        )

        self.flow = self.create_client(
            SetFlow,
            "/syringe/set_flow",
        )

        self.stop = self.create_client(
            Trigger,
            "/syringe/stop",
        )

        self.robot = ActionClient(
            self,
            ExecuteTrajectory,
            "/execute_trajectory",
        )

        self.robot_goal = None
        self.goal_future = None
        self.abort_pending_goal = False

    def status(self, message):
        if message.data.startswith("STATE "):
            self.health = message.data[6:]
            self.last_health = time.monotonic()

        kind = message.data.split()[0:1]

        if self.extruding and kind in (
            ["FAULT"],
            ["ERROR"],
            ["LEASE_EXPIRED"],
        ):
            self.error = message.data

    def tick(self):
        if not self.extruding:
            return

        now = time.monotonic()

        if now - self.last_ping >= 0.2:
            self.keepalive.publish(Empty())
            self.last_ping = now

        if self.error:
            raise RuntimeError(
                "Syringe fault: " + self.error
            )

        if (
            now - self.started > 0.75
            and self.health != "MOVING"
        ):
            raise RuntimeError(
                "Syringe is no longer moving: "
                + self.health
            )

        if (
            now - max(self.last_health, self.started)
            > 1.5
        ):
            raise RuntimeError(
                "Lost syringe node status"
            )

    def late_goal(self, future):
        if self.abort_pending_goal:
            handle = future.result()

            if handle is not None and handle.accepted:
                handle.cancel_goal_async()

    def cleanup(self):
        self.extruding = False
        self.abort_pending_goal = True

        # Attempt both operations even if one fails.
        try:
            if self.stop.service_is_ready():
                result = self.wait(
                    self.stop.call_async(
                        Trigger.Request()
                    ),
                    3.0,
                )

                if not result.success:
                    self.get_logger().error(
                        result.message
                    )

            else:
                self.get_logger().error(
                    "Syringe STOP unavailable; "
                    "2-second flow lease is fallback"
                )

        except Exception as error:
            self.get_logger().error(
                f"Syringe STOP failed: {error}"
            )

        finally:
            if (
                self.robot_goal is not None
                and self.robot_goal.accepted
            ):
                try:
                    reply = self.wait(
                        self.robot_goal.cancel_goal_async(),
                        3.0,
                    )

                    if not reply.goals_canceling:
                        self.get_logger().warning(
                            "Robot cancel not accepted "
                            "(it may already be finished)"
                        )

                except Exception as error:
                    self.get_logger().error(
                        f"Robot cancel unconfirmed: {error}"
                    )


def run(node):
    path = node.parameter(
        "trajectory_path",
        "/tmp/surface_line_plan.yaml",
    )

    execute = bool(
        node.parameter("execute", False)
    )

    extrude = bool(
        node.parameter("extrude", False)
    )

    flow = float(
        node.parameter("flow_ml_per_min", 1.0)
    )

    tolerance = float(
        node.parameter("start_tolerance_rad", 0.03)
    )

    margin = float(
        node.parameter(
            "execution_timeout_margin_sec",
            5.0,
        )
    )

    if (
        not math.isfinite(flow)
        or flow <= 0
        or not 0 < tolerance <= 0.1
        or not 0 < margin <= 60
    ):
        raise ValueError(
            "Invalid flow, start tolerance, or timeout margin"
        )

    data = yaml.safe_load(
        Path(path).read_text()
    )

    if data.get("schema") != "surface_print_plan_v1":
        raise ValueError(
            "Unsupported plan file"
        )

    trajectory = RobotTrajectory()

    set_message_fields(
        trajectory,
        data["trajectory"],
    )

    duration = validate_trajectory(
        trajectory
    )

    joints = node.fresh_joints()

    current = dict(
        zip(
            joints.name,
            joints.position,
        )
    )

    first = trajectory.joint_trajectory.points[0]

    for name, position in zip(
        trajectory.joint_trajectory.joint_names,
        first.positions,
    ):
        if (
            name not in current
            or not math.isfinite(current[name])
            or abs(current[name] - position) > tolerance
        ):
            raise RuntimeError(
                "Robot is not at saved trajectory start: "
                f"{name}; replan"
            )

    node.get_logger().info(
        f"Plan duration={duration:.2f}s, "
        f"execute={execute}, extrude={extrude}"
    )

    if not execute:
        node.get_logger().info(
            "CHECK ONLY: robot and syringe commands "
            "were not sent"
        )
        return

    if not node.robot.wait_for_server(timeout_sec=5.0):
        raise RuntimeError(
            "MoveIt /execute_trajectory unavailable"
        )

    if extrude:
        if (
            not node.flow.wait_for_service(timeout_sec=5.0)
            or not node.stop.wait_for_service(timeout_sec=5.0)
        ):
            raise RuntimeError(
                "Syringe volume services unavailable"
            )

        baseline = node.wait(
            node.stop.call_async(
                Trigger.Request()
            ),
            3.0,
        )

        if not baseline.success:
            raise RuntimeError(
                "Could not establish syringe stopped state"
            )

    try:
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        node.goal_future = node.robot.send_goal_async(
            goal
        )

        node.goal_future.add_done_callback(
            node.late_goal
        )

        node.robot_goal = node.wait(
            node.goal_future,
            5.0,
        )

        if not node.robot_goal.accepted:
            raise RuntimeError(
                "MoveIt rejected trajectory; "
                "no extrusion started"
            )

        result_future = (
            node.robot_goal.get_result_async()
        )

        # The robot may begin moving before flow starts.
        # This is software coordination, not exact synchronization.
        if extrude:
            if result_future.done():
                raise RuntimeError(
                    "Robot finished before flow could start"
                )

            request = SetFlow.Request()
            request.enabled = True
            request.flow_ml_per_min = flow
            request.retract = False

            response = node.wait(
                node.flow.call_async(request),
                3.0,
            )

            if not response.success:
                raise RuntimeError(
                    response.message
                )

            node.extruding = True
            node.started = time.monotonic()
            node.last_ping = 0.0

            node.tick()

            node.get_logger().info(
                "Effective flow="
                f"{response.effective_flow_ml_per_min:.4f} "
                "mL/min"
            )

        response = node.wait(
            result_future,
            duration + margin,
        )

        node.extruding = False

        if extrude:
            stopped = node.wait(
                node.stop.call_async(
                    Trigger.Request()
                ),
                3.0,
            )

            if not stopped.success:
                raise RuntimeError(
                    "Robot finished but syringe STOP "
                    "unconfirmed"
                )

        if (
            response.status != GoalStatus.STATUS_SUCCEEDED
            or response.result.error_code.val != 1
        ):
            raise RuntimeError(
                "Robot execution failed: "
                f"status={response.status}, "
                "MoveIt code="
                f"{response.result.error_code.val}"
            )

        node.robot_goal = None

        node.get_logger().info(
            "Line completed; extrusion stopped"
        )

    except BaseException:
        node.cleanup()
        raise


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )

    def interrupt(signum, frame):
        raise KeyboardInterrupt()

    signal.signal(
        signal.SIGINT,
        interrupt,
    )

    signal.signal(
        signal.SIGTERM,
        interrupt,
    )

    node = PrintNode()

    try:
        run(node)

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()