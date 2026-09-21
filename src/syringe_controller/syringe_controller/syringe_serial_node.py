#!/usr/bin/env python3

"""Pulse and volume APIs sharing one exclusive serial backend."""

import math
import queue
import time

import rclpy
import serial

from rclpy.action import (
    ActionServer,
    CancelResponse,
    GoalResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import (
    ExternalShutdownException,
    MultiThreadedExecutor,
)
from rclpy.node import Node

from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

from syringe_interfaces.action import (
    DispenseVolume,
    MovePlunger,
)
from syringe_interfaces.srv import (
    SetExtrusion,
    SetFlow,
)

from .calibration import Calibration
from .serial_backend import SerialBackend


class SyringeSerialNode(Node):
    def __init__(self):
        super().__init__("syringe_serial_node")

        defaults = {
            "port": "/dev/ttyACM0",
            "baud_rate": 115200,
            "startup_timeout_sec": 5.0,
            "ack_timeout_sec": 1.0,
            "move_timeout_margin_sec": 3.0,
            "heartbeat_period_sec": 0.2,
            "maximum_step_rate_hz": 5000.0,
            "pulses_per_ml": 555.555556,
            "pulses_per_mm": 203.006329,
            "extrusion_direction": -1,
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def parameter(name):
            return self.get_parameter(name).value

        self.cal = Calibration(
            pulses_per_ml=float(parameter("pulses_per_ml")),
            extrusion_direction=int(
                parameter("extrusion_direction")
            ),
            maximum_step_rate_hz=float(
                parameter("maximum_step_rate_hz")
            ),
        )

        self.margin = float(
            parameter("move_timeout_margin_sec")
        )

        for name in (
            "startup_timeout_sec",
            "ack_timeout_sec",
            "move_timeout_margin_sec",
            "heartbeat_period_sec",
            "pulses_per_mm",
        ):
            value = float(parameter(name))

            if not math.isfinite(value) or value <= 0:
                raise ValueError(
                    f"{name} must be finite and positive"
                )

        if float(parameter("heartbeat_period_sec")) > 0.5:
            raise ValueError(
                "heartbeat_period_sec must be <= 0.5 "
                "for the 2-second watchdog"
            )

        self.group = ReentrantCallbackGroup()
        self.lines = queue.Queue(maxsize=1000)

        self.publisher = self.create_publisher(
            String,
            "syringe/status",
            10,
        )

        port = serial.Serial(
            str(parameter("port")),
            int(parameter("baud_rate")),
            timeout=0.05,
            write_timeout=0.5,
        )

        try:
            deadline = (
                time.monotonic()
                + float(parameter("startup_timeout_sec"))
            )

            pending = bytearray()
            ready = False

            while time.monotonic() < deadline and not ready:
                pending.extend(port.read(256))

                while b"\n" in pending:
                    line, _, pending = pending.partition(b"\n")

                    if line.strip() == b"READY SYRINGE_DEMO_V1":
                        ready = True

                if len(pending) > 4096:
                    raise RuntimeError(
                        "Invalid startup serial data"
                    )

            if not ready:
                raise RuntimeError(
                    "Arduino did not send READY; "
                    "flash firmware/syringe_demo (SYRINGE_DEMO_V1) and check USB"
                )

            self.backend = SerialBackend(
                port,
                float(parameter("ack_timeout_sec")),
                float(parameter("heartbeat_period_sec")),
                self._status,
            )

            if not self.backend.stop():
                raise RuntimeError(
                    "Startup STOP was not acknowledged"
                )

        except Exception:
            if hasattr(self, "backend"):
                self.backend.close()
            else:
                port.close()

            raise

        self.timer = self.create_timer(
            0.05,
            self._publish,
        )

        self.health_timer = self.create_timer(
            0.25,
            self._health,
        )

        self.keepalive = self.create_subscription(
            Empty,
            "syringe/flow_keepalive",
            lambda message: self.backend.renew(),
            10,
        )

        self.stop_srv = self.create_service(
            Trigger,
            "syringe/stop",
            self._stop,
            callback_group=self.group,
        )

        self.pulse_srv = self.create_service(
            SetExtrusion,
            "syringe/set_extrusion",
            self._extrusion,
            callback_group=self.group,
        )

        self.flow_srv = self.create_service(
            SetFlow,
            "syringe/set_flow",
            self._flow,
            callback_group=self.group,
        )

        self.actions = []

        for action_type, name, volume in (
            (MovePlunger, "move_plunger", False),
            (DispenseVolume, "dispense_volume", True),
        ):
            self.actions.append(
                ActionServer(
                    self,
                    action_type,
                    "syringe/" + name,
                    execute_callback=(
                        lambda handle, v=volume:
                        self._execute(handle, v)
                    ),
                    goal_callback=(
                        lambda goal, v=volume:
                        self._goal(goal, v)
                    ),
                    cancel_callback=(
                        lambda handle:
                        CancelResponse.ACCEPT
                    ),
                    callback_group=self.group,
                )
            )

        self.get_logger().info(
            "Syringe controller is ready "
            "(pulse and volume APIs)"
        )

    def _status(self, line):
        try:
            self.lines.put_nowait(line)
        except queue.Full:
            pass

    def _publish(self):
        for _ in range(100):
            try:
                line = self.lines.get_nowait()
            except queue.Empty:
                break

            self.publisher.publish(
                String(data=line)
            )

    def _health(self):
        state = self.backend.snapshot()

        if state["fault"]:
            value = "FAULT"
        elif state["moving"]:
            value = "MOVING"
        else:
            value = "IDLE"

        self.publisher.publish(
            String(data="STATE " + value)
        )

    def _stop(self, request, response):
        response.success = self.backend.stop()

        response.message = (
            "STOP acknowledged"
            if response.success
            else (
                "STOP unconfirmed; firmware watchdog is "
                "fallback; relaunch required"
            )
        )

        fault = self.backend.snapshot()["fault"]

        if fault:
            response.message += (
                "; fault remains latched: " + fault
            )

        return response

    def _start_flow(self, direction, rate, leased=True):
        if direction not in (-1, 1):
            return False, "direction must be -1 or +1"

        owner = ("flow", object())

        lease = 2.0 if leased else None

        if not self.backend.reserve(owner, lease_sec=lease):
            return (
                False,
                "Controller busy or faulted; stop current "
                "motion or relaunch after fault",
            )

        success = self.backend.start(
            f"START {direction} {rate}",
            owner,
        )

        if not success:
            self.backend.release(owner)

        return (
            success,
            "Flow started"
            if success
            else "Flow start failed or was stopped",
        )

    def _extrusion(self, request, response):
        if not request.enabled:
            return self._stop(request, response)

        try:
            rate = self.cal.rate(request.step_rate_hz)

            response.success, response.message = (
                self._start_flow(
                    request.direction,
                    rate,
                )
            )

        except ValueError as error:
            response.success = False
            response.message = str(error)

        return response

    def _flow(self, request, response):
        if not request.enabled:
            return self._stop(request, response)

        try:
            rate, effective = self.cal.flow(
                request.flow_ml_per_min
            )

            direction = (
                -self.cal.extrusion_direction
                if request.retract
                else self.cal.extrusion_direction
            )

            response.success, response.message = (
                self._start_flow(
                    direction,
                    rate,
                    leased=True,
                )
            )

            if response.success:
                response.effective_flow_ml_per_min = effective
                response.step_rate_hz = float(rate)

        except ValueError as error:
            response.success = False
            response.message = str(error)

        return response

    def _convert(self, goal, volume):
        if volume:
            steps = self.cal.volume(goal.volume_ml)

            rate, effective = self.cal.flow(
                goal.flow_ml_per_min
            )

        else:
            steps = goal.steps

            if steps == 0 or abs(steps) > 2147483647:
                raise ValueError(
                    "steps must be nonzero and within "
                    "+/-2147483647"
                )

            rate = self.cal.rate(goal.step_rate_hz)

            effective = (
                rate * 60.0 / self.cal.pulses_per_ml
            )

        if abs(steps) > 3000 or abs(steps) / rate > 100.0:
            raise ValueError("Demo limit: <=3000 pulses and <=100 seconds per move")
        return steps, rate, effective

    def _goal(self, goal, volume):
        try:
            self._convert(goal, volume)

        except ValueError as error:
            self.get_logger().warning(str(error))
            return GoalResponse.REJECT

        # Reserve before the execute callback is scheduled.
        if self.backend.reserve("action"):
            return GoalResponse.ACCEPT

        return GoalResponse.REJECT

    def _execute(self, handle, volume):
        kind = DispenseVolume if volume else MovePlunger
        result = kind.Result()

        steps, rate, effective = self._convert(
            handle.request,
            volume,
        )

        if volume:
            result.commanded_steps = steps
            result.effective_flow_ml_per_min = effective

        success = False
        canceled = False
        message = "Movement failed"
        count = 0

        try:
            if handle.is_cancel_requested:
                confirmed = self.backend.stop()
                canceled = confirmed

                message = (
                    "Canceled before movement"
                    if confirmed
                    else "Cancel STOP unconfirmed"
                )

            elif not self.backend.start(
                f"MOVE {steps} {rate}",
                "action",
            ):
                message = (
                    self.backend.snapshot()["fault"]
                    or "Movement stopped before starting"
                )

            else:
                deadline = (
                    time.monotonic()
                    + abs(steps) / rate
                    + self.margin
                )

                while rclpy.ok():
                    state = self.backend.snapshot()

                    count = (
                        state["completed"]
                        if state["done"]
                        else state["progress"]
                    )

                    if handle.is_cancel_requested:
                        confirmed = self.backend.stop()
                        canceled = confirmed

                        message = (
                            "Movement canceled; STOP acknowledged"
                            if confirmed
                            else "Cancel STOP unconfirmed"
                        )
                        break

                    if state["fault"]:
                        self.backend.stop()
                        message = state["fault"]
                        break

                    if (
                        state["stopped"]
                        or state["stop_requested"]
                    ):
                        confirmed = self.backend.stop()

                        message = (
                            "Movement stopped"
                            if confirmed
                            else "STOP unconfirmed"
                        )
                        break

                    if state["done"]:
                        success = count == abs(steps)

                        message = (
                            "Movement completed"
                            if success
                            else "DONE count differs from command"
                        )

                        if not success:
                            self.backend.fail(message)
                            self.backend.stop()

                        break

                    if time.monotonic() > deadline:
                        self.backend.fail(
                            "Movement timed out; relaunch required"
                        )

                        self.backend.stop()
                        message = "Movement timed out"
                        break

                    feedback = kind.Feedback()
                    feedback.steps_completed = count

                    if volume:
                        sign = (
                            1.0
                            if handle.request.volume_ml > 0
                            else -1.0
                        )

                        feedback.estimated_volume_ml = (
                            sign
                            * count
                            / self.cal.pulses_per_ml
                        )

                        feedback.effective_flow_ml_per_min = (
                            effective
                        )

                    handle.publish_feedback(feedback)

                    time.sleep(0.05)

                else:
                    self.backend.stop()
                    message = "ROS shutdown"

            state = self.backend.snapshot()

            count = (
                state["completed"]
                if state["done"]
                else state["progress"]
            )

            result.success = success
            result.message = message
            result.steps_completed = count

            if volume:
                sign = (
                    1.0
                    if handle.request.volume_ml > 0
                    else -1.0
                )

                result.estimated_volume_ml = (
                    sign
                    * count
                    / self.cal.pulses_per_ml
                )

            if canceled:
                handle.canceled()
            elif success:
                handle.succeed()
            else:
                handle.abort()

            return result

        except Exception:
            self.backend.fail(
                "Action exception; relaunch required"
            )

            self.backend.stop()
            raise

        finally:
            self.backend.release("action")

    def destroy_node(self):
        if hasattr(self, "backend"):
            self.backend.close()

        for action in getattr(self, "actions", []):
            action.destroy()

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None
    executor = MultiThreadedExecutor(num_threads=4)

    try:
        node = SyringeSerialNode()
        executor.add_node(node)
        executor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        # Stop before waiting for a long-running action to finish.
        if node is not None:
            node.backend.stop()

        executor.shutdown()

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()