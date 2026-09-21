"""Serial transport with exclusive motion ownership and heartbeat."""

import threading
import time


class SerialBackend:
    def __init__(
        self,
        port,
        ack_timeout=1.0,
        heartbeat_period=0.2,
        status=None,
    ):
        self.port = port
        self.ack_timeout = ack_timeout
        self.heartbeat_period = heartbeat_period
        self.status = status or (lambda line: None)

        self.cv = threading.Condition(threading.RLock())
        self.command_lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.shutdown = threading.Event()

        self.owner = None
        self.stops_pending = 0

        self.moving = False
        self.stop_requested = False
        self.fault = ""

        self.expected_ack = None
        self.acked = False

        self.done = False
        self.stopped = False

        self.progress = 0
        self.completed = 0

        self.lease_deadline = None
        self.last_rx = time.monotonic()
        self.motion_started = None

        self.reader = threading.Thread(
            target=self._read,
            daemon=True,
        )

        self.heartbeat = threading.Thread(
            target=self._heartbeat,
            daemon=True,
        )

        self.reader.start()
        self.heartbeat.start()

    def fail(self, message):
        with self.cv:
            self.fault = self.fault or message

            # Stop PINGs so the firmware watchdog remains a fallback.
            self.moving = False

            self.cv.notify_all()

    def reserve(self, owner, lease_sec=None):
        with self.cv:
            if (
                self.owner is not None
                or self.stops_pending
                or self.fault
                or self.shutdown.is_set()
            ):
                return False

            self.owner = owner
            self.lease_sec = lease_sec

            self.lease_deadline = (
                None
                if lease_sec is None
                else time.monotonic() + lease_sec
            )

            self.stop_requested = False

            self.done = False
            self.stopped = False

            self.progress = 0
            self.completed = 0

            return True

    def release(self, owner):
        with self.cv:
            if self.owner == owner:
                self.owner = None

    def snapshot(self):
        with self.cv:
            return {
                "fault": self.fault,
                "done": self.done,
                "stopped": self.stopped,
                "stop_requested": self.stop_requested,
                "progress": self.progress,
                "completed": self.completed,
                "moving": self.moving,
            }

    def _write(self, command):
        try:
            payload = (command + "\n").encode("ascii")

            with self.write_lock:
                written = self.port.write(payload)

                if written != len(payload):
                    raise OSError("Incomplete serial write")

            return True

        except (OSError, ValueError) as error:
            self.fail(f"Serial write failed: {error}")
            return False

    def _ack(
        self,
        command,
        expected,
        ignore_fault=False,
    ):
        with self.cv:
            self.expected_ack = expected
            self.acked = False

        if not self._write(command):
            return False

        deadline = time.monotonic() + self.ack_timeout

        with self.cv:
            while not self.acked:
                if self.shutdown.is_set():
                    break

                if self.fault and not ignore_fault:
                    break

                remaining = deadline - time.monotonic()

                if remaining <= 0:
                    break

                self.cv.wait(remaining)

            success = self.acked
            self.expected_ack = None

            return success

    def start(self, command, owner):
        with self.command_lock:
            with self.cv:
                if (
                    self.owner != owner
                    or self.stop_requested
                    or self.fault
                ):
                    return False

                self.moving = True
                self.last_rx = time.monotonic()
                self.motion_started = time.monotonic()

            expected = command.split()[0]

            if self._ack(command, expected):
                with self.cv:
                    faulted = bool(self.fault)

                    accepted = (
                        not faulted
                        and not self.stop_requested
                    )

                if faulted:
                    self._stop_locked()

                return accepted

            # A missing ACK does not prove that motion never started.
            self.fail(
                "Start failed or acknowledgment timed out; "
                "relaunch required"
            )

            self._stop_locked()

            return False

    def renew(self):
        with self.cv:
            if (
                self.owner is not None
                and self.lease_deadline is not None
            ):
                self.lease_deadline = (
                    time.monotonic() + self.lease_sec
                )

    def stop(self, owner=None):
        with self.cv:
            if owner is not None and self.owner != owner:
                return True

            # Also blocks accepted actions that have not started yet.
            self.stop_requested = True
            self.stops_pending += 1

        try:
            with self.command_lock:
                return self._stop_locked()

        finally:
            with self.cv:
                self.stops_pending -= 1

    def _stop_locked(self):
        acknowledged = self._ack(
            "STOP",
            "STOP",
            ignore_fault=True,
        )

        with self.cv:
            self.moving = False

            if acknowledged:
                self.stopped = True

                if (
                    isinstance(self.owner, tuple)
                    and self.owner[0] == "flow"
                ):
                    self.owner = None

            else:
                self.fault = (
                    self.fault
                    or "STOP unconfirmed; relaunch required"
                )

            self.cv.notify_all()

        return acknowledged

    def _line(self, line):
        self.last_rx = time.monotonic()
        self.status(line)

        fields = line.split()

        if not fields:
            return

        with self.cv:
            kind = fields[0]

            if kind == "ACK" and len(fields) >= 2:
                if fields[1] == self.expected_ack:
                    self.acked = True

            elif (
                kind in ("PROGRESS", "DONE")
                and len(fields) >= 2
            ):
                try:
                    count = abs(int(fields[1]))

                    if count > 2147483647:
                        raise ValueError()

                except ValueError:
                    self.fail("Invalid firmware step count")
                    return

                if kind == "PROGRESS":
                    self.progress = count

                else:
                    self.completed = count
                    self.done = True
                    self.moving = False

            elif kind == "STOPPED":
                self.stopped = True
                self.moving = False

            elif kind in ("FAULT", "ERROR", "READY"):
                self.fail(
                    f"Firmware fault/reset: {line}"
                )

            self.cv.notify_all()

    def _read(self):
        pending = bytearray()

        try:
            while not self.shutdown.is_set():
                data = self.port.read(256)

                if not data:
                    continue

                pending.extend(data)

                while b"\n" in pending:
                    line, _, pending = pending.partition(b"\n")

                    decoded = line.decode(
                        "ascii",
                        errors="replace",
                    ).strip()

                    self._line(decoded)

                if len(pending) > 4096:
                    self.fail(
                        "Serial line exceeds 4096 bytes"
                    )
                    return

        except (OSError, ValueError) as error:
            if not self.shutdown.is_set():
                self.fail(
                    f"Serial read failed: {error}"
                )

    def _heartbeat(self):
        while not self.shutdown.wait(self.heartbeat_period):
            with self.cv:
                active = self.moving and not self.fault

                expired_owner = (
                    self.owner
                    if (
                        self.lease_deadline is not None
                        and time.monotonic() > self.lease_deadline
                    )
                    else None
                )

            if expired_owner is not None:
                self.status("LEASE_EXPIRED")
                self.stop(expired_owner)

            elif active:
                if time.monotonic() - self.last_rx > 1.0:
                    self.fail("Firmware feedback lost; relaunch required")
                    self.stop()
                elif time.monotonic() - self.motion_started >= 110.0:
                    self.fail("Host maximum run time exceeded")
                    self.stop()
                else:
                    self._write("PING")

    def close(self):
        self.stop()
        self.shutdown.set()

        self.reader.join(timeout=1.0)
        self.heartbeat.join(timeout=1.0)

        self.port.close()