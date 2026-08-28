"""Scheme-A controller emulator for serial tests and virtual-port debugging."""

from __future__ import annotations

import argparse
import threading
import time

from fruit_picking_arm.communication.protocol import (
    Ack,
    AckStatus,
    ClawAction,
    ClawResult,
    ClawResultCode,
    ClawState,
    ClawStateCode,
    Frame,
    FrameParser,
    MessageType,
    MotionResult,
    ResultCode,
    RobotMode,
    RobotState,
    decode_ack,
    decode_claw_command,
    decode_trajectory_begin,
    decode_trajectory_point,
    encode_ack,
    encode_claw_result,
    encode_claw_state,
    encode_motion_result,
    encode_robot_state,
)
from fruit_picking_arm.communication.transport import open_serial


class BoardEmulator:
    def __init__(
        self, serial_device, state_rate_hz: float = 20.0,
        execution_delay: float = 0.01, watchdog_timeout_s: float = 1.0,
        claw_action_duration_s: float = 1.5,
    ):
        self._serial = serial_device
        self._parser = FrameParser()
        self._state_period = 1.0 / max(1.0, state_rate_hz)
        self._execution_delay = execution_delay
        self._claw_action_duration = max(0.0, float(claw_action_duration_s))
        self._watchdog_timeout = watchdog_timeout_s
        self._running = threading.Event()
        self._thread: threading.Thread | None = None
        self._write_lock = threading.Lock()
        self._sequence = 1000
        self._seen: dict[tuple[int, int], AckStatus] = {}
        self._last_rx = time.monotonic()
        self._last_state = 0.0
        self.mode = RobotMode.READY
        self.error_code = 0
        # Safe midpoint of the current production joint convention. Starting
        # at all zeros would place J2/J3 on their physical hard stops and must
        # not make the real-controller acceptance test motion-ready.
        self.joints = (0.0, 1.265363708, -1.570796327, 0.0, 0.0, 0.0)
        self.expected_points = 0
        self.points = []
        self.motion_execution_count = 0
        self.claw_execution_count = 0
        self.claw_state = ClawStateCode.UNKNOWN
        self._pending_result: tuple[Frame, float, int] | None = None

    def start(self) -> None:
        self._running.set()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._running.clear()
        try:
            self._serial.close()
        finally:
            if self._thread:
                self._thread.join(timeout=1.0)

    def trigger_estop(self) -> None:
        self.mode = RobotMode.ESTOP
        self.error_code = 0x0401

    def _next_seq(self) -> int:
        self._sequence = (self._sequence % 0xFFFF) + 1
        return self._sequence

    def _run(self) -> None:
        while self._running.is_set():
            data = self._serial.read(512)
            if data:
                for frame in self._parser.feed(data):
                    self._handle(frame)
            now = time.monotonic()
            if now - self._last_state >= self._state_period:
                self._last_state = now
                self._send_state()
            if self._pending_result and now - self._pending_result[1] >= 0.1:
                frame, _sent, retries = self._pending_result
                if retries >= 5:
                    self._pending_result = None
                else:
                    self._write(frame.encode())
                    self._pending_result = (frame, now, retries + 1)
            if (
                self.mode == RobotMode.BUSY
                and now - self._last_rx > self._watchdog_timeout
            ):
                self.mode = RobotMode.ERROR
                self.error_code = 0x0201

    def _handle(self, frame: Frame) -> None:
        self._last_rx = time.monotonic()
        if frame.msg_type == MessageType.HEARTBEAT:
            return
        if frame.msg_type == MessageType.ACK:
            try:
                ack = decode_ack(frame.payload)
            except Exception:
                return
            if self._pending_result and frame.seq == self._pending_result[0].seq and ack.status == AckStatus.OK:
                self._pending_result = None
            return

        key = (int(frame.msg_type), frame.seq)
        if key in self._seen:
            self._send_ack(frame.seq, self._seen[key])
            return
        status = self._execute_command(frame)
        self._seen[key] = status
        if len(self._seen) > 64:
            self._seen.pop(next(iter(self._seen)))
        self._send_ack(frame.seq, status)

    def _execute_command(self, frame: Frame) -> AckStatus:
        if self.mode in (RobotMode.ERROR, RobotMode.ESTOP):
            return AckStatus.FAULT
        try:
            if frame.msg_type == MessageType.TRAJECTORY_BEGIN:
                if self.mode != RobotMode.READY:
                    return AckStatus.BUSY
                self.expected_points = decode_trajectory_begin(frame.payload).point_count
                self.points = []
                return AckStatus.OK
            if frame.msg_type == MessageType.TRAJECTORY_POINT:
                point = decode_trajectory_point(frame.payload)
                if point.index != len(self.points) or point.index >= self.expected_points:
                    return AckStatus.MISSING_POINT
                if self.points and point.time_ms <= self.points[-1].time_ms:
                    return AckStatus.BAD_DATA
                self.points.append(point)
                return AckStatus.OK
            if frame.msg_type == MessageType.TRAJECTORY_END:
                if len(self.points) != self.expected_points or not self.points:
                    return AckStatus.MISSING_POINT
                self.mode = RobotMode.BUSY
                self.motion_execution_count += 1
                threading.Thread(
                    target=self._finish_motion, args=(frame.seq,), daemon=True
                ).start()
                return AckStatus.OK
            if frame.msg_type == MessageType.STOP:
                self.mode = RobotMode.READY
                self.points = []
                if self.claw_state in (ClawStateCode.OPENING, ClawStateCode.CLOSING):
                    self.claw_state = ClawStateCode.UNKNOWN
                    self._send_claw_state()
                return AckStatus.OK
            if frame.msg_type == MessageType.CLAW_COMMAND:
                command = decode_claw_command(frame.payload)
                if command.action == ClawAction.STOP:
                    if self.claw_state in (ClawStateCode.OPENING, ClawStateCode.CLOSING):
                        self.claw_state = ClawStateCode.UNKNOWN
                    self._send_claw_state()
                    return AckStatus.OK
                self.claw_execution_count += 1
                self.claw_state = (
                    ClawStateCode.OPENING
                    if command.action == ClawAction.OPEN
                    else ClawStateCode.CLOSING
                )
                self._send_claw_state()
                threading.Thread(
                    target=self._finish_claw,
                    args=(frame.seq, command.action == ClawAction.OPEN),
                    daemon=True,
                ).start()
                return AckStatus.OK
        except Exception:
            return AckStatus.BAD_DATA
        return AckStatus.BAD_DATA

    def _finish_motion(self, command_seq: int) -> None:
        time.sleep(self._execution_delay)
        if self.mode != RobotMode.BUSY:
            return
        self.joints = tuple(self.points[-1].positions)
        self.mode = RobotMode.READY
        self._send_reliable(
            MessageType.MOTION_RESULT,
            encode_motion_result(MotionResult(command_seq, 0, ResultCode.SUCCESS, 0)),
            sequence=command_seq,
        )

    def _finish_claw(self, command_seq: int, opening: bool) -> None:
        time.sleep(self._claw_action_duration)
        expected = ClawStateCode.OPENING if opening else ClawStateCode.CLOSING
        if self.claw_state != expected:
            return
        self.claw_state = ClawStateCode.OPEN if opening else ClawStateCode.CLOSED
        self._send_claw_state()
        self._send_reliable(
            MessageType.CLAW_RESULT,
            encode_claw_result(ClawResult(ClawResultCode.COMMAND_COMPLETED_UNVERIFIED)),
            sequence=command_seq,
        )

    def _send_ack(self, sequence: int, status: AckStatus) -> None:
        self._write(Frame(MessageType.ACK, sequence, encode_ack(Ack(status))).encode())

    def _send_state(self) -> None:
        payload = encode_robot_state(RobotState(self.mode, self.error_code, tuple(self.joints)))
        self._write(Frame(MessageType.ROBOT_STATE, self._next_seq(), payload).encode())
        self._send_claw_state()

    def _send_claw_state(self) -> None:
        payload = encode_claw_state(ClawState(self.claw_state, verified=False))
        self._write(Frame(MessageType.CLAW_STATE, self._next_seq(), payload).encode())

    def _send_reliable(
        self, msg_type: MessageType, payload: bytes, sequence: int | None = None,
    ) -> None:
        frame = Frame(msg_type, self._next_seq() if sequence is None else sequence, payload)
        self._write(frame.encode())
        self._pending_result = (frame, time.monotonic(), 0)

    def _write(self, packet: bytes) -> None:
        with self._write_lock:
            self._serial.write(packet)
            flush = getattr(self._serial, "flush", None)
            if callable(flush):
                flush()


def main(args=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--execution-delay", type=float, default=0.01)
    ns = parser.parse_args(args)
    emulator = BoardEmulator(
        open_serial(ns.port, ns.baudrate), execution_delay=ns.execution_delay
    )
    emulator.start()
    print(f"Scheme-A board emulator running on {ns.port} @ {ns.baudrate}")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        emulator.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
