"""Scheme-A trajectory and claw commands over the fixed AA55 protocol."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable, Sequence

from fruit_picking_arm.communication.protocol import (
    ClawAction,
    ClawCommand,
    ClawResult,
    MessageType,
    MotionResult,
    RobotMode,
    RobotState,
    TrajectoryBegin,
    TrajectoryPoint,
    decode_claw_result,
    decode_motion_result,
    decode_robot_state,
    encode_claw_command,
    encode_trajectory_begin,
    encode_trajectory_point,
)
from fruit_picking_arm.communication.transport import SerialSession, SerialTransportError, open_serial


class ControlLink:
    def __init__(self, session: SerialSession, state_timeout_s: float = 1.0):
        self.session = session
        self._state_timeout_s = max(0.1, float(state_timeout_s))
        self._motion_lock = threading.RLock()
        self._state_lock = threading.Lock()
        self._latest_state: RobotState | None = None
        self._latest_state_monotonic = 0.0
        self._state_callbacks: list[Callable[[RobotState], None]] = []
        self._result_callbacks: list[Callable[[MotionResult], None]] = []
        session.add_callback(MessageType.ROBOT_STATE, self._on_state)
        session.add_callback(MessageType.MOTION_RESULT, self._on_result)

    @classmethod
    def open(
        cls, port: str, baudrate: int = 115200, ack_timeout: float = 0.25,
        retries: int = 3, state_timeout_s: float = 1.0,
    ) -> "ControlLink":
        session = SerialSession(open_serial(port, baudrate), ack_timeout=ack_timeout, retries=retries)
        session.start()
        return cls(session, state_timeout_s)

    def close(self) -> None:
        self.session.close()

    @property
    def latest_state(self) -> RobotState | None:
        with self._state_lock:
            return self._latest_state

    @property
    def ready(self) -> bool:
        with self._state_lock:
            state = self._latest_state
            age = time.monotonic() - self._latest_state_monotonic
        return bool(
            state is not None and age <= self._state_timeout_s
            and state.mode == RobotMode.READY and state.error_code == 0
        )

    def add_state_callback(self, callback: Callable[[RobotState], None]) -> None:
        self._state_callbacks.append(callback)

    def add_result_callback(self, callback: Callable[[MotionResult], None]) -> None:
        self._result_callbacks.append(callback)

    def send_heartbeat(self) -> int:
        return self.session.send_message(MessageType.HEARTBEAT, b"", require_ack=False)

    def send_trajectory(
        self, points: Sequence[TrajectoryPoint], wait_result: bool = True,
        result_timeout: float | None = None,
    ) -> MotionResult | None:
        with self._motion_lock:
            return self._send_trajectory(points, wait_result, result_timeout)

    def _send_trajectory(
        self, points: Sequence[TrajectoryPoint], wait_result: bool,
        result_timeout: float | None,
    ) -> MotionResult | None:
        if not points or len(points) > 100:
            raise ValueError("trajectory point count is outside C-board cache range 1..100")
        previous_time = -1
        payloads: list[bytes] = []
        for expected_index, point in enumerate(points):
            if point.index != expected_index:
                raise ValueError("trajectory point indices must be contiguous")
            if point.time_ms <= previous_time:
                raise ValueError("trajectory point times must increase strictly")
            previous_time = point.time_ms
            payloads.append(encode_trajectory_point(point))

        self.session.send_message(
            MessageType.TRAJECTORY_BEGIN,
            encode_trajectory_begin(TrajectoryBegin(len(points))), require_ack=True,
        )
        try:
            for payload in payloads:
                self.session.send_message(MessageType.TRAJECTORY_POINT, payload, require_ack=True)
            end_sequence = self.session.send_message(
                MessageType.TRAJECTORY_END, b"", require_ack=True
            )
        except Exception:
            self.send_abort(best_effort=True)
            raise
        if not wait_result:
            return None
        timeout = result_timeout
        if timeout is None:
            timeout = max(10.0, points[-1].time_ms / 1000.0 + 10.0)
        frame = self.session.wait_for(
            lambda f: f.msg_type == MessageType.MOTION_RESULT and f.seq == end_sequence,
            timeout,
        )
        if frame is None:
            self.send_abort(best_effort=True)
            raise SerialTransportError("trajectory result timeout")
        result = decode_motion_result(frame.payload)
        return MotionResult(frame.seq, result.object_id, result.result_code, result.error_code)

    def send_gripper(
        self, opening_mm: int, wait_result: bool = True, result_timeout: float = 10.0,
    ) -> ClawResult | None:
        action = ClawAction.CLOSE if int(opening_mm) == 0 else ClawAction.OPEN
        with self._motion_lock:
            command_seq = self.session.send_message(
                MessageType.CLAW_COMMAND,
                encode_claw_command(ClawCommand(action)), require_ack=True,
            )
            if not wait_result:
                return None
            frame = self.session.wait_for(
                lambda f: f.msg_type == MessageType.CLAW_RESULT and f.seq == command_seq,
                result_timeout,
            )
            if frame is None:
                raise SerialTransportError("claw result timeout")
            return decode_claw_result(frame.payload)

    def send_abort(self, best_effort: bool = False) -> None:
        try:
            self.session.send_message(MessageType.STOP, b"", require_ack=True)
        except Exception:
            if not best_effort:
                raise

    def _on_state(self, frame: object) -> None:
        try:
            state = decode_robot_state(frame.payload)
        except Exception:
            return
        with self._state_lock:
            self._latest_state = state
            self._latest_state_monotonic = time.monotonic()
        for callback in tuple(self._state_callbacks):
            callback(state)

    def _on_result(self, frame: object) -> None:
        try:
            decoded = decode_motion_result(frame.payload)
            result = MotionResult(
                frame.seq, decoded.object_id, decoded.result_code, decoded.error_code
            )
        except Exception:
            return
        for callback in tuple(self._result_callbacks):
            callback(result)
