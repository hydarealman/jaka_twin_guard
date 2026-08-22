"""High-level commands shared by target and trajectory serial modes."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable, Sequence

from jaka_single_arm.communication.protocol import (
    FruitTarget,
    GripperCommand,
    GripperMode,
    MessageType,
    MotionResult,
    RobotMode,
    RobotState,
    TrajectoryBegin,
    TrajectoryEnd,
    TrajectoryPoint,
    decode_motion_result,
    decode_robot_state,
    encode_fruit_target,
    encode_gripper_command,
    encode_trajectory_begin,
    encode_trajectory_end,
    encode_trajectory_point,
    trajectory_points_crc32,
)
from jaka_single_arm.communication.transport import SerialSession, SerialTransportError, open_serial


class ControlLink:
    """Reliable command API used by both ROS2 bridge nodes."""

    def __init__(self, session: SerialSession, state_timeout_s: float = 1.0):
        self.session = session
        self._state_timeout_s = max(0.1, float(state_timeout_s))
        self._trajectory_id = 0
        self._gripper_command_id = 0
        self._id_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._latest_state: RobotState | None = None
        self._latest_state_monotonic = 0.0
        self._state_callbacks: list[Callable[[RobotState], None]] = []
        self._result_callbacks: list[Callable[[MotionResult], None]] = []
        session.add_callback(MessageType.ROBOT_STATE, self._on_state)
        session.add_callback(MessageType.MOTION_RESULT, self._on_result)

    @classmethod
    def open(
        cls,
        port: str,
        baudrate: int = 115200,
        ack_timeout: float = 0.25,
        retries: int = 3,
        state_timeout_s: float = 1.0,
    ) -> "ControlLink":
        device = open_serial(port, baudrate)
        session = SerialSession(device, ack_timeout=ack_timeout, retries=retries)
        session.start()
        return cls(session, state_timeout_s=state_timeout_s)

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
        return (
            state is not None
            and age <= self._state_timeout_s
            and state.mode == RobotMode.READY
            and state.error_code == 0
        )

    def add_state_callback(self, callback: Callable[[RobotState], None]) -> None:
        self._state_callbacks.append(callback)

    def add_result_callback(self, callback: Callable[[MotionResult], None]) -> None:
        self._result_callbacks.append(callback)

    def send_heartbeat(self) -> int:
        timestamp_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
        return self.session.send_message(
            MessageType.HEARTBEAT,
            timestamp_ms.to_bytes(4, "little"),
            require_ack=False,
        )

    def send_fruit_target(self, target: FruitTarget, wait_result: bool = True, result_timeout: float = 60.0) -> MotionResult | None:
        command_seq = self.session.send_message(
            MessageType.FRUIT_TARGET,
            encode_fruit_target(target),
            require_ack=True,
        )
        if not wait_result:
            return None
        frame = self.session.wait_for(
            lambda f: f.msg_type == MessageType.MOTION_RESULT
            and _result_matches(f.payload, target.target_id, command_seq, strict_seq=True),
            timeout=result_timeout,
        )
        if frame is None:
            raise SerialTransportError(f"fruit target {target.target_id} result timeout")
        return decode_motion_result(frame.payload)

    def send_trajectory(
        self,
        points: Sequence[TrajectoryPoint],
        wait_result: bool = True,
        result_timeout: float | None = None,
    ) -> MotionResult | None:
        if not points:
            raise ValueError("cannot send an empty trajectory")
        joint_count = len(points[0].positions)
        if any(len(point.positions) != joint_count for point in points):
            raise ValueError("trajectory points have inconsistent joint counts")
        trajectory_id = self._next_trajectory_id()
        include_velocities = True
        begin = TrajectoryBegin(trajectory_id, len(points), joint_count, include_velocities)
        begin_seq = self.session.send_message(
            MessageType.TRAJECTORY_BEGIN,
            encode_trajectory_begin(begin),
            require_ack=True,
        )

        point_payloads: list[bytes] = []
        try:
            for point in points:
                payload = encode_trajectory_point(trajectory_id, point, include_velocities)
                point_payloads.append(payload)
                self.session.send_message(MessageType.TRAJECTORY_POINT, payload, require_ack=True)
            end = TrajectoryEnd(
                trajectory_id=trajectory_id,
                point_count=len(points),
                points_crc32=trajectory_points_crc32(point_payloads),
            )
            self.session.send_message(
                MessageType.TRAJECTORY_END,
                encode_trajectory_end(end),
                require_ack=True,
            )
        except Exception:
            self.send_abort(best_effort=True)
            raise

        if not wait_result:
            return None
        if result_timeout is None:
            result_timeout = max(10.0, points[-1].time_ms / 1000.0 + 10.0)
        frame = self.session.wait_for(
            lambda f: f.msg_type == MessageType.MOTION_RESULT
            and _result_matches(f.payload, trajectory_id, begin_seq),
            timeout=result_timeout,
        )
        if frame is None:
            self.send_abort(best_effort=True)
            raise SerialTransportError(f"trajectory {trajectory_id} result timeout")
        return decode_motion_result(frame.payload)

    def send_gripper(
        self,
        opening_mm: int,
        speed_mm_s: int = 100,
        force_permille: int = 500,
        mode: GripperMode = GripperMode.POSITION,
        wait_result: bool = True,
        result_timeout: float = 10.0,
    ) -> MotionResult | None:
        command_id = self._next_gripper_command_id()
        command = GripperCommand(
            command_id=command_id,
            mode=mode,
            opening_mm=int(opening_mm),
            speed_mm_s=int(speed_mm_s),
            force_permille=int(force_permille),
        )
        command_seq = self.session.send_message(
            MessageType.GRIPPER_COMMAND,
            encode_gripper_command(command),
            require_ack=True,
        )
        if not wait_result:
            return None
        frame = self.session.wait_for(
            lambda f: f.msg_type == MessageType.MOTION_RESULT
            and _result_matches(
                f.payload, command_id, command_seq, strict_seq=True
            ),
            timeout=result_timeout,
        )
        if frame is None:
            raise SerialTransportError(
                f"gripper command {command_id} result timeout"
            )
        return decode_motion_result(frame.payload)

    def send_abort(self, best_effort: bool = False) -> None:
        try:
            self.session.send_message(MessageType.ABORT, b"", require_ack=True)
        except Exception:
            if not best_effort:
                raise

    def _next_trajectory_id(self) -> int:
        with self._id_lock:
            self._trajectory_id = (self._trajectory_id % 0xFFFF) + 1
            return self._trajectory_id

    def _next_gripper_command_id(self) -> int:
        with self._id_lock:
            self._gripper_command_id = (
                self._gripper_command_id % 0xFFFF
            ) + 1
            return self._gripper_command_id

    def _on_state(self, frame) -> None:
        try:
            state = decode_robot_state(frame.payload)
        except Exception:
            return
        with self._state_lock:
            self._latest_state = state
            self._latest_state_monotonic = time.monotonic()
        for callback in tuple(self._state_callbacks):
            callback(state)

    def _on_result(self, frame) -> None:
        try:
            result = decode_motion_result(frame.payload)
        except Exception:
            return
        for callback in tuple(self._result_callbacks):
            callback(result)


def _result_matches(
    payload: bytes,
    object_id: int,
    command_seq: int,
    strict_seq: bool = False,
) -> bool:
    try:
        result = decode_motion_result(payload)
    except Exception:
        return False
    # Target commands require an exact sequence to prevent a stale target
    # result from completing a new request. Trajectory firmware may report the
    # BEGIN or END sequence, so trajectory matching is by unique trajectory id.
    return result.object_id == object_id and (
        not strict_seq or result.command_seq == command_seq
    )
