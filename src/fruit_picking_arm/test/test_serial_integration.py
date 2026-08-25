import threading
import time

import pytest

from fruit_picking_arm.communication.board_emulator import BoardEmulator
from fruit_picking_arm.communication.control_link import ControlLink
from fruit_picking_arm.communication.protocol import (
    ClawResultCode,
    Frame,
    MessageType,
    MotionResult,
    ResultCode,
    RobotMode,
    TrajectoryPoint,
    encode_motion_result,
)
from fruit_picking_arm.communication.transport import AckRejected, SerialSession


class MemorySerial:
    def __init__(self):
        self.peer = None
        self.timeout = 0.01
        self._buffer = bytearray()
        self._condition = threading.Condition()
        self._closed = False

    def write(self, data):
        with self.peer._condition:
            self.peer._buffer.extend(data)
            self.peer._condition.notify_all()
        return len(data)

    def read(self, size):
        with self._condition:
            if not self._buffer and not self._closed:
                self._condition.wait(self.timeout)
            data = bytes(self._buffer[:size])
            del self._buffer[:size]
            return data

    def flush(self):
        pass

    def close(self):
        self._closed = True
        with self._condition:
            self._condition.notify_all()


class DropFirstFrameSerial(MemorySerial):
    def __init__(self, msg_type):
        super().__init__()
        self._drop_type = msg_type
        self._dropped = False

    def write(self, data):
        if not self._dropped:
            try:
                frame = Frame.decode(bytes(data))
            except Exception:
                frame = None
            if frame is not None and frame.msg_type == self._drop_type:
                self._dropped = True
                return len(data)
        return super().write(data)


class ReadFailSerial(MemorySerial):
    def read(self, _size):
        raise OSError("device disconnected")


def make_pair(board_cls=MemorySerial):
    host, board = MemorySerial(), board_cls()
    host.peer, board.peer = board, host
    return host, board


def wait_ready(link):
    deadline = time.monotonic() + 1.0
    while not link.ready and time.monotonic() < deadline:
        time.sleep(0.01)
    assert link.ready


def points():
    return [
        TrajectoryPoint(0, 0, (0.0,) * 6, (0.0,) * 6),
        TrajectoryPoint(1, 200, (0.1, -0.1, 0.2, -0.2, 0.3, -0.3), (0.0,) * 6),
    ]


def test_serial_reader_fails_closed_on_device_error():
    session = SerialSession(ReadFailSerial())
    session.start()
    deadline = time.monotonic() + 1.0
    while session.is_running and time.monotonic() < deadline:
        time.sleep(0.01)
    assert not session.is_running
    assert "device disconnected" in (session.fatal_error or "")
    session.close()


def test_trajectory_state_feedback_and_claw_end_to_end():
    host, board = make_pair()
    emulator = BoardEmulator(
        board, state_rate_hz=50, execution_delay=0.01,
        claw_action_duration_s=0.01,
    )
    emulator.start()
    link = ControlLink(SerialSession(host, ack_timeout=0.1, retries=2))
    link.session.start()
    try:
        wait_ready(link)
        board.write(Frame(
            MessageType.MOTION_RESULT,
            65000,
            encode_motion_result(MotionResult(65000, 0, ResultCode.FAILED, 99)),
        ).encode())
        result = link.send_trajectory(points(), result_timeout=1.0)
        assert result.result_code == ResultCode.SUCCESS
        assert result.command_seq != 65000
        deadline = time.monotonic() + 0.5
        while link.latest_state.joint_positions != points()[-1].positions and time.monotonic() < deadline:
            time.sleep(0.01)
        assert link.latest_state.joint_positions == points()[-1].positions
        claw = link.send_gripper(0, result_timeout=1.0)
        assert claw.result_code == ClawResultCode.COMMAND_COMPLETED_UNVERIFIED
    finally:
        link.close()
        emulator.close()


def test_lost_ack_reuses_sequence_and_executes_once():
    host, board = make_pair(lambda: DropFirstFrameSerial(MessageType.ACK))
    emulator = BoardEmulator(board, state_rate_hz=50, execution_delay=0.01)
    emulator.start()
    link = ControlLink(SerialSession(host, ack_timeout=0.05, retries=3))
    link.session.start()
    try:
        wait_ready(link)
        assert link.send_trajectory(points(), result_timeout=1.0).result_code == ResultCode.SUCCESS
        assert emulator.motion_execution_count == 1
    finally:
        link.close()
        emulator.close()


def test_lost_claw_ack_reuses_sequence_and_executes_once():
    host, board = make_pair(lambda: DropFirstFrameSerial(MessageType.ACK))
    emulator = BoardEmulator(
        board, state_rate_hz=50, claw_action_duration_s=0.01,
    )
    emulator.start()
    link = ControlLink(SerialSession(host, ack_timeout=0.05, retries=3))
    link.session.start()
    try:
        wait_ready(link)
        result = link.send_gripper(0, result_timeout=1.0)
        assert result.result_code == ClawResultCode.COMMAND_COMPLETED_UNVERIFIED
        assert emulator.claw_execution_count == 1
    finally:
        link.close()
        emulator.close()


def test_claw_result_retransmits_with_command_sequence_until_ack():
    host, board = make_pair(lambda: DropFirstFrameSerial(MessageType.CLAW_RESULT))
    emulator = BoardEmulator(
        board, state_rate_hz=50, claw_action_duration_s=0.01,
    )
    emulator.start()
    link = ControlLink(SerialSession(host, ack_timeout=0.1, retries=1))
    link.session.start()
    try:
        wait_ready(link)
        result = link.send_gripper(100, result_timeout=1.0)
        assert result.result_code == ClawResultCode.COMMAND_COMPLETED_UNVERIFIED
        assert emulator.claw_execution_count == 1
    finally:
        link.close()
        emulator.close()


def test_motion_result_retransmits_until_ack():
    host, board = make_pair(lambda: DropFirstFrameSerial(MessageType.MOTION_RESULT))
    emulator = BoardEmulator(board, state_rate_hz=50, execution_delay=0.01)
    emulator.start()
    link = ControlLink(SerialSession(host, ack_timeout=0.1, retries=1))
    link.session.start()
    try:
        wait_ready(link)
        assert link.send_trajectory(points(), result_timeout=1.0).result_code == ResultCode.SUCCESS
        assert emulator.motion_execution_count == 1
    finally:
        link.close()
        emulator.close()


def test_estop_rejects_normal_commands_and_stop_does_not_clear_it():
    host, board = make_pair()
    emulator = BoardEmulator(board, state_rate_hz=50)
    emulator.start()
    session = SerialSession(host, ack_timeout=0.1, retries=0)
    session.start()
    try:
        emulator.trigger_estop()
        with pytest.raises(AckRejected):
            session.send_message(MessageType.STOP, require_ack=True)
        assert emulator.mode == RobotMode.ESTOP
    finally:
        session.close()
        emulator.close()
