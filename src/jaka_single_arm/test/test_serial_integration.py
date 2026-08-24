import threading
import time

import pytest

from jaka_single_arm.communication.board_emulator import BoardEmulator
from jaka_single_arm.communication.control_link import ControlLink
from jaka_single_arm.communication.protocol import (
    Frame,
    FruitClass,
    FruitTarget,
    MessageType,
    ResultCode,
    TrajectoryPoint,
    TrajectoryBegin,
    encode_trajectory_begin,
)
from jaka_single_arm.communication.transport import AckRejected, SerialSession


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


def make_pair():
    first, second = MemorySerial(), MemorySerial()
    first.peer, second.peer = second, first
    return first, second


def make_faulty_pair(board_type):
    host, board = MemorySerial(), DropFirstFrameSerial(board_type)
    host.peer, board.peer = board, host
    return host, board


def test_serial_reader_fails_closed_on_device_error():
    session = SerialSession(ReadFailSerial())
    session.start()
    deadline = time.monotonic() + 1.0
    while session.is_running and time.monotonic() < deadline:
        time.sleep(0.01)
    assert not session.is_running
    assert "device disconnected" in (session.fatal_error or "")
    session.close()


def test_both_control_modes_against_board_emulator():
    host, board = make_pair()
    emulator = BoardEmulator(board, state_rate_hz=50.0, execution_delay=0.01)
    emulator.start()
    session = SerialSession(host, ack_timeout=0.1, retries=1)
    session.start()
    link = ControlLink(session)
    try:
        deadline = time.monotonic() + 1.0
        while not link.ready and time.monotonic() < deadline:
            time.sleep(0.01)
        assert link.ready

        target_result = link.send_fruit_target(
            FruitTarget(7, FruitClass.HEALTHY, 0.9, 500, 0, 340, 35),
            result_timeout=1.0,
        )
        assert target_result.result_code == ResultCode.SUCCESS

        trajectory_result = link.send_trajectory([
            TrajectoryPoint(0, 100, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), (0.0,) * 6),
            TrajectoryPoint(1, 200, (0.1, -0.1, 0.2, -0.2, 0.3, -0.3), (1.0, -1.0, 2.0, -2.0, 3.0, -3.0)),
        ], result_timeout=1.0)
        assert trajectory_result.result_code == ResultCode.SUCCESS
        assert link.latest_state.joint_positions == (0.1, -0.1, 0.2, -0.2, 0.3, -0.3)

        gripper_result = link.send_gripper(
            opening_mm=68,
            speed_mm_s=80,
            force_permille=650,
            result_timeout=1.0,
        )
        assert gripper_result.result_code == ResultCode.SUCCESS
        assert link.latest_state.gripper_state == 68
    finally:
        link.close()
        emulator.close()


def test_duplicate_command_is_acked_without_double_execution():
    host, board = make_faulty_pair(MessageType.ACK)
    emulator = BoardEmulator(board, state_rate_hz=50.0, execution_delay=0.01)
    emulator.start()
    session = SerialSession(host, ack_timeout=0.05, retries=2)
    session.start()
    link = ControlLink(session)
    try:
        deadline = time.monotonic() + 1.0
        while not link.ready and time.monotonic() < deadline:
            time.sleep(0.01)
        result = link.send_trajectory([
            TrajectoryPoint(0, 100, (0.0,) * 6, (0.0,) * 6),
            TrajectoryPoint(1, 200, (0.1, -0.1, 0.2, -0.2, 0.3, -0.3), (0.0,) * 6),
        ], result_timeout=1.0)
        assert result.result_code == ResultCode.SUCCESS
        assert emulator.motion_execution_count == 1
    finally:
        link.close()
        emulator.close()


def test_motion_result_is_retransmitted_when_first_result_is_lost():
    host, board = make_faulty_pair(MessageType.MOTION_RESULT)
    emulator = BoardEmulator(board, state_rate_hz=50.0, execution_delay=0.01)
    emulator.start()
    session = SerialSession(host, ack_timeout=0.1, retries=1)
    session.start()
    link = ControlLink(session)
    try:
        deadline = time.monotonic() + 1.0
        while not link.ready and time.monotonic() < deadline:
            time.sleep(0.01)
        result = link.send_trajectory([
            TrajectoryPoint(0, 100, (0.0,) * 6, (0.0,) * 6),
            TrajectoryPoint(1, 200, (0.1, -0.1, 0.2, -0.2, 0.3, -0.3), (0.0,) * 6),
        ], result_timeout=1.0)
        assert result.result_code == ResultCode.SUCCESS
        assert emulator.motion_execution_count == 1
    finally:
        link.close()
        emulator.close()


def test_emulator_watchdog_enters_error_and_abort_cannot_clear_it():
    host, board = make_pair()
    emulator = BoardEmulator(
        board, state_rate_hz=50.0, execution_delay=0.01, watchdog_timeout_s=0.05
    )
    emulator.start()
    session = SerialSession(host, ack_timeout=0.1, retries=0)
    session.start()
    try:
        session.send_message(
            MessageType.TRAJECTORY_BEGIN,
            encode_trajectory_begin(TrajectoryBegin(11, 1, 6, True)),
            require_ack=True,
        )
        deadline = time.monotonic() + 1.0
        while emulator.mode != 3 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert emulator.mode.name == "ERROR"
        with pytest.raises(AckRejected):
            session.send_message(MessageType.ABORT, require_ack=True)
        assert emulator.mode.name == "ERROR"
    finally:
        session.close()
        emulator.close()


def test_physical_estop_is_not_cleared_by_abort():
    host, board = make_pair()
    emulator = BoardEmulator(board, state_rate_hz=50.0)
    emulator.start()
    session = SerialSession(host, ack_timeout=0.1, retries=0)
    session.start()
    try:
        emulator.trigger_estop()
        with pytest.raises(AckRejected):
            session.send_message(MessageType.ABORT, require_ack=True)
        assert emulator.mode.name == "ESTOP"
    finally:
        session.close()
        emulator.close()
