import threading
import time

from jaka_single_arm.communication.board_emulator import BoardEmulator
from jaka_single_arm.communication.control_link import ControlLink
from jaka_single_arm.communication.protocol import (
    FruitClass,
    FruitTarget,
    ResultCode,
    TrajectoryPoint,
)
from jaka_single_arm.communication.transport import SerialSession


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


def make_pair():
    first, second = MemorySerial(), MemorySerial()
    first.peer, second.peer = second, first
    return first, second


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
            TrajectoryPoint(0, 100, (0.0, 0.0), (0.0, 0.0)),
            TrajectoryPoint(1, 200, (0.1, -0.1), (1.0, -1.0)),
        ], result_timeout=1.0)
        assert trajectory_result.result_code == ResultCode.SUCCESS
        assert link.latest_state.joint_positions == (0.1, -0.1)

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
