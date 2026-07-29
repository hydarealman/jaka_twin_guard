"""PC-side emulator for the future C-board firmware.

Use it with a virtual serial-port pair to validate both integration modes
before electrical-control firmware exists.  The emulator ACKs packets,
validates trajectory counts/checksums, emits robot state, and returns a
successful motion result without driving hardware.
"""

from __future__ import annotations

import argparse
import threading
import time

from jaka_single_arm.communication.protocol import (
    Ack,
    AckStatus,
    Frame,
    FrameFlags,
    FrameParser,
    FruitClass,
    MessageType,
    MotionResult,
    ResultCode,
    RobotMode,
    RobotState,
    TrajectoryBegin,
    TrajectoryPoint,
    decode_fruit_target,
    decode_trajectory_begin,
    decode_trajectory_end,
    decode_trajectory_point,
    encode_ack,
    encode_motion_result,
    encode_robot_state,
    trajectory_points_crc32,
)
from jaka_single_arm.communication.transport import open_serial


class BoardEmulator:
    def __init__(self, serial_device, state_rate_hz: float = 10.0, execution_delay: float = 0.1):
        self._serial = serial_device
        self._parser = FrameParser()
        self._state_period = 1.0 / max(0.1, state_rate_hz)
        self._execution_delay = max(0.0, execution_delay)
        self._running = threading.Event()
        self._write_lock = threading.Lock()
        self._seq = 0
        self._mode = RobotMode.READY
        self._error_code = 0
        self._joint_positions: tuple[float, ...] = ()
        self._tcp_xyz = (0, 0, 0)
        self._trajectory_begin: TrajectoryBegin | None = None
        self._trajectory_begin_seq = 0
        self._trajectory_payloads: dict[int, bytes] = {}
        self._trajectory_points: dict[int, TrajectoryPoint] = {}
        self._seen_sequences: set[int] = set()
        self._thread: threading.Thread | None = None
        self._state_thread: threading.Thread | None = None

    def start(self) -> None:
        if self._running.is_set():
            return
        self._running.set()
        self._thread = threading.Thread(target=self._read_loop, name="board-emulator-rx", daemon=True)
        self._state_thread = threading.Thread(target=self._state_loop, name="board-emulator-state", daemon=True)
        self._thread.start()
        self._state_thread.start()

    def close(self) -> None:
        self._running.clear()
        try:
            self._serial.close()
        finally:
            for thread in (self._thread, self._state_thread):
                if thread and thread is not threading.current_thread():
                    thread.join(timeout=1.0)

    def _read_loop(self) -> None:
        while self._running.is_set():
            try:
                data = self._serial.read(512)
                if not data:
                    continue
                for frame in self._parser.feed(data):
                    self._process(frame)
            except Exception:
                if self._running.is_set():
                    time.sleep(0.02)

    def _process(self, frame: Frame) -> None:
        duplicate = frame.seq in self._seen_sequences
        status, error, post_ack = AckStatus.OK, 0, None
        if not duplicate:
            try:
                status, error, post_ack = self._handle_new(frame)
            except Exception:
                status, error = AckStatus.BAD_PAYLOAD, 1
            if status == AckStatus.OK:
                self._seen_sequences.add(frame.seq)
                if len(self._seen_sequences) > 4096:
                    self._seen_sequences.clear()

        if frame.flags & FrameFlags.ACK_REQUIRED:
            self._send(
                MessageType.ACK,
                encode_ack(Ack(frame.seq, status, error)),
                FrameFlags.RESPONSE,
            )
        if not duplicate and status == AckStatus.OK and post_ack is not None:
            post_ack()

    def _handle_new(self, frame: Frame):
        if frame.msg_type in (MessageType.HELLO, MessageType.HEARTBEAT):
            return AckStatus.OK, 0, None
        if frame.msg_type == MessageType.ACK:
            return AckStatus.OK, 0, None
        if frame.msg_type == MessageType.FRUIT_TARGET:
            target = decode_fruit_target(frame.payload)
            if target.fruit_class == FruitClass.UNKNOWN:
                return AckStatus.BAD_PAYLOAD, int(ResultCode.INVALID_TARGET), None
            if self._mode != RobotMode.READY:
                return AckStatus.BUSY, 0, None
            self._mode = RobotMode.BUSY
            return AckStatus.OK, 0, lambda: self._finish_later(frame.seq, target.target_id, ())
        if frame.msg_type == MessageType.TRAJECTORY_BEGIN:
            if self._mode != RobotMode.READY:
                return AckStatus.BUSY, 0, None
            begin = decode_trajectory_begin(frame.payload)
            self._trajectory_begin = begin
            self._trajectory_begin_seq = frame.seq
            self._trajectory_payloads.clear()
            self._trajectory_points.clear()
            self._mode = RobotMode.BUSY
            return AckStatus.OK, 0, None
        if frame.msg_type == MessageType.TRAJECTORY_POINT:
            begin = self._trajectory_begin
            if begin is None:
                return AckStatus.BAD_PAYLOAD, 2, None
            trajectory_id, point = decode_trajectory_point(
                frame.payload, begin.joint_count, begin.include_velocities
            )
            if trajectory_id != begin.trajectory_id or point.index >= begin.point_count:
                return AckStatus.BAD_PAYLOAD, 3, None
            self._trajectory_payloads[point.index] = frame.payload
            self._trajectory_points[point.index] = point
            return AckStatus.OK, 0, None
        if frame.msg_type == MessageType.TRAJECTORY_END:
            begin = self._trajectory_begin
            if begin is None:
                return AckStatus.BAD_PAYLOAD, 4, None
            end = decode_trajectory_end(frame.payload)
            ordered_payloads = [self._trajectory_payloads[i] for i in sorted(self._trajectory_payloads)]
            valid = (
                end.trajectory_id == begin.trajectory_id
                and end.point_count == begin.point_count
                and len(ordered_payloads) == begin.point_count
                and trajectory_points_crc32(ordered_payloads) == end.points_crc32
            )
            if not valid:
                self._mode = RobotMode.ERROR
                self._error_code = 5
                return AckStatus.BAD_PAYLOAD, 5, None
            last = self._trajectory_points[begin.point_count - 1].positions
            command_seq = self._trajectory_begin_seq
            object_id = begin.trajectory_id
            self._trajectory_begin = None
            return AckStatus.OK, 0, lambda: self._finish_later(command_seq, object_id, last)
        if frame.msg_type == MessageType.ABORT:
            self._trajectory_begin = None
            self._mode = RobotMode.READY
            return AckStatus.OK, 0, None
        return AckStatus.UNSUPPORTED, 0, None

    def _finish_later(self, command_seq: int, object_id: int, final_joints: tuple[float, ...]) -> None:
        def finish():
            if self._execution_delay:
                time.sleep(self._execution_delay)
            if final_joints:
                self._joint_positions = tuple(final_joints)
            self._mode = RobotMode.READY
            self._send_state()
            self._send(
                MessageType.MOTION_RESULT,
                encode_motion_result(MotionResult(command_seq, object_id, ResultCode.SUCCESS, 0)),
                FrameFlags.RESPONSE | FrameFlags.ACK_REQUIRED,
            )

        threading.Thread(target=finish, name="board-emulator-motion", daemon=True).start()

    def _state_loop(self) -> None:
        while self._running.is_set():
            # A PTY pair can briefly report EIO while the host side is still
            # opening its endpoint.  Do not let that transient startup race
            # kill the state thread; the real firmware keeps retrying too.
            try:
                self._send_state()
            except Exception:
                if self._running.is_set():
                    time.sleep(0.05)
            time.sleep(self._state_period)

    def _send_state(self) -> None:
        state = RobotState(
            timestamp_ms=int(time.monotonic() * 1000),
            mode=self._mode,
            gripper_state=0,
            error_code=self._error_code,
            tcp_xyz_mm=self._tcp_xyz,
            joint_positions=self._joint_positions,
        )
        self._send(MessageType.ROBOT_STATE, encode_robot_state(state), FrameFlags.RESPONSE)

    def _send(self, msg_type: MessageType, payload: bytes, flags: FrameFlags) -> None:
        self._seq = (self._seq % 0xFFFF) + 1
        packet = Frame(msg_type, self._seq, payload, flags).encode()
        with self._write_lock:
            self._serial.write(packet)
            flush = getattr(self._serial, "flush", None)
            if callable(flush):
                flush()


def main(args=None) -> int:
    parser = argparse.ArgumentParser(description="JAKA serial C-board emulator")
    parser.add_argument("--port", required=True, help="Virtual/physical serial port")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--state-rate", type=float, default=10.0)
    parser.add_argument("--execution-delay", type=float, default=0.1)
    ns = parser.parse_args(args)
    device = open_serial(ns.port, ns.baudrate)
    emulator = BoardEmulator(device, ns.state_rate, ns.execution_delay)
    emulator.start()
    print(f"C-board emulator running on {ns.port} @ {ns.baudrate}")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        emulator.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
