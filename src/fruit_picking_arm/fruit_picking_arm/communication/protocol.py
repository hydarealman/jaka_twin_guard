"""Scheme-A fixed-length AA55 protocol shared with the STM32 C module."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, IntFlag
import math
import struct

MAGIC = b"\xAA\x55"
ANGLE_SCALE = 1_000_000
JOINT_COUNT = 6


class ProtocolError(ValueError):
    pass


class MessageType(IntEnum):
    TRAJECTORY_BEGIN = 0x01
    TRAJECTORY_POINT = 0x02
    TRAJECTORY_END = 0x03
    STOP = 0x04
    HEARTBEAT = 0x05
    CLAW_COMMAND = 0x06
    ACK = 0x80
    ROBOT_STATE = 0x81
    MOTION_RESULT = 0x82
    CLAW_RESULT = 0x83


class FrameFlags(IntFlag):
    """Compatibility shim; fixed protocol has no flags on the wire."""
    NONE = 0
    ACK_REQUIRED = 1
    RESPONSE = 2


class AckStatus(IntEnum):
    OK = 0x00
    BAD_DATA = 0x01
    BUSY = 0x02
    OUT_OF_RANGE = 0x03
    MISSING_POINT = 0x04
    FAULT = 0x05


class RobotMode(IntEnum):
    BOOTING = 0x00
    READY = 0x01
    BUSY = 0x02
    ERROR = 0x03
    ESTOP = 0x04


class ResultCode(IntEnum):
    SUCCESS = 0x00
    FAILED = 0x01
    CANCELLED = 0x02
    ESTOP = 0x03
    TIMEOUT = 0x04
    # Legacy architecture-B names retained for import compatibility only.
    UNREACHABLE = 0x01
    GRASP_MISSED = 0x01
    OBJECT_DROPPED = 0x01
    INVALID_TARGET = 0x01
    TRAJECTORY_REJECTED = 0x01
    INTERNAL_ERROR = 0x01


class FruitClass(IntEnum):
    UNKNOWN = 0
    HEALTHY = 1
    UNHEALTHY = 2


@dataclass(frozen=True)
class FruitTarget:
    target_id: int
    fruit_class: FruitClass
    confidence: float
    x_mm: int
    y_mm: int
    z_mm: int
    radius_mm: int
    ttl_ms: int = 1000
    capture_time_ms: int = 0


class ClawAction(IntEnum):
    OPEN = 0x01
    CLOSE = 0x02
    STOP = 0x03


class ClawResultCode(IntEnum):
    COMMAND_COMPLETED_UNVERIFIED = 0x00
    INTERRUPTED = 0x01
    TIMEOUT = 0x02
    FAULT = 0x03


PAYLOAD_SIZES = {
    MessageType.TRAJECTORY_BEGIN: 2,
    MessageType.TRAJECTORY_POINT: 54,
    MessageType.TRAJECTORY_END: 0,
    MessageType.STOP: 0,
    MessageType.HEARTBEAT: 0,
    MessageType.CLAW_COMMAND: 1,
    MessageType.ACK: 1,
    MessageType.ROBOT_STATE: 27,
    MessageType.MOTION_RESULT: 3,
    MessageType.CLAW_RESULT: 1,
}


@dataclass(frozen=True)
class Frame:
    msg_type: MessageType
    seq: int
    payload: bytes = b""
    flags: FrameFlags = FrameFlags.NONE

    def encode(self) -> bytes:
        try:
            msg_type = MessageType(self.msg_type)
        except ValueError as exc:
            raise ProtocolError("unknown message type") from exc
        if not 0 <= self.seq <= 0xFFFF:
            raise ProtocolError("sequence is outside uint16")
        expected = PAYLOAD_SIZES[msg_type]
        if len(self.payload) != expected:
            raise ProtocolError(
                f"{msg_type.name} payload must be {expected} bytes, got {len(self.payload)}"
            )
        body = struct.pack("<BH", int(msg_type), self.seq) + bytes(self.payload)
        return MAGIC + body + struct.pack("<H", crc16_ccitt(body))

    @classmethod
    def decode(cls, packet: bytes) -> "Frame":
        parser = FrameParser()
        frames = parser.feed(packet)
        if len(frames) != 1 or parser.discarded_bytes or parser.crc_errors:
            raise ProtocolError("invalid or incomplete frame")
        return frames[0]


class FrameParser:
    def __init__(self) -> None:
        self._buffer = bytearray()
        self.crc_errors = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> list[Frame]:
        self._buffer.extend(data)
        frames: list[Frame] = []
        while len(self._buffer) >= 7:
            marker = self._buffer.find(MAGIC)
            if marker < 0:
                keep = 1 if self._buffer[-1] == 0xAA else 0
                self.discarded_bytes += len(self._buffer) - keep
                del self._buffer[: len(self._buffer) - keep]
                break
            if marker:
                self.discarded_bytes += marker
                del self._buffer[:marker]
            if len(self._buffer) < 7:
                break
            try:
                msg_type = MessageType(self._buffer[2])
            except ValueError:
                self.discarded_bytes += 1
                del self._buffer[0]
                continue
            payload_size = PAYLOAD_SIZES[msg_type]
            total = payload_size + 7
            if len(self._buffer) < total:
                break
            body = bytes(self._buffer[2 : 5 + payload_size])
            expected_crc = struct.unpack_from("<H", self._buffer, 5 + payload_size)[0]
            if crc16_ccitt(body) != expected_crc:
                self.crc_errors += 1
                self.discarded_bytes += 1
                del self._buffer[0]
                continue
            frames.append(Frame(msg_type, struct.unpack_from("<H", body, 1)[0], body[3:]))
            del self._buffer[:total]
        return frames


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Ack:
    status: AckStatus = AckStatus.OK


def encode_ack(value: Ack) -> bytes:
    return struct.pack("<B", int(value.status))


def decode_ack(payload: bytes) -> Ack:
    _size(payload, 1, "ACK")
    try:
        return Ack(AckStatus(payload[0]))
    except ValueError as exc:
        raise ProtocolError("unknown ACK status") from exc


@dataclass(frozen=True)
class TrajectoryBegin:
    point_count: int


def encode_trajectory_begin(value: TrajectoryBegin) -> bytes:
    if not 1 <= value.point_count <= 0xFFFF:
        raise ProtocolError("point_count is outside uint16 or zero")
    return struct.pack("<H", value.point_count)


def decode_trajectory_begin(payload: bytes) -> TrajectoryBegin:
    _size(payload, 2, "TRAJECTORY_BEGIN")
    value = TrajectoryBegin(struct.unpack("<H", payload)[0])
    if value.point_count == 0:
        raise ProtocolError("point_count cannot be zero")
    return value


@dataclass(frozen=True)
class TrajectoryPoint:
    index: int
    time_ms: int
    positions: tuple[float, ...]
    velocities: tuple[float, ...]


def encode_trajectory_point(value: TrajectoryPoint) -> bytes:
    if not 0 <= value.index <= 0xFFFF or not 0 <= value.time_ms <= 0xFFFFFFFF:
        raise ProtocolError("trajectory index or time is outside wire range")
    if len(value.positions) != JOINT_COUNT or len(value.velocities) != JOINT_COUNT:
        raise ProtocolError("trajectory point must contain six positions and velocities")
    raw = [_angle_to_i32(v) for v in (*value.positions, *value.velocities)]
    return struct.pack("<HI12i", value.index, value.time_ms, *raw)


def decode_trajectory_point(payload: bytes) -> TrajectoryPoint:
    _size(payload, 54, "TRAJECTORY_POINT")
    values = struct.unpack("<HI12i", payload)
    return TrajectoryPoint(
        values[0], values[1],
        tuple(v / ANGLE_SCALE for v in values[2:8]),
        tuple(v / ANGLE_SCALE for v in values[8:14]),
    )


@dataclass(frozen=True)
class MotionResult:
    command_seq: int
    object_id: int
    result_code: ResultCode
    error_code: int = 0


def encode_motion_result(value: MotionResult) -> bytes:
    return struct.pack("<BH", int(value.result_code), value.error_code)


def decode_motion_result(payload: bytes) -> MotionResult:
    _size(payload, 3, "MOTION_DONE")
    result, error = struct.unpack("<BH", payload)
    try:
        return MotionResult(0, 0, ResultCode(result), error)
    except ValueError as exc:
        raise ProtocolError("unknown motion result") from exc


@dataclass(frozen=True)
class RobotState:
    mode: RobotMode
    error_code: int
    joint_positions: tuple[float, ...]


def encode_robot_state(value: RobotState) -> bytes:
    if len(value.joint_positions) != JOINT_COUNT:
        raise ProtocolError("ROBOT_STATE must contain six joints")
    return struct.pack(
        "<BH6i", int(value.mode), value.error_code,
        *(_angle_to_i32(v) for v in value.joint_positions),
    )


def decode_robot_state(payload: bytes) -> RobotState:
    _size(payload, 27, "ROBOT_STATE")
    mode, error, *joints = struct.unpack("<BH6i", payload)
    try:
        return RobotState(RobotMode(mode), error, tuple(v / ANGLE_SCALE for v in joints))
    except ValueError as exc:
        raise ProtocolError("unknown robot mode") from exc


@dataclass(frozen=True)
class ClawCommand:
    action: ClawAction


def encode_claw_command(value: ClawCommand) -> bytes:
    return struct.pack("<B", int(value.action))


def decode_claw_command(payload: bytes) -> ClawCommand:
    _size(payload, 1, "CLAW_COMMAND")
    try:
        return ClawCommand(ClawAction(payload[0]))
    except ValueError as exc:
        raise ProtocolError("unknown claw action") from exc


@dataclass(frozen=True)
class ClawResult:
    result_code: ClawResultCode


def encode_claw_result(value: ClawResult) -> bytes:
    return struct.pack("<B", int(value.result_code))


def decode_claw_result(payload: bytes) -> ClawResult:
    _size(payload, 1, "CLAW_RESULT")
    try:
        return ClawResult(ClawResultCode(payload[0]))
    except ValueError as exc:
        raise ProtocolError("unknown claw result") from exc


def _angle_to_i32(value: float) -> int:
    if not math.isfinite(value):
        raise ProtocolError("joint value is not finite")
    scaled = round(value * ANGLE_SCALE)
    if not -(1 << 31) <= scaled < (1 << 31):
        raise ProtocolError("scaled joint value is outside int32")
    return scaled


def _size(payload: bytes, expected: int, name: str) -> None:
    if len(payload) != expected:
        raise ProtocolError(f"{name} payload must be {expected} bytes")


def stable_u16_id(text: str) -> int:
    value = 0xFFFF
    for byte in text.encode("utf-8"):
        value = ((value << 5) ^ (value >> 11) ^ byte) & 0xFFFF
    return value or 1
