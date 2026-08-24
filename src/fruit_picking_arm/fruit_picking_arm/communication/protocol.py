"""Binary serial protocol used by both proposed control architectures.

Architecture A sends a MoveIt generated joint trajectory.  Architecture B
sends a stable fruit target in the robot base frame.  Both use the same frame,
ACK, result, heartbeat and robot-state messages.

All multi-byte fields are little-endian.  Cartesian units are millimetres,
angles are encoded as microradians, confidence is 0..1000, and time is
milliseconds.  CRC is CRC-16/CCITT-FALSE over the header after the magic bytes
plus the payload.
"""

from __future__ import annotations

import math
import struct
import zlib
from dataclasses import dataclass, field
from enum import IntEnum, IntFlag
from typing import Iterable


MAGIC = b"\xAA\x55"
PROTOCOL_VERSION = 1
MAX_PAYLOAD = 4096
ANGLE_SCALE = 1_000_000.0  # radians -> signed microradians

# magic, version, message type, flags, sequence, payload length
_HEADER = struct.Struct("<2sBBBHH")
_CRC = struct.Struct("<H")


class ProtocolError(ValueError):
    """Raised when a packet or payload violates the protocol contract."""


class MessageType(IntEnum):
    HELLO = 0x01
    HEARTBEAT = 0x02
    ACK = 0x03
    ROBOT_STATE = 0x04
    ERROR = 0x05

    FRUIT_TARGET = 0x10

    TRAJECTORY_BEGIN = 0x20
    TRAJECTORY_POINT = 0x21
    TRAJECTORY_END = 0x22
    ABORT = 0x23
    GRIPPER_COMMAND = 0x24

    MOTION_RESULT = 0x30


class FrameFlags(IntFlag):
    NONE = 0
    ACK_REQUIRED = 1 << 0
    RESPONSE = 1 << 1


class AckStatus(IntEnum):
    OK = 0
    BAD_PAYLOAD = 1
    BUSY = 2
    UNSUPPORTED = 3
    CRC_ERROR = 4
    OUT_OF_RANGE = 5


class FruitClass(IntEnum):
    HEALTHY = 0
    UNHEALTHY = 1
    UNKNOWN = 2


class RobotMode(IntEnum):
    BOOTING = 0
    READY = 1
    BUSY = 2
    ERROR = 3
    ESTOP = 4


class ResultCode(IntEnum):
    SUCCESS = 0
    UNREACHABLE = 1
    GRASP_MISSED = 2
    OBJECT_DROPPED = 3
    ESTOP = 4
    INVALID_TARGET = 5
    TRAJECTORY_REJECTED = 6
    CANCELLED = 7
    TIMEOUT = 8
    INTERNAL_ERROR = 9


class GripperMode(IntEnum):
    STOP = 0
    OPEN = 1
    CLOSE = 2
    POSITION = 3


@dataclass(frozen=True)
class Frame:
    msg_type: MessageType
    seq: int
    payload: bytes = b""
    flags: FrameFlags = FrameFlags.NONE
    version: int = PROTOCOL_VERSION

    def encode(self) -> bytes:
        if int(self.version) != PROTOCOL_VERSION:
            raise ProtocolError(f"unsupported protocol version: {self.version}")
        try:
            message_type = MessageType(int(self.msg_type))
        except ValueError as exc:
            raise ProtocolError(f"unknown message type: 0x{int(self.msg_type):02X}") from exc
        if not 0 <= int(self.seq) <= 0xFFFF:
            raise ProtocolError(f"sequence out of range: {self.seq}")
        if int(self.flags) & ~int(FrameFlags.ACK_REQUIRED | FrameFlags.RESPONSE):
            raise ProtocolError(f"unknown frame flags: 0x{int(self.flags):02X}")
        if len(self.payload) > MAX_PAYLOAD:
            raise ProtocolError(f"payload too large: {len(self.payload)}")
        header = _HEADER.pack(
            MAGIC,
            int(self.version),
            int(message_type),
            int(self.flags),
            int(self.seq),
            len(self.payload),
        )
        crc = crc16_ccitt(header[2:] + self.payload)
        return header + self.payload + _CRC.pack(crc)

    @classmethod
    def decode(cls, packet: bytes) -> "Frame":
        if len(packet) < _HEADER.size + _CRC.size:
            raise ProtocolError("packet is shorter than the frame header")
        magic, version, raw_type, flags, seq, length = _HEADER.unpack_from(packet)
        if magic != MAGIC:
            raise ProtocolError("invalid frame magic")
        expected = _HEADER.size + length + _CRC.size
        if len(packet) != expected:
            raise ProtocolError(f"frame length mismatch: {len(packet)} != {expected}")
        if length > MAX_PAYLOAD:
            raise ProtocolError(f"payload too large: {length}")
        if flags & ~int(FrameFlags.ACK_REQUIRED | FrameFlags.RESPONSE):
            raise ProtocolError(f"unknown frame flags: 0x{flags:02X}")
        payload = packet[_HEADER.size : _HEADER.size + length]
        received_crc = _CRC.unpack_from(packet, _HEADER.size + length)[0]
        computed_crc = crc16_ccitt(packet[2 : _HEADER.size] + payload)
        if received_crc != computed_crc:
            raise ProtocolError(
                f"CRC mismatch: received=0x{received_crc:04X}, "
                f"computed=0x{computed_crc:04X}"
            )
        if version != PROTOCOL_VERSION:
            raise ProtocolError(f"unsupported protocol version: {version}")
        try:
            msg_type = MessageType(raw_type)
        except ValueError as exc:
            raise ProtocolError(f"unknown message type: 0x{raw_type:02X}") from exc
        return cls(msg_type, seq, payload, FrameFlags(flags), version)


class FrameParser:
    """Incremental byte-stream parser with corruption re-synchronisation."""

    def __init__(self, max_payload: int = MAX_PAYLOAD):
        self._buffer = bytearray()
        self._max_payload = max_payload
        self.crc_errors = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> list[Frame]:
        if data:
            self._buffer.extend(data)
        frames: list[Frame] = []
        minimum = _HEADER.size + _CRC.size
        while len(self._buffer) >= minimum:
            start = self._buffer.find(MAGIC)
            if start < 0:
                keep = 1 if self._buffer[-1:] == MAGIC[:1] else 0
                discarded = len(self._buffer) - keep
                self.discarded_bytes += discarded
                del self._buffer[:discarded]
                break
            if start:
                self.discarded_bytes += start
                del self._buffer[:start]
            if len(self._buffer) < minimum:
                break
            _, version, raw_type, flags, seq, length = _HEADER.unpack_from(self._buffer)
            if length > self._max_payload:
                self.discarded_bytes += 1
                del self._buffer[0]
                continue
            total = _HEADER.size + length + _CRC.size
            if len(self._buffer) < total:
                break
            packet = bytes(self._buffer[:total])
            try:
                frame = Frame.decode(packet)
            except ProtocolError:
                self.crc_errors += 1
                self.discarded_bytes += 1
                del self._buffer[0]
                continue
            frames.append(frame)
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
    acked_seq: int
    status: AckStatus = AckStatus.OK
    error_code: int = 0


_ACK_PAYLOAD = struct.Struct("<HBH")


def encode_ack(value: Ack) -> bytes:
    _range(value.acked_seq, 0, 0xFFFF, "acked_seq")
    _range(value.error_code, 0, 0xFFFF, "error_code")
    return _ACK_PAYLOAD.pack(value.acked_seq, int(value.status), value.error_code)


def decode_ack(payload: bytes) -> Ack:
    _expect_size(payload, _ACK_PAYLOAD.size, "ACK")
    seq, status, error = _ACK_PAYLOAD.unpack(payload)
    return Ack(seq, AckStatus(status), error)


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
    flags: int = 0


_FRUIT_TARGET = struct.Struct("<HBBHiiiHHI")


def encode_fruit_target(value: FruitTarget) -> bytes:
    confidence = int(round(_clamp(value.confidence, 0.0, 1.0) * 1000.0))
    _range(value.target_id, 0, 0xFFFF, "target_id")
    _range(value.radius_mm, 0, 0xFFFF, "radius_mm")
    _range(value.ttl_ms, 0, 0xFFFF, "ttl_ms")
    return _FRUIT_TARGET.pack(
        value.target_id,
        int(value.fruit_class),
        value.flags & 0xFF,
        confidence,
        value.x_mm,
        value.y_mm,
        value.z_mm,
        value.radius_mm,
        value.ttl_ms,
        value.capture_time_ms & 0xFFFFFFFF,
    )


def decode_fruit_target(payload: bytes) -> FruitTarget:
    _expect_size(payload, _FRUIT_TARGET.size, "FRUIT_TARGET")
    target_id, fruit_class, flags, confidence, x, y, z, radius, ttl, capture = _FRUIT_TARGET.unpack(payload)
    return FruitTarget(
        target_id=target_id,
        fruit_class=FruitClass(fruit_class),
        confidence=confidence / 1000.0,
        x_mm=x,
        y_mm=y,
        z_mm=z,
        radius_mm=radius,
        ttl_ms=ttl,
        capture_time_ms=capture,
        flags=flags,
    )


@dataclass(frozen=True)
class TrajectoryBegin:
    trajectory_id: int
    point_count: int
    joint_count: int
    include_velocities: bool = True


_TRAJECTORY_BEGIN = struct.Struct("<HHBB")


def encode_trajectory_begin(value: TrajectoryBegin) -> bytes:
    _range(value.trajectory_id, 1, 0xFFFF, "trajectory_id")
    if not 1 <= value.joint_count <= 16:
        raise ProtocolError(f"joint_count out of range: {value.joint_count}")
    if not 1 <= value.point_count <= 0xFFFF:
        raise ProtocolError(f"point_count out of range: {value.point_count}")
    return _TRAJECTORY_BEGIN.pack(
        value.trajectory_id,
        value.point_count,
        value.joint_count,
        1 if value.include_velocities else 0,
    )


def decode_trajectory_begin(payload: bytes) -> TrajectoryBegin:
    _expect_size(payload, _TRAJECTORY_BEGIN.size, "TRAJECTORY_BEGIN")
    trajectory_id, point_count, joint_count, flags = _TRAJECTORY_BEGIN.unpack(payload)
    _range(trajectory_id, 1, 0xFFFF, "trajectory_id")
    _range(point_count, 1, 0xFFFF, "point_count")
    _range(joint_count, 1, 16, "joint_count")
    if flags & ~1:
        raise ProtocolError(f"unknown TRAJECTORY_BEGIN flags: 0x{flags:02X}")
    return TrajectoryBegin(trajectory_id, point_count, joint_count, bool(flags & 1))


@dataclass(frozen=True)
class TrajectoryPoint:
    index: int
    time_ms: int
    positions: tuple[float, ...]
    velocities: tuple[float, ...] = field(default_factory=tuple)


_TRAJECTORY_POINT_PREFIX = struct.Struct("<HHI")


def encode_trajectory_point(trajectory_id: int, value: TrajectoryPoint, include_velocities: bool = True) -> bytes:
    if not value.positions:
        raise ProtocolError("trajectory point has no joint positions")
    _range(trajectory_id, 1, 0xFFFF, "trajectory_id")
    _range(value.index, 0, 0xFFFF, "point index")
    _range(value.time_ms, 0, 0xFFFFFFFF, "time_ms")
    if len(value.positions) > 16:
        raise ProtocolError("too many joints in trajectory point")
    if include_velocities and value.velocities and len(value.velocities) != len(value.positions):
        raise ProtocolError("position/velocity joint counts do not match")
    positions = [_angle_to_i32(v) for v in value.positions]
    payload = bytearray(_TRAJECTORY_POINT_PREFIX.pack(trajectory_id, value.index, value.time_ms))
    payload.extend(struct.pack(f"<{len(positions)}i", *positions))
    if include_velocities:
        velocities = value.velocities or tuple(0.0 for _ in positions)
        payload.extend(struct.pack(f"<{len(positions)}i", *[_angle_to_i32(v) for v in velocities]))
    return bytes(payload)


def decode_trajectory_point(payload: bytes, joint_count: int, include_velocities: bool = True) -> tuple[int, TrajectoryPoint]:
    _range(joint_count, 1, 16, "joint_count")
    values_per_joint = 2 if include_velocities else 1
    expected = _TRAJECTORY_POINT_PREFIX.size + joint_count * 4 * values_per_joint
    _expect_size(payload, expected, "TRAJECTORY_POINT")
    trajectory_id, index, time_ms = _TRAJECTORY_POINT_PREFIX.unpack_from(payload)
    offset = _TRAJECTORY_POINT_PREFIX.size
    raw_positions = struct.unpack_from(f"<{joint_count}i", payload, offset)
    offset += joint_count * 4
    raw_velocities: tuple[int, ...] = ()
    if include_velocities:
        raw_velocities = struct.unpack_from(f"<{joint_count}i", payload, offset)
    point = TrajectoryPoint(
        index=index,
        time_ms=time_ms,
        positions=tuple(v / ANGLE_SCALE for v in raw_positions),
        velocities=tuple(v / ANGLE_SCALE for v in raw_velocities),
    )
    return trajectory_id, point


@dataclass(frozen=True)
class TrajectoryEnd:
    trajectory_id: int
    point_count: int
    points_crc32: int


_TRAJECTORY_END = struct.Struct("<HHI")


def encode_trajectory_end(value: TrajectoryEnd) -> bytes:
    _range(value.trajectory_id, 1, 0xFFFF, "trajectory_id")
    _range(value.point_count, 1, 0xFFFF, "point_count")
    _range(value.points_crc32, 0, 0xFFFFFFFF, "points_crc32")
    return _TRAJECTORY_END.pack(value.trajectory_id, value.point_count, value.points_crc32)


def decode_trajectory_end(payload: bytes) -> TrajectoryEnd:
    _expect_size(payload, _TRAJECTORY_END.size, "TRAJECTORY_END")
    value = TrajectoryEnd(*_TRAJECTORY_END.unpack(payload))
    _range(value.trajectory_id, 1, 0xFFFF, "trajectory_id")
    _range(value.point_count, 1, 0xFFFF, "point_count")
    return value


def trajectory_points_crc32(payloads: Iterable[bytes]) -> int:
    checksum = 0
    for payload in payloads:
        checksum = zlib.crc32(payload, checksum)
    return checksum & 0xFFFFFFFF


@dataclass(frozen=True)
class GripperCommand:
    command_id: int
    mode: GripperMode
    opening_mm: int
    speed_mm_s: int = 100
    force_permille: int = 500


_GRIPPER_COMMAND = struct.Struct("<HBHHH")


def encode_gripper_command(value: GripperCommand) -> bytes:
    _range(value.command_id, 1, 0xFFFF, "command_id")
    _range(value.opening_mm, 0, 100, "opening_mm")
    _range(value.speed_mm_s, 0, 0xFFFF, "speed_mm_s")
    _range(value.force_permille, 0, 1000, "force_permille")
    return _GRIPPER_COMMAND.pack(
        value.command_id,
        int(value.mode),
        value.opening_mm,
        value.speed_mm_s,
        value.force_permille,
    )


def decode_gripper_command(payload: bytes) -> GripperCommand:
    _expect_size(payload, _GRIPPER_COMMAND.size, "GRIPPER_COMMAND")
    command_id, mode, opening_mm, speed_mm_s, force_permille = (
        _GRIPPER_COMMAND.unpack(payload)
    )
    if opening_mm > 100:
        raise ProtocolError(f"opening_mm out of range: {opening_mm}")
    if force_permille > 1000:
        raise ProtocolError(f"force_permille out of range: {force_permille}")
    _range(command_id, 1, 0xFFFF, "command_id")
    return GripperCommand(
        command_id=command_id,
        mode=GripperMode(mode),
        opening_mm=opening_mm,
        speed_mm_s=speed_mm_s,
        force_permille=force_permille,
    )


@dataclass(frozen=True)
class MotionResult:
    command_seq: int
    object_id: int
    result_code: ResultCode
    error_code: int = 0


_MOTION_RESULT = struct.Struct("<HHBH")


def encode_motion_result(value: MotionResult) -> bytes:
    _range(value.command_seq, 0, 0xFFFF, "command_seq")
    _range(value.object_id, 0, 0xFFFF, "object_id")
    _range(value.error_code, 0, 0xFFFF, "error_code")
    return _MOTION_RESULT.pack(value.command_seq, value.object_id, int(value.result_code), value.error_code)


def decode_motion_result(payload: bytes) -> MotionResult:
    _expect_size(payload, _MOTION_RESULT.size, "MOTION_RESULT")
    command_seq, object_id, result_code, error_code = _MOTION_RESULT.unpack(payload)
    return MotionResult(command_seq, object_id, ResultCode(result_code), error_code)


@dataclass(frozen=True)
class RobotState:
    timestamp_ms: int
    mode: RobotMode
    gripper_state: int
    error_code: int
    tcp_xyz_mm: tuple[int, int, int] = (0, 0, 0)
    tcp_rpy_mdeg: tuple[int, int, int] = (0, 0, 0)
    joint_positions: tuple[float, ...] = field(default_factory=tuple)


_ROBOT_STATE_PREFIX = struct.Struct("<IBBH6iB")


def encode_robot_state(value: RobotState) -> bytes:
    if len(value.tcp_xyz_mm) != 3 or len(value.tcp_rpy_mdeg) != 3:
        raise ProtocolError("TCP state must contain XYZ and RPY triples")
    if len(value.joint_positions) > 16:
        raise ProtocolError("too many joint positions in ROBOT_STATE")
    if value.gripper_state != 255:
        _range(value.gripper_state, 0, 100, "gripper_state")
    _range(value.error_code, 0, 0xFFFF, "error_code")
    if any(not -(2**31) <= int(coordinate) <= 2**31 - 1
           for coordinate in (*value.tcp_xyz_mm, *value.tcp_rpy_mdeg)):
        raise ProtocolError("TCP state value is outside int32 range")
    payload = bytearray(_ROBOT_STATE_PREFIX.pack(
        value.timestamp_ms & 0xFFFFFFFF,
        int(value.mode),
        value.gripper_state & 0xFF,
        value.error_code & 0xFFFF,
        *value.tcp_xyz_mm,
        *value.tcp_rpy_mdeg,
        len(value.joint_positions),
    ))
    if value.joint_positions:
        payload.extend(struct.pack(
            f"<{len(value.joint_positions)}i",
            *[_angle_to_i32(v) for v in value.joint_positions],
        ))
    return bytes(payload)


def decode_robot_state(payload: bytes) -> RobotState:
    if len(payload) < _ROBOT_STATE_PREFIX.size:
        raise ProtocolError("ROBOT_STATE payload is truncated")
    unpacked = _ROBOT_STATE_PREFIX.unpack_from(payload)
    timestamp, mode, gripper, error = unpacked[:4]
    tcp_xyz = tuple(unpacked[4:7])
    tcp_rpy = tuple(unpacked[7:10])
    joint_count = unpacked[10]
    if joint_count > 16:
        raise ProtocolError(f"too many joints in ROBOT_STATE: {joint_count}")
    if gripper != 255:
        _range(gripper, 0, 100, "gripper_state")
    expected = _ROBOT_STATE_PREFIX.size + joint_count * 4
    _expect_size(payload, expected, "ROBOT_STATE")
    raw_joints = struct.unpack_from(f"<{joint_count}i", payload, _ROBOT_STATE_PREFIX.size) if joint_count else ()
    return RobotState(
        timestamp_ms=timestamp,
        mode=RobotMode(mode),
        gripper_state=gripper,
        error_code=error,
        tcp_xyz_mm=tcp_xyz,
        tcp_rpy_mdeg=tcp_rpy,
        joint_positions=tuple(v / ANGLE_SCALE for v in raw_joints),
    )


def stable_u16_id(text: str) -> int:
    """Create a deterministic non-zero uint16 id from an arbitrary track id."""
    value = crc16_ccitt(text.encode("utf-8"))
    return value or 1


def _angle_to_i32(value: float) -> int:
    if not math.isfinite(value):
        raise ProtocolError(f"non-finite joint value: {value}")
    scaled = int(round(value * ANGLE_SCALE))
    _range(scaled, -(2**31), 2**31 - 1, "scaled joint value")
    return scaled


def _expect_size(payload: bytes, expected: int, name: str) -> None:
    if len(payload) != expected:
        raise ProtocolError(f"{name} payload length {len(payload)} != {expected}")


def _range(value: int, lower: int, upper: int, name: str) -> None:
    if not lower <= value <= upper:
        raise ProtocolError(f"{name} out of range: {value}")


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))
