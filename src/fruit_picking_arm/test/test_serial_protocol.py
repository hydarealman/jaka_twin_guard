import pytest

from fruit_picking_arm.communication.protocol import (
    ClawAction,
    ClawCommand,
    ClawResult,
    ClawResultCode,
    ClawState,
    ClawStateCode,
    Frame,
    FrameParser,
    MessageType,
    RobotMode,
    RobotState,
    TrajectoryPoint,
    decode_claw_command,
    decode_claw_result,
    decode_claw_state,
    decode_robot_state,
    decode_trajectory_point,
    encode_claw_command,
    encode_claw_result,
    encode_claw_state,
    encode_robot_state,
    encode_trajectory_point,
    crc16_ccitt,
)


def test_crc16_ccitt_false_known_vector():
    assert crc16_ccitt(b"123456789") == 0x29B1


def test_fixed_frame_round_trip_chunking_and_noise_resync():
    packet = Frame(MessageType.TRAJECTORY_BEGIN, 17, b"\x02\x00").encode()
    parser = FrameParser()
    assert parser.feed(b"noise" + packet[:4]) == []
    frames = parser.feed(packet[4:])
    assert frames == [Frame(MessageType.TRAJECTORY_BEGIN, 17, b"\x02\x00")]
    assert parser.discarded_bytes == 5


def test_cpp_python_trajectory_point_golden_vector():
    point = TrajectoryPoint(
        0, 100,
        (0.1, -0.2, 0.3, -0.4, 0.5, -0.6),
        (1.0, -1.0, 2.0, -2.0, 3.0, -3.0),
    )
    payload = encode_trajectory_point(point)
    packet = Frame(MessageType.TRAJECTORY_POINT, 1, payload).encode()
    assert packet.hex() == (
        "aa55020100000064000000a0860100c0f2fcffe093040080e5f9ff20a1070040d8"
        "f6ff40420f00c0bdf0ff80841e00807be1ffc0c62d004039d2ff9dc4"
    )
    assert decode_trajectory_point(payload) == point


def test_parser_recovers_after_bad_crc():
    bad = bytearray(Frame(MessageType.HEARTBEAT, 1).encode())
    bad[-1] ^= 0x7F
    good = Frame(MessageType.HEARTBEAT, 2).encode()
    parser = FrameParser()
    assert parser.feed(bytes(bad) + good) == [Frame(MessageType.HEARTBEAT, 2)]
    assert parser.crc_errors == 1


def test_fixed_payload_size_is_enforced():
    with pytest.raises(ValueError):
        Frame(MessageType.HEARTBEAT, 1, b"unexpected").encode()


def test_robot_state_has_six_actual_joint_angles():
    original = RobotState(
        RobotMode.READY, 0,
        (0.1, -0.2, 0.3, -0.4, 0.5, -0.6),
    )
    assert decode_robot_state(encode_robot_state(original)) == original


def test_claw_command_and_result_round_trip():
    command = ClawCommand(ClawAction.CLOSE)
    assert decode_claw_command(encode_claw_command(command)) == command
    result = ClawResult(ClawResultCode.COMMAND_COMPLETED_UNVERIFIED)
    assert decode_claw_result(encode_claw_result(result)) == result


def test_claw_state_is_explicitly_unverified():
    state = ClawState(ClawStateCode.OPEN, verified=False)
    assert encode_claw_state(state) == b"\x02\x00"
    assert decode_claw_state(encode_claw_state(state)) == state
    with pytest.raises(ValueError):
        decode_claw_state(b"\x02\x80")
