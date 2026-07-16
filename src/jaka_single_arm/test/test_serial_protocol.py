from jaka_single_arm.communication.protocol import (
    Frame,
    FrameFlags,
    FrameParser,
    FruitClass,
    FruitTarget,
    MessageType,
    RobotMode,
    RobotState,
    decode_fruit_target,
    decode_robot_state,
    encode_fruit_target,
    encode_robot_state,
)


def test_frame_round_trip_and_chunked_parser():
    packet = Frame(
        MessageType.FRUIT_TARGET, 17, b"payload", FrameFlags.ACK_REQUIRED
    ).encode()
    parser = FrameParser()
    frames = parser.feed(b"noise" + packet[:5])
    assert frames == []
    frames = parser.feed(packet[5:])
    assert len(frames) == 1
    assert frames[0].seq == 17
    assert frames[0].payload == b"payload"
    assert parser.discarded_bytes == 5


def test_parser_recovers_after_corrupt_frame():
    bad = bytearray(Frame(MessageType.HEARTBEAT, 1, b"bad").encode())
    bad[-1] ^= 0x7F
    good = Frame(MessageType.HEARTBEAT, 2, b"good").encode()
    frames = FrameParser().feed(bytes(bad) + good)
    assert [frame.seq for frame in frames] == [2]


def test_fruit_target_payload_round_trip():
    original = FruitTarget(
        target_id=42,
        fruit_class=FruitClass.UNHEALTHY,
        confidence=0.913,
        x_mm=512,
        y_mm=-83,
        z_mm=337,
        radius_mm=36,
        ttl_ms=900,
        capture_time_ms=123456,
    )
    decoded = decode_fruit_target(encode_fruit_target(original))
    assert decoded == original


def test_robot_state_payload_round_trip():
    original = RobotState(
        timestamp_ms=123,
        mode=RobotMode.READY,
        gripper_state=1,
        error_code=0,
        tcp_xyz_mm=(500, -20, 400),
        tcp_rpy_mdeg=(180000, 0, 0),
        joint_positions=(0.1, -0.2, 0.3, -0.4, 0.5, -0.6),
    )
    decoded = decode_robot_state(encode_robot_state(original))
    assert decoded == original
