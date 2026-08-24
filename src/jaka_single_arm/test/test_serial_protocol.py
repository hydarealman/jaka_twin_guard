from jaka_single_arm.communication.protocol import (
    Frame,
    FrameFlags,
    FrameParser,
    FruitClass,
    FruitTarget,
    GripperCommand,
    GripperMode,
    MessageType,
    RobotMode,
    RobotState,
    decode_fruit_target,
    decode_gripper_command,
    decode_robot_state,
    encode_fruit_target,
    encode_gripper_command,
    encode_robot_state,
    crc16_ccitt,
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


def test_crc16_ccitt_false_known_vector():
    assert crc16_ccitt(b"123456789") == 0x29B1


def test_cpp_python_trajectory_point_golden_vector():
    from jaka_single_arm.communication.protocol import (
        Frame,
        FrameFlags,
        MessageType,
        TrajectoryPoint,
        encode_trajectory_point,
    )

    payload = encode_trajectory_point(
        1,
        TrajectoryPoint(
            0,
            100,
            (0.1, -0.2, 0.3, -0.4, 0.5, -0.6),
            (1.0, -1.0, 2.0, -2.0, 3.0, -3.0),
        ),
        True,
    )
    packet = Frame(
        MessageType.TRAJECTORY_POINT, 1, payload, FrameFlags.ACK_REQUIRED
    ).encode()
    assert packet.hex() == (
        "aa55012101010038000100000064000000a0860100c0f2fcffe093040080e5f9ff"
        "20a1070040d8f6ff40420f00c0bdf0ff80841e00807be1ffc0c62d004039d2fffd03"
    )


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


def test_gripper_command_payload_round_trip():
    original = GripperCommand(
        command_id=9,
        mode=GripperMode.POSITION,
        opening_mm=68,
        speed_mm_s=80,
        force_permille=650,
    )
    assert decode_gripper_command(
        encode_gripper_command(original)
    ) == original
