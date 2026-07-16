"""Serial communication backends shared by both hardware architectures.

The package is intentionally split into a ROS-independent protocol/transport
layer and thin ROS2 adapters.  This keeps packet tests runnable on a normal
development PC without sourcing a ROS2 installation.
"""

from jaka_single_arm.communication.protocol import (
    Ack,
    AckStatus,
    Frame,
    FrameFlags,
    FrameParser,
    FruitClass,
    FruitTarget,
    MessageType,
    MotionResult,
    ResultCode,
    RobotMode,
    RobotState,
    TrajectoryPoint,
)

__all__ = [
    "Ack",
    "AckStatus",
    "Frame",
    "FrameFlags",
    "FrameParser",
    "FruitClass",
    "FruitTarget",
    "MessageType",
    "MotionResult",
    "ResultCode",
    "RobotMode",
    "RobotState",
    "TrajectoryPoint",
]
