"""Scheme-A serial communication backend.

The package is intentionally split into a ROS-independent protocol/transport
layer and thin ROS2 adapters.  This keeps packet tests runnable on a normal
development PC without sourcing a ROS2 installation.
"""

from fruit_picking_arm.communication.protocol import (
    Ack,
    AckStatus,
    Frame,
    FrameFlags,
    FrameParser,
    FruitClass,
    FruitTarget,
    ClawAction,
    ClawResult,
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
    "ClawAction",
    "ClawResult",
    "MessageType",
    "MotionResult",
    "ResultCode",
    "RobotMode",
    "RobotState",
    "TrajectoryPoint",
]
