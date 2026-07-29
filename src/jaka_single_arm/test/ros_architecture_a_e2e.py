"""ROS2 end-to-end acceptance client for architecture A."""

import math
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS = [
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
    "left_finger_joint", "right_finger_joint",
]
FINAL = [0.12, -0.20, 0.31, -0.10, 0.08, 0.02, 0.015, -0.015]


class ArchitectureAClient(Node):
    def __init__(self):
        super().__init__("architecture_a_acceptance_client")
        self.client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.last_state = None
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)

    def _on_state(self, msg):
        self.last_state = msg


def spin_until(node, future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise TimeoutError("ROS future timed out")
    return future.result()


def main():
    rclpy.init()
    node = ArchitectureAClient()
    try:
        if not node.client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError("FollowJointTrajectory action server not available")
        # Fast DDS discovery on a cold WSL boot can report an action server
        # before every underlying goal/result endpoint has finished matching.
        # A bounded settling window prevents a send_goal future from hanging.
        discovery_deadline = time.monotonic() + 10.0
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS

        first = JointTrajectoryPoint()
        first.positions = [0.0] * len(JOINTS)
        first.velocities = [0.0] * len(JOINTS)
        first.time_from_start.sec = 1
        goal.trajectory.points.append(first)

        second = JointTrajectoryPoint()
        second.positions = FINAL
        second.velocities = [0.0] * len(JOINTS)
        second.time_from_start.sec = 2
        goal.trajectory.points.append(second)

        handle = spin_until(node, node.client.send_goal_async(goal), 15.0)
        if not handle.accepted:
            raise RuntimeError("trajectory goal was rejected")
        wrapped_result = spin_until(node, handle.get_result_async(), 8.0)
        if wrapped_result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(
                f"trajectory failed: {wrapped_result.result.error_code} "
                f"{wrapped_result.result.error_string}"
            )

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            state = node.last_state
            if state and len(state.position) == len(FINAL):
                if all(math.isclose(a, b, abs_tol=2e-6) for a, b in zip(state.position, FINAL)):
                    print("ARCHITECTURE_A_E2E_PASS: action success and final joint state verified")
                    return 0
        raise RuntimeError("final /joint_states did not match the trajectory endpoint")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ARCHITECTURE_A_E2E_FAIL: {exc}", file=sys.stderr)
        raise
