"""ROS2 end-to-end acceptance client for architecture A."""

import math
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
ALL_JOINTS = ARM_JOINTS + ["left_finger_joint", "right_finger_joint"]
START = [0.0, 1.265363708, -1.570796327, 0.0, 0.0, 0.0]
FINAL = [0.12, 1.10, -1.30, -0.10, 0.08, 0.02]


class ArchitectureAClient(Node):
    def __init__(self):
        super().__init__("architecture_a_acceptance_client")
        self.client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self.gripper_client = ActionClient(
            self, GripperCommand, "/gripper_controller/gripper_cmd"
        )
        self.last_state = None
        self.last_gripper_state = None
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)
        self.create_subscription(
            UInt8MultiArray,
            "/gripper/state_estimate",
            self._on_gripper_state,
            10,
        )

    def _on_state(self, msg):
        self.last_state = msg

    def _on_gripper_state(self, msg):
        self.last_gripper_state = list(msg.data)


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
        if not node.gripper_client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError("GripperCommand action server not available")
        # Fast DDS discovery on a cold WSL boot can report an action server
        # before every underlying goal/result endpoint has finished matching.
        # A bounded settling window prevents a send_goal future from hanging.
        discovery_deadline = time.monotonic() + 10.0
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS

        first = JointTrajectoryPoint()
        first.positions = START
        first.velocities = [0.0] * len(ARM_JOINTS)
        first.time_from_start.sec = 1
        goal.trajectory.points.append(first)

        second = JointTrajectoryPoint()
        second.positions = FINAL
        second.velocities = [0.0] * len(ARM_JOINTS)
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

        final_state_verified = False
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            state = node.last_state
            # The physical ROBOT_STATE protocol measures the six arm joints.
            # The controller appends a clearly-labelled command-derived
            # gripper endpoint estimate so robot_state_publisher and MoveIt
            # receive a complete kinematic state.
            if state and state.name == ALL_JOINTS and len(state.position) == len(ALL_JOINTS):
                if all(
                    math.isclose(a, b, abs_tol=2e-6)
                    for a, b in zip(state.position, FINAL)
                ):
                    final_state_verified = True
                    break
        if not final_state_verified:
            raise RuntimeError("final /joint_states did not match the trajectory endpoint")

        # RViz can emit a one-point plan after HOME when Plan/Execute is
        # clicked again without changing the goal. The real controller must
        # acknowledge this as a no-op without transmitting another motor
        # trajectory to the C board.
        no_op = FollowJointTrajectory.Goal()
        no_op.trajectory.joint_names = ARM_JOINTS
        hold = JointTrajectoryPoint()
        hold.positions = FINAL
        hold.velocities = [0.0] * len(ARM_JOINTS)
        no_op.trajectory.points.append(hold)
        no_op_handle = spin_until(node, node.client.send_goal_async(no_op), 8.0)
        if not no_op_handle.accepted:
            raise RuntimeError("near-zero hold trajectory was rejected")
        no_op_result = spin_until(node, no_op_handle.get_result_async(), 3.0)
        if no_op_result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError("near-zero hold trajectory did not complete as a no-op")

        opened = GripperCommand.Goal()
        opened.command.position = 0.056
        opened.command.max_effort = 50.0
        open_handle = spin_until(
            node, node.gripper_client.send_goal_async(opened), 8.0
        )
        if not open_handle.accepted:
            raise RuntimeError("OPEN endpoint was rejected")
        open_result = spin_until(node, open_handle.get_result_async(), 8.0)
        if not open_result.result.reached_goal:
            raise RuntimeError("OPEN endpoint did not complete")

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if (
                node.last_state
                and math.isclose(node.last_state.position[6], 0.056, abs_tol=2e-6)
                and math.isclose(node.last_state.position[7], -0.056, abs_tol=2e-6)
                and node.last_gripper_state == [2, 0]
            ):
                break
        else:
            raise RuntimeError("OPEN state estimate did not update /joint_states")

        intermediate = GripperCommand.Goal()
        intermediate.command.position = 0.028
        rejected = spin_until(
            node, node.gripper_client.send_goal_async(intermediate), 8.0
        )
        if rejected.accepted:
            raise RuntimeError("intermediate binary gripper position was accepted")

        print(
            "ARCHITECTURE_A_E2E_PASS: six-axis trajectory, near-zero no-op filter, "
            "separate binary GripperCommand, unverified state estimate, and "
            "midpoint rejection verified"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ARCHITECTURE_A_E2E_FAIL: {exc}", file=sys.stderr)
        raise
