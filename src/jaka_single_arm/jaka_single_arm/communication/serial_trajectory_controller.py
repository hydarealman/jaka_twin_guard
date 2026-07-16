"""FollowJointTrajectory action server backed by the serial C-board protocol.

This is architecture A: MoveIt remains on the upper computer.  Existing code
continues to call /arm_controller/follow_joint_trajectory; this node replaces
the ros2_control trajectory controller and streams the planned trajectory to
the C board.
"""

from __future__ import annotations

import threading

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from jaka_single_arm.communication.control_link import ControlLink
from jaka_single_arm.communication.protocol import ResultCode, RobotMode, RobotState, TrajectoryPoint


class SerialTrajectoryController(Node):
    def __init__(self):
        super().__init__("serial_trajectory_controller")
        self.declare_parameter("serial_port", "COM3")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("ack_timeout", 0.25)
        self.declare_parameter("retries", 3)
        self.declare_parameter("action_name", "/arm_controller/follow_joint_trajectory")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("require_ready", True)
        self.declare_parameter("heartbeat_rate", 2.0)
        self.declare_parameter("joint_names", [
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6",
            "left_finger_joint", "right_finger_joint",
        ])

        gp = self.get_parameter
        self._joint_names = list(gp("joint_names").value)
        self._require_ready = bool(gp("require_ready").value)
        self._link = ControlLink.open(
            port=str(gp("serial_port").value),
            baudrate=int(gp("baudrate").value),
            ack_timeout=float(gp("ack_timeout").value),
            retries=int(gp("retries").value),
        )
        self._link.add_state_callback(self._on_robot_state)
        self._joint_pub = self.create_publisher(JointState, str(gp("joint_state_topic").value), 20)
        self._active_lock = threading.Lock()
        self._active_goal = None

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            str(gp("action_name").value),
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
        )
        heartbeat_rate = max(0.1, float(gp("heartbeat_rate").value))
        self._heartbeat_timer = self.create_timer(1.0 / heartbeat_rate, self._heartbeat)
        self.get_logger().info(
            f"Serial trajectory controller ready: port={gp('serial_port').value}, "
            f"action={gp('action_name').value}"
        )

    def _goal(self, goal_request) -> GoalResponse:
        trajectory = goal_request.trajectory
        if not trajectory.points or not trajectory.joint_names:
            return GoalResponse.REJECT
        if len(set(trajectory.joint_names)) != len(trajectory.joint_names):
            return GoalResponse.REJECT
        if any(name not in self._joint_names for name in trajectory.joint_names):
            return GoalResponse.REJECT
        with self._active_lock:
            if self._active_goal is not None:
                return GoalResponse.REJECT
        if self._require_ready and not self._link.ready:
            self.get_logger().warning("Rejecting trajectory: C board is not READY")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel(self, _goal_handle) -> CancelResponse:
        self._link.send_abort(best_effort=True)
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        result = FollowJointTrajectory.Result()
        with self._active_lock:
            self._active_goal = goal_handle
        try:
            points = self._convert_trajectory(goal_handle.request.trajectory)
            motion_result = self._link.send_trajectory(points, wait_result=True)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "cancelled by requester"
            elif motion_result and motion_result.result_code == ResultCode.SUCCESS:
                goal_handle.succeed()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = ""
            else:
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                code = motion_result.result_code.name if motion_result else "NO_RESULT"
                result.error_string = f"C board motion failed: {code}"
        except Exception as exc:
            self.get_logger().error(f"Serial trajectory failed: {exc}")
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
        finally:
            with self._active_lock:
                self._active_goal = None
        return result

    @staticmethod
    def _convert_trajectory(trajectory) -> list[TrajectoryPoint]:
        converted: list[TrajectoryPoint] = []
        previous_ms = -1
        for index, point in enumerate(trajectory.points):
            time_ms = int(point.time_from_start.sec * 1000 + point.time_from_start.nanosec / 1_000_000)
            if time_ms <= previous_ms:
                raise ValueError("trajectory time_from_start must increase strictly")
            if len(point.positions) != len(trajectory.joint_names):
                raise ValueError("trajectory point position count does not match joint_names")
            velocities = tuple(point.velocities) if point.velocities else tuple(0.0 for _ in point.positions)
            converted.append(TrajectoryPoint(index, time_ms, tuple(point.positions), velocities))
            previous_ms = time_ms
        return converted

    def _on_robot_state(self, state: RobotState) -> None:
        if state.joint_positions:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self._joint_names[: len(state.joint_positions)]
            msg.position = list(state.joint_positions)
            self._joint_pub.publish(msg)

        with self._active_lock:
            goal_handle = self._active_goal
        if goal_handle is not None and state.joint_positions:
            feedback = FollowJointTrajectory.Feedback()
            feedback.header.stamp = self.get_clock().now().to_msg()
            feedback.joint_names = self._joint_names[: len(state.joint_positions)]
            feedback.actual.positions = list(state.joint_positions)
            try:
                goal_handle.publish_feedback(feedback)
            except Exception:
                pass

    def _heartbeat(self) -> None:
        try:
            self._link.send_heartbeat()
        except Exception as exc:
            self.get_logger().warning(f"Serial heartbeat failed: {exc}")

    def destroy_node(self):
        self._action_server.destroy()
        self._link.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SerialTrajectoryController()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
