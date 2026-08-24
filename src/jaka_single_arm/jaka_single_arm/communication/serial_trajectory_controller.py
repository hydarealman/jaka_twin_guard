"""FollowJointTrajectory action server backed by the serial C-board protocol.

This is architecture A: MoveIt remains on the upper computer.  Existing code
continues to call /arm_controller/follow_joint_trajectory; this node replaces
the ros2_control trajectory controller and streams the planned trajectory to
the C board.
"""

from __future__ import annotations

import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from jaka_single_arm.communication.control_link import ControlLink
from jaka_single_arm.communication.protocol import ResultCode, RobotState, TrajectoryPoint
from jaka_single_arm.communication.trajectory_validation import (
    DEFAULT_JOINT_NAMES,
    DEFAULT_LOWER_LIMITS,
    DEFAULT_UPPER_LIMITS,
    validate_trajectory_positions,
)


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
        self.declare_parameter("reconnect_interval", 2.0)
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("joint_lower_limits", DEFAULT_LOWER_LIMITS)
        self.declare_parameter("joint_upper_limits", DEFAULT_UPPER_LIMITS)

        gp = self.get_parameter
        self._joint_names = list(gp("joint_names").value)
        lower_limits = [float(value) for value in gp("joint_lower_limits").value]
        upper_limits = [float(value) for value in gp("joint_upper_limits").value]
        if not (
            len(self._joint_names) == len(lower_limits) == len(upper_limits)
        ):
            raise ValueError(
                "joint_names, joint_lower_limits and joint_upper_limits "
                "must have equal lengths"
            )
        if len(self._joint_names) < 6:
            raise ValueError("at least six arm joint names and limits are required")
        self._arm_joint_names = self._joint_names[:6]
        self._joint_limits = {
            name: (lower, upper)
            for name, lower, upper in zip(
                self._joint_names, lower_limits, upper_limits
            )
        }
        self._require_ready = bool(gp("require_ready").value)
        self._serial_port = str(gp("serial_port").value)
        self._baudrate = int(gp("baudrate").value)
        self._ack_timeout = float(gp("ack_timeout").value)
        self._retries = int(gp("retries").value)
        self._reconnect_interval = max(
            0.5, float(gp("reconnect_interval").value)
        )
        self._link_lock = threading.Lock()
        self._link: ControlLink | None = None
        self._last_connect_attempt = 0.0
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
            f"Serial trajectory controller started: port={self._serial_port}, "
            f"action={gp('action_name').value}; motion remains blocked until READY"
        )
        self._try_connect()

    def _current_link(self) -> ControlLink | None:
        with self._link_lock:
            return self._link

    def _try_connect(self) -> ControlLink | None:
        """Open the board without killing or respawning the ROS process."""
        if self._current_link() is not None:
            return self._current_link()
        now = time.monotonic()
        if now - self._last_connect_attempt < self._reconnect_interval:
            return None
        self._last_connect_attempt = now
        try:
            candidate = ControlLink.open(
                port=self._serial_port,
                baudrate=self._baudrate,
                ack_timeout=self._ack_timeout,
                retries=self._retries,
            )
            candidate.add_state_callback(self._on_robot_state)
        except Exception as exc:
            self.get_logger().warning(
                f"Serial unavailable ({exc}); retrying in "
                f"{self._reconnect_interval:.1f}s"
            )
            return None
        with self._link_lock:
            if self._link is None:
                self._link = candidate
                accepted = True
            else:
                accepted = False
        if not accepted:
            candidate.close()
            return self._current_link()
        self.get_logger().info(
            "Serial connected; waiting for fresh C-board READY state"
        )
        return candidate

    def _drop_link(self, failed_link: ControlLink, reason: str) -> None:
        with self._link_lock:
            if self._link is not failed_link:
                return
            self._link = None
        try:
            failed_link.close()
        except Exception:
            pass
        self.get_logger().warning(
            f"Serial connection lost ({reason}); motion blocked while reconnecting"
        )

    def _goal(self, goal_request) -> GoalResponse:
        trajectory = goal_request.trajectory
        if not trajectory.points or not trajectory.joint_names:
            self.get_logger().warning("Rejecting trajectory: empty points/joint_names")
            return GoalResponse.REJECT
        if len(set(trajectory.joint_names)) != len(trajectory.joint_names):
            self.get_logger().warning("Rejecting trajectory: duplicate joint names")
            return GoalResponse.REJECT
        if any(name not in self._joint_names for name in trajectory.joint_names):
            self.get_logger().warning("Rejecting trajectory: unknown joint name")
            return GoalResponse.REJECT
        if set(trajectory.joint_names) != set(self._arm_joint_names):
            self.get_logger().warning(
                "Rejecting trajectory: architecture A requires exactly J1..J6"
            )
            return GoalResponse.REJECT
        rejection = validate_trajectory_positions(trajectory, self._joint_limits)
        if rejection is not None:
            self.get_logger().error(f"Rejecting unsafe trajectory: {rejection}")
            return GoalResponse.REJECT
        with self._active_lock:
            if self._active_goal is not None:
                return GoalResponse.REJECT
        link = self._current_link()
        if link is None:
            self.get_logger().warning("Rejecting trajectory: serial is disconnected")
            return GoalResponse.REJECT
        if self._require_ready and not link.ready:
            self.get_logger().warning("Rejecting trajectory: C board is not READY")
            return GoalResponse.REJECT
        self.get_logger().info("Accepted trajectory goal")
        return GoalResponse.ACCEPT

    def _cancel(self, _goal_handle) -> CancelResponse:
        link = self._current_link()
        if link is not None:
            link.send_abort(best_effort=True)
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        result = FollowJointTrajectory.Result()
        with self._active_lock:
            self._active_goal = goal_handle
        try:
            link = self._current_link()
            if link is None or (self._require_ready and not link.ready):
                raise RuntimeError("serial disconnected or C board is not READY")
            points = self._convert_trajectory(goal_handle.request.trajectory)
            motion_result = link.send_trajectory(points, wait_result=True)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
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
            if 'link' in locals() and link is not None:
                self._drop_link(link, str(exc))
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
        finally:
            with self._active_lock:
                self._active_goal = None
        return result

    def _convert_trajectory(self, trajectory) -> list[TrajectoryPoint]:
        converted: list[TrajectoryPoint] = []
        indices = {
            name: index for index, name in enumerate(trajectory.joint_names)
        }
        if set(indices) != set(self._arm_joint_names):
            raise ValueError("architecture A requires exactly six arm joints")
        previous_ms = -1
        for index, point in enumerate(trajectory.points):
            total_ns = int(point.time_from_start.sec) * 1_000_000_000 + int(
                point.time_from_start.nanosec
            )
            if total_ns < 0:
                raise ValueError("trajectory time_from_start cannot be negative")
            time_ms = int(point.time_from_start.sec) * 1000 + int(
                point.time_from_start.nanosec
            ) // 1_000_000
            if time_ms <= previous_ms or time_ms > 0xFFFFFFFF:
                raise ValueError("trajectory time_from_start must increase strictly")
            if len(point.positions) != len(trajectory.joint_names):
                raise ValueError("trajectory point position count does not match joint_names")
            positions = tuple(point.positions[indices[name]] for name in self._arm_joint_names)
            velocities = (
                tuple(point.velocities[indices[name]] for name in self._arm_joint_names)
                if point.velocities
                else (0.0,) * len(self._arm_joint_names)
            )
            converted.append(TrajectoryPoint(index, time_ms, positions, velocities))
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
        link = self._current_link()
        if link is None:
            self._try_connect()
            return
        try:
            link.send_heartbeat()
        except Exception as exc:
            self._drop_link(link, str(exc))

    def destroy_node(self):
        self._action_server.destroy()
        link = self._current_link()
        if link is not None:
            self._drop_link(link, "node shutdown")
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
