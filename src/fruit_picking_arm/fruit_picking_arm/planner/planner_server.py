#!/usr/bin/env python3
"""Planner Server — MoveIt2 motion planning wrapper for single arm.

Encapsulates MoveIt2 service clients and provides a clean Python API
for joint-space and pose-target motion planning.

Reference:
  - jaka_dual_arm/planner/planner_server.py — dual-arm version
  - MoveIt2 C++ API — GetMotionPlan, GetPositionIK services
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import (
    Constraints, DisplayTrajectory, JointConstraint, RobotState, RobotTrajectory,
)
from moveit_msgs.srv import GetMotionPlan, GetPositionIK, ApplyPlanningScene
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from builtin_interfaces.msg import Duration


class SingleArmPlannerServer(Node):
    """MoveIt2 planning server wrapper for a single arm.

    Usage:
        planner = SingleArmPlannerServer()
        traj = planner.plan_joint_target(target_joint_values)
        if traj:
            planner.execute(traj)
    """

    def __init__(self, node_name: str = "planner_server"):
        super().__init__(node_name)

        # Parameters (overridable via YAML/launch)
        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
        self.declare_parameter("num_planning_attempts", 10)
        self.declare_parameter("allowed_planning_time", 8.0)
        self.declare_parameter("max_velocity_scaling_factor", 0.5)
        self.declare_parameter("max_acceleration_scaling_factor", 0.5)
        self.declare_parameter("joint_tolerance", 0.005)
        self.declare_parameter("goal_time_tolerance", 0.5)
        self.declare_parameter("execution_timeout", 60.0)
        self.declare_parameter("settle_velocity_threshold", 0.10)
        self.declare_parameter("settle_timeout", 2.0)
        self.declare_parameter("settle_samples", 3)
        self.declare_parameter("controller_reports_stopped", False)

        # Service clients
        # 请求Moveit2根据当前机器人状态和目标状态,规划一条运动轨迹
        self._motion_plan_client = self.create_client(
            GetMotionPlan, "/plan_kinematic_path"
        )
        # 根据末端目标位姿,计算机械臂关节角
        self._ik_client = self.create_client(
            GetPositionIK, "/compute_ik"
        )
        # 修改moveit2的世界模型 -> 告诉Moveit: 环境发生变化
        self._apply_scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )

        # Action client for execution
        self._arm_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self._gripper_client = ActionClient(
            self, GripperCommand, "/gripper_controller/gripper_cmd"
        )
        self._display_trajectory_pub = self.create_publisher(
            DisplayTrajectory, "/display_planned_path", 10
        )

        # Joint state cache (thread-safe)
        self._joint_positions: dict[str, float] = {}
        self._joint_velocities: dict[str, float] = {}
        self._joint_states_received: bool = False
        self._joint_state_sequence: int = 0
        self._lock = threading.Lock()
        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )

        # Runner node reference — spun alongside planner during blocking calls
        # to keep the safety monitor's joint_state callback alive.
        self._runner_node: Optional[Node] = None
        # PickPlaceRunner owns one executor for the entire task.  Blocking
        # planning/action waits must spin that same executor: adding these
        # nodes to a temporary executor would silently remove them from the
        # runner executor and make the operator approval services stop
        # receiving callbacks after the first plan.
        self._shared_executor = None
        self._active_goal_handle = None
        self._active_goal_lock = threading.Lock()
        self._command_lock = threading.RLock()
        self._cancel_epoch = 0
        self._motion_latched = False
        self._motion_guard = lambda: True
        self._pending_goals = set()
        self._tracked_goals = {}

        self._logger = self.get_logger()

    def set_runner_node(self, runner_node: Node):
        """Register the runner node for dual-node spinning.

        During blocking calls (execute, plan, solve_ik), both the planner
        and runner nodes are spun together so that the safety monitor's
        /joint_states subscription keeps receiving callbacks. Without this,
        synchronous plan+execute sequences exceeding joint_state_timeout
        will trigger false ESTOP.
        """
        self._runner_node = runner_node

    def set_motion_guard(self, guard):
        self._motion_guard = guard

    def arm_motion(self):
        with self._command_lock:
            if self._pending_goals or self._tracked_goals:
                return False
            self._motion_latched = False
            return True

    def _send_guarded_goal(self, client, goal, timeout):
        # Submit and latch cancellation under one lock. A late acceptance is
        # canceled too; it must never escape a stop during the ACK wait.
        with self._command_lock:
            if self._motion_latched or not self._motion_guard():
                return None
            epoch = self._cancel_epoch
            future = client.send_goal_async(goal)
            self._pending_goals.add(future)

        def accepted(completed):
            with self._command_lock:
                try:
                    handle = completed.result()
                except Exception:
                    self._motion_latched = True
                    return
                self._pending_goals.discard(completed)
                if handle is None or not handle.accepted:
                    return
                try:
                    result = handle.get_result_async()
                except Exception:
                    self._tracked_goals[id(handle)] = (handle, None)
                    self._motion_latched = True
                    handle.cancel_goal_async()
                    return
                self._tracked_goals[id(handle)] = (handle, result)
                result.add_done_callback(lambda done: self._forget_goal(handle, done))
                if epoch != self._cancel_epoch or self._motion_latched or not self._motion_guard():
                    handle.cancel_goal_async()

        future.add_done_callback(accepted)
        self._spin_both(future, timeout_sec=timeout)
        if not future.done():
            self.cancel_active_goal()
            return None
        handle = future.result()
        with self._command_lock:
            if epoch != self._cancel_epoch or self._motion_latched or not self._motion_guard():
                if handle is not None and handle.accepted:
                    handle.cancel_goal_async()
                return None
        return handle

    def _forget_goal(self, handle, result):
        with self._command_lock:
            try:
                terminal = result.result().status in (
                    GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                    GoalStatus.STATUS_CANCELED,
                )
            except Exception:
                terminal = False
            if terminal:
                self._tracked_goals.pop(id(handle), None)
            else:
                # A failed result request is not proof that motors stopped.
                self._motion_latched = True
                handle.cancel_goal_async()

    def _await_result(self, handle, timeout):
        with self._command_lock:
            tracked = self._tracked_goals.get(id(handle))
        result = tracked[1] if tracked is not None else handle.get_result_async()
        if result is None:
            self.cancel_active_goal()
            return None
        self._spin_both(result, timeout_sec=timeout)
        if not result.done():
            self.cancel_active_goal()
            # Keep the handle tracked until the server confirms a terminal
            # state. A new start cannot clear this outstanding cancellation.
            self._spin_both(result, timeout_sec=3.0)
            return None
        return result.result()

    def set_shared_executor(self, executor) -> None:
        """Use the task runner's executor for synchronous ROS waits."""
        self._shared_executor = executor

    def spin_callbacks_once(self, timeout_sec: float = 0.1) -> None:
        """Process task callbacks without moving nodes between executors."""
        if self._shared_executor is not None:
            self._shared_executor.spin_once(timeout_sec=timeout_sec)
        else:
            rclpy.spin_once(self, timeout_sec=timeout_sec)

    def services_ready(self) -> bool:
        """Return whether all planning/execution endpoints are discoverable."""
        return (
            self._motion_plan_client.service_is_ready()
            and self._ik_client.service_is_ready()
            and self._arm_client.server_is_ready()
        )

    def _spin_both(self, future, timeout_sec: float):
        """Spin both planner and runner nodes until future completes or timeout.

        Replaces rclpy.spin_until_future_complete(self, ...) to keep the
        runner node's callbacks (safety monitor, TF, etc.) alive during
        long blocking operations.
        """
        if self._shared_executor is not None:
            self._shared_executor.spin_until_future_complete(
                future, timeout_sec=timeout_sec
            )
            return

        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(self)
        if self._runner_node is not None:
            executor.add_node(self._runner_node)
        try:
            executor.spin_until_future_complete(future, timeout_sec)
        finally:
            executor.remove_node(self)
            if self._runner_node is not None:
                try:
                    executor.remove_node(self._runner_node)
                except Exception:
                    pass

    # ── Configuration ────────────────────────────────────────

    def configure(
        self, planner_cfg: dict, robot_cfg: dict, *, real_mode: bool = False
    ):
        """Apply YAML configuration."""
        self._planning_group = planner_cfg.get("planning_group",
                                self.get_parameter("planning_group").value)
        self._planner_id = planner_cfg.get("planner_id",
                            self.get_parameter("planner_id").value)
        self._num_attempts = planner_cfg.get("num_planning_attempts",
                               self.get_parameter("num_planning_attempts").value)
        self._planning_time = planner_cfg.get("allowed_planning_time",
                                self.get_parameter("allowed_planning_time").value)
        self._vel_scaling = planner_cfg.get("max_velocity_scaling_factor",
                              self.get_parameter("max_velocity_scaling_factor").value)
        self._acc_scaling = planner_cfg.get("max_acceleration_scaling_factor",
                               self.get_parameter("max_acceleration_scaling_factor").value)
        self._joint_tol = planner_cfg.get("joint_tolerance",
                            self.get_parameter("joint_tolerance").value)
        self._goal_time_tol = planner_cfg.get("goal_time_tolerance",
                                self.get_parameter("goal_time_tolerance").value)
        self._exec_timeout = planner_cfg.get("execution_timeout",
                               self.get_parameter("execution_timeout").value)
        self._settle_velocity = planner_cfg.get(
            "settle_velocity_threshold",
            self.get_parameter("settle_velocity_threshold").value,
        )
        self._settle_timeout = planner_cfg.get(
            "settle_timeout", self.get_parameter("settle_timeout").value
        )
        self._settle_samples = int(planner_cfg.get(
            "settle_samples", self.get_parameter("settle_samples").value
        ))
        self._real_mode = bool(real_mode)
        self._controller_reports_stopped = bool(planner_cfg.get(
            "controller_reports_stopped",
            self.get_parameter("controller_reports_stopped").value,
        ))

        self._arm_joints = robot_cfg.get("arm_joints",
                            ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"])
        self._gripper_joints = robot_cfg.get("gripper_joints",
                                ["left_finger_joint", "right_finger_joint"])
        self._all_joints = self._arm_joints + self._gripper_joints
        self._ik_link = robot_cfg.get("ik_link", "gripper_tcp")

        self._logger.info(
            f"PlannerServer configured: group={self._planning_group}, "
            f"planner={self._planner_id}, joints={len(self._arm_joints)}"
        )

    # ── Joint State ──────────────────────────────────────────

    def _on_joint_state(self, msg: JointState):
        with self._lock:
            for index, name in enumerate(msg.name):
                if index < len(msg.position):
                    self._joint_positions[name] = msg.position[index]
                if index < len(msg.velocity):
                    self._joint_velocities[name] = msg.velocity[index]
            self._joint_states_received = True
            self._joint_state_sequence += 1

    def get_current_arm_positions(self) -> list[float]:
        with self._lock:
            try:
                return [self._joint_positions[j] for j in self._arm_joints]
            except KeyError:
                return []

    def has_joint_states(self) -> bool:
        """Return True only after every required arm and gripper joint exists."""
        with self._lock:
            return self._joint_states_received and all(
                joint in self._joint_positions for joint in self._all_joints
            )

    def get_current_gripper_positions(self) -> list[float]:
        with self._lock:
            try:
                return [self._joint_positions[j] for j in self._gripper_joints]
            except KeyError:
                return []

    def wait_until_stopped(self) -> bool:
        """Wait for fresh, consecutive low-velocity arm joint states.

        A FollowJointTrajectory result and the final zero-velocity joint-state
        sample are delivered by different ROS callbacks.  Without this gate,
        the behavior tree can run its safety check against the preceding
        in-motion sample and report a false velocity HALT, especially when the
        Gazebo GUI slows callback scheduling.
        """
        from rclpy.executors import MultiThreadedExecutor

        deadline = time.monotonic() + float(self._settle_timeout)
        required = max(1, int(self._settle_samples))
        stable = 0
        with self._lock:
            last_sequence = self._joint_state_sequence

        owns_executor = self._shared_executor is None
        executor = self._shared_executor or MultiThreadedExecutor()
        if owns_executor:
            executor.add_node(self)
            if self._runner_node is not None:
                executor.add_node(self._runner_node)
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
                with self._lock:
                    sequence = self._joint_state_sequence
                    velocities = [
                        abs(self._joint_velocities.get(joint, float("inf")))
                        for joint in self._arm_joints
                    ]
                if sequence == last_sequence:
                    continue
                last_sequence = sequence
                if velocities and max(velocities) <= float(self._settle_velocity):
                    stable += 1
                    if stable >= required:
                        return True
                else:
                    stable = 0
        finally:
            if owns_executor:
                executor.remove_node(self)
                if self._runner_node is not None:
                    try:
                        executor.remove_node(self._runner_node)
                    except Exception:
                        pass

        with self._lock:
            peak = max(
                (abs(self._joint_velocities.get(joint, 0.0))
                 for joint in self._arm_joints),
                default=0.0,
            )
        self._logger.error(
            f"Joints did not settle below {self._settle_velocity:.2f} rad/s "
            f"within {self._settle_timeout:.1f}s (latest peak={peak:.2f})"
        )
        return False

    # ── Motion Planning ──────────────────────────────────────

    def plan_joint_target(self, target: list[float],
                          start: list[float] = None) -> JointTrajectory | None:
        """Plan a joint-space trajectory to target positions.

        Args:
            target: Target joint positions for arm joints (6 values).
            start: Starting joint positions. Uses current state if None.

        Returns:
            JointTrajectory with arm joints only, or None on failure.
        """
        if start is None:
            start = self.get_current_arm_positions()
        if len(start) != len(self._arm_joints):
            self._logger.error("Cannot plan without a complete arm joint state")
            return None
        if len(target) != len(self._arm_joints):
            self._logger.error("Joint target does not match configured arm joints")
            return None

        req = GetMotionPlan.Request()
        mr = req.motion_plan_request
        mr.group_name = self._planning_group
        mr.planner_id = self._planner_id
        mr.num_planning_attempts = self._num_attempts
        mr.allowed_planning_time = self._planning_time
        mr.max_velocity_scaling_factor = self._vel_scaling
        mr.max_acceleration_scaling_factor = self._acc_scaling

        # Start state
        mr.start_state.is_diff = True
        mr.start_state.joint_state.name = list(self._arm_joints)
        mr.start_state.joint_state.position = list(start)

        # Goal constraints
        constraints = Constraints()
        for jn, jp in zip(self._arm_joints, target):
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = jp
            jc.tolerance_above = self._joint_tol
            jc.tolerance_below = self._joint_tol
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        mr.goal_constraints.append(constraints)

        return self._call_plan_service(req)

    def plan_pose_target(self, pose_stamped: PoseStamped,
                         cartesian: bool = False) -> JointTrajectory | None:
        """Plan a pose-target trajectory.

        Args:
            pose_stamped: Target end-effector pose in world frame.
            cartesian: Use Cartesian path if True.

        Returns:
            JointTrajectory or None.
        """
        # Match the field-validated RViz workflow: solve the exact pose with
        # collision-aware IK, then let MoveIt plan to that joint target.  The
        # cartesian flag remains for compatibility with the existing skills.
        ik_solution = self.solve_ik(pose_stamped)
        if ik_solution is None:
            self._logger.error("IK failed for pose target")
            return None

        return self.plan_joint_target(ik_solution)

    def solve_ik(self, pose_stamped: PoseStamped,
                 seed: list[float] = None,
                 timeout: float = 0.5) -> list[float] | None:
        """Solve inverse kinematics for a pose target.

        Args:
            pose_stamped: Target pose in world frame.
            seed: Seed joint positions.
            timeout: IK timeout in seconds.

        Returns:
            List of 6 joint values or None.
        """
        if not self._ik_client.wait_for_service(timeout_sec=2.0):
            self._logger.error("IK service not available")
            return None

        if seed is None:
            seed = self.get_current_arm_positions()
        if len(seed) != len(self._arm_joints):
            self._logger.error("Cannot solve IK without a complete arm joint state")
            return None

        req = GetPositionIK.Request()
        ik = req.ik_request
        ik.group_name = self._planning_group
        ik.pose_stamped = pose_stamped
        ik.ik_link_name = self._ik_link
        ik.timeout.sec = int(timeout)
        ik.timeout.nanosec = int((timeout - int(timeout)) * 1e9)
        ik.avoid_collisions = True

        if seed is not None:
            ik.robot_state.is_diff = True
            ik.robot_state.joint_state.name = list(self._arm_joints)
            ik.robot_state.joint_state.position = list(seed)

        future = self._ik_client.call_async(req)
        self._spin_both(future, timeout_sec=timeout + 2.0)
        result = future.result()

        if result is None or result.error_code.val != 1:
            return None

        sol = result.solution.joint_state
        jmap = {n: p for n, p in zip(sol.name, sol.position)}
        try:
            return [jmap[j] for j in self._arm_joints]
        except KeyError:
            return None

    # ── Trajectory Execution ─────────────────────────────────

    def publish_trajectory_preview(self, trajectory: JointTrajectory) -> bool:
        """Show the exact stored BT trajectory in RViz without executing it."""
        if trajectory is None or not trajectory.points:
            self._logger.error("Cannot preview an empty trajectory")
            return False
        current_arm = self.get_current_arm_positions()
        current_gripper = self.get_current_gripper_positions()
        if len(current_arm) != len(self._arm_joints):
            self._logger.error("Cannot preview without a complete start state")
            return False

        display = DisplayTrajectory()
        display.model_id = "fruit_picking_arm"
        display.trajectory_start = RobotState()
        display.trajectory_start.is_diff = False
        display.trajectory_start.joint_state.name = list(self._arm_joints)
        display.trajectory_start.joint_state.position = list(current_arm)
        if len(current_gripper) == len(self._gripper_joints):
            display.trajectory_start.joint_state.name.extend(self._gripper_joints)
            display.trajectory_start.joint_state.position.extend(current_gripper)
        robot_trajectory = RobotTrajectory()
        robot_trajectory.joint_trajectory = trajectory
        display.trajectory.append(robot_trajectory)
        self._display_trajectory_pub.publish(display)
        return True

    def execute(self, trajectory: JointTrajectory,
                gripper_positions: list[float] = None) -> bool:
        """Execute a trajectory via FollowJointTrajectory action.

        Args:
            trajectory: JointTrajectory for arm joints only.
            gripper_positions: Optional gripper positions to merge in.

        Returns:
            True if execution succeeded.
        """
        if not self._arm_client.wait_for_server(timeout_sec=5.0):
            self._logger.error("Arm controller action server not available")
            return False

        # The physical serial controller deliberately accepts the six arm
        # joints only; its binary gripper has a separate GripperCommand action.
        # Gazebo's existing controller still expects the two display fingers
        # merged into the arm trajectory.
        grip = gripper_positions or self.get_current_gripper_positions()
        if not self._real_mode and len(grip) != len(self._gripper_joints):
            self._logger.error("Cannot execute without a complete gripper state")
            return False

        full = JointTrajectory()
        full.joint_names = (
            list(self._arm_joints) if self._real_mode else list(self._all_joints)
        )
        jmap = {n: i for i, n in enumerate(trajectory.joint_names)}

        if any(joint not in jmap for joint in self._arm_joints):
            self._logger.error("Trajectory is missing a required arm joint")
            return False

        from trajectory_msgs.msg import JointTrajectoryPoint
        for pt in trajectory.points:
            fp = JointTrajectoryPoint()
            fp.positions = [
                pt.positions[jmap[j]] if j in jmap
                else grip[self._gripper_joints.index(j)] if j in self._gripper_joints
                else 0.0
                for j in full.joint_names
            ]
            if len(pt.velocities) != len(trajectory.joint_names):
                self._logger.error("Trajectory point is missing joint velocities")
                return False
            fp.velocities = [
                pt.velocities[jmap[j]] if j in jmap else 0.0
                for j in full.joint_names
            ]
            if len(pt.accelerations) == len(trajectory.joint_names):
                fp.accelerations = [
                    pt.accelerations[jmap[j]] if j in jmap else 0.0
                    for j in full.joint_names
                ]
            fp.time_from_start = pt.time_from_start
            full.points.append(fp)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = full

        def _dur(sec):
            return Duration(sec=int(sec), nanosec=int((sec - int(sec)) * 1e9))

        goal.goal_time_tolerance = _dur(self._goal_time_tol)

        handle = self._send_guarded_goal(self._arm_client, goal, self._exec_timeout)
        if not handle or not handle.accepted:
            self._logger.error("Trajectory goal rejected")
            return False

        self._set_active_goal(handle)
        try:
            result = self._await_result(handle, self._exec_timeout)
            if result is None:
                return False

            ok = (
                result.status == GoalStatus.STATUS_SUCCEEDED
                and result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            )
            if not ok:
                self._logger.error(f"Execution failed: {result.result.error_string}")
                return False
            # The H7 returns SUCCESS only after all six axes satisfy its goal
            # tolerance and measured-speed threshold for a stable interval.
            # Requiring three more 20 Hz ROS samples repeats the same gate and
            # inserts a visible pause after every physical trajectory.  Keep
            # the ROS-side gate for controllers that do not provide this
            # stronger completion contract (including simulation).
            if getattr(self, "_controller_reports_stopped", False):
                return True
            return self.wait_until_stopped()
        finally:
            self._clear_active_goal(handle)

    # ── Gripper Control ──────────────────────────────────────

    def send_binary_gripper_command(
        self, position: float, max_effort: float = 0.0
    ) -> bool:
        """Send a real binary gripper endpoint through GripperCommand."""
        if not self._gripper_client.wait_for_server(timeout_sec=3.0):
            self._logger.error("GripperCommand action server is unavailable")
            return False

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = max(0.0, float(max_effort))
        handle = self._send_guarded_goal(self._gripper_client, goal, 5.0)
        if not handle or not handle.accepted:
            return False

        wrapped = self._await_result(handle, 10.0)
        return bool(
            wrapped is not None
            and wrapped.status == GoalStatus.STATUS_SUCCEEDED
            and wrapped.result.reached_goal
        )

    def send_gripper_command(self, positions: list[float],
                             time_from_start: float = 1.0) -> bool:
        """Send a gripper-only command while holding arm position.

        Args:
            positions: Gripper joint positions [left, right].
            time_from_start: Duration for the gripper movement.

        Returns:
            True if executed.
        """
        arm = self.get_current_arm_positions()
        if len(arm) != len(self._arm_joints):
            self._logger.error("Cannot command gripper without complete arm state")
            return False

        traj = JointTrajectory()
        traj.joint_names = list(self._all_joints)

        from trajectory_msgs.msg import JointTrajectoryPoint
        pt = JointTrajectoryPoint()
        pt.positions = list(arm) + list(positions)
        pt.time_from_start = Duration(
            sec=int(time_from_start),
            nanosec=int((time_from_start - int(time_from_start)) * 1e9),
        )
        traj.points.append(pt)

        return self.execute_gripper_trajectory(traj)

    def execute_gripper_trajectory(self, traj: JointTrajectory) -> bool:
        """Execute gripper trajectory via the same arm controller action."""
        if not self._arm_client.wait_for_server(timeout_sec=3.0):
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance = Duration(sec=0, nanosec=500_000_000)

        handle = self._send_guarded_goal(self._arm_client, goal, 10.0)
        if not handle or not handle.accepted:
            return False

        self._set_active_goal(handle)
        try:
            r = self._await_result(handle, 10.0)
            if r is None:
                return False
            if (
                r.status != GoalStatus.STATUS_SUCCEEDED
                or r.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL
            ):
                return False
            return self.wait_until_stopped()
        finally:
            self._clear_active_goal(handle)

    def cancel_active_goal(self) -> None:
        """Latch STOP across planning, pending acceptance, arm and gripper."""
        with self._command_lock:
            self._motion_latched = True
            self._cancel_epoch += 1
            for handle, _result in list(self._tracked_goals.values()):
                handle.cancel_goal_async()

    def _set_active_goal(self, handle) -> None:
        with self._active_goal_lock:
            self._active_goal_handle = handle

    def _clear_active_goal(self, handle) -> None:
        with self._active_goal_lock:
            if self._active_goal_handle is handle:
                self._active_goal_handle = None

    # ── Internal ─────────────────────────────────────────────

    def _call_plan_service(self, req: GetMotionPlan.Request) -> JointTrajectory | None:
        if not self._motion_plan_client.wait_for_service(timeout_sec=3.0):
            self._logger.error("Motion plan service not available")
            return None

        started_at = time.monotonic()
        future = self._motion_plan_client.call_async(req)
        self._spin_both(future, timeout_sec=self._planning_time + 4.0)
        result = future.result()

        if result is None:
            return None

        resp = result.motion_plan_response
        if resp.error_code.val != 1:
            self._logger.error(f"Plan failed: code={resp.error_code.val}")
            return None

        traj = resp.trajectory.joint_trajectory
        if not traj.points:
            self._logger.error("Empty trajectory returned")
            return None

        duration = (
            traj.points[-1].time_from_start.sec
            + traj.points[-1].time_from_start.nanosec / 1e9
        )
        self._logger.info(
            f"PLANNING_TIMING: compute_s={time.monotonic() - started_at:.3f} "
            f"points={len(traj.points)} planned_s={duration:.3f}"
        )

        return traj

    # ── Properties ───────────────────────────────────────────

    @property
    def arm_joints(self) -> list[str]:
        return list(self._arm_joints)

    @property
    def gripper_joints(self) -> list[str]:
        return list(self._gripper_joints)

    @property
    def planning_group(self) -> str:
        return self._planning_group
