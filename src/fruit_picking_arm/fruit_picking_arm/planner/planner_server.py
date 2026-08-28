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
    Constraints, JointConstraint,
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
        self._active_goal_handle = None
        self._active_goal_lock = threading.Lock()

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

    def configure(self, planner_cfg: dict, robot_cfg: dict):
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

        executor = MultiThreadedExecutor()
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
        # Use IK to get joint target, then plan in joint space
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

        grip = gripper_positions or self.get_current_gripper_positions()
        if len(grip) != len(self._gripper_joints):
            self._logger.error("Cannot execute without a complete gripper state")
            return False

        # Merge arm trajectory with gripper
        full = JointTrajectory()
        full.joint_names = list(self._all_joints)
        jmap = {n: i for i, n in enumerate(trajectory.joint_names)}

        from trajectory_msgs.msg import JointTrajectoryPoint
        for pt in trajectory.points:
            fp = JointTrajectoryPoint()
            fp.positions = [
                pt.positions[jmap[j]] if j in jmap
                else grip[self._gripper_joints.index(j)] if j in self._gripper_joints
                else 0.0
                for j in self._all_joints
            ]
            fp.time_from_start = pt.time_from_start
            full.points.append(fp)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = full

        def _dur(sec):
            return Duration(sec=int(sec), nanosec=int((sec - int(sec)) * 1e9))

        goal.goal_time_tolerance = _dur(self._goal_time_tol)

        future = self._arm_client.send_goal_async(goal)
        self._spin_both(future, timeout_sec=self._exec_timeout)
        handle = future.result()
        if not handle or not handle.accepted:
            self._logger.error("Trajectory goal rejected")
            return False

        self._set_active_goal(handle)
        try:
            result_future = handle.get_result_async()
            self._spin_both(result_future, timeout_sec=self._exec_timeout)
            result = result_future.result()
            if result is None:
                return False

            ok = (
                result.status == GoalStatus.STATUS_SUCCEEDED
                and result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            )
            if not ok:
                self._logger.error(f"Execution failed: {result.result.error_string}")
                return False
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
        future = self._gripper_client.send_goal_async(goal)
        self._spin_both(future, timeout_sec=5.0)
        handle = future.result()
        if not handle or not handle.accepted:
            return False

        result_future = handle.get_result_async()
        self._spin_both(result_future, timeout_sec=10.0)
        wrapped = result_future.result()
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

        future = self._arm_client.send_goal_async(goal)
        self._spin_both(future, timeout_sec=10.0)
        handle = future.result()
        if not handle or not handle.accepted:
            return False

        self._set_active_goal(handle)
        try:
            result_future = handle.get_result_async()
            self._spin_both(result_future, timeout_sec=10.0)
            r = result_future.result()
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
        """Cancel host-side motion when real perception data becomes stale."""
        with self._active_goal_lock:
            handle = self._active_goal_handle
        if handle is not None:
            self._logger.error("Cancelling active trajectory because perception data is stale")
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
