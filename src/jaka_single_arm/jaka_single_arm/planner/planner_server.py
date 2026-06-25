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
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.msg import (
    Constraints, JointConstraint, PositionConstraint, OrientationConstraint,
    MotionPlanRequest,
)
from moveit_msgs.srv import GetMotionPlan, GetPositionIK, ApplyPlanningScene
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from shape_msgs.msg import SolidPrimitive
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

        # Service clients
        self._motion_plan_client = self.create_client(
            GetMotionPlan, "/plan_kinematic_path"
        )
        self._ik_client = self.create_client(
            GetPositionIK, "/compute_ik"
        )
        self._apply_scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )

        # Action client for execution
        self._arm_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )

        # Joint state cache (thread-safe)
        self._joint_positions: dict[str, float] = {}
        self._joint_states_received: bool = False
        self._lock = threading.Lock()
        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )

        # Shared executor for multi-node spinning
        self._shared_executor: Optional[rclpy.executors.Executor] = None

        self._logger = self.get_logger()

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

        self._arm_joints = robot_cfg.get("arm_joints",
                            ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"])
        self._gripper_joints = robot_cfg.get("gripper_joints",
                                ["left_finger_joint", "right_finger_joint"])
        self._all_joints = self._arm_joints + self._gripper_joints
        self._ik_link = robot_cfg.get("ik_link", "tool_flange")

        self._logger.info(
            f"PlannerServer configured: group={self._planning_group}, "
            f"planner={self._planner_id}, joints={len(self._arm_joints)}"
        )

    # ── Joint State ──────────────────────────────────────────

    def _on_joint_state(self, msg: JointState):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = pos
            self._joint_states_received = True

    def get_current_arm_positions(self) -> list[float]:
        with self._lock:
            try:
                return [self._joint_positions[j] for j in self._arm_joints]
            except KeyError:
                return [0.0] * len(self._arm_joints)

    def has_joint_states(self) -> bool:
        """Return True if joint states have been received at least once."""
        with self._lock:
            return self._joint_states_received

    def get_current_gripper_positions(self) -> list[float]:
        with self._lock:
            try:
                return [self._joint_positions[j] for j in self._gripper_joints]
            except KeyError:
                return [0.0] * len(self._gripper_joints)

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
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout + 2.0)
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
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._exec_timeout)
        handle = future.result()
        if not handle or not handle.accepted:
            self._logger.error("Trajectory goal rejected")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self._exec_timeout)
        result = result_future.result()
        if result is None:
            return False

        ok = result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        if not ok:
            self._logger.error(f"Execution failed: {result.result.error_string}")
        return ok

    # ── Gripper Control ──────────────────────────────────────

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
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if not handle or not handle.accepted:
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
        r = result_future.result()
        if r is None:
            return False
        return r.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

    # ── Internal ─────────────────────────────────────────────

    def _call_plan_service(self, req: GetMotionPlan.Request) -> JointTrajectory | None:
        if not self._motion_plan_client.wait_for_service(timeout_sec=3.0):
            self._logger.error("Motion plan service not available")
            return None

        future = self._motion_plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._planning_time + 4.0)
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
