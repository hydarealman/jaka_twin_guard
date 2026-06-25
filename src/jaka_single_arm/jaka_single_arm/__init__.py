"""jaka_single_arm — Industrial-grade Single-Arm Pick-and-Place Framework.

Layered architecture (inspired by jaka_dual_arm and ros2_moveit2_ur5e_grasp):
  Layer 1: control/     — Safety monitoring & gripper control
  Layer 2: perception/  — Depth camera abstraction & object detection
           scene/       — PlanningScene collision object management
  Layer 3: planner/     — MoveIt2 planning server wrapper
  Layer 4: skills/      — Atomic manipulation skills
  Layer 5: behavior/    — BehaviorTree task orchestration

Architecture inspired by:
  - jaka_dual_arm (hydarealman) — layered planner/skills/BT design
  - ros2_moveit2_ur5e_grasp (Nackustb) — modular vision + MoveIt2
  - mycobot_ros2 (AutomaticAddison) — PCL object segmentation
  - ManyMove (pastoriomarco/manymove) — layered planner/skills/BT design
  - MoveIt Task Constructor (PickNik) — stage-based task composition
"""
