"""jaka_dual_arm — Industrial-grade Dual-Arm Manipulation Framework.

Layered architecture:
  Layer 2: scene/       — Scene management & perception interface
  Layer 3: planner/     — MoveIt2 planning server
  Layer 4: skills/      — Atomic manipulation skills
  Layer 5: behavior/    — BehaviorTree task orchestration

Architecture inspired by:
  - ManyMove (pastoriomarco/manymove) — layered planner/skills/BT design
  - ARIAC 2024 (NIST) — industrial automation pipeline
  - MoveIt Task Constructor (PickNik) — stage-based task composition
  - multipanda_ros2 (TUM) — ros2_control multi-robot pattern
"""
