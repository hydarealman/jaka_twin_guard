"""Gazebo-only visual bridge for placing released fruit into sorting bins."""

from __future__ import annotations

import json
import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from rclpy.node import Node
from std_msgs.msg import String


class GazeboSceneBridge(Node):
    """Map detected fruit positions back to Gazebo models on release.

    This node exists only in simulation. It provides a deterministic visual
    placement result without pretending to model contact/friction quality of
    a real gripper.
    """

    def __init__(self):
        super().__init__("gazebo_scene_bridge")
        config_path = os.path.join(
            get_package_share_directory("jaka_single_arm"),
            "config",
            "scene_params.yaml",
        )
        with open(config_path, "r", encoding="utf-8") as stream:
            self._scene = yaml.safe_load(stream) or {}
        self._objects = list(self._scene.get("objects", []))
        self._placed: set[str] = set()
        self._bin_counts = {"healthy": 0, "unhealthy": 0}
        self._client = self.create_client(SetEntityState, "/set_entity_state")
        self._subscription = self.create_subscription(
            String, "/simulation/place_fruit", self._on_place, 10
        )
        self.get_logger().info(
            "Gazebo scene bridge ready: visual release events -> sorting bins"
        )

    def _on_place(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
            source_x = float(event["source_x"])
            source_y = float(event["source_y"])
            radius = float(event.get("radius", 0.03))
            kind = (
                "unhealthy"
                if str(event.get("health", "")).lower().startswith("un")
                else "healthy"
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Invalid place event: {exc}")
            return

        available = [
            obj for obj in self._objects if str(obj.get("id", "")) not in self._placed
        ]
        if not available:
            self.get_logger().warning("No unplaced Gazebo fruit model remains")
            return
        selected = min(
            available,
            key=lambda obj: math.hypot(
                float(obj.get("position", {}).get("x", 0.0)) - source_x,
                float(obj.get("position", {}).get("y", 0.0)) - source_y,
            ),
        )
        model_name = str(selected.get("id", ""))
        if not model_name:
            return

        bins = self._scene.get("bins", {})
        bin_cfg = bins.get(kind) or bins.get("healthy") or self._scene.get("bin", {})
        center = bin_cfg.get("center", {"x": 0.55, "y": 0.45})
        top_z = float(bin_cfg.get("top_z", 0.30))
        offsets = [(-0.035, 0.0), (0.035, 0.0), (0.0, -0.035), (0.0, 0.035)]
        index = self._bin_counts[kind]
        dx, dy = offsets[index % len(offsets)]

        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = model_name
        request.state.reference_frame = "world"
        request.state.pose.position.x = float(center.get("x", 0.55)) + dx
        request.state.pose.position.y = float(center.get("y", 0.0)) + dy
        # Start just above the bin and let Gazebo gravity settle the fruit.
        request.state.pose.position.z = top_z + max(radius, 0.02) + 0.03
        request.state.pose.orientation.w = 1.0

        if not self._client.service_is_ready():
            self.get_logger().warning("/set_entity_state is not ready")
            return
        future = self._client.call_async(request)
        future.add_done_callback(
            lambda completed, name=model_name, bin_kind=kind: self._on_result(
                completed, name, bin_kind
            )
        )
        self._placed.add(model_name)
        self._bin_counts[kind] += 1

    def _on_result(self, future, model_name: str, kind: str) -> None:
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"Placed Gazebo model {model_name} into {kind} bin"
                )
            else:
                self.get_logger().error(
                    f"Gazebo rejected placement for {model_name}: {response.status_message}"
                )
        except Exception as exc:
            self.get_logger().error(f"Gazebo placement failed for {model_name}: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboSceneBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

