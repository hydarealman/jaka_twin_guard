"""ROS2 end-to-end acceptance client for architecture B."""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose


class ArchitectureBClient(Node):
    def __init__(self):
        super().__init__("architecture_b_acceptance_client")
        self.publisher = self.create_publisher(
            Detection3DArray, "/perception/stable_fruit_targets", 10
        )
        self.result = None
        self.create_subscription(String, "/serial_bridge/result", self._on_result, 10)

    def _on_result(self, msg):
        self.result = json.loads(msg.data)

    def target_message(self):
        output = Detection3DArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "base_link"
        detection = Detection3D()
        detection.header = output.header
        detection.id = "acceptance_fruit_001"
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = "Healthy"
        hypothesis.hypothesis.score = 0.93
        detection.results.append(hypothesis)
        detection.bbox.center.position.x = 0.50
        detection.bbox.center.position.y = 0.00
        detection.bbox.center.position.z = 0.34
        detection.bbox.size.x = 0.07
        detection.bbox.size.y = 0.07
        detection.bbox.size.z = 0.07
        output.detections.append(detection)
        return output


def main():
    rclpy.init()
    node = ArchitectureBClient()
    try:
        deadline = time.monotonic() + 8.0
        next_publish = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                node.publisher.publish(node.target_message())
                next_publish = now + 0.2
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.result is not None:
                if not node.result.get("success"):
                    raise RuntimeError(f"target failed: {node.result}")
                if node.result.get("result_code") != "SUCCESS":
                    raise RuntimeError(f"unexpected result: {node.result}")
                print(
                    "ARCHITECTURE_B_E2E_PASS: FruitTarget ACK/result and ROS result topic verified"
                )
                return 0
        raise TimeoutError("no /serial_bridge/result received")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ARCHITECTURE_B_E2E_FAIL: {exc}", file=sys.stderr)
        raise
