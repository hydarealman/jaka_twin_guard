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

    def target_message(self, label, target_id):
        output = Detection3DArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "base_link"
        detection = Detection3D()
        detection.header = output.header
        detection.id = target_id
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = label
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

    def wait_for_result(self, label, target_id):
        self.result = None
        deadline = time.monotonic() + 8.0
        next_publish = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self.publisher.publish(self.target_message(label, target_id))
                next_publish = now + 0.2
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.result is not None:
                result = self.result
                if not result.get("success"):
                    raise RuntimeError(f"{label} target failed: {result}")
                if result.get("result_code") != "SUCCESS":
                    raise RuntimeError(f"unexpected {label} result: {result}")
                if result.get("fruit_class") != ("HEALTHY" if label == "Healthy" else "UNHEALTHY"):
                    raise RuntimeError(f"class mapping was not preserved: {result}")
                return
        raise TimeoutError(f"no {label} /serial_bridge/result received")


def main():
    rclpy.init()
    node = ArchitectureBClient()
    try:
        # Allow DDS publisher/subscriber matching before the first target is
        # sent; otherwise the one-shot result can be published before this
        # acceptance client has discovered the bridge.
        # Cold-start DDS discovery under WSL can take several seconds even
        # after the bridge process has printed its ready message.
        discovery_deadline = time.monotonic() + 10.0
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.wait_for_result("Healthy", "acceptance_healthy_001")
        node.wait_for_result("Unhealthy", "acceptance_unhealthy_001")
        print(
            "ARCHITECTURE_B_E2E_PASS: Healthy/Unhealthy FruitTarget ACK/result "
            "and class mapping verified"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ARCHITECTURE_B_E2E_FAIL: {exc}", file=sys.stderr)
        raise
