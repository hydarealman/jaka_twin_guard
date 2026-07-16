"""Send stable 3D fruit targets directly to a full-control C board.

This is architecture B.  The board is expected to own grasp pose generation,
IK, trajectory generation, joint control and gripper sequencing.
"""

from __future__ import annotations

import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

from jaka_single_arm.communication.control_link import ControlLink
from jaka_single_arm.communication.protocol import (
    FruitClass,
    FruitTarget,
    ResultCode,
    stable_u16_id,
)


class SerialFruitTargetBridge(Node):
    def __init__(self):
        super().__init__("serial_fruit_target_bridge")
        self.declare_parameter("serial_port", "COM3")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("ack_timeout", 0.25)
        self.declare_parameter("retries", 3)
        self.declare_parameter("target_topic", "/perception/stable_fruit_targets")
        self.declare_parameter("result_topic", "/serial_bridge/result")
        self.declare_parameter("require_ready", True)
        self.declare_parameter("target_ttl_ms", 1000)
        self.declare_parameter("result_timeout", 60.0)
        self.declare_parameter("workspace_min_mm", [200, -600, 0])
        self.declare_parameter("workspace_max_mm", [900, 600, 1000])
        self.declare_parameter("skip_failed_targets", True)
        self.declare_parameter("heartbeat_rate", 2.0)

        gp = self.get_parameter
        self._require_ready = bool(gp("require_ready").value)
        self._ttl_ms = int(gp("target_ttl_ms").value)
        self._result_timeout = float(gp("result_timeout").value)
        self._workspace_min = tuple(int(v) for v in gp("workspace_min_mm").value)
        self._workspace_max = tuple(int(v) for v in gp("workspace_max_mm").value)
        self._skip_failed = bool(gp("skip_failed_targets").value)
        self._link = ControlLink.open(
            port=str(gp("serial_port").value),
            baudrate=int(gp("baudrate").value),
            ack_timeout=float(gp("ack_timeout").value),
            retries=int(gp("retries").value),
        )
        self._result_pub = self.create_publisher(String, str(gp("result_topic").value), 10)
        self._subscription = self.create_subscription(
            Detection3DArray, str(gp("target_topic").value), self._on_targets, 10
        )
        self._lock = threading.Lock()
        self._active_track: str | None = None
        self._completed_tracks: set[str] = set()
        heartbeat_rate = max(0.1, float(gp("heartbeat_rate").value))
        self._heartbeat_timer = self.create_timer(1.0 / heartbeat_rate, self._heartbeat)
        self.get_logger().info(
            f"Direct fruit-target bridge: port={gp('serial_port').value}, "
            f"topic={gp('target_topic').value}"
        )

    def _on_targets(self, msg: Detection3DArray) -> None:
        if msg.header.frame_id not in ("base_link", "world"):
            self.get_logger().error(
                f"Rejecting targets in frame '{msg.header.frame_id}'; expected base_link/world"
            )
            return
        if self._require_ready and not self._link.ready:
            return
        with self._lock:
            if self._active_track is not None:
                return
            selected = None
            target = None
            # An invalid/out-of-workspace cluster must not block later valid
            # fruit in the same Detection3DArray.
            for candidate in msg.detections:
                if candidate.id in self._completed_tracks:
                    continue
                candidate_target = self._to_target(candidate, msg)
                if candidate_target is not None:
                    selected = candidate
                    target = candidate_target
                    break
            if selected is None or target is None:
                return
            self._active_track = selected.id
            track_id = selected.id
        threading.Thread(
            target=self._send_worker,
            args=(track_id, target),
            name=f"fruit-target-{target.target_id}",
            daemon=True,
        ).start()

    def _to_target(self, detection, array_msg) -> FruitTarget | None:
        if not detection.results:
            return None
        hypothesis = detection.results[0].hypothesis
        label = str(hypothesis.class_id).strip().lower()
        if label in ("healthy", "good", "0"):
            fruit_class = FruitClass.HEALTHY
        elif label in ("unhealthy", "bad", "1"):
            fruit_class = FruitClass.UNHEALTHY
        else:
            return None
        center = detection.bbox.center.position
        xyz = tuple(int(round(value * 1000.0)) for value in (center.x, center.y, center.z))
        if any(value < low or value > high for value, low, high in zip(xyz, self._workspace_min, self._workspace_max)):
            self.get_logger().error(f"Rejecting out-of-workspace target {detection.id}: {xyz} mm")
            return None
        radius_mm = int(round(max(
            detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z
        ) * 500.0))
        capture_ms = (
            int(array_msg.header.stamp.sec * 1000 + array_msg.header.stamp.nanosec / 1_000_000)
            & 0xFFFFFFFF
        )
        return FruitTarget(
            target_id=stable_u16_id(detection.id),
            fruit_class=fruit_class,
            confidence=float(hypothesis.score),
            x_mm=xyz[0], y_mm=xyz[1], z_mm=xyz[2],
            radius_mm=max(0, radius_mm),
            ttl_ms=self._ttl_ms,
            capture_time_ms=capture_ms,
        )

    def _send_worker(self, track_id: str, target: FruitTarget) -> None:
        success = False
        payload: dict = {"track_id": track_id, "target_id": target.target_id}
        try:
            result = self._link.send_fruit_target(
                target, wait_result=True, result_timeout=self._result_timeout
            )
            success = result is not None and result.result_code == ResultCode.SUCCESS
            payload.update({
                "success": success,
                "result_code": result.result_code.name if result else "NO_RESULT",
                "error_code": result.error_code if result else 0,
            })
            self.get_logger().info(
                f"Fruit target {track_id} completed: "
                f"result={payload['result_code']}, success={success}"
            )
        except Exception as exc:
            payload.update({"success": False, "result_code": "COMMUNICATION_ERROR", "error": str(exc)})
            self.get_logger().error(f"Fruit target {track_id} failed: {exc}")
        finally:
            with self._lock:
                if success or self._skip_failed:
                    self._completed_tracks.add(track_id)
                self._active_track = None
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self._result_pub.publish(msg)

    def _heartbeat(self) -> None:
        try:
            self._link.send_heartbeat()
        except Exception as exc:
            self.get_logger().warning(f"Serial heartbeat failed: {exc}")

    def destroy_node(self):
        self._link.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SerialFruitTargetBridge()
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
