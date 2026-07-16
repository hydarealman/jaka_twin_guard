"""Publish the calibrated fixed camera-to-base transform for eye-to-hand use."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class HandEyeStaticTf(Node):
    def __init__(self):
        super().__init__("hand_eye_static_tf")
        self.declare_parameter("calibrated", False)
        self.declare_parameter("parent_frame", "base_link")
        self.declare_parameter("child_frame", "camera_depth_optical_frame")
        self.declare_parameter("translation_m", [0.0, 0.0, 0.0])
        self.declare_parameter("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
        if not bool(self.get_parameter("calibrated").value):
            self.get_logger().error(
                "hand_eye_params.yaml is not calibrated; no base→camera TF was published"
            )
            return
        translation = list(self.get_parameter("translation_m").value)
        quaternion = list(self.get_parameter("quaternion_xyzw").value)
        if len(translation) != 3 or len(quaternion) != 4:
            raise ValueError("hand-eye translation must have 3 values and quaternion 4 values")
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = str(self.get_parameter("parent_frame").value)
        transform.child_frame_id = str(self.get_parameter("child_frame").value)
        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(transform)
        self.get_logger().info(
            f"Published calibrated TF: {transform.header.frame_id} → {transform.child_frame_id}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandEyeStaticTf()
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
