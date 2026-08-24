"""Publish the calibrated fixed camera-to-base transform for eye-to-hand use."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


# ===== ROS2 参数赋值流程（含外部 YAML 注入） =====
# 1. 节点启动前：系统将 YAML 中的参数（按节点名匹配）预先载入节点“内部参数表”。
# 2. declare_parameter(name, default)：
#      若参数表中已存在该 name（YAML 提供），则忽略代码中的 default；
#      若不存在，则将该 default 写入参数表。
# 3. get_parameter(name).value：从参数表中读取当前存储的最终值。
# 4. Python 的 "="：将读取到的值正式赋给当前局部变量（如 translation）。
# ==================================================
class HandEyeStaticTf(Node):
    def __init__(self):
        # 声明参数
        super().__init__("hand_eye_static_tf")
        self.declare_parameter("calibrated", False)
        self.declare_parameter("parent_frame", "base_link")
        self.declare_parameter("child_frame", "camera_link")
        self.declare_parameter("translation_m", [0.0, 0.0, 0.0])
        self.declare_parameter("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
        if not bool(self.get_parameter("calibrated").value):
            self.get_logger().error(
                "hand_eye_params.yaml is not calibrated; no base→camera TF was published"
            )
            return
        # 将参数赋值给变量
        translation = list(self.get_parameter("translation_m").value)
        quaternion = list(self.get_parameter("quaternion_xyzw").value)
        # 校验参数长度
        if len(translation) != 3 or len(quaternion) != 4:
            raise ValueError("hand-eye translation must have 3 values and quaternion 4 values")
        # 构造并发布静态变换
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
