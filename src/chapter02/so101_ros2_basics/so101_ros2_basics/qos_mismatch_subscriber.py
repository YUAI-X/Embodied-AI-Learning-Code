"""故意使用 Reliable QoS 订阅 Best Effort 图像，供 QoS 排错实验使用。"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class QosMismatchSubscriber(Node):
    """故意用Reliable订阅Best Effort图像，用于复现QoS不兼容。"""

    def __init__(self):
        super().__init__("qos_mismatch_subscriber")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, "/camera/color/image_raw", self._callback, qos)
        self.get_logger().warning("本节点故意使用 Reliable；发布者是 Best Effort 时可能完全收不到数据。")

    def _callback(self, _message):
        """仅在QoS实际兼容并收到图像时输出提示。"""
        self.get_logger().info("收到图像；请用 ros2 topic info -v 检查实际 QoS")


def main(args=None):
    """启动故意配置错误QoS的图像订阅节点。"""
    rclpy.init(args=args)
    node = QosMismatchSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
