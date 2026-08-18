"""在相机光学坐标系中发布一个模拟物体位姿。"""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class MockObjectPublisher(Node):
    """周期发布camera_color_optical_frame中的固定教学物体位姿。"""

    def __init__(self):
        super().__init__("mock_object_publisher")
        self.publisher = self.create_publisher(PoseStamped, "/detected_object_pose", 10)
        self.create_timer(0.5, self._publish)

    def _publish(self):
        """发布位于相机前方0.4米的单位朝向PoseStamped。"""
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "camera_color_optical_frame"
        # 光学坐标系中：x 向右、y 向下、z 向前。
        message.pose.position.x = 0.03
        message.pose.position.y = 0.02
        message.pose.position.z = 0.40
        message.pose.orientation.w = 1.0
        self.publisher.publish(message)


def main(args=None):
    """初始化并持续运行模拟目标位姿发布器。"""
    rclpy.init(args=args)
    node = MockObjectPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
