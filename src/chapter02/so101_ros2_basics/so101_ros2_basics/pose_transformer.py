"""使用 TF2 将相机坐标系中的物体位姿转换到 base_link。"""
# 作者：宇哥的具身笔记


import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
import tf2_geometry_msgs  # noqa: F401：导入后会注册 PoseStamped 转换类型。


class PoseTransformer(Node):
    """使用TF2把检测位姿从消息源Frame转换到base_link。"""

    def __init__(self):
        super().__init__("pose_transformer")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(PoseStamped, "/object_pose_in_base", 10)
        self.create_subscription(
            PoseStamped, "/detected_object_pose", self._callback, 10
        )

    def _callback(self, message):
        """按PoseStamped时间戳查询TF并发布Base Frame位姿。"""
        try:
            transformed = self.buffer.transform(
                message, "base_link", timeout=Duration(seconds=0.2)
            )
            self.publisher.publish(transformed)
            p = transformed.pose.position
            self.get_logger().info(
                f"物体在 base_link 中：x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}",
                throttle_duration_sec=2.0,
            )
        except TransformException as error:
            self.get_logger().warning(f"坐标转换失败：{error}", throttle_duration_sec=2.0)


def main(args=None):
    """初始化并持续运行TF2目标位姿转换节点。"""
    rclpy.init(args=args)
    node = PoseTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
