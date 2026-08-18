"""近似同步 RGB 和深度图，演示时间戳与队列。"""

import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class RgbdSyncNode(Node):
    """用Sensor Data QoS近似同步彩色图与米制深度图。"""

    def __init__(self):
        super().__init__("rgbd_sync_node")
        self.bridge = CvBridge()
        self.color_sub = Subscriber(
            self, Image, "/camera/color/image_raw", qos_profile=qos_profile_sensor_data
        )
        self.depth_sub = Subscriber(
            self, Image, "/camera/depth/image_raw", qos_profile=qos_profile_sensor_data
        )
        synchronizer = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub], queue_size=10, slop=0.05
        )
        synchronizer.registerCallback(self._callback)
        self.synchronizer = synchronizer  # 保留引用，避免被垃圾回收。
        self.counter = 0

    def _callback(self, color_message, depth_message):
        """计算两消息时间差，并周期报告中心像素深度。"""
        depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="32FC1")
        center_depth = float(depth[depth.shape[0] // 2, depth.shape[1] // 2])
        self.counter += 1
        if self.counter % 15 == 0:
            color_time = (
                color_message.header.stamp.sec
                + color_message.header.stamp.nanosec * 1e-9
            )
            depth_time = (
                depth_message.header.stamp.sec
                + depth_message.header.stamp.nanosec * 1e-9
            )
            self.get_logger().info(
                f"同步成功：中心深度={center_depth:.3f} m，时间差={abs(color_time-depth_time)*1000:.2f} ms"
            )


def main(args=None):
    """初始化并持续运行RGB-D近似时间同步教学节点。"""
    rclpy.init(args=args)
    node = RgbdSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
