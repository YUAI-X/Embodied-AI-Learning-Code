"""发布同步的合成 RGB、深度图和 CameraInfo。
# 作者：宇哥的具身笔记


默认不连接真实相机，便于所有学员学习 sensor_msgs/Image、encoding、时间戳和 QoS。
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class RgbdCameraPublisher(Node):
    """发布同时间戳的bgr8彩色图、32FC1米制深度和两组CameraInfo。"""

    def __init__(self):
        super().__init__("rgbd_camera_publisher")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("depth_m", 0.45)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        fps = float(self.get_parameter("fps").value)
        self.bridge = CvBridge()
        self.frame_index = 0

        self.color_pub = self.create_publisher(
            Image, "/camera/color/image_raw", qos_profile_sensor_data
        )
        self.depth_pub = self.create_publisher(
            Image, "/camera/depth/image_raw", qos_profile_sensor_data
        )
        self.color_info_pub = self.create_publisher(
            CameraInfo, "/camera/color/camera_info", qos_profile_sensor_data
        )
        self.depth_info_pub = self.create_publisher(
            CameraInfo, "/camera/depth/camera_info", qos_profile_sensor_data
        )
        self.create_timer(1.0 / fps, self._publish)
        self.get_logger().info("发布合成 RGB-D：RGB=bgr8，Depth=32FC1（单位：米）")

    def _camera_info(self, stamp, frame_id):
        """按当前分辨率构造无畸变针孔CameraInfo，焦距单位为像素。"""
        focal = 525.0
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.width = self.width
        message.height = self.height
        message.distortion_model = "plumb_bob"
        message.d = [0.0] * 5
        message.k = [
            focal,
            0.0,
            self.width / 2.0,
            0.0,
            focal,
            self.height / 2.0,
            0.0,
            0.0,
            1.0,
        ]
        message.p = [
            focal,
            0.0,
            self.width / 2.0,
            0.0,
            0.0,
            focal,
            self.height / 2.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return message

    def _publish(self):
        """生成移动红色方块及对应深度，并发布一组同步RGB-D消息。"""
        stamp = self.get_clock().now().to_msg()
        color = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        color[:] = (35, 35, 35)

        # 让方块轻微左右运动，可观察 RGB 和深度是否保持同一时间戳。
        center_x = self.width // 2 + int(80 * np.sin(self.frame_index * 0.05))
        center_y = self.height // 2
        half = 45
        cv2.rectangle(
            color,
            (center_x - half, center_y - half),
            (center_x + half, center_y + half),
            (30, 80, 230),
            -1,
        )
        cv2.putText(
            color,
            "SO-101 RGB-D demo",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (230, 230, 230),
            2,
        )

        # 深度图以米为单位；背景较远，方块较近。
        depth = np.full((self.height, self.width), 0.75, dtype=np.float32)
        object_depth = float(self.get_parameter("depth_m").value)
        row_slice = slice(center_y - half, center_y + half)
        column_slice = slice(center_x - half, center_x + half)
        depth[row_slice, column_slice] = object_depth

        color_message = self.bridge.cv2_to_imgmsg(color, encoding="bgr8")
        color_message.header.stamp = stamp
        color_message.header.frame_id = "camera_color_optical_frame"
        depth_message = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        depth_message.header.stamp = stamp
        depth_message.header.frame_id = "camera_depth_optical_frame"

        self.color_pub.publish(color_message)
        self.depth_pub.publish(depth_message)
        self.color_info_pub.publish(
            self._camera_info(stamp, "camera_color_optical_frame")
        )
        self.depth_info_pub.publish(
            self._camera_info(stamp, "camera_depth_optical_frame")
        )
        self.frame_index += 1


def main(args=None):
    """初始化并持续运行无需硬件的合成RGB-D相机。"""
    rclpy.init(args=args)
    node = RgbdCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
