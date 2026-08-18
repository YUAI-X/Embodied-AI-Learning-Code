"""发布3.1–3.5共用的合成桌面RGB-D和CameraInfo。"""

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from so101_vision.synthetic_data import synthetic_rgbd


class SyntheticRgbdPublisher(Node):
    """按相同时间戳发布对齐BGR、32FC1米制Depth与CameraInfo。"""

    def __init__(self):
        super().__init__("synthetic_rgbd_publisher")
        self.declare_parameter("fps", 5.0)
        self.color, self.depth, self.intrinsics, _ = synthetic_rgbd()
        self.bridge = CvBridge()
        self.color_publisher = self.create_publisher(
            Image, "/camera/color/image_raw", qos_profile_sensor_data
        )
        self.depth_publisher = self.create_publisher(
            Image, "/camera/depth/image_raw", qos_profile_sensor_data
        )
        self.info_publisher = self.create_publisher(
            CameraInfo, "/camera/depth/camera_info", qos_profile_sensor_data
        )
        fps = float(self.get_parameter("fps").value)
        self.create_timer(1.0 / fps, self.publish_frame)
        self.get_logger().info("发布第三章合成桌面RGB-D：Depth为32FC1米制数据")

    def camera_info(self, stamp):
        """构造深度光学Frame的无畸变CameraInfo消息。"""
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = "camera_depth_optical_frame"
        info.width = self.intrinsics.width
        info.height = self.intrinsics.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = self.intrinsics.matrix.reshape(-1).tolist()
        info.p = [
            self.intrinsics.fx,
            0.0,
            self.intrinsics.cx,
            0.0,
            0.0,
            self.intrinsics.fy,
            self.intrinsics.cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return info

    def publish_frame(self):
        """为一帧RGB-D设置共同时间戳后发布三条传感器消息。"""
        stamp = self.get_clock().now().to_msg()
        color_message = self.bridge.cv2_to_imgmsg(self.color, encoding="bgr8")
        color_message.header.stamp = stamp
        color_message.header.frame_id = "camera_color_optical_frame"
        depth_message = self.bridge.cv2_to_imgmsg(self.depth, encoding="32FC1")
        depth_message.header.stamp = stamp
        depth_message.header.frame_id = "camera_depth_optical_frame"
        self.color_publisher.publish(color_message)
        self.depth_publisher.publish(depth_message)
        self.info_publisher.publish(self.camera_info(stamp))


def main(args=None):
    """初始化并持续运行合成RGB-D发布器。"""
    rclpy.init(args=args)
    node = SyntheticRgbdPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
