"""利用深度图和 CameraInfo，把中心像素恢复为相机坐标系三维点。"""

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class DepthPixelTo3D(Node):
    """同步32FC1深度与CameraInfo，并发布中心像素的米制三维点。"""

    def __init__(self):
        super().__init__("depth_pixel_to_3d")
        self.bridge = CvBridge()
        self.depth_sub = Subscriber(
            self, Image, "/camera/depth/image_raw", qos_profile=qos_profile_sensor_data
        )
        self.info_sub = Subscriber(
            self,
            CameraInfo,
            "/camera/depth/camera_info",
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = ApproximateTimeSynchronizer(
            [self.depth_sub, self.info_sub], 10, 0.05
        )
        self.sync.registerCallback(self._callback)
        self.publisher = self.create_publisher(PointStamped, "/object_point_camera", 10)

    def _callback(self, depth_message, info):
        """读取中心像素Z，用针孔公式发布camera_depth_optical_frame中的XYZ。"""
        depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="32FC1")
        u, v = info.width // 2, info.height // 2
        z = float(depth[v, u])  # 32FC1 在本项目中使用米。
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        point = PointStamped()
        point.header = depth_message.header
        point.point.x = (u - cx) * z / fx
        point.point.y = (v - cy) * z / fy
        point.point.z = z
        self.publisher.publish(point)


def main(args=None):
    """初始化并持续运行深度像素反投影节点。"""
    rclpy.init(args=args)
    node = DepthPixelTo3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
