"""把同步Depth与CameraInfo转换成sensor_msgs/PointCloud2。"""
# 作者：宇哥的具身笔记


import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from so101_vision.camera_geometry import depth_to_pointcloud

from .ros_helpers import intrinsics_from_info


class RgbdPointCloudNode(Node):
    """同步米制Depth与CameraInfo，并发布相机光学Frame场景点云。"""

    def __init__(self):
        super().__init__("rgbd_pointcloud_node")
        self.declare_parameter("pixel_stride", 4)
        self.declare_parameter("max_depth_m", 1.2)
        self.bridge = CvBridge()
        self.depth_subscriber = Subscriber(
            self, Image, "/camera/depth/image_raw", qos_profile=qos_profile_sensor_data
        )
        self.info_subscriber = Subscriber(
            self,
            CameraInfo,
            "/camera/depth/camera_info",
            qos_profile=qos_profile_sensor_data,
        )
        self.synchronizer = ApproximateTimeSynchronizer(
            [self.depth_subscriber, self.info_subscriber], 10, 0.04
        )
        self.synchronizer.registerCallback(self.callback)
        self.publisher = self.create_publisher(PointCloud2, "/vision/scene_cloud", 10)

    def callback(self, depth_message, info_message):
        """将一组同步消息反投影；pixel_stride用于限制教学点云密度。"""
        depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="32FC1")
        stride = int(self.get_parameter("pixel_stride").value)
        mask = stride_mask(depth.shape, stride)
        points, _ = depth_to_pointcloud(
            depth,
            intrinsics_from_info(info_message),
            mask=mask,
            max_depth=float(self.get_parameter("max_depth_m").value),
        )
        cloud = point_cloud2.create_cloud_xyz32(depth_message.header, points.tolist())
        self.publisher.publish(cloud)


def stride_mask(shape, stride):
    """显式生成采样Mask，便于学员理解点数和像素步长的关系。"""
    import numpy as np

    if stride < 1:
        raise ValueError("pixel_stride必须大于或等于1")
    mask = np.zeros(shape, dtype=bool)
    mask[::stride, ::stride] = True
    return mask


def main(args=None):
    """初始化并持续运行RGB-D场景点云节点。"""
    rclpy.init(args=args)
    node = RgbdPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
