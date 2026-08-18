"""文本颜色目标→Mask→目标点云→Camera/Base Frame Pose。"""

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from so101_vision.segmentation import masked_target_cloud, segment_color_prompt
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener

from .ros_helpers import intrinsics_from_info, point_pose


class TargetLocalizerNode(Node):
    """同步RGB-D，发布目标Mask/点云及Camera、Base两种Frame位姿。"""

    def __init__(self):
        super().__init__("target_localizer_node")
        self.declare_parameter("prompt", "red cup")
        self.declare_parameter("erosion_pixels", 2)
        self.declare_parameter("output_frame", "base_link")
        self.bridge = CvBridge()
        self.color_subscriber = Subscriber(
            self, Image, "/camera/color/image_raw", qos_profile=qos_profile_sensor_data
        )
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
            [self.color_subscriber, self.depth_subscriber, self.info_subscriber],
            10,
            0.04,
        )
        self.synchronizer.registerCallback(self.callback)
        self.mask_publisher = self.create_publisher(Image, "/vision/target_mask", 10)
        self.cloud_publisher = self.create_publisher(
            PointCloud2, "/vision/target_cloud", 10
        )
        self.camera_pose_publisher = self.create_publisher(
            PoseStamped, "/vision/target_pose_camera", 10
        )
        self.base_pose_publisher = self.create_publisher(
            PoseStamped, "/vision/target_pose_base", 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.counter = 0

    def callback(self, color_message, depth_message, info_message):
        """执行Prompt分割、深度融合，并用TF2转换目标中心到output_frame。"""
        color = self.bridge.imgmsg_to_cv2(color_message, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="32FC1")
        prompt = str(self.get_parameter("prompt").value)
        try:
            mask = segment_color_prompt(color, prompt)
            points, _, eroded = masked_target_cloud(
                color,
                depth,
                mask,
                intrinsics_from_info(info_message),
                int(self.get_parameter("erosion_pixels").value),
            )
        except ValueError as error:
            self.get_logger().warn(str(error))
            return

        mask_message = self.bridge.cv2_to_imgmsg(eroded, encoding="mono8")
        mask_message.header = color_message.header
        self.mask_publisher.publish(mask_message)
        cloud_header = depth_message.header
        cloud = point_cloud2.create_cloud_xyz32(cloud_header, points[::3].tolist())
        self.cloud_publisher.publish(cloud)
        center = point_pose(depth_message.header, points.mean(axis=0))
        self.camera_pose_publisher.publish(center)

        output_frame = str(self.get_parameter("output_frame").value)
        try:
            # 使用Buffer中最新的完整TF链。若要求高速运动中的严格时序，应改用
            # MultiThreadedExecutor，并按图像时间戳等待对应TF。
            transform = self.tf_buffer.lookup_transform(
                output_frame,
                center.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.08),
            )
            base_pose = do_transform_pose_stamped(center, transform)
            self.base_pose_publisher.publish(base_pose)
        except TransformException as error:
            if self.counter % 20 == 0:
                self.get_logger().warn(f"等待TF：{error}")
        self.counter += 1


def main(args=None):
    """初始化并持续运行文本目标三维定位节点。"""
    rclpy.init(args=args)
    node = TargetLocalizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
