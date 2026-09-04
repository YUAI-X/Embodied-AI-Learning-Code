"""ROS消息与第三章NumPy数据之间的薄适配层。"""
# 作者：宇哥的具身笔记


from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from so101_vision.camera_geometry import CameraIntrinsics


def intrinsics_from_info(message: CameraInfo) -> CameraIntrinsics:
    """从CameraInfo.K提取与消息分辨率一致的针孔内参。"""
    return CameraIntrinsics(
        int(message.width),
        int(message.height),
        float(message.k[0]),
        float(message.k[4]),
        float(message.k[2]),
        float(message.k[5]),
    )


def point_pose(header, xyz) -> PoseStamped:
    """把米制三维点包装成单位朝向PoseStamped，并保留原Header。"""
    pose = PoseStamped()
    pose.header = header
    pose.pose.position.x = float(xyz[0])
    pose.pose.position.y = float(xyz[1])
    pose.pose.position.z = float(xyz[2])
    pose.pose.orientation.w = 1.0
    return pose
