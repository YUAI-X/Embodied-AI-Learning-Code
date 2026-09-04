"""启动合成 RGB-D、同步和深度像素转三维点节点。"""
# 作者：宇哥的具身笔记


from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """启动合成RGB-D、近似同步与中心深度反投影节点。"""
    config = PathJoinSubstitution(
        [FindPackageShare("so101_ros2_basics"), "config", "rgbd_camera.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="so101_ros2_basics",
                executable="rgbd_camera_publisher",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="so101_ros2_basics",
                executable="rgbd_sync_node",
                output="screen",
            ),
            Node(
                package="so101_ros2_basics",
                executable="depth_pixel_to_3d",
                output="screen",
            ),
        ]
    )
