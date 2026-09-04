"""使用静态 TF 演示相机位姿到机器人基座的转换。"""
# 作者：宇哥的具身笔记


from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """启动静态相机TF、模拟物体Pose与Base Frame转换节点。"""
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x", "0.18", "--y", "-0.10", "--z", "0.25",
                    "--roll", "-1.570796", "--pitch", "0.0", "--yaw", "-1.047198",
                    "--frame-id", "base_link",
                    "--child-frame-id", "camera_color_optical_frame",
                ],
            ),
            Node(package="so101_ros2_basics", executable="mock_object_publisher"),
            Node(package="so101_ros2_basics", executable="pose_transformer", output="screen"),
        ]
    )
