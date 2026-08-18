"""同时启动 Action/Service/Topic 与 PID Parameter 教学节点。"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """启动MoveJoints Action/Reset Service与PID Parameter教学节点。"""
    config = PathJoinSubstitution(
        [FindPackageShare("so101_ros2_basics"), "config", "basics.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="so101_ros2_basics",
                executable="move_joints_server",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="so101_ros2_basics",
                executable="pid_parameter_node",
                parameters=[config],
                output="screen",
            ),
        ]
    )
