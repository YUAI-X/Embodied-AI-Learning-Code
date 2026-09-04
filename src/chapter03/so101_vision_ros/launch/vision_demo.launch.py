"""一条命令启动SO101、合成RGB-D、点云和目标定位教学流水线。"""
# 作者：宇哥的具身笔记


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """组合第2章SO101控制、合成RGB-D、感知节点和可选RViz。"""
    use_rviz = LaunchConfiguration("use_rviz")
    prompt = LaunchConfiguration("prompt")
    control_launch = PathJoinSubstitution(
        [FindPackageShare("so101_control"), "launch", "mock_control.launch.py"]
    )
    parameters = PathJoinSubstitution(
        [FindPackageShare("so101_vision_ros"), "config", "vision.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("so101_vision_ros"), "rviz", "vision_demo.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "prompt",
                default_value="red cup",
                description="轻量示例支持red/blue/green或对应中文颜色词",
            ),
            # 复用第2章：官方SO101模型、robot_state_publisher与Mock Hardware。
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(control_launch),
                launch_arguments={"use_rviz": "false"}.items(),
            ),
            Node(
                package="so101_vision_ros",
                executable="synthetic_rgbd_publisher",
                parameters=[parameters],
                output="screen",
            ),
            Node(
                package="so101_vision_ros",
                executable="rgbd_pointcloud_node",
                parameters=[parameters],
                output="screen",
            ),
            Node(
                package="so101_vision_ros",
                executable="target_localizer_node",
                parameters=[parameters, {"prompt": prompt}],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
