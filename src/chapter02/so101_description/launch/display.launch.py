"""在 RViz 中显示教学版 SO-101，并用滑块修改关节角。"""
# 作者：宇哥的具身笔记


from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """启动SO101 RobotModel、关节滑块、robot_state_publisher与RViz。"""
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("so101_description"), "urdf", "so101.urdf.xacro"]
    )
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file]), value_type=str
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("so101_description"), "rviz", "so101_model.rviz"]
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
