"""SO-101 MoveIt 2 + ros2_control Mock Hardware 一键启动。"""
# 作者：宇哥的具身笔记


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """组合SO101 MoveIt、Mock控制器、Move Group和可选RViz。"""
    use_rviz = LaunchConfiguration("use_rviz")
    moveit_config = (
        MoveItConfigsBuilder("so101", package_name="so101_moveit_config")
        .robot_description(mappings={"use_mock_hardware": "true"})
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs()
    )
    controllers = PathJoinSubstitution(
        [FindPackageShare("so101_control"), "config", "ros2_controllers.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("so101_moveit_config"), "rviz", "moveit.rviz"]
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "capabilities": "",
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="是否启动 RViz；无桌面环境可设为 false",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[moveit_config.robot_description],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[moveit_config.robot_description, controllers],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gripper_controller"],
            ),
            move_group,
            Node(
                package="rviz2",
                executable="rviz2",
                condition=IfCondition(use_rviz),
                arguments=["-d", rviz_config],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                ],
                output="screen",
            ),
        ]
    )
