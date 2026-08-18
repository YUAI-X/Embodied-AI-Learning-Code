"""用 rclpy 直接调用 MoveIt 2 的标准 MoveGroup Action。

ROS 2 Humble 的官方入门接口以 C++ 为主。本辅助类把较长的消息构造封装起来，同时保留
Action 的 Goal/Result 结构，便于和 2.4 的知识衔接。
"""

from typing import Sequence
import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient

from .task_geometry import ARM_JOINTS


class MoveGroupClient:
    """把五关节目标包装为MoveGroup Goal，并同步等待规划与虚拟执行。"""

    def __init__(self, node):
        self.node = node
        self.client = ActionClient(node, MoveGroup, "/move_action")

    def move_joints(self, positions: Sequence[float]) -> bool:
        """规划并执行按ARM_JOINTS排列的五个弧度目标；返回MoveIt是否成功。"""
        if len(positions) != len(ARM_JOINTS):
            raise ValueError("arm 规划组需要 5 个关节目标")
        self.node.get_logger().info("等待 MoveIt /move_action...")
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error(
                "找不到 /move_action，请先启动 so101_moveit_config/demo.launch.py"
            )
            return False

        constraints = Constraints()
        constraints.name = "教学关节目标"
        for name, position in zip(ARM_JOINTS, positions):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(position)
            # 抓放姿态需要毫米级几何一致性。3 mrad仍给控制器保留合理余量，
            # 同时避免1 cm级关节容差累积成肉眼可见的方块跳动。
            joint.tolerance_above = 0.003
            joint.tolerance_below = 0.003
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)

        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.goal_constraints = [constraints]
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.25
        goal.request.max_acceleration_scaling_factor = 0.25
        # plan_only=False：MoveIt 规划成功后继续通过 FollowJointTrajectory 执行。
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.node.get_logger().error("MoveGroup Goal 被拒绝")
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result = result_future.result().result
        success = result.error_code.val == MoveItErrorCodes.SUCCESS
        if success:
            self.node.get_logger().info("规划与执行成功")
        else:
            self.node.get_logger().error(
                f"MoveIt 失败，错误码：{result.error_code.val}"
            )
        return success
