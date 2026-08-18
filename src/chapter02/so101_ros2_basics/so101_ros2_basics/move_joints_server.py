"""用于讲解 ROS 2 Action 的模拟关节控制器。"""

import asyncio
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState
from so101_interfaces.action import MoveJoints
from so101_interfaces.srv import ResetRobot
from so101_math.kinematics import JOINT_NAMES, JOINT_LIMITS


HOME = np.array([0.0, -0.45, 0.9, -0.45, 0.0])


class MoveJointsServer(Node):
    """发布关节状态，并提供带反馈和取消功能的关节运动 Action。"""

    def __init__(self):
        super().__init__("move_joints_server")
        self.declare_parameter("publish_rate", 30.0)
        self._positions = HOME.copy()
        self._publisher = self.create_publisher(JointState, "/joint_states", 10)
        period = 1.0 / float(self.get_parameter("publish_rate").value)
        self.create_timer(period, self._publish_state)
        self.create_service(ResetRobot, "/reset_robot", self._reset_callback)
        self._action_server = ActionServer(
            self,
            MoveJoints,
            "/move_joints",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info("模拟控制器已启动：/joint_states、/reset_robot、/move_joints")

    def _publish_state(self):
        """以固定顺序发布当前五关节与夹爪的JointState。"""
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES) + ["gripper"]
        message.position = self._positions.tolist() + [0.6]
        self._publisher.publish(message)

    def _goal_callback(self, goal):
        """在执行前检查关节顺序、有限值、官方限制与持续时间。"""
        if list(goal.joint_names) != list(JOINT_NAMES):
            self.get_logger().warning("拒绝目标：关节名或顺序不正确")
            return GoalResponse.REJECT
        positions = np.asarray(goal.positions, dtype=float)
        if positions.shape != (5,) or not np.all(np.isfinite(positions)):
            return GoalResponse.REJECT
        if np.any(positions < JOINT_LIMITS[:, 0]) or np.any(
            positions > JOINT_LIMITS[:, 1]
        ):
            return GoalResponse.REJECT
        if goal.duration <= 0.0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        """接受客户端取消请求，实际停止由执行循环检测。"""
        self.get_logger().info("收到取消请求")
        return CancelResponse.ACCEPT

    async def _execute_callback(self, goal_handle):
        """用smoothstep插值模拟运动，并周期发布进度与当前关节反馈。"""
        start = self._positions.copy()
        target = np.asarray(goal_handle.request.positions, dtype=float)
        duration = float(goal_handle.request.duration)
        steps = max(2, int(duration * 30.0))
        feedback = MoveJoints.Feedback()

        for index in range(1, steps + 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = MoveJoints.Result()
                result.success = False
                result.message = "运动已由客户端取消"
                return result
            ratio = index / steps
            # smoothstep 插值在起点和终点处速度为零，比直接线性插值更容易观察。
            smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio)
            self._positions = start + (target - start) * smooth_ratio
            feedback.progress = float(ratio)
            feedback.current_positions = self._positions.tolist()
            goal_handle.publish_feedback(feedback)
            await asyncio.sleep(duration / steps)

        goal_handle.succeed()
        result = MoveJoints.Result()
        result.success = True
        result.message = "目标关节位置已到达"
        return result

    def _reset_callback(self, _request, response):
        """立即恢复教学Home关节值，并返回Service响应。"""
        self._positions = HOME.copy()
        response.success = True
        response.message = "已立即复位到 home；Service 适合短时请求，不提供运动进度"
        return response


def main(args=None):
    """初始化JointState、Reset Service与MoveJoints Action综合服务节点。"""
    rclpy.init(args=args)
    node = MoveJointsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
