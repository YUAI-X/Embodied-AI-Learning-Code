"""发送 MoveJoints Goal，并打印 Action Feedback 和 Result。"""
# 作者：宇哥的具身笔记


import argparse
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from so101_interfaces.action import MoveJoints
from so101_math.kinematics import JOINT_NAMES


class MoveJointsClient(Node):
    """发送五关节MoveJoints目标，并依次处理反馈、接受状态与结果。"""

    def __init__(self, positions, duration):
        super().__init__("move_joints_client")
        self._positions = positions
        self._duration = duration
        self._client = ActionClient(self, MoveJoints, "/move_joints")

    def send(self):
        """等待Action Server，按SO101固定关节顺序异步发送目标。"""
        self.get_logger().info("等待 /move_joints Action Server...")
        self._client.wait_for_server()
        goal = MoveJoints.Goal()
        goal.joint_names = list(JOINT_NAMES)
        goal.positions = list(self._positions)
        goal.duration = float(self._duration)
        future = self._client.send_goal_async(goal, feedback_callback=self._feedback)
        future.add_done_callback(self._goal_response)

    def _feedback(self, message):
        """把Action反馈中的0~1进度转换成百分比日志。"""
        self.get_logger().info(f"执行进度：{message.feedback.progress * 100:5.1f}%")

    def _goal_response(self, future):
        """处理目标接受/拒绝，并为已接受目标注册结果回调。"""
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Goal 被拒绝：请检查关节顺序、范围和 duration")
            rclpy.shutdown()
            return
        handle.get_result_async().add_done_callback(self._result)

    def _result(self, future):
        """输出Action最终结果并结束客户端ROS上下文。"""
        result = future.result().result
        self.get_logger().info(f"结果：success={result.success}, message={result.message}")
        rclpy.shutdown()


def main(args=None):
    """解析五关节弧度目标与持续时间，运行MoveJoints客户端。"""
    parser = argparse.ArgumentParser(description="MoveJoints Action 客户端")
    parser.add_argument(
        "--positions", type=float, nargs=5, default=[0.3, -0.6, 1.0, -0.4, 0.3]
    )
    parser.add_argument("--duration", type=float, default=3.0)
    parsed, ros_args = parser.parse_known_args(args=args)
    rclpy.init(args=ros_args)
    node = MoveJointsClient(parsed.positions, parsed.duration)
    node.send()
    rclpy.spin(node)
