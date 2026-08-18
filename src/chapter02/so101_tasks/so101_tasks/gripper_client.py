"""ros2_control GripperActionController 的简洁 Python 客户端。"""

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient


class GripperClient:
    """同步封装ros2_control GripperCommand Action，供顺序任务调用。"""

    def __init__(self, node):
        self.node = node
        self.client = ActionClient(
            node, GripperCommand, "/gripper_controller/gripper_cmd"
        )

    def command(self, position):
        """发送夹爪位置目标并等待结果；成功返回True，单位沿控制器配置。"""
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error("找不到夹爪 Action")
            return False
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 5.0
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        return bool(result_future.result())
