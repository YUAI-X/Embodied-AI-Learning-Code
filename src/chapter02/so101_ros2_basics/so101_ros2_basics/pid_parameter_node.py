"""演示如何把 2.1 的 PID 参数放进 ROS 2 Parameter。"""
# 作者：宇哥的具身笔记


import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node


class PidParameterNode(Node):
    """声明可运行时调整的非负PID参数，并周期打印当前值。"""

    def __init__(self):
        super().__init__("pid_parameter_node")
        self.declare_parameter("kp", 3.0)
        self.declare_parameter("ki", 0.0)
        self.declare_parameter("kd", 0.2)
        self.add_on_set_parameters_callback(self._on_parameters)
        self.create_timer(5.0, self._print_parameters)
        self._print_parameters()

    def _on_parameters(self, parameters):
        """拒绝负PID增益；本节点仅演示Parameter校验，不驱动控制器。"""
        for parameter in parameters:
            if parameter.name in {"kp", "ki", "kd"} and parameter.value < 0.0:
                return SetParametersResult(successful=False, reason="本教学样例要求 PID 参数非负")
        return SetParametersResult(successful=True)

    def _print_parameters(self):
        """读取并打印节点当前Kp、Ki、Kd。"""
        self.get_logger().info(
            "当前参数：Kp=%s, Ki=%s, Kd=%s"
            % tuple(self.get_parameter(name).value for name in ("kp", "ki", "kd"))
        )


def main(args=None):
    """初始化并持续运行PID Parameter教学节点。"""
    rclpy.init(args=args)
    node = PidParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
