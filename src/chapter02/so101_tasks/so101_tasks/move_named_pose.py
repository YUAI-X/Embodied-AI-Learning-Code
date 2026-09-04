"""移动到 task_geometry.py 中定义的任一教学姿态。"""
# 作者：宇哥的具身笔记


import argparse
import rclpy
from rclpy.node import Node
from .move_group_client import MoveGroupClient
from .task_geometry import NAMED_POSES


def main(args=None):
    """选择预定义SO101关节姿态，并通过MoveGroup规划和执行。"""
    parser = argparse.ArgumentParser(description="移动到 SO-101 命名姿态")
    parser.add_argument("--pose", choices=sorted(NAMED_POSES), default="ready")
    parser.add_argument("--list", action="store_true", help="列出可用姿态后退出")
    parsed, ros_args = parser.parse_known_args(args=args)
    if parsed.list:
        print("\n".join(sorted(NAMED_POSES)))
        return
    rclpy.init(args=ros_args)
    node = Node("move_named_pose")
    node.get_logger().info(f"目标命名姿态：{parsed.pose}")
    MoveGroupClient(node).move_joints(NAMED_POSES[parsed.pose])
    node.destroy_node()
    rclpy.shutdown()
