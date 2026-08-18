"""章末任务：无 Gazebo、无真机的 SO-101 虚拟抓放。"""

import time
import rclpy
from rclpy.node import Node
from .gripper_client import GripperClient
from .move_group_client import MoveGroupClient
from .planning_scene_client import PlanningSceneClient
from .task_geometry import (
    GRIPPER_HOLD,
    GRIPPER_OPEN,
    NAMED_POSES,
    geometry_errors,
)


def require(ok, step):
    """把任务步骤的False结果转为带步骤名称的异常。"""
    if not ok:
        raise RuntimeError(f"步骤失败：{step}")


def main(args=None):
    """按几何一致的接近、抓取、抬升、搬运与放置顺序执行虚拟任务。"""
    rclpy.init(args=args)
    node = Node("virtual_pick_place")
    mover = MoveGroupClient(node)
    scene = PlanningSceneClient(node)
    gripper = GripperClient(node)
    succeeded = False
    try:
        pick_error, place_error = geometry_errors()
        node.get_logger().info(
            f"抓取/放置几何误差：{pick_error * 1000:.2f} / "
            f"{place_error * 1000:.2f} mm"
        )
        require(max(pick_error, place_error) < 0.002, "校验任务几何")

        node.get_logger().info("1/11 添加桌子和方块")
        require(scene.add_world(), "添加 Planning Scene")
        time.sleep(1.0)

        node.get_logger().info("2/11 打开夹爪并移动到 ready")
        require(gripper.command(GRIPPER_OPEN), "打开夹爪")
        require(mover.move_joints(NAMED_POSES["ready"]), "移动到 ready")

        node.get_logger().info("3/11 从方块上方移动到 pregrasp")
        require(mover.move_joints(NAMED_POSES["pregrasp"]), "移动到 pregrasp")

        node.get_logger().info("4/11 允许夹指接触方块并下降到 grasp")
        require(scene.allow_gripper_cube_collision(True), "允许夹爪接触方块")
        require(mover.move_joints(NAMED_POSES["grasp"]), "移动到 grasp")

        node.get_logger().info("5/11 闭合到方块宽度并逻辑附着")
        require(gripper.command(GRIPPER_HOLD), "闭合夹爪")
        require(scene.attach_cube(), "附着方块")
        require(scene.allow_gripper_cube_collision(False), "恢复方块碰撞检查")
        time.sleep(0.8)

        node.get_logger().info("6/11 沿抓取路径抬升回 pregrasp")
        require(mover.move_joints(NAMED_POSES["pregrasp"]), "抓取后抬升")

        node.get_logger().info("7/11 搬运到 carry")
        require(mover.move_joints(NAMED_POSES["carry"]), "移动到 carry")

        node.get_logger().info("8/11 移动到目标上方 preplace")
        require(mover.move_joints(NAMED_POSES["preplace"]), "移动到 preplace")

        node.get_logger().info("9/11 下降到 place")
        require(mover.move_joints(NAMED_POSES["place"]), "移动到 place")

        node.get_logger().info("10/11 分离方块、打开夹爪并抬升")
        require(scene.allow_gripper_cube_collision(True), "允许夹爪离开放置方块")
        require(scene.detach_cube(), "分离方块")
        require(gripper.command(GRIPPER_OPEN), "打开夹爪")
        require(mover.move_joints(NAMED_POSES["preplace"]), "放置后抬升")
        require(scene.allow_gripper_cube_collision(False), "恢复方块碰撞检查")

        node.get_logger().info("11/11 返回 home")
        require(mover.move_joints(NAMED_POSES["home"]), "返回 home")
        node.get_logger().info("章末虚拟抓放任务完成")
        succeeded = True
    except RuntimeError as error:
        node.get_logger().error(str(error))
    finally:
        # 任务中途退出时也恢复碰撞检查，避免下一次运行继承宽松ACM。
        scene.allow_gripper_cube_collision(False)
        node.destroy_node()
        rclpy.shutdown()
    if not succeeded:
        raise SystemExit(1)
