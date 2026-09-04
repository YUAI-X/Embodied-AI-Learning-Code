"""向 MoveIt Planning Scene 添加、附着和分离教学物体。"""
# 作者：宇哥的具身笔记


import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneComponents,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive

from .task_geometry import (
    CUBE_IN_TOOL0,
    CUBE_SIZE,
    GRIPPER_TOUCH_LINKS,
    PICK_CUBE_POSITION,
    PLACE_CUBE_POSITION,
)


def box_object(object_id, frame_id, size, xyz):
    """构造轴对齐Box碰撞物；size与xyz均以米表示。"""
    collision = CollisionObject()
    collision.header.frame_id = frame_id
    collision.id = object_id
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(map(float, size))
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, xyz)
    pose.orientation.w = 1.0
    collision.primitives = [primitive]
    collision.primitive_poses = [pose]
    collision.operation = CollisionObject.ADD
    return collision


class PlanningSceneClient:
    """封装Planning Scene差分服务，用于添加、附着和释放教学方块。"""

    def __init__(self, node):
        self.node = node
        self.client = node.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        self.get_client = node.create_client(
            GetPlanningScene, "/get_planning_scene"
        )

    def apply(self, scene):
        """调用/apply_planning_scene并同步等待服务结果。"""
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.node.get_logger().error("找不到 /apply_planning_scene")
            return False
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return bool(future.result() and future.result().success)

    def add_world(self):
        """清理残留附着物，并在base_link中加入桌面与待抓方块。"""
        scene = PlanningScene()
        scene.is_diff = True
        # 允许脚本反复运行：先清理上一次中途退出后可能残留的附着方块。
        scene.robot_state.is_diff = True
        stale_attachments = []
        # 同时兼容修复前的gripper_link附着方式与当前tool0附着方式。
        for link_name in ("gripper_link", "tool0"):
            stale_attachment = AttachedCollisionObject()
            stale_attachment.link_name = link_name
            stale_attachment.object.id = "cube"
            stale_attachment.object.operation = CollisionObject.REMOVE
            stale_attachments.append(stale_attachment)
        scene.robot_state.attached_collision_objects = stale_attachments
        scene.world.collision_objects = [
            # Planning Scene 的模型根坐标系是 base_link。直接使用它可以避免
            # 在场景服务处理差分消息时依赖外部 TF 的到达时序。
            box_object(
                "table", "base_link", [0.55, 0.55, 0.04], [0.20, 0.0, -0.02]
            ),
            box_object(
                "cube",
                "base_link",
                [CUBE_SIZE] * 3,
                PICK_CUBE_POSITION,
            ),
        ]
        return self.apply(scene)

    def allow_gripper_cube_collision(self, allowed):
        """仅切换方块与夹爪触碰Link之间的碰撞许可。

        接近方块前需要允许两夹指接触方块；机械臂其他Link与方块仍参与碰撞检查。
        """
        if not self.get_client.wait_for_service(timeout_sec=10.0):
            self.node.get_logger().error("找不到 /get_planning_scene")
            return False
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )
        future = self.get_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        if response is None:
            return False

        matrix = response.scene.allowed_collision_matrix
        names = list(matrix.entry_names)
        rows = [list(entry.enabled) for entry in matrix.entry_values]
        required_names = ["cube", *GRIPPER_TOUCH_LINKS]
        for name in required_names:
            if name in names:
                continue
            for row in rows:
                row.append(False)
            names.append(name)
            rows.append([False] * len(names))

        cube_index = names.index("cube")
        for link_name in GRIPPER_TOUCH_LINKS:
            link_index = names.index(link_name)
            rows[cube_index][link_index] = bool(allowed)
            rows[link_index][cube_index] = bool(allowed)

        matrix.entry_names = names
        matrix.entry_values = []
        for row in rows:
            entry = AllowedCollisionEntry()
            entry.enabled = row
            matrix.entry_values.append(entry)
        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = matrix
        return self.apply(scene)

    def attach_cube(self):
        """从World移除方块，再按当前几何位置附着到tool0。"""
        # 方块在tool0中的偏移与grasp关节姿态经过FK配对，因此附着前后方块
        # 中心保持在同一世界坐标，不会突然跳到夹爪的另一侧。
        remove_scene = PlanningScene()
        remove_scene.is_diff = True
        remove = CollisionObject()
        remove.header.frame_id = "base_link"
        remove.id = "cube"
        remove.operation = CollisionObject.REMOVE
        remove_scene.world.collision_objects = [remove]
        if not self.apply(remove_scene):
            return False

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        attached = AttachedCollisionObject()
        attached.link_name = "tool0"
        attached.object = box_object(
            "cube", "tool0", [CUBE_SIZE] * 3, CUBE_IN_TOOL0
        )
        attached.touch_links = GRIPPER_TOUCH_LINKS
        scene.robot_state.attached_collision_objects = [attached]
        return self.apply(scene)

    def detach_cube(self):
        """解除夹爪附着，并在base_link中的放置位置重新加入方块。"""
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        attached = AttachedCollisionObject()
        attached.link_name = "tool0"
        attached.object.id = "cube"
        attached.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects = [attached]
        scene.world.collision_objects = [
            box_object(
                "cube",
                "base_link",
                [CUBE_SIZE] * 3,
                PLACE_CUBE_POSITION,
            )
        ]
        # 此时夹指仍与刚释放的方块接触。调用方应先打开夹爪并抬升，
        # 离开方块后再恢复碰撞检查，否则MoveIt会判定起始状态碰撞。
        return self.apply(scene)
