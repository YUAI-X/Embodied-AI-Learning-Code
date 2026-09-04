"""虚拟抓放共用的关节姿态与几何常量。
# 作者：宇哥的具身笔记


所有位置单位为米、关节单位为弧度。方块在tool0中的偏移根据官方SO101夹爪网格确定：
它位于固定指与活动指之间，而不是gripper_link的任意轴向偏移。
"""

import numpy as np

from so101_math.kinematics import forward_kinematics


ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]

# 抓取路径先从方块上方进入，附着后原路抬升；放置路径同理。
NAMED_POSES = {
    "home": [0.0, -0.45, 0.90, -0.45, 0.0],
    "ready": [0.15, -0.70, 1.15, -0.35, 0.0],
    "pregrasp": [-0.082329, -0.340473, 1.023120, -0.319015, -0.004955],
    "grasp": [-0.082337, -0.075981, 0.924257, -0.210834, -0.005287],
    "carry": [-0.35, -0.35, 0.75, -0.25, 0.0],
    "preplace": [0.648321, -0.808158, 1.026143, 0.060328, -0.252863],
    "place": [0.648366, -0.281617, 1.027674, -0.000652, -0.251075],
}

CUBE_SIZE = 0.035
PICK_CUBE_POSITION = np.array([0.29, 0.02, 0.06])
PLACE_CUBE_POSITION = np.array([0.22, -0.14, 0.06])

# tool0位于官方gripper_frame_link。该偏移把35 mm方块中心放在两夹指之间。
CUBE_IN_TOOL0 = np.array([-0.014, 0.0, -0.028])

GRIPPER_OPEN = 0.8
# 约0.24 rad时，两指在方块高度处的间距接近35 mm。
GRIPPER_HOLD = 0.24

GRIPPER_TOUCH_LINKS = [
    "gripper_link",
    "moving_jaw_so101_v1_link",
    "gripper_frame_link",
    "tool0",
]


def cube_center_in_base(joints) -> np.ndarray:
    """根据五关节角计算附着方块中心在base_link中的位置。"""
    tool_transform = forward_kinematics(np.asarray(joints, dtype=float))
    homogeneous = np.r_[CUBE_IN_TOOL0, 1.0]
    return (tool_transform @ homogeneous)[:3]


def geometry_errors() -> tuple[float, float]:
    """返回抓取和放置姿态的方块中心误差，单位为米。"""
    pick_error = np.linalg.norm(
        cube_center_in_base(NAMED_POSES["grasp"]) - PICK_CUBE_POSITION
    )
    place_error = np.linalg.norm(
        cube_center_in_base(NAMED_POSES["place"]) - PLACE_CUBE_POSITION
    )
    return float(pick_error), float(place_error)
