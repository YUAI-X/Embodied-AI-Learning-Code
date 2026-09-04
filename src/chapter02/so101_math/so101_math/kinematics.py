"""SO-101 教学运动学工具。
# 作者：宇哥的具身笔记


这里使用官方 new-calibration URDF 的 joint origin、RPY 和 axis。代码刻意只依赖
NumPy，方便学员把齐次变换与 URDF 逐项对应起来。
"""

from dataclasses import dataclass
import numpy as np


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]

# 每个关节相对父 Link 的固定平移与 RPY，数据来自官方 SO-101 URDF。
JOINT_ORIGINS = np.array(
    [
        [0.0388353, -8.97657e-09, 0.0624],
        [-0.0303992, -0.0182778, -0.0542],
        [-0.11257, -0.028, 1.73763e-16],
        [-0.1349, 0.0052, 3.62355e-17],
        [5.55112e-17, -0.0611, 0.0181],
    ],
    dtype=float,
)

JOINT_RPYS = np.array(
    [
        [np.pi, 4.18253e-17, -np.pi],
        [-np.pi / 2.0, -np.pi / 2.0, 0.0],
        [-3.63608e-16, 8.74301e-16, np.pi / 2.0],
        [4.02456e-15, 8.67362e-16, -np.pi / 2.0],
        [np.pi / 2.0, 0.0486795, np.pi],
    ],
    dtype=float,
)

# 官方 URDF 中 5 个机械臂关节都绕各自 joint frame 的 Z 轴转动。
JOINT_AXES = np.tile([0.0, 0.0, 1.0], (5, 1))

JOINT_LIMITS = np.array(
    [
        [-1.91986, 1.91986],
        [-1.74533, 1.74533],
        [-1.69, 1.69],
        [-1.65806, 1.65806],
        [-2.74385, 2.84121],
    ],
    dtype=float,
)

# gripper_link 到官方 gripper_frame_link（本仓库 tool0 与其重合）的固定变换。
TOOL_ORIGIN = np.array([-0.0079, -0.000218121, -0.0981274], dtype=float)
TOOL_RPY = np.array([0.0, np.pi, 0.0], dtype=float)


def translation_matrix(vector: np.ndarray) -> np.ndarray:
    """将三维平移向量转换为 4×4 齐次变换矩阵。"""
    result = np.eye(4)
    result[:3, 3] = np.asarray(vector, dtype=float)
    return result


def axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """使用 Rodrigues 公式计算绕任意单位轴的旋转矩阵。"""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    one_minus_c = 1.0 - c
    rotation = np.array(
        [
            [
                c + x * x * one_minus_c,
                x * y * one_minus_c - z * s,
                x * z * one_minus_c + y * s,
            ],
            [
                y * x * one_minus_c + z * s,
                c + y * y * one_minus_c,
                y * z * one_minus_c - x * s,
            ],
            [
                z * x * one_minus_c - y * s,
                z * y * one_minus_c + x * s,
                c + z * z * one_minus_c,
            ],
        ]
    )
    result = np.eye(4)
    result[:3, :3] = rotation
    return result


def rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    """按 URDF 约定把 roll、pitch、yaw 转成齐次旋转矩阵。"""
    roll, pitch, yaw = np.asarray(rpy, dtype=float)
    return (
        axis_angle_matrix([0, 0, 1], yaw)
        @ axis_angle_matrix([0, 1, 0], pitch)
        @ axis_angle_matrix([1, 0, 0], roll)
    )


def forward_kinematics(joints: np.ndarray) -> np.ndarray:
    """根据 5 个机械臂关节角计算 base_link 到 tool0 的变换。"""
    joints = np.asarray(joints, dtype=float)
    if joints.shape != (5,):
        raise ValueError("需要按顺序提供 5 个机械臂关节角")

    transform = np.eye(4)
    for origin, rpy, axis, angle in zip(
        JOINT_ORIGINS, JOINT_RPYS, JOINT_AXES, joints
    ):
        transform = transform @ translation_matrix(origin) @ rpy_matrix(rpy)
        transform = transform @ axis_angle_matrix(axis, angle)
    return transform @ translation_matrix(TOOL_ORIGIN) @ rpy_matrix(TOOL_RPY)


def numerical_position_jacobian(
    joints: np.ndarray, epsilon: float = 1e-5
) -> np.ndarray:
    """用有限差分计算末端位置对关节角的 3×5 雅可比矩阵。"""
    joints = np.asarray(joints, dtype=float)
    base_position = forward_kinematics(joints)[:3, 3]
    jacobian = np.zeros((3, 5))
    for index in range(5):
        perturbed = joints.copy()
        perturbed[index] += epsilon
        new_position = forward_kinematics(perturbed)[:3, 3]
        jacobian[:, index] = (new_position - base_position) / epsilon
    return jacobian


@dataclass
class IkResult:
    """位置IK求解结果。

    joints按JOINT_NAMES排列且单位为弧度，error为末端位置误差（米）。
    """

    success: bool
    joints: np.ndarray
    iterations: int
    error: float


def solve_position_ik(
    target: np.ndarray,
    initial_joints: np.ndarray | None = None,
    max_iterations: int = 250,
    tolerance: float = 1e-3,
    damping: float = 0.04,
) -> IkResult:
    """使用阻尼最小二乘法求位置 IK。

    只求末端位置，暂不严格约束姿态。这符合 5 自由度 SO-101 的教学边界。
    """
    target = np.asarray(target, dtype=float)
    joints = np.array(
        initial_joints if initial_joints is not None else [0, -0.4, 0.8, -0.4, 0]
    )

    for iteration in range(1, max_iterations + 1):
        current = forward_kinematics(joints)[:3, 3]
        error_vector = target - current
        error_norm = float(np.linalg.norm(error_vector))
        if error_norm < tolerance:
            return IkResult(True, joints, iteration, error_norm)

        jacobian = numerical_position_jacobian(joints)
        # J^T (J J^T + λ²I)^-1 是阻尼伪逆，可缓解奇异位形附近数值爆炸。
        damped_inverse = jacobian.T @ np.linalg.inv(
            jacobian @ jacobian.T + damping**2 * np.eye(3)
        )
        delta = damped_inverse @ error_vector
        # 限制单次变化，避免迭代突然跳到很远的关节姿态。
        delta = np.clip(delta, -0.08, 0.08)
        joints = np.clip(joints + delta, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])

    final_error = float(np.linalg.norm(target - forward_kinematics(joints)[:3, 3]))
    return IkResult(False, joints, max_iterations, final_error)
