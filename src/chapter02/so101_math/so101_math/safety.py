"""关节 Action 的基础安全检查。"""
# 作者：宇哥的具身笔记


from dataclasses import dataclass
import numpy as np
from .kinematics import JOINT_LIMITS


@dataclass
class SafetyResult:
    """关节安全过滤结果；positions为限速后的五关节弧度值。"""

    accepted: bool
    positions: np.ndarray
    message: str


def filter_joint_action(current, target, max_step=0.08) -> SafetyResult:
    """检查并限制一个关节位置 Action。

    max_step 表示一个控制周期内允许的最大角度变化，单位为弧度。
    """
    current = np.asarray(current, dtype=float)
    target = np.asarray(target, dtype=float)
    if current.shape != (5,) or target.shape != (5,):
        return SafetyResult(False, target, "关节数量错误：应为 5 个机械臂关节")
    if not np.all(np.isfinite(target)):
        return SafetyResult(False, target, "目标包含 NaN 或无穷大")
    if np.any(target < JOINT_LIMITS[:, 0]) or np.any(target > JOINT_LIMITS[:, 1]):
        return SafetyResult(False, target, "目标超出 URDF 关节位置限制")

    limited = current + np.clip(target - current, -max_step, max_step)
    changed = not np.allclose(limited, target)
    message = "目标通过检查" if not changed else "目标有效，但单步变化已按速度限制裁剪"
    return SafetyResult(True, limited, message)
