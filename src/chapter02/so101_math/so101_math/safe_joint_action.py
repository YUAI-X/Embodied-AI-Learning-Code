"""演示 Action 在进入控制器前为什么要经过安全过滤。"""

import numpy as np
from .kinematics import JOINT_NAMES
from .safety import filter_joint_action


def main():
    """用正常、越界和NaN目标演示关节命令进入控制器前的过滤。"""
    current = np.array([0.0, -0.4, 0.8, -0.4, 0.0])
    examples = {
        "正常但变化过快": np.array([0.3, -0.7, 1.0, -0.2, 0.4]),
        "关节越界": np.array([4.0, 0.0, 0.0, 0.0, 0.0]),
        "包含 NaN": np.array([0.0, np.nan, 0.0, 0.0, 0.0]),
    }
    print("关节顺序：", JOINT_NAMES)
    print("当前位置：", np.round(current, 3))
    for name, target in examples.items():
        result = filter_joint_action(current, target)
        print(f"\n{name}：")
        print("  是否接受：", result.accepted)
        print("  说明：", result.message)
        print("  输出：", np.round(result.positions, 3))
