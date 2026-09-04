"""SO-101 位置目标数值 IK。"""
# 作者：宇哥的具身笔记


import argparse
import numpy as np
from .kinematics import JOINT_NAMES, forward_kinematics, solve_position_ik


def main():
    """求解米制末端位置目标，并使用FK验证五关节位置IK结果。"""
    parser = argparse.ArgumentParser(description="SO-101 阻尼最小二乘位置 IK")
    parser.add_argument("--x", type=float, default=0.43)
    parser.add_argument("--y", type=float, default=0.08)
    parser.add_argument("--z", type=float, default=0.11)
    args = parser.parse_args()
    target = np.array([args.x, args.y, args.z])
    result = solve_position_ik(target)
    reached = forward_kinematics(result.joints)[:3, 3]
    print("关节顺序：", JOINT_NAMES)
    print("目标位置：", np.round(target, 4))
    print("是否收敛：", result.success)
    print("迭代次数：", result.iterations)
    print("最终误差(m)：", round(result.error, 6))
    print("关节解(rad)：", np.round(result.joints, 4))
    print("FK 验证位置：", np.round(reached, 4))
    if not result.success:
        print("提示：目标可能不可达、处于关节限制外，或初值不适合。")
