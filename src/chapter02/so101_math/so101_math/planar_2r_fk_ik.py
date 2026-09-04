"""二维二连杆 FK/IK：先建立直觉，再进入 SO-101 五关节计算。"""
# 作者：宇哥的具身笔记


import argparse
import math
import numpy as np


def fk(theta1, theta2, length1, length2):
    """计算二维二连杆末端位置；角度用弧度，连杆长度与输出位置用米。"""
    x = length1 * math.cos(theta1) + length2 * math.cos(theta1 + theta2)
    y = length1 * math.sin(theta1) + length2 * math.sin(theta1 + theta2)
    return np.array([x, y])


def ik(x, y, length1, length2):
    """求二维目标的两组解析IK；目标不可达时返回空列表。"""
    cosine2 = (x * x + y * y - length1 * length1 - length2 * length2) / (
        2 * length1 * length2
    )
    if abs(cosine2) > 1.0:
        return []
    solutions = []
    for theta2 in (math.acos(cosine2), -math.acos(cosine2)):
        theta1 = math.atan2(y, x) - math.atan2(
            length2 * math.sin(theta2), length1 + length2 * math.cos(theta2)
        )
        solutions.append(np.array([theta1, theta2]))
    return solutions


def main():
    """运行二维FK/IK示例，并用FK逐一验证解析IK解。"""
    parser = argparse.ArgumentParser(description="二维二连杆 FK/IK")
    parser.add_argument("--x", type=float, default=0.18)
    parser.add_argument("--y", type=float, default=0.10)
    args = parser.parse_args()
    length1, length2 = 0.115, 0.135
    solutions = ik(args.x, args.y, length1, length2)
    if not solutions:
        print("目标不可达：距离超出二连杆工作空间。")
        return
    for index, solution in enumerate(solutions, 1):
        verified = fk(*solution, length1, length2)
        print(
            f"解 {index}：角度(rad)={np.round(solution, 4)}，FK 验证={np.round(verified, 4)}"
        )
