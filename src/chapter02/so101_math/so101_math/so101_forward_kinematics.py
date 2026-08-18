"""计算教学版 SO-101 的正运动学。"""

import argparse
import numpy as np
from .kinematics import JOINT_NAMES, forward_kinematics
from .pose_representation_demo import rotation_to_quaternion


def main():
    """读取五个弧度关节角，输出base_link到tool0的位姿。"""
    parser = argparse.ArgumentParser(description="SO-101 正运动学")
    parser.add_argument(
        "--joints",
        type=float,
        nargs=5,
        default=[0.0, -0.4, 0.8, -0.4, 0.0],
        metavar=("PAN", "SHOULDER", "ELBOW", "WRIST", "ROLL"),
    )
    args = parser.parse_args()
    transform = forward_kinematics(np.array(args.joints))
    print("关节顺序：", JOINT_NAMES)
    print("关节角(rad)：", np.round(args.joints, 4))
    print("\nbase_link 到 tool0：\n", np.round(transform, 5))
    print("\n末端位置(m)：", np.round(transform[:3, 3], 5))
    print("末端四元数 [x,y,z,w]：", np.round(rotation_to_quaternion(transform[:3, :3]), 5))
