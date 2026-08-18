"""实验 4.4-A：把 UMI 位姿转换为供 LeIsaac 重定向的相对末端动作。"""

import argparse
from pathlib import Path

import numpy as np

from so101_leisaac_course.umi import filter_for_workspace, load_umi_episode, relative_actions


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    args = parser.parse_args()
    episode = load_umi_episode(args.episode)
    np.set_printoptions(precision=4, suppress=True)
    print("供 LeIsaac 重定向的动作 [dx,dy,dz,rx,ry,rz,gripper_m]：")
    print(relative_actions(episode, "previous"))
    reasons = filter_for_workspace(episode)
    print("LeIsaac 回放前初筛：", "通过" if not reasons else "需要处理")
    for reason in reasons:
        print("-", reason)


if __name__ == "__main__":
    main()
