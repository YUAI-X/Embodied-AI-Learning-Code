"""实验 4.2-A：生成 LeIsaac 官方任务枚举命令。"""
# 作者：宇哥的具身笔记


import argparse
from pathlib import Path

from so101_leisaac_course.commands import LeIsaacCommandBuilder, render_command


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--leisaac-root", type=Path, required=True)
    args = parser.parse_args()
    command = LeIsaacCommandBuilder(args.leisaac_root).list_envs()
    print(render_command(command))
    print("输出中应包含：LeIsaac-SO101-LiftCube-v0")


if __name__ == "__main__":
    main()
