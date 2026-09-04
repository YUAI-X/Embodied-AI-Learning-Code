"""实验 4.3-A：生成 LeIsaac 官方 HDF5 回放命令。"""
# 作者：宇哥的具身笔记


import argparse
from pathlib import Path

from so101_leisaac_course.commands import (
    COURSE_TASK,
    DATASET_TASK_TYPES,
    STATE_MACHINE_TASK,
    LeIsaacCommandBuilder,
    render_command,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--leisaac-root", type=Path, required=True)
    parser.add_argument(
        "--task",
        choices=(COURSE_TASK, STATE_MACHINE_TASK),
        default=COURSE_TASK,
        help="必须与录制HDF5时的任务一致",
    )
    parser.add_argument(
        "--record-device",
        choices=DATASET_TASK_TYPES,
        default="keyboard",
        help="录制该 HDF5 时使用的设备",
    )
    parser.add_argument("dataset", type=Path)
    args = parser.parse_args()
    command = LeIsaacCommandBuilder(args.leisaac_root).replay(
        args.dataset, task=args.task, task_type=args.record_device
    )
    print(render_command(command))


if __name__ == "__main__":
    main()
