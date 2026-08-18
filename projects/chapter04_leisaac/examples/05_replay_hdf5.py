"""实验 4.3-A：生成 LeIsaac 官方 HDF5 回放命令。"""

import argparse
from pathlib import Path

from so101_leisaac_course.commands import LeIsaacCommandBuilder, render_command


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--leisaac-root", type=Path, required=True)
    parser.add_argument(
        "--record-device",
        choices=("keyboard", "gamepad", "so101leader"),
        default="keyboard",
        help="录制该 HDF5 时使用的设备",
    )
    parser.add_argument("dataset", type=Path)
    args = parser.parse_args()
    command = LeIsaacCommandBuilder(args.leisaac_root).replay(
        args.dataset, task_type=args.record_device
    )
    print(render_command(command))


if __name__ == "__main__":
    main()
