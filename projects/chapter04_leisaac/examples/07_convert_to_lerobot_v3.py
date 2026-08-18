"""实验 4.3-C：生成 LeIsaac HDF5 到 LeRobot Dataset v3 的官方转换命令。"""

import argparse
from pathlib import Path

from so101_leisaac_course.commands import LeIsaacCommandBuilder, render_command


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--leisaac-root", type=Path, required=True)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--repo-id", default="local/so101_lift_cube_course")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--record-device",
        choices=("keyboard", "gamepad", "so101leader"),
        default="keyboard",
        help="录制该 HDF5 时使用的设备",
    )
    args = parser.parse_args()
    builder = LeIsaacCommandBuilder(args.leisaac_root)
    command = builder.convert_v3(
        args.dataset,
        args.repo_id,
        fps=args.fps,
        task_type=args.record_device,
    )
    print(render_command(command))


if __name__ == "__main__":
    main()
