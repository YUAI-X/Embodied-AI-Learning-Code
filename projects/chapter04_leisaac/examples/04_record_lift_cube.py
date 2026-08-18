"""实验 4.2-B：生成 SO101 LiftCube 遥操作与 HDF5 录制命令。"""

import argparse
from pathlib import Path

from so101_leisaac_course.commands import LeIsaacCommandBuilder, render_command


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--leisaac-root", type=Path, required=True)
    parser.add_argument(
        "--device",
        choices=("keyboard", "gamepad", "so101leader"),
        default="keyboard",
    )
    parser.add_argument("--dataset", type=Path, default=Path("datasets/lift_cube.hdf5"))
    parser.add_argument("--port", default="/dev/ttyACM0")
    args = parser.parse_args()
    builder = LeIsaacCommandBuilder(args.leisaac_root)
    command = builder.teleop(device=args.device, dataset_file=args.dataset, port=args.port)
    print(render_command(command))
    print("窗口内按 b 开始；按 r 丢弃失败轨迹；按 n 保存成功轨迹并重置。")


if __name__ == "__main__":
    main()
