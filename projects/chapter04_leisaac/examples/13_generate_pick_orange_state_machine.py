"""实验 4.7：生成 SO101 PickOrange 状态机合成数据命令。
# 作者：宇哥的具身笔记


本入口不复制 Isaac Lab 控制循环，而是对 LeIsaac 官方状态机脚本做参数校验和
可复现配置。这样上游修复物理控制或录制器时，课程项目无需维护一份分叉实现。
"""

import argparse
from pathlib import Path

from so101_leisaac_course.commands import LeIsaacCommandBuilder, render_command


def main() -> None:
    parser = argparse.ArgumentParser(description="生成 PickOrange 状态机批量采集命令")
    parser.add_argument("--leisaac-root", type=Path, required=True, help="LeIsaac源码仓库根目录")
    parser.add_argument(
        "--dataset",
        type=Path,
        default=Path("datasets/pick_orange_state_machine.hdf5"),
        help="HDF5输出路径；续录时必须指向已有文件",
    )
    parser.add_argument("--num-demos", type=int, default=50, help="目标示范数，必须大于0")
    parser.add_argument("--num-envs", type=int, default=1, help="并行仿真环境数")
    parser.add_argument("--step-hz", type=int, default=60, help="仿真控制循环频率")
    parser.add_argument("--seed", type=int, default=42, help="环境随机种子")
    parser.add_argument("--resume", action="store_true", help="从已有HDF5继续录制")
    parser.add_argument("--headless", action="store_true", help="无窗口运行，仍启用相机")
    parser.add_argument("--quality", action="store_true", help="启用上游高质量渲染模式")
    parser.add_argument(
        "--lerobot-repo-id",
        default=None,
        help="直接录制LeRobot Dataset，例如local/pick_orange；省略则录制HDF5",
    )
    parser.add_argument("--lerobot-fps", type=int, default=30)
    parser.add_argument("--camera-width", type=int, default=640, help="相机宽度；默认与参考数据集一致")
    parser.add_argument("--camera-height", type=int, default=480, help="相机高度；默认与参考数据集一致")
    parser.add_argument("--orange-index", type=int, choices=(1, 2, 3), default=1, help="单集抓取的橘子编号")
    args = parser.parse_args()

    command = LeIsaacCommandBuilder(args.leisaac_root).state_machine_generate(
        dataset_file=args.dataset,
        num_demos=args.num_demos,
        num_envs=args.num_envs,
        step_hz=args.step_hz,
        seed=args.seed,
        resume=args.resume,
        headless=args.headless,
        quality=args.quality,
        lerobot_repo_id=args.lerobot_repo_id,
        lerobot_fps=args.lerobot_fps,
        camera_width=args.camera_width,
        camera_height=args.camera_height,
        orange_index=args.orange_index,
    )
    print(render_command(command))
    if args.lerobot_repo_id:
        print("复制以上命令执行；完成后检查meta/info.json、Parquet和两路MP4是否齐全。")
    else:
        print("复制以上命令执行；完成后先用样例06检查HDF5，再使用上游replay.py抽样回放。")


if __name__ == "__main__":
    main()
