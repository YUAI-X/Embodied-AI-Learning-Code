"""实验 4.3-E：检查 LeIsaac Dataset v3 的时间、索引、维度和动作连续性。"""
# 作者：宇哥的具身笔记


import argparse
from pathlib import Path

from so101_leisaac_course.dataset import load_lerobot_frames
from so101_leisaac_course.quality import format_report, validate_frames
from so101_leisaac_course.schema import load_dataset_info


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--repo-id", required=True)
    args = parser.parse_args()
    info = load_dataset_info(args.dataset)
    frames = load_lerobot_frames(args.dataset, args.repo_id)
    report = validate_frames(frames, info)
    print(format_report(report))
    if not report.ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
