"""实验 4.0：确认完整 LeIsaac GPU 环境和上游源码均已就绪。"""
# 作者：宇哥的具身笔记


import argparse
from pathlib import Path

from so101_leisaac_course.environment import format_results, run_checks


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--leisaac-root", type=Path, required=True)
    args = parser.parse_args()
    results = run_checks(args.leisaac_root)
    print(format_results(results))
    if any(not item.ok for item in results):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
