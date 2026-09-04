"""实验 4.3-D：读取 LeIsaac 转换后 Dataset v3 的 info.json。"""
# 作者：宇哥的具身笔记


import argparse
from pathlib import Path

from so101_leisaac_course.dataset import summarize_dataset


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    args = parser.parse_args()
    print(summarize_dataset(args.dataset))


if __name__ == "__main__":
    main()
