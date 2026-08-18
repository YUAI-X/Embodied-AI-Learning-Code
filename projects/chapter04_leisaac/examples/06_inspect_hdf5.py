"""实验 4.3-B：只读查看 LeIsaac HDF5 中的 episode、观测与动作数组。"""

import argparse
from pathlib import Path

from so101_leisaac_course.hdf5_tree import inspect_hdf5


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    args = parser.parse_args()
    print(inspect_hdf5(args.dataset))


if __name__ == "__main__":
    main()
