"""实验 4.3-F：按完整 LeIsaac episode 切分训练、验证和测试集合。"""

import argparse

from so101_leisaac_course.splits import split_episode_ids


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", type=int, required=True, help="Dataset v3 中的 episode 总数")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    split = split_episode_ids(list(range(args.episodes)), seed=args.seed)
    print("train     :", split.train)
    print("validation:", split.validation)
    print("test      :", split.test)


if __name__ == "__main__":
    main()
