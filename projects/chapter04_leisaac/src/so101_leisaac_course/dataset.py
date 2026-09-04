"""读取 LeIsaac 转换得到的官方 LeRobot Dataset v3。"""
# 作者：宇哥的具身笔记


from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import numpy as np

from .schema import format_info, load_dataset_info


def _python_value(value: Any) -> Any:
    """把 torch/NumPy 值转成质量检查器可直接处理的 Python 值。"""
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    if isinstance(value, np.ndarray):
        return value.item() if value.ndim == 0 else value
    if isinstance(value, np.generic):
        return value.item()
    return value


def load_lerobot_frames(root: Path, repo_id: str) -> list[dict[str, Any]]:
    """通过 LeRobot 0.4.2 API 读取正式 Parquet/视频数据集的全部帧。

    图像字段可能很大，质量检查只保留索引、时间、状态和动作，避免复制视频张量。
    """
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as error:
        raise RuntimeError("当前环境缺少课程要求的 lerobot==0.4.2") from error

    dataset = LeRobotDataset(repo_id=repo_id, root=root.expanduser())
    # 直接读取 Parquet 表，而不调用 dataset[index]。后者会解码相机视频，
    # 但当前质量规则只需要低维状态、动作和索引，逐帧解码会浪费显存与时间。
    table = dataset.hf_dataset
    if table is None:
        table = dataset.load_hf_dataset()
    retained = {
        "index",
        "episode_index",
        "frame_index",
        "timestamp",
        "observation.state",
        "action",
    }
    return [
        {key: _python_value(value) for key, value in table[index].items() if key in retained}
        for index in range(len(table))
    ]


def summarize_dataset(root: Path) -> str:
    """读取 meta/info.json，展示 LeIsaac 数据转换后的规模与字段定义。"""
    return format_info(load_dataset_info(root))


def main() -> None:
    """控制台入口：检查 Dataset v3 的元数据说明书。"""
    parser = argparse.ArgumentParser(description="查看 LeIsaac 转换后的 LeRobot Dataset v3")
    parser.add_argument("dataset", type=Path, help="数据集根目录或 meta/info.json")
    args = parser.parse_args()
    print(summarize_dataset(args.dataset))


if __name__ == "__main__":
    main()
