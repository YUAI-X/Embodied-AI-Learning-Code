"""按episode切分训练/验证/测试集，防止相邻帧泄漏。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class EpisodeSplit:
    """互不重叠的episode编号集合。"""

    train: tuple[int, ...]
    validation: tuple[int, ...]
    test: tuple[int, ...]


def split_episode_ids(
    episode_ids: list[int],
    train_ratio: float = 0.7,
    validation_ratio: float = 0.15,
    seed: int = 42,
) -> EpisodeSplit:
    """随机但可复现地按整段episode切分，绝不按frame随机切分。"""
    unique = np.array(sorted(set(episode_ids)), dtype=int)
    if len(unique) < 3:
        raise ValueError("至少需要3个episode才能切分train/validation/test")
    if not 0 < train_ratio < 1 or not 0 <= validation_ratio < 1:
        raise ValueError("切分比例不合法")
    if train_ratio + validation_ratio >= 1:
        raise ValueError("train_ratio + validation_ratio必须小于1")
    rng = np.random.default_rng(seed)
    rng.shuffle(unique)
    train_end = max(1, int(len(unique) * train_ratio))
    validation_end = max(train_end + 1, int(len(unique) * (train_ratio + validation_ratio)))
    validation_end = min(validation_end, len(unique) - 1)
    return EpisodeSplit(
        train=tuple(sorted(unique[:train_end].tolist())),
        validation=tuple(sorted(unique[train_end:validation_end].tolist())),
        test=tuple(sorted(unique[validation_end:].tolist())),
    )
