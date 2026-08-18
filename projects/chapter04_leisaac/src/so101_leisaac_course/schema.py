"""读取和校验LeRobot Dataset的核心元信息。"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any


class SchemaError(ValueError):
    """数据集说明书缺少必要字段或字段类型不正确。"""


@dataclass(frozen=True)
class FeatureSpec:
    """LeRobot feature的教学子集：类型、形状和各维名称。"""

    dtype: str
    shape: tuple[int, ...]
    names: tuple[str, ...] = ()

    @classmethod
    def from_dict(cls, name: str, value: dict[str, Any]) -> "FeatureSpec":
        """从info.json字段创建对象并给出易懂的错误信息。"""
        if "dtype" not in value or "shape" not in value:
            raise SchemaError(f"feature {name!r} 必须包含dtype和shape")
        shape = tuple(value["shape"])
        if not all(isinstance(size, int) and size > 0 for size in shape):
            raise SchemaError(f"feature {name!r} 的shape必须是正整数列表")
        names_value = value.get("names") or []
        names = tuple(names_value) if isinstance(names_value, list) else ()
        return cls(str(value["dtype"]), shape, names)


@dataclass(frozen=True)
class DatasetInfo:
    """课程关心的info.json字段；额外字段由原始字典保留。"""

    codebase_version: str
    robot_type: str
    fps: float
    total_episodes: int
    total_frames: int
    features: dict[str, FeatureSpec]
    raw: dict[str, Any]

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "DatasetInfo":
        """校验数据规模、采样频率与features schema。"""
        required = (
            "codebase_version",
            "robot_type",
            "fps",
            "total_episodes",
            "total_frames",
            "features",
        )
        missing = [key for key in required if key not in value]
        if missing:
            raise SchemaError(f"info.json缺少字段：{', '.join(missing)}")
        fps = float(value["fps"])
        total_episodes = int(value["total_episodes"])
        total_frames = int(value["total_frames"])
        if fps <= 0 or total_episodes < 0 or total_frames < 0:
            raise SchemaError("fps必须为正数，episode/frame数量不能为负")
        feature_values = value["features"]
        if not isinstance(feature_values, dict) or not feature_values:
            raise SchemaError("features必须是非空字典")
        features = {
            name: FeatureSpec.from_dict(name, spec)
            for name, spec in feature_values.items()
        }
        return cls(
            codebase_version=str(value["codebase_version"]),
            robot_type=str(value["robot_type"]),
            fps=fps,
            total_episodes=total_episodes,
            total_frames=total_frames,
            features=features,
            raw=value,
        )


def find_info_path(dataset_root: Path) -> Path:
    """兼容直接传info.json或传数据集根目录。"""
    candidate = dataset_root.expanduser()
    if candidate.is_file():
        return candidate
    return candidate / "meta" / "info.json"


def load_dataset_info(dataset_root: Path) -> DatasetInfo:
    """以UTF-8读取info.json并返回类型化结果。"""
    path = find_info_path(dataset_root)
    if not path.is_file():
        raise FileNotFoundError(f"找不到数据集说明书：{path}")
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise SchemaError("info.json根节点必须是对象")
    return DatasetInfo.from_dict(value)


def format_info(info: DatasetInfo) -> str:
    """生成面向初学者的数据集摘要。"""
    lines = [
        f"LeRobot版本: {info.codebase_version}",
        f"机器人类型: {info.robot_type}",
        f"采样频率: {info.fps:g} Hz",
        f"数据规模: {info.total_episodes} episodes / {info.total_frames} frames",
        "字段:",
    ]
    for name, feature in info.features.items():
        names = f" names={list(feature.names)}" if feature.names else ""
        lines.append(f"  - {name}: dtype={feature.dtype} shape={list(feature.shape)}{names}")
    return "\n".join(lines)
