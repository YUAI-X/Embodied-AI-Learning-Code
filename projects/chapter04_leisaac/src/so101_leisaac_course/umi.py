"""解析简化UMI轨迹并生成相对末端动作。"""
# 作者：宇哥的具身笔记


from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

import numpy as np

from .transforms import pose7_to_matrix, relative_transform, transform_to_action6


@dataclass(frozen=True)
class WorkspaceBounds:
    """目标机器人可用工作空间的教学近似，真实部署应替换为IK和碰撞检查。"""

    minimum_xyz: tuple[float, float, float] = (0.10, -0.30, 0.02)
    maximum_xyz: tuple[float, float, float] = (0.42, 0.30, 0.42)
    max_linear_speed_m_s: float = 0.50
    max_angular_speed_rad_s: float = 3.00


@dataclass(frozen=True)
class UmiEpisode:
    """课程约定的单手UMI子集。

    poses使用[x,y,z,qx,qy,qz,qw]；gripper_width_m使用米；所有数组长度相同。
    """

    timestamps: np.ndarray
    poses: np.ndarray
    gripper_width_m: np.ndarray
    task: str

    def __post_init__(self) -> None:
        count = len(self.timestamps)
        if count < 2:
            raise ValueError("UMI episode至少需要2帧")
        if self.timestamps.shape != (count,) or self.poses.shape != (count, 7):
            raise ValueError("timestamps应为[N]，poses应为[N,7]")
        if self.gripper_width_m.shape != (count,):
            raise ValueError("gripper_width_m应为[N]")
        if not all(
            np.all(np.isfinite(value))
            for value in (self.timestamps, self.poses, self.gripper_width_m)
        ):
            raise ValueError("UMI数据不能包含NaN或Inf")
        if np.any(np.diff(self.timestamps) <= 0):
            raise ValueError("timestamps必须严格递增")
        if np.any(self.gripper_width_m < 0):
            raise ValueError("夹爪宽度不能为负")


def load_umi_episode(path: Path) -> UmiEpisode:
    """读取课程JSON；真实UMI MP4/VIO输出应先转成这个清晰的中间结构。"""
    with path.expanduser().open("r", encoding="utf-8") as stream:
        value: dict[str, Any] = json.load(stream)
    return UmiEpisode(
        timestamps=np.asarray(value["timestamps"], dtype=float),
        poses=np.asarray(value["poses"], dtype=float),
        gripper_width_m=np.asarray(value["gripper_width_m"], dtype=float),
        task=str(value.get("task", "unknown task")),
    )


def relative_actions(episode: UmiEpisode, reference: str = "previous") -> np.ndarray:
    """生成[N,7]动作：[相对平移3，相对旋转轴角3，目标夹爪宽度1]。

    previous表示逐帧增量，episode_start对应教材公式inv(T_ee(t0))@T_ee(t)。
    """
    if reference not in {"previous", "episode_start"}:
        raise ValueError("reference只能是previous或episode_start")
    transforms = [pose7_to_matrix(pose) for pose in episode.poses]
    actions = []
    for index, target in enumerate(transforms):
        reference_index = max(0, index - 1) if reference == "previous" else 0
        relative = relative_transform(transforms[reference_index], target)
        actions.append(np.r_[transform_to_action6(relative), episode.gripper_width_m[index]])
    return np.asarray(actions)


def filter_for_workspace(
    episode: UmiEpisode, bounds: WorkspaceBounds = WorkspaceBounds()
) -> list[str]:
    """执行部署前的初筛，返回不可接受原因；通过不等于SO101一定可达。"""
    reasons: list[str] = []
    minimum = np.asarray(bounds.minimum_xyz)
    maximum = np.asarray(bounds.maximum_xyz)
    for index, xyz in enumerate(episode.poses[:, :3]):
        if np.any(xyz < minimum) or np.any(xyz > maximum):
            reasons.append(f"frame {index}: 位置{xyz.round(3).tolist()}超出教学工作空间")
    actions = relative_actions(episode, reference="previous")
    dt = np.diff(episode.timestamps)
    linear_speed = np.linalg.norm(actions[1:, :3], axis=1) / dt
    angular_speed = np.linalg.norm(actions[1:, 3:6], axis=1) / dt
    for index, speed in enumerate(linear_speed, start=1):
        if speed > bounds.max_linear_speed_m_s:
            reasons.append(f"frame {index}: 线速度{speed:.3f} m/s超过阈值")
    for index, speed in enumerate(angular_speed, start=1):
        if speed > bounds.max_angular_speed_rad_s:
            reasons.append(f"frame {index}: 角速度{speed:.3f} rad/s超过阈值")
    return reasons


def to_leisaac_adapter_frames(episode: UmiEpisode) -> list[dict[str, Any]]:
    """把 UMI 轨迹映射为等待 LeIsaac 回放验证的中间帧。

    输出不是训练数据。它只用于核对时间、坐标和动作语义；通过 LeIsaac 重定向与回放后，
    仍应使用官方录制器和转换器生成 LeRobot Dataset v3。
    """
    actions = relative_actions(episode, reference="previous")
    frames = []
    for index, (timestamp, pose, gripper, action) in enumerate(
        zip(episode.timestamps, episode.poses, episode.gripper_width_m, actions)
    ):
        frames.append(
            {
                "episode_index": 0,
                "frame_index": index,
                "timestamp": float(timestamp - episode.timestamps[0]),
                "observation.state": np.r_[pose, gripper].round(7).tolist(),
                "action": action.round(7).tolist(),
                "task": episode.task,
            }
        )
    return frames
