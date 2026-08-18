"""检查episode完整性、时间同步、字段维度和动作连续性。"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .dataset import load_lerobot_frames
from .schema import DatasetInfo, load_dataset_info


@dataclass(frozen=True)
class QualityIssue:
    """一条可定位到episode/frame的数据问题。"""

    level: str
    message: str
    episode_index: int | None = None
    frame_index: int | None = None

    def __str__(self) -> str:
        location = ""
        if self.episode_index is not None:
            location = f" episode={self.episode_index} frame={self.frame_index}"
        return f"[{self.level}]{location} {self.message}"


@dataclass(frozen=True)
class QualityReport:
    """质量检查结果；error表示不能用于训练，warning表示需要人工复核。"""

    frames: int
    episodes: int
    issues: tuple[QualityIssue, ...]

    @property
    def ok(self) -> bool:
        return not any(issue.level == "error" for issue in self.issues)


def _vector(value: Any, expected_size: int) -> np.ndarray | None:
    """把列表转为有限浮点向量；格式错误时返回None交给调用方报告。"""
    try:
        vector = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None
    if vector.shape != (expected_size,) or not np.all(np.isfinite(vector)):
        return None
    return vector


def validate_frames(
    frames: list[dict[str, Any]],
    info: DatasetInfo,
    timestamp_tolerance_ratio: float = 0.20,
    max_action_jump: float = 0.35,
) -> QualityReport:
    """按episode检查索引、时间间隔、向量维度和相邻动作跳变。"""
    issues: list[QualityIssue] = []
    state_size = info.features["observation.state"].shape[0]
    action_size = info.features["action"].shape[0]
    expected_dt = 1.0 / info.fps
    episode_ids: set[int] = set()
    previous_by_episode: dict[int, dict[str, Any]] = {}

    required = {
        "episode_index",
        "frame_index",
        "timestamp",
        "observation.state",
        "action",
    }
    for row_number, frame in enumerate(frames):
        missing = sorted(required - frame.keys())
        if missing:
            issues.append(QualityIssue("error", f"缺少字段：{missing}"))
            continue
        episode = int(frame["episode_index"])
        frame_index = int(frame["frame_index"])
        episode_ids.add(episode)
        state = _vector(frame["observation.state"], state_size)
        action = _vector(frame["action"], action_size)
        if state is None:
            issues.append(QualityIssue("error", "state维度错误或包含NaN", episode, frame_index))
        if action is None:
            issues.append(QualityIssue("error", "action维度错误或包含NaN", episode, frame_index))

        previous = previous_by_episode.get(episode)
        if previous is None:
            if frame_index != 0:
                issues.append(QualityIssue("error", "episode首帧不是0", episode, frame_index))
        else:
            previous_index = int(previous["frame_index"])
            if frame_index != previous_index + 1:
                issues.append(QualityIssue("error", "frame_index不连续", episode, frame_index))
            dt = float(frame["timestamp"]) - float(previous["timestamp"])
            if dt <= 0:
                issues.append(QualityIssue("error", "timestamp没有严格递增", episode, frame_index))
            elif abs(dt - expected_dt) > expected_dt * timestamp_tolerance_ratio:
                issues.append(
                    QualityIssue(
                        "warning",
                        f"帧间隔{dt:.4f}s偏离期望{expected_dt:.4f}s",
                        episode,
                        frame_index,
                    )
                )
            previous_action = _vector(previous["action"], action_size)
            if action is not None and previous_action is not None:
                jump = float(np.max(np.abs(action - previous_action)))
                if jump > max_action_jump:
                    issues.append(
                        QualityIssue(
                            "warning",
                            f"最大动作跳变{jump:.3f}超过阈值{max_action_jump:.3f}",
                            episode,
                            frame_index,
                        )
                    )
        previous_by_episode[episode] = frame

        if int(frame.get("index", row_number)) != row_number:
            issues.append(QualityIssue("warning", "全局index与文件行号不一致", episode, frame_index))

    if len(frames) != info.total_frames:
        issues.append(
            QualityIssue("error", f"info记录{info.total_frames}帧，实际读取{len(frames)}帧")
        )
    if len(episode_ids) != info.total_episodes:
        issues.append(
            QualityIssue(
                "error", f"info记录{info.total_episodes}个episode，实际读取{len(episode_ids)}个"
            )
        )
    return QualityReport(len(frames), len(episode_ids), tuple(issues))


def format_report(report: QualityReport) -> str:
    """输出适合课程验收的摘要。"""
    status = "通过" if report.ok else "失败"
    lines = [f"质量检查：{status}；{report.episodes} episodes / {report.frames} frames"]
    if report.issues:
        lines.extend(str(issue) for issue in report.issues)
    else:
        lines.append("未发现索引、时间、维度或动作连续性问题。")
    return "\n".join(lines)


def main() -> None:
    """命令行入口：校验 LeIsaac 转换后的正式 Dataset v3。"""
    parser = argparse.ArgumentParser(description="检查 SO101 LeIsaac 数据质量")
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--repo-id", required=True, help="转换时使用的 namespace/name")
    args = parser.parse_args()
    info = load_dataset_info(args.dataset)
    report = validate_frames(load_lerobot_frames(args.dataset, args.repo_id), info)
    print(format_report(report))
    if not report.ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
