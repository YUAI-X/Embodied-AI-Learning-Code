import json

import pytest

from so101_leisaac_course.dataset import summarize_dataset
from so101_leisaac_course.quality import validate_frames
from so101_leisaac_course.schema import DatasetInfo, SchemaError, load_dataset_info


def sample_info(total_frames=4):
    """构造与 LeIsaac 转换结果同语义的最小 Dataset v3 元数据。"""
    return DatasetInfo.from_dict(
        {
            "codebase_version": "v3.0",
            "robot_type": "so101_follower",
            "fps": 30,
            "total_episodes": 1,
            "total_frames": total_frames,
            "features": {
                "observation.state": {"dtype": "float32", "shape": [6]},
                "action": {"dtype": "float32", "shape": [6]},
            },
        }
    )


def sample_frames():
    return [
        {
            "index": index,
            "episode_index": 0,
            "frame_index": index,
            "timestamp": index / 30,
            "observation.state": [0.01 * index] * 6,
            "action": [0.01 * (index + 1)] * 6,
        }
        for index in range(4)
    ]


def test_validate_official_lerobot_semantics():
    report = validate_frames(sample_frames(), sample_info())
    assert report.ok
    assert report.frames == 4


def test_summarize_dataset_v3_info(tmp_path):
    info = sample_info().raw
    info_path = tmp_path / "meta" / "info.json"
    info_path.parent.mkdir()
    info_path.write_text(json.dumps(info), encoding="utf-8")
    summary = summarize_dataset(tmp_path)
    assert "so101_follower" in summary
    assert "1 episodes / 4 frames" in summary


def test_schema_reports_missing_fields(tmp_path):
    info_path = tmp_path / "meta" / "info.json"
    info_path.parent.mkdir()
    info_path.write_text(json.dumps({"fps": 30}), encoding="utf-8")
    with pytest.raises(SchemaError, match="缺少字段"):
        load_dataset_info(tmp_path)


def test_quality_detects_non_increasing_timestamp():
    frames = sample_frames()
    frames[1]["timestamp"] = frames[0]["timestamp"]
    report = validate_frames(frames, sample_info())
    assert not report.ok
    assert any("timestamp" in issue.message for issue in report.issues)
