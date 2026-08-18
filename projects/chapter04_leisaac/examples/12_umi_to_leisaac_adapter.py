"""实验 4.4-B：输出用于接入 LeIsaac 重定向与回放的 UMI 中间帧。"""

import argparse
import json
from pathlib import Path

from so101_leisaac_course.umi import load_umi_episode, to_leisaac_adapter_frames


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    parser.add_argument("--output", type=Path, default=Path("outputs/umi_leisaac_adapter.jsonl"))
    args = parser.parse_args()
    frames = to_leisaac_adapter_frames(load_umi_episode(args.episode))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as stream:
        for frame in frames:
            stream.write(json.dumps(frame, ensure_ascii=False) + "\n")
    print(f"写入 {len(frames)} 帧：{args.output.resolve()}")
    print("下一步必须在 LeIsaac 中完成尺度/坐标标定、IK 重定向和回放验证。")


if __name__ == "__main__":
    main()
