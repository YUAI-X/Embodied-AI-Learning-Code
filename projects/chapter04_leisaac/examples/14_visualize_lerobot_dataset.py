"""实验 4.8：使用 Rerun 可视化本地 LeRobot Dataset。
# 作者：宇哥的具身笔记


同时展示 front/wrist 相机、六维关节状态和六维关节动作。数据一次加载
完成后，可以在 Rerun 时间轴中拖动、播放或逐帧检查。
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rerun as rr
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset


def as_numpy(value):
    """把 Torch/NumPy 数据统一转换为 CPU NumPy 数组。"""
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().numpy()
    return np.asarray(value)


def as_image(value) -> np.ndarray:
    """将 LeRobot 的 CHW 浮点图像转换成 Rerun 使用的 HWC uint8。"""
    image = as_numpy(value)
    if image.ndim == 3 and image.shape[0] in (1, 3, 4):
        image = np.transpose(image, (1, 2, 0))
    if np.issubdtype(image.dtype, np.floating):
        if image.max() <= 1.01:
            image = image * 255.0
        image = np.clip(image, 0, 255).astype(np.uint8)
    return image


def main() -> None:
    parser = argparse.ArgumentParser(description="用 Rerun 查看本地 LeRobot 数据")
    parser.add_argument("dataset", type=Path, help="LeRobot 数据集根目录")
    parser.add_argument("--repo-id", default="local/pick_orange", help="用于初始化 LeRobotDataset 的标识")
    parser.add_argument("--episode", type=int, default=0, help="要加载的 episode 编号")
    parser.add_argument("--save", type=Path, help="保存为 .rrd；省略时打开 Rerun 窗口")
    args = parser.parse_args()

    dataset = LeRobotDataset(repo_id=args.repo_id, root=args.dataset.expanduser().resolve())
    if not 0 <= args.episode < dataset.num_episodes:
        raise ValueError(f"episode 应在 0 到 {dataset.num_episodes - 1} 之间")

    rr.init("LeRobot Pick Orange", spawn=args.save is None)
    if args.save is not None:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        rr.save(str(args.save))

    names = dataset.meta.features["observation.state"]["names"]
    episode_indices = dataset.hf_dataset["episode_index"]
    selected = [index for index, episode in enumerate(episode_indices) if int(episode) == args.episode]
    print(f"加载 episode={args.episode}: {len(selected)} frames, {dataset.meta.fps} FPS")

    for local_frame, global_index in enumerate(selected):
        sample = dataset[global_index]
        rr.set_time("frame", sequence=local_frame)
        rr.set_time("time", duration=local_frame / dataset.meta.fps)

        for camera_key in dataset.meta.camera_keys:
            rr.log(camera_key.replace("observation.images.", "camera/"), rr.Image(as_image(sample[camera_key])))

        state = as_numpy(sample["observation.state"]).reshape(-1)
        action = as_numpy(sample["action"]).reshape(-1)
        for joint_name, state_value, action_value in zip(names, state, action, strict=True):
            joint = joint_name.removesuffix(".pos")
            rr.log(f"joints/{joint}/state", rr.Scalars(float(state_value)))
            rr.log(f"joints/{joint}/action", rr.Scalars(float(action_value)))

    print("数据已加载到 Rerun，可使用时间轴播放或逐帧查看。")


if __name__ == "__main__":
    main()
