"""安全构造LeIsaac官方脚本命令，不在库函数中启动仿真。"""
# 作者：宇哥的具身笔记


from __future__ import annotations

import shlex
from dataclasses import dataclass
from pathlib import Path


COURSE_TASK = "LeIsaac-SO101-LiftCube-v0"
STATE_MACHINE_TASK = "LeIsaac-SO101-PickOrange-v0"
TELEOP_DEVICES = ("keyboard", "gamepad", "so101leader")
DATASET_TASK_TYPES = (*TELEOP_DEVICES, "so101_state_machine")
SUPPORTED_DATASET_TASKS = (COURSE_TASK, STATE_MACHINE_TASK)


@dataclass(frozen=True)
class LeIsaacCommandBuilder:
    """根据源码仓库位置构造可复制执行的参数列表。"""

    root: Path
    python: str = "python"

    def _script(self, relative: str) -> str:
        path = self.root.expanduser().resolve() / relative
        if not path.is_file():
            raise FileNotFoundError(f"找不到LeIsaac官方脚本：{path}")
        return str(path)

    def list_envs(self) -> list[str]:
        """构造官方任务枚举命令，用于确认 SO101 LiftCube 已注册。"""
        return [
            self.python,
            self._script("scripts/environments/list_envs.py"),
        ]

    def teleop(
        self,
        task: str = COURSE_TASK,
        device: str = "keyboard",
        dataset_file: Path = Path("datasets/lift_cube.hdf5"),
        port: str = "/dev/ttyACM0",
    ) -> list[str]:
        """构造HDF5遥操作采集命令；初学者默认使用键盘。"""
        if task != COURSE_TASK:
            raise ValueError(f"本章只使用任务：{COURSE_TASK}")
        if device not in TELEOP_DEVICES:
            raise ValueError(f"不支持的输入设备：{device}")
        command = [
            self.python,
            self._script("scripts/environments/teleoperation/teleop_se3_agent.py"),
            f"--task={task}",
            f"--teleop_device={device}",
            "--num_envs=1",
            "--device=cuda",
            "--enable_cameras",
            "--record",
            f"--dataset_file={dataset_file}",
        ]
        if device == "so101leader":
            command.append(f"--port={port}")
        return command

    def convert_v3(
        self,
        dataset_file: Path,
        repo_id: str = "local/so101_lift_cube_course",
        task: str = COURSE_TASK,
        fps: int = 30,
        task_type: str = "keyboard",
    ) -> list[str]:
        """构造HDF5→LeRobot Dataset v3命令，默认不上传Hub。"""
        if "/" not in repo_id:
            raise ValueError("repo_id应采用namespace/name形式")
        if task not in SUPPORTED_DATASET_TASKS:
            raise ValueError(f"不支持的数据集任务：{task}")
        if task_type not in DATASET_TASK_TYPES:
            raise ValueError(f"未知的录制设备：{task_type}")
        return [
            self.python,
            self._script("scripts/convert/isaaclab2lerobotv3.py"),
            f"--task_name={task}",
            f"--repo_id={repo_id}",
            f"--fps={fps}",
            f"--hdf5_root={dataset_file.parent}",
            f"--hdf5_files={dataset_file.name}",
            "--device=cuda",
            "--enable_cameras",
            *([f"--task_type={task_type}"] if task_type != "so101leader" else []),
        ]

    def replay(
        self,
        dataset_file: Path,
        task: str = COURSE_TASK,
        task_type: str = "keyboard",
    ) -> list[str]:
        """构造录制后回放命令，先回放再转换可发现大量采集问题。"""
        if task not in SUPPORTED_DATASET_TASKS:
            raise ValueError(f"不支持的数据集任务：{task}")
        if task_type not in DATASET_TASK_TYPES:
            raise ValueError(f"未知的录制设备：{task_type}")
        command = [
            self.python,
            self._script("scripts/environments/teleoperation/replay.py"),
            f"--task={task}",
            f"--dataset_file={dataset_file}",
            "--device=cuda",
            "--enable_cameras",
        ]
        if task_type != "so101leader":
            command.append(f"--task_type={task_type}")
        return command

    def state_machine_generate(
        self,
        dataset_file: Path = Path("datasets/pick_orange_state_machine.hdf5"),
        task: str = STATE_MACHINE_TASK,
        num_demos: int = 50,
        num_envs: int = 1,
        step_hz: int = 60,
        seed: int | None = 42,
        resume: bool = False,
        headless: bool = False,
        quality: bool = False,
        lerobot_repo_id: str | None = None,
        lerobot_fps: int = 30,
        camera_width: int = 640,
        camera_height: int = 480,
        orange_index: int | None = 1,
    ) -> list[str]:
        """构造上游状态机合成命令。

        当前 LeIsaac 的 ``TASK_REGISTRY`` 只注册 PickOrange 状态机，因此这里使用
        严格白名单，避免传入 LiftCube 后启动 Isaac Sim 很久才报错。有限的
        ``num_demos`` 也可防止课程命令意外无限运行；上游脚本原生支持 0 表示无限。
        """
        if task != STATE_MACHINE_TASK:
            raise ValueError(f"当前上游状态机只支持任务：{STATE_MACHINE_TASK}")
        if num_demos < 1:
            raise ValueError("num_demos必须为正整数；课程入口不允许无限录制")
        if num_envs < 1:
            raise ValueError("num_envs必须为正整数")
        if step_hz < 1:
            raise ValueError("step_hz必须为正整数")
        if dataset_file.suffix.lower() not in {".hdf5", ".h5"}:
            raise ValueError("状态机默认录制HDF5，输出文件应使用.hdf5或.h5后缀")
        if lerobot_repo_id is not None and "/" not in lerobot_repo_id:
            raise ValueError("LeRobot repo_id应采用namespace/name形式")
        if lerobot_fps < 1 or camera_width < 1 or camera_height < 1:
            raise ValueError("LeRobot fps和相机尺寸必须为正整数")
        if orange_index not in {None, 1, 2, 3}:
            raise ValueError("orange_index只能是1、2、3或None")

        command = [
            self.python,
            self._script("scripts/datagen/state_machine/generate.py"),
            f"--task={task}",
            f"--num_envs={num_envs}",
            "--device=cuda",
            "--enable_cameras",
            "--record",
            f"--dataset_file={dataset_file}",
            f"--num_demos={num_demos}",
            f"--step_hz={step_hz}",
        ]
        if seed is not None:
            command.append(f"--seed={seed}")
        if resume:
            command.append("--resume")
        if headless:
            command.append("--headless")
        if quality:
            command.append("--quality")
        if orange_index is not None:
            command.append(f"--orange_index={orange_index}")
        if lerobot_repo_id is not None:
            command.extend(
                [
                    "--use_lerobot_recorder",
                    f"--lerobot_dataset_repo_id={lerobot_repo_id}",
                    f"--lerobot_dataset_fps={lerobot_fps}",
                    f"--camera_width={camera_width}",
                    f"--camera_height={camera_height}",
                ]
            )
        return command


def render_command(arguments: list[str]) -> str:
    """使用shell安全转义生成可复制命令。"""
    return shlex.join(arguments)
