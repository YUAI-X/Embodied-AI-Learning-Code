"""安全构造LeIsaac官方脚本命令，不在库函数中启动仿真。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import shlex


COURSE_TASK = "LeIsaac-SO101-LiftCube-v0"
TELEOP_DEVICES = ("keyboard", "gamepad", "so101leader")


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
        if task != COURSE_TASK:
            raise ValueError(f"本章只使用任务：{COURSE_TASK}")
        if task_type not in TELEOP_DEVICES:
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
            *([f"--task_type={task_type}"] if task_type in {"keyboard", "gamepad"} else []),
        ]

    def replay(
        self,
        dataset_file: Path,
        task: str = COURSE_TASK,
        task_type: str = "keyboard",
    ) -> list[str]:
        """构造录制后回放命令，先回放再转换可发现大量采集问题。"""
        if task != COURSE_TASK:
            raise ValueError(f"本章只使用任务：{COURSE_TASK}")
        if task_type not in TELEOP_DEVICES:
            raise ValueError(f"未知的录制设备：{task_type}")
        command = [
            self.python,
            self._script("scripts/environments/teleoperation/replay.py"),
            f"--task={task}",
            f"--dataset_file={dataset_file}",
            "--device=cuda",
            "--enable_cameras",
        ]
        if task_type in {"keyboard", "gamepad"}:
            command.append(f"--task_type={task_type}")
        return command


def render_command(arguments: list[str]) -> str:
    """使用shell安全转义生成可复制命令。"""
    return shlex.join(arguments)
