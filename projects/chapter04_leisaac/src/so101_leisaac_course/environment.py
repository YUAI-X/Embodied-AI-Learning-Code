"""检查 LeIsaac 课程所需的完整 NVIDIA GPU 软件栈。"""
# 作者：宇哥的具身笔记


from __future__ import annotations

import argparse
from dataclasses import dataclass
from importlib import metadata
from pathlib import Path
import platform
import shutil
import subprocess
import sys


@dataclass(frozen=True)
class CheckResult:
    """单项必需检查；任何一项失败都表示本章环境尚未就绪。"""

    name: str
    ok: bool
    detail: str


LEISAAC_PROFILE = {
    "python": "3.11",
    "isaacsim": "5.1.0",
    "isaaclab": "2.3.0",
    "torch": "2.7.0",
    "cuda": "12.8",
    "lerobot": "0.4.2",
    "leisaac": "0.4.0",
    "numpy": "1.26.0",
}


def package_version(distribution: str) -> str | None:
    """返回发行包版本；未安装时返回 None，供检查报告统一展示。"""
    try:
        return metadata.version(distribution)
    except metadata.PackageNotFoundError:
        return None


def check_nvidia_smi() -> CheckResult:
    """用 nvidia-smi 确认 NVIDIA 驱动和 GPU 对当前进程可见。"""
    executable = shutil.which("nvidia-smi")
    if executable is None:
        return CheckResult("NVIDIA GPU", False, "未找到 nvidia-smi")
    try:
        result = subprocess.run(
            [executable, "--query-gpu=name,driver_version", "--format=csv,noheader"],
            check=True,
            capture_output=True,
            text=True,
            timeout=8,
        )
    except (subprocess.SubprocessError, OSError) as error:
        return CheckResult("NVIDIA GPU", False, str(error))
    detail = result.stdout.strip() or "nvidia-smi 没有返回 GPU"
    return CheckResult("NVIDIA GPU", bool(result.stdout.strip()), detail)


def check_torch_cuda() -> CheckResult:
    """确认 PyTorch 不仅已安装，而且真的能创建 CUDA 上下文。"""
    try:
        import torch
    except ImportError:
        return CheckResult("PyTorch CUDA", False, "无法导入 torch")
    if not torch.cuda.is_available():
        return CheckResult("PyTorch CUDA", False, "torch.cuda.is_available() 为 False")
    cuda_version = torch.version.cuda or "未知"
    ok = cuda_version.startswith(LEISAAC_PROFILE["cuda"])
    detail = f"{torch.cuda.get_device_name(0)}；PyTorch CUDA {cuda_version}（课程要求 12.8）"
    return CheckResult("PyTorch CUDA", ok, detail)


def check_leisaac_root(root: Path) -> list[CheckResult]:
    """检查本章直接调用的 LeIsaac 上游脚本，避免运行时才发现路径错误。"""
    resolved = root.expanduser().resolve()
    expected = {
        "任务列表脚本": "scripts/environments/list_envs.py",
        "遥操作录制脚本": "scripts/environments/teleoperation/teleop_se3_agent.py",
        "HDF5 回放脚本": "scripts/environments/teleoperation/replay.py",
        "Dataset v3 转换脚本": "scripts/convert/isaaclab2lerobotv3.py",
        "状态机数据生成脚本": "scripts/datagen/state_machine/generate.py",
        "PickOrange状态机": "source/leisaac/leisaac/datagen/state_machine/pick_orange.py",
    }
    return [
        CheckResult(name, (resolved / relative).is_file(), str(resolved / relative))
        for name, relative in expected.items()
    ]


def run_checks(leisaac_root: Path) -> list[CheckResult]:
    """按课程锁定版本检查完整 LeIsaac 环境，所有结果都是必需项。"""
    python_ok = sys.version_info[:2] == (3, 11)
    results = [
        CheckResult(
            "Python",
            python_ok,
            f"{platform.python_version()}（课程要求 3.11）",
        ),
        check_nvidia_smi(),
        check_torch_cuda(),
    ]
    for distribution in ("numpy", "torch", "isaacsim", "isaaclab", "lerobot", "leisaac"):
        installed = package_version(distribution)
        expected = LEISAAC_PROFILE.get(distribution)
        # CUDA wheel 可能显示为 2.7.0+cu128；本地版本后缀不改变课程锁定的基础版本。
        base_version = installed.split("+", 1)[0] if installed is not None else None
        version_ok = installed is not None and (expected is None or base_version == expected)
        detail = installed or "未安装"
        if expected is not None:
            detail += f"（课程版本 {expected}）"
        results.append(CheckResult(distribution, version_ok, detail))
    results.extend(check_leisaac_root(leisaac_root))
    return results


def format_results(results: list[CheckResult]) -> str:
    """把检查结果格式化成适合终端阅读的中文表格。"""
    lines = ["状态  必需检查项              结果"]
    for result in results:
        status = "通过" if result.ok else "失败"
        lines.append(f"{status:<4}  {result.name:<20} {result.detail}")
    return "\n".join(lines)


def main() -> None:
    """命令行入口：任一 GPU、版本或源码检查失败时返回非零状态码。"""
    parser = argparse.ArgumentParser(description="检查 SO101 LeIsaac 完整 GPU 环境")
    parser.add_argument("--leisaac-root", type=Path, required=True, help="LeIsaac 源码仓库路径")
    args = parser.parse_args()
    results = run_checks(args.leisaac_root)
    print(format_results(results))
    if any(not item.ok for item in results):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
