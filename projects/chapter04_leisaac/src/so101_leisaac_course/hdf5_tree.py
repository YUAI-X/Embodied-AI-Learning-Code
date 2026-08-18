"""只读打印 LeIsaac 遥操作录制得到的 Isaac Lab HDF5 结构。"""

from __future__ import annotations

from pathlib import Path
from typing import Any


def inspect_hdf5(path: Path) -> str:
    """列出group、dataset形状/类型和根属性，不把大数组加载进内存。"""
    try:
        import h5py
    except ImportError as error:
        raise RuntimeError("当前 LeIsaac 环境缺少课程要求的 h5py") from error

    lines: list[str] = []
    with h5py.File(path.expanduser(), "r") as file:
        if file.attrs:
            lines.append("根属性:")
            for name, value in file.attrs.items():
                lines.append(f"  {name}={value}")

        def visitor(name: str, obj: Any) -> None:
            if isinstance(obj, h5py.Group):
                lines.append(f"[group] /{name}")
            else:
                lines.append(f"[data ] /{name} shape={obj.shape} dtype={obj.dtype}")

        file.visititems(visitor)
    return "\n".join(lines)
