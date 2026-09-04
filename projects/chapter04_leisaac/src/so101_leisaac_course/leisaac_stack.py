"""把容易混淆的名词放回 LeIsaac 实际运行链路中解释。"""
# 作者：宇哥的具身笔记


from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class StackLayer:
    """LeIsaac 技术栈中的一层，以及本章真正使用它的位置。"""

    name: str
    role: str
    in_course: str


STACK = (
    StackLayer("PhysX", "刚体、关节和接触物理求解", "LiftCube 中 SO101、方块和桌面的运动"),
    StackLayer("Isaac Sim", "USD 场景、渲染、相机与仿真应用", "显示场景并生成 RGB/深度观测"),
    StackLayer("Isaac Lab", "任务环境、配置、并行环境和控制接口", "定义 LiftCube 的观测、动作与奖励"),
    StackLayer("LeIsaac", "面向具身数据采集的任务与遥操作工具", "SO101 遥操作、HDF5 录制和回放"),
    StackLayer("LeRobot", "统一的数据集和策略训练接口", "接收 LeIsaac 转换出的 Dataset v3"),
)


def format_stack() -> str:
    """按依赖顺序输出层级，避免把 LeIsaac 误解为另一款物理引擎。"""
    lines = ["LeIsaac 运行链路："]
    for index, layer in enumerate(STACK, start=1):
        lines.append(f"{index}. {layer.name}: {layer.role}；本章对应：{layer.in_course}")
    return "\n".join(lines)
