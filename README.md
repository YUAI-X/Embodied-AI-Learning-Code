# 《具身智能从入门到实践》笔记学习代码

> 作者：宇哥的具身笔记

本仓库是《具身智能从入门到实践》的配套学习代码，以开源 LeRobot SO101 机械臂为统一案例，覆盖机器人学基础、ROS 2、运动控制、机器人视觉、三维感知、仿真操作和数据集处理。

项目按“基础控制 → 环境感知 → 仿真与数据”的顺序组织。第 2、3 章组成一个 ROS 2 工作空间，可以在没有真实机械臂和相机的情况下，通过 Mock Hardware 与合成 RGB-D 数据完成核心实验；第 4 章是独立的 LeIsaac/Isaac Lab 项目，需要 NVIDIA GPU 环境。

> 本仓库用于教学和算法验证，不包含真实 SO101 硬件驱动。连接真机前，请自行完成限位、急停、碰撞保护和通信安全检查。

## 项目内容

| 章节 | 主题 | 主要内容 | 运行环境 |
|---|---|---|---|
| 第 2 章 | 机器人学、ROS 系统与控制 | URDF/Xacro、TF、PID、正逆运动学、ROS 2 通信、ros2_control、MoveIt 2、虚拟抓放 | Ubuntu 22.04、ROS 2 Humble、Python 3.10 |
| 第 3 章 | 机器人视觉与三维感知 | 相机模型、标定、手眼标定、RGB-D、点云、分割、PnP 位姿估计、TF2 | 与第 2 章共用 ROS 2 工作空间 |
| 第 4 章 | 具身智能仿真与数据 | LeIsaac 环境检查、遥操作、HDF5 录制与回放、LeRobot Dataset v3、数据质检、PickOrange 状态机数据生成 | Python 3.11、NVIDIA GPU、Isaac Sim、Isaac Lab、LeIsaac |

三章之间的关系如下：

```text
机器人模型与控制（第 2 章）
  → 相机、点云与目标位姿（第 3 章）
  → 仿真操作、轨迹采集与训练数据（第 4 章）
```

## 仓库结构

```text
.
├── assets/
│   ├── robots/so101_follower.usd          SO101 仿真模型
│   └── scenes/kitchen_with_orange/        PickOrange 场景、物体和纹理
├── docs/
│   ├── chapter02/                         第 2 章学习指南与故障排查
│   ├── chapter03/                         第 3 章学习指南与可选模型说明
│   └── chapter04/                         第 4 章学习指南
├── src/
│   ├── chapter02/                         ROS 2、运动学、控制与任务包
│   └── chapter03/                         视觉算法与 ROS 2 视觉节点
├── projects/
│   └── chapter04_leisaac/                 独立的 LeIsaac 数据闭环项目
├── NOTICE.md                              第三方来源与修改说明
└── LICENSE                                Apache-2.0 许可证
```

第 2、3 章虽然按章节分目录存放，但共同位于当前仓库的 `src/` 下，应在仓库根目录统一执行 `colcon build`。第 3 章会直接复用第 2 章的 SO101 模型、相机坐标系、TF 和 Mock Hardware。

第 4 章位于 `projects/`，不是 ROS 2 包，不参与 `colcon build`，也不应安装到 ROS 2 的 Python 环境中。

## 第 2、3 章：ROS 2 工作空间

### 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10（Ubuntu/ROS 系统 Python）
- `colcon`、`rosdep`
- MoveIt 2 与 ros2_control 相关依赖

### 克隆与构建

```bash
git clone https://github.com/YUAI-X/Embodied-AI-Learning-Code.git
cd Embodied-AI-Learning-Code

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3

source install/setup.bash
```

如果系统安装了 Anaconda/Miniconda，建议先退出 Conda 环境，并保留上面的两个 Python 参数，避免 ROS 2 错误链接到 Conda Python。

### 快速验证

显示 SO101 模型、关节和 TF：

```bash
ros2 launch so101_description display.launch.py
```

启动 MoveIt 2 虚拟规划环境：

```bash
ros2 launch so101_moveit_config demo.launch.py
```

启动第 3 章综合视觉实验：

```bash
ros2 launch so101_vision_ros vision_demo.launch.py
```

这些实验默认使用仿真或合成输入，不要求连接真实机械臂和 RGB-D 相机。更完整的运行顺序、参数解释和预期结果见各章学习指南。

## 第 4 章：LeIsaac 仿真数据闭环

第 4 章与 ROS 2 工作空间相互独立，主要完成以下流程：

```text
检查 GPU 与软件版本
  → 枚举 LeIsaac 任务
  → 遥操作或状态机生成轨迹
  → 保存 HDF5
  → 仿真回放
  → 转换为 LeRobot Dataset v3
  → 检查数据结构与轨迹质量
  → 按 episode 划分训练集和验证集
```

课程锁定的参考组合为 Python 3.11、CUDA Toolkit 12.8、PyTorch 2.7.0、Isaac Sim 5.1.0、Isaac Lab 2.3.0、LeIsaac 0.4.0、LeRobot 0.4.2 和 NumPy 1.26.0。准确版本以 [`compatibility.json`](projects/chapter04_leisaac/configs/compatibility.json) 为准。

准备好 LeIsaac 环境后，安装本章项目：

```bash
cd projects/chapter04_leisaac
python -m pip install -e .
python examples/01_check_environment.py --leisaac-root ~/third_party/leisaac
```

主要任务包括：

- `LeIsaac-SO101-LiftCube-v0`：人工遥操作、HDF5 录制、回放和 Dataset v3 转换主线。
- `LeIsaac-SO101-PickOrange-v0`：调用 LeIsaac 状态机批量生成抓取轨迹。

生成 PickOrange 命令的入口：

```bash
python examples/13_generate_pick_orange_state_machine.py \
  --leisaac-root ~/third_party/leisaac \
  --dataset datasets/pick_orange_state_machine.hdf5 \
  --num-demos 50 \
  --num-envs 1 \
  --seed 42
```

该入口负责参数校验并打印 LeIsaac 官方命令；实际仿真、控制和录制仍由 LeIsaac 执行。详细安装过程、14 个递进样例和数据验收方法见[第 4 章项目说明](projects/chapter04_leisaac/README.md)。

## 资产与生成数据

仓库已包含运行 PickOrange 场景所需的 SO101 USD 模型、厨房场景、三个橘子、托盘和纹理资源，统一放在 `assets/` 下。

仿真生成的 HDF5、LeRobot 数据集、日志、缓存和可视化输出不属于源代码，不应提交到 Git。建议统一写入 `projects/chapter04_leisaac/datasets/` 或其他本地数据目录，并在采集前确认磁盘空间充足。

## 测试

第 2、3 章：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon test --packages-select \
  so101_description so101_interfaces so101_math so101_ros2_basics \
  so101_control so101_moveit_config so101_tasks \
  so101_vision so101_vision_ros
colcon test-result --verbose
```

第 4 章的函数级测试不启动 Isaac Sim：

```bash
cd projects/chapter04_leisaac
python -m pytest
```

## 文档导航

| 内容 | 文档 |
|---|---|
| 第 2 章完整学习路线 | [第 2 章学习指南](docs/chapter02/LEARNING_GUIDE.md) |
| 第 2 章代码入口 | [第 2 章 README](src/chapter02/README.md) |
| ROS 2 常见环境问题 | [故障排查](docs/chapter02/TROUBLESHOOTING.md) |
| 第 3 章完整学习路线 | [第 3 章学习指南](docs/chapter03/LEARNING_GUIDE.md) |
| 第 3 章代码入口 | [第 3 章 README](src/chapter03/README.md) |
| Grounding DINO 与 SAM | [可选模型说明](docs/chapter03/OPTIONAL_MODELS.md) |
| 第 4 章完整学习路线 | [第 4 章学习指南](docs/chapter04/LEARNING_GUIDE.md) |
| LeIsaac 项目与样例 | [第 4 章项目说明](projects/chapter04_leisaac/README.md) |

## 许可与来源

SO101 主体结构与 STL 来源于 TheRobotStudio 的 [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) 项目，第三方资源和修改说明见 [NOTICE.md](NOTICE.md)。

本仓库采用 [Apache-2.0](LICENSE) 许可证。
