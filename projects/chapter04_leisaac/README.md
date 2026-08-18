# 第4章：用 LeIsaac 完成 SO101 仿真数据闭环

本项目只讲一条主线：在 NVIDIA GPU 环境中运行开源 LeIsaac 的
`LeIsaac-SO101-LiftCube-v0`，完成遥操作、HDF5 录制、仿真回放、LeRobot Dataset v3
转换和数据质量检查，不扩展其他仿真器路线。

项目不是 ROS 2 包：它没有 `package.xml`，放在 `projects/` 下，不参与 `colcon build`。
第2、3章继续使用 ROS Humble 环境；本章必须使用独立的 Python 3.11 Conda 环境。

## 1. LeIsaac 主线与名词对应

```text
NVIDIA GPU
  → Isaac Sim（USD 场景、渲染、RGB/深度相机）
  → PhysX（SO101 关节、方块、桌面和接触物理）
  → Isaac Lab（LiftCube 环境、观测、动作、控制与并行环境）
  → LeIsaac（SO101 任务、遥操作、HDF5 录制与回放）
  → LeRobot（Dataset v3 与后续策略训练接口）
```

- `observation`：LeIsaac 在当前时刻提供给策略的信息，例如 SO101 关节状态和相机图像。
- `action`：LeIsaac 在一个控制周期中交给 SO101 控制器的目标。
- `frame`：同一时间点的 observation、action、timestamp 和索引。
- `episode`：从场景重置到成功或失败结束的一整段 LiftCube 尝试。
- HDF5：LeIsaac 遥操作阶段的原始录制文件，便于仿真回放。
- Dataset v3：经 LeIsaac 官方脚本转换后的 LeRobot 训练数据格式。

运行 `python examples/02_explain_leisaac_stack.py` 可在终端查看这组对应关系。

## 2. 目录

```text
chapter04_leisaac/
├── configs/compatibility.json       课程锁定的完整 GPU 版本组合
├── examples/01...12                 按学习顺序排列的入口
├── requirements/leisaac.txt         数据读取和转换依赖
├── samples/umi_episode.json         UMI→LeIsaac 适配概念样例
├── src/so101_leisaac_course/        命令构造、读取、质检和坐标变换
├── tests/                            不启动仿真的函数级回归测试
└── pyproject.toml                    独立 Python 项目配置
```

LeIsaac 上游源码不会复制进本项目。课程代码直接检查并调用其中的官方脚本，避免维护一套
容易过期的仿真副本。

## 3. 强制环境要求

- Linux 和可用的 NVIDIA GPU；`nvidia-smi` 必须正常。
- Python 3.11、CUDA Toolkit 12.8。
- PyTorch 2.7.0 / torchvision 0.22.0，CUDA 12.8 wheel。
- Isaac Sim 5.1.0、Isaac Lab 2.3.0。
- LeRobot 0.4.2、NumPy 1.26.0。
- 递归克隆的 LeIsaac 源码。

版本锁定在 [compatibility.json](configs/compatibility.json)。这些组件耦合紧密，不要只升级
其中一个再继续使用本章命令。

按 LeIsaac 源码安装方式准备环境：

```bash
git clone --recursive https://github.com/LightwheelAI/leisaac.git ~/third_party/leisaac

conda create -n leisaac python=3.11 -y
conda activate leisaac
conda install -c "nvidia/label/cuda-12.8.1" cuda-toolkit -y
pip install -U torch==2.7.0 torchvision==0.22.0 \
  --index-url https://download.pytorch.org/whl/cu128
pip install --upgrade pip
pip install "isaacsim[all,extscache]==5.1.0" \
  --extra-index-url https://pypi.nvidia.com

cd ~/third_party/leisaac/dependencies/IsaacLab
./isaaclab.sh --install
cd ~/third_party/leisaac
pip install -e "source/leisaac[lerobot]"
pip install lerobot==0.4.2 numpy==1.26.0
```

最后安装本教学项目：

```bash
cd /你的代码仓/projects/chapter04_leisaac
pip install -e .
```

完整检查：

```bash
python examples/01_check_environment.py --leisaac-root ~/third_party/leisaac
```

每一项都是必需项。脚本还会实际检查 `torch.cuda.is_available()`，并确认任务枚举、遥操作、
回放和 v3 转换脚本存在。

## 4. 十二个递进样例

样例 03～07 默认打印经过 shell 转义的上游命令，便于先理解参数再复制执行。所有仿真命令
都明确使用 `--device=cuda` 和 `--enable_cameras`。

| 样例 | 学习内容 | 与 LeIsaac 的关系 |
|---|---|---|
| 01 | 完整环境检查 | 验证 GPU、版本和官方脚本 |
| 02 | 技术栈名词 | 把 PhysX、Isaac Sim、Isaac Lab、LeRobot 放回 LeIsaac 链路 |
| 03 | 枚举任务 | 调用 LeIsaac `list_envs.py` |
| 04 | 遥操作录制 | 运行 SO101 LiftCube，保存 HDF5 |
| 05 | 仿真回放 | 在 LeIsaac 中复核动作与物理结果 |
| 06 | HDF5 结构 | 查看 episode、observation、action 的形状 |
| 07 | v3 转换 | 调用 LeIsaac 官方转换器 |
| 08 | 数据说明书 | 阅读 Dataset v3 `meta/info.json` |
| 09 | 数据质检 | 检查索引、时间、维度和动作连续性 |
| 10 | episode 切分 | 防止同段 LiftCube 轨迹泄漏到不同集合 |
| 11 | UMI 相对动作 | 得到等待 LeIsaac 重定向的末端动作 |
| 12 | UMI 中间帧 | 明确其仍须在 LeIsaac 中回放验证 |

## 5. 从任务到 Dataset v3

先确认任务已注册：

```bash
python examples/03_list_leisaac_tasks.py --leisaac-root ~/third_party/leisaac
```

复制终端打印的命令运行，输出中应包含 `LeIsaac-SO101-LiftCube-v0`。

生成键盘遥操作录制命令：

```bash
python examples/04_record_lift_cube.py \
  --leisaac-root ~/third_party/leisaac \
  --device keyboard \
  --dataset datasets/lift_cube.hdf5
```

也可以把 `--device` 改成 `gamepad` 或 `so101leader`。使用真实 Leader 时再传
`--port /dev/ttyACM0`。Isaac Sim 窗口出现后：按 `b` 开始，失败按 `r` 丢弃并重置，成功
按 `n` 保存并重置。采集时始终用 `--num_envs=1`，否则多个环境会混淆人工输入语义。

录制后必须先回放：

```bash
python examples/05_replay_hdf5.py \
  --leisaac-root ~/third_party/leisaac \
  --record-device keyboard \
  datasets/lift_cube.hdf5
```

复制打印命令执行，并在 Isaac Sim 窗口检查夹爪是否对准方块、闭合时刻是否正确、方块是否
稳定离桌。回放失败的数据不要直接转换。

查看原始文件结构：

```bash
python examples/06_inspect_hdf5.py datasets/lift_cube.hdf5
```

转换为 Dataset v3：

```bash
python examples/07_convert_to_lerobot_v3.py \
  --leisaac-root ~/third_party/leisaac \
  datasets/lift_cube.hdf5 \
  --repo-id local/so101_lift_cube_course \
  --record-device keyboard \
  --fps 30
```

`repo-id` 必须是 `namespace/name`。转换输出位置由 LeRobot 0.4.2 的数据集根目录规则决定；
找到其 `meta/info.json` 后，用实际路径执行：

```bash
python examples/08_inspect_lerobot_dataset.py /实际/Dataset根目录
python examples/09_validate_lerobot_dataset.py /实际/Dataset根目录 \
  --repo-id local/so101_lift_cube_course
python examples/10_split_episodes.py --episodes 20 --seed 42
```

质量检查中的 error 必须修复；动作跳变和采样间隔 warning 要回到 LeIsaac 回放中人工复核。
训练、验证、测试必须按 episode 切分，不能随机拆散 frame。

## 6. UMI 概念只作为 LeIsaac 输入适配

本章不把 UMI 轨迹直接称为 SO101 训练数据。样例只解释：外部末端位姿如何转换成相对
SE(3) 动作，并进入 LeIsaac 的尺度标定、坐标对齐、IK、关节限位、碰撞检查和回放验证。

```bash
python examples/11_umi_relative_actions.py samples/umi_episode.json
python examples/12_umi_to_leisaac_adapter.py samples/umi_episode.json
```

`outputs/umi_leisaac_adapter.jsonl` 是调试中间文件，不是 LeRobot Dataset。只有动作在
LeIsaac SO101 场景中通过回放验证，再由官方录制/转换链路生成的数据才进入训练流程。

## 7. 测试与常见问题

```bash
python -m pytest
python -m ruff check src tests examples
```

这些是函数级回归测试，不替代 GPU 验收。真正完成本章必须看到 Isaac Sim 的 LiftCube 场景，
完成至少一条成功轨迹的录制、回放与转换。

- `nvidia-smi` 正常但 CUDA 检查失败：通常是 PyTorch wheel 与 CUDA 环境不匹配。
- 找不到官方脚本：确认仓库使用 `--recursive` 克隆，并把 `--leisaac-root` 指向仓库根目录。
- Isaac Sim 启动后黑屏或相机为空：确认命令包含 `--enable_cameras`，并检查驱动兼容性。
- 回放时夹爪和方块错位：优先排查 task 名、HDF5 来源、坐标系和录制时的环境版本。
- Dataset 帧数不符：检查录制过程中是否错误中断，再查看 HDF5 episode 结构。

更细的逐步说明见 [第4章详细学习文档](../../docs/chapter04/LEARNING_GUIDE.md)。
