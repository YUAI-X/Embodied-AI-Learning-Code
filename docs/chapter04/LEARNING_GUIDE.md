# 第4章详细学习文档：从 PickOrange 数据合成到 LeRobot Dataset v3

## 0. 本章边界

本章只使用开源 LeIsaac，以“合成、检查、回放、转换、校验”为学习主线。核心实践使用
`LeIsaac-SO101-PickOrange-v0` 讲解上游状态机数据合成，因为当前版本尚未注册 LiftCube
状态机；`LeIsaac-SO101-LiftCube-v0` 保留为人工遥操作对照。学习者必须具备 NVIDIA GPU
环境，文档不扩展比较其他仿真器。

最终验收不是“脚本能打印”，而是完成以下闭环：

```text
检查 GPU 软件栈
  → 用 PickOrange 状态机合成 HDF5
  → 在同一 LeIsaac 环境回放
  → 转换为 LeRobot Dataset v3
  → 检查元数据和逐帧质量
  → 按 episode 切分
```

教学代码位于 `projects/chapter04_leisaac`。它是独立 Python 项目，不是 ROS 2 包，不需要也
不应该被 `colcon build` 编译。

### 0.1 初学者先跑哪几个样例

第一次学习不要从 01 到 14 机械地全部执行。先按下面顺序跑通最短闭环：

| 顺序 | 样例 | 得到什么 | 通过标准 |
|---|---|---|---|
| 1 | 01 | 环境与上游脚本检查 | 所有必需项通过 |
| 2 | 13 | 2 个 PickOrange HDF5 episode | 官方命令执行结束，文件可读 |
| 3 | 06 | HDF5 结构 | episode、observation、action、相机数据完整 |
| 4 | 05 | 仿真回放 | 物理行为与状态机动作一致 |
| 5 | 07 | LeRobot Dataset v3 | 生成 `meta/`、`data/`、`videos/` |
| 6 | 08、09 | 数据合同与逐帧质检 | 无 error，warning 有解释 |
| 7 | 14 | Rerun 可视化 | 能同步查看相机、关节状态和动作 |
| 8 | 10 | episode 切分 | train/validation/test 无 episode 泄漏 |

其中 05、07、13 是“命令生成器”：课程脚本先检查参数并打印上游命令，必须再复制打印结果
执行。只看到“复制以上命令执行”不代表已经录制、回放或转换。

本章存在两条数据来源，但只有一条后处理链路：

```text
主线：PickOrange 状态机 ─┐
                         ├→ HDF5 检查 → 仿真回放 → Dataset v3 → 校验/可视化
对照：LiftCube 人工遥操作 ─┘
```

两条来源不能混用任务参数。PickOrange 数据固定配对
`LeIsaac-SO101-PickOrange-v0 + so101_state_machine`；LiftCube 键盘数据配对
`LeIsaac-SO101-LiftCube-v0 + keyboard`（手柄或 Leader 则换成实际设备）。

### 0.2 两种格式分别解决什么问题

| 格式 | 主要用途 | 学员重点检查 |
|---|---|---|
| HDF5 | LeIsaac 原始轨迹录制与仿真回放 | episode 是否完整，动作与相机是否对齐，任务是否成功 |
| LeRobot Dataset v3 | 策略训练、共享和统一读取 | `info.json`、Parquet、视频、索引、fps 和 feature schema |

格式转换不是压缩、复制或修改后缀。它会把 HDF5 内的环境轨迹整理成训练侧约定的数据结构，
并写入元数据合同。HDF5 回放通过后再转换，可以减少把物理失败轨迹带入训练集的风险。

### 0.3 运行前先固定 LeIsaac 根目录

所有命令中的 `--leisaac-root` 都必须指向真实源码根目录。推荐在当前终端统一设置：

```bash
export LEISAAC_ROOT=/你的实际路径/leisaac
test -f "$LEISAAC_ROOT/scripts/datagen/state_machine/generate.py"
```

后续统一写 `--leisaac-root "$LEISAAC_ROOT"`。不要假设源码一定在
`~/third_party/leisaac`；路径写错会在启动仿真前直接报 `FileNotFoundError`。

## 1. 先理解 LeIsaac 中的五层关系

### 1.1 PhysX 在这里做什么

PhysX 是 LeIsaac 场景底层的物理求解器。SO101 关节转动、夹爪与方块接触、方块重力和桌面
碰撞都由它计算。课程不会直接写 PhysX 程序；我们通过 Isaac Lab 的任务配置间接使用它。

### 1.2 Isaac Sim 在这里做什么

Isaac Sim 提供仿真应用、USD 场景、渲染器与相机传感器。运行遥操作命令后看到的 SO101、
桌面和方块窗口属于 Isaac Sim；`--enable_cameras` 打开的 RGB/深度观测也由它生成。

### 1.3 Isaac Lab 在这里做什么

Isaac Lab 把场景组织成机器人学习环境，规定 reset、observation、action、控制周期和终止条件。
LiftCube 的环境配置在 LeIsaac 源码中实现，但遵循 Isaac Lab 的环境接口。`--num_envs=1`
表示遥操作时只运行一个环境；批量训练才会使用并行环境。

### 1.4 LeIsaac 在这里做什么

LeIsaac 是本章直接操作的项目。它提供 SO101 资产与任务注册、键盘/手柄/Leader 遥操作、
HDF5 录制、回放和 Dataset 转换脚本。本章命令构造器始终调用这些官方脚本，而不是重新实现
一份录制器。

### 1.5 LeRobot 在这里做什么

LeRobot 接收转换后的 Dataset v3，为后续策略训练提供统一数据接口。HDF5 是 LeIsaac 仿真
回放友好的原始格式，Dataset v3 是训练侧格式；两者用途不同，不能只改后缀名替代转换。

运行：

```bash
cd projects/chapter04_leisaac
python examples/02_explain_leisaac_stack.py
```

对应实现为 `src/so101_leisaac_course/leisaac_stack.py`。阅读 `STACK` 中每个 `in_course`
字段，确保每个名词都能指向本章的一项具体操作。

## 2. 建立唯一的 GPU 环境

### 2.1 锁定版本

本章使用以下组合：Python 3.11、CUDA Toolkit 12.8、PyTorch 2.7.0、torchvision 0.22.0、
Isaac Sim 5.1.0、Isaac Lab 2.3.0、LeRobot 0.4.2、NumPy 1.26.0。配置同时保存在
`projects/chapter04_leisaac/configs/compatibility.json`。

不要在第2、3章 ROS Humble 的 Python 环境中直接安装这套依赖。两套环境处理不同章节，
代码仍可保存在同一 Git 仓库。

### 2.2 安装 LeIsaac 源码栈

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

cd /你的代码仓/projects/chapter04_leisaac
pip install -e .
```

### 2.3 样例 01：环境检查

```bash
python examples/01_check_environment.py --leisaac-root ~/third_party/leisaac
```

检查器在 `environment.py` 中完成四类工作：

1. 检查 Python 是否严格为 3.11；
2. 调用 `nvidia-smi`，确认驱动和 GPU 可见；
3. 导入 PyTorch 并检查 `torch.cuda.is_available()`；
4. 检查锁定包版本、常规数据闭环脚本以及状态机生成源码。

输出没有“可选”项。任何失败都会返回非零状态码，表示当前环境不能继续本章实验。

## 3. 用 LiftCube 理解仿真任务（对照知识）

本节和第 4 节解释人工遥操作，帮助理解 observation/action 如何进入 HDF5；它们不是自动
合成主线的前置操作。要直接实践状态机合成，可读完 3.2 后跳到第 8 节。

### 3.1 样例 03：任务注册

```bash
python examples/03_list_leisaac_tasks.py --leisaac-root ~/third_party/leisaac
```

脚本打印对上游 `scripts/environments/list_envs.py` 的调用。复制执行后，应找到：

```text
LeIsaac-SO101-LiftCube-v0
```

任务名不是随意标签。它让 LeIsaac/Isaac Lab 找到 SO101 机器人、LiftCube 场景、观测项、
动作控制器和终止条件的整套配置。后续录制、回放和转换要保持同一个任务名。

LeIsaac 还注册了 `LeIsaac-SO101-LiftCube-Direct-v0`。本章固定使用非 Direct 版本，避免在
学习主线中来回切换环境实现方式。

### 3.2 observation、action、frame、episode

在 LiftCube 中，可按以下方式理解数据：

- observation.state：当前 SO101 状态向量；具体维度以转换后的 `info.json` 为准。
- 相机 observation：场景相机产生的 RGB/深度信息；必须启用 camera 才会采集。
- action：遥操作设备经 LeIsaac 映射后发给控制器的目标。
- frame：一个控制时刻的数据快照，带 timestamp、episode_index 和 frame_index。
- episode：一次完整抓取尝试。成功保存或失败丢弃后，环境 reset 并开始下一段。

这里最重要的约束是时间对齐：同一 frame 的 observation 与 action 必须对应同一控制周期。
图像、状态和动作各自正常，并不等于组合起来就是有效训练样本。

## 4. 对照实验：遥操作并录制仿真数据

### 4.1 样例 04：生成录制命令

```bash
python examples/04_record_lift_cube.py \
  --leisaac-root ~/third_party/leisaac \
  --device keyboard \
  --dataset datasets/lift_cube.hdf5
```

教学脚本只负责校验参数并打印官方命令。这样可以清楚看到每个参数，再复制执行：

- `--task=LeIsaac-SO101-LiftCube-v0`：固定课程任务。
- `--teleop_device=keyboard`：输入设备，也可用 `gamepad`、`so101leader`。
- `--num_envs=1`：一个操作者对应一个仿真环境。
- `--device=cuda`：固定使用 NVIDIA GPU。
- `--enable_cameras`：启动相机观测。
- `--record`：启用 HDF5 录制。
- `--dataset_file=...`：指定输出文件。

窗口内操作：按 `b` 开始；失败按 `r` 丢弃当前轨迹并 reset；成功按 `n` 保存当前轨迹并
reset。先采集少量轨迹验证闭环，再扩大数量。

使用 SO101 Leader 时：

```bash
python examples/04_record_lift_cube.py \
  --leisaac-root ~/third_party/leisaac \
  --device so101leader \
  --port /dev/ttyACM0 \
  --dataset datasets/lift_cube.hdf5
```

`commands.py` 中的 `LeIsaacCommandBuilder.teleop()` 集中维护任务、设备白名单与命令参数。
`render_command()` 使用 shell 安全转义，避免路径含空格时生成错误命令。

## 5. 两条路线共用：先回放 HDF5，再做格式转换

### 5.1 样例 05：LeIsaac 回放

```bash
python examples/05_replay_hdf5.py \
  --leisaac-root ~/third_party/leisaac \
  --record-device keyboard \
  datasets/lift_cube.hdf5
```

复制打印出的官方 `replay.py` 命令执行。观察四点：夹爪接近方向、闭合时刻、方块离桌高度、
抬起后的稳定性。回放是物理语义检查；仅查看数组无法判断抓取是否真的成立。

`--record-device` 必须填写录制时实际使用的设备；状态机数据填写
`so101_state_machine`。LeIsaac 会据此恢复正确的动作类型。录制和回放必须使用同一任务、
资产版本和控制配置。若位置对不上，先检查 task 和环境版本，
再排查坐标系、动作语义或损坏的 episode。

### 5.2 样例 06：查看 HDF5 树

```bash
python examples/06_inspect_hdf5.py datasets/lift_cube.hdf5
```

`hdf5_tree.py` 使用 `visititems()` 只列出 group、dataset、shape 和 dtype，不把图像大数组
整体载入内存。学习时重点确认：有多少 episode、每段长度、observation/action 第一维是否
一致、相机数组是否存在。

### 5.3 样例 07：转换到 Dataset v3

```bash
python examples/07_convert_to_lerobot_v3.py \
  --leisaac-root ~/third_party/leisaac \
  datasets/lift_cube.hdf5 \
  --repo-id local/so101_lift_cube_course \
  --record-device keyboard \
  --fps 30
```

脚本调用 LeIsaac 官方 `scripts/convert/isaaclab2lerobotv3.py`。`--task` 和
`--record-device` 必须同时与录制阶段一致。转换负责把 HDF5 中的 episode
和数组组织为 LeRobot 0.4.2 的元数据、Parquet 与视频布局。`fps` 必须与采集控制频率一致；
随意填写会让 timestamp 和动作速度语义失真。

## 6. 读取与检查 Dataset v3

### 6.1 样例 08：把 info.json 当作数据合同

```bash
python examples/08_inspect_lerobot_dataset.py /实际/Dataset根目录
```

`schema.py` 读取 `meta/info.json` 并验证：版本、robot_type、fps、总 episode/frame 数以及
features 的 dtype/shape。不要在代码中猜 SO101 状态或动作维度，应始终以当前转换结果为准。

### 6.2 样例 09：逐帧质量检查

```bash
python examples/09_validate_lerobot_dataset.py /实际/Dataset根目录 \
  --repo-id local/so101_lift_cube_course
```

`dataset.py` 通过 LeRobot 0.4.2 的 `LeRobotDataset` API 读取正式数据，不再生成合成
JSONL。为避免复制大图像，检查器只取索引、timestamp、observation.state 和 action。

`quality.py` 检查：

- 必需字段是否存在；
- 每个 episode 是否从 frame 0 开始且索引连续；
- timestamp 是否严格递增并接近 `1/fps`；
- state/action 是否匹配 info.json 维度且没有 NaN/Inf；
- 相邻 action 是否出现异常跳变；
- 实际 frame/episode 数是否与 info.json 一致。

error 表示不能进入训练。warning 不一定是坏数据，但必须结合样例 05 的 LeIsaac 回放判断。

### 6.3 样例 10：按 episode 切分

```bash
python examples/10_split_episodes.py --episodes 20 --seed 42
```

连续 frame 高度相关。如果随机把 frame 分到训练集和验证集，验证结果会虚高。
`splits.py` 先去重并打乱 episode ID，再整体切分，保证三个集合互不重叠。

## 7. UMI 名词如何与 LeIsaac 关联

本节只解释外部轨迹进入 LeIsaac 前的适配，不建立另一条数据路线。

样例输入每个末端位姿使用 `[x,y,z,qx,qy,qz,qw]`。`transforms.py` 将其变成齐次矩阵，
并计算：

```text
T_relative(t) = inverse(T(t-1)) × T(t)
```

相对动作写成 `[dx,dy,dz,rx,ry,rz,gripper_width_m]`，旋转部分是轴角向量。

```bash
python examples/11_umi_relative_actions.py samples/umi_episode.json
python examples/12_umi_to_leisaac_adapter.py samples/umi_episode.json
```

输出 JSONL 只是 LeIsaac 适配调试文件。继续使用之前必须完成：

1. 把 UMI 坐标系标定到 LeIsaac 世界/机器人基座坐标系；
2. 统一米、弧度和夹爪宽度定义；
3. 通过 IK 把末端动作重定向到 SO101 关节目标；
4. 检查关节限位、速度和碰撞；
5. 在 LiftCube 环境回放并观察物理结果；
6. 通过后再走 LeIsaac 官方录制与 Dataset v3 转换链路。

因此 `to_leisaac_adapter_frames()` 的函数名和注释明确避免把中间 JSONL 误称为训练数据。

## 8. 主线实践：状态机自动合成 PickOrange 数据

### 8.1 它生成什么

LeIsaac 0.4.0 新增 `scripts/datagen/state_machine/generate.py`。当前脚本的 `TASK_REGISTRY`
只映射：

```text
LeIsaac-SO101-PickOrange-v0
  → PickOrangeStateMachine
  → so101_state_machine 动作设备
```

状态机不读取人工示范。它直接观察仿真中的橙子、托盘、机器人基座和末端位置，然后按以下
阶段生成 8 维 IK 目标：

```text
接近橙子上方
  → 下探
  → 闭合夹爪
  → 抬起
  → 移到托盘
  → 放下并松开
  → 抬起夹爪
  → 处理下一颗橙子或回到休息位
```

每个 episode reset 时，PickOrange 环境自身的 domain randomization 会改变物体/相机等
配置。因此同一套控制程序可产生不同初态下的轨迹。它属于程序化合成，不是对现有样本做
裁剪、复制，也不同于基于人工示范片段重组的 MimicGen。

### 8.2 为什么课程代码只构造上游命令

状态机依赖 Isaac Lab 的实时环境、USD 场景、Torch CUDA 张量、SO101 IK 动作项和 LeIsaac
录制器。把 `PickOrangeStateMachine` 复制进课程项目会形成难以同步的分叉。因此
`LeIsaacCommandBuilder.state_machine_generate()` 负责：

1. 检查上游 `generate.py` 确实存在；
2. 将任务限制为上游已经注册的 PickOrange；
3. 拒绝 `num_demos=0` 的无限课程作业；
4. 检查并行数、频率和 HDF5 后缀；
5. 生成经过 shell 安全转义、可复制执行的官方命令。

真实控制循环和关键相位继续由上游源码负责。这既是整合，也是清晰的维护边界。

### 8.3 第一次运行：两条可视化验收

在已经通过样例 01 的 LeIsaac 0.4.0 环境中运行：

```bash
cd projects/chapter04_leisaac
python examples/13_generate_pick_orange_state_machine.py \
  --leisaac-root "$LEISAAC_ROOT" \
  --dataset datasets/pick_orange_2episodes.hdf5 \
  --num-demos 2 \
  --num-envs 1 \
  --step-hz 60 \
  --seed 42
```

教学入口只打印命令。复制执行后观察：夹爪接近是否平滑、橙子是否进入托盘、释放后
是否稳定、程序是否在达到目标数量后退出。首次不要添加 `--headless`，否则物理错误只能从
数组和日志间接判断。

固定 `--seed 42` 用于复现问题，不表示每个 episode 完全相同；仿真状态和并行调度仍可能
带来差异。改变 seed 才能有计划地扩大初始状态覆盖。

### 8.4 批量运行、续录与资源参数

小规模验收通过后再生成 50 条：

```bash
python examples/13_generate_pick_orange_state_machine.py \
  --leisaac-root "$LEISAAC_ROOT" \
  --dataset datasets/pick_orange_state_machine.hdf5 \
  --num-demos 50 \
  --num-envs 1 \
  --seed 43 \
  --headless
```

参数含义：

- `--num-demos`：目标成功示范计数；课程入口要求大于 0。
- `--num-envs`：并行 Isaac Lab 环境数。先用 1 验收，再根据显存扩大。
- `--step-hz`：环境 step 节奏，默认 60 Hz；它不自动等于转换后的 Dataset fps。
- `--seed`：环境随机种子。记录每批数据使用的 seed，便于复现和去重分析。
- `--headless`：不显示窗口，但命令仍保留 `--enable_cameras` 录制相机观测。
- `--quality`：使用上游高质量渲染模式，会增加 GPU 成本。
- `--resume`：对已有 HDF5 续录。文件不存在或结构不完整时不要使用。

续录示例：

```bash
python examples/13_generate_pick_orange_state_machine.py \
  --leisaac-root "$LEISAAC_ROOT" \
  --dataset datasets/pick_orange_state_machine.hdf5 \
  --num-demos 100 \
  --seed 44 \
  --resume \
  --headless
```

`--num-demos` 在续录时表示文件最终要达到的目标成功示范总数，不是本次额外追加数。续录前
应先备份文件并用样例 06 确认已有 episode 可读；不要同时启动两个进程写同一个 HDF5。

### 8.5 合成后验收

第一步检查结构：

```bash
python examples/06_inspect_hdf5.py datasets/pick_orange_state_machine.hdf5
```

然后使用相同任务和 commit 抽样回放。状态机写入的是 8 维 IK 动作，必须显式传入
`so101_state_machine`；若省略，上游会按默认 `so101leader` 关节动作解释，语义和维度均不对：

```bash
python examples/05_replay_hdf5.py \
  --leisaac-root "$LEISAAC_ROOT" \
  --task LeIsaac-SO101-PickOrange-v0 \
  --record-device so101_state_machine \
  datasets/pick_orange_state_machine.hdf5
```

回放至少核对：

- observation、action、相机数据的时间长度一致；
- episode 能完整结束，没有进程中断留下的尾段；
- `--orange-index` 指定的橙子到达托盘，机器人最终回到允许误差内的休息位；
- 不同 episode 的物体初态确有变化；
- 失败 episode 是否被当前上游 HDF5 导出模式保留。

最后一点不能只看文件总数。生成脚本以成功示范数决定退出，但标准 HDF5 录制器的导出模式
可能随 LeIsaac/Isaac Lab 版本变化。先检查当前 HDF5 是否提供可依赖的成功/终止字段；若没有，
应记录逐集回放抽检结果，不能臆造固定字段名。若要转换为 LeRobot Dataset，必须同时使用
PickOrange 任务名和状态机动作类型：

```bash
python examples/07_convert_to_lerobot_v3.py \
  --leisaac-root "$LEISAAC_ROOT" \
  --task LeIsaac-SO101-PickOrange-v0 \
  --record-device so101_state_machine \
  --repo-id local/so101_pick_orange_state_machine \
  --fps 30 \
  datasets/pick_orange_state_machine.hdf5
```

这里的 `--fps 30` 必须按实际采样/降采样链路确认，不能因为状态机 `--step-hz 60` 就盲目
填写 60。转换完成后继续执行样例 08、09，并按 episode 切分。

### 8.6 已知边界

- 当前没有 `LiftCubeStateMachine`，因此样例 13 不接受 LiftCube。
- 状态机动作是任务专用规则，不保证迁移到其他资产或场景版本。
- 自动成功不等于高质量；碰撞、抖动和临界抓取仍需人工抽检。
- 本功能扩大的是仿真数据量和初态覆盖，不自动解决 sim-to-real 外观差异。

## 9. 代码阅读顺序

建议按以下顺序阅读核心模块：

1. `leisaac_stack.py`：建立名词与运行链路的映射；
2. `environment.py`：理解完整环境验收；
3. `commands.py`：掌握任务、遥操作、回放、转换和状态机生成参数；
4. `hdf5_tree.py`：认识原始录制结构；
5. `schema.py`、`dataset.py`：读取正式 Dataset v3；
6. `quality.py`、`splits.py`：训练前数据治理；
7. `transforms.py`、`umi.py`：理解外部轨迹为何必须经 LeIsaac 重定向验证。

主要函数均有中文 docstring；复杂步骤旁保留了“为什么这样做”的注释。样例文件只负责参数解析
和调用，核心逻辑集中在 `src/`，便于单元测试。

## 10. 练习与验收

### 练习一：完成一条成功轨迹

用键盘完成至少一次 LiftCube，保存 HDF5，并说明 `b/r/n` 三个按键分别改变哪个 episode
状态。

### 练习二：从物理现象定位数据问题

回放一条失败轨迹，在记录中写清：错误是接近方向、闭合时刻、接触稳定性还是抬升高度。
再对应到 observation、action 或环境配置。

### 练习三：解释格式转换

分别列出 HDF5 和 Dataset v3 的用途，并从 `info.json` 读出 fps、episode 数、frame 数、状态
维度和动作维度。

### 练习四：质量与切分

运行样例 09 和 10，确认无 error，记录 warning 的回放结论，并证明 train/validation/test
之间没有重复 episode。

### 练习五：状态机合成

使用两个不同 seed 各生成至少 3 条 PickOrange 轨迹，比较物体初态和成功率；抽样回放并说明
它与 MimicGen“从已有示范生成新示范”的区别。

### 最终验收清单

- [ ] `nvidia-smi` 和 `torch.cuda.is_available()` 均通过。
- [ ] 环境检查的所有版本和官方脚本通过。
- [ ] 任务列表中存在 `LeIsaac-SO101-PickOrange-v0`；若做对照实验，也存在 LiftCube。
- [ ] Isaac Sim 窗口能显示 SO101、桌面、橙子、托盘和相机结果。
- [ ] 至少 2 个成功 PickOrange episode 已写入 HDF5。
- [ ] 同一 HDF5 能在 LeIsaac 中正确回放。
- [ ] 已使用官方脚本转换为 LeRobot Dataset v3。
- [ ] 元数据规模与实际读取结果一致，质量检查无 error。
- [ ] 数据按完整 episode 切分。
- [ ] PickOrange 合成数据已经完成结构检查和抽样回放。
- [ ] 已用样例 14 同步查看相机、关节状态和动作时间轴。

## 11. 故障定位

### GPU 检查失败

先运行 `nvidia-smi`，再在 Conda 环境运行：

```bash
python -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"
```

驱动可见但 PyTorch 返回 False，通常表示装错 wheel 或环境被其他 Python 污染。

如果报错路径混入 `~/.local/lib/python3.10/site-packages`，而当前环境是 Python 3.11，先隔离
用户级包再重试：

```bash
unset PYTHONPATH
export PYTHONNOUSERSITE=1
python -c "import sys, numpy; print(sys.version); print(numpy.__version__, numpy.__file__)"
```

NumPy 应为 1.26.0，并来自当前 Conda 环境。不要通过删除系统 Python 或用户目录来临时绕过。

### 找不到 LeIsaac 脚本

`--leisaac-root` 必须指向含 `scripts/` 和 `source/` 的仓库根目录。若缺少子模块，重新检查
最初是否使用 `git clone --recursive`。

如果只缺少 `scripts/datagen/state_machine/generate.py`，通常是 LeIsaac 版本早于 0.4.0；更新
仓库后要重新执行可编辑安装，并重新跑样例 01，而不是单独复制一个生成脚本。

### 状态机任务未注册

先确认命令使用 `LeIsaac-SO101-PickOrange-v0`，并检查上游生成脚本的 `TASK_REGISTRY`。
LiftCube 的普通任务存在不代表存在对应状态机。若未来上游新增任务，应先验证动作维度、成功
条件和回放，再扩展本项目白名单。

### 有窗口但没有相机数据

确认实际执行的命令含 `--enable_cameras`。再用样例 06 检查 HDF5 中是否出现图像相关数组；
如果窗口正常而文件没有相机字段，应回到录制参数和任务配置排查。

### 回放位置不一致

保证录制与回放使用同一个 task、LeIsaac commit、Isaac Lab 版本和资产。之后再检查坐标系、
action 表示、时间步长与 episode 是否完整，不能通过手工平移方块掩盖问题。

### `colcon build` 是否会编译本章

不会。本章没有 ROS `package.xml`，并位于仓库的 `projects/chapter04_leisaac`。进入本章目录
使用 Conda、pip 和 Python 命令；第2、3章才进入 ROS 工作空间执行 colcon。
