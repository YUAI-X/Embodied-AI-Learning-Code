# 3.4选做：接入真实Grounding DINO与SAM

这一部分是进阶选做，不影响第3章基础代码的构建。建议先完成轻量颜色 Prompt、目标点云和
TF2 综合实验，再安装模型，否则很难区分错误来自模型环境、图像输入还是三维坐标链。

## 1. 模型在流水线中的职责

```text
Grounding DINO：RGB + 文本Prompt → 一个或多个检测框
SAM：RGB + 检测框 → 像素级Mask
本项目：Mask + 对齐Depth + CameraInfo → 目标点云与三维位置
```

Grounding DINO 负责“要找什么以及在哪里”，SAM 负责“目标的精确像素边界”。二者都不直接
输出 SO101 `base_link` 中的三维位置；Depth 反投影和 TF2 仍由本章后半段完成。

## 2. 为什么不放进默认依赖

真实模型通常涉及 PyTorch、CUDA、编译扩展、配置文件和较大的权重。不同显卡、驱动与
PyTorch 组合需要不同安装方式。若把这些依赖写进基础 ROS 包，初学者即使只想运行相机模型
也可能被 GPU 环境阻塞。

因此 `grounded_sam_optional.py` 使用延迟导入：

- 不运行它时，不要求安装模型环境；
- `colcon build` 和基础测试不会下载权重；
- 模型输出仍保持普通 Mask，后续三维代码不绑定模型实现。

## 3. 安装前检查

建议单独创建模型 Python 环境，并先确认：

```bash
nvidia-smi
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
```

CPU 也可以运行部分配置，但通常较慢。具体 PyTorch/CUDA 安装命令应根据机器驱动选择，不要
盲目复制其他电脑的 CUDA 版本。

安装与权重下载以官方说明为准：

- [Grounding DINO官方仓库](https://github.com/IDEA-Research/GroundingDINO)
- [Segment Anything官方仓库](https://github.com/facebookresearch/segment-anything)
- [Grounded Segment Anything官方集成仓库](https://github.com/IDEA-Research/Grounded-Segment-Anything)

准备完成后至少应能导入：

```bash
python3 -c "import groundingdino; from segment_anything import SamPredictor"
```

## 4. 准备文件

运行入口需要：

| 文件 | 参数 | 说明 |
|---|---|---|
| 输入RGB图片 | `--image` | 常见PNG或JPG |
| Grounding DINO配置 | `--grounding-config` | 与所用权重匹配 |
| Grounding DINO权重 | `--grounding-checkpoint` | 检测模型权重 |
| SAM权重 | `--sam-checkpoint` | 与`--sam-type`匹配 |

先查看本地入口参数：

```bash
ros2 run so101_vision grounded_sam_optional --help
```

## 5. 推理命令模板

下面是参数结构示例，路径需要替换成自己的文件：

```bash
ros2 run so101_vision grounded_sam_optional \
  --image /path/to/rgb.png \
  --prompt "red cup" \
  --grounding-config /path/to/GroundingDINO_config.py \
  --grounding-checkpoint /path/to/groundingdino.pth \
  --sam-checkpoint /path/to/sam_vit_b.pth \
  --sam-type vit_b \
  --device cuda \
  --box-threshold 0.35 \
  --text-threshold 0.25 \
  --output /tmp/grounded_sam_mask.png
```

程序会打印检测短语和置信度，并保存单通道 Mask。没有 GPU 时可尝试 `--device cpu`。

## 6. 阈值如何调整

- `box-threshold` 太高：目标可能完全没有检测框；
- `box-threshold` 太低：可能出现大量无关框；
- `text-threshold` 太高：文本匹配更严格，可能漏检；
- Prompt 太宽泛：可能返回多个同类目标；
- Prompt 与训练语义差异大：即使目标可见也可能无法定位。

先保存并检查检测/Mask 结果，再接深度。不要在 2D 结果明显错误时继续调点云参数。

## 7. 接回本章三维流水线

真实模型最终应输出与原 RGB 图相同宽高的 Mask：目标像素为非零，背景为零。随后调用：

```text
masked_target_cloud(color, depth, mask, intrinsics)
```

处理顺序保持不变：

1. 确保 RGB 与 Depth 已对齐；
2. 适当腐蚀 Mask，减少物体边缘背景深度；
3. 过滤 0、NaN 和量程外深度；
4. 将 Mask 内像素反投影为目标点云；
5. 估计 Camera Frame 目标中心或完整位姿；
6. 通过 TF2 转换到 SO101 `base_link`；
7. 使用 MoveIt 检查抓取位姿可达性和碰撞。

如果模型输出多个 Mask，应先按检测分数、文本匹配或任务规则选择目标，不要直接把所有 Mask
合并成一个点云，否则多个物体会得到一个没有意义的平均中心。

## 8. 常见问题

### `No module named groundingdino`

模型仓库没有安装到当前运行 `ros2` 所使用的 Python 环境。检查 `which python3`、
`which ros2` 和模型安装位置。ROS 2 Humble 默认使用系统 Python 3.10。

### CUDA不可用或显存不足

先改用较小的 SAM 模型类型和较小输入图，或使用 CPU 验证接口。不要仅通过降低检测阈值
处理显存问题，两者没有关系。

### 有检测框但Mask错误

检查传给 SAM 的框是否已从归一化中心格式正确转换为像素 `xyxy`，并确认 RGB/BGR 顺序。
本项目入口已经执行框格式转换，替换模型 API 时需保留这一步。

### Mask正确但目标点云错误

问题通常不在大模型，而在 RGB/Depth 未对齐、深度单位错误、CameraInfo 分辨率不匹配，或
Mask 边缘混入背景。返回[第三章详细学习README](LEARNING_GUIDE.md)的 3.1 和真实相机迁移
章节逐项检查。
