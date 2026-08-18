# SO101 ROS 2机器人课程学习代码

本仓库是机器人课程第2～4章的配套项目，以开源 LeRobot SO101 机械臂为统一案例。
第2、3章适用于 Ubuntu 22.04、ROS 2 Humble 和 Python 3.10；第4章是独立的
LeIsaac/Isaac Lab 教学项目，使用 NVIDIA GPU 和 Conda Python 3.11。

第2、3章核心实验可使用 Mock Hardware 与合成 RGB-D 完成，不要求机械臂或相机。第4章
要求完整的 NVIDIA GPU、Isaac Sim、Isaac Lab 与 LeIsaac 环境。本仓库是教学项目，
不是真机驱动。

## 文档导航

| 内容 | 文档 |
|---|---|
| 第2章：机器人学、ROS系统与控制 | [第2章学习指南](docs/chapter02/LEARNING_GUIDE.md) |
| 第3章：机器人视觉与三维感知 | [第3章学习指南](docs/chapter03/LEARNING_GUIDE.md) |
| 第3章代码入口 | [第3章README](src/chapter03/README.md) |
| 第4章：具身智能数据与仿真 | [第4章学习指南](docs/chapter04/LEARNING_GUIDE.md) |
| 第4章代码入口 | [LeIsaac课程README](projects/chapter04_leisaac/README.md) |
| Grounding DINO与SAM | [可选模型说明](docs/chapter03/OPTIONAL_MODELS.md) |
| 常见环境问题 | [故障排查](docs/chapter02/TROUBLESHOOTING.md) |

## 目录结构

```text
src/
├── chapter02/                    机器人学、ROS 2与控制
│   ├── so101_description/        SO101模型、Xacro、TF与RViz
│   ├── so101_math/               PID、坐标变换、FK与IK
│   ├── so101_ros2_basics/        ROS 2基础与RGB-D消息
│   ├── so101_control/            Mock Hardware
│   ├── so101_moveit_config/      MoveIt 2配置
│   └── ...
└── chapter03/                    视觉与三维感知
    ├── so101_vision/             纯视觉算法
    └── so101_vision_ros/         ROS 2视觉节点

projects/
└── chapter04_leisaac/            LeIsaac SO101仿真数据闭环项目
```

第2、3章分目录存放，但仍属于同一个colcon工作空间。第3章直接复用第2章的SO101模型、TF
和 Mock Hardware。第4章位于`projects/`，不参与colcon构建，也不与ROS Python环境混装。

## 构建

```bash
source /opt/ros/humble/setup.bash
cd ~/so101_ros2_learning_ws
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3

source install/setup.bash
```

安装了 Anaconda 时，请保留两个系统 Python 参数。

## 快速运行

显示 SO101：

```bash
ros2 launch so101_description display.launch.py
```

启动第3章综合视觉实验：

```bash
ros2 launch so101_vision_ros vision_demo.launch.py
```

完整学习步骤与预期结果请直接阅读对应章节学习指南，README 不再重复教程内容。

## 模型与许可

SO101 主体结构与 STL 来自 TheRobotStudio 的
[SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) 项目，修改说明见 [NOTICE.md](NOTICE.md)。
本仓库使用 Apache-2.0 许可证。
