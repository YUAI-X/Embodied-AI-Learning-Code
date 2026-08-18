# 常见问题

## 找不到包或可执行文件

确认当前终端依次加载了 ROS 2 和本工作空间：

```bash
source /opt/ros/humble/setup.bash
source ~/so101_ros2_learning_ws/install/setup.bash
ros2 pkg list | grep so101
```

每次构建后都要重新加载工作空间环境。

## Anaconda 导致构建或导入失败

ROS 2 Humble 在 Ubuntu 22.04 上使用系统 Python 3.10。若终端中的 `python3`
指向 Conda，请显式指定系统解释器：

```bash
colcon build --symlink-install --cmake-clean-cache \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
```

同时设置两个变量是为了兼容 ROS 2 Humble 中新旧 CMake 查找模块；个别包提示其中一个变量
未使用不影响构建。运行前也可以先执行 `conda deactivate`。

## RViz 中模型不显示

检查 Fixed Frame 是否为 `base_link`，再检查 TF 和机器人描述：

```bash
ros2 topic echo /robot_description --once
ros2 run tf2_ros tf2_echo base_link tool0
```

如果只看到模型不动，确认 `joint_states` 正在发布。

## 控制器没有 active

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

正常情况下应看到 `joint_state_broadcaster`、`arm_controller` 和
`gripper_controller` 为 `active`。不要同时启动 `mock_control.launch.py` 和
MoveIt 的 `demo.launch.py`，两者都会创建 controller_manager。

## RGB-D 节点看不到图像

`rgbd_demo.launch.py` 只发布图像，不会自动打开窗口。保持 Launch 运行，在另一个已 source
工作空间的终端启动：

```bash
ros2 run rqt_image_view rqt_image_view
```

选择 `/camera/color/image_raw` 或 `/camera/depth/image_raw`。如果找不到命令，安装：

```bash
sudo apt install ros-humble-rqt-image-view
```

本仓库图像发布端采用传感器数据 QoS。先检查话题频率和编码：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic echo /camera/depth/image_raw --once
```

深度图编码为 `32FC1`，单位是米；RGB 图编码为 `bgr8`。自定义订阅者应使用
兼容的传感器 QoS，否则即使话题名正确也可能收不到消息。

## 数值 IK 没有收敛

SO-101 主体只有 5 个关节，且工作空间有限。先使用默认目标，再逐步修改位置；目标
过远、位于基座内部或靠近奇异位形时都可能失败。本例 IK 只约束末端位置，不保证任意姿态。

## MoveIt 规划失败

依次检查：目标是否在关节限制内、场景障碍物是否与机器人重叠、规划组是否为 `arm`、
控制器是否 active。虚拟抓放是逻辑附着，不模拟夹爪与方块的接触力。

## 虚拟抓放时方块与夹爪错位或突然跳动

先确认终端使用的是本次构建结果，再观察任务启动日志：

```bash
source install/setup.bash
ros2 run so101_tasks virtual_pick_place
```

正常应打印“抓取/放置几何误差”，两项都应小于 2 mm。方块坐标、抓放关节姿态和方块相对
`tool0` 的偏移统一放在 `so101_tasks/task_geometry.py`，修改其中任意一项时必须同步求解另外两项。
不要将方块直接附着到 `gripper_link` 的任意偏移；本项目的 `tool0` 与官方
`gripper_frame_link` 重合，方块中心偏移由官方夹爪网格确定。

如果日志数值正确但 RViz 仍显示旧位置，通常是旧进程或未重新 source。关闭旧的 MoveIt/RViz，
重新构建 `so101_tasks`，在两个新终端中分别 source 后再启动演示和任务。

## RViz末端只能前后拖动或目标变红

SO101 主体只有5自由度，无法普遍满足任意位置和完整姿态组成的6维目标。本项目已在
`so101_moveit_config/config/kinematics.yaml` 中设置 `position_only_ik: true`，让交互拖动
主要约束末端 XYZ。

更新配置后重新构建并加载环境：

```bash
colcon build --symlink-install --packages-select so101_moveit_config
source install/setup.bash
ros2 launch so101_moveit_config demo.launch.py
```

在 RViz 中选择 `arm`、工具栏 `Interact`，先回到 `ready` 再小范围平移。目标仍然变红时，
检查工作空间、关节限制、自碰撞和场景碰撞。

## 无桌面环境如何验证

```bash
ros2 launch so101_control mock_control.launch.py use_rviz:=false
ros2 launch so101_moveit_config demo.launch.py use_rviz:=false
```

无桌面模式仍可通过命令行查看控制器、Action 和 TF。

## Matplotlib坐标轴或标题中文乱码

`so101_math` 会自动从系统字体中选择 Noto Sans CJK、思源黑体、微软雅黑、黑体、文泉驿或
Droid Sans Fallback，并关闭容易缺字的 Unicode 负号。运行绘图样例时会打印实际字体：

```bash
ros2 run so101_math pid_parameter_tuning --no-show --save /tmp/pid.png
ros2 run so101_math sample_workspace --no-show --save /tmp/workspace.png
```

如果输出“未找到中文字体”，Ubuntu 可安装中文 Noto 字体后重新运行：

```bash
sudo apt update
sudo apt install fonts-noto-cjk
```

旧的 Matplotlib 字体缓存仍可能保留错误结果。必要时删除用户目录下的 Matplotlib 字体缓存，
再重新启动终端。项目代码不硬编码某一个字体文件路径，因此可在不同系统选择已有字体。
