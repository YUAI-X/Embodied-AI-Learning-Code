# so101_tasks：MoveIt 虚拟抓放

本包用 ROS 2 Humble 的标准 Action 和 Planning Scene 接口，演示官方 SO101 模型的命名姿态、
碰撞场景、夹爪控制和虚拟抓放。它不依赖 Gazebo 或真机，适合先理解任务流程。

## 1. 启动

在工作空间根目录构建并加载环境：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

终端 A 启动 MoveIt 和 RViz：

```bash
ros2 launch so101_moveit_config demo.launch.py
```

终端 B 重新加载环境后运行完整任务：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run so101_tasks virtual_pick_place
```

也可以单独测试一个命名姿态：

```bash
ros2 run so101_tasks move_named_pose --pose ready
ros2 run so101_tasks move_named_pose --pose grasp
ros2 run so101_tasks move_named_pose --list
```

## 2. 任务流程

完整任务按以下顺序运行：

```text
添加桌面和方块
  → ready
  → pregrasp（抓取点上方 5 cm）
  → grasp
  → 闭合夹爪并附着方块
  → pregrasp
  → carry
  → preplace（放置点上方 10 cm）
  → place
  → 分离方块、打开夹爪并抬升
  → home
```

靠近方块前，程序只临时允许方块和夹爪接触；机械臂其他 Link 仍执行碰撞检查。方块附着到
`tool0` 后随机械臂运动，释放时在准确的放置世界坐标重新加入场景。

## 3. 为什么方块不会在抓取时跳位

以下三类数据必须属于同一套坐标几何：

- Planning Scene 中方块中心的世界坐标；
- `grasp` 和 `place` 的五关节角；
- 方块中心相对 `tool0` 的固定偏移。

它们统一定义在 `so101_tasks/task_geometry.py`。`tool0` 与官方 SO101 URDF 的
`gripper_frame_link` 重合；35 mm 方块放在官方固定指和活动指之间。程序启动时会通过正运动学
计算抓取、放置误差，任一误差达到 2 mm 就停止任务，避免带着错误坐标继续演示。

夹爪开度 `GRIPPER_HOLD = 0.24 rad` 是根据官方夹爪结构给出的教学近似值。本例只是 MoveIt
逻辑附着，不模拟接触力、摩擦、挤压或物体滑落。

## 4. 主要文件

| 文件 | 学习重点 |
|---|---|
| `task_geometry.py` | 抓放姿态、场景坐标、tool0 偏移和几何自检 |
| `move_group_client.py` | 构造并发送 MoveGroup Action 目标 |
| `planning_scene_client.py` | 添加物体、碰撞许可、附着与分离 |
| `gripper_client.py` | 发送 GripperCommand Action |
| `virtual_pick_place.py` | 串联完整任务和错误处理 |
| `config/task_poses.yaml` | 便于阅读和实验的姿态数据副本 |

## 5. 验证与常见问题

正常启动时应看到类似日志：

```text
抓取/放置几何误差：0.00 / 0.00 mm
```

数值允许有很小的浮点误差，但必须小于 2 mm。如果修改了方块位置或关节角，请同步更新
`task_geometry.py` 中的配套数据并运行测试：

```bash
colcon test --packages-select so101_tasks
colcon test-result --verbose
```

若 RViz 仍显示旧的抓放位置，关闭残留的 MoveIt/RViz 进程，重新构建，并确保每个新终端都执行
`source install/setup.bash`。更多说明见仓库的 `docs/chapter02/TROUBLESHOOTING.md`。
