# so101_moveit_config

```bash
ros2 launch so101_moveit_config demo.launch.py
```

启动后检查：

```bash
ros2 control list_controllers
ros2 action list | grep trajectory
ros2 topic echo /joint_states --once
```

在 RViz 中选择 `arm`，优先测试 `home`、`ready`、`carry` 命名姿态。SO-101 主体为
5 自由度，因此本项目为 `arm` 启用了 `position_only_ik`：拖动末端时主要求解 XYZ 位置，
姿态不作为严格目标。仍然无法到达工作空间外、关节越界或碰撞位置。

建议先切换到 `ready`，选择 RViz 工具栏的 `Interact`，再从小范围平移开始拖动。
