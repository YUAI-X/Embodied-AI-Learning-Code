# 第2章：机器人学、ROS系统与控制
> 作者：宇哥的具身笔记


本目录是第2章配套代码入口。原理、公式、逐步实验、预期输出、参数练习和故障排查统一放在
[第2章学习指南](../../docs/chapter02/LEARNING_GUIDE.md)中，本页只提供代码导航和快速命令。

## 代码包

| 包 | 作用 |
|---|---|
| `so101_description` | 开源SO101结构、URDF/Xacro、相机Frame、TF与RViz |
| `so101_math` | PID参数、坐标变换、FK、IK、工作空间与安全过滤 |
| `so101_interfaces` | 教学用Action和Service接口 |
| `so101_ros2_basics` | Topic、Service、Action、Parameter、QoS与RGB-D消息 |
| `so101_control` | ros2_control Mock Hardware与控制器 |
| `so101_moveit_config` | MoveIt 2规划与虚拟执行配置 |
| `so101_tasks` | 命名姿态和虚拟抓取、搬运、放置任务 |

第2章与第3章分目录存放，但属于同一个 ROS 2 工作空间。第3章会直接复用本章的 SO101
模型、相机坐标系、TF 与 Mock Hardware。

## 推荐学习顺序

```bash
# 2.1 机器人本体与PID参数
ros2 run so101_math inspect_so101_model
ros2 run so101_math pid_parameter_tuning --kp 3.0 --ki 0.0 --kd 0.2

# 2.2 坐标系与位姿
ros2 run so101_math pose_representation_demo
ros2 launch so101_ros2_basics coordinate_demo.launch.py

# 2.3 正逆运动学
ros2 run so101_math planar_2r_fk_ik
ros2 run so101_math so101_forward_kinematics
ros2 run so101_math so101_numerical_ik --x 0.43 --y 0.08 --z 0.11

# 2.4 ROS 2与RGB-D
ros2 launch so101_ros2_basics basics_demo.launch.py
ros2 launch so101_ros2_basics rgbd_demo.launch.py

# 2.5 URDF、TF与RViz
ros2 launch so101_description display.launch.py

# 2.6 ros2_control与MoveIt 2
ros2 launch so101_moveit_config demo.launch.py
```

完成单项实验后，可在 MoveIt 启动期间运行章末任务：

```bash
ros2 run so101_tasks virtual_pick_place
```

## 测试

```bash
colcon test --packages-select \
  so101_description so101_interfaces so101_math so101_ros2_basics \
  so101_control so101_moveit_config so101_tasks
colcon test-result --verbose
```

## 继续阅读

- [第2章完整学习指南](../../docs/chapter02/LEARNING_GUIDE.md)
- [第3章代码入口](../chapter03/README.md)
- [项目总README](../../README.md)
- [故障排查](../../docs/chapter02/TROUBLESHOOTING.md)
