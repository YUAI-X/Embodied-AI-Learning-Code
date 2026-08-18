# 第3章：机器人视觉与三维感知

本目录是第3章配套代码入口。原理、公式、逐步实验、预期输出、参数分析和故障排查统一放在
[第3章学习指南](../../docs/chapter03/LEARNING_GUIDE.md)中，本页只提供代码导航和快速命令。


## 代码包

| 包 | 作用 |
|---|---|
| `so101_vision` | 相机模型、标定、点云、目标分割和6D位姿等纯算法 |
| `so101_vision_ros` | RGB-D消息、时间同步、PointCloud2、TF2和RViz |

第3章与第2章分目录存放，但属于同一个 ROS 2 工作空间。综合实验直接复用第2章的 SO101
URDF、相机 TF 与 Mock Hardware。

## 推荐学习顺序

```bash
ros2 run so101_vision camera_model_demo
ros2 run so101_vision calibration_demo
ros2 run so101_vision hand_eye_demo
ros2 run so101_vision pointcloud_demo
ros2 run so101_vision segmentation_demo --prompt "红色杯子"
ros2 run so101_vision pnp_pose_demo
```

完成单项算法后启动综合实验：

```bash
ros2 launch so101_vision_ros vision_demo.launch.py
```

切换目标或关闭 RViz：

```bash
ros2 launch so101_vision_ros vision_demo.launch.py \
  prompt:="blue box" \
  use_rviz:=false
```

## 测试

```bash
colcon test --packages-select so101_vision so101_vision_ros
colcon test-result --verbose
```

## 继续阅读

- [第3章完整学习指南](../../docs/chapter03/LEARNING_GUIDE.md)
- [Grounding DINO与SAM选做说明](../../docs/chapter03/OPTIONAL_MODELS.md)
- [纯算法包README](so101_vision/README.md)
- [ROS 2视觉包README](so101_vision_ros/README.md)
