# so101_vision_ros

第3章 ROS 2集成包，负责把 `so101_vision` 算法接入标准相机消息、时间同步、PointCloud2、
TF2 和 RViz。

## 节点

| 节点 | 输入与输出 |
|---|---|
| `synthetic_rgbd_publisher` | 发布合成RGB、米制Depth和CameraInfo |
| `rgbd_pointcloud_node` | Depth + CameraInfo → `/vision/scene_cloud` |
| `target_localizer_node` | RGB-D + Prompt → Mask、目标点云和Base Frame Pose |

节点参数位于 `config/vision.yaml`，综合启动文件是 `launch/vision_demo.launch.py`。

## 运行

```bash
ros2 launch so101_vision_ros vision_demo.launch.py
```

指定目标：

```bash
ros2 launch so101_vision_ros vision_demo.launch.py prompt:="green part"
```

检查最终目标位姿：

```bash
ros2 topic echo /vision/target_pose_base --once
```

消息的 `frame_id` 应为 `base_link`。

## 核心参数

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `fps` | 5.0 | 合成相机频率 |
| `pixel_stride` | 4 | 场景点云采样步长 |
| `max_depth_m` | 1.2 | 最大有效深度 |
| `erosion_pixels` | 2 | Mask边缘腐蚀宽度 |
| `output_frame` | `base_link` | 目标Pose输出坐标系 |

消息流、TF关系、真实相机接入和故障排查见
[第3章学习指南](../../../docs/chapter03/LEARNING_GUIDE.md)。
