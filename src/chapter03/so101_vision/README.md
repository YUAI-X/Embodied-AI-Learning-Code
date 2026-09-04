# so101_vision
> 作者：宇哥的具身笔记


第3章纯视觉算法包，只处理 NumPy 数组和齐次变换矩阵，可脱离 ROS 通信单独学习和测试。

## 主要内容

| 文件 | 内容 |
|---|---|
| `camera_geometry.py` | 相机内参、投影、反投影、Depth转点云 |
| `calibration_demo.py` | 合成棋盘格相机标定 |
| `hand_eye_demo.py` | Eye-in-Hand手眼标定 |
| `pointcloud.py` | 体素、离群点、RANSAC、聚类、PCA OBB和ICP |
| `segmentation.py` | 文本颜色Mask与目标点云 |
| `pose_estimation.py` | RANSAC+PnP、ADD和ADD-S |

## 运行

```bash
ros2 run so101_vision camera_model_demo
ros2 run so101_vision calibration_demo
ros2 run so101_vision hand_eye_demo
ros2 run so101_vision pointcloud_demo
ros2 run so101_vision segmentation_demo --prompt "红色杯子"
ros2 run so101_vision pnp_pose_demo
```

## 测试

```bash
colcon test --packages-select so101_vision
colcon test-result --verbose
```

各算法的原理、参数实验和预期结果见
[第3章学习指南](../../../docs/chapter03/LEARNING_GUIDE.md)。真实 Grounding DINO 与 SAM
属于选做内容，见[可选模型说明](../../../docs/chapter03/OPTIONAL_MODELS.md)。
