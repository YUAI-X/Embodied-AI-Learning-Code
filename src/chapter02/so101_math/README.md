# so101_math
> 作者：宇哥的具身笔记


纯 Python/NumPy 教学样例，不需要启动其他 ROS 2节点或连接机械臂。

```bash
ros2 run so101_math inspect_so101_model
ros2 run so101_math pid_parameter_tuning --kp 3 --ki 0 --kd 0.2
ros2 run so101_math pose_representation_demo
ros2 run so101_math so101_forward_kinematics
ros2 run so101_math so101_numerical_ik --x 0.43 --y 0.08 --z 0.11
```

所有角度统一使用弧度，位置统一使用米。

PID与工作空间图会自动选择系统中的中文字体，并在终端打印实际字体名；如果未找到字体，
请按[故障排查](../../../docs/chapter02/TROUBLESHOOTING.md)安装 `fonts-noto-cjk`。
