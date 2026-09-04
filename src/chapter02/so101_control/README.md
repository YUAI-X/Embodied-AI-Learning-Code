# so101_control
> 作者：宇哥的具身笔记


启动不依赖 Gazebo 的 ros2_control Mock Hardware：

```bash
ros2 launch so101_control mock_control.launch.py
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 action list
```

直接发送一段关节轨迹：

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll], points: [{positions: [0.2, -0.6, 1.0, -0.4, 0.0], time_from_start: {sec: 3}}]}}"
```

Mock Hardware 会把命令位置作为状态反馈，但不会模拟力、碰撞、摩擦和 PID。
