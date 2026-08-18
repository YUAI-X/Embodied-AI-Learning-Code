# so101_interfaces

本包集中定义第 2.4 节使用的自定义 ROS 2 接口，避免把接口定义和节点逻辑混在一起。

## MoveJoints.action

目标包含关节名、目标角度和持续时间；反馈返回完成百分比；结果说明动作是否成功。
它用于演示 Action 适合“耗时、可反馈、可取消”的任务。

```bash
ros2 interface show so101_interfaces/action/MoveJoints
```

## ResetRobot.srv

请求为空，调用后立即回到 home；响应返回是否成功和文字说明。它用于说明 Service
更适合“一问一答”的短操作，而不是持续反馈的运动过程。

```bash
ros2 interface show so101_interfaces/srv/ResetRobot
```

接口文件修改后需要重新执行 `colcon build`，并重新加载 `install/setup.bash`。
