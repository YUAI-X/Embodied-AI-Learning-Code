# so101_ros2_basics

## Topic、Service、Action 与 Parameter

```bash
ros2 launch so101_ros2_basics basics_demo.launch.py
ros2 run so101_ros2_basics move_joints_client
ros2 service call /reset_robot so101_interfaces/srv/ResetRobot '{}'
ros2 param set /pid_parameter_node kp 5.0
```

## RGB-D

```bash
ros2 launch so101_ros2_basics rgbd_demo.launch.py
```

另开终端查看与检查：

```bash
ros2 run rqt_image_view rqt_image_view /camera/color/image_raw
ros2 topic hz /camera/color/image_raw
ros2 topic echo /object_point_camera --once
```

Launch 只发布图像，显示窗口需要另开终端运行 `rqt_image_view`。深度图 encoding 为
`32FC1`，单位是米。可单独运行
`qos_mismatch_subscriber` 观察 QoS 不兼容警告。

## TF2

```bash
ros2 launch so101_ros2_basics coordinate_demo.launch.py
ros2 topic echo /object_pose_in_base
```

## rosbag2

```bash
ros2 bag record /camera/color/image_raw /camera/depth/image_raw \
  /camera/color/camera_info /camera/depth/camera_info /joint_states /tf /tf_static
```
