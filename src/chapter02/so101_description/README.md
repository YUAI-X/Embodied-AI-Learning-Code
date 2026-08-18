# so101_description

该包以 TheRobotStudio 官方 SO-101 follower new-calibration URDF 和 STL 为主体，
并叠加课程需要的相机坐标系、`tool0` 与 ros2_control Mock Hardware 接口。

```bash
ros2 launch so101_description display.launch.py
```

主要文件：

- `so101.urdf.xacro`：课程入口与扩展层；
- `so101_official.urdf.xacro`：官方 CAD 导出的关节结构和惯性参数；
- `meshes/`：官方底座、舵机、大小臂、腕部和夹爪 STL；
- `so101_camera.xacro`：腕部 RGB-D 相机及 optical frame；
- `so101_ros2_control.xacro`：Mock Hardware 接口；
- `so101_model.rviz`：RobotModel 和 TF 显示配置。

官方主体采用 new calibration：各关节虚拟零位位于关节范围中点。本仓库仍不提供真机驱动，
课程相机只添加不可见坐标系，不改变官方外形；这些坐标仍不应用作真机标定。
