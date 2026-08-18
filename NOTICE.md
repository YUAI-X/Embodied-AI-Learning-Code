# 第三方来源与修改说明

本仓库的 SO-101 教学模型参考：

- 项目：TheRobotStudio/SO-ARM100
- 地址：https://github.com/TheRobotStudio/SO-ARM100
- 上游模型：`Simulation/SO101/so101_new_calib.urdf`
- 上游许可证：Apache License 2.0

本仓库直接包含该目录下 Apache-2.0 许可的官方 STL 网格，并将
`so101_new_calib.urdf` 保存为 `so101_official.urdf.xacro`。只进行了以下适配：

- 将网格路径改为 ROS 2 `package://so101_description/meshes/...`；
- 省略根链接惯性，避免 KDL 根惯性警告；
- 在外层 Xacro 添加课程使用的 `tool0`、RGB-D 坐标系和 Mock Hardware；
- 保留官方关节名称、父子结构、origin、axis、限制和其余惯性参数。

官方 STL 与原始结构用于正确显示 SO-101 follower 外形。课程扩展仍不代表真机驱动或标定。
