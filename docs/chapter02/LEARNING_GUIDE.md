# 第2章详细学习指南：机器人学、ROS系统与控制

本章以开源 LeRobot SO101 机械臂为统一案例，从机械臂关节与 PID 参数开始，依次学习
坐标变换、正逆运动学、ROS 2 通信、RGB-D 消息、URDF/TF、ros2_control、MoveIt 2，最后
完成一个不依赖真机和 Gazebo 的虚拟抓取—搬运—放置任务。

本章默认采用 TheRobotStudio 官方 SO101 follower new-calibration 结构与 STL。课程代码只在
外层增加 `tool0`、不可见相机坐标系和 Mock Hardware接口，不修改官方机械臂外形。

## 1. 学习目标与完成标准

完成本章后，应能够回答并验证以下问题：

- SO101 有哪些可动关节？关节名、顺序、单位和限制为什么必须统一？
- 调节 `Kp、Ki、Kd` 后，响应速度、稳态误差和超调怎样变化？
- 如何用齐次矩阵表示位姿，并正确组合 `T_base_camera` 与 `T_camera_object`？
- FK、IK、雅可比、工作空间和奇异性之间是什么关系？
- Topic、Service、Action、Parameter 和 Launch 分别适合什么任务？
- RGB 图、深度图、`CameraInfo`、时间戳、Frame 和 QoS 为什么必须配套？
- URDF、`robot_state_publisher`、`joint_states` 和 TF Tree 如何共同驱动 RViz 模型？
- ros2_control 控制器与 MoveIt 2 规划器各自负责什么？
- 为什么虚拟抓放中的“附着物体”不等于真实夹持？

最终验收标准：

1. 能在 RViz 中显示与开源 SO101 一致的机械臂结构；
2. 能调节 PID 参数并解释曲线变化；
3. 能完成相机目标到 `base_link` 的 TF2 转换；
4. 能运行 SO101 FK、位置 IK 与工作空间采样；
5. 能检查 ROS 2 Action、Service、Parameter 和 RGB-D 话题；
6. 能启动 Mock Hardware 与 MoveIt 2，并完成虚拟抓放任务。

## 2. 目录与包职责

第2章代码位于 `src/chapter02`：

```text
chapter02/
├── so101_description/       官方SO101结构、Xacro、相机Frame与RViz
├── so101_math/              PID、坐标变换、FK、IK、工作空间
├── so101_interfaces/        MoveJoints.action与ResetRobot.srv
├── so101_ros2_basics/       ROS 2通信、TF2、RGB-D与QoS
├── so101_control/           ros2_control Mock Hardware
├── so101_moveit_config/     SRDF、OMPL、KDL、控制器与RViz配置
└── so101_tasks/             命名姿态、Planning Scene与虚拟抓放
```

第3章位于 `src/chapter03`，但两章属于同一个 colcon 工作空间。第3章不会复制模型，而是复用
本章的 `so101_description` 与 `so101_control`。

## 3. 环境准备

### 3.1 推荐环境

- Ubuntu 22.04
- ROS 2 Humble Desktop
- MoveIt 2、ros2_control、RViz 2
- Python 3.10
- NumPy、Matplotlib、OpenCV 与 CvBridge

本章不要求真机、Gazebo 或 GPU。Mock Hardware 只模拟控制接口和位置反馈，不模拟质量、
摩擦、碰撞、接触或舵机通信。

### 3.2 安装依赖

```bash
source /opt/ros/humble/setup.bash
cd ~/so101_ros2_learning_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3.3 构建

```bash
source /opt/ros/humble/setup.bash
cd ~/so101_ros2_learning_ws
colcon build --symlink-install \
  --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

若电脑安装了 Anaconda，请保留两个系统 Python 参数。每次打开新终端，都要重新加载 ROS 2
与工作空间环境：

```bash
source /opt/ros/humble/setup.bash
source ~/so101_ros2_learning_ws/install/setup.bash
```

检查第2章包：

```bash
colcon list | grep so101
ros2 pkg prefix so101_description
ros2 pkg executables so101_math
```

## 4. 本章系统关系

```text
SO101 URDF/Xacro ── robot_description ─┬─ robot_state_publisher ── TF Tree ── RViz
                                      │
                                      └─ ros2_control_node
                                               │
                                     arm / gripper controllers
                                               │
MoveIt 2 ── 规划轨迹 ──────────────────────────┘
   │
   └─ Planning Scene：桌面、方块、附着物体

纯数学样例 ── 验证PID、位姿、FK、IK和工作空间
ROS基础样例 ── 验证Topic、Service、Action、Parameter、RGB-D与TF2
```

建议严格按 2.1→2.6 学习。遇到 MoveIt 问题时，先退回 2.5 检查模型与 TF，再退回 2.4
检查消息和 Action；不要直接在最上层任务中猜原因。

## 5. 2.1 机器人本体与控制基础

### 5.1 SO101关节组成

SO101 教学模型包含 5 个机械臂关节和 1 个夹爪关节：

```text
shoulder_pan
shoulder_lift
elbow_flex
wrist_flex
wrist_roll
gripper
```

机械臂算法统一使用前五个关节，顺序不能交换；角度统一使用弧度，位置统一使用米。

运行模型检查：

```bash
ros2 run so101_math inspect_so101_model
```

程序会从安装后的 Xacro 解析可动关节、类型、旋转轴和上下限。预期最后显示“5 个机械臂
关节 + 1 个夹爪关节”。

需要理解的边界：URDF 可以描述结构、惯性和关节限制，但不能单独告诉我们真实舵机精度、
负载能力、摩擦、回差和控制性能。

### 5.2 PID参数调节样例

本节样例的重点只是讲清楚 PID 参数可以调节，以及三个参数对曲线的主要影响；不要求学员
在这里实现真机控制器或整定理论。

```text
控制量 = Kp · 误差 + Ki · 误差积分 + Kd · 误差变化率
```

基础运行：

```bash
ros2 run so101_math pid_parameter_tuning \
  --kp 3.0 --ki 0.0 --kd 0.2
```

程序输出最大超调量和末段平均误差，并显示位置响应与控制量曲线。没有图形界面时：

```bash
ros2 run so101_math pid_parameter_tuning \
  --kp 3.0 --ki 0.0 --kd 0.2 \
  --no-show --save /tmp/pid.png
```

参数作用：

| 参数 | 增大后的主要趋势 | 过大时的常见现象 |
|---|---|---|
| `Kp` | 响应更快、误差减小 | 超调、振荡、控制量变大 |
| `Ki` | 消除恒定负载造成的稳态误差 | 积分累积、恢复慢、超调增大 |
| `Kd` | 增加阻尼、抑制快速变化 | 对测量噪声敏感、控制量抖动 |

推荐对比时一次只改一个参数：

```bash
ros2 run so101_math pid_parameter_tuning --kp 1.0 --ki 0.0 --kd 0.2
ros2 run so101_math pid_parameter_tuning --kp 6.0 --ki 0.0 --kd 0.2
ros2 run so101_math pid_parameter_tuning --kp 3.0 --ki 0.8 --kd 0.2
ros2 run so101_math pid_parameter_tuning --kp 3.0 --ki 0.0 --kd 0.8
```

记录每组参数的超调量与末段误差，不要只凭曲线“看起来更快”判断。这个简化二阶模型包含
阻尼、恒定负载、控制限幅和简单抗积分饱和，但它不代表 SO101 真机动力学。

### 5.3 关节命令安全过滤

```bash
ros2 run so101_math safe_joint_action
```

程序依次检查正常但变化过快、关节越界和包含 NaN 的目标，展示进入控制器前至少应验证：

- 关节名和顺序；
- 数组长度和有限数值；
- 位置上下限；
- 单步最大变化或速度限制；
- 时间长度是否为正。

安全过滤样例只用于教学，不能代替真机急停、限位开关、扭矩限制和硬件保护。

## 6. 2.2 坐标系、位姿表示与坐标转换

### 6.1 齐次变换

位姿由旋转矩阵 `R` 和平移向量 `t` 组成：

```text
T = [ R  t ]
    [ 0  1 ]
```

本项目使用 `T_A_B` 表示“把 B Frame 中的点转换到 A Frame”：

```text
p_base = T_base_camera · p_camera
T_base_object = T_base_camera · T_camera_object
```

矩阵乘法从右向左执行，旋转顺序通常不可以交换。

运行纯数学样例：

```bash
ros2 run so101_math pose_representation_demo
```

预期验证：

- 正交旋转矩阵的逆等于转置；
- `T · T⁻¹` 接近单位矩阵；
- Camera Frame 物体点可以转换到 Base Frame；
- 旋转矩阵可以表示成 `[x,y,z,w]` 四元数；
- 绕 X 后绕 Y 与绕 Y 后绕 X 的结果不同。

### 6.2 TF2坐标转换实验

```bash
ros2 launch so101_ros2_basics coordinate_demo.launch.py
```

该 Launch 启动：

- `static_transform_publisher`：发布 `base_link→camera_color_optical_frame`；
- `mock_object_publisher`：发布 `/detected_object_pose`；
- `pose_transformer`：转换并发布 `/object_pose_in_base`。

另开终端检查：

```bash
ros2 topic echo /detected_object_pose --once
ros2 topic echo /object_pose_in_base --once
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

第一条 Pose 的 `frame_id` 应为 `camera_color_optical_frame`，转换后应为 `base_link`。两者位置
数值不同是正常现象。

ROS 相机光学坐标通常约定 `+X` 向图像右侧、`+Y` 向图像下方、`+Z` 朝相机前方。不要把
`camera_link` 与 `camera_*_optical_frame` 混为同一方向。

### 6.3 常见错误

- 把 `T_base_camera` 误用成 `T_camera_base`；
- 四元数顺序写成 `[w,x,y,z]`，而 ROS 消息字段顺序是 `x,y,z,w`；
- 角度在度与弧度之间混用；
- 只修改 Pose 数值，没有正确填写 `header.frame_id`；
- TF Tree 中不存在从源 Frame 到目标 Frame 的完整链。

## 7. 2.3 正运动学、逆运动学与工作空间

### 7.1 从二维二连杆建立直觉

```bash
ros2 run so101_math planar_2r_fk_ik
```

默认目标通常会得到两组解，对应类似“肘上”和“肘下”的姿态。每组 IK 解都会再次用 FK
验证是否回到目标位置。

测试不可达目标：

```bash
ros2 run so101_math planar_2r_fk_ik --x 0.40 --y 0.40
```

二维解析解用于理解“一点可能有多组关节解”和“目标可能超出工作空间”，不代表 SO101
五关节 IK 可以直接套用同一公式。

### 7.2 SO101正运动学

```bash
ros2 run so101_math so101_forward_kinematics
```

程序按官方 URDF 的 Joint origin、RPY 和 axis 逐级相乘，输出：

- 关节顺序和关节角；
- `T_base_tool0` 齐次矩阵；
- 末端位置；
- 末端四元数。

自定义关节角：

```bash
ros2 run so101_math so101_forward_kinematics \
  --joints 0.2 -0.6 1.0 -0.4 0.1
```

FK 输入必须严格遵循 `[shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll]`。

### 7.3 阻尼最小二乘位置IK

```bash
ros2 run so101_math so101_numerical_ik \
  --x 0.43 --y 0.08 --z 0.11
```

程序输出是否收敛、迭代次数、最终位置误差、关节解和 FK 验证位置。算法使用：

```text
Δq = Jᵀ · (J·Jᵀ + λ²I)⁻¹ · Δx
```

阻尼 `λ` 用于缓解奇异位形附近伪逆数值过大；每次关节变化也会限幅，并裁剪到官方关节
范围。

本样例只约束末端位置，不严格约束完整姿态。SO101 主体是 5 自由度，对任意位置与任意
三维方向同时施加六维严格约束时，可能没有解。

第2章 MoveIt 配置也采用相同教学策略，在 `kinematics.yaml` 中为 `arm` 设置
`position_only_ik: true`。这样 RViz 拖动末端时主要求解 XYZ，而不会因为缺少第六个自由度
强制保持任意完整姿态。

尝试明显不可达目标：

```bash
ros2 run so101_math so101_numerical_ik --x 1.0 --y 0.0 --z 0.8
```

求解失败不一定是程序错误，也可能是目标不可达、关节限制、奇异性或初值导致。

### 7.4 工作空间与奇异性趋势

```bash
ros2 run so101_math sample_workspace --samples 1500
```

无桌面环境时：

```bash
ros2 run so101_math sample_workspace \
  --samples 3000 --no-show --save /tmp/so101_workspace.png
```

程序在关节限制内随机采样，通过 FK 得到末端点云，颜色使用雅可比条件数的对数表示奇异性
趋势。条件数较大意味着某些方向运动能力变弱，关节速度可能被放大；它不是碰撞检查结果。

## 8. 2.4 ROS 2核心概念与RGB-D消息

### 8.1 通信方式如何选择

| 机制 | 特点 | 本章案例 |
|---|---|---|
| Topic | 连续发布、发布者不等待结果 | `/joint_states`、RGB-D图像 |
| Service | 一问一答、适合短操作 | `/reset_robot` |
| Action | 耗时、反馈、结果、可取消 | `/move_joints` |
| Parameter | 节点运行参数，可查询和修改 | `kp、ki、kd` |
| Launch | 组合节点、参数与启动顺序 | `basics_demo.launch.py` |

### 8.2 Action、Service与Parameter综合实验

终端A：

```bash
ros2 launch so101_ros2_basics basics_demo.launch.py
```

终端B检查接口：

```bash
ros2 topic echo /joint_states --once
ros2 service list
ros2 action list
ros2 param list /pid_parameter_node
```

发送 Action：

```bash
ros2 run so101_ros2_basics move_joints_client
```

客户端会打印执行进度与最终结果。改变持续时间和目标角：

```bash
ros2 run so101_ros2_basics move_joints_client \
  --positions 0.2 -0.7 1.0 -0.3 0.2 \
  --duration 5.0
```

调用 Service：

```bash
ros2 service call /reset_robot \
  so101_interfaces/srv/ResetRobot \
  "{}"
```

查看并调整 PID Parameter：

```bash
ros2 param get /pid_parameter_node kp
ros2 param set /pid_parameter_node kp 5.0
ros2 param set /pid_parameter_node kp -1.0
```

负数参数应被节点拒绝。本节点只演示“参数可以声明、查询、修改和校验”，不会实时驱动 2.1
的仿真曲线或真机控制器。

查看自定义接口：

```bash
ros2 interface show so101_interfaces/action/MoveJoints
ros2 interface show so101_interfaces/srv/ResetRobot
```

### 8.3 RGB与深度图在哪一节学习

相机 RGB、深度图、`CameraInfo`、消息同步和 QoS 集中放在 2.4 ROS 2节点这一节；第3章再
使用这些标准消息完成点云、分割和位姿估计。

启动合成 RGB-D：

```bash
ros2 launch so101_ros2_basics rgbd_demo.launch.py
```

这个 Launch 负责发布和处理图像，但不会自动打开图像窗口。要看到画面，需要保持上述终端
运行，再打开第二个终端并启动图像查看器：

```bash
source /opt/ros/humble/setup.bash
source ~/so101_ros2_learning_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

在 `rqt_image_view` 顶部话题列表中选择：

```text
/camera/color/image_raw
```

预期看到深灰色背景、左右缓慢移动的红色方块，以及 `SO-101 RGB-D demo` 文字。也可以在
启动时直接指定彩色图话题：

```bash
ros2 run rqt_image_view rqt_image_view /camera/color/image_raw
```

查看深度图时，在同一界面选择：

```text
/camera/depth/image_raw
```

深度图是单通道浮点图，`rqt_image_view` 通常会自动归一化成灰度显示。颜色深浅只是显示
映射，实际数值应通过消息或代码读取：背景约为 `0.75 m`，移动方块约为 `0.45 m`。

如果系统没有图像查看器，先安装：

```bash
sudo apt update
sudo apt install ros-humble-rqt-image-view
```

默认发布：

```text
/camera/color/image_raw          bgr8
/camera/depth/image_raw          32FC1，单位米
/camera/color/camera_info
/camera/depth/camera_info
/object_point_camera             中心像素反投影三维点
```

检查：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /camera/depth/camera_info --once
ros2 topic echo /object_point_camera --once
```

默认图像频率约为 `15 Hz`。能在 `ros2 topic list` 中看到话题，只能说明话题名称已注册；
还需要通过 `topic hz`、`rqt_image_view` 和节点日志分别确认数据频率、可视内容与同步状态。

`rgbd_sync_node` 使用近似时间同步器，正常日志应显示中心深度与 RGB/Depth 时间差。合成发布器
给同一对图像相同时间戳，因此时间差应接近零。

中心像素反投影使用：

```text
X = (u-cx)·Z/fx
Y = (v-cy)·Z/fy
Z = depth[v,u]
```

本项目深度是 `32FC1` 米制数据。真实相机可能发布 `16UC1` 毫米或设备刻度，必须根据驱动
说明换算，不能只看数值大小猜单位。

### 8.4 QoS排错

图像发布器使用 Sensor Data QoS。运行故意不匹配的订阅者：

```bash
ros2 run so101_ros2_basics qos_mismatch_subscriber
ros2 topic info /camera/color/image_raw --verbose
```

当发布者与订阅者的 Reliability 等策略不兼容时，话题名存在也可能收不到任何消息。排错时
不要只运行 `ros2 topic list`，还要检查端点 QoS。

### 8.5 rosbag2记录与回放

```bash
ros2 bag record \
  /camera/color/image_raw \
  /camera/depth/image_raw \
  /camera/color/camera_info \
  /camera/depth/camera_info \
  /joint_states /tf /tf_static
```

停止后检查与回放：

```bash
ros2 bag info <bag目录>
ros2 bag play <bag目录>
```

记录视觉数据时要同时记录 `CameraInfo`、时间戳相关消息和 TF，否则只有图像而缺少内参与坐标
关系，后续很难复现实验。

## 9. 2.5 URDF、TF2与RViz

### 9.1 模型文件

| 文件 | 作用 |
|---|---|
| `so101.urdf.xacro` | 课程统一入口，组合官方模型与扩展 |
| `so101_official.urdf.xacro` | 官方SO101关节、惯性与网格结构 |
| `so101_camera.xacro` | 不可见相机Link和optical frame |
| `so101_ros2_control.xacro` | Mock Hardware命令与状态接口 |
| `meshes/*.stl` | 官方底座、舵机、臂、腕部与夹爪外形 |
| `so101_model.rviz` | RobotModel和TF显示配置 |

课程相机只添加坐标系，没有虚构相机外壳，因此 RViz 中机械臂主体仍保持开源 SO101 外形。
相机安装位置是教学示例，不能当作真机手眼标定结果。

### 9.2 显示模型

```bash
ros2 launch so101_description display.launch.py
```

拖动 Joint State Publisher GUI 滑块，观察机械臂和 TF 坐标轴变化。另开终端：

```bash
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo base_link tool0
ros2 run tf2_tools view_frames
```

模型显示链路：

```text
Xacro → robot_description
joint_states + robot_description → robot_state_publisher
robot_state_publisher → /tf与/tf_static
robot_description + TF → RViz RobotModel
```

### 9.3 URDF阅读顺序

建议按以下顺序阅读一个关节：

1. Parent Link 与 Child Link；
2. Joint type；
3. `origin xyz/rpy`；
4. `axis xyz`；
5. `limit lower/upper/velocity/effort`；
6. Link 的 visual、collision 与 inertial。

尝试修改 Joint origin 或 axis 前先复制原值。修改 Xacro 后重新构建并 source；错误轴方向可能
仍能显示模型，却会让运动学、TF 和规划全部产生错误结果。

### 9.4 常见显示问题

- 只有网格没有运动：检查 `/joint_states` 名称是否与 URDF 完全一致；
- RViz 报 No transform：检查 Fixed Frame 和 TF 链；
- 网格丢失：检查 `package://so101_description/meshes/...` 与安装规则；
- 模型散架：检查 Joint origin、RPY、axis 和父子 Link；
- 相机方向错误：区分普通 `camera_link` 与 ROS optical frame。

## 10. 2.6 ros2_control与MoveIt 2

### 10.1 ros2_control Mock Hardware

启动：

```bash
ros2 launch so101_control mock_control.launch.py
```

检查：

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 action list | grep controller
ros2 topic echo /joint_states --once
```

正常情况下，`joint_state_broadcaster`、`arm_controller` 和 `gripper_controller` 应处于
`active`。机械臂控制器接收五关节位置轨迹，夹爪使用 Gripper Action。

直接发送一段关节轨迹：

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll], points: [{positions: [0.2, -0.6, 1.0, -0.4, 0.0], time_from_start: {sec: 3}}]}}"
```

Mock Hardware 会把命令位置反馈为状态，但不会验证真实负载、碰撞、夹持力或 PID 效果。

### 10.2 MoveIt 2规划与执行

```bash
ros2 launch so101_moveit_config demo.launch.py
```

该 Launch 会启动 world→base 静态 TF、SO101 模型、ros2_control、Move Group 与 RViz。

在 RViz MotionPlanning 面板中：

1. Planning Group 选择 `arm`；
2. RViz 工具栏选择 `Interact`；
3. 先选择命名姿态 `home` 或 `ready`；
4. 使用末端彩色箭头做小范围平移；
5. 点击 Plan；
6. 检查轨迹和碰撞结果；
7. 再点击 Execute。

SO101 主体只有 5 自由度。交互式末端 Marker 同时约束任意位置与完整方向时，IK 可能无解。
本项目已设置 `position_only_ik: true`，因此主要约束末端 XYZ，姿态允许随关节解调整。该配置
只解决“5自由度无法普遍满足6维目标”的问题，不会绕过工作空间、关节限制、自碰撞和场景
碰撞。初学阶段仍应优先使用命名姿态，从 `ready` 开始做小范围拖动。

命令行发送命名姿态：

```bash
ros2 run so101_tasks move_named_pose --pose ready
ros2 run so101_tasks move_named_pose --pose home
```

可用姿态包括 `home、ready、pregrasp、grasp、carry、preplace、place`。其中
`pregrasp` 位于抓取点上方约 5 cm，`preplace` 位于放置点上方约 10 cm；不要把
`grasp` 与 `place` 当作任意关节角，它们和场景方块坐标是一组配套几何数据。

### 10.3 章末虚拟抓放

保持 MoveIt 终端运行，另开终端：

```bash
ros2 run so101_tasks virtual_pick_place
```

任务依次执行：

1. 向 Planning Scene 添加桌面和方块；
2. 打开夹爪并移动到 `ready`；
3. 移动到方块正上方的 `pregrasp`；
4. 只允许夹指接触方块，再下降到 `grasp`；
5. 将夹爪闭合到适合 35 mm 方块的开度，并把方块逻辑附着到 `tool0`；
6. 沿原接近方向抬升回 `pregrasp`；
7. 经过 `carry` 搬运；
8. 移动到放置点正上方 10 cm 的 `preplace`；
9. 下降到 `place`；
10. 在当前世界坐标解除附着、打开夹爪，再抬升到 `preplace`；
11. 返回 `home`。

场景方块中心、`grasp/place` 关节角，以及方块相对 `tool0` 的偏移统一定义在
`so101_tasks/task_geometry.py`。程序启动时会用官方 SO101 运动学做一次校验，正常日志应显示
抓取和放置误差均小于 2 mm。这样在附着和分离的消息边界，RViz 中的方块不会跳到另一个位置。

这里的“抓取”通过 MoveIt Planning Scene 附着物体表达，不包含接触、摩擦、夹持力和目标滑落。
它用于学习规划场景、控制器接口和任务顺序，不代表真实抓取成功率。

### 10.4 MoveIt配置关系

| 文件 | 作用 |
|---|---|
| `so101.srdf` | `arm`、`gripper`组、命名姿态、碰撞对 |
| `kinematics.yaml` | KDL运动学求解器配置 |
| `ompl_planning.yaml` | OMPL规划器与规划参数 |
| `joint_limits.yaml` | MoveIt速度和加速度限制 |
| `moveit_controllers.yaml` | MoveIt到ros2_control控制器映射 |
| `ros2_controllers.yaml` | 实际启动的控制器类型与关节列表 |

MoveIt 报控制器错误时，要同时检查后两个文件的控制器名和关节列表是否一致。

## 11. 建议的完整学习流程

第一次学习可按下面的验收顺序执行：

```text
1. inspect_so101_model          确认官方模型关节
2. pid_parameter_tuning        对比Kp/Ki/Kd
3. pose_representation_demo    验证齐次变换
4. coordinate_demo.launch.py   验证TF2
5. planar_2r_fk_ik             理解多解与不可达
6. so101_forward_kinematics    对应官方URDF
7. so101_numerical_ik          验证位置IK
8. basics_demo.launch.py       对比ROS通信方式
9. rgbd_demo.launch.py         学习RGB、Depth和CameraInfo
10. display.launch.py          验证模型与TF
11. mock_control.launch.py     验证控制器
12. demo.launch.py             MoveIt规划与执行
13. virtual_pick_place         完成章末任务
```

每一步出错时先解决当前层，不要带着错误继续进入下一层。

## 12. 常见问题

### 找不到包或可执行程序

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon list | grep so101
```

如果 `colcon list` 看不到包，确认命令在工作空间根目录运行，且源码位于 `src/chapter02`。

### Matplotlib不能打开窗口

服务器或容器没有图形界面时，使用 `--no-show --save /tmp/result.png`。

若窗口能打开但中文显示为方框，程序会自动选择系统中可用的中文字体，并在终端打印字体名。
没有可用字体时安装 `fonts-noto-cjk`，详细步骤见[故障排查](TROUBLESHOOTING.md)。

### Action Goal被拒绝

检查关节名、顺序、五个位置值、关节范围和正的 duration。Action Server 会主动拒绝 NaN、
越界或顺序错误的目标。

### 有话题名但没有RGB-D数据

检查 QoS：

```bash
ros2 topic info /camera/color/image_raw --verbose
```

同时检查节点是否使用系统 Python、CvBridge 是否能导入，以及图像发布频率是否大于零。

### TF查不到目标Frame

确认 `robot_state_publisher` 或静态 TF 发布器正在运行，源 Frame 名与消息 `header.frame_id`
一致，并检查 `base_link` 到源 Frame 是否有连续链路。

### IK不收敛

先用 FK 和工作空间图检查目标是否合理。再考虑关节限制、奇异性与初值。SO101 五自由度本来
就不能保证所有六维目标可达。

### MoveIt只能Plan不能Execute

检查三个控制器是否 active、`moveit_controllers.yaml` 与 `ros2_controllers.yaml` 名称是否一致，
以及 FollowJointTrajectory Action 是否存在。

### RViz末端只能沿少数方向拖动

确认 `so101_moveit_config/config/kinematics.yaml` 中包含：

```yaml
position_only_ik: true
```

修改后必须重新构建、source 并完全重启 MoveIt。操作时选择 `arm` 规划组与 `Interact` 工具，
先切换到 `ready`，再做小范围平移。即使使用仅位置 IK，工作空间外、关节越界、奇异位形或
碰撞目标仍会显示为无效，不能通过拖动到达。

### RViz模型与真实SO101外形不一致

确认加载的是 `so101_description/urdf/so101.urdf.xacro`，其主体应包含官方
`so101_official.urdf.xacro` 和官方 STL。不要用早期教学简化几何替代主体模型。

## 13. 测试与课后练习

运行第2章测试：

```bash
colcon test --packages-select \
  so101_description so101_interfaces so101_math so101_ros2_basics \
  so101_control so101_moveit_config so101_tasks
colcon test-result --verbose
```

建议练习：

1. 2.1：分别只改变 `Kp、Ki、Kd`，整理超调量和末段误差表；
2. 2.1：给安全过滤增加 duration 和最大速度检查；
3. 2.2：增加 `object_frame`，用 TF2 验证两种变换路径一致；
4. 2.3：比较二维 IK 的两组解，解释同一目标为何对应不同关节姿态；
5. 2.3：为数值 IK 增加不同初值，比较解和迭代次数；
6. 2.4：取消 Action Goal，观察取消反馈与最终状态；
7. 2.4：修改 RGB 和 Depth 时间戳差，观察同步器阈值；
8. 2.5：在 RViz 只显示 TF，手工画出 `base_link→tool0→camera` 关系；
9. 2.6：增加一个 Planning Scene 障碍物，比较规划路径；
10. 综合：把第3章 `target_pose_base` 接到 MoveIt，只进行可达性与碰撞检查，不直接执行。

学习记录应至少包含运行命令、参数、预期结果、实际结果和失败原因。对于机器人系统，“命令
没有报错”不是完整验收；必须同时确认单位、Frame、关节顺序、限制、控制器状态和碰撞结果。
