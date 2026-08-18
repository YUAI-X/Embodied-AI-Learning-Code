# 第3章详细学习README：机器人视觉与三维感知

本章以开源 LeRobot SO101 机械臂为载体，从一张 RGB 图和一张深度图出发，逐步完成
相机建模、标定、三维点云、目标分割、6D 位姿估计，以及目标从 Camera Frame 到 SO101
`base_link` 的坐标转换。

本章不是简单展示几个 API。每个样例都包含以下四层内容：

1. 能独立运行的最小算法；
2. 可重复的合成输入和明确的预期输出；
3. ROS 2 消息、时间同步与 TF2 接口；
4. 从教学数据迁移到真实 RGB-D 相机时需要替换的部分。

## 1. 学习目标与完成标准

完成本章后，应能够回答并实际验证以下问题：

- 相机内参 `fx、fy、cx、cy` 分别表示什么？图像缩放后为什么必须修改内参？
- 怎样用深度值把像素 `(u,v)` 恢复为相机坐标系三维点 `(X,Y,Z)`？
- `sensor_msgs/Image`、`CameraInfo` 和 `PointCloud2` 之间是什么关系？
- 相机标定与手眼标定分别求解哪个变换？
- 体素、离群点过滤、RANSAC 和欧式聚类各自解决什么问题？
- Grounding DINO 和 SAM 在感知流水线中分别承担什么职责？
- PnP、ICP、ADD 和 ADD-S 应在什么场景使用？
- 如何计算 `T_base_object`，避免把 Camera Frame 的坐标直接交给机械臂？

最终验收标准是运行综合 Launch 后能够收到：

```text
/camera/color/image_raw          RGB图
/camera/depth/image_raw          深度图，32FC1，单位米
/camera/depth/camera_info        相机内参
/vision/scene_cloud              整幅深度图生成的场景点云
/vision/target_mask              文本目标Mask
/vision/target_cloud             Mask筛选出的目标点云
/vision/target_pose_camera       Camera Frame中的目标中心
/vision/target_pose_base         SO101 base_link中的目标中心
```

## 2. 为什么第2章与第3章分目录

推荐分目录，但不拆成两个工作空间：

```text
src/
├── chapter02/
│   ├── so101_description/      SO101官方结构、相机Frame、TF与RViz
│   ├── so101_control/          ros2_control Mock Hardware
│   └── ...                     第2章其他包
└── chapter03/
    ├── so101_vision/           纯NumPy/OpenCV/SciPy算法
    └── so101_vision_ros/       ROS 2消息、同步、TF与RViz
```

这样组织有三个好处：章节代码边界清楚；视觉算法可以脱离 ROS 单独调试；第三章仍能直接按
ROS 包名复用第二章的 SO101 URDF、`robot_state_publisher` 和控制器。不要在第3章复制一套
SO101 模型，否则两份 URDF 很容易逐渐不一致。

## 3. 环境准备

### 3.1 推荐环境

- Ubuntu 22.04
- ROS 2 Humble Desktop
- Python 3.10
- NumPy、OpenCV、SciPy
- `cv_bridge`、`message_filters`、`sensor_msgs_py`、TF2、RViz 2

基础课程不依赖 GPU、Open3D、真实相机、Grounding DINO 或 SAM。真实大模型属于 3.4 的
进阶选做内容。

### 3.2 安装依赖

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

检查关键依赖：

```bash
/usr/bin/python3 -c "import cv2, numpy, scipy; print(cv2.__version__)"
ros2 pkg prefix cv_bridge
ros2 pkg prefix tf2_ros
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

如果使用 Anaconda，必须保留两个 Python 参数，避免 ROS 2 Humble 加载到版本不兼容的
Conda Python。每次打开新终端，都要重新执行两个 `source` 命令。

检查两个第三章包是否被发现：

```bash
colcon list | grep so101_vision
ros2 pkg prefix so101_vision
ros2 pkg prefix so101_vision_ros
```

## 4. 先认识本章数据流

综合实验的数据关系如下：

```text
合成或真实RGB ───────────────┐
                             ├─ 文本目标分割 ─ Mask ─┐
合成或真实Depth ─┬─ 全场反投影 ─ scene_cloud         ├─ target_cloud
                 │                                   │
CameraInfo ──────┴─ 提供K和图像尺寸 ─────────────────┘
                                                       │
                                                       ▼
                                            target_pose_camera
                                                       │ TF2
SO101 URDF + joint_states ── TF Tree ──────────────────┘
                                                       ▼
                                             target_pose_base
```

`so101_vision` 只处理数组与矩阵；`so101_vision_ros` 负责把 ROS 消息转换为数组，并把结果
重新发布为标准消息。调试时应先验证纯算法，再检查消息编码、时间戳和 TF。

## 5. 3.1 相机成像模型与RGB-D相机

### 5.1 需要理解的公式

针孔模型把 Camera Frame 中的三维点投影到像素：

```text
u = fx · X / Z + cx
v = fy · Y / Z + cy
```

已知像素与深度 `Z` 时可反投影：

```text
X = (u - cx) · Z / fx
Y = (v - cy) · Z / fy
```

在 ROS 相机光学坐标系中，通常 `+Z` 朝相机前方、`+X` 向图像右侧、`+Y` 向图像下方。
像素写作 `(u,v)`，NumPy 图像访问却是 `image[v,u]`，两者顺序不能混用。

### 5.2 对应代码

| 文件 | 学习内容 |
|---|---|
| `so101_vision/camera_geometry.py` | 内参类、投影、反投影、Depth转点云 |
| `so101_vision/synthetic_data.py` | 合成RGB、Depth、目标Mask和无效深度 |
| `so101_vision/camera_model_demo.py` | 像素与三维点往返验证 |
| `so101_vision_ros/synthetic_rgbd_publisher.py` | 发布Image与CameraInfo |
| `so101_vision_ros/rgbd_pointcloud_node.py` | Depth与CameraInfo同步、发布PointCloud2 |

### 5.3 实验A：投影与反投影

```bash
ros2 run so101_vision camera_model_demo
```

预期看到：

- 内参矩阵 `K`；
- 三个目标像素及对应 Camera Frame 三维点；
- 重新投影后回到原像素；
- 最大往返误差接近 `0 px`；
- 图像缩小一半后，焦距和主点也同步缩小。

本项目的 320×240 合成相机使用：

```text
fx = fy = 280 px
cx = 159.5 px
cy = 119.5 px
```

### 5.4 实验B：无效深度与点云

合成深度中故意放入 `0` 和 `NaN`。阅读 `depth_to_pointcloud()`，确认有效条件同时包含：

```text
isfinite(depth) AND min_depth <= depth <= max_depth
```

不要把 16 位深度图直接当米使用。常见真实相机的 `16UC1` 可能以毫米保存，需要先乘相机
给出的 `depth_scale`；本项目发布的是 `32FC1`，数值单位已经是米。

### 5.5 建议动手修改

1. 把 `camera_model_demo.py` 中像素改到无效深度位置，观察错误原因。
2. 将图像改为 640×480，验证 `fx、fy` 是否同比变为 560 px。
3. 修改 `pixel_stride` 为 1、4、8，观察点云密度与计算量。

## 6. 3.2 相机标定、手眼标定与坐标转换

这一节包含两个不同问题：相机标定求 `K` 和畸变；手眼标定求相机与机械臂之间的刚体
变换。两者不能互相替代。

### 6.1 实验A：相机内参标定

```bash
ros2 run so101_vision calibration_demo --views 24 --noise-px 0.18
```

样例生成 24 组不同视角的棋盘角点，用 `cv2.calibrateCamera()` 估计：

- 内参矩阵 `K`；
- 径向畸变 `k1、k2、k3`；
- 切向畸变 `p1、p2`；
- 每张图的外参；
- 总体和单视角重投影误差。

正常输出中，估计的 `fx、fy、cx、cy` 应接近真值，RMS 重投影误差通常接近设置的像素
噪声量级。畸变高阶项可能比焦距更不稳定，这是因为有限视角对高阶参数约束较弱。

对比实验：

```bash
ros2 run so101_vision calibration_demo --views 6 --noise-px 0.8
ros2 run so101_vision calibration_demo --views 40 --noise-px 0.1
```

记录两组结果的焦距误差和重投影误差。真实采集时，不要只在图像中央拍摄标定板；标定板应
覆盖四角、边缘、远近和不同倾角。

### 6.2 实验B：Eye-in-Hand手眼标定

```bash
ros2 run so101_vision hand_eye_demo --poses 24
```

样例假设相机固定在 SO101 末端，生成多组机械臂位姿和标定板观测，再用
`cv2.calibrateHandEye()` 求 `T_gripper_camera`。无噪声合成数据中，平移与旋转误差应接近
零。

本项目统一采用 `T_A_B` 表示“把 B Frame 中的点变换到 A Frame”：

```text
p_base = T_base_gripper · T_gripper_camera · p_camera
T_base_object = T_base_gripper · T_gripper_camera · T_camera_object
```

矩阵相乘顺序从右向左执行。若是 Eye-to-Hand，即相机固定在机械臂外部，则需要求固定的
`T_base_camera`，不能照搬 Eye-in-Hand 变换链。

### 6.3 手眼标定常见失败原因

- 采样姿态只有平移，旋转变化不足；
- 每次拍照时机械臂还在运动；
- 棋盘尺寸或单位填写错误；
- 把 `T_camera_target` 和 `T_target_camera` 用反；
- 机械臂位姿与图像没有正确配对；
- 标定后只看求解成功，没有用独立姿态验证误差。

## 7. 3.3 深度图、点云处理与三维场景理解

### 7.1 实验流程

```bash
ros2 run so101_vision pointcloud_demo
```

默认输入包含桌面、三个长方体目标和随机离群噪声。程序依次执行：

1. 体素下采样：同一体素内保留质心，降低点数；
2. 统计离群点过滤：删除近邻平均距离明显偏大的孤立点；
3. RANSAC 平面拟合：寻找桌面并移除平面内点；
4. 高度过滤：去掉桌面下方或贴近桌面的残留点；
5. 欧式聚类：根据三维邻接关系分离物体；
6. PCA OBB：估计每个物体的中心、主轴和尺寸。

正常情况下应识别出 3 个物体聚类，桌面法向量应接近 `[0,0,1]`。

### 7.2 参数含义

| 参数 | 默认值 | 太小的现象 | 太大的现象 |
|---|---:|---|---|
| `--voxel` | 0.008 m | 点数多、计算慢 | 小物体细节丢失 |
| `--plane-threshold` | 0.006 m | 桌面移除不完整 | 目标底部被当成桌面 |
| `--cluster-distance` | 0.025 m | 同一物体被拆开 | 相邻物体粘成一类 |

尝试：

```bash
ros2 run so101_vision pointcloud_demo --voxel 0.012
ros2 run so101_vision pointcloud_demo --plane-threshold 0.002
ros2 run so101_vision pointcloud_demo --cluster-distance 0.06
```

每次只修改一个参数，否则无法判断结果变化来自哪个环节。

### 7.3 ICP的正确定位

`pointcloud.py` 提供点到点 ICP。ICP 是局部优化方法，适合在 PnP、模板匹配或上一帧结果
已经给出合理初值后精修。初始位姿差得太大时，最近邻对应会出错，即使程序报告收敛也可能
收敛到错误位置。

## 8. 3.4 Grounding DINO与SAM目标分割

### 8.1 先学习接口，再安装大模型

本章基础样例用颜色词实现轻量替身，但保留真实流水线的输入输出关系：

```text
文本Prompt + RGB
       │
       ▼
开放词汇检测器 ── Bounding Box
       │
       ▼
分割模型 ── Mask
       │ 与Depth逐像素融合
       ▼
目标三维点云与目标中心
```

轻量模式不是 Grounding DINO 或 SAM 的算法实现，它的目的，是让没有 GPU 和模型权重的
学员先把后续 Mask、Depth 和 TF 流程跑通。基础模式仅支持 red、blue、green 及对应中文
颜色词。

### 8.2 实验A：文本目标到Mask

```bash
ros2 run so101_vision segmentation_demo \
  --prompt "红色杯子" \
  --save /tmp/red_cup.png
```

预期输出包括检测框、Mask 像素数、有效三维点数、Camera Frame 中位点和目标点云尺寸。
保存图中目标区域保持原亮度，其余区域会变暗。

继续尝试：

```bash
ros2 run so101_vision segmentation_demo --prompt "blue box"
ros2 run so101_vision segmentation_demo --prompt "绿色零件"
```

Mask 在转换目标点云前会轻微腐蚀，目的是减少目标边缘混入背景深度。腐蚀过大会损失小目标，
不腐蚀则可能把桌面点带入目标点云。

### 8.3 实验B：真实Grounded-SAM

真实推理入口是 `grounded_sam_optional`，使用延迟导入，因此没有安装 PyTorch 和模型仓库
时不会影响其他样例。安装方法、权重和命令见 [OPTIONAL_MODELS.md](OPTIONAL_MODELS.md)。

迁移真实模型时，应保持输出为与原图等尺寸的单通道 Mask。这样
`masked_target_cloud()`、ROS 目标定位和 TF 代码都无需改写。

## 9. 3.5 传统6D物体位姿估计

### 9.1 PnP实验

```bash
ros2 run so101_vision pnp_pose_demo --noise-px 0.35
```

样例流程：

1. 创建已知尺寸的立方体模型点；
2. 用真值位姿把三维模型点投影到图像；
3. 添加像素噪声，并故意污染一个对应点；
4. 用 `solvePnPRansac()` 排除错误对应并估计 `T_camera_object`；
5. 计算内点重投影 RMSE、平移误差、旋转误差、ADD 和 ADD-S；
6. 生成 `T_base_object` 和 `T_base_grasp`。

如果把 `--noise-px` 逐渐增大，应看到位姿误差上升。PnP 求解成功不等于结果可用，必须同时
检查内点数、重投影误差和位姿跳变。

### 9.2 ADD与ADD-S

- ADD：同名模型点在估计位姿和真值位姿下的平均距离，适合非对称物体；
- ADD-S：每个估计点寻找真值模型中的最近点，能减少对称等价姿态带来的误判。

对圆柱、瓶子或重复结构物体，单独使用普通 ADD 可能把视觉上等价的姿态判断为错误。

### 9.3 从物体位姿到抓取位姿

```text
T_base_grasp = T_base_camera · T_camera_object · T_object_grasp
```

`T_object_grasp` 不是视觉算法自动产生的固定答案，它描述针对该类物体预先设计或由抓取网络
预测的抓取偏移。得到抓取位姿后，还必须由第2章 MoveIt 检查 SO101 是否可达、是否碰撞。

## 10. ROS 2章末综合实验

### 10.1 一条命令启动

```bash
source /opt/ros/humble/setup.bash
source ~/so101_ros2_learning_ws/install/setup.bash
ros2 launch so101_vision_ros vision_demo.launch.py
```

Launch 会启动：

- 第2章的 SO101 Mock Hardware 和 `robot_state_publisher`；
- 合成 RGB-D 发布器；
- 场景点云节点；
- 文本目标定位节点；
- 第3章 RViz 配置。

RViz 中应看到官方 SO101 结构、TF、蓝灰色场景点云、红色目标点云和目标 Pose 坐标轴。

需要注意，RViz 的默认布局用于观察三维结果，不直接显示二维彩色图和深度图。上述 Launch
默认发布的是课程生成的合成 RGB-D 图像；它是完整的 ROS `Image` 消息，可以正常查看和处理，
但不是物理摄像头实拍画面。

### 10.2 在新终端查看RGB、深度图和目标Mask

保持 `vision_demo.launch.py` 所在终端运行。打开一个新终端并加载 ROS 2 与工作空间：

```bash
source /opt/ros/humble/setup.bash
source ~/so101_ros2_learning_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

`rqt_image_view` 会打开图像窗口。在窗口左上角的话题列表中依次选择：

| 话题 | 显示内容 | 编码 |
|---|---|---|
| `/camera/color/image_raw` | 合成彩色图 | `bgr8` |
| `/camera/depth/image_raw` | 合成深度图 | `32FC1`，单位米 |
| `/vision/target_mask` | 文本目标对应的二值Mask | `mono8` |

一次选择一个话题即可。彩色图应看到课程生成的桌面目标画面；深度图是按距离着色或灰度化的
结果；Mask 中目标区域为白色，其余区域为黑色。终端本身只打印节点日志，图像像素需要在
`rqt_image_view` 窗口中查看。

如果系统提示找不到 `rqt_image_view`，安装后重新打开终端：

```bash
sudo apt update
sudo apt install ros-humble-rqt-image-view
```

若窗口的话题列表为空，先确认 Launch 仍在运行，再检查图像是否持续发布：

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic hz /vision/target_mask
```

默认频率约为 5 Hz。查看真实摄像头画面时，操作工具仍然是 `rqt_image_view`，但需要按照
第 11 节停用合成发布器并启动真实相机驱动。

### 10.3 分终端检查

另开终端并 source 环境：

```bash
ros2 node list
ros2 topic list
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /camera/depth/camera_info --once
ros2 topic echo /vision/target_pose_camera --once
ros2 topic echo /vision/target_pose_base --once
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
```

`target_pose_camera.header.frame_id` 应为 `camera_depth_optical_frame`，而
`target_pose_base.header.frame_id` 应为 `base_link`。两条消息数值不同是正常现象，说明 TF2
真正执行了坐标变换。

### 10.4 Launch参数

```bash
ros2 launch so101_vision_ros vision_demo.launch.py \
  prompt:="green part" \
  use_rviz:=false
```

| Launch参数 | 默认值 | 说明 |
|---|---|---|
| `prompt` | `red cup` | 目标文本提示 |
| `use_rviz` | `true` | 是否打开RViz |

节点参数位于 `so101_vision_ros/config/vision.yaml`：

| 节点参数 | 默认值 | 说明 |
|---|---:|---|
| `fps` | 5.0 | 合成相机发布频率 |
| `pixel_stride` | 4 | 点云像素采样步长 |
| `max_depth_m` | 1.2 | 最大有效深度，单位米 |
| `erosion_pixels` | 2 | Mask边缘腐蚀像素 |
| `output_frame` | `base_link` | 目标Pose输出坐标系 |

## 11. 接入真实RGB-D相机

基础算法无需修改，按以下顺序迁移：

1. 停止 `synthetic_rgbd_publisher`；
2. 启动真实相机 ROS 2 驱动；
3. 确认彩色图、对齐深度图和 `CameraInfo` 的话题名；
4. 将订阅话题改为真实话题，或用 ROS remapping；
5. 确认 Depth 编码和单位，特别区分 `16UC1` 与 `32FC1`；
6. 确保 RGB 和 Depth 已对齐，且 `CameraInfo` 对应深度图分辨率；
7. 在 SO101 URDF 中提供真实的相机安装外参，或发布静态 TF；
8. 先检查 `/vision/scene_cloud`，再接目标分割和位姿估计。

话题重映射示意：

```bash
ros2 run so101_vision_ros rgbd_pointcloud_node --ros-args \
  -r /camera/depth/image_raw:=/your_camera/aligned_depth/image_raw \
  -r /camera/depth/camera_info:=/your_camera/aligned_depth/camera_info
```

本章综合节点假设 RGB 与深度像素对齐。如果相机驱动只提供未对齐深度，不能直接用 RGB
Mask 索引深度图，必须先使用相机驱动的对齐功能或完成彩色/深度外参重投影。

## 12. 常见问题

### 找不到包或可执行程序

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg executables so101_vision
```

仍找不到时重新构建，确认 `colcon list` 能发现 `src/chapter03` 下的包。

### `cv_bridge`或OpenCV导入失败

通常是 Conda Python 与 ROS 系统 Python 混用。用 `/usr/bin/python3` 检查依赖，并按本页
构建命令显式指定 Python。

### 有图像但没有点云

依次检查：Depth 编码、深度单位、图像尺寸是否与 CameraInfo 一致、有效深度范围，以及
`pixel_stride` 是否大于等于 1。

### 有Camera Frame Pose但没有Base Frame Pose

说明视觉算法已经工作，问题通常在 TF。检查：

```bash
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
```

若查不到变换，确认第2章 `robot_state_publisher` 已启动、`joint_states` 正在发布、相机 Frame
名称与图像消息 `header.frame_id` 完全一致。

### Mask为空

轻量模式只理解 red、blue、green 或对应中文颜色词。真实开放词汇目标需要安装 3.4 可选
模型。即使 Prompt 合法，真实场景还应检查光照、颜色范围与输入图像编码。

### 点云方向或位置明显不对

最常见原因是深度单位错误、内参来自另一分辨率、把 `(u,v)` 写成 `(v,u)`、光学坐标系与
普通相机 Link 混用，或手眼变换方向取反。

## 13. 测试与课后练习

运行测试：

```bash
colcon test --packages-select so101_vision so101_vision_ros
colcon test-result --verbose
```

当前测试覆盖投影往返、内参缩放、无效深度、体素、桌面 RANSAC、ICP、颜色 Mask 与 PnP。

建议按难度完成以下练习：

1. 入门：增加一个黄色目标，并扩展中文 Prompt 映射；
2. 入门：发布 `/vision/target_mask` 后用 `rqt_image_view` 查看；
3. 基础：把目标点云中心从均值改成中位数，对比离群点影响；
4. 基础：给合成深度加入高斯噪声，重新选择 RANSAC 阈值；
5. 进阶：读取真实棋盘图像，增加角点检测和标定结果 YAML 保存；
6. 进阶：将 PnP 结果作为 ICP 初值，用目标深度点云精修；
7. 综合：把 `T_base_grasp` 发送给第2章 MoveIt，先只规划、不执行；
8. 综合：接入真实 RGB-D，并形成相机内参、手眼外参和重复定位误差报告。

完成练习时应保留每次实验的参数、输出误差和失败案例。视觉系统的重点不是“某一次看起来
成功”，而是明确输入约束、量化误差，并能解释失败发生在哪一层。
