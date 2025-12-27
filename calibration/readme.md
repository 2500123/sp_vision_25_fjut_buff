## Calibration 工具说明

本目录包含标定与数据采集相关的小工具，用于建立和维护相机、云台/机器人以及世界坐标系之间的一致关系。

目录结构概览：

- `calibrate_camera.cpp`：相机内参与畸变标定
- `calibrate_handeye.cpp`：手眼标定（相机–云台/机械体外参）
- `calibrate_robotworld_handeye.cpp`：机器人–世界–手眼联合标定
- `capture.cpp`：标定/调试用图像或视频采集
- `split_video.cpp`：将视频切帧或切片

下面给出每个工具的用途说明和典型编译 / 运行示例。

> 说明：具体可执行文件名取决于顶层 `CMakeLists.txt` 的配置，下面假设你已经在工程根目录完成过一次 CMake 构建（例如生成到 `build/` 目录）。

---

## 1. 编译本工程

在工程根目录下：

```bash
cd /home/hyc/10/sp_vision_25_fjut_tuned
cmake -B build
cmake --build build -j
```

完成后，标定相关可执行文件通常位于 `build/` 目录下，例如：

- `build/calibrate_camera`
- `build/calibrate_handeye`
- `build/calibrate_robotworld_handeye`
- `build/capture`
- `build/split_video`

（如名称有出入，可在 `build/` 下使用 `ls` 或 `grep calibrate` 自行确认。）

---

## 2. `calibrate_camera.cpp` — 相机标定

**用途：**
- 标定相机内参（`camera_matrix`）和畸变系数（`distort_coeffs`）。
- 标定结果通常写入 `configs/*.yaml`，供自瞄和 PnP 解算使用。

**典型流程：**
1. 使用 `capture` 采集多张棋盘格/标定板图片。
2. 运行 `calibrate_camera`，指定图片路径或使用程序内默认路径。
3. 得到内参、畸变参数，并填写到对应配置文件中。

**示例命令：**

```bash
cd /home/hyc/10/sp_vision_25_fjut_tuned

# 假设可执行文件名为 calibrate_camera
./build/calibrate_camera
```

程序通常会提示：
- 输入标定图片目录，或使用代码中预设路径。
- 最终在终端或文件中输出内参、畸变系数。

---

## 3. `calibrate_handeye.cpp` — 手眼标定

**用途：**
- 求解相机坐标系到云台/机械体坐标系的外参：
	- `R_camera2gimbal`
	- `t_camera2gimbal`
- 结果会写入 `configs/*.yaml`，用于将相机观测坐标变换到云台坐标，进而生成控制指令。

**数据准备：**
- 多组不同姿态下：
	- 机械体/云台的位姿（编码器、IMU、机器人末端位姿等）。
	- 相机对标定板（棋盘格）的位姿观测。

**示例命令：**

```bash
cd /home/hyc/10/sp_vision_25_fjut_tuned

./build/calibrate_handeye
```

运行流程一般为：
- 程序读取预定义的数据文件 / 实时话题。
- 求解 `R_camera2gimbal` 与 `t_camera2gimbal`。
- 在终端打印或写入标定结果，供你拷贝到 YAML：

```yaml
R_camera2gimbal: [r11, r12, r13, r21, r22, r23, r31, r32, r33]
t_camera2gimbal: [tx, ty, tz]
```

---

## 4. `calibrate_robotworld_handeye.cpp` — 机器人/世界联合标定

**用途：**
- 建立机器人基坐标系 / 世界坐标系与相机/末端之间的统一关系。适用于使用机械臂或需要全局坐标的场景。

**典型应用：**
- 求解类似 `R_robot2world`, `t_robot2world`，使得视觉测量能在世界坐标下表示，便于高级规划或全局显示。

**示例命令：**

```bash
cd /home/hyc/10/sp_vision_25_fjut_tuned

./build/calibrate_robotworld_handeye
```

具体输入/输出格式取决于程序内部实现，一般会：
- 读取多组机器人基座–末端位姿与摄像机–标定板位姿。
- 输出机器人–世界及相机–末端的变换矩阵。

---

## 5. `capture.cpp` — 采集图像/视频

**用途：**
- 从指定相机采集图像或视频，用于：
	- 相机内参与畸变标定。
	- 手眼标定数据采集。
	- 算法离线调试（生成带装甲板/能量机关的视频）。

**示例命令：**

```bash
cd /home/hyc/10/sp_vision_25_fjut_tuned

./build/capture
```

常见行为：
- 程序打开配置中指定的相机（如 `camera_name: hikrobot`）。
- 显示实时画面，并根据按键或内部逻辑保存：
	- 静态图片：`calib_001.png`, `calib_002.png`, ...
	- 或视频文件：`capture_2025xxxx.avi` 等。

> 建议：在相机标定时，尽量在不同距离、视角、光照条件下采集多组棋盘格图像，以提高精度。

---

## 6. `split_video.cpp` — 视频切帧/切片

**用途：**
- 将长视频拆分为帧图片或多段小视频，用于：
	- 从录制视频中提取标定图像。
	- 构建目标检测/跟踪的训练与测试数据集。

**示例命令：**

```bash
cd /home/hyc/10/sp_vision_25_fjut_tuned

./build/split_video
```

典型流程：
- 程序要求输入源视频路径（或使用默认）。
- 按固定间隔（如每帧/每 N 帧）将视频帧保存为图片：
	- `frames/frame_0001.jpg`, `frame_0002.jpg`, ...
- 或按时间/帧数拆分为多段视频：
	- `part_01.avi`, `part_02.avi`, ...

---

## 7. 推荐工作流

1. **相机标定**
	 - 使用 `./build/capture` 采集棋盘格图片。
	 - 运行 `./build/calibrate_camera` 标定内参和畸变系数。
	 - 将结果写入 `configs/*.yaml` 中的 `camera_matrix` 和 `distort_coeffs`。

2. **相机–云台/机械体外参（手眼）标定**
	 - 在多种云台姿态下，采集相机看到的标定板数据（可由 `capture` 或在线方式获得）。
	 - 运行 `./build/calibrate_handeye`，求解 `R_camera2gimbal` 与 `t_camera2gimbal`。
	 - 将结果写入对应配置 YAML。

3. **（可选）机器人–世界–手眼联合标定**
	 - 如果有机械臂或明确的世界坐标需求，使用 `./build/calibrate_robotworld_handeye` 完成高级标定。

4. **数据集构建与算法调试**
	 - 使用 `./build/capture` 录制实际比赛/测试视频。
	 - 用 `./build/split_video` 将视频切帧，作为检测/跟踪/自瞄算法的离线数据。

完成以上步骤后，确保：
- 相机参数、外参已经正确填入 `configs/*.yaml`。
- 上层模块（如 `tasks/auto_aim`、`tasks/auto_buff`）可以直接使用这些标定结果进行精确的空间解算和瞄准。













calibration 目录说明

本目录包含相机标定、手眼标定、机器人-世界坐标系标定，以及数据采集/视频切分等辅助工具，用于搭建和维护整套视觉坐标系与机械坐标系的一致性。

目录结构：

calibrate_camera.cpp：相机内参与畸变标定
calibrate_handeye.cpp：手眼标定（相机与机械臂/云台坐标系关系）
calibrate_robotworld_handeye.cpp：机器人基坐标系与世界坐标系的联合标定
capture.cpp：采集标定或调试使用的图片/视频
split_video.cpp：将长视频切分成图片序列或多段视频
1. calibrate_camera.cpp — 相机标定

用途：

标定相机的内参矩阵（焦距 fx, fy 和主点 cx, cy）以及畸变系数（k1, k2, p1, p2, k3 等）。
为后续所有 3D 重建、PnP 解算、弹道补偿提供准确的成像模型。
典型流程：

输入：
一组标定板图片（棋盘格或圆点阵），可以由 capture.cpp 采集。
处理步骤（逻辑上通常包括）：
读取图像。
检测棋盘格/标定板角点。
利用 OpenCV cv::calibrateCamera 计算内参和畸变。
输出：
相机内参 camera_matrix。
畸变系数 distort_coeffs。
可直接写入 configs/*.yaml 中对应字段，例如：
camera_matrix: [...]
distort_coeffs: [...]
使用场景：

更换相机、镜头或分辨率、焦段后必须重新标定。
当前工程中，多数三维解算（如 solver.*、buff_solver.*）依赖这些参数。
2. calibrate_handeye.cpp — 手眼标定

用途：

求解相机坐标系到“机械体/IMU/云台”坐标系的外参关系（旋转和平移）。
常用模型：AX = XB 或 AX = ZB 类型。
在本工程中的典型目标：

得到 R_camera2gimbal、t_camera2gimbal 等参数，用于将目标从相机坐标转换到云台（或机器人）坐标。
典型流程：

输入：
多组机械臂/云台位姿（比如来自 IMU 或编码器，位姿矩阵 A）。
对应时刻下相机观测到的标定板位姿（位姿矩阵 B，通过相机+标定板求得）。
处理步骤：
使用手眼标定算法（OpenCV calibrateHandEye 或自实现）。
求解相机到机械体坐标系的固定变换。
输出：
旋转矩阵 R_camera2gimbal。
平移向量 t_camera2gimbal。
同样可填入 YAML 配置：
R_camera2gimbal: [...]
t_camera2gimbal: [...]
使用场景：

任何需要将相机看到的装甲板/能量机关位置映射到云台控制指令的场景。
云台、相机安装刚性改变时，需要重新标定。
3. calibrate_robotworld_handeye.cpp — 机器人-世界-手眼联合标定

用途：

建立“机器人基坐标系/世界坐标系”和“相机/手眼坐标系”之间的统一变换关系。
适用于有机械臂或存在“世界坐标系”（如场地全局坐标）的高级场景。
典型目标：

得到机器人基坐标系到世界坐标系的变换 R_robot2world、t_robot2world 等。
结合相机-机械体外参，实现从相机测量点到世界坐标的完整链路。
典型流程（概念）：

输入：
机器人末端在基坐标系下的位姿序列。
相机/标定板位姿在相机或世界坐标下的观测。
处理步骤：
使用机器人-世界-手眼联合标定算法（如 OpenCV 或自实现）。
同时解出机器人基座与世界、相机与末端等多个坐标系之间的变换。
输出：
一组外参矩阵，用于全局定姿和轨迹映射。
使用场景：

高级多坐标系融合：如机器人在场地中的绝对位置、绝对路径规划等。
如果你的应用只需要“视觉 + 云台”局部坐标，可能只会用到 calibrate_camera + calibrate_handeye。
4. capture.cpp — 数据采集工具

用途：

从相机采集图像或视频，用于：
相机标定（采集棋盘格图）。
手眼标定（采集多姿态下的标定板图）。
算法调试（录制实战视频、环境样本）。
典型功能：

打开指定相机（如 hikrobot、mindvision 等）。
根据键盘/命令触发保存当前帧为图片或视频。
设置分辨率、曝光、增益等基本参数（通常来自 configs/*.yaml）。
输出形式：

标定图片：calib_001.png, calib_002.png, …。
或一段/多段 *.avi/*.mp4 视频，用于后续离线分析或 split_video.cpp 处理。
5. split_video.cpp — 视频切分工具

用途：

将录制好的视频切分成帧图像，或按时间/帧数分成多段视频。
常用于：
从比赛/测试视频中提取关键帧做标定或训练数据。
为检测/跟踪算法准备离线数据集。
典型流程：

输入：
一段视频文件（由 capture.cpp 或其他工具生成）。
处理步骤：
打开视频，逐帧读取。
根据设定间隔（如每帧/每 N 帧）保存为图片序列，或按时间切割。
输出：
一系列图片：frame_0001.jpg, frame_0002.jpg, …。
或多段小视频：part_01.avi, part_02.avi, …。
6. 典型工作流建议

相机内参与畸变标定

使用 capture.cpp 采集标定板图像。
用 calibrate_camera.cpp 计算 camera_matrix 和 distort_coeffs。
将结果写入 configs/*.yaml。
手眼标定（相机-云台/IMU）

设置相机和云台/机器人，在多个姿态下拍摄标定板。
用 calibrate_handeye.cpp 求解 R_camera2gimbal、t_camera2gimbal。
写入 configs/*.yaml 对应字段。
（可选）机器人-世界联合标定

如果有机械臂/世界坐标需求，使用 calibrate_robotworld_handeye.cpp 完成。
将外参用于全局定位及路径规划。
数据采集与算法调试

使用 capture.cpp 录制实战或测试视频。
用 split_video.cpp 切分视频，构建离线数据集，调试检测/跟踪/自瞄算法。
7. 与主工程的关系

calibration 得到的标定结果（内参、外参）会直接写入 configs/*.yaml，例如：
camera_matrix, distort_coeffs
R_camera2gimbal, t_camera2gimbal
这些参数被：
tasks/auto_aim/solver.*
tasks/auto_buff/buff_solver.*
以及其他使用 3D 解算/弹道补偿的模块调用。
标定质量直接决定自瞄系统在空间定位、角度补偿和命中率上的表现。