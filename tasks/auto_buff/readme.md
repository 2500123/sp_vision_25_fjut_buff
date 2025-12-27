这套模块实现“打符（能量机关）”的完整链路：检测扇叶/圆心 → 估计旋转与相位 → 三维/像素几何解算 → 预测开火时机与瞄准角 → 下发串口/ CAN 指令。
对应文件夹内各组件分工明确：检测器、目标/类型组织、预测器、解算器、瞄准器，以及 YOLO 推理封装。
数据处理流程（从相机到开火）
相机帧与姿态输入
相机帧来源：io/camera（工业相机或USB相机）。
姿态/弹速来源：io/gimbal（MicroUSB虚拟串口）或 io/cboard（USB2CAN）。姿态四元数用于坐标系变换，弹速用于弹道解算。
检测与几何提取（buff_detector）
可走两路：
神经网络（yolo11_buff.cpp）：OpenVINO 推理，产出扇叶/扇叶头/“R”标等候选框+置信度，通过 NMS 后进入后处理。
传统阈值（configs 中 detect: brightness/morphology/面积阈值等）：二值化+形态学+轮廓筛选，匹配标准扇叶模板（standard_fanblade.jpg）以提升稳健性。
输出：当前帧的“扇叶集合/圆心候选/标准化关键点”，并附带质量指标（得分、面积、角度等）。
目标装配与类型识别（buff_target + buff_type）
将检测到的扇叶与圆心组装成“目标对象”（Target/Fan/Blade），筛掉不合理组合。
判定类型/状态（如标准/大符/小符、是否可击打扇叶、当前活动扇叶编号等），为后续预测与瞄准做约束。
旋转/相位估计与预测（buff_predict）
根据历史若干帧的圆心角、扇叶索引和时间戳，估计角速度 ω、相位 φ，建立旋转运动的简单模型。
结合配置 predict_time（或与弹道飞行/系统延迟叠加）预测“未来时刻”的扇叶目标角度与位置，给瞄准器使用。
三维/像素解算（buff_solver）
根据已知几何（机关盘半径、扇叶形状）和相机标定（camera_matrix、distort_coeffs、R_camera2gimbal、t_camera2gimbal、R_gimbal2imubody）在像素↔三维之间进行转换。
输出“该时刻应瞄的位置”在不同坐标系（相机/云台/世界）的表示，提供给瞄准器进行弹道计算。
瞄准与开火决策（buff_aimer）
从预测的目标“未来位置”出发，结合当前弹速，调用弹道工具（tools::Trajectory）解算 pitch；yaw = atan2(y,x) + yaw_offset（配置项，单位度）。
根据 fire_gap_time（最小开火间隔）、aim_time/wait_time 等，判断该帧是否下发“开火”。
输出：io::Command（control, fire, yaw, pitch…），或 VisionToGimbal 帧（含 yaw/pitch/速度/加速度前馈）。
下发与记录
通过 gimbal（串口）或 io/cboard（CAN）下发控制；Plotter/Logger 记录曲线与日志，便于调参。
指令与状态传输流程（视觉 ↔ 下位机）
上行（下位机 → 视觉）：
串口（MicroUSB，gimbal）：结构体 GimbalToVision，含四元数 q（wxyz）、yaw/pitch/速度、bullet_speed、bullet_count、mode 等，CRC16 校验；Gimbal::q(t) 支持按时间戳插值。
CAN（USB2CAN，io/cboard）：解析 IMU 姿态四元数 CAN ID、弹速 CAN ID，在 CBoard::imu_at(timestamp) 中按时间插值。
下行（视觉 → 下位机）：
串口：VisionToGimbal 结构体，mode（不控/控云台/开火）、yaw/pitch 及其 vel/acc 前馈、CRC16，Gimbal::send(...) 写串口。
CAN：CBoard::send(Command) 将 yaw/pitch/fire 等编码进指定 CAN ID 并发送。
参数入口：configs/*.yaml 中：
通信：com_port: "/dev/gimbal"（串口）、can_interface: "can0" 与各 CAN ID。
打符：buff_detector.detect.*、buff_aimer.*（fire_gap_time、predict_time 等）。
相机与标定：camera_matrix、distort_coeffs、R_camera2gimbal、t_camera2gimbal、R_gimbal2imubody。
偏置：yaw_offset、pitch_offset（度，代码中会转弧度）。
各文件职责与“输入/输出”小合同
yolo11_buff.{hpp,cpp}（推理封装）

输入：cv::Mat（BGR/灰度）或预处理后的 blob；模型路径（yolo11_buff_int8.xml）
输出：检测框/类别/置信度；NMS 后的扇叶、圆心、R 标候选
失败模式：无检测/置信度过低 → 交给传统/下游过滤
buff_detector.{hpp,cpp}（检测器）

输入：原始图像、配置 detect、可选 ROI；从模型或传统法得出的候选
输出：扇叶列表、圆心、关键点、匹配得分；可能包含“标准扇叶模板匹配”的结果
失败模式：光照极端/阈值不当导致误检漏检 → 调 detect、对比网络/传统两路结果
buff_type.{hpp,cpp}（类型识别/标签）

输入：检测出的几何/模式
输出：类型标签（例如标准/大小符、可击打状态），供后续策略分支与限幅
buff_target.{hpp,cpp}（目标对象/状态聚合）

输入：当前帧检测输出 + 历史状态
输出：更新后的目标（圆心角、扇叶索引、可用性标志），作为预测与瞄准输入
失败模式：目标跳变/遮挡 → 标志位与回退策略
buff_predict.hpp（预测）

输入：历史角度/时间序列（ω、φ估计），配置 predict_time
输出：预测时刻的目标角、对应像素或三维点
失败模式：速度突变 → 做限速/限加/滑动窗口稳健估计
buff_solver.{hpp,cpp}（几何/坐标解算）

输入：相机标定、云台-IMU外参、IMU姿态、像素点/目标几何
输出：目标在相机/云台/世界的坐标与朝向；必要时像素重投影用于误差评估
失败模式：PnP 不可解/重投影误差大 → 丢弃该帧或回退前值
buff_aimer.{hpp,cpp}（瞄准与开火）

输入：预测目标的未来位置（相机/云台坐标）、当前弹速、配置参数
输出：控制指令（yaw/pitch/vel/acc + fire），满足 fire_gap_time 与窗口约束
失败模式：弹道不可解（距离/高度与弹速矛盾）→ 本帧不发
CMakeLists.txt

组织以上组件编译为库/目标，供主程序链接
时序（文字版）
相机帧(t) + 姿态q(t) → buff_detector → buff_target（装配） → buff_predict（t+Δ预测） → buff_solver（坐标变换） → buff_aimer（弹道/决策） → io（串口/CAN 下发） 并行：io 持续上报姿态/弹速，plotter/logger 记录调试数据。

关键调参提示
入口：configs/* 的 buff_detector.detect.* 与 buff_aimer.*；模型路径在 model。
光照差异大时，优先对 brightness、brightness_threshold、形态学核尺寸与面积阈值做细调；
fire_gap_time、predict_time 直接影响“放枪频率”和“提前量”；
标定项若不准（R_camera2gimbal/t_camera2gimbal/camera_matrix/distort_coeffs），会系统性偏差，先标定再调偏置。









图像帧 → buff_detector（检测扇叶/圆心等）→ buff_type（类型判别）→ buff_target（装配成目标、维护状态） → buff_predict（根据角速度/相位预测未来位置） → buff_solver（像素/三维几何与坐标变换） → buff_aimer（弹道解算与开火决策） → IO 下发串口/ CAN 指令
文件职责一览

yolo11_buff.hpp / yolo11_buff.cpp

作用：能量机关（扇叶、圆心、R标等）的神经网络推理封装（OpenVINO/ONNX）。
输入：当前图像帧、模型路径（如 assets/yolo11_buff_int8.xml）和阈值。
输出：候选框（类别、置信度、坐标），内部做 NMS/后处理供检测器使用。
buff_detector.hpp / buff_detector.cpp

作用：打符检测器。可走“神经网络”或“传统方法”（二值化/形态学/面积阈值）两路，支持 ROI 与模板匹配。
输入：图像帧、配置 detect 参数（对比度/亮度/阈值/形态学核/面积阈值等）。
输出：扇叶/圆心/关键点集合及质量评分；为目标装配与类型判断提供基础数据。
buff_type.hpp / buff_type.cpp

作用：类型/状态判定（标准/大符/小符/是否可击打等），并做必要的几何约束。
输入：检测到的结构与几何信息。
输出：目标的类别标签/状态标志，影响后续预测和开火逻辑。
buff_target.hpp / buff_target.cpp

作用：将检测结果组装成“目标对象”，维护跨帧状态（索引/序列、是否跳变、历史角度等）。
输入：本帧检测结果 + 历史缓冲。
输出：规范化的目标（含圆心角、扇叶编号、可用性），供预测器与瞄准器调用。
buff_predict.hpp

作用：预测模块（轻量化头文件实现），根据历史角度/角速度估计未来时刻的扇叶角度与位置。
输入：目标当前状态（角度、角速度/相位）与 predict_time（配置）。
输出：t + Δ 的目标角度/像素或坐标估计（为 solver/aimer 使用）。
buff_solver.hpp / buff_solver.cpp

作用：几何与坐标解算。结合相机内外参与 IMU 姿态，把像素/扇叶几何变换到相机/云台/世界坐标；必要时做 PnP 或重投影计算。
输入：相机标定（camera_matrix、distort_coeffs）、外参（R_camera2gimbal、t_camera2gimbal、R_gimbal2imubody）、IMU 四元数、目标几何/像素信息。
输出：目标在相机/云台/世界坐标系下的点位/朝向、用于瞄准的空间信息。
buff_aimer.hpp / buff_aimer.cpp

作用：瞄准与开火决策。基于预测位置与弹速进行弹道解算（tools::Trajectory），计算 yaw/pitch，按 fire_gap_time/aim_time/wait_time 决定是否开火。
输入：预测后的目标空间位置、当前弹速、偏置（yaw_offset/pitch_offset）、时延与间隔参数。
输出：控制/开火指令（io::Command 或 VisionToGimbal 内容），交由 IO 层下发。
CMakeLists.txt

作用：该功能组的构建脚本，组织上述源文件编译链接，供上层程序使用。
readme.md

作用：模块级说明文档（算法思路、使用方式、注意事项）。
补充说明与调参入口

模型与检测阈值：configs/* 的 buff_detector.detect.* 与 yolo11_buff 模型路径。
预测与开火：configs/* 的 buff_aimer.*（predict_time、fire_gap_time、aim_time、wait_time 等）。
标定与坐标系：camera_matrix、distort_coeffs、R_camera2gimbal、t_camera2gimbal、R_gimbal2imubody（影响 solver 的解算精度）。
通信：串口 com_port（MicroUSB 虚拟串口走 io/gimbal）、或 CAN 接口 can_interface 与各 CAN ID（走 io/cboard）。









本目录实现了能量机关（buff）自动识别、预测与瞄准打击的完整流程。
整体流程包括：图像检测 → 目标类型识别 → 旋转运动建模与预测 → 三维解算与弹道补偿 → 瞄准/开火决策。

目录结构如下：

核心流程：
buff_detector.*：能量机关目标检测
buff_type.*：装甲扇叶/中心 R 标记等类型解析
buff_target.*：当前选中扇叶/目标建模
buff_predict.hpp：旋转运动预测模型
buff_solver.*：空间位姿解算与弹道补偿
buff_aimer.*：高层逻辑整合（瞄准与控制接口）
YOLO 检测器：
yolo11_buff.*：能量机关专用 YOLOv11 检测封装
构建文件：
CMakeLists.txt
readme.md：本说明文件
1. 顶层瞄准控制：buff_aimer.cpp / buff_aimer.hpp
buff_aimer 是能量机关自瞄模块的“总控与调度中心”，主要职责：

对外提供统一接口：
输入：当前相机图像、云台/IMU 姿态、比赛模式/小陀螺等信息。
输出：推荐云台 yaw/pitch 角度、是否开火、攻击哪一扇叶等控制量。
内部调用各子模块：
使用 buff_detector / yolo11_buff 从图像中检测能量机关结构（中心 R、扇叶、装甲板等）。
使用 buff_type 分析当前是大符/小符、激活扇叶、装甲板 ID 等。
用 buff_target 维护当前锁定目标（某一扇叶或 R 标记）。
使用 buff_predict 进行目标旋转运动预测（估计未来某一时刻扇叶角度）。
使用 buff_solver 做 PnP/弹道补偿，将预测结果转为云台控制角度。
根据命中窗口、时间误差、可靠性等做开火决策。
你可以理解为：buff_aimer 把所有能量机关相关的“识别 + 预测 + 控制”流程串起来，对上层（标准模式主程序、电控通信模块）只暴露一组简单接口。

2. 检测与类型识别
2.1 能量机关检测：buff_detector.cpp / buff_detector.hpp
负责从相机图像中检测出能量机关的关键元素：
圆心/中心 R 标记位置。
扇叶装甲板位置（多个子目标）。
一些几何信息：圆半径、扇叶角度、面积、长宽比等。
可能包含两类检测方式：
传统视觉：二值化、轮廓提取、拟合圆和矩形等。
神经网络检测结果融合（由 yolo11_buff 提供候选框）。
输出一组候选目标/扇叶，为后续模块提供基础观测量。
2.2 类型与状态解析：buff_type.cpp / buff_type.hpp
对检测结果进行语义/规则解析：
区分大符/小符。
确定当前激活扇叶、预备扇叶。
分析能量机关旋转方向（顺/逆时针）、转速模式。
标记目标是 R 中心区域还是外圈扇叶。
这种“类型信息”被 buff_target 和 buff_predict 使用，用于：
决定要攻击哪扇叶。
决定预测策略（大符有提前量，小符可能不同）。
3. 目标建模与预测
3.1 目标模型：buff_target.cpp / buff_target.hpp
抽象“当前锁定目标扇叶/装甲板”的数据结构和更新逻辑：
记录目标在图像坐标 / 相机坐标 / 世界坐标下的位置。
记录当前扇叶的角度（相对圆心角度）、角速度、角加速度估计。
保存是否激活、得分/置信度等。
提供接口：
根据新一帧检测结果更新目标（跟踪/重识别）。
计算当前目标与参考扇叶（例如固定红扇叶）的角度差。
是 buff_predict 和 buff_solver 的核心输入。
3.2 旋转运动预测：buff_predict.hpp
主要用于对能量机关旋转扇叶的未来角度进行预测：
使用简单运动模型或拟合模型，如：
匀角速度模型：
θ(t+Δt)=θ(t)+ωΔt
θ(t+Δt)=θ(t)+ωΔt。
含加速度模型：θ(t+Δt)=θ(t)+ωΔt+12α(Δt)2
θ(t+Δt)=θ(t)+ωΔt+21​α(Δt)2。
或根据上层配置切换不同预测模式（大符有固定预判偏移等）。
接口通常包含：
输入当前/历史扇叶角度和时间戳，输出在子弹击中时刻的预测角度。
可以根据不同子弹初速/延迟时间调整预测时间窗。
buff_predict 直接影响打击时机和命中率，是能量机关自瞄的关键部分。

4. 三维解算与弹道补偿：buff_solver.cpp / buff_solver.hpp
将 2D 图像信息转换为 3D 目标位置，并结合弹道模型给出瞄准解：
使用相机内参加畸变系数，将扇叶 ROI 四点或中心点通过 PnP 解算出目标在相机坐标系下的三维坐标。
通过相机-云台外参，将坐标转换到云台/世界坐标系。
根据子弹初速、重力加速度等建立抛物线弹道模型，计算应补偿的俯仰角。
典型输出：
推荐云台 yaw/pitch 角度。
目标空间距离等信息。
在能量机关场景中，会与 buff_predict 的结果结合：
先预测扇叶未来的空间位置，再进行 PnP/弹道解算。
或先在 2D 空间预测扇叶角度，再将该扇叶位置映射到 3D。
5. YOLO 能量机关检测：yolo11_buff.cpp / yolo11_buff.hpp
封装 YOLOv11 版本的能量机关检测模型，通常基于 ONNXRuntime 进行推理：
加载 yolo11_buff_int8.xml / ONNX 或对应权重文件。
对输入图像进行预处理（缩放、归一化、色彩空间转换等）。
前向推理输出涉及：
中心 R 标记框。
外圈扇叶/装甲板框。
后处理逻辑：
NMS 抑制重叠框。
将检测框转换为 buff_detector 或 buff_target 可用的数据结构（像素坐标、类别、置信度）。
与 buff_detector 结合：
可以提供初始候选框，再由传统几何约束进一步筛选。
或在光照/遮挡复杂情况下提升鲁棒性。
6. CMake 构建：CMakeLists.txt
该文件定义 auto_buff 子模块的构建规则：
收集本目录下的 .cpp 源文件，编译为静态/动态库。
链接依赖：
OpenCV（图像处理和矩阵操作）。
Eigen（数值计算，如旋转矩阵、向量运算）。
ONNXRuntime 或其他推理引擎（用于 yolo11_buff）。
yaml-cpp（读取参数配置，如旋转速度、预测模型参数等）。
对外导出包含目录和库名称，供 auto_buff_debug.cpp、standard_mpc.cpp 等上层程序使用。
7. 模块协同流程概述
以一帧图像处理为例，整体数据流可以概括为：

相机采集当前图像帧，传入 buff_aimer。
buff_aimer 调用：
yolo11_buff / buff_detector 对整幅图像做能量机关目标检测：
检出圆心/中心 R、多个扇叶装甲板候选。
buff_type 对检测结果进行解析，确定：
当前激活扇叶、旋转方向、模式（大符/小符）。
buff_target 根据当前检测结果和历史帧：
更新或重建当前锁定扇叶目标。
估计目标的角度、角速度等。
buff_predict 使用 buff_target 的状态和子弹飞行时间，预测：
未来时刻扇叶应在的位置（角度）。
buff_solver 将预测到的扇叶在图像/角度空间的位置映射到三维：
通过 PnP + 外参获得目标在世界/云台坐标下的 3D 点。
结合重力和初速进行弹道补偿，得到期望 yaw/pitch。
buff_aimer 综合判断是否满足开火条件：
误差角是否在允许范围。
扇叶是否处于“可击打窗口”（时间/角度双重约束）。
输出是否开火信号和瞄准角度，交由上层通信模块发给电控。
8. 使用与调参建议
配置文件：
旋转预测相关参数（如默认角速度、模型模式、大符提前角等）一般放在 configs/*.yaml 中。
弹道相关（子弹初速、重力、装甲板尺寸）在 solver 或通用配置中设置。
调试步骤建议：
在录播视频上单独调试 buff_detector / yolo11_buff，确保检测框正确。
打印或可视化扇叶角度曲线，验证 buff_target 和 buff_predict 的预测是否平滑、相位正确。
固定能量机关不转动时，验证 buff_solver 的三维距离和角度补偿是否正确。
实战/仿真中再逐步放开 MPC/预测开火逻辑，调节误差阈值和时间预判