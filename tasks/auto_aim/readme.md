本目录实现了标准自瞄（装甲板识别与打击）功能，从目标检测、识别、跟踪、预测到射击决策的完整流程。整体由以下几个层次组成：

YOLO 系列检测器（yolo*.cpp/hpp, yolos/）
传统装甲板检测与候选生成（detector.*, armor.*, classifier.*, voter.*）
目标建模与跟踪（target.*, tracker.*）
位姿解算与弹道补偿（solver.*）
总体决策与控制接口（aimer.*, shooter.*）
多线程调度与指令生成（multithread/）
MPC 运动预测与控制（planner/ 及 planner/tinympc/）
1. 顶层模块
aimer.cpp / aimer.hpp
负责整个自瞄流程的“调度和总控”，对外提供统一接口。
典型职责：
接收相机图像和 IMU/云台姿态信息。
调用装甲板检测、识别、跟踪、解算等子模块。
使用 planner 结果进行预测、补偿。
将最终的瞄准角度、是否开火等指令交给 shooter 或上层控制模块。
可以理解为“自瞄系统的主脑”。
shooter.cpp / shooter.hpp
负责与电控/发射机构的交互决策：
依据 aimer / planner 输出的目标状态（目标角度、速度、置信度等）判断是否开火。
综合子弹飞行时间、摩擦轮状态、模式（连续射击/点射）等因素。
对上层表现为“给我目标状态，我告诉你要不要打”。
2. 目标与检测模块
armor.cpp / armor.hpp
定义“装甲板”目标的数据结构和基础操作：
像素坐标下的四点、矩形框、面积、长宽比等几何信息。
识别出的数字/类别（0~9、英雄、工程、步兵等）。
得分、置信度、距离等属性。
可能包含：
排序/比较方法（例如按得分排序）。
向世界坐标或相机坐标的转换接口（与 solver 配合）。
detector.cpp / detector.hpp
对输入图像进行传统视觉的装甲板候选检测：
预处理（灰度、二值化、形态学等）。
轮廓提取、灯条/矩形拟合。
候选装甲板区域生成，并转为 Armor 对象。
为后续识别/筛选提供候选框，部分项目中会和 YOLO 结果融合。
classifier.cpp / classifier.hpp
对候选装甲板进行分类/数字识别：
使用小型 CNN、SVM 或其他分类器，对 ROI 图像进行分类。
输出装甲板 ID、置信度。
通常在 detector 之后被调用，将候选框升级为“带标签的目标”。
voter.cpp / voter.hpp
在多帧、多检测源之间进行投票/融合：
解决单帧误检、短暂识别错误的问题。
通过时间窗口内的统计，对装甲板 ID、位置进行平滑。
常见逻辑如：出现次数最多的 ID 作为当前真实目标，或对角度/距离做时间加权平均。
3. 目标建模与跟踪
target.cpp / target.hpp
抽象“当前锁定的目标”：
存储目标在相机/世界坐标系下的位置、速度、加速度。
存储所属装甲板、机器人 ID、阵营信息。
通常是 tracker 的跟踪单元，也是 planner、solver 的输入对象。
tracker.cpp / tracker.hpp
对装甲板/目标进行时间上的跟踪：
利用卡尔曼滤波、匀速/加速度模型等方法预测下一帧位置。
实现“丢失、重识别、目标切换”等逻辑。
核心功能：
在目标消失短时间内仍能给出平滑、连续的目标估计。
为 MPC 预测提供更稳定的输入。
4. 位姿解算与弹道补偿
solver.cpp / solver.hpp
负责从像素坐标转到空间坐标，并补偿弹道：
PnP 求解目标在相机坐标系下的 3D 位姿（使用相机内参、畸变、装甲板物理尺寸）。
根据云台姿态（俯仰/偏航）、枪口位置等转换到世界或云台坐标。
考虑子弹初速、重力、空气阻力（如有）进行抛物线补偿，给出推荐的瞄准角度。
输出通常为：
期望云台 yaw/pitch。
目标距离/高度等信息，用于 planner 和 shooter。
5. YOLO 检测模块
yolo.cpp / yolo.hpp
对 YOLO 系列装甲板检测进行统一封装：
加载 ONNX 权重。
前向推理，输出装甲板候选框。
可能还包括后处理（NMS、坐标变换）。
具体的不同版本实现放在 yolos/ 子目录。
yolos/ 目录
yolo11.cpp/hpp
yolov5.cpp/hpp
yolov8.cpp/hpp
这些文件分别封装了对应版本网络的细节差异，例如：

不同的输入尺寸、预处理（归一化、色彩空间等）。
网络输出 tensor 的解析逻辑（anchor-free/anchor-based、输出层布局等）。
用统一的接口将结果转为通用装甲板候选框列表（供 voter、tracker 等模块使用）。
6. 多线程模块 multithread/
mt_detector.cpp / mt_detector.hpp
多线程版本的装甲板检测：
将检测过程（YOLO 推理 + 传统视觉）放到独立线程执行。
降低对主线程的阻塞，使系统具有更高帧率/更好实时性。
一般提供：
启动/停止检测线程的接口。
线程安全的结果获取方式（互斥锁、环形缓冲等）。
commandgener.cpp / commandgener.hpp
“命令生成器”，从多线程检测/跟踪结果生成对云台/发射机构的控制指令：
根据当前模式（自瞄/小能量机关等）决定指令形式。
打包为通信协议数据结构（如 yaw/pitch 角度、是否开火、目标 ID 等）。
属于“连接视觉与电控通信”的一部分。
7. 运动规划与 MPC planner/
planner.cpp / planner.hpp
负责目标运动预测和云台控制量规划：
根据 tracker/target 提供的位置、速度等状态，构建 MPC 优化问题。
约束云台角速度、角加速度等硬件限制。
输出未来若干时刻的参考轨迹和当前时刻应执行的控制量。
与 tinyMPC 子模块紧密耦合。
planner/tinympc/ 子目录
该目录是轻量级 MPC 求解库的实现（本工程内置），主要文件功能如下：

admm.cpp/hpp
使用 ADMM（Alternating Direction Method of Multipliers）算法求解 MPC 中的二次规划问题。

tiny_api.cpp/hpp
对外提供简洁的 C++ API：初始化 MPC、设置系统模型、求解控制量等。

types.hpp
定义 MPC 中用到的数据类型、结构体（状态向量、控制向量、代价矩阵等）。

tiny_api_constants.hpp
一些固定参数/常量配置，如最大维度、默认迭代次数等。

codegen.cpp/hpp
可能用于根据系统模型自动生成部分矩阵或代码，以减少运行时计算。

rho_benchmark.cpp/hpp
与 ADMM 参数 rho 调优和性能测试相关，用于基准评估。

error.hpp
定义 MPC 求解过程中的错误码和异常处理。

CMakeLists.txt
定义 tinympc 子库的编译规则。

整体上，planner/ + tinympc/ 提供了一个可配置的 MPC 预测控制模块，用于更精准地控制云台跟踪高速目标。

8. CMake 构建
CMakeLists.txt（auto_aim 下）
定义本子模块的库/目标：
收集上述 .cpp 文件，编译成静态或动态库。
链接依赖：OpenCV、Eigen、ONNXRuntime、yaml-cpp、tinympc 等（具体依赖需查看文件内容）。
可能还会：
设置包含目录供 standard.cpp、standard_mpc.cpp 等主程序使用。
为测试或 demo 可执行文件增加 target。
9. 模块协同流程概述
综合以上模块，一个典型的单帧处理流程大致如下（简化描述）：

相机采集图像，传入 aimer / mt_detector。
在 multithread 或主线程中：
YOLO (yolo* / yolos/) 或传统 detector 对图像做装甲板检测。
classifier 识别装甲板数字/类别。
voter 在多帧、多检测结果间进行投票筛选。
tracker 接收本帧目标，更新并预测目标状态，输出 target。
solver 利用 target + 相机/云台标定参数，进行 PnP 解算与弹道补偿，得到期望瞄准角。
planner + tinympc 根据目标动态和云台约束求解一串最优控制量，提供更加平滑的角度/速度指令。
aimer 综合各模块输出，更新当前瞄准状态。
shooter 根据命中概率、冷却/弹速等条件决策是否开火，将指令通过通信协议发给电控。