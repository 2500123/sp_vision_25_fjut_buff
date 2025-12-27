.gitkeep

占位文件，用于保证空目录也能被 Git 跟踪。
auto_aim_debug_mpc.cpp
自瞄（装甲板）调试版，带 MPC（Model Predictive Control）控制逻辑或预测模块的变体。
常用于验证自瞄目标定位、姿态解算、以及带 MPC 的控制器响应与参数。

auto_buff_debug_mpc.cpp
“能量机关/打符”场景的调试版，结合 MPC 的版本。
侧重圆周/周期目标（旋转扇叶/符）的几何拟合、相位预测与射击时机决策的验证。

auto_buff_debug.cpp
“能量机关/打符”的非 MPC 调试版。
更适合对检测/几何筛选/相位估计等基础流程做可视化与快速调参。

mt_auto_aim_debug.cpp
自瞄调试的多线程版本（mt = multithread）。
典型做法是将相机采集、检测/识别、跟踪/EKF、解算/控制拆成多个线程或流水线阶段，提高帧率与延迟表现。

mt_standard.cpp
标准模式的多线程版本。
与 standard.cpp 的功能相近，但采用并行/流水线结构，适合在算力有限设备上提升实时性。

sentry.cpp
哨兵模式主程序（单线程/常规版）。
面向固定点防御/巡逻的场景，包含哨兵策略、目标优先级、扇区扫描等逻辑。

sentry_bp.cpp
哨兵模式的一个“bp”变体分支（具体含义由项目定义，常见是策略/弹道预测/基准方案等变更）。
用于在哨兵场景下验证另一套策略或组件组合。

sentry_debug.cpp
哨兵模式的调试版，增强日志/可视化与参数试验。
便于快速定位策略问题或识别/定位误差。

sentry_multithread.cpp
哨兵模式的多线程版本。
将采集/识别/策略/下发拆分并行，提高低延迟与稳定性。

standard_mpc.cpp
标准模式的 MPC 版本。
比 standard.cpp 多了 MPC 相关预测/控制模块，适合需要更平滑/鲁棒控制的场合。

standard.cpp
标准自瞄模式主程序（常规版）。
典型流程：读取配置 → 初始化相机/IMU/CAN/串口 → 检测装甲板 → PnP解算位姿（Solver）→ 目标融合/跟踪 → 瞄准弹道（Aimer）→ 下发控制（io::*）。

uav_debug.cpp
无人机/步兵空中模式的调试版。
适配 UAV 特性（视角、运动模型、控制限幅等）的调试入口。

uav.cpp
UAV 模式主程序（常规版）。
集成 UAV 运动模型、目标预测与控制下发，适配空中载具的控制接口与坐标系。

MPC 是什么
MPC（模型预测控制，Model Predictive Control）是一种基于系统模型的优化控制方法。
它在每个控制周期上，预测未来一段时间内系统的行为，在满足约束（如角速度/加速度/输出范围）的前提下，求解出使代价最小的控制输入序列，并仅执行当前步的输入，随后滚动优化。
优点：能显式处理约束，轨迹跟随更平滑、超调更小、抗扰动更好。
在本项目中，带有 “_mpc” 后缀的入口程序（例如 standard_mpc.cpp、auto_aim_debug_mpc.cpp、auto_buff_debug_mpc.cpp）就是包含 MPC 思路或相关求解的变体；README 也提到在轨迹规划/控制里用过 MPC 或类似的加速度约束优化。
src 中是否有使用 MicroUSB 虚拟串口的程序
有。项目支持用 MicroUSB 虚拟串口与下位机通信（README 3.1 明确写了“通信方式：USB2CAN（旧）、MicroUSB虚拟串口（新）”，并提供了“串口设置”的 udev 规则）。
具体实现位于 serial 模块；应用层的所有入口程序（src 下的 standard.cpp、sentry*.cpp、uav*.cpp、auto_* 等）都是通过统一的 IO 抽象来和下位机通信，链接时已经把 serial 库并入了 io（见 CMakeLists.txt：add_subdirectory(serial) 且 target_link_libraries(io ... serial ...)）。
运行时，这些程序会通过你配置的串口设备与下位机通信（例如 /dev/ttyACM0，建议按 README 3.2 的“7. 串口设置”为它创建固定别名 /dev/gimbal），端口名通常在 configs/*.yaml 中配置。
如果你需要，我可以帮你：

确认你当前使用的串口设备名是否生效（/dev/gimbal 是否存在）；
在某个具体 src/*.cpp 入口里加一行启动日志，打印实际使用的串口路径，便于排查连接问题。