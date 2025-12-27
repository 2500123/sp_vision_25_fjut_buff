用串口通路就选 io/gimbal/Gimbal 这套类，而不是 CAN 的 io/CBoard。
Gimbal 构造函数会从配置里读取串口端口名 com_port，然后用 serial::Serial 打开它。
所以你只需要在配置 YAML 里把 com_port 设置为你的 C 板串口设备即可。
最小配置步骤

给 C 板串口赋权限
将当前用户加入 dialout 组（一次性操作，执行后需重新登录）
sudo usermod -a -G dialout $USER
确认设备名
插上 C 板 MicroUSB 后，常见设备名类似 /dev/ttyACM0 或 /dev/ttyUSB0。
建议按 README 的“串口设置”创建 udev 规则，固定别名为 /dev/gimbal，避免每次变动。
配置 com_port
在你的配置文件（例如 standard3.yaml 或你实际运行的 YAML）里设置：
com_port: "/dev/gimbal"
Gimbal 的代码会用 tools::load(config_path) 读取这个字段：
auto com_port = tools::readstd::string(yaml, "com_port");
serial_.setPort(com_port); serial_.open();
运行时验证
程序启动后会打印
[Gimbal] read_thread started.
[Gimbal] First q received.
如果打开失败，会报 “[Gimbal] Failed to open serial: ...” 并退出；
运行中断连会自动重连，看到 “[Gimbal] Too many errors, attempting to reconnect...” 然后尝试重开串口。
常见问题排查

打不开串口：权限不足（按步骤1处理），设备名不对（核查 /dev/ 下名称），线缆/供电问题。
串口有但没有姿态：下位机未按协议发送，或帧头/CRC 不匹配。Gimbal 期望的上行帧结构是 GimbalToVision（头字节 'S','P' + 数据 + CRC16）。
端口随机变化：用 udev 规则固定 /dev/gimbal，你的 YAML 就能一直写同一个路径。
总结

串口是 C 板就用 Gimbal（io/gimbal/gimbal.hpp/.cpp），把 YAML 的 com_port 指到你的设备（推荐 /dev/gimbal）。如果需要，我可以帮你在某个具体 YAML 里补上这一项，或者加一条启动日志，开机打印实际串口路径，便于检查。
