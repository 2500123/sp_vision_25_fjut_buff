#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"
#include <sstream>
#include <iomanip>

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(921600);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(20);
    serial_.setTimeout(time_out);
    serial_.open();
    
    tools::logger()->info("[Gimbal] Serial port opened: port={}, baudrate=921600", com_port);
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  // Prefer cached samples (non-blocking). If we don't have enough data, fall back to last known q.
  Eigen::Quaterniond q0, q1;
  std::chrono::steady_clock::time_point t0, t1;
  bool has0 = false;
  bool has1 = false;
  {
    std::lock_guard<std::mutex> lock(q_mutex_);
    has1 = has_last_q_;
    has0 = has_prev_q_;
    q1 = last_q_;
    t1 = last_q_time_;
    q0 = prev_q_;
    t0 = prev_q_time_;
  }

  if (!has1) {
    return Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  }
  if (!has0) {
    return q1;
  }

  // If the requested timestamp is outside the cached interval, just return the nearest.
  if (t <= t0) return q0;
  if (t >= t1) return q1;

  const double dt01 = tools::delta_time(t0, t1);
  if (std::abs(dt01) < 1e-6) return q1;

  const double dt0t = tools::delta_time(t0, t);
  const double k = std::clamp(dt0t / dt01, 0.0, 1.0);
  return q0.slerp(k, q1).normalized();
}

double Gimbal::q_age_ms(std::chrono::steady_clock::time_point now) const
{
  std::lock_guard<std::mutex> lock(q_mutex_);
  if (!has_last_q_) return std::numeric_limits<double>::infinity();
  return std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(now - last_q_time_)
    .count();
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  std::lock_guard<std::mutex> lock(tx_mutex_);
  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  std::lock_guard<std::mutex> lock(tx_mutex_);
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;
  bool frame_synced = false;  // 是否已同步到帧头

  while (!quit_) {
    if (error_count > 50) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      frame_synced = false;
      reconnect();
      continue;
    }

    // 帧同步：逐字节读，直到找到 'SP' 帧头
    if (!frame_synced) {
      uint8_t byte1 = 0;
      // 循环读直到找到 'S' (0x53)
      while (!quit_) {
        if (!read(&byte1, 1)) {
          error_count++;
          break;
        }
        if (byte1 == 'S') {
          // 找到可能的帧头第一个字节，读第二个字节
          uint8_t byte2 = 0;
          if (read(&byte2, 1)) {
            if (byte2 == 'P') {
              // 找到完整的 'SP' 帧头
              rx_data_.head[0] = 'S';
              rx_data_.head[1] = 'P';
              frame_synced = true;
              break;
            } else {
              // 不是 'P'，继续找下一个 'S'
              byte1 = byte2;  // 检查这个字节是否是 'S'
              if (byte1 != 'S') {
                continue;  // 继续读下一个字节
              }
            }
          } else {
            error_count++;
            break;
          }
        }
      }
      
      if (!frame_synced) {
        error_count++;
        continue;
      }
    }

    auto t = std::chrono::steady_clock::now();

    // 帧已同步，读剩余的数据（41 字节）
    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
          sizeof(rx_data_) - sizeof(rx_data_.head))) {
      error_count++;
      frame_synced = false;
      continue;
    }

    // 检查 CRC
    // MCU 发送的是小端（低字节在前，高字节在后）
    uint16_t recv_crc = rx_data_.crc16;
    uint16_t calc_crc = tools::get_crc16(
      reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_) - sizeof(rx_data_.crc16));

    if (recv_crc != calc_crc) {
      error_count++;
      frame_synced = false;
      continue;
    }

    error_count = 0;
    frame_synced = false;  // 重置帧同步标志，为下一帧做准备
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);

    {
      std::lock_guard<std::mutex> q_lock(q_mutex_);
      if (has_last_q_) {
        prev_q_ = last_q_;
        prev_q_time_ = last_q_time_;
        has_prev_q_ = true;
      }
      last_q_ = q;
      last_q_time_ = t;
      has_last_q_ = true;
    }

    queue_.push({q, t});

    std::lock_guard<std::mutex> lock(mutex_);

    state_.yaw = rx_data_.yaw;
    state_.yaw_vel = rx_data_.yaw_vel;
    state_.pitch = rx_data_.pitch;
    state_.pitch_vel = rx_data_.pitch_vel;
    state_.bullet_speed = rx_data_.bullet_speed;
    state_.bullet_count = rx_data_.bullet_count;

    switch (rx_data_.mode) {
      case 0:
        mode_ = GimbalMode::IDLE;
        break;
      case 1:
        mode_ = GimbalMode::AUTO_AIM;
        break;
      case 2:
        mode_ = GimbalMode::SMALL_BUFF;
        break;
      case 3:
        mode_ = GimbalMode::BIG_BUFF;
        break;
      default:
        mode_ = GimbalMode::IDLE;
        tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
        break;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io