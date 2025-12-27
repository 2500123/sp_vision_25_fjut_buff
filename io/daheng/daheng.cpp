#include "daheng.hpp"

#include <libusb-1.0/libusb.h>
#include <stdexcept>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

namespace io
{

static void gx_check(GX_STATUS st, const char *what)
{
  if (st == GX_STATUS_SUCCESS) return;
  GX_STATUS err = GX_STATUS_SUCCESS;
  size_t size = 0;
  GXGetLastError(&err, nullptr, &size);
  std::string msg(size, '\0');
  if (size > 0) GXGetLastError(&err, msg.data(), &size);
  tools::logger()->warn("{} failed: {} ({})", what, (int)st, msg);
  throw std::runtime_error(std::string(what) + " failed");
}

Daheng::Daheng(double exposure_ms, double gain, const std::string & vid_pid)
: exposure_ms_(exposure_ms), gain_(gain), quit_(false), ok_(false), queue_(1)
{
  set_vid_pid(vid_pid);
  if (libusb_init(NULL)) tools::logger()->warn("Unable to init libusb!");

  // Init Galaxy lib once
  gx_check(GXInitLib(), "GXInitLib");

  try_open();

  // 守护线程：断流后重启相机并重置USB
  daemon_thread_ = std::thread{[this] {
    tools::logger()->info("Daheng's daemon thread started.");
    while (!quit_) {
      std::this_thread::sleep_for(100ms);
      if (ok_) continue;

      if (capture_thread_.joinable()) capture_thread_.join();
      close();
      reset_usb();
      try_open();
    }
    tools::logger()->info("Daheng's daemon thread stopped.");
  }};
}

Daheng::~Daheng()
{
  quit_ = true;
  if (daemon_thread_.joinable()) daemon_thread_.join();
  if (capture_thread_.joinable()) capture_thread_.join();
  close();
  GXCloseLib();
  tools::logger()->info("Daheng destructed.");
}

void Daheng::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  CameraData data;
  queue_.pop(data);
  img = data.img;
  timestamp = data.timestamp;
}

void Daheng::open()
{
  // 更新设备列表并打开第一个设备（后续可基于 vid/pid 过滤）
  uint32_t num = 0;
  gx_check(GXUpdateAllDeviceList(&num, 1000), "GXUpdateAllDeviceList");
  if (num == 0) throw std::runtime_error("Not found Daheng camera!");

  GX_OPEN_PARAM open_param = {};
  open_param.accessMode = GX_ACCESS_EXCLUSIVE;
  open_param.openMode = GX_OPEN_INDEX;
  open_param.pszContent = (char *)"1";  // 打开第1个

  gx_check(GXOpenDevice(&open_param, &handle_), "GXOpenDevice");

  // 设置曝光/增益/像素格式/触发为连续
  // 曝光时间单位根据相机，一般为微秒
  gx_check(GXSetEnumValue(handle_, "ExposureAuto", 0), "GXSetEnumValue(ExposureAuto)" );
  gx_check(GXSetFloatValue(handle_, "ExposureTime", exposure_ms_ * 1000.0), "GXSetFloatValue(ExposureTime)");
  // 增益
  GX_STATUS st = GXSetFloatValue(handle_, "Gain", gain_);
  if (st != GX_STATUS_SUCCESS) {
    // 有的型号没有 Gain float，降级到 AnalogGain or DigitalGain
    GXSetFloatValue(handle_, "AnalogGain", gain_);
  }
  // 连续采集
  gx_check(GXSetEnumValue(handle_, "TriggerMode", 0), "GXSetEnumValue(TriggerMode)");

  gx_check(GXStreamOn(handle_), "GXStreamOn");

  // 采集线程
  capture_thread_ = std::thread{[this] {
    tools::logger()->info("Daheng's capture thread started.");
    ok_ = true;

    while (!quit_) {
      std::this_thread::sleep_for(1ms);

      PGX_FRAME_BUFFER pframe = nullptr;
      GX_STATUS st = GXDQBuf(handle_, &pframe, 100);
      if (st != GX_STATUS_SUCCESS) {
        tools::logger()->warn("GXDQBuf timeout or error: {}", (int)st);
        ok_ = false;
        break;
      }

      auto timestamp = std::chrono::steady_clock::now();

      int w = pframe->nWidth;
      int h = pframe->nHeight;
      int pix = pframe->nPixelFormat;

      cv::Mat img;
      switch (pix) {
        case GX_PIXEL_FORMAT_RGB8:
        case GX_PIXEL_FORMAT_BGR8: {
          img = cv::Mat(h, w, CV_8UC3, pframe->pImgBuf).clone();
          break;
        }
        case GX_PIXEL_FORMAT_BAYER_RG8:
        case GX_PIXEL_FORMAT_BAYER_GR8:
        case GX_PIXEL_FORMAT_BAYER_GB8:
        case GX_PIXEL_FORMAT_BAYER_BG8: {
          cv::Mat raw(h, w, CV_8UC1, pframe->pImgBuf);
          int code = cv::COLOR_BayerRG2RGB;
          if (pix == GX_PIXEL_FORMAT_BAYER_GR8) code = cv::COLOR_BayerGR2RGB;
          else if (pix == GX_PIXEL_FORMAT_BAYER_GB8) code = cv::COLOR_BayerGB2RGB;
          else if (pix == GX_PIXEL_FORMAT_BAYER_BG8) code = cv::COLOR_BayerBG2RGB;
          cv::cvtColor(raw, img, code);
          break;
        }
        default:
          tools::logger()->warn("Unsupported pixel format: {}. Please set camera to 8-bit Bayer or RGB8/BGR8.", pix);
          img = cv::Mat();
          break;
      }


      queue_.push({img, timestamp});

      GX_STATUS st2 = GXQBuf(handle_, pframe);
      if (st2 != GX_STATUS_SUCCESS) {
        tools::logger()->warn("GXQBuf failed: {}", (int)st2);
        ok_ = false;
        break;
      }
    }

    tools::logger()->info("Daheng's capture thread stopped.");
  }};

  tools::logger()->info("Daheng opened.");
}

void Daheng::try_open()
{
  try {
    open();
  } catch (const std::exception & e) {
    tools::logger()->warn("{}", e.what());
  }
}

void Daheng::close()
{
  if (!handle_) return;
  GXStreamOff(handle_);
  GXCloseDevice(handle_);
  handle_ = nullptr;
}

void Daheng::set_vid_pid(const std::string & vid_pid)
{
  auto index = vid_pid.find(':');
  if (index == std::string::npos) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
    return;
  }

  auto vid_str = vid_pid.substr(0, index);
  auto pid_str = vid_pid.substr(index + 1);

  try {
    vid_ = std::stoi(vid_str, 0, 16);
    pid_ = std::stoi(pid_str, 0, 16);
  } catch (const std::exception &) {
    tools::logger()->warn("Invalid vid_pid: \"{}\"", vid_pid);
  }
}

void Daheng::reset_usb() const
{
  if (vid_ == -1 || pid_ == -1) return;
  auto handle = libusb_open_device_with_vid_pid(NULL, vid_, pid_);
  if (!handle) {
    tools::logger()->warn("Unable to open usb!");
    return;
  }
  if (libusb_reset_device(handle))
    tools::logger()->warn("Unable to reset usb!");
  else
    tools::logger()->info("Reset usb successfully :)");
  libusb_close(handle);
}



}  // namespace io
