// 大恒相机适配层
#ifndef IO__DAHENG_HPP
#define IO__DAHENG_HPP

#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

// Galaxy SDK
#include "include/GxIAPI.h"
#include "include/DxImageProc.h"

namespace io
{
class Daheng : public CameraBase
{
public:
	// 简化构造：仅设定曝光、增益与可选 USB vid:pid（用于掉线时 reset）
	Daheng(double exposure_ms, double gain, const std::string & vid_pid);
	~Daheng() override;
	void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

private:
	struct CameraData
	{
		cv::Mat img;
		std::chrono::steady_clock::time_point timestamp;
	};

	double exposure_ms_;
	double gain_;

	std::thread daemon_thread_;
	std::atomic<bool> quit_;
	std::atomic<bool> ok_;

	GX_DEV_HANDLE handle_ = nullptr;

	std::thread capture_thread_;
	tools::ThreadSafeQueue<CameraData> queue_;

	int vid_ = -1, pid_ = -1;

	void try_open();
	void open();
	void close();
	void set_vid_pid(const std::string & vid_pid);
	void reset_usb() const;
};

}  // namespace io

#endif  // IO__DAHENG_HPP

