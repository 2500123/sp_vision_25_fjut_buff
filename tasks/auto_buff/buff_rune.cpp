#include "buff_rune.hpp"

#include "tools/logger.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace auto_buff
{
namespace {
constexpr int kMlpInputSize = 28;
}

Buff_Rune::Buff_Rune(const std::string & config)
{
	// Defaults
	model_path_ = "assets/tiny_resnet.onnx";
	input_color = RED;

	// Load config (best-effort)
	try {
		const YAML::Node cfg = YAML::LoadFile(config);
		if (cfg["min_threshold"]) min_threshold = cfg["min_threshold"].as<int>();
		if (cfg["bin_threshold_target"]) bin_threshold_target = cfg["bin_threshold_target"].as<int>();
		if (cfg["min_area"]) min_area = cfg["min_area"].as<int>();
		if (cfg["variance_threshold"]) variance_threshold = cfg["variance_threshold"].as<int>();
		if (cfg["model_path"]) model_path_ = cfg["model_path"].as<std::string>();
		if (cfg["color"]) {
			const std::string c = cfg["color"].as<std::string>();
			input_color = (c == "BLUE" || c == "blue") ? BLUE : RED;
		}
	} catch (const std::exception & e) {
		tools::logger()->warn(std::string("Buff_Rune config load failed: ") + e.what());
	} catch (...) {
		tools::logger()->warn("Buff_Rune config load failed: unknown error");
	}

	// Load model (best-effort)
	try {
		mlp_net_ = cv::dnn::readNet(model_path_);
		if (mlp_net_.empty()) {
			tools::logger()->error("Failed to load model: " + model_path_);
		}
	} catch (const cv::Exception & e) {
		tools::logger()->error(std::string("OpenCV dnn error loading model: ") + e.what());
	}
}

cv::Mat Buff_Rune::letterbox(
	const cv::Mat & src,
	int target_w,
	int target_h,
	cv::Scalar pad_color)
{
	if (src.empty() || src.cols <= 0 || src.rows <= 0 || target_w <= 0 || target_h <= 0) {
		return {};
	}
	const int src_w = src.cols;
	const int src_h = src.rows;
	const float scale = std::min(target_w / float(src_w), target_h / float(src_h));
	const int new_w = int(std::round(src_w * scale));
	const int new_h = int(std::round(src_h * scale));
	if (new_w <= 0 || new_h <= 0) {
		return {};
	}

	cv::Mat resized;
	cv::resize(src, resized, cv::Size(new_w, new_h));

	cv::Mat out(target_h, target_w, src.type(), pad_color);
	const int x = (target_w - new_w) / 2;
	const int y = (target_h - new_h) / 2;
	resized.copyTo(out(cv::Rect(x, y, new_w, new_h)));
	return out;
}

float Buff_Rune::calculateVariance(const std::vector<cv::Point2f> & points)
{
	if (points.empty()) return 0.0f;

	cv::Point2f mean(0, 0);
	for (const auto & pt : points) {
		mean.x += pt.x;
		mean.y += pt.y;
	}
	mean.x /= static_cast<float>(points.size());
	mean.y /= static_cast<float>(points.size());

	float var_x = 0.0f;
	float var_y = 0.0f;
	for (const auto & pt : points) {
		const float dx = pt.x - mean.x;
		const float dy = pt.y - mean.y;
		var_x += dx * dx;
		var_y += dy * dy;
	}
	return (var_x + var_y) / static_cast<float>(points.size());
}

std::vector<cv::Point2f> Buff_Rune::sortQuadPoints(
	const std::vector<cv::Point2f> & points,
	const cv::Point2f & O,
	const cv::Point2f & A)
{
	if (points.size() != 4) {
		std::cerr << "必须提供4个点。" << std::endl;
		return {};
	}

	std::vector<float> angles;
	angles.reserve(points.size());
	for (const auto & pt : points) {
		const float dx = pt.x - O.x;
		const float dy = pt.y - O.y;
		float angle = std::atan2(dy, dx);
		if (angle < 0) angle += 2 * CV_PI;
		angles.push_back(angle);
	}

	std::vector<size_t> indices = {0, 1, 2, 3};
	std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) { return angles[a] < angles[b]; });

	std::vector<cv::Point2f> sortedPoints;
	sortedPoints.reserve(points.size());
	for (auto i : indices) {
		sortedPoints.push_back(points[i]);
	}

	int k = 0;
	float minDist = std::numeric_limits<float>::max();
	for (int i = 0; i < 4; ++i) {
		const float dist = static_cast<float>(cv::norm(sortedPoints[i] - A));
		if (dist < minDist) {
			minDist = dist;
			k = i;
		}
	}
	std::rotate(sortedPoints.begin(), sortedPoints.begin() + k, sortedPoints.end());
	return sortedPoints;
}

bool Buff_Rune::preprocess(const cv::Mat & input_img, cv::Mat & r_center_roi_img)
{
	// Default behavior: if we can't preprocess this frame, fall back to the original image.
	r_center_roi_img.release();
	if (input_img.empty()) {
		tools::logger()->debug("preprocess: input_img empty");
		return false;
	}
	r_center_roi_img = input_img.clone();

	try {
		filtered_intersections.clear();
		intersections.clear();
		square_points.clear();
		four_points.clear();

		if (mlp_net_.empty()) {
			tools::logger()->error("preprocess: mlp_net_ empty; model not loaded: " + model_path_);
			return false;
		}

	// Reset per-frame state
	max_r_logo_prob = -1.0f;
	max_target_prob = -1.0f;
	max_r_logo_center = cv::Point2f(-1, -1);
	max_target_center = cv::Point2f(-1, -1);
	r_logo_roi = cv::Rect();
	target_roi = cv::Rect();

	// ---- Stage 1: full-image preprocessing + MLP classification (former preprocess_1) ----
	cv::Mat gray_img;
	if (input_img.channels() == 3) {
		cv::cvtColor(input_img, gray_img, cv::COLOR_BGR2GRAY);
	} else if (input_img.channels() == 4) {
		cv::cvtColor(input_img, gray_img, cv::COLOR_BGRA2GRAY);
	} else if (input_img.channels() == 1) {
		gray_img = input_img;
	} else {
		tools::logger()->debug("preprocess: unsupported channels");
		return false;
	}

	cv::Mat binary;
	cv::threshold(gray_img, binary, min_threshold, 255, cv::THRESH_BINARY);
	if (binary.empty()) {
		return false;
	}

	const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
	cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

	std::vector<std::vector<cv::Point>> contours_;
	cv::findContours(binary, contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	if (contours_.empty()) {
		return false;
	}

	const cv::Rect full_rect(0, 0, input_img.cols, input_img.rows);

	for (const auto & contour : contours_) {
		const cv::RotatedRect rect = cv::minAreaRect(contour);
		const double w = rect.size.width;
		const double h = rect.size.height;
		if (w <= 0.0 || h <= 0.0) continue;
		double aspect_ratio = w / h;
		aspect_ratio = (aspect_ratio < 1.0) ? 1.0 / aspect_ratio : aspect_ratio;
		const double area = cv::contourArea(contour);

		if (aspect_ratio > 1.5 || area < min_area) {
			continue;
		}

		const cv::Rect bounding_rect = (cv::boundingRect(contour) & full_rect);
		if (bounding_rect.width <= 0 || bounding_rect.height <= 0) continue;

		// Color filter
		const cv::Scalar mean_color = cv::mean(input_img(bounding_rect));
		if ((input_color == RED && mean_color[0] > mean_color[2]) ||
				(input_color == BLUE && mean_color[2] > mean_color[0])) {
			continue;
		}

		cv::Mat orig_bin = binary(bounding_rect).clone();
		if (orig_bin.empty()) continue;

		cv::Mat roi = this->letterbox(orig_bin, kMlpInputSize, kMlpInputSize, cv::Scalar(0));
		if (roi.empty()) continue;
		if (roi.channels() == 3) cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);

		cv::Mat input_blob = cv::dnn::blobFromImage(
			roi,
			1.0 / 255.0,
			cv::Size(kMlpInputSize, kMlpInputSize),
			cv::Scalar(),
			true,
			false);
		mlp_net_.setInput(input_blob);
		cv::Mat output = mlp_net_.forward();
		if (output.empty() || output.total() < 5) continue;

		float * floatarr = output.ptr<float>();
		std::vector<float> outputs(floatarr, floatarr + 5);
		const int max_index = int(std::max_element(outputs.begin(), outputs.end()) - outputs.begin());
		if (outputs[max_index] < 0) continue;

		const cv::Moments moment = cv::moments(contour);
		if (moment.m00 == 0) continue;
		const cv::Point2f center(moment.m10 / moment.m00, moment.m01 / moment.m00);

		switch (max_index) {
			case 0:  // R
				if (outputs[max_index] > max_r_logo_prob) {
					max_r_logo_prob = outputs[max_index];
					r_logo_roi = cv::boundingRect(contour);
					max_r_logo_center = center;
				}
				break;
			case 3:  // T
				if (outputs[max_index] > max_target_prob) {
					max_target_prob = outputs[max_index];
					target_roi = cv::boundingRect(contour);
					max_target_center = center;
				}
				break;
			default:
				break;
		}
	}

	if (max_r_logo_center == cv::Point2f(-1, -1)) {
		return false;
	}

	// Expand ROI (target ROI) if possible; used for estimating radius.
	if (max_target_center != cv::Point2f(-1, -1) && target_roi.width > 0 && target_roi.height > 0) {
		static constexpr int expand_pix = 5;
		cv::Rect expanded_roi = target_roi;
		expanded_roi.x -= expand_pix;
		expanded_roi.y -= expand_pix;
		expanded_roi.width += expand_pix * 2;
		expanded_roi.height += expand_pix * 2;
		expanded_roi &= full_rect;
		target_roi = expanded_roi;
	}

	// ---- Stage 2: ROI skeleton + intersections (former preprocess_2) ----
	float radius = -1.0f;
	if (target_roi.width > 0 && target_roi.height > 0) {
		cv::Mat target_roi_img = gray_img(target_roi).clone();
		if (!target_roi_img.empty()) {
			cv::GaussianBlur(target_roi_img, target_roi_img, cv::Size(5, 5), 0);
			cv::threshold(target_roi_img, target_roi_img, bin_threshold_target, 255, cv::THRESH_BINARY);

			cv::Mat roi_work = target_roi_img.clone();
			cv::Mat goal_roi_mask = roi_work.clone();
			if (!goal_roi_mask.empty() && !roi_work.empty()) {
				cv::ellipse(
					goal_roi_mask,
					cv::Point2f(roi_work.cols / 2.0f, roi_work.rows / 2.0f),
					cv::Size(roi_work.cols / 2, roi_work.rows / 2),
					0,
					0,
					360,
					cv::Scalar(0),
					-1);
				roi_work.setTo(cv::Scalar(0), goal_roi_mask);
			}

			cv::Mat skeleton_roi;
			cv::ximgproc::thinning(roi_work, skeleton_roi, cv::ximgproc::THINNING_GUOHALL);
			if (!skeleton_roi.empty()) {
				for (int y = 1; y < skeleton_roi.rows - 1; y++) {
					for (int x = 1; x < skeleton_roi.cols - 1; x++) {
						if (skeleton_roi.at<uchar>(y, x) == 255) {
							int neighbors = 0;
							for (int dy = -1; dy <= 1; dy++) {
								for (int dx = -1; dx <= 1; dx++) {
									if (dy == 0 && dx == 0) continue;
									neighbors += (skeleton_roi.at<uchar>(y + dy, x + dx) == 255);
								}
							}
							if (neighbors >= 3) intersections.push_back(cv::Point2f(x, y));
						}
					}
				}

				for (size_t i = 0; i < intersections.size(); i++) {
					bool is_duplicate = false;
					for (size_t j = 0; j < i; j++) {
						if (cv::norm(intersections[i] - intersections[j]) < 5.0) {
							is_duplicate = true;
							break;
						}
					}
					if (!is_duplicate) filtered_intersections.push_back(intersections[i]);
				}

				intersections = std::move(filtered_intersections);
				filtered_intersections.clear();

				const cv::Point2f orig(target_roi.x, target_roi.y);
				if (intersections.size() >= 4 && max_target_center != cv::Point2f(-1, -1)) {
					std::sort(
						intersections.begin(),
						intersections.end(),
						[orig, this](const cv::Point2f & a, const cv::Point2f & b) {
							return cv::norm(a + orig - this->max_target_center) >
									 cv::norm(b + orig - this->max_target_center);
						});

					four_points.clear();
					for (size_t i = 0; i < 4; i++) {
						four_points.push_back(intersections[i] + orig);
					}

					if (four_points.size() == 4 && this->calculateVariance(four_points) > variance_threshold) {
						square_points = this->sortQuadPoints(four_points, max_target_center, max_r_logo_center);
						if (!square_points.empty()) {
							const float base_dist = static_cast<float>(cv::norm(square_points[0] - max_r_logo_center));
							if (base_dist >= 1.0f) {
								radius = base_dist * 1.7f;
							}
						}
					}
				}
			}
		}
	}

	// Fallback radius if skeleton/intersections path didn't provide it
	if (radius < 1.0f) {
		if (r_logo_roi.width > 0 && r_logo_roi.height > 0) {
			radius = 3.0f * static_cast<float>(std::max(r_logo_roi.width, r_logo_roi.height));
		} else if (target_roi.width > 0 && target_roi.height > 0) {
			radius = 0.6f * static_cast<float>(std::max(target_roi.width, target_roi.height));
		} else {
			radius = 50.0f;
		}
	}

	const int x = static_cast<int>(std::round(max_r_logo_center.x - radius));
	const int y = static_cast<int>(std::round(max_r_logo_center.y - radius));
	const int w = static_cast<int>(std::round(radius * 2.0f));
	const int h = w;
	if (w <= 0 || h <= 0) {
		return false;
	}

	cv::Rect roi(x, y, w, h);
	roi &= cv::Rect(0, 0, input_img.cols, input_img.rows);
	if (roi.width <= 0 || roi.height <= 0) {
		return false;
	}

	cv::Mat roi_img = input_img(roi);
	if (roi_img.empty()) {
		return false;
	}
	r_center_roi_img = roi_img.clone();
	return true;
	} catch (const cv::Exception & e) {
		tools::logger()->warn(std::string("preprocess: OpenCV exception: ") + e.what());
		r_center_roi_img = input_img.clone();
		return false;
	} catch (const std::exception & e) {
		tools::logger()->warn(std::string("preprocess: exception: ") + e.what());
		r_center_roi_img = input_img.clone();
		return false;
	} catch (...) {
		tools::logger()->warn("preprocess: unknown exception");
		r_center_roi_img = input_img.clone();
		return false;
	}
}


}  // namespace auto_buff

