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

Buff_Rune::Buff_Rune(const std::string & config){
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
    } catch (const std::exception &e) {
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
    } catch (const cv::Exception &e) {
        tools::logger()->error(std::string("OpenCV dnn error loading model: ") + e.what());
    }
}

cv::Mat Buff_Rune::letterbox(const cv::Mat& src, int target_w, int target_h, cv::Scalar pad_color) {
    if (src.empty() || src.cols <= 0 || src.rows <= 0 || target_w <= 0 || target_h <= 0) {
        return {};
    }
    const int src_w = src.cols;
    const int src_h = src.rows;
    const float scale = std::min(target_w / float(src_w), target_h / float(src_h));
    const int new_w = int(src_w * scale);
    const int new_h = int(src_h * scale);

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

float Buff_Rune::calculateVariance(const std::vector<cv::Point2f>& points) {
    if (points.empty()) return 0.0f;
    
    cv::Point2f mean(0, 0);
    for (const auto& pt : points) {
        mean.x += pt.x;
        mean.y += pt.y;
    }
    mean.x /= points.size();
    mean.y /= points.size();

    float var_x = 0.0f;
    float var_y = 0.0f;
    for (const auto& pt : points) {
        var_x += std::pow(pt.x - mean.x, 2);
        var_y += std::pow(pt.y - mean.y, 2);
    }
    return (var_x + var_y) / points.size();
}

void Buff_Rune::process_1(const cv::Mat & input_img, cv::Mat & output_img, cv::Mat & target_roi_img)
{
    // Ensure outputs are not left as stale data when early-returning.
    output_img.release();
    target_roi_img.release();

    if (input_img.empty()) {
        tools::logger()->debug("input_img is empty");
        return;
    }

    // Keep output_img available for downstream visualization even if we early-return.
    output_img = input_img.clone();

    if (mlp_net_.empty()) {
        tools::logger()->error("mlp_net_ is empty; model not loaded: " + model_path_);
        return;
    }

    // Reset per-frame state
    max_r_logo_prob = -1.0f;
    max_target_prob = -1.0f;
    max_r_logo_center = cv::Point2f(-1, -1);
    max_target_center = cv::Point2f(-1, -1);
    r_logo_roi = cv::Rect();
    target_roi = cv::Rect();

    cv::Mat gray_img;
    if (input_img.channels() == 3) {
        cv::cvtColor(input_img, gray_img, cv::COLOR_BGR2GRAY);
    } else if (input_img.channels() == 4) {
        cv::cvtColor(input_img, gray_img, cv::COLOR_BGRA2GRAY);
    } else if (input_img.channels() == 1) {
        gray_img = input_img;
    } else {
        tools::logger()->debug("input_img has unsupported channels");
        return;
    }

    cv::Mat binary;
    cv::threshold(gray_img, binary, min_threshold, 255, cv::THRESH_BINARY);

    if (binary.empty()) {
        tools::logger()->debug("binary is empty");
        return;
    }

    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(binary, binary, cv::MORPH_OPEN, kernel);

    std::vector<std::vector<cv::Point>> contours_;
    cv::findContours(binary, contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours_.empty()) {
        // tools::logger()->debug("contour is empty");
        return;
    }

    const cv::Rect full_rect(0, 0, input_img.cols, input_img.rows);

    for (const auto& contour : contours_) {
        const cv::RotatedRect rect = cv::minAreaRect(contour);
        double w = rect.size.width, h = rect.size.height;
        if (h == 0) continue;
        double aspect_ratio = w / h;
        aspect_ratio = (aspect_ratio < 1.0) ? 1.0 / aspect_ratio : aspect_ratio;
        const double area = cv::contourArea(contour);

        if (aspect_ratio > 1.2 || area < min_area) {
            cv::drawContours(binary, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0), cv::FILLED);
            continue;
        }
        cv::Rect bounding_rect = cv::boundingRect(contour) & full_rect;
        if (bounding_rect.width <= 0 || bounding_rect.height <= 0) continue;

        const cv::Mat color_roi = input_img(bounding_rect);
        const cv::Scalar mean_color = cv::mean(color_roi);

        if ((this->input_color == RED && mean_color[0] > mean_color[2]) ||
            (this->input_color == BLUE && mean_color[2] > mean_color[0])) {
            cv::drawContours(binary, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0), cv::FILLED);
            continue;
        }

        const cv::Rect image_roi(0, 0, binary.cols, binary.rows);
        const cv::Rect valid_rect = bounding_rect & image_roi;
        if (valid_rect.width <= 0 || valid_rect.height <= 0) continue;

        const cv::Mat orig_roi = binary(valid_rect);

        cv::Mat roi = this->letterbox(orig_roi, kMlpInputSize, kMlpInputSize, cv::Scalar(0));
        if (roi.empty()) continue;
        if (roi.channels() == 3)
            cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);

        cv::Mat input_blob = cv::dnn::blobFromImage(roi, 1.0 / 255.0, cv::Size(kMlpInputSize, kMlpInputSize), cv::Scalar(), true, false);
        mlp_net_.setInput(input_blob);
        cv::Mat output = mlp_net_.forward();

        if (output.empty() || output.total() < 5) continue;


        float* floatarr = output.ptr<float>();
        std::vector<float> outputs(floatarr, floatarr + 5);
        int max_index = std::max_element(outputs.begin(), outputs.end()) - outputs.begin();
        if (outputs[max_index] < 0) continue;

        const cv::Moments moment = cv::moments(contour);
        if (moment.m00 == 0) continue;
        cv::Point2f center(moment.m10 / moment.m00, moment.m01 / moment.m00);

        switch (max_index) {
            case 0: // R
                if (outputs[max_index] > this->max_r_logo_prob) {
                    this->max_r_logo_prob = outputs[max_index];
                    this->r_logo_roi = cv::boundingRect(contour);
                    this->max_r_logo_center = center;
                }
                break;
            case 3: // T
                if (outputs[max_index] > this->max_target_prob) {
                    this->max_target_prob = outputs[max_index];
                    this->target_roi = cv::boundingRect(contour);
                    this->max_target_center = center;
                }
                break;
        }
    }

    if (this->max_r_logo_center != cv::Point2f(-1, -1) && this->max_target_center != cv::Point2f(-1, -1)) {
        static constexpr int expand_pix = 5;
        cv::Rect expanded_roi = this->target_roi;
        expanded_roi.x -= expand_pix;
        expanded_roi.y -= expand_pix;
        expanded_roi.width += expand_pix * 2;
        expanded_roi.height += expand_pix * 2;
        expanded_roi &= cv::Rect(0, 0, input_img.cols, input_img.rows);
        this->target_roi = expanded_roi;
    }

    if (this->target_roi.width <= 0 || this->target_roi.height <= 0) return;

    target_roi_img = gray_img(this->target_roi).clone();
    cv::GaussianBlur(target_roi_img, target_roi_img, cv::Size(5, 5), 0);
    cv::threshold(target_roi_img, target_roi_img, bin_threshold_target, 255, cv::THRESH_BINARY);
    return;
}

void Buff_Rune::process_2(const cv::Mat &output_img, cv::Mat &target_roi_img, cv::Mat &processed_img){
    processed_img.release();

    if (output_img.empty()) return;

    // ROI visualization should update every frame.
    const cv::Rect img_rect(0, 0, output_img.cols, output_img.rows);
    cv::Rect roi_show = this->target_roi & img_rect;
    bool roi_show_valid = (roi_show.width > 0 && roi_show.height > 0);

    auto show_roi = [&]() {
        cv::Mat vis_img = output_img.clone();
        if (!vis_img.empty() && roi_show_valid) {
            cv::rectangle(vis_img, roi_show, cv::Scalar(0, 0, 255), 2);
        }
        if (!vis_img.empty()) {
            cv::imshow("Buff Rune ROI Box", vis_img);
        }

        if (roi_show_valid) {
            cv::imshow("Buff Rune ROI Crop", output_img(roi_show));
        } else {
            cv::Mat blank = cv::Mat::zeros(200, 200, output_img.type());
            cv::imshow("Buff Rune ROI Crop", blank);
        }
    };

    cv::Mat skeleton_roi;
    this->filtered_intersections.clear();
    this->intersections.clear();
    this->square_points.clear();
    this->four_points.clear();

    if (target_roi_img.empty()) {
        processed_img = output_img.clone();
        show_roi();
        return;
    }

    // Work on a local grayscale copy to avoid mutating the caller's Mat.
    cv::Mat roi_gray;
    if (target_roi_img.type() == CV_8UC1) {
        roi_gray = target_roi_img;
    } else if (target_roi_img.channels() == 3) {
        cv::cvtColor(target_roi_img, roi_gray, cv::COLOR_BGR2GRAY);
    } else if (target_roi_img.channels() == 4) {
        cv::cvtColor(target_roi_img, roi_gray, cv::COLOR_BGRA2GRAY);
    } else if (target_roi_img.channels() == 1) {
        target_roi_img.convertTo(roi_gray, CV_8UC1);
    } else {
        processed_img = output_img.clone();
        return;
    }

    if (roi_gray.empty()) {
        processed_img = output_img.clone();
        show_roi();
        return;
    }

    // Apply an ellipse mask (same as reference preprocess_2)
    cv::Mat goal_roi_mask = roi_gray.clone();
    if (goal_roi_mask.empty()) {
        processed_img = output_img.clone();
        show_roi();
        return;
    }
    cv::ellipse(
      goal_roi_mask,
      cv::Point2f(static_cast<float>(roi_gray.cols) / 2.0f, static_cast<float>(roi_gray.rows) / 2.0f),
      cv::Size(roi_gray.cols / 2, roi_gray.rows / 2),
      0,
      0,
      360,
      cv::Scalar(0),
      -1);
    cv::Mat roi_work = roi_gray.clone();
    if (roi_work.empty()) {
        processed_img = output_img.clone();
        show_roi();
        return;
    }
    roi_work.setTo(cv::Scalar(0), goal_roi_mask);

    cv::ximgproc::thinning(roi_work, skeleton_roi, cv::ximgproc::THINNING_GUOHALL);
    if (skeleton_roi.empty()) {
        processed_img = output_img.clone();
        show_roi();
        return;
    }

    for (int y = 1; y < skeleton_roi.rows - 1; y++) {
        for (int x = 1; x < skeleton_roi.cols - 1; x++) {
            if (skeleton_roi.at<uchar>(y, x) == 255) {
                int neighbors = 0;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (dy == 0 && dx == 0) continue;
                        neighbors += skeleton_roi.at<uchar>(y + dy, x + dx) == 255;
                    }
                }
                if (neighbors >= 3)
                    this->intersections.push_back(cv::Point2f(x, y));
            }
        }
    }

    for (size_t i = 0; i < this->intersections.size(); i++) {
        bool is_duplicate = false;
        for (size_t j = 0; j < i; j++) {
            if (cv::norm(this->intersections[i] - this->intersections[j]) < 5.0) {
                is_duplicate = true;
                break;
            }
        }
        if (!is_duplicate)
            this->filtered_intersections.push_back(this->intersections[i]);
    }

    this->intersections = std::move(this->filtered_intersections);
    cv::Point2f orig(this->target_roi.x, this->target_roi.y);
    if (this->intersections.size() < 4) {
        processed_img = output_img.clone();
        show_roi();
        return;
    }

    std::sort(this->intersections.begin(), this->intersections.end(),
              [orig, this](const cv::Point2f& a, const cv::Point2f& b) {
                  return cv::norm(a + orig - this->max_target_center) >
                         cv::norm(b + orig - this->max_target_center);
              });
    for (size_t i = 0; i < 4; i++) {
        this->four_points.push_back(this->intersections[i] + orig);
    }

    if (this->four_points.size() != 4) {
        processed_img = output_img.clone();
        show_roi();
        return;
    }

    if (this->calculateVariance(this->four_points) > variance_threshold) {
        cv::Point2f orig(this->target_roi.x, this->target_roi.y);
        this->square_points = this->sortQuadPoints(this->four_points, this->max_target_center, this->max_r_logo_center);

        // 只保留：以 R 标为中心截取 ROI
        if (!this->square_points.empty() && this->max_r_logo_center != cv::Point2f(-1, -1)) {
            const float base_dist = static_cast<float>(cv::norm(this->square_points[0] - this->max_r_logo_center));
            const float radius = base_dist * 2.0f;
            if (radius >= 1.0f) {
                const int x = static_cast<int>(std::round(this->max_r_logo_center.x - radius));
                const int y = static_cast<int>(std::round(this->max_r_logo_center.y - radius));
                const int w = static_cast<int>(std::round(radius * 2.0f));
                const int h = w;
                cv::Rect roi(x, y, w, h);
                roi &= cv::Rect(0, 0, output_img.cols, output_img.rows);
                if (roi.width > 0 && roi.height > 0) {
                    // Keep output the same size as the original image to preserve
                    // coordinate consistency for downstream detector/solver.
                    processed_img = cv::Mat::zeros(output_img.size(), output_img.type());
                    output_img(roi).copyTo(processed_img(roi));

                    // Update the per-frame ROI to display.
                    roi_show = roi;
                    roi_show_valid = true;
                    show_roi();
                }
            }
        }
        if (processed_img.empty()) {
            processed_img = output_img.clone();
        }
        show_roi();
        return;
    }

    // Reference logic: only proceed to R-centered ROI crop when variance is above threshold.
    if (this->calculateVariance(this->four_points) <= variance_threshold) {
        processed_img = output_img.clone();
        show_roi();
        return;
    }

    // Sort quad points (O: target center, A: R center)
    this->square_points = this->sortQuadPoints(this->four_points, this->max_target_center, this->max_r_logo_center);

    // R-centered ROI crop (keep processed_img same size as output_img)
    if (!this->square_points.empty() && this->max_r_logo_center != cv::Point2f(-1, -1)) {
        const float base_dist = static_cast<float>(cv::norm(this->square_points[0] - this->max_r_logo_center));
        const float radius = base_dist * 1.7f;
        if (radius >= 1.0f) {
            const int x = static_cast<int>(std::round(this->max_r_logo_center.x - radius));
            const int y = static_cast<int>(std::round(this->max_r_logo_center.y - radius));
            const int w = static_cast<int>(std::round(radius * 2.0f));
            const int h = w;
            cv::Rect roi(x, y, w, h);
            roi &= img_rect;
            if (roi.width > 0 && roi.height > 0) {
                processed_img = cv::Mat::zeros(output_img.size(), output_img.type());
                output_img(roi).copyTo(processed_img(roi));
                roi_show = roi;
                roi_show_valid = true;
            }
        }
    }

    if (processed_img.empty()) processed_img = output_img.clone();

    show_roi();
}

std::vector<cv::Point2f> Buff_Rune::sortQuadPoints(const std::vector<cv::Point2f>& points,
                                                   const cv::Point2f& O,
                                                   const cv::Point2f& A) {
    if (points.size() != 4) {
        std::cerr << "必须提供4个点。" << std::endl;
        return {};
    }

    std::vector<float> angles;
    angles.reserve(points.size());
    for (const auto& pt : points) {
        const float dx = pt.x - O.x;
        const float dy = pt.y - O.y;
        float angle = atan2(dy, dx);
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


} // namespace auto_buff
