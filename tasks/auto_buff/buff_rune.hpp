#ifndef AUTO_BUFF__BUFF_RUNE_HPP
#define AUTO_BUFF__BUFF_RUNE_HPP

#include <iostream>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <thread>
#include <deque>
#include <optional>
#include <yaml-cpp/yaml.h>
#include "tools/img_tools.hpp"

namespace auto_buff {

enum Color { RED, BLUE };

class Buff_Rune {
public:
    Buff_Rune(const std::string &config);

    bool preprocess(const cv::Mat & input_img, cv::Mat & r_center_roi_img);

private:
    cv::Mat letterbox(const cv::Mat& src, int target_w, int target_h, cv::Scalar pad_color = cv::Scalar(0));
    std::vector<cv::Point2f> sortQuadPoints(const std::vector<cv::Point2f>& points, const cv::Point2f& O, const cv::Point2f& A);
    float calculateVariance(const std::vector<cv::Point2f>& points);

    // Parameters
    int min_threshold = 50;                // 第一次二值化阈值
    int bin_threshold_target = 50;         // 第二次二值化阈值
    int min_area = 100;
    int variance_threshold = 150;
    std::string model_path_;
    Color input_color = RED;

    // Model
    cv::dnn::Net mlp_net_;

    // State variables
    float max_r_logo_prob = -1.0;
    float max_target_prob = -1.0;
    cv::Point2f max_r_logo_center = cv::Point2f(-1, -1);
    cv::Point2f max_target_center = cv::Point2f(-1, -1);
    cv::Rect r_logo_roi;
    cv::Rect target_roi;

    // Temporary storage for process_2
    std::vector<cv::Point2f> filtered_intersections;
    std::vector<cv::Point2f> intersections;
    std::vector<cv::Point2f> square_points;
    std::vector<cv::Point2f> four_points;
};

} // namespace auto_buff

#endif  // AUTO_BUFF__BUFF_RUNE_HPP