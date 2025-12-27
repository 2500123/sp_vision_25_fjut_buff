#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"                 

#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_rune.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{config-path c  | configs/standard4.yaml | yaml配置文件的路径}";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>("config-path");

  tools::Plotter plotter;
  tools::Exiter exiter;

  io::Camera camera(config_path);

  auto_buff::Buff_Detector detector(config_path);
  auto_buff::Buff_Rune buff_rune(config_path);
  auto_buff::Solver solver(config_path);
  auto_buff::BigTarget target;
  auto_buff::Aimer aimer(config_path);

  cv::Mat img, drawing;
  io::Command last_command;

  for (; !exiter.exit();) {
    // 1. 读相机一帧
    std::chrono::steady_clock::time_point timestamp;
    camera.read(img, timestamp);
    if (img.empty()) {    
      continue;
    }

    // 2. 从IMU/云台获取姿态四元数
    double w, x, y, z;
 
    w = 1.0; x = 0.0; y = 0.0; z = 0.0;

    solver.set_R_gimbal2world({w, x, y, z});

    // buff_rune.preprocess(img, drawing);

    // 3. Buff 检测 + 解算 + 目标选择 + Aimer
    auto power_runes = detector.detect_24(img);
    solver.solve(power_runes);
    target.get_target(power_runes, timestamp);

    auto target_copy = target;
    auto command = aimer.aim(target_copy, timestamp, 22, false);

    // 4. 可视化 & 数据输出
    nlohmann::json data;

    if (power_runes.has_value()) {
      const auto & p = power_runes.value();
      data["buff_R_yaw"] = p.ypd_in_world[0];
      data["buff_R_pitch"] = p.ypd_in_world[1];
      data["buff_R_dis"] = p.ypd_in_world[2];
      data["buff_yaw"] = p.ypr_in_world[0] * 57.3;
      data["buff_pitch"] = p.ypr_in_world[1] * 57.3;
      data["buff_roll"] = p.ypr_in_world[2] * 57.3;
    }

    if (!target.is_unsolve() && power_runes.has_value()) {
      auto & p = power_runes.value();

      for (int i = 0; i < 4; i++) tools::draw_point(img, p.target().points[i]);
      tools::draw_point(img, p.target().center, {0, 0, 255}, 3);
      tools::draw_point(img, p.r_center, {0, 0, 255}, 3);

      auto Rxyz_in_world_now = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
      auto image_points =
        solver.reproject_buff(Rxyz_in_world_now, target.ekf_x()[4], target.ekf_x()[5]);
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin(), image_points.begin() + 4), {0, 255, 0});
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin() + 4, image_points.end()), {0, 255, 0});

    //   double dangle = target.ekf_x()[5] - target_copy.ekf_x()[5];
    //   auto Rxyz_in_world_pre = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
    //   image_points =
    //     solver.reproject_buff(Rxyz_in_world_pre, target_copy.ekf_x()[4], target_copy.ekf_x()[5]);
    //   tools::draw_points(
    //     img, std::vector<cv::Point2f>(image_points.begin(), image_points.begin() + 4), {255, 0, 0});
    //   tools::draw_points(
    //     img, std::vector<cv::Point2f>(image_points.begin() + 4, image_points.end()), {255, 0, 0});

      Eigen::VectorXd x_state = target.ekf_x();
      data["R_yaw"] = x_state[0];
      data["R_V_yaw"] = x_state[1];
      data["R_pitch"] = x_state[2];
      data["R_dis"] = x_state[3];
      data["yaw"] = x_state[4] * 57.3;
      data["angle"] = x_state[5] * 57.3;
      data["spd"] = x_state[6] * 57.3;
      if (x_state.size() >= 10) {
        data["spd"] = x_state[6];
        data["a"] = x_state[7];
        data["w"] = x_state[8];
        data["fi"] = x_state[9];
        data["spd0"] = target.spd;
      }
    }

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    data["gimbal_yaw"] = ypr[0] * 57.3;
    data["gimbal_pitch"] = -ypr[1] * 57.3;

    if (command.control) {
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
    }

    plotter.plot(data);
    cv::imshow("result", img);
    
    // cv::imshow("drawing", drawing);

    int key = cv::waitKey(1);
    if (key == 'q') break;
    while (key == ' ') {
      int y = cv::waitKey(30);
      if (y == 'q') break;
    }
  }

  cv::destroyAllWindows();
  return 0;
}