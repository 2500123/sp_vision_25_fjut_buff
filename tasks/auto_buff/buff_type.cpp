#include "buff_type.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

#include "tools/logger.hpp"
namespace auto_buff
{
FanBlade::FanBlade(
  const std::vector<cv::Point2f> & kpt, cv::Point2f keypoints_center, FanBlade_type t)
: center(keypoints_center), type(t)
{
  points.insert(points.end(), kpt.begin(), kpt.end());
}

FanBlade::FanBlade(FanBlade_type t) : type(t)
{
  assert(t == _unlight);
}

PowerRune::PowerRune(
  std::vector<FanBlade> & ts, const cv::Point2f center, std::optional<PowerRune> last_powerrune)
: r_center(center), light_num(ts.size())
{
  /// 找出target

  if (ts.empty()) {
    tools::logger()->debug("[PowerRune] 识别出错!");
    unsolvable_ = true;
    return;
  }

  // Helper: keep lock by associating to the nearest last target center.
  auto choose_nearest_last_target = [&](const cv::Point2f & last_target_center) {
    auto target_it = ts.begin();
    float min_distance = cv::norm(ts[0].center - last_target_center);
    const float tie_eps_px = 1.0f;
    for (auto it = ts.begin(); it != ts.end(); ++it) {
      float distance = cv::norm(it->center - last_target_center);
      if (distance < min_distance - tie_eps_px) {
        min_distance = distance;
        target_it = it;
      } else if (std::abs(distance - min_distance) <= tie_eps_px) {
        // Tie-break deterministically so target doesn't flip when detection order changes.
        const double a_new = atan_angle(it->center);
        const double a_old = atan_angle(target_it->center);
        if (a_new < a_old - 1e-6) {
          target_it = it;
        } else if (std::abs(a_new - a_old) <= 1e-6 && it->center.x < target_it->center.x) {
          target_it = it;
        }
      }
    }
    target_it->type = _target;
    std::iter_swap(ts.begin(), target_it);
  };

  // No history: pick a deterministic target (smallest absolute angle around r_center).
  // This avoids random switching when detection order changes.
  if (!last_powerrune.has_value()) {
    auto target_it = ts.begin();
    double best_angle = atan_angle(ts[0].center);
    for (auto it = ts.begin(); it != ts.end(); ++it) {
      double a = atan_angle(it->center);
      if (a < best_angle) {
        best_angle = a;
        target_it = it;
      }
    }
    target_it->type = _target;
    std::iter_swap(ts.begin(), target_it);
  }

  // With history: default to stable association; only switch to a "newly lit" blade when confident.
  else {
    const auto last_target_center = last_powerrune.value().fanblades[0].center;

    // If we see one extra blade, it *might* be the newly lit target.
    // Gate the switch by distance, otherwise keep the existing lock (prevents jitter jumps).
    if (light_num == last_powerrune.value().light_num + 1) {
      const auto & last_fanblades = last_powerrune.value().fanblades;

      // Estimate radius in image space (used to scale thresholds across distances).
      float radius = 0.0f;
      for (const auto & b : ts) radius += cv::norm(b.center - r_center);
      radius /= static_cast<float>(ts.size());

      // If the previous target is still present in the current detections, keep tracking it.
      // This prevents a newly appeared candidate (often a false positive) from stealing the lock.
      float closest_to_last_target = std::numeric_limits<float>::max();
      for (const auto & b : ts) {
        closest_to_last_target =
          std::min(closest_to_last_target, static_cast<float>(cv::norm(b.center - last_target_center)));
      }
      const float keep_gate = std::max(25.0f, 0.25f * radius);
      if (closest_to_last_target <= keep_gate) {
        choose_nearest_last_target(last_target_center);
      } else {

        float max_min_distance = -1.0f;
        auto new_it = ts.begin();
        for (auto it = ts.begin(); it != ts.end(); ++it) {
          float min_distance = std::numeric_limits<float>::max();
          // distance to the set of previously observed (lit) blades
          for (const auto & last_fanblade : last_fanblades) {
            if (last_fanblade.type == _unlight) continue;
            float distance = static_cast<float>(cv::norm(it->center - last_fanblade.center));
            if (distance < min_distance) min_distance = distance;
          }
          if (min_distance > max_min_distance) {
            max_min_distance = min_distance;
            new_it = it;
          }
        }

        // Adjacent blades are separated by ~1.17*R; false positives near an existing blade have small min_distance.
        const float switch_gate = std::max(30.0f, 0.4f * radius);
        if (max_min_distance > switch_gate) {
          new_it->type = _target;
          std::iter_swap(ts.begin(), new_it);
        } else {
          choose_nearest_last_target(last_target_center);
        }
      }
    } else {
      choose_nearest_last_target(last_target_center);
    }
  }

  /// 填充FanBlade.angle

  double angle = atan_angle(ts[0].center);
  for (auto & t : ts) {
    t.angle = atan_angle(t.center) - angle;
    if (t.angle < -1e-3) t.angle += CV_2PI;
  }

  /// fanblades调整顺序

  std::sort(ts.begin(), ts.end(), [](const FanBlade & a, const FanBlade & b) {
    return a.angle < b.angle;
  });  // 按照 t.angle 从小到大排序 ts
  const std::vector<double> target_angles = {
    0, 2.0 * CV_PI / 5.0, 4.0 * CV_PI / 5.0, 6.0 * CV_PI / 5.0, 8.0 * CV_PI / 5.0};
  int j = 0;
  for (int i = 0; i < 5; i++) {
    if (j < static_cast<int>(ts.size()) && std::fabs(ts[j].angle - target_angles[i]) < CV_PI / 5.0) {
      fanblades.emplace_back(ts[j++]);
    } else {
      fanblades.emplace_back(FanBlade(_unlight));
    }
  }
};

double PowerRune::atan_angle(cv::Point2f point) const
{
  auto v = point - r_center;
  auto angle = std::atan2(v.y, v.x);
  return angle >= 0 ? angle : angle + CV_2PI;
}
}  // namespace auto_buff
