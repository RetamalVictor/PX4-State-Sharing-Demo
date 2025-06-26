#include "velocity_controller.hpp"
#include <algorithm>
#include <iostream>

namespace state_sharing_demo {

Eigen::Vector3d VelocityController::compute() {
  if (rel_distances_.empty()) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  for (const auto& [id, rel] : rel_distances_) {
    double r = rel.norm();
    if (r < 1e-6) {
      std::cout << "[VelocityController] Skipping neighbor " << id << " due to zero distance" << std::endl;
      continue;
    }

    double r_eff = std::max(r, r_safe_);
    double error = r_eff - desired_dist_;
    std::cout << "[VelocityController] neighbor " << id
              << ": distance=" << r
              << ", effective_distance=" << r_eff
              << ", error=" << error << std::endl;

    v += kp_ * error * (rel / r);
  }

  // Average over neighbors
  v /= static_cast<double>(rel_distances_.size());

  // Damping term to reduce oscillations
  v -= kd_ * vel_current_;

  // Speed saturation
  double speed = v.norm();
  if (speed > v_max_) {
    v *= (v_max_ / speed);
  }

  std::cout << "[VelocityController] Final velocity: ["
            << v.x() << ", " << v.y() << ", " << v.z() << "]" << std::endl;
  return v;
}


} // namespace state_sharing_demo
