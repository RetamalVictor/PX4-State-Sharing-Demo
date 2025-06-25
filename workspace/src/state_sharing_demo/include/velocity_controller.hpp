#pragma once

#include <Eigen/Dense>
#include <unordered_map>
#include <cstdint>

namespace state_sharing_demo {

/**
 * @brief  Simple flocking‐style consensus controller:
 *
 *   v = kp * (mean(neighbours) − self)
 */
class VelocityController
{
public:
  /// Set proportional gain
  void setGain(double kp) { kp_ = kp; }

  /// Update yourself (optional, you can also pass self each compute)
  void updateSelf(const Eigen::Vector3d &self) { self_ = self; }

  /// Add or update a neighbour’s position
  void updateNeighbour(uint16_t id, const Eigen::Vector3d &pos) {
    neighbours_[id] = pos;
  }

  /// Compute the velocity command based on current self-position
  Eigen::Vector3d compute(const Eigen::Vector3d &self) {
    if (neighbours_.empty()) {
      return Eigen::Vector3d::Zero();
    }
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (auto &kv : neighbours_) {
      mean += kv.second;
    }
    mean /= static_cast<double>(neighbours_.size());
    return kp_ * (mean - self);
  }

private:
  double kp_{0.5};
  Eigen::Vector3d self_{0,0,0};
  std::unordered_map<uint16_t, Eigen::Vector3d> neighbours_;
};

} // namespace state_sharing_demo
