#pragma once

#include <Eigen/Dense>
#include <unordered_map>
#include <cstdint>

namespace state_sharing_demo {

/**
 * @brief  Simple flocking‐style consensus controller:
 */
class VelocityController
{
public:
  void setGain(double kp) { kp_ = kp; }
  void setIdent(uint8_t ident) { ident_ = ident; }
  void setDesiredDistance(double d0) { desired_dist_ = d0; }
  void setSafetyDistance(double r_safe) { r_safe_ = r_safe; }
  void setDampingGain(double kd) { kd_ = kd; }
  void setMaxSpeed(double v_max) { v_max_ = v_max; }

  void updateSelf(const Eigen::Vector3d &self) { self_ = self; }
  void updateCurrentVelocity(const Eigen::Vector3d &vel) { vel_current_ = vel; }
  void updateNeighbourDistance(uint16_t id, const Eigen::Vector3d &rel) {
    rel_distances_[id] = rel;
  }
  const std::unordered_map<uint16_t, Eigen::Vector3d> &getRelDistances() const {
    return rel_distances_;
  }

  /// Compute the velocity command via attraction–repulsion over rel_distances_
  Eigen::Vector3d compute();

private:
  double kp_{0.7};           ///< Spring gain
  double desired_dist_{10.0};///< Equilibrium distance d0
  double r_safe_{1.5};       ///< Minimum effective distance
  double kd_{0.2};           ///< Damping gain
  double v_max_{3.0};        ///< Maximum allowed speed

  Eigen::Vector3d self_{0,0,0};       ///< Self position (unused in this compute)
  Eigen::Vector3d vel_current_{0,0,0};///< Current velocity for damping
  std::unordered_map<uint16_t, Eigen::Vector3d> rel_distances_;
  uint8_t ident_{0};                  ///< Unique identifier (optional)
};

} // namespace state_sharing_demo
