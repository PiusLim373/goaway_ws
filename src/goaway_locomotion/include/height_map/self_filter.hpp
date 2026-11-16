#pragma once

#include <array>

#include <Eigen/Core>

namespace height_map
{

// The robot's own L1 hits, in base_link. The L1 can't see the body, only its own window and
// the legs, mostly the calves, plus ghost points up to ~10 cm beside them (a beam half on the
// leg, half on the floor). This also cuts the floor right around each foot; those cells are
// remembered or filled.
struct SelfFilterConfig
{
  Eigen::Vector3d body_min{-0.33, -0.12, -0.10};   // the LiDAR's own head and chin
  Eigen::Vector3d body_max{0.40, 0.12, 0.20};
  double leg_radius = 0.08;   // around each hip - thigh joint - knee - foot segment
};

class SelfFilter
{
public:
  static constexpr int kLegs = 4;   // LowState order: FR, FL, RR, RL

  explicit SelfFilter(const SelfFilterConfig & config) : config_(config) {}

  // Legs from the joint angles (each leg hip, thigh, calf), go2.urdf kinematics.
  void setJoints(const std::array<double, 12> & q);
  bool isSelf(const Eigen::Vector3d & p) const;

private:
  SelfFilterConfig config_;
  // hip joint, thigh joint, knee, foot centre per leg
  std::array<std::array<Eigen::Vector3d, 4>, kLegs> legs_{};
};

}  // namespace height_map
