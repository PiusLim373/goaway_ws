#pragma once

#include <array>

#include <tf2/LinearMath/Vector3.h>

namespace goaway_odometry
{

// Body velocity from the Go2's legs. A planted foot doesn't move in the room, so the body
// moves opposite to how that foot moves relative to it. The foot sensors read nothing on
// this Go2, so planted feet are guessed: the loaded ones (calf torque) that agree.
class LegOdometry
{
public:
  static constexpr int kLegs = 4;   // LowState order: FR, FL, RR, RL

  struct Input
  {
    std::array<double, 12> q{};     // joint angles, each leg hip, thigh, calf
    std::array<double, 12> dq{};    // joint speeds
    std::array<double, 12> tau{};   // estimated joint torques
    tf2::Vector3 gyro{0.0, 0.0, 0.0};   // body turn rate, body frame
  };

  struct Output
  {
    tf2::Vector3 velocity{0.0, 0.0, 0.0};   // body frame
    std::array<tf2::Vector3, kLegs> feet{};  // foot centres, body frame
    std::array<tf2::Vector3, kLegs> foot_velocity{};   // body velocity each foot implies if planted
    std::array<bool, kLegs> planted{};
    bool any_planted{false};
  };

  Output update(const Input & in) const;

  double load_fraction{0.4};   // planted candidates: calf torque above this share of the largest
  double limp_torque{2.0};     // N m: below this on every calf, the legs are limp (all candidates)
  double agreement{0.15};      // m/s: planted feet must give body speeds this close
};

}  // namespace goaway_odometry
