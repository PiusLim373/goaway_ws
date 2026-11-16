#include "height_map/self_filter.hpp"

#include <algorithm>
#include <cmath>

namespace height_map
{

namespace
{
// go2.urdf: hip joints in the trunk, then thigh offset, thigh and calf lengths
constexpr double kHipX = 0.1934;
constexpr double kHipY = 0.0465;
constexpr double kThighY = 0.0955;
constexpr double kLink = 0.213;
constexpr double kHipXSign[SelfFilter::kLegs] = {1.0, 1.0, -1.0, -1.0};
constexpr double kSide[SelfFilter::kLegs] = {-1.0, 1.0, -1.0, 1.0};   // right -1, left +1

double segmentDistance2(const Eigen::Vector3d & p, const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
  const Eigen::Vector3d ab = b - a;
  const double len2 = ab.squaredNorm();
  const double u = len2 > 1e-12 ? std::clamp((p - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
  return (p - (a + u * ab)).squaredNorm();
}
}  // namespace

void SelfFilter::setJoints(const std::array<double, 12> & q)
{
  for (int i = 0; i < kLegs; ++i) {
    const double s = kSide[i];
    const double q0 = q[3 * i], q1 = q[3 * i + 1], q2 = q[3 * i + 2];
    const double c0 = std::cos(q0), s0 = std::sin(q0);
    const Eigen::Vector3d hip(kHipXSign[i] * kHipX, s * kHipY, 0.0);
    // In the hip's sagittal plane, then rolled about x by the hip joint.
    auto rolled = [&](double ux, double uy, double uz) -> Eigen::Vector3d {
        return hip + Eigen::Vector3d(ux, c0 * uy - s0 * uz, s0 * uy + c0 * uz);
      };
    const double uy = s * kThighY;
    legs_[i][0] = hip;
    legs_[i][1] = rolled(0.0, uy, 0.0);
    legs_[i][2] = rolled(-kLink * std::sin(q1), uy, -kLink * std::cos(q1));
    legs_[i][3] = rolled(
      -kLink * (std::sin(q1) + std::sin(q1 + q2)), uy, -kLink * (std::cos(q1) + std::cos(q1 + q2)));
  }
}

bool SelfFilter::isSelf(const Eigen::Vector3d & p) const
{
  if ((p.array() > config_.body_min.array()).all() && (p.array() < config_.body_max.array()).all()) {
    return true;
  }
  const double r2 = config_.leg_radius * config_.leg_radius;
  for (const auto & leg : legs_) {
    for (int j = 0; j < 3; ++j) {
      if (segmentDistance2(p, leg[j], leg[j + 1]) < r2) {return true;}
    }
  }
  return false;
}

}  // namespace height_map
