#include "goaway_odometry/leg_odometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace goaway_odometry
{

namespace
{
// go2.urdf: hip joints in the trunk, then thigh offset, thigh and calf lengths
constexpr double kHipX = 0.1934;
constexpr double kHipY = 0.0465;
constexpr double kThighY = 0.0955;
constexpr double kLink = 0.213;
constexpr double kHipXSign[LegOdometry::kLegs] = {1.0, 1.0, -1.0, -1.0};
constexpr double kSide[LegOdometry::kLegs] = {-1.0, 1.0, -1.0, 1.0};   // right -1, left +1

// Foot centre and its velocity relative to the body, from one leg's joints.
void legKinematics(
  int leg, const double * q, const double * dq, tf2::Vector3 & foot, tf2::Vector3 & foot_vel)
{
  const double s = kSide[leg];
  const double c0 = std::cos(q[0]), s0 = std::sin(q[0]);
  const double s1 = std::sin(q[1]), c1 = std::cos(q[1]);
  const double s12 = std::sin(q[1] + q[2]), c12 = std::cos(q[1] + q[2]);
  // In the hip's sagittal plane, then rolled about x by the hip joint.
  const double ux = -kLink * (s1 + s12);
  const double uz = -kLink * (c1 + c12);
  const double uy = s * kThighY;
  foot.setValue(kHipXSign[leg] * kHipX + ux, s * kHipY + c0 * uy - s0 * uz, s0 * uy + c0 * uz);

  const double dux = uz * dq[1] - kLink * c12 * dq[2];
  const double duz = -ux * dq[1] + kLink * s12 * dq[2];
  foot_vel.setValue(
    dux,
    (-s0 * uy - c0 * uz) * dq[0] - s0 * duz,
    (c0 * uy - s0 * uz) * dq[0] + c0 * duz);
}
}  // namespace

LegOdometry::Output LegOdometry::update(const Input & in) const
{
  Output out;
  std::array<tf2::Vector3, kLegs> body_vel;
  std::array<double, kLegs> load;
  double max_load = 0.0;
  for (int i = 0; i < kLegs; ++i) {
    tf2::Vector3 foot_vel;
    legKinematics(i, &in.q[3 * i], &in.dq[3 * i], out.feet[i], foot_vel);
    // The foot stays put, so the body moves the other way; minus what turning alone causes.
    body_vel[i] = -(foot_vel + in.gyro.cross(out.feet[i]));
    out.foot_velocity[i] = body_vel[i];
    load[i] = std::abs(in.tau[3 * i + 2]);
    max_load = std::max(max_load, load[i]);
  }

  std::array<bool, kLegs> candidate;
  for (int i = 0; i < kLegs; ++i) {
    candidate[i] = max_load < limp_torque || load[i] > load_fraction * max_load;
  }

  // The largest group of candidates (at least 2) whose speeds agree; the tightest one wins.
  int best_mask = 0;
  double best_spread = std::numeric_limits<double>::infinity();
  for (int size = kLegs; size >= 2 && best_mask == 0; --size) {
    for (int mask = 1; mask < (1 << kLegs); ++mask) {
      if (__builtin_popcount(mask) != size) {continue;}
      tf2::Vector3 mean(0.0, 0.0, 0.0);
      bool ok = true;
      for (int i = 0; i < kLegs; ++i) {
        if (mask & (1 << i)) {
          ok = ok && candidate[i];
          mean += body_vel[i];
        }
      }
      if (!ok) {continue;}
      mean /= size;
      double spread = 0.0;
      for (int i = 0; i < kLegs; ++i) {
        if (mask & (1 << i)) {spread = std::max(spread, (body_vel[i] - mean).length());}
      }
      if (spread < agreement && spread < best_spread) {
        best_spread = spread;
        best_mask = mask;
      }
    }
  }
  if (best_mask == 0) {   // nothing agrees: trust the load
    for (int i = 0; i < kLegs; ++i) {
      if (candidate[i]) {best_mask |= 1 << i;}
    }
  }

  int count = 0;
  for (int i = 0; i < kLegs; ++i) {
    out.planted[i] = best_mask & (1 << i);
    if (out.planted[i]) {
      out.velocity += body_vel[i];
      ++count;
    }
  }
  out.any_planted = count > 0;
  if (count > 0) {out.velocity /= count;}
  return out;
}

}  // namespace goaway_odometry
