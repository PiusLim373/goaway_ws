#pragma once

#include <deque>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "height_map/scan_lookup.hpp"

namespace height_map
{

// The instant scan (the training scan of perceptive v3, mjlab InstantScan): only the L1 points
// of the last window, in goaway_odometry's short-term legs + IMU frame, nothing remembered. A grid
// point is seen when a point lies within half_cell of it (heading-aligned) and takes the highest;
// the others take the nearest seen point within fill_radius (on a tie the highest), else the
// floor under the planted feet. The rule the training's L1 hit rates were fitted to.
class InstantScan
{
public:
  InstantScan(double window, double half_cell)
  : window_(window), half_cell_(half_cell) {}

  // Points (local frame) and their times.
  void add(const std::vector<Eigen::Vector3d> & points, const std::vector<double> & times);
  void clear();
  // Time of the newest point, -inf without any.
  double newest() const {return newest_;}

  // The grid around a base at (x, y, base_z) heading yaw, from the window's points before the
  // newest one (a whole window even right after a cloud's delay). feet_z: the floor under the
  // planted feet. wx, wy: offsets from the base, heading-aligned (base_footprint's frame).
  ScanResult scan(
    double x, double y, double base_z, double yaw, double feet_z,
    const LookupConfig & config) const;

private:
  struct Point
  {
    double t;
    Eigen::Vector3d p;
  };

  double window_;
  double half_cell_;
  double newest_ = -std::numeric_limits<double>::infinity();
  std::deque<Point> points_;   // in arrival order
};

}  // namespace height_map
