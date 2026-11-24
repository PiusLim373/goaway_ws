#include "height_map/instant_scan.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace height_map
{

void InstantScan::add(
  const std::vector<Eigen::Vector3d> & points, const std::vector<double> & times)
{
  for (size_t i = 0; i < points.size(); ++i) {
    points_.push_back({times[i], points[i]});
    newest_ = std::max(newest_, times[i]);
  }
  // A cloud's points aren't quite in time order: scan() checks each one, this only trims.
  while (!points_.empty() && points_.front().t < newest_ - window_) {points_.pop_front();}
}

void InstantScan::clear()
{
  points_.clear();
  newest_ = -std::numeric_limits<double>::infinity();
}

ScanResult InstantScan::scan(
  double x, double y, double base_z, double yaw, double feet_z, const LookupConfig & config) const
{
  constexpr float kUnseen = -std::numeric_limits<float>::infinity();
  std::array<float, kNumPoints> top;
  top.fill(kUnseen);
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (const Point & pt : points_) {
    if (pt.t < newest_ - window_) {continue;}
    const double dx = pt.p.x() - x, dy = pt.p.y() - y;
    const double bx = c * dx + s * dy, by = -s * dx + c * dy;
    const long ix = std::lround((bx - kMinX) / kResolution);
    const long iy = std::lround((by - kMinY) / kResolution);
    if (ix < 0 || ix >= kNx || iy < 0 || iy >= kNy) {continue;}
    if (std::abs(bx - (kMinX + kResolution * ix)) > half_cell_ ||
      std::abs(by - (kMinY + kResolution * iy)) > half_cell_) {continue;}
    float & h = top[iy * kNx + ix];
    h = std::max(h, static_cast<float>(pt.p.z()));
  }

  ScanResult out;
  const double r = config.fill_radius / kResolution;   // in grid steps
  const int ri = static_cast<int>(std::floor(r + 1e-6));
  for (int k = 0; k < kNumPoints; ++k) {
    const GridOffset o = gridOffset(k);
    out.wx[k] = o.x;
    out.wy[k] = o.y;
    double floor_z = feet_z;
    if (top[k] != kUnseen) {
      floor_z = top[k];
      out.source[k] = kSeen;
    } else {
      // Nearest seen point; on a tie the highest, so an edge doesn't vanish.
      const int ix = k % kNx, iy = k / kNx;
      int best_d2 = std::numeric_limits<int>::max();
      float best_h = kUnseen;
      for (int dy = -ri; dy <= ri; ++dy) {
        for (int dx = -ri; dx <= ri; ++dx) {
          const int d2 = dx * dx + dy * dy;
          if (d2 == 0 || d2 > r * r + 1e-6 || d2 > best_d2) {continue;}
          const int nx = ix + dx, ny = iy + dy;
          if (nx < 0 || nx >= kNx || ny < 0 || ny >= kNy) {continue;}
          const float h = top[ny * kNx + nx];
          if (h == kUnseen) {continue;}
          if (d2 < best_d2 || h > best_h) {
            best_d2 = d2;
            best_h = h;
          }
        }
      }
      if (best_d2 != std::numeric_limits<int>::max()) {
        floor_z = best_h;
        out.source[k] = kNeighbour;
      } else {
        out.source[k] = kFeet;
      }
    }
    const double raw = base_z - floor_z;
    const double v = std::clamp(raw, config.clamp_min, config.clamp_max);
    out.clamped[k] = v != raw;
    out.heights[k] = static_cast<float>(v);
    ++out.count[out.source[k]];
  }
  return out;
}

}  // namespace height_map
