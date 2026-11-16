#include "height_map/scan_lookup.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace height_map
{

ScanResult lookupScan(
  const RollingHeightMap & map, double x, double y, double base_z, double yaw, double feet_z,
  double now, const LookupConfig & config)
{
  ScanResult out;
  const double c = std::cos(yaw), s = std::sin(yaw);
  const int r = static_cast<int>(std::lround(config.fill_radius / map.cellSize()));

  for (int k = 0; k < kNumPoints; ++k) {
    const GridOffset o = gridOffset(k);
    out.wx[k] = x + c * o.x - s * o.y;
    out.wy[k] = y + s * o.x + c * o.y;
    const long gx = map.globalIndex(out.wx[k]);
    const long gy = map.globalIndex(out.wy[k]);

    double floor_z = feet_z;
    const Cell * cell = map.cellAtIndex(gx, gy);
    if (cell && std::isfinite(cell->height)) {
      floor_z = cell->height;
      out.source[k] = now - cell->last_seen <= config.fresh_time ? kSeen : kRemembered;
    } else {
      // Nearest known cell; on a tie the highest, so an edge doesn't vanish.
      int best_d2 = std::numeric_limits<int>::max();
      float best_h = -std::numeric_limits<float>::infinity();
      for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
          const int d2 = dx * dx + dy * dy;
          if (d2 == 0 || d2 > r * r || d2 > best_d2) {continue;}
          const Cell * n = map.cellAtIndex(gx + dx, gy + dy);
          if (!n || !std::isfinite(n->height)) {continue;}
          if (d2 < best_d2 || n->height > best_h) {
            best_d2 = d2;
            best_h = n->height;
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
