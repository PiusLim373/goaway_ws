#include "height_map/rolling_height_map.hpp"

#include <algorithm>
#include <utility>

namespace height_map
{

RollingHeightMap::RollingHeightMap(double cell_size, int cells, const CellRule & rule)
: cell_size_(cell_size), n_(cells), rule_(rule), cells_(static_cast<size_t>(cells * cells)),
  scan_max_(static_cast<size_t>(cells * cells), 0.0f),
  scan_hit_(static_cast<size_t>(cells * cells), false) {}

void RollingHeightMap::recenter(double x, double y)
{
  const long nx = globalIndex(x) - n_ / 2;
  const long ny = globalIndex(y) - n_ / 2;
  if (!placed_) {
    ox_ = nx;
    oy_ = ny;
    placed_ = true;
    return;
  }
  if (nx == ox_ && ny == oy_) {return;}
  std::vector<Cell> moved(cells_.size());
  for (int j = 0; j < n_; ++j) {
    const long nj = oy_ + j - ny;
    if (nj < 0 || nj >= n_) {continue;}
    for (int i = 0; i < n_; ++i) {
      const long ni = ox_ + i - nx;
      if (ni >= 0 && ni < n_) {
        moved[static_cast<size_t>(nj * n_ + ni)] = cells_[static_cast<size_t>(j * n_ + i)];
      }
    }
  }
  cells_ = std::move(moved);
  ox_ = nx;
  oy_ = ny;
}

int RollingHeightMap::update(const std::vector<Eigen::Vector3d> & points, double stamp)
{
  touched_.clear();
  for (const auto & p : points) {
    const long i = globalIndex(p.x()) - ox_;
    const long j = globalIndex(p.y()) - oy_;
    if (i < 0 || i >= n_ || j < 0 || j >= n_) {continue;}
    const auto idx = static_cast<size_t>(j * n_ + i);
    const auto z = static_cast<float>(p.z());
    if (!scan_hit_[idx]) {
      scan_hit_[idx] = true;
      scan_max_[idx] = z;
      touched_.push_back(static_cast<int>(idx));
    } else {
      scan_max_[idx] = std::max(scan_max_[idx], z);
    }
  }
  for (int idx : touched_) {
    const auto u = static_cast<size_t>(idx);
    apply(cells_[u], scan_max_[u], stamp);
    scan_hit_[u] = false;
  }
  return static_cast<int>(touched_.size());
}

void RollingHeightMap::apply(Cell & cell, float scan_max, double stamp) const
{
  const bool recent = std::isfinite(cell.height) && stamp - cell.last_seen < rule_.blend_window;
  if (!recent) {
    cell.height = scan_max;
    cell.pending = std::numeric_limits<float>::quiet_NaN();
  } else if (scan_max > cell.height + rule_.rise_confirm) {
    if (std::isfinite(cell.pending) && std::abs(scan_max - cell.pending) < rule_.rise_confirm) {
      cell.height = scan_max;   // seen twice: real
      cell.pending = std::numeric_limits<float>::quiet_NaN();
    } else {
      cell.pending = scan_max;
    }
  } else {
    cell.height += static_cast<float>(rule_.blend) * (scan_max - cell.height);
    cell.pending = std::numeric_limits<float>::quiet_NaN();
  }
  cell.last_seen = stamp;
}

void RollingHeightMap::clear()
{
  std::fill(cells_.begin(), cells_.end(), Cell{});
  placed_ = false;
}

const Cell * RollingHeightMap::cellAtIndex(long gx, long gy) const
{
  const long i = gx - ox_;
  const long j = gy - oy_;
  if (!placed_ || i < 0 || i >= n_ || j < 0 || j >= n_) {return nullptr;}
  return &cells_[static_cast<size_t>(j * n_ + i)];
}

}  // namespace height_map
