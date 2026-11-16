#pragma once

#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>

namespace height_map
{

struct Cell
{
  float height = std::numeric_limits<float>::quiet_NaN();   // floor height in odom, NaN = unknown
  float pending = std::numeric_limits<float>::quiet_NaN();  // a rise waiting for a second scan
  double last_seen = -std::numeric_limits<double>::infinity();
};

// How a scan changes a cell. Within a scan the cell takes its highest point, like the training
// ray that stops at the top surface. Across scans a running max would only ever go up (noise,
// ghost points beside the legs, people walking past), so the new value is averaged in instead.
struct CellRule
{
  double blend_window = 1.0;   // s: a value seen this recently is averaged with the new one
  double blend = 0.5;          // weight of the new scan
  double rise_confirm = 0.04;  // m: a rise this big over a recent value needs a second scan
};

// Odom-aligned height map of n x n cells centred on the robot. It moves in whole cells and keeps
// the overlap, so the floor under the body (which the L1 can't see) stays known from earlier.
// Cells are addressed by global index g = floor(odom / cell_size).
class RollingHeightMap
{
public:
  RollingHeightMap(double cell_size, int cells, const CellRule & rule);

  // Move so that (x, y) is in the centre cell. New cells start unknown.
  void recenter(double x, double y);
  // One scan of odom points. Returns the number of cells it touched.
  int update(const std::vector<Eigen::Vector3d> & points, double stamp);
  void clear();

  long globalIndex(double v) const {return static_cast<long>(std::floor(v / cell_size_));}
  double cellCenter(long g) const {return (static_cast<double>(g) + 0.5) * cell_size_;}
  // nullptr outside the map
  const Cell * cellAtIndex(long gx, long gy) const;
  double cellSize() const {return cell_size_;}

  // f(gx, gy, cell) for every known cell
  template<class F>
  void forEachKnown(F && f) const
  {
    for (int j = 0; j < n_; ++j) {
      for (int i = 0; i < n_; ++i) {
        const Cell & c = cells_[static_cast<size_t>(j * n_ + i)];
        if (std::isfinite(c.height)) {f(ox_ + i, oy_ + j, c);}
      }
    }
  }

private:
  void apply(Cell & cell, float scan_max, double stamp) const;

  double cell_size_;
  int n_;
  CellRule rule_;
  long ox_ = 0;
  long oy_ = 0;
  bool placed_ = false;
  std::vector<Cell> cells_;
  // per-scan scratch
  std::vector<float> scan_max_;
  std::vector<bool> scan_hit_;
  std::vector<int> touched_;
};

}  // namespace height_map
