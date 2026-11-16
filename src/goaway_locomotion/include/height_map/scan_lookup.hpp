#pragma once

#include <array>
#include <cstdint>

#include "height_map/grid.hpp"
#include "height_map/rolling_height_map.hpp"

namespace height_map
{

// Values of goaway_msgs/HeightScan source
enum Source : uint8_t
{
  kSeen = 0,        // its own cell, seen within fresh_time
  kRemembered = 1,  // its own cell, seen earlier
  kNeighbour = 2,   // the nearest known cell within fill_radius
  kFeet = 3,        // nothing known nearby: the ground under the planted feet
};

struct LookupConfig
{
  double fresh_time = 1.0;   // s
  double fill_radius = 0.15;  // m
  double clamp_min = -0.50;   // the range seen in training
  double clamp_max = 1.30;
};

struct ScanResult
{
  std::array<float, kNumPoints> heights{};   // base z - floor z, clamped
  std::array<uint8_t, kNumPoints> source{};
  std::array<bool, kNumPoints> clamped{};
  std::array<double, kNumPoints> wx{};       // odom position of each point
  std::array<double, kNumPoints> wy{};
  std::array<int, 4> count{};                // per source
};

// The grid around a base at (x, y, base_z) heading yaw. feet_z is the floor under the planted
// feet, used where nothing is known: flat floor there, never the training's 5 m miss value,
// which the policy would read as a hole.
ScanResult lookupScan(
  const RollingHeightMap & map, double x, double y, double base_z, double yaw, double feet_z,
  double now, const LookupConfig & config);

}  // namespace height_map
