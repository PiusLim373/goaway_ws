#pragma once

#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "height_map/rolling_height_map.hpp"
#include "height_map/scan_lookup.hpp"

namespace height_map
{

// The scan as the MuJoCo viewer draws it: a sphere where each ray hits (cyan like the viewer
// when seen; blue remembered, orange neighbour fill, red feet fill, magenta clamped).
// Optional green rays down from base height.
visualization_msgs::msg::MarkerArray scanMarkers(
  const ScanResult & scan, double base_z, bool show_rays, const std::string & frame,
  const builtin_interfaces::msg::Time & stamp);

// Every known cell as a thin tile at its height, turbo colours over feet_z -0.3 .. +0.3 m.
visualization_msgs::msg::MarkerArray mapMarkers(
  const RollingHeightMap & map, double feet_z, const std::string & frame,
  const builtin_interfaces::msg::Time & stamp);

// Empties both, e.g. while odom is paused.
visualization_msgs::msg::MarkerArray deleteAll(const std::string & frame);

}  // namespace height_map
