#include "height_map/markers.hpp"

#include <algorithm>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace height_map
{

namespace
{
using visualization_msgs::msg::Marker;

std_msgs::msg::ColorRGBA rgba(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

geometry_msgs::msg::Point point(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

Marker baseMarker(
  const std::string & frame, const builtin_interfaces::msg::Time & stamp, const std::string & ns,
  int type)
{
  Marker m;
  m.header.frame_id = frame;
  m.header.stamp = stamp;
  m.ns = ns;
  m.type = type;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  return m;
}

std_msgs::msg::ColorRGBA sourceColor(const ScanResult & scan, int k)
{
  if (scan.clamped[k]) {return rgba(1.0f, 0.0f, 1.0f, 1.0f);}
  switch (scan.source[k]) {
    case kSeen: return rgba(0.0f, 1.0f, 1.0f, 1.0f);
    case kRemembered: return rgba(0.2f, 0.45f, 1.0f, 1.0f);
    case kNeighbour: return rgba(1.0f, 0.6f, 0.0f, 1.0f);
    default: return rgba(1.0f, 0.0f, 0.0f, 1.0f);
  }
}

// Google's Turbo colour map, polynomial fit, v in [0, 1]
std_msgs::msg::ColorRGBA turbo(double v, float alpha)
{
  const double t = std::clamp(v, 0.0, 1.0);
  const double r = 0.13572138 + t * (4.61539260 + t * (-42.66032258 + t * (132.13108234 +
    t * (-152.94239396 + t * 59.28637943))));
  const double g = 0.09140261 + t * (2.19418839 + t * (4.84296658 + t * (-14.18503333 +
    t * (4.27729857 + t * 2.82956604))));
  const double b = 0.10667330 + t * (12.64194608 + t * (-60.58204836 + t * (110.36276771 +
    t * (-89.90310912 + t * 27.34824973))));
  return rgba(
    static_cast<float>(std::clamp(r, 0.0, 1.0)), static_cast<float>(std::clamp(g, 0.0, 1.0)),
    static_cast<float>(std::clamp(b, 0.0, 1.0)), alpha);
}
}  // namespace

visualization_msgs::msg::MarkerArray scanMarkers(
  const ScanResult & scan, double base_z, bool show_rays, const std::string & frame,
  const builtin_interfaces::msg::Time & stamp)
{
  Marker hits = baseMarker(frame, stamp, "hits", Marker::SPHERE_LIST);
  hits.scale.x = hits.scale.y = hits.scale.z = 0.03;
  Marker rays = baseMarker(frame, stamp, "rays", Marker::LINE_LIST);
  rays.scale.x = 0.003;
  rays.color = rgba(0.0f, 1.0f, 0.0f, 0.5f);
  for (int k = 0; k < kNumPoints; ++k) {
    const double z = base_z - scan.heights[k];
    hits.points.push_back(point(scan.wx[k], scan.wy[k], z));
    hits.colors.push_back(sourceColor(scan, k));
    rays.points.push_back(point(scan.wx[k], scan.wy[k], base_z));
    rays.points.push_back(point(scan.wx[k], scan.wy[k], z));
  }
  if (!show_rays) {
    rays.action = Marker::DELETE;
    rays.points.clear();
  }
  visualization_msgs::msg::MarkerArray out;
  out.markers = {hits, rays};
  return out;
}

visualization_msgs::msg::MarkerArray mapMarkers(
  const RollingHeightMap & map, double feet_z, const std::string & frame,
  const builtin_interfaces::msg::Time & stamp)
{
  Marker tiles = baseMarker(frame, stamp, "map", Marker::CUBE_LIST);
  tiles.scale.x = tiles.scale.y = map.cellSize();
  tiles.scale.z = 0.01;
  map.forEachKnown(
    [&](long gx, long gy, const Cell & cell) {
      tiles.points.push_back(point(map.cellCenter(gx), map.cellCenter(gy), cell.height));
      tiles.colors.push_back(turbo((cell.height - (feet_z - 0.3)) / 0.6, 0.8f));
    });
  visualization_msgs::msg::MarkerArray out;
  out.markers.push_back(tiles);
  return out;
}

visualization_msgs::msg::MarkerArray deleteAll(const std::string & frame)
{
  Marker m;
  m.header.frame_id = frame;
  m.action = Marker::DELETEALL;
  visualization_msgs::msg::MarkerArray out;
  out.markers.push_back(m);
  return out;
}

}  // namespace height_map
