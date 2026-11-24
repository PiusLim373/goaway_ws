// height_map_node: the raw L1 cloud -> the training's 17 x 11 height scan (goaway_msgs/HeightScan),
// drawn for Lichtblick like the MuJoCo viewer. A ROS node, separate from goaway_locomotion_node,
// which reads the scan as unitree_go/HeightMap on /height_scan (rt/height_scan) while it can be
// trusted. mode "instant" (perceptive v3): only the last window of points, moved with
// goaway_odometry's short-term legs + IMU pose. mode "map" (perceptive v2): a rolling height map
// on our odom (legs or fused, not Unitree's).

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <goaway_msgs/msg/height_scan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unitree_go/msg/height_map.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "height_map/grid.hpp"
#include "height_map/instant_scan.hpp"
#include "height_map/markers.hpp"
#include "height_map/rolling_height_map.hpp"
#include "height_map/scan_lookup.hpp"
#include "height_map/self_filter.hpp"

namespace height_map
{

namespace
{
constexpr double kJointHistory = 0.5;   // s of /lowstate kept for the leg cutout
constexpr double kMaxPoseLag = 0.1;     // s: newest odom may be this much older than a point

double toSec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

tf2::TimePoint toTimePoint(double t)
{
  return tf2::TimePoint(std::chrono::nanoseconds(static_cast<int64_t>(std::llround(t * 1e9))));
}

Eigen::Vector3d vec3(const std::vector<double> & v)
{
  return Eigen::Vector3d(v.at(0), v.at(1), v.at(2));
}

double yawOf(const Eigen::Isometry3d & pose)
{
  const Eigen::Vector3d x = pose.linear().col(0);
  return std::atan2(x.y(), x.x());
}
}  // namespace

class HeightMapNode : public rclcpp::Node
{
public:
  HeightMapNode()
  : Node("height_map"),
    map_(
      declare_parameter<double>("cell_size", 0.05),
      static_cast<int>(declare_parameter<int>("map_cells", 60)),
      CellRule{
        declare_parameter<double>("blend_window", 1.0), declare_parameter<double>("blend", 0.5),
        declare_parameter<double>("rise_confirm", 0.04)}),
    filter_config_{
      vec3(declare_parameter<std::vector<double>>("body_box_min", {-0.33, -0.12, -0.10})),
      vec3(declare_parameter<std::vector<double>>("body_box_max", {0.40, 0.12, 0.20})),
      declare_parameter<double>("leg_radius", 0.08)},
    instant_(
      declare_parameter<double>("window", 0.2), 0.5 * get_parameter("cell_size").as_double())
  {
    const std::string mode = declare_parameter<std::string>("mode", "instant");
    instant_mode_ = mode != "map";
    if (instant_mode_ && mode != "instant") {
      RCLCPP_WARN(get_logger(), "unknown mode \"%s\", using instant", mode.c_str());
    }
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    footprint_frame_ = declare_parameter<std::string>("footprint_frame", "base_footprint");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    // Instant: goaway_odometry's legs + IMU tree, a twin of odom -> base_footprint -> base_link
    pose_frame_ = instant_mode_ ?
      declare_parameter<std::string>("local_frame", "local_odom") : odom_frame_;
    pose_base_frame_ = instant_mode_ ?
      declare_parameter<std::string>("local_base_frame", "local_base") : base_frame_;
    pose_footprint_frame_ = instant_mode_ ?
      declare_parameter<std::string>("local_footprint_frame", "local_footprint") : footprint_frame_;
    // Instant: not ready once the newest point is this old
    cloud_timeout_ = declare_parameter<double>("cloud_timeout", 0.3);
    // Points nearer than this are the LiDAR's own window; farther ones can't reach the map.
    min_range_ = declare_parameter<double>("min_range", 0.1);
    max_range_ = declare_parameter<double>("max_range", 3.0);
    // Highest surface counted (m above base_link): the training rays start 1 m up, so walls
    // read at that height.
    max_above_base_ = declare_parameter<double>("max_above_base", 1.0);
    // Points within this much time share one pose and one leg pose.
    slice_time_ = declare_parameter<double>("slice_time", 0.008);
    lookup_.fresh_time = declare_parameter<double>("fresh_time", 1.0);
    lookup_.fill_radius = declare_parameter<double>("fill_radius", 0.15);
    lookup_.clamp_min = declare_parameter<double>("clamp_min", -0.5);
    lookup_.clamp_max = declare_parameter<double>("clamp_max", 1.3);
    // Ready once this share of the grid is known (own or neighbour cell).
    ready_fraction_ = declare_parameter<double>("ready_fraction", 0.8);
    const double output_rate = declare_parameter<double>("output_rate", 50.0);
    // Markers go over rosbridge, so slower than the scan.
    markers_period_ = 1.0 / declare_parameter<double>("markers_rate", 10.0);
    map_markers_period_ = 1.0 / declare_parameter<double>("map_markers_rate", 3.0);
    show_rays_ = declare_parameter<bool>("show_rays", false);
    const double stats_period = declare_parameter<double>("stats_period", 0.0);   // s, 0 = off

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      declare_parameter<std::string>("cloud_topic", "/utlidar/cloud"), rclcpp::SensorDataQoS(),
      std::bind(&HeightMapNode::cloudCallback, this, std::placeholders::_1));
    lowstate_sub_ = create_subscription<unitree_go::msg::LowState>(
      declare_parameter<std::string>("lowstate_topic", "/lowstate"), rclcpp::SensorDataQoS(),
      std::bind(&HeightMapNode::lowStateCallback, this, std::placeholders::_1));
    status_sub_ = create_subscription<std_msgs::msg::String>(
      declare_parameter<std::string>("odom_status_topic", "/goaway_odometry/status"),
      rclcpp::QoS(10), std::bind(&HeightMapNode::statusCallback, this, std::placeholders::_1));

    scan_pub_ = create_publisher<goaway_msgs::msg::HeightScan>("~/scan", 10);
    // For goaway_locomotion_node (SDK DDS), only while ready. Reliable, so any reader matches.
    policy_scan_pub_ = create_publisher<unitree_go::msg::HeightMap>(
      declare_parameter<std::string>("policy_scan_topic", "/height_scan"), rclcpp::QoS(1));
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", 1);
    map_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/map_markers", 1);

    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / output_rate),
      std::bind(&HeightMapNode::timerCallback, this));
    RCLCPP_INFO(get_logger(), "mode %s", instant_mode_ ? "instant" : "map");
    if (stats_period > 0.0) {
      stats_timer_ = create_wall_timer(
        std::chrono::duration<double>(stats_period), std::bind(&HeightMapNode::logStats, this));
    }
  }

private:
  struct Slice
  {
    bool tried = false;
    bool ok = false;
    double t = 0.0;
    Eigen::Isometry3d odom_base = Eigen::Isometry3d::Identity();
    SelfFilter filter;
  };

  void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg)
  {
    std::array<double, 12> q;
    for (int i = 0; i < 12; ++i) {q[i] = msg->motor_state[i].q;}
    const double t = now().seconds();
    if (!joints_.empty() && t < joints_.back().first) {joints_.clear();}   // replay restarted
    joints_.emplace_back(t, q);
    while (t - joints_.front().first > kJointHistory) {joints_.pop_front();}
  }

  void statusCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    // {"source": ..., "paused": true, ...} from goaway_odometry
    static const std::regex kSource("\"source\"\\s*:\\s*\"(\\w+)\"");
    static const std::regex kPaused("\"paused\"\\s*:\\s*(true|false)");
    std::smatch m;
    if (!instant_mode_ && std::regex_search(msg->data, m, kSource)) {
      // Only our odom (legs or fused) carries the floor height the map is built on.
      const bool own = m[1] == "legs" || m[1] == "fused";
      if (own != own_odom_) {
        RCLCPP_INFO(get_logger(), "odom source %s: %s", m[1].str().c_str(),
          own ? "map cleared, scan for the policy on" : "no scan for the policy");
        if (own) {
          // Built on sport odom (no floor height) so far: start over.
          map_.clear();
        }
      }
      own_odom_ = own;
    }
    if (!std::regex_search(msg->data, m, kPaused)) {return;}
    const bool paused = m[1] == "true";
    if (paused && !paused_) {
      // Odom freezes while the robot moves: what the map holds no longer lines up. The instant
      // window starts over too (goaway_odometry restarts the local height after the skill).
      map_.clear();
      instant_.clear();
      markers_pub_->publish(deleteAll(markersFrame()));
      map_markers_pub_->publish(deleteAll(odom_frame_));
      RCLCPP_INFO(get_logger(), "odom paused: %s cleared", instant_mode_ ? "window" : "map");
    } else if (!paused && paused_) {
      RCLCPP_INFO(get_logger(), "odom resumed: mapping again");
    }
    paused_ = paused;
  }

  // Joint angles nearest to t
  std::optional<std::array<double, 12>> jointsAt(double t) const
  {
    if (joints_.empty()) {return std::nullopt;}
    auto it = std::lower_bound(
      joints_.begin(), joints_.end(), t,
      [](const auto & e, double s) {return e.first < s;});
    if (it == joints_.end()) {--it;}
    if (it != joints_.begin() && t - std::prev(it)->first < it->first - t) {--it;}
    if (std::abs(it->first - t) > 0.05) {return std::nullopt;}
    return it->second;
  }

  // odom -> base_link (instant: local_odom -> local_base) at t, or the newest if t is just past it
  bool poseAt(double t, Eigen::Isometry3d & out)
  {
    try {
      out = tf2::transformToEigen(
        tf_buffer_->lookupTransform(pose_frame_, pose_base_frame_, toTimePoint(t)));
      return true;
    } catch (const tf2::TransformException &) {
    }
    try {
      const auto latest =
        tf_buffer_->lookupTransform(pose_frame_, pose_base_frame_, tf2::TimePointZero);
      if (std::abs(toSec(latest.header.stamp) - t) > kMaxPoseLag) {return false;}
      out = tf2::transformToEigen(latest);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (paused_) {return;}
    if (!have_base_lidar_) {
      try {
        base_lidar_ = tf2::transformToEigen(
          tf_buffer_->lookupTransform(base_frame_, msg->header.frame_id, tf2::TimePointZero));
        have_base_lidar_ = true;
      } catch (const tf2::TransformException & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "no %s -> %s from the URDF yet: %s",
          base_frame_.c_str(), msg->header.frame_id.c_str(), e.what());
        return;
      }
    }
    const auto start = std::chrono::steady_clock::now();
    const double stamp = toSec(msg->header.stamp);
    // Each point's own time (s after the stamp); the L1's last point is its latest.
    bool has_time = false;
    for (const auto & f : msg->fields) {
      has_time = has_time ||
        (f.name == "time" && f.datatype == sensor_msgs::msg::PointField::FLOAT32);
    }

    slices_.clear();
    points_.clear();
    times_.clear();
    double newest = stamp;
    Eigen::Isometry3d newest_pose = Eigen::Isometry3d::Identity();
    bool have_pose = false;
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"), iy(*msg, "y"), iz(*msg, "z");
    std::optional<sensor_msgs::PointCloud2ConstIterator<float>> it;
    if (has_time) {it.emplace(*msg, "time");}
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const double dt = it ? static_cast<double>(**it) : 0.0;
      if (it) {++*it;}
      const Eigen::Vector3d p(*ix, *iy, *iz);
      if (!p.allFinite()) {continue;}
      const double r = p.norm();
      if (r < min_range_ || r > max_range_) {continue;}

      const auto si = static_cast<size_t>(std::max(0.0, std::floor(dt / slice_time_)));
      if (si >= slices_.size()) {
        slices_.resize(si + 1, Slice{false, false, 0.0, {}, SelfFilter(filter_config_)});
      }
      Slice & s = slices_[si];
      if (!s.tried) {
        s.tried = true;
        const double ts = stamp + (static_cast<double>(si) + 0.5) * slice_time_;
        s.t = ts;
        const auto q = jointsAt(ts);
        s.ok = q && poseAt(ts, s.odom_base);
        if (s.ok) {
          s.filter.setJoints(*q);
          if (ts >= newest) {
            newest = ts;
            newest_pose = s.odom_base;
            have_pose = true;
          }
        }
      }
      if (!s.ok) {continue;}
      const Eigen::Vector3d pb = base_lidar_ * p;
      if (s.filter.isSelf(pb)) {continue;}
      const Eigen::Vector3d pw = s.odom_base * pb;
      if (pw.z() > s.odom_base.translation().z() + max_above_base_) {continue;}
      points_.push_back(pw);
      times_.push_back(s.t);
    }
    if (!have_pose) {
      ++stats_.clouds_skipped;
      return;
    }
    if (instant_mode_) {
      instant_.add(points_, times_);
    } else {
      map_.recenter(newest_pose.translation().x(), newest_pose.translation().y());
      map_.update(points_, newest);
    }

    ++stats_.clouds;
    stats_.points += points_.size();
    const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    stats_.update_ms += ms;
    stats_.update_ms_max = std::max(stats_.update_ms_max, ms);
  }

  void timerCallback()
  {
    Eigen::Isometry3d odom_base, odom_footprint;
    builtin_interfaces::msg::Time stamp;
    try {
      const auto base =
        tf_buffer_->lookupTransform(pose_frame_, pose_base_frame_, tf2::TimePointZero);
      stamp = base.header.stamp;
      odom_base = tf2::transformToEigen(base);
      odom_footprint = tf2::transformToEigen(
        tf_buffer_->lookupTransform(pose_frame_, pose_footprint_frame_, tf2_ros::fromMsg(stamp)));
    } catch (const tf2::TransformException &) {
      return;
    }
    const double t = toSec(stamp);
    if (t == last_output_) {return;}   // no newer odom
    last_output_ = t;

    const double x = odom_footprint.translation().x();
    const double y = odom_footprint.translation().y();
    const double yaw = yawOf(odom_footprint);
    const double base_z = odom_base.translation().z();
    const double feet_z = odom_footprint.translation().z();   // the floor under the planted feet
    const ScanResult scan = instant_mode_ ?
      instant_.scan(x, y, base_z, yaw, feet_z, lookup_) :
      lookupScan(map_, x, y, base_z, yaw, feet_z, t, lookup_);
    const int known = kNumPoints - scan.count[kFeet];
    // Instant: any recent cloud will do, the fill covers the rest as in training.
    const bool ready = instant_mode_ ?
      !paused_ && t - instant_.newest() <= cloud_timeout_ :
      !paused_ && own_odom_ && known >= ready_fraction_ * kNumPoints;

    goaway_msgs::msg::HeightScan out;
    out.header.stamp = stamp;
    out.header.frame_id = markersFrame();
    out.nx = kNx;
    out.ny = kNy;
    out.resolution = static_cast<float>(kResolution);
    out.heights.assign(scan.heights.begin(), scan.heights.end());
    out.source.assign(scan.source.begin(), scan.source.end());
    out.ready = ready;
    scan_pub_->publish(out);
    if (out.ready) {
      // Silent otherwise: the controller then walks on flat ground and reports the scan lost.
      unitree_go::msg::HeightMap policy;
      policy.stamp = t;
      policy.frame_id = base_frame_;
      policy.resolution = static_cast<float>(kResolution);
      policy.width = kNx;
      policy.height = kNy;
      policy.origin = {static_cast<float>(x), static_cast<float>(y)};
      policy.data = out.heights;
      policy_scan_pub_->publish(policy);
    }

    ++stats_.outputs;
    stats_.ready += out.ready ? 1 : 0;
    for (int i = 0; i < 4; ++i) {stats_.source[i] += scan.count[i];}

    if (paused_) {return;}
    // Throttled on odom time; time going back (a replay restarting) resets it.
    if (t < last_markers_ || t - last_markers_ >= markers_period_) {
      // Instant: base-relative, drawn on base_footprint (local_footprint's twin) of the odom tree
      markers_pub_->publish(scanMarkers(
        scan, instant_mode_ ? base_z - feet_z : base_z, show_rays_, markersFrame(), stamp));
      last_markers_ = t;
    }
    if (instant_mode_) {return;}
    if (t < last_map_markers_ || t - last_map_markers_ >= map_markers_period_) {
      map_markers_pub_->publish(mapMarkers(map_, feet_z, odom_frame_, stamp));
      last_map_markers_ = t;
    }
  }

  // Frame of the scan's markers and HeightScan
  const std::string & markersFrame() const {return instant_mode_ ? footprint_frame_ : odom_frame_;}

  void logStats()
  {
    const double total = std::max(1, stats_.outputs) * static_cast<double>(kNumPoints);
    RCLCPP_INFO(
      get_logger(),
      "seen %.0f%% remembered %.0f%% neighbour %.0f%% feet %.0f%%, ready %.0f%% | clouds %d "
      "(skipped %d), %.0f points kept each, update %.2f ms avg %.2f max",
      100.0 * stats_.source[kSeen] / total, 100.0 * stats_.source[kRemembered] / total,
      100.0 * stats_.source[kNeighbour] / total, 100.0 * stats_.source[kFeet] / total,
      100.0 * stats_.ready / std::max(1, stats_.outputs), stats_.clouds, stats_.clouds_skipped,
      stats_.clouds > 0 ? static_cast<double>(stats_.points) / stats_.clouds : 0.0,
      stats_.clouds > 0 ? stats_.update_ms / stats_.clouds : 0.0, stats_.update_ms_max);
    stats_ = Stats{};
  }

  struct Stats
  {
    int outputs = 0;
    int ready = 0;
    std::array<long, 4> source{};
    int clouds = 0;
    int clouds_skipped = 0;
    size_t points = 0;
    double update_ms = 0.0;
    double update_ms_max = 0.0;
  };

  // Parameters
  bool instant_mode_ = true;
  std::string odom_frame_, footprint_frame_, base_frame_;
  std::string pose_frame_, pose_base_frame_, pose_footprint_frame_;   // the pose the points use
  double min_range_, max_range_, max_above_base_, slice_time_, ready_fraction_, cloud_timeout_;
  double markers_period_, map_markers_period_;
  bool show_rays_;
  LookupConfig lookup_;

  // State
  RollingHeightMap map_;
  SelfFilterConfig filter_config_;
  InstantScan instant_;
  std::deque<std::pair<double, std::array<double, 12>>> joints_;
  Eigen::Isometry3d base_lidar_ = Eigen::Isometry3d::Identity();
  bool have_base_lidar_ = false;
  bool paused_ = false;
  bool own_odom_ = false;
  std::vector<Slice> slices_;
  std::vector<Eigen::Vector3d> points_;
  std::vector<double> times_;
  double last_output_ = -1.0;
  double last_markers_ = -1e9;
  double last_map_markers_ = -1e9;
  Stats stats_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Publisher<goaway_msgs::msg::HeightScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<unitree_go::msg::HeightMap>::SharedPtr policy_scan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_markers_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

}  // namespace height_map

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<height_map::HeightMapNode>());
  rclcpp::shutdown();
  return 0;
}
