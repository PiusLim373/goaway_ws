#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace mapping_utils
{

using Cloud = sensor_msgs::msg::PointCloud2;
using Scan = sensor_msgs::msg::LaserScan;

// Stacks the last `window` seconds of LiDAR clouds in a fixed frame (odom), each moved there with
// the pose at its own time, so the stack doesn't smear while the robot walks or turns (one L1
// message only sees a sliver of the room). Publishes the stack, and a 2D scan cut from a band
// above the floor (as pointcloud_to_laserscan, but the band edges can rise with distance), and
// each cloud's points inside that band for the costmap.
class CloudAccumulator : public rclcpp::Node
{
public:
  CloudAccumulator()
  : Node("cloud_accumulator")
  {
    const auto input = declare_parameter<std::string>("input_topic", "/utlidar/cloud_filtered");
    const auto output =
      declare_parameter<std::string>("output_topic", "/utlidar/cloud_accumulated");
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
    window_ = declare_parameter<double>("window", 1.0);
    // Publish on every Nth input cloud (L1: 15 Hz in, 5 Hz out at 3)
    publish_every_ = std::max<int64_t>(1, declare_parameter<int64_t>("publish_every", 3));
    // How long to wait for the fixed frame's pose at a cloud's time
    transform_timeout_ = declare_parameter<double>("transform_timeout", 0.1);
    // Per-point time in seconds from the header stamp (L1: "time"); each cloud is placed with
    // the pose halfway through its points. Without the field, the pose at the stamp.
    time_field_ = declare_parameter<std::string>("time_field", "time");

    // Scan (empty topic = off): points between min_height and max_height in scan_frame, nearest
    // per angle_increment. Both edges rise by distance * tan(height_rise), so a small tilt error
    // can't lift the far floor (near-flat rays) into the band.
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "");
    scan_frame_ = declare_parameter<std::string>("scan_frame", "base_footprint");
    min_height_ = declare_parameter<double>("min_height", 0.10);
    max_height_ = declare_parameter<double>("max_height", 0.45);
    const double rise_deg = declare_parameter<double>("height_rise", 0.0);
    rise_ = std::tan(std::clamp(rise_deg, 0.0, 30.0) * M_PI / 180.0);
    angle_min_ = declare_parameter<double>("angle_min", -M_PI);
    angle_max_ = declare_parameter<double>("angle_max", M_PI);
    angle_increment_ = std::max(1e-4, declare_parameter<double>("angle_increment", M_PI / 180.0));
    range_min_ = declare_parameter<double>("range_min", 0.3);
    range_max_ = declare_parameter<double>("range_max", 8.0);
    use_inf_ = declare_parameter<bool>("use_inf", true);
    // Band cloud (empty topic = off): each cloud's points inside the scan band, in scan_frame at
    // the cloud's time. Every cloud, also when empty, so the costmap's source never goes stale.
    const auto obstacle_topic = declare_parameter<std::string>("obstacle_topic", "");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = create_publisher<Cloud>(output, rclcpp::SensorDataQoS());
    if (!scan_topic.empty()) {
      scan_pub_ = create_publisher<Scan>(scan_topic, rclcpp::SensorDataQoS());
    }
    if (!obstacle_topic.empty()) {
      obstacle_pub_ = create_publisher<Cloud>(obstacle_topic, rclcpp::SensorDataQoS());
    }
    sub_ = create_subscription<Cloud>(
      input, rclcpp::SensorDataQoS(),
      std::bind(&CloudAccumulator::cloudCallback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "stacking %.2f s of %s in %s -> %s", window_, input.c_str(),
      fixed_frame_.c_str(), output.c_str());
    if (scan_pub_) {
      RCLCPP_INFO(
        get_logger(), "scan %s in %s: %.2f-%.2f m, edges rising %.1f deg with distance",
        scan_topic.c_str(), scan_frame_.c_str(), min_height_, max_height_, rise_deg);
    }
    if (obstacle_pub_) {
      RCLCPP_INFO(get_logger(), "band cloud %s in %s", obstacle_topic.c_str(), scan_frame_.c_str());
    }
  }

private:
  struct Entry
  {
    rclcpp::Time time;   // when its pose was taken
    Cloud cloud;         // in the fixed frame
  };

  void cloudCallback(Cloud::UniquePtr msg)
  {
    const rclcpp::Time time = rclcpp::Time(msg->header.stamp) +
      rclcpp::Duration::from_seconds(midTime(*msg));
    if (!window_entries_.empty() && time < window_entries_.back().time) {
      // Time went back (bag loop, clock reset)
      window_entries_.clear();
    }

    Entry entry;
    entry.time = time;
    try {
      // The TF listener has its own thread, so waiting here is fine.
      const auto tf = tf_buffer_->lookupTransform(
        fixed_frame_, msg->header.frame_id, time,
        rclcpp::Duration::from_seconds(transform_timeout_));
      tf2::doTransform(*msg, entry.cloud, tf);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "no %s -> %s at the cloud's time: %s",
        fixed_frame_.c_str(), msg->header.frame_id.c_str(), e.what());
      return;
    }
    if (!window_entries_.empty() && !sameLayout(window_entries_.front().cloud, entry.cloud)) {
      window_entries_.clear();
    }
    window_entries_.push_back(std::move(entry));
    if (obstacle_pub_ && listened(obstacle_pub_)) {
      publishObstacles(window_entries_.back().cloud, time);
    }
    while (!window_entries_.empty() &&
      (time - window_entries_.front().time).seconds() > window_)
    {
      window_entries_.pop_front();
    }
    if (++received_ % publish_every_ == 0) {
      // The stack is for viewing: only built when someone listens.
      if (listened(pub_)) {publish(time);}
      if (scan_pub_) {publishScan(time);}
    }
  }

  static bool listened(const rclcpp::Publisher<Cloud>::SharedPtr & pub)
  {
    return pub->get_subscription_count() + pub->get_intra_process_subscription_count() > 0;
  }

  // fixed_frame -> scan_frame at time
  bool toScanFrame(const rclcpp::Time & time, tf2::Transform & to_scan)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        scan_frame_, fixed_frame_, time, rclcpp::Duration::from_seconds(transform_timeout_));
      const auto & r = tf.transform.rotation;
      const auto & t = tf.transform.translation;
      to_scan = tf2::Transform(tf2::Quaternion(r.x, r.y, r.z, r.w), tf2::Vector3(t.x, t.y, t.z));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "no %s -> %s for the scan band: %s",
        fixed_frame_.c_str(), scan_frame_.c_str(), e.what());
      return false;
    }
    return true;
  }

  // Inside the band, a point in scan_frame; range = its distance in the xy plane.
  bool inBand(const tf2::Vector3 & p, double & range) const
  {
    range = std::hypot(p.x(), p.y());
    if (!std::isfinite(range) || range < range_min_ || range > range_max_) {return false;}
    const double rise = range * rise_;
    return p.z() >= min_height_ + rise && p.z() <= max_height_ + rise;
  }

  // One cloud's band points as xyz in scan_frame.
  void publishObstacles(const Cloud & cloud, const rclcpp::Time & time)
  {
    tf2::Transform to_scan;
    if (!toScanFrame(time, to_scan)) {return;}

    auto out = std::make_unique<Cloud>();
    out->header.stamp = time;
    out->header.frame_id = scan_frame_;
    out->height = 1;
    out->width = 0;
    sensor_msgs::PointCloud2Modifier modifier(*out);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(static_cast<std::size_t>(cloud.width) * cloud.height);
    sensor_msgs::PointCloud2Iterator<float> ox(*out, "x"), oy(*out, "y"), oz(*out, "z");
    std::size_t kept = 0;
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
      const tf2::Vector3 p = to_scan * tf2::Vector3(*x, *y, *z);
      double range;
      if (!inBand(p, range)) {continue;}
      *ox = static_cast<float>(p.x());
      *oy = static_cast<float>(p.y());
      *oz = static_cast<float>(p.z());
      ++ox; ++oy; ++oz; ++kept;
    }
    modifier.resize(kept);
    out->is_dense = true;
    obstacle_pub_->publish(std::move(out));
  }

  // The band of the stack as a 2D scan in scan_frame, at the newest cloud's pose time.
  void publishScan(const rclcpp::Time & time)
  {
    tf2::Transform to_scan;
    if (!toScanFrame(time, to_scan)) {return;}

    auto scan = std::make_unique<Scan>();
    scan->header.stamp = time;
    scan->header.frame_id = scan_frame_;
    scan->angle_min = angle_min_;
    scan->angle_max = angle_max_;
    scan->angle_increment = angle_increment_;
    scan->time_increment = 0.0;
    scan->scan_time = last_scan_ ? static_cast<float>((time - *last_scan_).seconds()) : 0.0f;
    scan->range_min = range_min_;
    scan->range_max = range_max_;
    const auto beams = static_cast<std::size_t>(
      std::ceil((angle_max_ - angle_min_) / angle_increment_));
    scan->ranges.assign(
      beams, use_inf_ ? std::numeric_limits<float>::infinity() : range_max_ + 1.0);

    for (const auto & e : window_entries_) {
      sensor_msgs::PointCloud2ConstIterator<float> x(e.cloud, "x"), y(e.cloud, "y"), z(e.cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        const tf2::Vector3 p = to_scan * tf2::Vector3(*x, *y, *z);
        double range;
        if (!inBand(p, range)) {continue;}
        const double angle = std::atan2(p.y(), p.x());
        if (angle < angle_min_ || angle > angle_max_) {continue;}
        const auto i = static_cast<std::size_t>((angle - angle_min_) / angle_increment_);
        if (i < beams && range < scan->ranges[i]) {scan->ranges[i] = static_cast<float>(range);}
      }
    }
    last_scan_ = time;
    scan_pub_->publish(std::move(scan));
  }

  // All clouds in the window as one, stamped with the newest one's pose time.
  void publish(const rclcpp::Time & time)
  {
    const Cloud & first = window_entries_.front().cloud;
    auto out = std::make_unique<Cloud>();
    out->header.stamp = time;
    out->header.frame_id = fixed_frame_;
    out->fields = first.fields;
    out->is_bigendian = first.is_bigendian;
    out->point_step = first.point_step;
    out->height = 1;
    out->is_dense = true;
    std::size_t points = 0;
    for (const auto & e : window_entries_) {
      points += static_cast<std::size_t>(e.cloud.width) * e.cloud.height;
    }
    out->width = static_cast<uint32_t>(points);
    out->row_step = out->width * out->point_step;
    out->data.reserve(out->row_step);
    for (const auto & e : window_entries_) {
      const auto & c = e.cloud;
      const std::size_t row_bytes = static_cast<std::size_t>(c.width) * c.point_step;
      for (uint32_t r = 0; r < c.height; ++r) {
        const auto * row = c.data.data() + static_cast<std::size_t>(r) * c.row_step;
        out->data.insert(out->data.end(), row, row + row_bytes);
      }
      out->is_dense = out->is_dense && c.is_dense;
    }
    pub_->publish(std::move(out));
  }

  // Halfway between the first and the last point's time, from the header stamp.
  double midTime(const Cloud & c) const
  {
    if (c.width == 0 || c.height == 0 || c.is_bigendian) {return 0.0;}
    for (const auto & f : c.fields) {
      if (f.name != time_field_ || f.datatype != sensor_msgs::msg::PointField::FLOAT32) {
        continue;
      }
      const std::size_t last = static_cast<std::size_t>(c.height - 1) * c.row_step +
        static_cast<std::size_t>(c.width - 1) * c.point_step;
      float a, b;
      std::memcpy(&a, c.data.data() + f.offset, sizeof(a));
      std::memcpy(&b, c.data.data() + last + f.offset, sizeof(b));
      return 0.5 * (static_cast<double>(a) + static_cast<double>(b));
    }
    return 0.0;
  }

  static bool sameLayout(const Cloud & a, const Cloud & b)
  {
    if (a.point_step != b.point_step || a.is_bigendian != b.is_bigendian ||
      a.fields.size() != b.fields.size())
    {
      return false;
    }
    for (std::size_t i = 0; i < a.fields.size(); ++i) {
      const auto & x = a.fields[i];
      const auto & y = b.fields[i];
      if (x.name != y.name || x.offset != y.offset || x.datatype != y.datatype ||
        x.count != y.count)
      {
        return false;
      }
    }
    return true;
  }

  std::string fixed_frame_;
  double window_;
  int64_t publish_every_;
  double transform_timeout_;
  std::string time_field_;
  std::string scan_frame_;
  double min_height_, max_height_, rise_;
  double angle_min_, angle_max_, angle_increment_;
  double range_min_, range_max_;
  bool use_inf_;

  std::deque<Entry> window_entries_;   // oldest first
  uint64_t received_{0};
  std::optional<rclcpp::Time> last_scan_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<Cloud>::SharedPtr pub_;
  rclcpp::Publisher<Scan>::SharedPtr scan_pub_;
  rclcpp::Publisher<Cloud>::SharedPtr obstacle_pub_;
  rclcpp::Subscription<Cloud>::SharedPtr sub_;
};

}  // namespace mapping_utils

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mapping_utils::CloudAccumulator>());
  rclcpp::shutdown();
  return 0;
}
