#include <algorithm>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace mapping_utils
{

using Cloud = sensor_msgs::msg::PointCloud2;

// Merges consecutive LiDAR messages into one longer scan for FAST_LIO: each L1 message sees
// only part of the room. FAST_LIO deskews from each point's time, so the later messages'
// point times are shifted by their offset from the first message.
class LidarMerger : public rclcpp::Node
{
public:
  LidarMerger()
  : Node("lidar_merger")
  {
    const auto input = declare_parameter<std::string>("input_topic", "/utlidar/cloud");
    const auto output = declare_parameter<std::string>("output_topic", "/utlidar/cloud_merged");
    // 1 = pass-through
    merge_count_ = std::max<int64_t>(1, declare_parameter<int64_t>("merge_count", 3));
    // Never merge across a dropout (a lost L1 message is a gap of about 0.07 s): largest gap
    // from one message's last point to the next.
    max_gap_ = declare_parameter<double>("max_gap", 0.1);
    // Per-point time in seconds from the header stamp (L1: "time")
    time_field_ = declare_parameter<std::string>("time_field", "time");

    pub_ = create_publisher<Cloud>(output, rclcpp::SensorDataQoS());
    sub_ = create_subscription<Cloud>(
      input, rclcpp::SensorDataQoS(),
      std::bind(&LidarMerger::cloudCallback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "merging %ld messages: %s -> %s", static_cast<long>(merge_count_),
      input.c_str(), output.c_str());
  }

private:
  void cloudCallback(Cloud::UniquePtr msg)
  {
    if (merge_count_ == 1) {
      pub_->publish(std::move(msg));
      return;
    }
    const auto * time = timeField(*msg);
    if (!time) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "no float '%s' field, passing clouds through",
        time_field_.c_str());
      flush();
      pub_->publish(std::move(msg));
      return;
    }

    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    if (!batch_.empty()) {
      const double gap = stamp - batch_end_;
      if (gap < -1e-3) {
        // Time went back (bag loop, clock reset): FAST_LIO would drop it anyway.
        RCLCPP_WARN(get_logger(), "cloud stamp went back %.3f s, starting over", -gap);
        batch_.clear();
      } else if (gap > max_gap_ || !sameLayout(*batch_.front(), *msg)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "cloud gap %.3f s or layout change, sending a short scan",
          gap);
        flush();
      }
    }
    batch_end_ = stamp + lastTime(*msg, *time);
    batch_.push_back(std::move(msg));
    if (static_cast<int64_t>(batch_.size()) >= merge_count_) {flush();}
  }

  // Publishes whatever is batched as one cloud.
  void flush()
  {
    if (batch_.empty()) {return;}
    if (batch_.size() == 1) {
      pub_->publish(std::move(batch_.front()));
      batch_.clear();
      return;
    }

    const Cloud & first = *batch_.front();
    const auto * time = timeField(first);
    const double t0 = rclcpp::Time(first.header.stamp).seconds();
    auto out = std::make_unique<Cloud>();
    out->header = first.header;
    out->fields = first.fields;
    out->is_bigendian = first.is_bigendian;
    out->point_step = first.point_step;
    out->height = 1;
    out->is_dense = true;
    std::size_t points = 0;
    for (const auto & m : batch_) {points += static_cast<std::size_t>(m->width) * m->height;}
    out->width = static_cast<uint32_t>(points);
    out->row_step = out->width * out->point_step;
    out->data.reserve(out->row_step);

    for (const auto & m : batch_) {
      const std::size_t start = out->data.size();
      const std::size_t row_bytes = static_cast<std::size_t>(m->width) * m->point_step;
      for (uint32_t r = 0; r < m->height; ++r) {
        const auto * row = m->data.data() + static_cast<std::size_t>(r) * m->row_step;
        out->data.insert(out->data.end(), row, row + row_bytes);
      }
      shiftTimes(
        out->data.data() + start, out->data.size() - start, out->point_step, *time,
        rclcpp::Time(m->header.stamp).seconds() - t0);
      out->is_dense = out->is_dense && m->is_dense;
    }
    batch_.clear();
    pub_->publish(std::move(out));
  }

  const sensor_msgs::msg::PointField * timeField(const Cloud & c) const
  {
    for (const auto & f : c.fields) {
      if (f.name == time_field_ && !c.is_bigendian &&
        (f.datatype == sensor_msgs::msg::PointField::FLOAT32 ||
        f.datatype == sensor_msgs::msg::PointField::FLOAT64))
      {
        return &f;
      }
    }
    return nullptr;
  }

  static double readTime(const uint8_t * p, const sensor_msgs::msg::PointField & f)
  {
    if (f.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      float v;
      std::memcpy(&v, p + f.offset, sizeof(v));
      return v;
    }
    double v;
    std::memcpy(&v, p + f.offset, sizeof(v));
    return v;
  }

  // The L1 puts its latest point last.
  static double lastTime(const Cloud & c, const sensor_msgs::msg::PointField & f)
  {
    if (c.width == 0 || c.height == 0) {return 0.0;}
    const std::size_t last = static_cast<std::size_t>(c.height - 1) * c.row_step +
      static_cast<std::size_t>(c.width - 1) * c.point_step;
    return readTime(c.data.data() + last, f);
  }

  static void shiftTimes(
    uint8_t * data, std::size_t bytes, uint32_t step, const sensor_msgs::msg::PointField & f,
    double dt)
  {
    if (dt == 0.0) {return;}
    for (std::size_t i = 0; i + step <= bytes; i += step) {
      if (f.datatype == sensor_msgs::msg::PointField::FLOAT32) {
        float v;
        std::memcpy(&v, data + i + f.offset, sizeof(v));
        v += static_cast<float>(dt);
        std::memcpy(data + i + f.offset, &v, sizeof(v));
      } else {
        double v;
        std::memcpy(&v, data + i + f.offset, sizeof(v));
        v += dt;
        std::memcpy(data + i + f.offset, &v, sizeof(v));
      }
    }
  }

  static bool sameLayout(const Cloud & a, const Cloud & b)
  {
    if (a.point_step != b.point_step || a.is_bigendian != b.is_bigendian ||
      a.header.frame_id != b.header.frame_id || a.fields.size() != b.fields.size())
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

  int64_t merge_count_;
  double max_gap_;
  std::string time_field_;

  std::vector<Cloud::UniquePtr> batch_;
  double batch_end_{0.0};   // stamp of the batch's last point

  rclcpp::Publisher<Cloud>::SharedPtr pub_;
  rclcpp::Subscription<Cloud>::SharedPtr sub_;
};

}  // namespace mapping_utils

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mapping_utils::LidarMerger>());
  rclcpp::shutdown();
  return 0;
}
