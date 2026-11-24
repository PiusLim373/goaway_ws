#pragma once

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <unitree_go/msg/low_state.hpp>

#include "goaway_odometry/leg_odometry.hpp"

namespace goaway_odometry
{

// A pose flattened to the floor: what odom -> base_footprint carries.
struct Planar
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double z{0.0};   // the floor's height (steps, slopes)
};

// One odom frame across Unitree and HTX locomotion:
//   odom -> base_footprint   x, y, yaw from our estimate or Unitree's odom (sport), and the
//                            floor height under the robot (ours only, held in sport)
//   base_footprint -> base_link   height above the planted feet, roll/pitch from the body IMU
//   odom -> camera_init   the link that places FAST_LIO's frame in odom
// Ours: the legs move the pose at the /lowstate rate (smooth, still when standing), turned by the
// body IMU. odom_source "legs": that alone, with a fixed scale; FAST_LIO only maps, and global
// corrections belong in map -> odom. "fused": FAST_LIO pulls it slowly into place, which also
// teaches the legs their scale. Whenever the source changes, the new one continues from the last
// pose, so odom never jumps.
// Skills (any HTX state but the calm ones) and falls pause it: the fused pose freezes and
// FAST_LIO stops. Calm again, the legs carry on; FAST_LIO starts fresh at the next stop, since
// it calibrates its IMU on the robot being still. A FAST_LIO that turns on its own or keeps
// jumping has lost track: the legs carry on and it restarts the same way. Relocalise after.
class GoawayOdometry : public rclcpp::Node
{
public:
  explicit GoawayOdometry(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class Source { kNone, kSport, kFused };

  // Our pose and the legs' own distance count, kept to compare with FAST_LIO's late poses.
  struct History
  {
    rclcpp::Time stamp;
    Planar pose;
    double leg_forward;   // unscaled leg distance, body frame
    double leg_side;
    double height;        // base_link above the planted feet
    double heading;       // body IMU heading, for FAST_LIO's turn check
  };

  void sportOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void fastlioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg);
  void locomotionStateCallback(const std_msgs::msg::String::SharedPtr msg);
  void statusTimerCallback();

  void updatePause(const rclcpp::Time & t, const tf2::Quaternion & body);
  void updateFloor(
    const std::array<bool, LegOdometry::kLegs> & planted,
    const std::array<double, LegOdometry::kLegs> & depth, double prev_height,
    std::array<double, LegOdometry::kLegs> & foot_z,
    std::array<bool, LegOdometry::kLegs> & anchored, double & floor_z);
  void updateLocal(
    const rclcpp::Time & t, double dt, const tf2::Quaternion & body, const tf2::Vector3 & accel,
    const LegOdometry::Output & leg, const std::array<bool, LegOdometry::kLegs> & on_floor,
    const std::array<double, LegOdometry::kLegs> & depth, double prev_height);
  void requestFastlioPause(bool pause);
  void holdFastlio();
  void updateFastlioStart(const rclcpp::Time & t);
  void fastlioLost(const char * why);
  void selectSource();
  void linkFastlio();
  void correctFused(const rclcpp::Time & stamp, const Planar & lio, double dt);
  void learnScale(
    const History & from, const History & to, const Planar & lio_from, const Planar & lio_to);
  void publishOdom(
    const rclcpp::Time & stamp, const Planar & pose, double vx, double vy, double wz);
  void publishStatus();

  bool sportOk(const rclcpp::Time & now) const;
  bool lookupBaseToImu();
  const History * historyAt(const rclcpp::Time & stamp) const;
  double fastlioSpeed(const rclcpp::Time & stamp) const;
  bool fastlioTurnsAlone(
    const rclcpp::Time & stamp, const Planar & lio, const History & now_h) const;
  tf2::Transform baseInOdom(const Planar & pose, double height) const;
  Planar fastlioPlanar(const tf2::Transform & cam_body) const;
  const char * sourceName(Source source) const;

  // Parameters
  std::string odom_frame_;
  std::string footprint_frame_;
  std::string base_frame_;
  std::string fastlio_world_frame_;
  std::string imu_frame_;
  std::string local_frame_;
  std::string local_base_frame_;
  std::string local_footprint_frame_;
  bool legs_only_;
  bool unitree_own_;
  double foot_radius_;
  double floor_agreement_;
  double sport_timeout_;
  double sport_settle_;
  double lowstate_timeout_;
  double fastlio_timeout_;
  double fastlio_settle_;
  double fastlio_still_time_;
  double fastlio_lost_turn_;   // rad
  double fastlio_lost_time_;
  double odom_tf_period_;
  double local_tf_period_;
  double fastlio_tau_;
  double fastlio_fast_tau_;
  double fastlio_yaw_tau_;
  double glide_time_;
  double still_speed_;
  double still_turn_;
  double still_time_;
  double carried_speed_;
  double carried_distance_;
  double catch_up_distance_;
  double max_jump_;
  double scale_learn_time_;
  double scale_min_;
  double scale_max_;
  std::vector<std::string> calm_states_;
  double pause_tilt_;   // rad
  double resume_time_;

  Source source_{Source::kNone};

  // Skill pause
  std::string locomotion_state_;
  rclcpp::Time locomotion_state_last_{0, 0, RCL_ROS_TIME};
  bool paused_{false};
  int pauses_{0};
  rclcpp::Time pause_start_{0, 0, RCL_ROS_TIME};
  rclcpp::Time calm_since_{0, 0, RCL_ROS_TIME};   // zero while not calm
  rclcpp::Time fastlio_restart_{0, 0, RCL_ROS_TIME};   // FAST_LIO poses before this are stale
  rclcpp::Time fastlio_pause_sent_{0, 0, RCL_ROS_TIME};
  bool fastlio_waiting_{false};   // held paused until the robot is still

  // Sport: odom = sport_offset_ (+) Unitree's pose. Identity until the first switch back.
  Planar sport_offset_;
  Planar sport_raw_;
  double sport_vx_{0.0};
  double sport_vy_{0.0};
  double sport_wz_{0.0};
  rclcpp::Time sport_last_{0, 0, RCL_ROS_TIME};
  rclcpp::Time sport_since_{0, 0, RCL_ROS_TIME};   // start of the current valid stretch

  // Legs and body IMU (/lowstate)
  LegOdometry legs_;
  tf2::Quaternion tilt_{0.0, 0.0, 0.0, 1.0};   // body attitude without heading
  double body_heading_{0.0};
  double height_;
  double leg_forward_{0.0};
  double leg_side_{0.0};
  double turn_rate_{0.0};
  double gyro_rate_{0.0};   // body turn rate on any axis, lightly smoothed
  rclcpp::Time gyro_quiet_since_{0, 0, RCL_ROS_TIME};   // zero while turning
  tf2::Vector3 still_velocity_{0.0, 0.0, 0.0};   // leg velocity, lightly smoothed
  bool still_{false};
  rclcpp::Time still_since_{0, 0, RCL_ROS_TIME};   // zero while moving
  rclcpp::Time lowstate_last_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tilt_tf_{0, 0, RCL_ROS_TIME};

  // Fused estimate and what it is compared against. Corrections land in fused_ at once and
  // glide into the published pose: shown = fused_ + glide_, glide_ fading to zero.
  Planar fused_;
  Planar glide_;
  std::deque<History> history_;
  double scale_forward_;
  double scale_side_;
  double scale_fwd_xy_;   // learning sums: leg x FAST_LIO and leg x leg, per axis
  double scale_fwd_xx_;
  double scale_side_xy_;
  double scale_side_xx_;
  bool carried_{false};
  // Floor: odom height of each planted foot's centre, set when it was planted
  std::array<double, LegOdometry::kLegs> foot_z_{};
  std::array<bool, LegOdometry::kLegs> foot_anchored_{};
  bool hold_anchored_{false};
  Planar hold_anchor_;   // FAST_LIO pose when the legs went still

  // Local: a short-term pose from the legs and the body IMU alone (no FAST_LIO, no Unitree odom),
  // for height_map_node's instant scan. Drifts, which only matters over its 0.2 s window.
  struct Local
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double vz{0.0};
    double legs_z{0.0};     // base_link from the planted feet
    double legs_vz{0.0};    // its rate, smoothed
    double gravity{9.81};   // vertical accelerometer reading at rest, learned while still
    bool ok{false};
  };
  Local local_;
  std::array<double, LegOdometry::kLegs> local_foot_z_{};
  std::array<bool, LegOdometry::kLegs> local_anchored_{};
  double local_floor_{0.0};
  rclcpp::Time last_local_tf_{0, 0, RCL_ROS_TIME};

  // FAST_LIO: camera_init -> body, recent history for linking at the switch time.
  std::deque<std::pair<rclcpp::Time, tf2::Transform>> fastlio_history_;
  rclcpp::Time fastlio_last_{0, 0, RCL_ROS_TIME};
  rclcpp::Time fastlio_since_{0, 0, RCL_ROS_TIME};
  tf2::Transform odom_to_camera_init_;
  bool linked_{false};
  tf2::Transform base_to_imu_;   // URDF base_link -> utlidar_imu (FAST_LIO's body)
  bool have_base_to_imu_{false};
  bool have_prev_lio_{false};
  rclcpp::Time prev_lio_stamp_{0, 0, RCL_ROS_TIME};
  Planar prev_lio_;
  History prev_lio_history_{rclcpp::Time(0, 0, RCL_ROS_TIME), {}, 0.0, 0.0, 0.0, 0.0};
  int jumps_{0};
  rclcpp::Time last_jump_link_{0, 0, RCL_ROS_TIME};
  rclcpp::Time turn_check_from_{0, 0, RCL_ROS_TIME};   // FAST_LIO stamps from its settle on

  // Last pose sent (or due) on odom -> base_footprint
  Planar output_;
  bool have_output_{false};
  rclcpp::Time last_tf_stamp_{0, 0, RCL_ROS_TIME};

  int links_{0};
  int lost_{0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sport_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fastlio_sub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr locomotion_state_sub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr fastlio_pause_client_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace goaway_odometry
