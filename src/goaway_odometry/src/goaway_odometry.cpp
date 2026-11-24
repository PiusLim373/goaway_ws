#include "goaway_odometry/goaway_odometry.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <regex>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace goaway_odometry
{

namespace
{
constexpr double kHistorySeconds = 3.0;   // our poses and FAST_LIO's, for its late poses
constexpr double kStillSmoothing = 0.1;   // s, leg speed and turn rate for the still test
constexpr double kSpeedWindow = 1.0;      // s, FAST_LIO speed for the carried test
constexpr double kTurnWindow = 2.0;       // s, FAST_LIO's turn vs the body IMU's
// Weight of the starting leg scale in the learning, about one learning window of walking.
constexpr double kScalePrior = 0.01;      // m^2
constexpr double kMinLearnStep = 0.01;    // m of leg travel per FAST_LIO update to learn from
constexpr double kLocomotionStateTimeout = 3.0;   // s, its state topic has a 1 s heartbeat
// Local pose height: the IMU over the short term, the legs over the long term.
constexpr double kLocalLegsSmoothing = 0.1;   // s, legs' vertical speed
constexpr double kLocalSpeedTau = 0.5;        // s, vertical speed pulled to the legs'
constexpr double kLocalHeightTau = 2.0;       // s, height pulled to the legs'
constexpr double kLocalGravityTau = 2.0;      // s, accelerometer at rest, while still

double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}

// Heading from the body's left axis, which stays level on four legs and upright
// (UprightWalk): forward = left x up. Keeps `fallback` while the left axis points up (mid-flip).
double heading(const tf2::Quaternion & q, double fallback)
{
  const tf2::Vector3 left = tf2::quatRotate(q, tf2::Vector3(0.0, 1.0, 0.0));
  if (std::hypot(left.x(), left.y()) < 0.1) {return fallback;}
  return std::atan2(-left.x(), left.y());
}

tf2::Quaternion yawQuat(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return q;
}

// a (+) b
Planar compose(const Planar & a, const Planar & b)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, wrap(a.yaw + b.yaw), a.z + b.z};
}

Planar inverse(const Planar & a)
{
  const double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {-c * a.x - s * a.y, s * a.x - c * a.y, -a.yaw, -a.z};
}

bool finitePose(const geometry_msgs::msg::Pose & p)
{
  const auto & o = p.orientation;
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
         std::isfinite(p.position.z) && std::isfinite(o.x) && std::isfinite(o.y) &&
         std::isfinite(o.z) && std::isfinite(o.w) &&
         o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w > 0.25;
}
}  // namespace

GoawayOdometry::GoawayOdometry(const rclcpp::NodeOptions & options)
: Node("goaway_odometry", options)
{
  const auto sport_topic =
    declare_parameter<std::string>("sport_odom_topic", "/utlidar/robot_odom");
  const auto fastlio_topic = declare_parameter<std::string>("fastlio_odom_topic", "/Odometry");
  const auto lowstate_topic = declare_parameter<std::string>("lowstate_topic", "/lowstate");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  footprint_frame_ = declare_parameter<std::string>("footprint_frame", "base_footprint");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  fastlio_world_frame_ = declare_parameter<std::string>("fastlio_world_frame", "camera_init");
  // FAST_LIO's body is this IMU (/utlidar/imu)
  imu_frame_ = declare_parameter<std::string>("imu_frame", "utlidar_imu");
  // Short-term legs + IMU pose for height_map_node's instant scan, a tree of its own:
  // local_frame -> local_base_frame (base_link's pose), local_frame -> local_footprint_frame
  // (heading only, z = the floor under the planted feet).
  local_frame_ = declare_parameter<std::string>("local_frame", "local_odom");
  local_base_frame_ = declare_parameter<std::string>("local_base_frame", "local_base");
  local_footprint_frame_ =
    declare_parameter<std::string>("local_footprint_frame", "local_footprint");
  const double local_tf_rate = declare_parameter<double>("local_tf_rate", 100.0);
  local_tf_period_ = local_tf_rate > 0.0 ? 1.0 / local_tf_rate : 0.0;
  // Our estimate: "legs" (legs + IMU only) or "fused" (FAST_LIO pulls it into place).
  const auto odom_source = declare_parameter<std::string>("odom_source", "legs");
  legs_only_ = odom_source != "fused";
  if (legs_only_ && odom_source != "legs") {
    RCLCPP_WARN(get_logger(), "odom_source '%s' unknown, using legs", odom_source.c_str());
  }
  // Unitree mode: "own" uses our estimate as in HTX, "sport" Unitree's odom.
  const auto unitree_source = declare_parameter<std::string>("unitree_source", "own");
  unitree_own_ = unitree_source != "sport";
  if (unitree_own_ && unitree_source != "own") {
    RCLCPP_WARN(get_logger(), "unitree_source '%s' unknown, using own", unitree_source.c_str());
  }
  RCLCPP_INFO(
    get_logger(), "odom from %s, in Unitree mode from %s",
    legs_only_ ? "the legs + IMU" : "the legs + IMU + FAST_LIO",
    unitree_own_ ? "the same" : "Unitree's odom");
  foot_radius_ = declare_parameter<double>("foot_radius", 0.02);    // URDF foot sphere
  // m/s: a planted foot counts for the floor only if the body vertical speed it implies is this
  // close to the other planted feet's (median)
  floor_agreement_ = declare_parameter<double>("floor_agreement", 0.2);
  height_ = declare_parameter<double>("default_height", 0.3);      // until the legs are known
  sport_timeout_ = declare_parameter<double>("sport_timeout", 0.3);
  // Back from HTX, Unitree sends NaN for a while, then restarts its pose at zero.
  sport_settle_ = declare_parameter<double>("sport_settle", 1.0);
  lowstate_timeout_ = declare_parameter<double>("lowstate_timeout", 0.1);
  fastlio_timeout_ = declare_parameter<double>("fastlio_timeout", 0.5);
  // FAST_LIO's first poses wander before it settles.
  fastlio_settle_ = declare_parameter<double>("fastlio_settle", 1.5);
  // FAST_LIO calibrates its IMU on its first scan as if the robot were still (a turn becomes
  // gyro bias, and it then turns on its own). So it starts only while still, and again if the
  // robot moves within this long (s).
  fastlio_still_time_ = declare_parameter<double>("fastlio_still_time", 1.0);
  // Lost track: turning this much (deg) more or less than the body IMU over kTurnWindow, or
  // jumping again within fastlio_lost_time (s) of the last relink.
  fastlio_lost_turn_ = declare_parameter<double>("fastlio_lost_turn", 20.0) * M_PI / 180.0;
  fastlio_lost_time_ = declare_parameter<double>("fastlio_lost_time", 5.0);
  // rosbridge relays every /tf, so no faster than this (Hz, 0 = every message).
  const double odom_tf_rate = declare_parameter<double>("odom_tf_rate", 50.0);
  odom_tf_period_ = odom_tf_rate > 0.0 ? 1.0 / odom_tf_rate : 0.0;
  // Time constants (s) of the pull to FAST_LIO: normally, when the legs are known wrong
  // (carried, sliding, far off), and for the heading.
  fastlio_tau_ = declare_parameter<double>("fastlio_tau", 2.0);
  fastlio_fast_tau_ = declare_parameter<double>("fastlio_fast_tau", 0.3);
  fastlio_yaw_tau_ = declare_parameter<double>("fastlio_yaw_tau", 0.7);
  // Corrections glide into the published pose over about this long (s), instead of stepping.
  glide_time_ = declare_parameter<double>("glide_time", 0.2);
  // Still: leg speed and turn rate under these for still_time. Odom then holds.
  still_speed_ = declare_parameter<double>("still_speed", 0.02);
  still_turn_ = declare_parameter<double>("still_turn", 0.05);
  still_time_ = declare_parameter<double>("still_time", 0.5);
  // Carried or sliding: legs still while FAST_LIO moves this fast or this far. Kept above
  // FAST_LIO's own wander while standing.
  carried_speed_ = declare_parameter<double>("carried_speed", 0.15);
  carried_distance_ = declare_parameter<double>("carried_distance", 0.30);
  catch_up_distance_ = declare_parameter<double>("catch_up_distance", 0.2);
  // FAST_LIO moving this much more than the legs in one update is a jump, not motion.
  max_jump_ = declare_parameter<double>("max_jump", 0.3);
  // Leg scale (true distance / legs): feet slide a little each step. Fixed in legs, learned
  // from FAST_LIO while walking in fused. Measured on the robot (HTX walk, L1 start/end match).
  scale_forward_ = declare_parameter<double>("scale_forward", 1.165);
  scale_side_ = declare_parameter<double>("scale_side", 1.15);
  scale_learn_time_ = declare_parameter<double>("scale_learn_time", 20.0);
  scale_min_ = declare_parameter<double>("scale_min", 0.8);
  scale_max_ = declare_parameter<double>("scale_max", 1.5);
  scale_fwd_xx_ = scale_side_xx_ = kScalePrior;
  scale_fwd_xy_ = scale_forward_ * kScalePrior;
  scale_side_xy_ = scale_side_ * kScalePrior;
  // HTX states that move like walking. Any other (skills, Recovery, UprightWalk) pauses odom.
  calm_states_ = declare_parameter<std::vector<std::string>>(
    "calm_states", {"Off", "Damping", "JointLock", "LieDown", "Velocity"});
  // Tilted further than this (deg) is a fall, which no state announces: pause too.
  pause_tilt_ = declare_parameter<double>("pause_tilt", 45.0) * M_PI / 180.0;
  // Calm and level this long (s) before odom and FAST_LIO go again.
  resume_time_ = declare_parameter<double>("resume_time", 1.0);
  const auto locomotion_state_topic =
    declare_parameter<std::string>("locomotion_state_topic", "/goaway_locomotion/state");
  const auto fastlio_pause_service =
    declare_parameter<std::string>("fastlio_pause_service", "/laser_mapping/pause");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

  // Best effort matches the robot's publishers whether they are reliable or not.
  sport_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    sport_topic, rclcpp::SensorDataQoS(),
    std::bind(&GoawayOdometry::sportOdomCallback, this, std::placeholders::_1));
  fastlio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    fastlio_topic, rclcpp::SensorDataQoS(),
    std::bind(&GoawayOdometry::fastlioOdomCallback, this, std::placeholders::_1));
  lowstate_sub_ = create_subscription<unitree_go::msg::LowState>(
    lowstate_topic, rclcpp::SensorDataQoS(),
    std::bind(&GoawayOdometry::lowStateCallback, this, std::placeholders::_1));
  locomotion_state_sub_ = create_subscription<std_msgs::msg::String>(
    locomotion_state_topic, rclcpp::SensorDataQoS(),
    std::bind(&GoawayOdometry::locomotionStateCallback, this, std::placeholders::_1));
  fastlio_pause_client_ = create_client<std_srvs::srv::SetBool>(fastlio_pause_service);

  odom_pub_ =create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  status_pub_ = create_publisher<std_msgs::msg::String>("~/status", 10);
  // Also notices every source going quiet, when no callback runs.
  status_timer_ = create_wall_timer(1s, std::bind(&GoawayOdometry::statusTimerCallback, this));
}

void GoawayOdometry::sportOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!finitePose(msg->pose.pose)) {return;}
  const auto & p = msg->pose.pose.position;
  const auto & o = msg->pose.pose.orientation;
  tf2::Quaternion q(o.x, o.y, o.z, o.w);
  q.normalize();

  const auto t = now();
  if (sport_last_.nanoseconds() == 0 || (t - sport_last_).seconds() > sport_timeout_) {
    sport_since_ = t;
  }
  sport_last_ = t;
  sport_raw_ = {p.x, p.y, heading(q, sport_raw_.yaw)};
  // Unitree's twist is in the body frame already.
  const auto & v = msg->twist.twist;
  const bool twist_ok =
    std::isfinite(v.linear.x) && std::isfinite(v.linear.y) && std::isfinite(v.angular.z);
  sport_vx_ = twist_ok ? v.linear.x : 0.0;
  sport_vy_ = twist_ok ? v.linear.y : 0.0;
  sport_wz_ = twist_ok ? v.angular.z : 0.0;

  selectSource();
  if (source_ == Source::kSport) {
    publishOdom(
      rclcpp::Time(msg->header.stamp, RCL_ROS_TIME), compose(sport_offset_, sport_raw_),
      sport_vx_, sport_vy_, sport_wz_);
  }
}

void GoawayOdometry::lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg)
{
  const auto & w = msg->imu_state.quaternion;   // w, x, y, z
  tf2::Quaternion body(w[1], w[2], w[3], w[0]);
  if (!std::isfinite(body.length2()) || body.length2() < 0.25) {return;}
  body.normalize();
  const auto t = now();
  const bool first = lowstate_last_.nanoseconds() == 0;
  double dt = first ? 0.0 : (t - lowstate_last_).seconds();
  if (dt < 0.0 || dt > lowstate_timeout_) {dt = 0.0;}   // after a gap there is nothing to add
  lowstate_last_ = t;

  // Tilt for base_link; heading changes turn the fused pose. The heading is absolute, so every
  // change counts, also without time to move in (same stamp, or after a gap).
  const double prev_heading = body_heading_;
  body_heading_ = heading(body, body_heading_);
  const double turn = first ? 0.0 : wrap(body_heading_ - prev_heading);
  tilt_ = yawQuat(body_heading_).inverse() * body;
  tilt_.normalize();
  updatePause(t, body);

  LegOdometry::Input in;
  for (int i = 0; i < 12; ++i) {
    const auto & m = msg->motor_state[i];
    in.q[i] = m.q;
    in.dq[i] = m.dq;
    in.tau[i] = m.tau_est;
  }
  const auto & g = msg->imu_state.gyroscope;
  in.gyro.setValue(g[0], g[1], g[2]);
  const auto leg = legs_.update(in);

  // base_link above the planted feet, level (works on slopes and steps, unlike the lowest foot)
  const double prev_height = height_;
  std::array<double, LegOdometry::kLegs> depth;   // each foot below base_link, level
  std::array<double, LegOdometry::kLegs> vz;      // body vertical speed each foot implies, level
  std::vector<double> planted_vz;
  for (int i = 0; i < LegOdometry::kLegs; ++i) {
    depth[i] = -tf2::quatRotate(tilt_, leg.feet[i]).z();
    vz[i] = tf2::quatRotate(tilt_, leg.foot_velocity[i]).z();
    if (leg.planted[i]) {planted_vz.push_back(vz[i]);}
  }
  // Feet on the floor: planted and moving up or down with the others. A swing foot can count as
  // planted while still coming down (calf torque, nothing agreeing), which lifts the floor a
  // little every step.
  std::array<bool, LegOdometry::kLegs> on_floor{};
  if (!planted_vz.empty()) {
    std::sort(planted_vz.begin(), planted_vz.end());
    const size_t m = planted_vz.size();
    const double median =
      m % 2 ? planted_vz[m / 2] : 0.5 * (planted_vz[m / 2 - 1] + planted_vz[m / 2]);
    for (int i = 0; i < LegOdometry::kLegs; ++i) {
      on_floor[i] = leg.planted[i] && std::abs(vz[i] - median) < floor_agreement_;
    }
  }
  double d = 0.0;
  int n = 0;
  for (int i = 0; i < LegOdometry::kLegs; ++i) {
    if (on_floor[i]) {
      d += depth[i];
      ++n;
    }
  }
  if (n > 0) {height_ = foot_radius_ + d / n;}

  if (dt > 0.0) {
    const double a = 1.0 - std::exp(-dt / kStillSmoothing);
    still_velocity_ += a * (leg.velocity - still_velocity_);
    turn_rate_ += a * (turn / dt - turn_rate_);
    gyro_rate_ += a * (in.gyro.length() - gyro_rate_);
    leg_forward_ += leg.velocity.x() * dt;
    leg_side_ += leg.velocity.y() * dt;
  }
  const bool quiet = std::hypot(still_velocity_.x(), still_velocity_.y()) < still_speed_ &&
    std::abs(turn_rate_) < still_turn_;
  if (!quiet) {
    still_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  } else if (still_since_.nanoseconds() == 0) {
    still_since_ = t;
  }
  still_ = quiet && (t - still_since_).seconds() >= still_time_;
  updateFastlioStart(t);

  selectSource();
  if (source_ == Source::kFused && paused_) {
    // Frozen through a skill, heading too
    publishOdom(t, fused_, 0.0, 0.0, 0.0);
  } else if (source_ == Source::kFused && dt > 0.0) {
    // Standing still means exactly still, like Unitree's own odom.
    const double vx = still_ ? 0.0 : scale_forward_ * leg.velocity.x();
    const double vy = still_ ? 0.0 : scale_side_ * leg.velocity.y();
    fused_.yaw = wrap(fused_.yaw + turn);
    const double c = std::cos(fused_.yaw), s = std::sin(fused_.yaw);
    fused_.x += (c * vx - s * vy) * dt;
    fused_.y += (s * vx + c * vy) * dt;
    updateFloor(on_floor, depth, prev_height, foot_z_, foot_anchored_, fused_.z);

    const double g = glide_time_ > 0.0 ? 1.0 - std::exp(-dt / glide_time_) : 1.0;
    const Planar step{-g * glide_.x, -g * glide_.y, -g * glide_.yaw, -g * glide_.z};
    glide_ = {glide_.x + step.x, glide_.y + step.y, glide_.yaw + step.yaw, glide_.z + step.z};
    const Planar shown{
      fused_.x + glide_.x, fused_.y + glide_.y, wrap(fused_.yaw + glide_.yaw), fused_.z + glide_.z};
    // The glide is motion too: add it to the velocity, in the body frame.
    const double gc = std::cos(shown.yaw), gs = std::sin(shown.yaw);
    const double gx = step.x / dt, gy = step.y / dt;
    publishOdom(
      t, shown, vx + gc * gx + gs * gy, vy - gs * gx + gc * gy,
      (still_ ? 0.0 : turn_rate_) + step.yaw / dt);
  } else if (source_ == Source::kFused) {
    fused_.yaw = wrap(fused_.yaw + turn);
  }
  // The estimate, not the shown pose, is what FAST_LIO is compared with.
  history_.push_back(
    {t, source_ == Source::kFused ? fused_ : output_, leg_forward_, leg_side_, height_,
      body_heading_});
  while ((t - history_.front().stamp).seconds() > kHistorySeconds) {history_.pop_front();}

  const auto & a = msg->imu_state.accelerometer;
  updateLocal(t, dt, body, tf2::Vector3(a[0], a[1], a[2]), leg, on_floor, depth, prev_height);

  if (odom_tf_period_ <= 0.0 || last_tilt_tf_.nanoseconds() == 0 ||
    (t - last_tilt_tf_).seconds() >= odom_tf_period_)
  {
    last_tilt_tf_ = t;
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = t;
    tf.header.frame_id = footprint_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.z = height_;
    tf.transform.rotation = tf2::toMsg(tilt_);
    tf_broadcaster_->sendTransform(tf);
  }
}

void GoawayOdometry::fastlioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!finitePose(msg->pose.pose)) {return;}
  if (!have_base_to_imu_ && !lookupBaseToImu()) {return;}
  const auto & p = msg->pose.pose.position;
  const auto & o = msg->pose.pose.orientation;
  tf2::Quaternion q(o.x, o.y, o.z, o.w);
  q.normalize();
  const tf2::Transform cam_body(q, tf2::Vector3(p.x, p.y, p.z));
  const rclcpp::Time stamp(msg->header.stamp, RCL_ROS_TIME);
  const auto t = now();

  // Paused, held until still, or from before FAST_LIO's fresh start
  const bool held = paused_ || fastlio_waiting_;
  if (held || stamp < fastlio_restart_) {
    // Still running while held: it missed the request or was respawned.
    if (held && (t - fastlio_pause_sent_).seconds() > 1.0) {requestFastlioPause(true);}
    return;
  }

  // A gap means FAST_LIO restarted (a new camera_init) or lost the LiDAR: settle, link again.
  if (fastlio_last_.nanoseconds() == 0 || (t - fastlio_last_).seconds() > fastlio_timeout_) {
    fastlio_since_ = t;
    fastlio_history_.clear();
    linked_ = false;
    have_prev_lio_ = false;
    last_jump_link_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    turn_check_from_ = stamp + rclcpp::Duration::from_seconds(fastlio_settle_);
  }
  fastlio_last_ = t;
  fastlio_history_.emplace_back(stamp, cam_body);
  while ((stamp - fastlio_history_.front().first).seconds() > kHistorySeconds) {
    fastlio_history_.pop_front();
  }

  // Keeps camera_init placed in odom whichever source drives.
  if (!linked_ && (t - fastlio_since_).seconds() >= fastlio_settle_) {linkFastlio();}
  selectSource();
  if (!linked_) {return;}

  // Compared with where we were at FAST_LIO's moment, which is a scan length or so ago.
  Planar lio = fastlioPlanar(cam_body);
  const History * h = historyAt(stamp);
  if (!h) {
    have_prev_lio_ = false;
    return;
  }
  const History now_h = *h;
  lio.z -= now_h.height;   // FAST_LIO places base_link, the floor is below it
  // Turning on its own: it started while the robot moved and took the motion as gyro bias.
  if (fastlioTurnsAlone(stamp, lio, now_h)) {
    fastlioLost("it turns without the body");
    return;
  }
  double dt = 0.0;
  if (have_prev_lio_) {
    dt = (stamp - prev_lio_stamp_).seconds();
    // A jump: FAST_LIO moved far more than we did. Ignore it; a run of them means it lost
    // track, so place its frame again from our pose.
    const double ex = (lio.x - prev_lio_.x) - (now_h.pose.x - prev_lio_history_.pose.x);
    const double ey = (lio.y - prev_lio_.y) - (now_h.pose.y - prev_lio_history_.pose.y);
    if (std::hypot(ex, ey) > max_jump_) {
      if (++jumps_ >= 3) {
        // Again soon after the last relink: not a slip but lost.
        if (last_jump_link_.nanoseconds() != 0 &&
          (t - last_jump_link_).seconds() < fastlio_lost_time_)
        {
          fastlioLost("it keeps jumping");
          return;
        }
        RCLCPP_WARN(get_logger(), "FAST_LIO keeps jumping, linking it again");
        linkFastlio();
        last_jump_link_ = t;
      }
      return;
    }
    jumps_ = 0;
  }
  // A fresh FAST_LIO steers odom only once the turn check has seen it over a whole window.
  // Legs only: it never does, it is only kept linked and checked for its map.
  const bool checked = (stamp - turn_check_from_).seconds() >= 0.75 * kTurnWindow;
  if (!legs_only_ && checked && dt > 0.0) {
    learnScale(prev_lio_history_, now_h, prev_lio_, lio);
    if (source_ == Source::kFused) {correctFused(stamp, lio, dt);}
  }
  prev_lio_ = lio;
  prev_lio_stamp_ = stamp;
  prev_lio_history_ = now_h;
  have_prev_lio_ = true;
}

void GoawayOdometry::updateFloor(
  const std::array<bool, LegOdometry::kLegs> & planted,
  const std::array<double, LegOdometry::kLegs> & depth, double prev_height,
  std::array<double, LegOdometry::kLegs> & foot_z,
  std::array<bool, LegOdometry::kLegs> & anchored, double & floor_z)
{
  // Each planted foot keeps the height it was planted at, so the floor only moves when a foot
  // plants higher or lower (steps, slopes). Positions only: the joint speeds spike at touchdown.
  double sum = 0.0;
  int n = 0;
  for (int i = 0; i < LegOdometry::kLegs; ++i) {
    if (planted[i] && anchored[i]) {
      sum += foot_z[i] + depth[i];
      ++n;
    }
  }
  // base_link's height from the feet that stay planted, else where it was
  const double base = n > 0 ? sum / n : floor_z + prev_height;
  double floor = 0.0;
  int m = 0;
  for (int i = 0; i < LegOdometry::kLegs; ++i) {
    if (!planted[i]) {
      anchored[i] = false;
      continue;
    }
    if (!anchored[i]) {
      foot_z[i] = base - depth[i];
      anchored[i] = true;
    }
    floor += foot_z[i];
    ++m;
  }
  if (m > 0) {floor_z = floor / m - foot_radius_;}
}

void GoawayOdometry::updateLocal(
  const rclcpp::Time & t, double dt, const tf2::Quaternion & body, const tf2::Vector3 & accel,
  const LegOdometry::Output & leg, const std::array<bool, LegOdometry::kLegs> & on_floor,
  const std::array<double, LegOdometry::kLegs> & depth, double prev_height)
{
  if (dt <= 0.0) {return;}
  updateFloor(on_floor, depth, prev_height, local_foot_z_, local_anchored_, local_floor_);
  const double legs_z = local_floor_ + height_;
  if (!local_.ok) {
    local_.z = local_.legs_z = legs_z;
    local_.vz = local_.legs_vz = 0.0;
    local_.ok = true;
  }

  // xy: the legs, turned with the body IMU (slopes included)
  if (!still_) {
    const tf2::Vector3 v = tf2::quatRotate(
      body, tf2::Vector3(
        scale_forward_ * leg.velocity.x(), scale_side_ * leg.velocity.y(), leg.velocity.z()));
    local_.x += v.x() * dt;
    local_.y += v.y() * dt;
  }
  // z: vertical acceleration integrated, its speed pulled to the legs' (they jump as feet
  // re-plant), its height slowly to theirs
  const double az = tf2::quatRotate(body, accel).z();
  if (still_) {local_.gravity += (az - local_.gravity) * (1.0 - std::exp(-dt / kLocalGravityTau));}
  local_.legs_vz += ((legs_z - local_.legs_z) / dt - local_.legs_vz) *
    (1.0 - std::exp(-dt / kLocalLegsSmoothing));
  local_.legs_z = legs_z;
  local_.vz += (az - local_.gravity) * dt + (local_.legs_vz - local_.vz) * dt / kLocalSpeedTau;
  local_.z += local_.vz * dt + (legs_z - local_.z) * dt / kLocalHeightTau;

  if (local_tf_period_ > 0.0 && last_local_tf_.nanoseconds() != 0 &&
    (t - last_local_tf_).seconds() < local_tf_period_)
  {
    return;
  }
  last_local_tf_ = t;
  geometry_msgs::msg::TransformStamped base;
  base.header.stamp = t;
  base.header.frame_id = local_frame_;
  base.child_frame_id = local_base_frame_;
  base.transform.translation.x = local_.x;
  base.transform.translation.y = local_.y;
  base.transform.translation.z = local_.z;
  base.transform.rotation = tf2::toMsg(body);
  geometry_msgs::msg::TransformStamped footprint = base;
  footprint.child_frame_id = local_footprint_frame_;
  footprint.transform.translation.z = local_.z - height_;
  footprint.transform.rotation = tf2::toMsg(yawQuat(body_heading_));
  tf_broadcaster_->sendTransform(
    std::vector<geometry_msgs::msg::TransformStamped>{base, footprint});
}

void GoawayOdometry::correctFused(const rclcpp::Time & stamp, const Planar & lio, double dt)
{
  const History * h = historyAt(stamp);
  if (!h) {return;}
  const double ex = lio.x - h->pose.x;
  const double ey = lio.y - h->pose.y;
  const double eyaw = wrap(lio.yaw - h->pose.yaw);
  const double ez = lio.z - h->pose.z;

  // Legs still but FAST_LIO moving steadily: carried or sliding, so the legs are wrong.
  carried_ = false;
  if (still_) {
    if (!hold_anchored_) {
      hold_anchor_ = lio;
      hold_anchored_ = true;
    }
    carried_ = std::hypot(lio.x - hold_anchor_.x, lio.y - hold_anchor_.y) > carried_distance_ ||
      fastlioSpeed(stamp) > carried_speed_;
    if (carried_) {hold_anchor_ = lio;}
  } else {
    hold_anchored_ = false;
  }
  // Standing: FAST_LIO's wander is noise, so hold still.
  if (still_ && !carried_) {return;}

  const double tau =
    carried_ || std::hypot(ex, ey) > catch_up_distance_ ? fastlio_fast_tau_ : fastlio_tau_;
  const double dx = (1.0 - std::exp(-dt / tau)) * ex;
  const double dy = (1.0 - std::exp(-dt / tau)) * ey;
  const double dyaw = (1.0 - std::exp(-dt / fastlio_yaw_tau_)) * eyaw;
  const double dz = (1.0 - std::exp(-dt / tau)) * ez;
  fused_.x += dx;
  fused_.y += dy;
  fused_.yaw = wrap(fused_.yaw + dyaw);
  fused_.z += dz;
  for (auto & z : foot_z_) {z += dz;}
  // The shown pose doesn't jump: the correction glides in from here.
  glide_ = {glide_.x - dx, glide_.y - dy, glide_.yaw - dyaw, glide_.z - dz};
  // Later history moves along, or the next comparison would count this correction again.
  for (auto & e : history_) {
    if (e.stamp > stamp) {
      e.pose.x += dx;
      e.pose.y += dy;
      e.pose.yaw = wrap(e.pose.yaw + dyaw);
      e.pose.z += dz;
    }
  }
}

void GoawayOdometry::learnScale(
  const History & from, const History & to, const Planar & lio_from, const Planar & lio_to)
{
  // Only while walking on the legs: still gives nothing to compare, carried is not walking.
  if (still_ || carried_) {return;}
  const double leg_f = to.leg_forward - from.leg_forward;
  const double leg_s = to.leg_side - from.leg_side;
  if (std::hypot(leg_f, leg_s) < kMinLearnStep) {return;}
  // FAST_LIO's move in the body frame
  const double dx = lio_to.x - lio_from.x, dy = lio_to.y - lio_from.y;
  const double c = std::cos(to.pose.yaw), s = std::sin(to.pose.yaw);
  const double lio_f = c * dx + s * dy;
  const double lio_s = -s * dx + c * dy;
  // Least squares FAST_LIO = scale x legs per axis, forgetting over scale_learn_time.
  const double a = 1.0 - std::exp(-(to.stamp - from.stamp).seconds() / scale_learn_time_);
  scale_fwd_xy_ += a * (leg_f * lio_f - scale_fwd_xy_);
  scale_fwd_xx_ += a * (leg_f * leg_f - scale_fwd_xx_);
  scale_side_xy_ += a * (leg_s * lio_s - scale_side_xy_);
  scale_side_xx_ += a * (leg_s * leg_s - scale_side_xx_);
  scale_forward_ = std::clamp(scale_fwd_xy_ / scale_fwd_xx_, scale_min_, scale_max_);
  scale_side_ = std::clamp(scale_side_xy_ / scale_side_xx_, scale_min_, scale_max_);
}

void GoawayOdometry::locomotionStateCallback(const std_msgs::msg::String::SharedPtr msg)
{
  // {"state": "Velocity", ...} from goaway_locomotion
  static const std::regex kState("\"state\"\\s*:\\s*\"([^\"]*)\"");
  std::smatch m;
  if (!std::regex_search(msg->data, m, kState)) {return;}
  locomotion_state_ = m[1];
  locomotion_state_last_ = now();
}

void GoawayOdometry::updatePause(const rclcpp::Time & t, const tf2::Quaternion & body)
{
  // Without the state topic (goaway_locomotion not running) only falls count.
  const bool state_fresh = locomotion_state_last_.nanoseconds() != 0 &&
    (t - locomotion_state_last_).seconds() < kLocomotionStateTimeout;
  const bool skill = state_fresh &&
    std::find(calm_states_.begin(), calm_states_.end(), locomotion_state_) == calm_states_.end();
  const bool tipped =
    tf2::quatRotate(body, tf2::Vector3(0.0, 0.0, 1.0)).z() < std::cos(pause_tilt_);
  const rclcpp::Time never(0, 0, RCL_ROS_TIME);

  if (!paused_) {
    if (!skill && !tipped) {return;}
    paused_ = true;
    ++pauses_;
    pause_start_ = t;
    calm_since_ = never;
    // Frozen where it was last shown, the glide folded in.
    if (source_ == Source::kFused && have_output_) {fused_ = output_;}
    glide_ = Planar{};
    // The FAST_LIO after the pause is a new one: nothing of this one is kept.
    holdFastlio();
    RCLCPP_INFO(
      get_logger(), "odom paused (%s)", skill ? locomotion_state_.c_str() : "tilted");
    publishStatus();
    return;
  }

  if (skill || tipped) {
    calm_since_ = never;
    return;
  }
  if (calm_since_.nanoseconds() == 0) {calm_since_ = t;}
  if ((t - calm_since_).seconds() < resume_time_) {return;}

  paused_ = false;
  foot_anchored_.fill(false);   // the feet moved during the skill
  local_anchored_.fill(false);
  local_.ok = false;            // a skill leaves the vertical speed wrong
  // The legs carry on from the frozen pose; FAST_LIO waits for the robot to be still.
  RCLCPP_INFO(
    get_logger(), "odom resumed after %.1f s, FAST_LIO starts at the next stop",
    (t - pause_start_).seconds());
  publishStatus();
}

void GoawayOdometry::holdFastlio()
{
  linked_ = false;
  fastlio_history_.clear();
  have_prev_lio_ = false;
  hold_anchored_ = false;
  carried_ = false;
  jumps_ = 0;
  fastlio_waiting_ = true;
  requestFastlioPause(true);
}

void GoawayOdometry::updateFastlioStart(const rclcpp::Time & t)
{
  // Legs still, and the body not turning on any axis for as long
  if (gyro_rate_ >= still_turn_) {
    gyro_quiet_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  } else if (gyro_quiet_since_.nanoseconds() == 0) {
    gyro_quiet_since_ = t;
  }
  const bool still = still_ && gyro_quiet_since_.nanoseconds() != 0 &&
    (t - gyro_quiet_since_).seconds() >= still_time_;
  if (paused_) {return;}
  if (fastlio_waiting_) {
    if (!still) {return;}
    // Fresh: settles, then links again from where the legs got to.
    fastlio_waiting_ = false;
    fastlio_restart_ = t;
    fastlio_last_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    requestFastlioPause(false);
    RCLCPP_INFO(get_logger(), "robot still: FAST_LIO restarting");
    publishStatus();
    return;
  }
  // Moved while it calibrated: its start can't be trusted.
  if (!still && fastlio_restart_.nanoseconds() != 0 &&
    (t - fastlio_restart_).seconds() < fastlio_still_time_)
  {
    holdFastlio();
    RCLCPP_INFO(get_logger(), "moved while FAST_LIO started: again at the next stop");
    publishStatus();
  }
}

void GoawayOdometry::fastlioLost(const char * why)
{
  ++lost_;
  holdFastlio();
  RCLCPP_WARN(
    get_logger(), "FAST_LIO lost track (%s): odom on the legs, FAST_LIO restarts at the next stop",
    why);
  publishStatus();
}

void GoawayOdometry::requestFastlioPause(bool pause)
{
  if (pause) {fastlio_pause_sent_ = now();}
  if (!fastlio_pause_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "FAST_LIO pause service %s not available",
      fastlio_pause_client_->get_service_name());
    return;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = pause;
  fastlio_pause_client_->async_send_request(
    request, [this, pause](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      if (!future.get()->success) {
        RCLCPP_WARN(
          get_logger(), "FAST_LIO refused to %s: %s", pause ? "pause" : "restart",
          future.get()->message.c_str());
      }
    });
}

void GoawayOdometry::statusTimerCallback()
{
  selectSource();
  publishStatus();
}

void GoawayOdometry::selectSource()
{
  const auto t = now();
  const bool sport = !unitree_own_ && sportOk(t);
  const bool legs =
    lowstate_last_.nanoseconds() != 0 && (t - lowstate_last_).seconds() < lowstate_timeout_;
  const Source want = sport ? Source::kSport : (legs ? Source::kFused : Source::kNone);
  if (want == source_) {return;}

  // The new source continues from the last pose sent, so odom doesn't jump.
  if (want == Source::kSport) {
    sport_offset_ = have_output_ ? compose(output_, inverse(sport_raw_)) : Planar{};
  } else if (want == Source::kFused) {
    fused_ = have_output_ ? output_ : Planar{};
    glide_ = Planar{};
    foot_anchored_.fill(false);
    hold_anchored_ = false;
    carried_ = false;
    // FAST_LIO continues from here, not from wherever the last source drifted to.
    if (linked_) {linkFastlio();}
  }
  RCLCPP_INFO(
    get_logger(), "odom source: %s -> %s at x %.2f y %.2f yaw %.1f deg%s",
    sourceName(source_), sourceName(want), output_.x, output_.y, output_.yaw * 180.0 / M_PI,
    want == Source::kNone ? " (holding the last pose)" : "");
  source_ = want;
  publishStatus();
}

void GoawayOdometry::linkFastlio()
{
  if (fastlio_history_.empty()) {return;}
  // Where FAST_LIO's frame sits in odom so that its latest pose matches ours at that moment:
  // odom -> camera_init = odom -> base_link * base_link -> imu * (camera_init -> body)^-1
  const auto & latest = fastlio_history_.back();
  const History * h = historyAt(latest.first);
  odom_to_camera_init_ = baseInOdom(h ? h->pose : output_, h ? h->height : height_) *
    base_to_imu_ * latest.second.inverse();
  linked_ = true;
  have_prev_lio_ = false;   // nothing to compare across a relink
  hold_anchored_ = false;
  jumps_ = 0;
  ++links_;

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now();
  t.header.frame_id = odom_frame_;
  t.child_frame_id = fastlio_world_frame_;
  t.transform = tf2::toMsg(odom_to_camera_init_);
  static_broadcaster_->sendTransform(t);
  RCLCPP_INFO(
    get_logger(), "linked %s to %s at x %.2f y %.2f yaw %.1f deg", fastlio_world_frame_.c_str(),
    odom_frame_.c_str(), output_.x, output_.y, output_.yaw * 180.0 / M_PI);
}

void GoawayOdometry::publishOdom(
  const rclcpp::Time & stamp, const Planar & pose, double vx, double vy, double wz)
{
  output_ = pose;
  have_output_ = true;

  // Throttled on the message stamps; a stamp going backwards resets it.
  if (odom_tf_period_ > 0.0 && last_tf_stamp_.nanoseconds() != 0) {
    const double dt = (stamp - last_tf_stamp_).seconds();
    if (dt >= 0.0 && dt < odom_tf_period_) {return;}
  }
  last_tf_stamp_ = stamp;

  const auto q = tf2::toMsg(yawQuat(pose.yaw));
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = odom_frame_;
  t.child_frame_id = footprint_frame_;
  t.transform.translation.x = pose.x;
  t.transform.translation.y = pose.y;
  t.transform.translation.z = pose.z;
  t.transform.rotation = q;
  tf_broadcaster_->sendTransform(t);

  nav_msgs::msg::Odometry odom;
  odom.header = t.header;
  odom.child_frame_id = footprint_frame_;
  odom.pose.pose.position.x = pose.x;
  odom.pose.pose.position.y = pose.y;
  odom.pose.pose.position.z = pose.z;
  odom.pose.pose.orientation = q;
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = wz;
  odom_pub_->publish(odom);
}

void GoawayOdometry::publishStatus()
{
  const auto t = now();
  const char * fastlio = "off";
  if (fastlio_waiting_ && !paused_) {
    fastlio = "waiting";   // for the robot to be still
  } else if (fastlio_last_.nanoseconds() != 0 &&
    (t - fastlio_last_).seconds() < fastlio_timeout_)
  {
    fastlio = linked_ ? "ready" : "settling";
  }
  char buf[256];
  std::snprintf(
    buf, sizeof(buf),
    "{\"source\": \"%s\", \"fastlio\": \"%s\", \"links\": %d, \"still\": %s, "
    "\"scale\": [%.3f, %.3f], \"paused\": %s, \"pauses\": %d, \"lost\": %d}",
    sourceName(source_), fastlio, links_, still_ ? "true" : "false", scale_forward_,
    scale_side_, paused_ ? "true" : "false", pauses_, lost_);
  std_msgs::msg::String msg;
  msg.data = buf;
  status_pub_->publish(msg);
}

bool GoawayOdometry::sportOk(const rclcpp::Time & now) const
{
  return sport_last_.nanoseconds() != 0 && (now - sport_last_).seconds() < sport_timeout_ &&
         (now - sport_since_).seconds() >= sport_settle_;
}

bool GoawayOdometry::lookupBaseToImu()
{
  try {
    const auto t = tf_buffer_->lookupTransform(base_frame_, imu_frame_, tf2::TimePointZero);
    tf2::fromMsg(t.transform, base_to_imu_);
    have_base_to_imu_ = true;
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "FAST_LIO waits for %s -> %s from the URDF: %s",
      base_frame_.c_str(), imu_frame_.c_str(), e.what());
  }
  return have_base_to_imu_;
}

const GoawayOdometry::History * GoawayOdometry::historyAt(const rclcpp::Time & stamp) const
{
  if (history_.empty() || (history_.front().stamp - stamp).seconds() > 0.05) {return nullptr;}
  auto it = std::lower_bound(
    history_.begin(), history_.end(), stamp,
    [](const History & e, const rclcpp::Time & s) {return e.stamp < s;});
  if (it == history_.end()) {return &history_.back();}
  if (it != history_.begin() && (stamp - std::prev(it)->stamp) < (it->stamp - stamp)) {--it;}
  return &*it;
}

double GoawayOdometry::fastlioSpeed(const rclcpp::Time & stamp) const
{
  // From the oldest pose within the window to the newest
  for (const auto & e : fastlio_history_) {
    const double dt = (stamp - e.first).seconds();
    if (dt <= kSpeedWindow) {
      if (dt < 0.5 * kSpeedWindow) {return 0.0;}
      const Planar a = fastlioPlanar(e.second);
      const Planar b = fastlioPlanar(fastlio_history_.back().second);
      return std::hypot(b.x - a.x, b.y - a.y) / dt;
    }
  }
  return 0.0;
}

bool GoawayOdometry::fastlioTurnsAlone(
  const rclcpp::Time & stamp, const Planar & lio, const History & now_h) const
{
  // Its turn and the body IMU's over the window, from the oldest settled pose within it
  for (const auto & e : fastlio_history_) {
    if (e.first < turn_check_from_ || (stamp - e.first).seconds() > kTurnWindow) {continue;}
    if ((stamp - e.first).seconds() < 0.75 * kTurnWindow) {return false;}
    const History * h = historyAt(e.first);
    if (!h) {return false;}
    const double lio_turn = wrap(lio.yaw - fastlioPlanar(e.second).yaw);
    const double body_turn = wrap(now_h.heading - h->heading);
    return std::abs(wrap(lio_turn - body_turn)) > fastlio_lost_turn_;
  }
  return false;
}

tf2::Transform GoawayOdometry::baseInOdom(const Planar & pose, double height) const
{
  return tf2::Transform(
    yawQuat(pose.yaw) * tilt_, tf2::Vector3(pose.x, pose.y, pose.z + height));
}

// z is base_link's height here, not the floor's
Planar GoawayOdometry::fastlioPlanar(const tf2::Transform & cam_body) const
{
  const tf2::Transform base = odom_to_camera_init_ * cam_body * base_to_imu_.inverse();
  return {
    base.getOrigin().x(), base.getOrigin().y(), heading(base.getRotation(), output_.yaw),
    base.getOrigin().z()};
}

const char * GoawayOdometry::sourceName(Source source) const
{
  switch (source) {
    case Source::kSport: return "sport";
    case Source::kFused: return legs_only_ ? "legs" : "fused";
    default: return "none";
  }
}

}  // namespace goaway_odometry
