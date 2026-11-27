#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/assisted_teleop.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

#include "htxgrrt_msgs/msg/navigation_plan.hpp"
#include "htxgrrt_msgs/msg/quadruped_main_status.hpp"
#include "htxgrrt_msgs/srv/string_trigger.hpp"
#include "goaway_quadruped_controller/api_client.hpp"

namespace goaway_quadruped_controller
{

// The Go2's ROS front end (after tmms's quadruped_controller):
//   - cmd_vel mux: UI (direct, or checked by nav2's assisted_teleop) while paused, nav while
//     unpaused -> /consolidated_quadruped_cmd_vel (goaway_locomotion_node reads it) and, in
//     Unitree mode, sport Move
//   - holds an assisted_teleop goal while paused (the UI's safety teleop), ~/safety_ready
//   - ~/quadruped_pause: unpause (nav drives) only while walking (HTX Velocity or Unitree
//     balance stand), never upright or mapping; auto-pause when the robot stops walking
//   - ~/quadruped_cmd: stand/walk/damp... for whichever locomotion is active, kicks (HTX),
//     use_htx / use_unitree to switch locomotion through Unitree's motion switcher
//   - quadruped_main_status (incl. goaway_yasmin / localization_manager mirrors), joint_states
class QuadrupedController : public rclcpp::Node
{
public:
  explicit QuadrupedController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~QuadrupedController() override;

private:
  using Status = htxgrrt_msgs::msg::QuadrupedMainStatus;
  using StringTrigger = htxgrrt_msgs::srv::StringTrigger;

  // Inputs
  void cmdVelUiCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void cmdVelNavCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void cmdVelTeleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg);
  void sportStateCallback(const unitree_go::msg::SportModeState::SharedPtr msg);
  void htxStateCallback(const std_msgs::msg::String::SharedPtr msg);
  // Navigation mirrors, as tmms
  void yasminStateCallback(const std_msgs::msg::String::SharedPtr msg);
  void localizationStatusCallback(const std_msgs::msg::String::SharedPtr msg);
  void currentMapCallback(const std_msgs::msg::String::SharedPtr msg);
  void currNavplanCallback(const htxgrrt_msgs::msg::NavigationPlan::SharedPtr msg);
  void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void mappingStatusCallback(const std_msgs::msg::String::SharedPtr msg);

  // Timers
  void moveTimerCallback();
  void mainStatusTimerCallback();
  void modePollTimerCallback();
  // Keeps an assisted_teleop goal running exactly while paused.
  void assistedTeleopTimerCallback();
  void publishSafetyReady();

  // Services
  void quadrupedCmdCallback(
    const StringTrigger::Request::SharedPtr req, StringTrigger::Response::SharedPtr res);
  void quadrupedPauseCallback(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  bool executeCmd(const std::string & cmd, std::string & message);
  bool sportCmd(int32_t api_id, const std::string & what, std::string & message);
  // These three: 0 ok, -1 no answer.
  int32_t htxSetState(const std::string & state, std::string & message);
  int32_t htxAction(const std::string & action, std::string & message);
  int32_t htxCall(
    int32_t api_id, const std::string & parameter, const std::string & what, std::string & message);

  // Locomotion switching: checks run in the service call, the switch in switch_thread_.
  bool startSwitch(const std::string & target, std::string & message);
  void switchToHtx();
  void switchToUnitree();
  void endSwitch(const std::string & policy);   // empty: policy unchanged
  bool checkMode(std::string & name);   // false: the motion switcher didn't answer
  bool waitMode(bool released, double timeout_s, std::string & name);
  bool htxDampingWithRetry(double timeout_s, std::string & message);

  std::string robotMode();
  // Why nav may not drive now ("" = it may): unpause refuses with it, auto-pause acts on it.
  // Caller holds mutex_; mode is robotMode(), which takes it itself.
  std::string notDrivableReasonLocked(const std::string & mode) const;
  void autoPause(const std::string & reason);
  void staleAllInputsLocked();
  void publishStop();

  std::string unitree_mode_;
  std::unique_ptr<ApiClient> sport_;
  std::unique_ptr<ApiClient> switcher_;
  std::unique_ptr<ApiClient> locomotion_;

  // All protected by mutex_
  std::mutex mutex_;
  geometry_msgs::msg::Twist ui_twist_;
  geometry_msgs::msg::Twist nav_twist_;
  geometry_msgs::msg::Twist teleop_twist_;   // the UI's safety teleop after assisted_teleop
  rclcpp::Time last_ui_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_nav_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_teleop_time_{0, 0, RCL_ROS_TIME};
  bool prev_teleop_zero_{true};
  bool paused_{true};   // starts paused: the UI drives, unpause hands the robot to nav
  rclcpp::Time not_drivable_since_{0, 0, RCL_ROS_TIME};   // unpaused, nav can't drive
  bool not_drivable_{false};
  // Mirrored for quadruped_main_status (tmms). navigation_state = paused ? PAUSED : this.
  std::string yasmin_state_{Status::UNLOCALIZED};
  std::string localization_status_{Status::NOT_STARTED};
  std::string current_map_;
  int64_t current_navplan_id_{0};
  std::string current_navplan_status_;
  geometry_msgs::msg::Pose pose_;   // /amcl_pose (map frame)
  bool mapping_{false};             // a mapping_manager session is running
  bool input_was_fresh_{false};
  uint32_t sport_code_{0};   // SportModeState.error_code (mcf state)
  rclcpp::Time last_sport_time_{0, 0, RCL_ROS_TIME};
  uint8_t battery_soc_{0};
  std::string htx_state_;
  std::string htx_upright_;   // what UprightWalk is doing (its state topic's "upright")
  bool htx_scan_lost_ = false;   // the walk is on flat ground (its state topic's "height_scan")
  rclcpp::Time last_htx_time_{0, 0, RCL_ROS_TIME};
  std::string policy_;   // Status::UNITREE / Status::HTX; empty until the motion switcher answers

  std::atomic<bool> switching_{false};
  std::atomic<int> switch_generation_{0};   // bumped when a switch ends
  std::thread switch_thread_;

  // assisted_teleop goal: only touched in default-group callbacks (its timer and the action
  // client's), which never overlap. No lock: rclcpp_action runs feedback under its own lock.
  using AssistedTeleop = nav2_msgs::action::AssistedTeleop;
  enum class TeleopGoal { kNone, kPending, kActive, kCanceling };
  rclcpp_action::Client<AssistedTeleop>::SharedPtr assisted_teleop_client_;
  rclcpp_action::ClientGoalHandle<AssistedTeleop>::SharedPtr assisted_teleop_goal_;
  TeleopGoal assisted_teleop_state_{TeleopGoal::kNone};
  rclcpp::Time assisted_teleop_seen_{0, 0, RCL_ROS_TIME};
  uint64_t assisted_teleop_seq_{0};   // a reply for an older goal is ignored
  bool safety_ready_{false};          // last published ~/safety_ready
  rclcpp::Time safety_ready_time_{0, 0, RCL_ROS_TIME};

  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr poll_group_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ui_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr htx_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yasmin_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr current_map_sub_;
  rclcpp::Subscription<htxgrrt_msgs::msg::NavigationPlan>::SharedPtr curr_navplan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mapping_status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr consolidated_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<Status>::SharedPtr main_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_ready_pub_;
  rclcpp::Service<StringTrigger>::SharedPtr quadruped_cmd_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr quadruped_pause_srv_;
  rclcpp::TimerBase::SharedPtr move_timer_;
  rclcpp::TimerBase::SharedPtr main_status_timer_;
  rclcpp::TimerBase::SharedPtr mode_poll_timer_;
  rclcpp::TimerBase::SharedPtr assisted_teleop_timer_;

  // LowState motor order: FR, FL, RR, RL legs, each hip, thigh, calf
  static constexpr std::size_t kNumJoints = 12;
  static const std::array<std::string, kNumJoints> kJointNames;
};

}  // namespace goaway_quadruped_controller
