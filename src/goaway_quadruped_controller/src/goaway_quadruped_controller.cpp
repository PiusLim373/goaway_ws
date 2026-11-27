#include "goaway_quadruped_controller/quadruped_controller.hpp"

#include <chrono>

#include "nlohmann/json.hpp"

using namespace std::chrono_literals;

namespace goaway_quadruped_controller
{

namespace
{
using Status = goaway_msgs::msg::QuadrupedMainStatus;

// Unitree sport service (Go2)
constexpr int32_t kSportDamp = 1001;
constexpr int32_t kSportBalanceStand = 1002;
constexpr int32_t kSportStandUp = 1004;
constexpr int32_t kSportStandDown = 1005;
constexpr int32_t kSportMove = 1008;
// Unitree motion switcher
constexpr int32_t kSwitcherCheckMode = 1001;
constexpr int32_t kSwitcherSelectMode = 1002;
constexpr int32_t kSwitcherReleaseMode = 1003;
// goaway_locomotion_node (LocomotionServer; codes from its FSMRequest)
constexpr int32_t kHtxSetState = 1002;
constexpr int32_t kHtxAction = 1004;
constexpr int32_t kHtxRefused = 4102;
constexpr int32_t kNoAnswer = -1;
// SportModeState.error_code (mcf) states that count as lying down
constexpr uint32_t kSportCodeDamping = 1001;
constexpr uint32_t kSportCodeLieDown = 2006;

constexpr double kUiTimeout = 1.0;        // s, as tmms
constexpr double kNavTimeout = 0.3;       // s, as tmms
// assisted_teleop publishes every cycle (10 Hz); its repeated zeros are dropped, so it goes
// stale like nav's stream.
constexpr double kTeleopTimeout = 0.3;    // s, as tmms
constexpr double kTeleopGoalSilence = 3.0;   // s without feedback: the goal is gone
constexpr double kSportStateTimeout = 1.0;
constexpr double kHtxStateTimeout = 3.0;  // its state topic has a 1 s heartbeat
// Unpaused but not drivable for this long: auto-pause (a stale or in-between state is shorter)
constexpr double kAutoPauseDelay = 0.5;   // s

// The mcf firmware leaves SportModeState.mode at 0 and reports its state in error_code.
// Undocumented (not in unitree_sdk2), measured on the robot. State and API numbers
// differ: the BalanceStand API (1002) gives state 100, state 1002 comes from StandUp (1004).
std::string sportRobotMode(uint32_t code)
{
  switch (code) {
    case kSportCodeDamping: return Status::DAMPING;
    case kSportCodeLieDown: return Status::LIE_DOWN;   // brief, then damping
    case 1002: return Status::JOINT_LOCK;
    case 100: return Status::BALANCE_STAND;            // also while walking
    case 1091: return Status::POSE;                    // not used by the UI yet
    default: return "";
  }
}

// Same meaning as Unitree's modes, so the UI reads both locomotions alike:
// joint_lock = standing, balance_stand = ready to walk.
std::string htxRobotMode(const std::string & state, const std::string & upright)
{
  if (state == "UprightWalk") {
    if (upright == "rising") {return Status::UPRIGHT_RISING;}
    if (upright == "paw_left") {return Status::SHAKE_HAND_LEFT;}
    if (upright == "paw_right") {return Status::SHAKE_HAND_RIGHT;}
    if (upright == "wave_left") {return Status::WAVE_LEFT;}
    if (upright == "wave_right") {return Status::WAVE_RIGHT;}
    if (upright == "arm_back") {return Status::UPRIGHT_ARM_BACK;}
    if (upright == "tipped") {return Status::UPRIGHT_TIPPED;}
    if (upright == "coming_down") {return Status::UPRIGHT_COMING_DOWN;}
    return Status::UPRIGHT_WALK;
  }
  if (state == "Off") {return Status::OFF;}
  if (state == "Damping") {return Status::DAMPING;}
  if (state == "JointLock") {return Status::JOINT_LOCK;}
  if (state == "Velocity") {return Status::BALANCE_STAND;}
  if (state == "LieDown") {return Status::LIE_DOWN;}
  if (state == "Recovery") {return Status::RECOVERY_STAND;}
  if (state == "DonkeyKick") {return Status::DONKEY_KICK;}
  if (state == "QuickBuck") {return Status::QUICK_BUCK;}
  if (state == "SideFlipLeft") {return Status::SIDE_FLIP_LEFT;}
  if (state == "SideFlipRight") {return Status::SIDE_FLIP_RIGHT;}
  if (state == "FrontFlip") {return Status::FRONT_FLIP;}
  if (state == "BackFlip") {return Status::BACK_FLIP;}
  return "";
}

// ~/quadruped_cmd: the same command for whichever locomotion is active.
struct Command
{
  const char * name;
  int32_t sport_api;        // 0: HTX only
  const char * sport_what;
  const char * htx_state;
};
constexpr Command kCommands[] = {
  {"damp", kSportDamp, "Damp", "Damping"},
  {"stand_up", kSportStandUp, "Stand up (joint lock)", "JointLock"},
  {"stand_down", kSportStandDown, "Stand down", "LieDown"},
  {"balance_stand", kSportBalanceStand, "Balance stand", "Velocity"},
  {"recovery_stand", 0, "", "Recovery"},
  {"donkey_kick", 0, "", "DonkeyKick"},
  {"quick_buck", 0, "", "QuickBuck"},
  {"side_flip_left", 0, "", "SideFlipLeft"},
  {"side_flip_right", 0, "", "SideFlipRight"},
  {"front_flip", 0, "", "FrontFlip"},
  {"back_flip", 0, "", "BackFlip"},
  {"upright_walk", 0, "", "UprightWalk"},
};

// ~/quadruped_cmd inside UprightWalk (HTX only): the command, then the HTX action.
struct HtxAction
{
  const char * name;
  const char * action;
};
constexpr HtxAction kHtxActions[] = {
  {"come_down", "come_down"},
  {"shake_hand_left", "paw_left"},
  {"shake_hand_right", "paw_right"},
  {"wave_left", "wave_left"},
  {"wave_right", "wave_right"},
};
}  // namespace

const std::array<std::string, QuadrupedController::kNumJoints> QuadrupedController::kJointNames = {
  "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
  "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
  "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
  "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
};

QuadrupedController::QuadrupedController(const rclcpp::NodeOptions & options)
: Node("goaway_quadruped_controller", options)
{
  // Unitree's own locomotion, selected when switching back from HTX.
  unitree_mode_ = declare_parameter<std::string>("unitree_mode", "mcf");
  const auto lowstate_topic = declare_parameter<std::string>("lowstate_topic", "/lf/lowstate");
  const auto sportstate_topic =
    declare_parameter<std::string>("sportstate_topic", "/lf/sportmodestate");

  sport_ = std::make_unique<ApiClient>(this, "sport");
  switcher_ = std::make_unique<ApiClient>(this, "motion_switcher");
  locomotion_ = std::make_unique<ApiClient>(this, "goaway_locomotion");

  // Services and the mode poll wait for API replies: own groups, so the cmd_vel mux and
  // the status keep running meanwhile.
  service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  poll_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  ui_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/quadruped_cmd_vel_ui", 10,
    std::bind(&QuadrupedController::cmdVelUiCallback, this, std::placeholders::_1));
  // nav2's final velocity, after velocity_smoother and collision_monitor
  nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&QuadrupedController::cmdVelNavCallback, this, std::placeholders::_1));
  // The UI's /safety_quadruped_cmd_vel_ui after nav2's assisted_teleop checked it against the
  // local costmap. assisted_teleop only runs while it has a goal, held here while paused.
  teleop_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/quadruped_cmd_vel_teleop", 10,
    std::bind(&QuadrupedController::cmdVelTeleopCallback, this, std::placeholders::_1));
  assisted_teleop_client_ = rclcpp_action::create_client<AssistedTeleop>(this, "/assisted_teleop");
  // Best effort matches the robot's publishers whether they are reliable or not.
  lowstate_sub_ = create_subscription<unitree_go::msg::LowState>(
    lowstate_topic, rclcpp::SensorDataQoS(),
    std::bind(&QuadrupedController::lowStateCallback, this, std::placeholders::_1));
  sport_state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
    sportstate_topic, rclcpp::SensorDataQoS(),
    std::bind(&QuadrupedController::sportStateCallback, this, std::placeholders::_1));
  htx_state_sub_ = create_subscription<std_msgs::msg::String>(
    "/goaway_locomotion/state", rclcpp::SensorDataQoS(),
    std::bind(&QuadrupedController::htxStateCallback, this, std::placeholders::_1));

  // Navigation mirrors for quadruped_main_status, as tmms. These are latched: a transient
  // local subscription still gets the value published before this node started.
  const auto latched = rclcpp::QoS(1).transient_local().reliable();
  yasmin_state_sub_ = create_subscription<std_msgs::msg::String>(
    "/yasmin_state", latched,
    std::bind(&QuadrupedController::yasminStateCallback, this, std::placeholders::_1));
  localization_status_sub_ = create_subscription<std_msgs::msg::String>(
    "/localization_status", latched,
    std::bind(&QuadrupedController::localizationStatusCallback, this, std::placeholders::_1));
  current_map_sub_ = create_subscription<std_msgs::msg::String>(
    "/current_map", latched,
    std::bind(&QuadrupedController::currentMapCallback, this, std::placeholders::_1));
  // Volatile, as goaway_yasmin publishes it
  curr_navplan_sub_ = create_subscription<goaway_msgs::msg::NavigationPlan>(
    "/curr_navplan", 10,
    std::bind(&QuadrupedController::currNavplanCallback, this, std::placeholders::_1));
  amcl_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", latched,
    std::bind(&QuadrupedController::amclPoseCallback, this, std::placeholders::_1));
  // mapping_manager's JSON status: no unpause during a session
  mapping_status_sub_ = create_subscription<std_msgs::msg::String>(
    "/mapping_manager/status", latched,
    std::bind(&QuadrupedController::mappingStatusCallback, this, std::placeholders::_1));

  // Read by goaway_locomotion_node in its Velocity state.
  consolidated_pub_ = create_publisher<geometry_msgs::msg::Twist>(
    "/consolidated_quadruped_cmd_vel", 10);
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  main_status_pub_ = create_publisher<Status>("quadruped_main_status", 10);
  // True while the safety teleop works (an assisted_teleop goal is running). Latched.
  safety_ready_pub_ = create_publisher<std_msgs::msg::Bool>(
    "~/safety_ready", rclcpp::QoS(1).transient_local());

  quadruped_cmd_srv_ = create_service<StringTrigger>(
    "~/quadruped_cmd",
    std::bind(&QuadrupedController::quadrupedCmdCallback, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(), service_group_);
  quadruped_pause_srv_ = create_service<std_srvs::srv::SetBool>(
    "~/quadruped_pause",
    std::bind(&QuadrupedController::quadrupedPauseCallback, this,
      std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(), service_group_);

  move_timer_ = create_wall_timer(
    20ms, std::bind(&QuadrupedController::moveTimerCallback, this));
  main_status_timer_ = create_wall_timer(
    200ms, std::bind(&QuadrupedController::mainStatusTimerCallback, this));
  // Keeps locomotion_policy true to the robot when a switch is done elsewhere (go2_mode).
  mode_poll_timer_ = create_wall_timer(
    2s, std::bind(&QuadrupedController::modePollTimerCallback, this), poll_group_);
  // Default group, like the action client's callbacks: they never overlap.
  assisted_teleop_timer_ = create_wall_timer(
    200ms, std::bind(&QuadrupedController::assistedTeleopTimerCallback, this));

  RCLCPP_INFO(
    get_logger(), "Quadruped controller started (lowstate %s, unitree_mode %s)",
    lowstate_topic.c_str(), unitree_mode_.c_str());
}

QuadrupedController::~QuadrupedController()
{
  if (switch_thread_.joinable()) {switch_thread_.join();}
}

void QuadrupedController::cmdVelUiCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!paused_) {return;}
  ui_twist_ = *msg;
  last_ui_time_ = now();
}

void QuadrupedController::cmdVelNavCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (paused_) {return;}
  nav_twist_ = *msg;
  last_nav_time_ = now();
}

void QuadrupedController::cmdVelTeleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!paused_) {return;}
  // assisted_teleop publishes every cycle, zeros included: take the first zero, then let the
  // input age out instead of commanding zero forever.
  const bool is_zero = msg->linear.x == 0.0 && msg->linear.y == 0.0 && msg->angular.z == 0.0;
  if (is_zero && prev_teleop_zero_) {return;}
  prev_teleop_zero_ = is_zero;
  teleop_twist_ = *msg;
  last_teleop_time_ = now();
}

void QuadrupedController::assistedTeleopTimerCallback()
{
  using GoalHandle = rclcpp_action::ClientGoalHandle<AssistedTeleop>;
  bool paused;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    paused = paused_;
  }
  const auto t = now();

  // A server that crashed or restarted never answers for its old goal.
  if (assisted_teleop_state_ != TeleopGoal::kNone &&
    (t - assisted_teleop_seen_).seconds() > kTeleopGoalSilence)
  {
    RCLCPP_WARN(get_logger(), "assisted_teleop went silent, dropping its goal");
    assisted_teleop_state_ = TeleopGoal::kNone;
    assisted_teleop_goal_.reset();
    ++assisted_teleop_seq_;
  }

  if (!paused && assisted_teleop_state_ == TeleopGoal::kActive) {
    try {
      assisted_teleop_client_->async_cancel_goal(assisted_teleop_goal_);
      assisted_teleop_state_ = TeleopGoal::kCanceling;
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
      assisted_teleop_state_ = TeleopGoal::kNone;
      assisted_teleop_goal_.reset();
    }
  } else if (paused && assisted_teleop_state_ == TeleopGoal::kNone &&
    assisted_teleop_client_->action_server_is_ready())
  {
    const uint64_t seq = ++assisted_teleop_seq_;
    rclcpp_action::Client<AssistedTeleop>::SendGoalOptions options;
    options.goal_response_callback = [this, seq](const GoalHandle::SharedPtr & handle) {
        if (seq != assisted_teleop_seq_) {
          // Abandoned while waiting: don't leave it running on the server.
          if (handle) {assisted_teleop_client_->async_cancel_goal(handle);}
          return;
        }
        if (!handle) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 10000, "assisted_teleop rejected the goal, retrying");
          assisted_teleop_state_ = TeleopGoal::kNone;
          return;
        }
        assisted_teleop_goal_ = handle;
        assisted_teleop_state_ = TeleopGoal::kActive;
        assisted_teleop_seen_ = now();
        RCLCPP_INFO(get_logger(), "Safety teleop ready (assisted_teleop goal running)");
      };
    options.feedback_callback = [this, seq](
      GoalHandle::SharedPtr, const std::shared_ptr<const AssistedTeleop::Feedback>) {
        if (seq == assisted_teleop_seq_) {assisted_teleop_seen_ = now();}
      };
    options.result_callback = [this, seq](const GoalHandle::WrappedResult & result) {
        if (seq != assisted_teleop_seq_) {return;}
        bool still_paused;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          still_paused = paused_;
        }
        if (result.code != rclcpp_action::ResultCode::CANCELED && still_paused) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 10000,
            "assisted_teleop goal ended (code %d), resending", static_cast<int>(result.code));
        }
        assisted_teleop_state_ = TeleopGoal::kNone;
        assisted_teleop_goal_.reset();
      };
    // time_allowance 0 = no timeout: the goal lasts until unpause cancels it.
    assisted_teleop_state_ = TeleopGoal::kPending;
    assisted_teleop_seen_ = t;
    assisted_teleop_client_->async_send_goal(AssistedTeleop::Goal(), options);
  }
  publishSafetyReady();
}

void QuadrupedController::publishSafetyReady()
{
  const bool ready = assisted_teleop_state_ == TeleopGoal::kActive;
  const auto t = now();
  // On change, and once a second for late joiners that miss the latched sample
  if (ready == safety_ready_ && (t - safety_ready_time_).seconds() < 1.0) {return;}
  if (ready != safety_ready_ && !ready) {
    RCLCPP_WARN(get_logger(), "Safety teleop not ready (no assisted_teleop goal)");
  }
  safety_ready_ = ready;
  safety_ready_time_ = t;
  std_msgs::msg::Bool msg;
  msg.data = ready;
  safety_ready_pub_->publish(msg);
}

void QuadrupedController::lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    battery_soc_ = msg->bms_state.soc;
  }

  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = now();
  joint_state.name.assign(kJointNames.begin(), kJointNames.end());
  joint_state.position.resize(kNumJoints);
  joint_state.velocity.resize(kNumJoints);
  joint_state.effort.resize(kNumJoints);
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const auto & motor = msg->motor_state[i];
    joint_state.position[i] = motor.q;
    joint_state.velocity[i] = motor.dq;
    joint_state.effort[i] = motor.tau_est;
  }
  joint_state_pub_->publish(joint_state);
}

void QuadrupedController::sportStateCallback(
  const unitree_go::msg::SportModeState::SharedPtr msg)
{
  const uint32_t code = msg->error_code;
  bool changed;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    changed = code != sport_code_;
    sport_code_ = code;
    last_sport_time_ = now();
  }
  // Only on change: how codes still missing from sportRobotMode() show up.
  if (changed) {
    const auto mode = sportRobotMode(code);
    RCLCPP_INFO(
      get_logger(), "Unitree state: %u (%s)", code, mode.empty() ? "unknown" : mode.c_str());
  }
}

void QuadrupedController::htxStateCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string state, upright, scan;
  try {
    const auto json = nlohmann::json::parse(msg->data);
    state = json.value("state", "");
    upright = json.value("upright", "");
    scan = json.value("height_scan", "");
  } catch (const nlohmann::json::exception &) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  const bool lost = scan == "lost";
  if (lost != htx_scan_lost_) {
    if (lost) {
      RCLCPP_WARN(get_logger(), "HTX walk: height scan lost, walking as if on flat ground");
    } else {
      RCLCPP_INFO(get_logger(), "HTX walk: height scan %s", scan == "ok" ? "back" : "not in use");
    }
  }
  htx_state_ = state;
  htx_upright_ = upright;
  htx_scan_lost_ = lost;
  last_htx_time_ = now();
}

void QuadrupedController::yasminStateCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  yasmin_state_ = msg->data;
}

void QuadrupedController::localizationStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  localization_status_ = msg->data;
}

void QuadrupedController::currentMapCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_map_ = msg->data;
}

void QuadrupedController::currNavplanCallback(
  const goaway_msgs::msg::NavigationPlan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_navplan_id_ = msg->navplan_id;
  current_navplan_status_ = msg->status;
}

void QuadrupedController::amclPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // Planar (x, y, yaw) and only on AMCL updates, i.e. after some motion: right for a map pose.
  std::lock_guard<std::mutex> lock(mutex_);
  pose_ = msg->pose.pose;
}

void QuadrupedController::mappingStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  bool mapping;
  try {
    mapping = nlohmann::json::parse(msg->data).value("mapping", false);
  } catch (const nlohmann::json::exception &) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  mapping_ = mapping;
}

std::string QuadrupedController::notDrivableReasonLocked(const std::string & mode) const
{
  if (switching_) {return "a locomotion switch is running";}
  if (mapping_) {return "a mapping session is running; save or stop it first";}
  if (mode == Status::BALANCE_STAND) {return "";}
  // The L1 sees the room from a different height and angle upright: no localization or costmap
  if (policy_ == Status::HTX && htx_state_ == "UprightWalk") {
    return "upright walk can't be navigated; come down and WALK first";
  }
  if (mode.empty()) {return "no robot state (is the locomotion running?)";}
  return "the robot isn't walking (" + mode + "); WALK first";
}

void QuadrupedController::autoPause(const std::string & reason)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (paused_) {return;}
    paused_ = true;
    not_drivable_ = false;
    staleAllInputsLocked();
  }
  publishStop();
  // The nav goal stays (as a pause in tmms): the progress checker holds it, CANCEL ends it.
  RCLCPP_WARN(get_logger(), "Auto-paused: %s. The UI drives again", reason.c_str());
}

void QuadrupedController::staleAllInputsLocked()
{
  // Nothing cached before a pause or switch may drive the robot after it.
  const rclcpp::Time never{0, 0, RCL_ROS_TIME};
  last_ui_time_ = never;
  last_nav_time_ = never;
  last_teleop_time_ = never;
}

void QuadrupedController::publishStop()
{
  consolidated_pub_->publish(geometry_msgs::msg::Twist{});
  bool unitree;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    unitree = policy_ == Status::UNITREE;
  }
  if (unitree) {
    sport_->send(kSportMove, nlohmann::json{{"x", 0.0}, {"y", 0.0}, {"z", 0.0}}.dump());
  }
}

void QuadrupedController::moveTimerCallback()
{
  geometry_msgs::msg::Twist chosen;
  bool publish;
  bool unitree;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto t = now();
    // Hard switch as in tmms: paused -> UI only (direct, or the safety teleop after
    // assisted_teleop; the UI drives one at a time), unpaused -> nav only. Nothing moves the
    // robot during a locomotion switch.
    const bool ui_fresh = paused_ && (t - last_ui_time_).seconds() < kUiTimeout;
    const bool teleop_fresh = paused_ && (t - last_teleop_time_).seconds() < kTeleopTimeout;
    const bool nav_fresh = !paused_ && (t - last_nav_time_).seconds() < kNavTimeout;
    const bool fresh = (ui_fresh || teleop_fresh || nav_fresh) && !switching_;
    if (fresh) {chosen = ui_fresh ? ui_twist_ : (teleop_fresh ? teleop_twist_ : nav_twist_);}
    // While fresh, plus one zero on the falling edge, then silent.
    publish = fresh || input_was_fresh_;
    input_was_fresh_ = fresh;
    unitree = policy_ == Status::UNITREE;
  }
  if (!publish) {return;}

  consolidated_pub_->publish(chosen);
  if (unitree) {
    sport_->send(
      kSportMove,
      nlohmann::json{{"x", chosen.linear.x}, {"y", chosen.linear.y}, {"z", chosen.angular.z}}.dump());
  }
}

std::string QuadrupedController::robotMode()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto t = now();
  if (policy_ == Status::HTX) {
    return (t - last_htx_time_).seconds() < kHtxStateTimeout ?
           htxRobotMode(htx_state_, htx_upright_) : "";
  }
  if (policy_ == Status::UNITREE) {
    return (t - last_sport_time_).seconds() < kSportStateTimeout ? sportRobotMode(sport_code_) : "";
  }
  return "";
}

void QuadrupedController::mainStatusTimerCallback()
{
  Status status;
  status.robot_mode = robotMode();
  std::string auto_pause;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Nav owns the robot but it can't drive any more (a fall, the E-stop, a stale state):
    // hand it back to the UI.
    if (!paused_) {
      const auto reason = notDrivableReasonLocked(status.robot_mode);
      if (reason.empty()) {
        not_drivable_ = false;
      } else if (!not_drivable_) {
        not_drivable_ = true;
        not_drivable_since_ = now();
      } else if ((now() - not_drivable_since_).seconds() >= kAutoPauseDelay) {
        auto_pause = reason;
      }
    }
    status.pose = pose_;
    status.battery_percentage = battery_soc_;
    status.is_paused = paused_ || !auto_pause.empty();
    // PAUSED overlays the state machine's state, as tmms
    status.navigation_state = status.is_paused ? Status::PAUSED : yasmin_state_;
    status.localization_status = localization_status_;
    status.current_map = current_map_;
    status.current_navplan_id = current_navplan_id_;
    status.current_navplan_status = current_navplan_status_;
    status.locomotion_policy = policy_;
    status.height_scan_lost = policy_ == Status::HTX && htx_scan_lost_ &&
      (now() - last_htx_time_).seconds() < kHtxStateTimeout;
  }
  if (!auto_pause.empty()) {autoPause(auto_pause);}
  main_status_pub_->publish(status);
}

bool QuadrupedController::checkMode(std::string & name)
{
  const auto reply = switcher_->call(kSwitcherCheckMode, "", 1500ms);
  if (!reply.answered || reply.code != 0) {return false;}
  try {
    name = nlohmann::json::parse(reply.data).value("name", "");
  } catch (const nlohmann::json::exception &) {
    return false;
  }
  return true;
}

bool QuadrupedController::waitMode(bool released, double timeout_s, std::string & name)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < end) {
    if (checkMode(name) && name.empty() == released) {return true;}
    std::this_thread::sleep_for(500ms);
  }
  return false;
}

void QuadrupedController::modePollTimerCallback()
{
  if (switching_) {return;}
  const int generation = switch_generation_;
  std::string name;
  if (!checkMode(name)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "Motion switcher not answering, locomotion_policy not updated");
    return;
  }
  // A switch started or finished meanwhile: this answer is stale.
  if (switching_ || generation != switch_generation_) {return;}

  const std::string policy = name.empty() ? Status::HTX : Status::UNITREE;
  std::string htx_state;
  bool htx_fresh;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (policy != policy_) {
      const std::string mode = name.empty() ? "released" : "mode " + name;
      RCLCPP_INFO(get_logger(), "Locomotion policy: %s (%s)", policy.c_str(), mode.c_str());
    }
    policy_ = policy;
    htx_state = htx_state_;
    htx_fresh = (now() - last_htx_time_).seconds() < kHtxStateTimeout;
  }
  if (policy == Status::HTX && (!htx_fresh || htx_state == "Off")) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "Unitree's controller is released but HTX isn't driving (%s): nothing controls the "
      "robot; 'damp' puts HTX in Damping",
      htx_fresh ? "state Off" : "goaway_locomotion_node not running");
  }
}

int32_t QuadrupedController::htxSetState(const std::string & state, std::string & message)
{
  return htxCall(kHtxSetState, nlohmann::json{{"state", state}}.dump(), state, message);
}

int32_t QuadrupedController::htxAction(const std::string & action, std::string & message)
{
  return htxCall(kHtxAction, nlohmann::json{{"action", action}}.dump(), action, message);
}

int32_t QuadrupedController::htxCall(
  int32_t api_id, const std::string & parameter, const std::string & what, std::string & message)
{
  const auto reply = locomotion_->call(api_id, parameter, 2000ms);
  if (!reply.answered) {
    message = "goaway_locomotion_node did not answer";
    return kNoAnswer;
  }
  std::string reason;
  try {
    reason = nlohmann::json::parse(reply.data).value("reason", "");
  } catch (const nlohmann::json::exception &) {
  }
  if (reply.code == 0) {
    message = "HTX: " + what + (reason.empty() ? "" : " (" + reason + ")");
  } else {
    message = "HTX refused " + what + " (code " + std::to_string(reply.code) + ")" +
      (reason.empty() ? "" : ": " + reason);
  }
  return reply.code;
}

bool QuadrupedController::htxDampingWithRetry(double timeout_s, std::string & message)
{
  // HTX's interlock refuses while it still hears another LowCmd, e.g. right after a release.
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (true) {
    const int32_t code = htxSetState("Damping", message);
    if (code == 0) {return true;}
    if (code != kHtxRefused || std::chrono::steady_clock::now() >= end) {return false;}
    std::this_thread::sleep_for(300ms);
  }
}

bool QuadrupedController::sportCmd(int32_t api_id, const std::string & what, std::string & message)
{
  const auto reply = sport_->call(api_id, "", 3000ms);
  if (!reply.answered) {
    message = what + ": the Unitree sport service did not answer";
    return false;
  }
  if (reply.code != 0) {
    message = what + " failed (sport code " + std::to_string(reply.code) + ")";
    return false;
  }
  message = what;
  return true;
}

bool QuadrupedController::executeCmd(const std::string & cmd, std::string & message)
{
  if (cmd == "use_htx") {return startSwitch(Status::HTX, message);}
  if (cmd == "use_unitree") {return startSwitch(Status::UNITREE, message);}
  if (switching_) {
    message = "Switching locomotion, try again when it is done";
    return false;
  }

  std::string policy;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    policy = policy_;
  }
  for (const auto & command : kCommands) {
    if (cmd != command.name) {continue;}
    bool ok;
    if (policy == Status::HTX) {
      ok = htxSetState(command.htx_state, message) == 0;
    } else if (policy == Status::UNITREE) {
      if (command.sport_api == 0) {
        message = cmd + " needs the HTX locomotion (use_htx first)";
        return false;
      }
      ok = sportCmd(command.sport_api, command.sport_what, message);
    } else {
      message = "Locomotion policy unknown: the motion switcher hasn't answered yet";
      return false;
    }
    RCLCPP_INFO(get_logger(), "%s: %s", cmd.c_str(), message.c_str());
    return ok;
  }
  for (const auto & action : kHtxActions) {
    if (cmd != action.name) {continue;}
    if (policy != Status::HTX) {
      message = cmd + " needs the HTX locomotion (use_htx first)";
      return false;
    }
    const bool ok = htxAction(action.action, message) == 0;
    RCLCPP_INFO(get_logger(), "%s: %s", cmd.c_str(), message.c_str());
    return ok;
  }
  message = "Unknown command: " + cmd;
  RCLCPP_WARN(get_logger(), "Unknown quadruped command: %s", cmd.c_str());
  return false;
}

bool QuadrupedController::startSwitch(const std::string & target, std::string & message)
{
  std::string policy, htx_state;
  uint32_t sport_code;
  bool sport_fresh, htx_fresh;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto t = now();
    policy = policy_;
    sport_code = sport_code_;
    sport_fresh = (t - last_sport_time_).seconds() < kSportStateTimeout;
    htx_state = htx_state_;
    htx_fresh = (t - last_htx_time_).seconds() < kHtxStateTimeout;
  }

  if (switching_) {
    message = "A locomotion switch is already running";
  } else if (policy.empty()) {
    message = "Locomotion policy unknown: the motion switcher hasn't answered yet";
  } else if (policy == target) {
    message = "Already using " + target;
  } else if (!htx_fresh) {
    message = "goaway_locomotion_node isn't running";
  } else if (target == Status::HTX) {
    // Unitree must have the robot lying down: HTX starts in Damping.
    if (!sport_fresh) {
      message = "No sport state from the robot, can't confirm it is lying down";
    } else if (sport_code != kSportCodeLieDown && sport_code != kSportCodeDamping) {
      const auto mode = sportRobotMode(sport_code);
      message = "Lie the robot down first (stand_down or damp); Unitree reports '" +
        (mode.empty() ? std::to_string(sport_code) : mode) + "'";
    } else if (htx_state != "Off") {
      message = "goaway_locomotion_node is in " + htx_state + ", expected Off";
    }
  } else if (htx_state != "Damping") {
    message = "Switch back only from Damping (damp first); HTX is in " + htx_state;
  }
  if (!message.empty()) {
    RCLCPP_WARN(get_logger(), "use_%s refused: %s", target.c_str(), message.c_str());
    return false;
  }

  if (switching_.exchange(true)) {
    message = "A locomotion switch is already running";
    return false;
  }
  if (switch_thread_.joinable()) {switch_thread_.join();}
  switch_thread_ = std::thread(
    target == Status::HTX ? &QuadrupedController::switchToHtx : &QuadrupedController::switchToUnitree,
    this);
  message = "Switching to " + target + " (quadruped_main_status.locomotion_policy changes when done)";
  RCLCPP_INFO(get_logger(), "%s", message.c_str());
  return true;
}

void QuadrupedController::endSwitch(const std::string & policy)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!policy.empty()) {policy_ = policy;}
    staleAllInputsLocked();
  }
  ++switch_generation_;
  switching_ = false;
}

void QuadrupedController::switchToHtx()
{
  std::string name;
  bool released = false;
  for (int attempt = 0; attempt < 3 && !released; ++attempt) {
    const auto reply = switcher_->call(kSwitcherReleaseMode, "", 5000ms);
    if (!reply.answered) {
      RCLCPP_WARN(get_logger(), "ReleaseMode: no answer");
    } else if (reply.code != 0) {
      RCLCPP_WARN(get_logger(), "ReleaseMode returned code %d", reply.code);
    }
    released = waitMode(true, 3.0, name);
  }
  if (!released) {
    RCLCPP_ERROR(get_logger(), "Switch to htx failed: Unitree's controller did not release");
    endSwitch("");
    return;
  }

  std::string message;
  if (htxDampingWithRetry(5.0, message)) {
    RCLCPP_INFO(get_logger(), "Switched to htx: HTX in Damping");
    endSwitch(Status::HTX);
    return;
  }

  // Never leave the robot without a controller: hand it back to Unitree.
  RCLCPP_ERROR(
    get_logger(), "Switch to htx failed (%s); selecting Unitree's '%s' again",
    message.c_str(), unitree_mode_.c_str());
  switcher_->call(kSwitcherSelectMode, nlohmann::json{{"name", unitree_mode_}}.dump(), 5000ms);
  if (!waitMode(false, 10.0, name)) {
    RCLCPP_ERROR(get_logger(), "Unitree's controller did not come back either: nothing controls the robot");
  }
  endSwitch("");
}

void QuadrupedController::switchToUnitree()
{
  std::string message;
  if (htxSetState("Off", message) != 0) {
    RCLCPP_ERROR(get_logger(), "Switch to unitree failed: %s", message.c_str());
    endSwitch("");
    return;
  }

  std::string name;
  const auto reply = switcher_->call(
    kSwitcherSelectMode, nlohmann::json{{"name", unitree_mode_}}.dump(), 5000ms);
  if (reply.answered && reply.code == 0 && waitMode(false, 10.0, name)) {
    RCLCPP_INFO(get_logger(), "Switched to unitree (mode %s)", name.c_str());
    endSwitch(Status::UNITREE);
    return;
  }

  const std::string why = !reply.answered ? "got no answer" :
    reply.code != 0 ? "returned code " + std::to_string(reply.code) : "never took effect";
  RCLCPP_ERROR(
    get_logger(), "Switch to unitree failed: SelectMode '%s' %s; putting HTX back in Damping",
    unitree_mode_.c_str(), why.c_str());
  if (!htxDampingWithRetry(5.0, message)) {
    RCLCPP_ERROR(get_logger(), "HTX didn't take over again (%s): check the robot", message.c_str());
  }
  endSwitch("");
}

void QuadrupedController::quadrupedCmdCallback(
  const StringTrigger::Request::SharedPtr req, StringTrigger::Response::SharedPtr res)
{
  bool paused;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    paused = paused_;
  }
  // damp is the E-stop: it works whoever owns the motion.
  if (!paused && req->data != "damp") {
    res->success = false;
    res->message = "Robot is not paused; nav owns motion. Pause via ~/quadruped_pause first.";
    return;
  }
  res->success = executeCmd(req->data, res->message);
}

void QuadrupedController::quadrupedPauseCallback(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  // Unpause hands an already walking robot to nav; nothing moves it by itself (the B2's
  // balance_stand in tmms leaves a body pose the Go2 doesn't have).
  const auto mode = req->data ? std::string() : robotMode();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!req->data) {
      const auto reason = notDrivableReasonLocked(mode);
      if (!reason.empty()) {
        res->success = false;
        res->message = "Unpause refused: " + reason;
        RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
        return;
      }
    }
    paused_ = req->data;
    not_drivable_ = false;
    staleAllInputsLocked();
  }
  // Stop now rather than waiting for the inputs to age out.
  publishStop();
  res->success = true;
  res->message = req->data ?
    "Paused: UI cmd_vel drives, nav cmd_vel ignored" :
    "Unpaused: nav cmd_vel drives, UI cmd_vel ignored";
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

}  // namespace goaway_quadruped_controller
