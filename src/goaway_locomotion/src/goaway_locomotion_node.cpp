// goaway_locomotion_node: the Go2 locomotion FSM (ported from unitree_rl_mjlab's
// go2_ctrl). An SDK program, not a ROS node: it must never link rclcpp (the SDK's
// CycloneDDS and ROS's can't share a process). ROS reaches it over DDS:
//   /api/goaway_locomotion/request|response   state changes (LocomotionServer)
//   /goaway_locomotion/state                  current state (JSON)
//   /consolidated_quadruped_cmd_vel           velocity command while in Velocity
#include "FSM/CtrlFSM.h"
#include "FSM/State_Off.h"
#include "FSM/State_Damping.h"
#include "FSM/State_JointLock.h"
#include "FSM/State_RLBase.h"
#include "FSM/State_Velocity.h"
#include "FSM/State_Recovery.h"
#include "FSM/State_DonkeyKick.h"
#include "FSM/State_UprightWalk.h"
#include "goaway_locomotion/goaway_locomotion.hpp"

#include <atomic>
#include <csignal>
#include <thread>

std::unique_ptr<LowCmd_t> FSMState::lowcmd = nullptr;
std::shared_ptr<LowState_t> FSMState::lowstate = nullptr;
std::shared_ptr<CmdVel_t> FSMState::cmd_vel = nullptr;
std::shared_ptr<Keyboard> FSMState::keyboard = nullptr;

namespace
{
std::atomic<bool> stop_requested{false};
void on_signal(int) { stop_requested = true; }
}

// Unlike go2_ctrl, this doesn't release Unitree's controller: the FSM starts in Off and
// sends nothing until asked (ROS switches the motion modes).
void init_fsm_state()
{
    FSMState::lowcmd = std::make_unique<LowCmd_t>();
    FSMState::lowstate = std::make_shared<LowState_t>();
    FSMState::cmd_vel = std::make_shared<CmdVel_t>("rt/consolidated_quadruped_cmd_vel");
    FSMState::cmd_vel->set_timeout_ms(500);
    spdlog::info("Waiting for connection to robot...");
    FSMState::lowstate->wait_for_connection();
    spdlog::info("Connected to robot.");
}

int main(int argc, char** argv)
{
    // Load parameters
    auto vm = param::helper(argc, argv);

    std::cout << " --- goaway --- \n";
    std::cout << "     Go2 locomotion (goaway_locomotion_node) \n";

    // Unitree DDS Config
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    init_fsm_state();

    // Initialize FSM
    auto fsm = std::make_unique<CtrlFSM>(param::config["FSM"]);
    fsm->start();

    goaway_locomotion::LocomotionServer server(*fsm);
    server.Init();
    server.Start();

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::cout << "Starts in Off (no motor commands). API: /api/goaway_locomotion/request,\n"
              << "  1001 GetState, 1002 SetState {\"state\": \"<name>\"}, 1003 Damp,\n"
              << "  1004 Action {\"action\": \"come_down\" | \"paw_left\" | \"paw_right\" | \"wave_left\"\n"
              << "             | \"wave_right\"} (UprightWalk; the same arm again puts it back).\n"
              << "Remote (Unitree's layout; PS pad in Sim2Sim: X Triangle, Y Square, Start Options), not in Off:\n"
              << "  [L2 + B]   Damping, from any state\n"
              << "  [L2 + A]   Damping -> JointLock (stand up); JointLock -> LieDown (lie down,\n"
              << "             then damping by itself); Velocity -> JointLock\n"
              << "  [Start]    JointLock <-> Velocity (walk); back to JointLock only when stopped\n"
              << "  [D-pad]    in Velocity: drive forward/back/sideways at a fixed speed\n"
              << "  [R2 + Y]   DonkeyKick, [R2 + X] QuickBuck: standing still, hands back by itself\n"
              << "  [R2 + D-pad L/R]  SideFlipLeft / SideFlipRight: the same (hold R2, then tap the D-pad)\n"
              << "  [R2 + D-pad up / down]  FrontFlip / BackFlip: the same\n"
              << "  [L2 + Up]  Recovery after a fall: lies down, then damping by itself\n"
              << "  [L2 + Y]   JointLock -> UprightWalk: rises on the rear legs. In it: sticks and D-pad walk,\n"
              << "             L2 + A comes down to JointLock, R2 + D-pad L/R toggle a paw,\n"
              << "             R1 + D-pad L/R a wave (one arm at a time)\n"
              << "In Velocity and UprightWalk, /consolidated_quadruped_cmd_vel drives while the remote is idle.\n";

    while (!stop_requested)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stopping: damp first so the robot lowers instead of losing its motors.
    if (!FSMStringMap.right.count("Off") || fsm->current_state() != FSMStringMap.right.at("Off"))
    {
        spdlog::info("Stopping: damping before exit.");
        server.set_state("Damping");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    spdlog::info("Stopped.");
    std::_Exit(0);
}
