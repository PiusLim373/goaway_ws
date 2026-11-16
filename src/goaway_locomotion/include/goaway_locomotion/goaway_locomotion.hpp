#pragma once

#include <map>
#include <string>
#include <vector>

#include <unitree/common/thread/recurrent_thread.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/server/server.hpp>

#include "FSM/CtrlFSM.h"
#include "FSM/FSMRequest.h"

namespace goaway_locomotion
{

// SDK service "goaway_locomotion" (ROS: /api/goaway_locomotion/request|response with
// unitree_api types) plus the state topic rt/goaway_locomotion/state (ROS:
// /goaway_locomotion/state, std_msgs/String). Parameters, replies and the topic are JSON.
// In UprightWalk the topic's "upright" is upright::phase_name (FSM/upright_phase.h); in Velocity
// its "height_scan" is "ok", or "lost" while the policy walks on flat ground (FSM/scan_status.h).
// Return codes: FSMRequest::Code.
class LocomotionServer : public unitree::robot::Server
{
public:
    static constexpr int32_t API_GET_STATE = 1001;   // -> {"state", "allowed", "cmd_vel_active"}
    static constexpr int32_t API_SET_STATE = 1002;   // {"state": "<name>"} -> {"state", "reason", "allowed"}
    static constexpr int32_t API_DAMP = 1003;        // SetState Damping
    static constexpr int32_t API_ACTION = 1004;      // {"action": "<name>"} -> {"state", "reason", "allowed"}

    explicit LocomotionServer(const CtrlFSM & fsm);

    void Init() override;

    // Asks the FSM for a state by name and waits for its answer.
    FSMRequest::Result set_state(const std::string & name);

private:
    int32_t on_get_state(const std::string & parameter, std::string & data);
    int32_t on_set_state(const std::string & parameter, std::string & data);
    int32_t on_damp(const std::string & parameter, std::string & data);
    int32_t on_action(const std::string & parameter, std::string & data);

    std::string state_json(int state) const;
    std::string reply_json(const FSMRequest::Result & result) const;
    void publish_state();

    const CtrlFSM & fsm_;
    std::map<std::string, std::vector<std::string>> allowed_;   // config.yaml transition targets
    unitree::robot::ChannelPublisherPtr<std_msgs::msg::dds_::String_> state_pub_;
    unitree::common::RecurrentThreadPtr state_thread_;
    int last_state_ = 0;
    std::string last_upright_;
    std::string last_scan_;
    int checks_since_publish_ = 0;
};

}  // namespace goaway_locomotion
