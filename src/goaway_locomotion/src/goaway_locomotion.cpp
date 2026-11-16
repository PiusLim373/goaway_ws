#include "goaway_locomotion/goaway_locomotion.hpp"

#include <yaml-cpp/yaml.h>

#include "FSM/FSMState.h"
#include "FSM/scan_status.h"
#include "FSM/upright_phase.h"

namespace goaway_locomotion
{

namespace
{
constexpr auto kAnswerTimeout = std::chrono::milliseconds(500);
constexpr uint64_t kStateCheckUs = 50000;   // state topic: checked for changes this often
constexpr int kHeartbeatChecks = 20;        // and published at least every this many checks

// Flow style with double-quoted strings is valid JSON.
YAML::Emitter & json_emitter(YAML::Emitter & out)
{
    out.SetStringFormat(YAML::DoubleQuoted);
    out.SetMapFormat(YAML::Flow);
    out.SetSeqFormat(YAML::Flow);
    return out;
}

std::string state_name(int state)
{
    return FSMStringMap.left.count(state) ? FSMStringMap.left.at(state) : "";
}

// What UprightWalk is doing (State_UprightWalk sets it).
std::string upright_phase(int state)
{
    if (state_name(state) != "UprightWalk") return "";
    return upright::phase_name(upright::phase);
}

// Whether the walk policy has its terrain scan (the height_scan observation sets it).
std::string height_scan_status(int state)
{
    if (state_name(state) != "Velocity") return "";
    return terrain_scan::lost ? "lost" : "ok";
}
}  // namespace

LocomotionServer::LocomotionServer(const CtrlFSM & fsm)
: Server("goaway_locomotion"), fsm_(fsm)
{
    auto fsm_cfg = param::config["FSM"];
    for (auto it = fsm_cfg["_"].begin(); it != fsm_cfg["_"].end(); ++it)
    {
        const auto name = it->first.as<std::string>();
        auto & targets = allowed_[name];
        auto transitions = fsm_cfg[name]["transitions"];
        if (!transitions) continue;
        for (auto t = transitions.begin(); t != transitions.end(); ++t)
        {
            const auto target = t->first.as<std::string>();
            if (FSMStringMap.right.count(target)) targets.push_back(target);
        }
    }
}

void LocomotionServer::Init()
{
    SetApiVersion(param::VERSION);
    UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(API_GET_STATE, &LocomotionServer::on_get_state);
    UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(API_SET_STATE, &LocomotionServer::on_set_state);
    UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(API_DAMP, &LocomotionServer::on_damp);
    UT_ROBOT_SERVER_REG_API_HANDLER_NO_LEASE(API_ACTION, &LocomotionServer::on_action);

    state_pub_ = std::make_shared<unitree::robot::ChannelPublisher<std_msgs::msg::dds_::String_>>(
        "rt/goaway_locomotion/state");
    state_pub_->InitChannel();
    state_thread_ = std::make_shared<unitree::common::RecurrentThread>(
        "state_pub", UT_CPU_ID_NONE, kStateCheckUs, &LocomotionServer::publish_state, this);
}

FSMRequest::Result LocomotionServer::set_state(const std::string & name)
{
    FSMRequest::Result result{FSMRequest::NO_ANSWER, "", ""};
    if (!FSMStringMap.right.count(name))
    {
        result = {FSMRequest::BAD_REQUEST, "", "unknown state '" + name + "'"};
    }
    else
    {
        result = FSMRequest::ask(FSMStringMap.right.at(name), kAnswerTimeout);
    }
    if (result.state.empty()) result.state = state_name(fsm_.current_state());
    spdlog::info("API: SetState {} -> {} ({}){}", name, result.code, result.state,
                 result.reason.empty() ? "" : ", " + result.reason);
    return result;
}

int32_t LocomotionServer::on_get_state(const std::string &, std::string & data)
{
    data = state_json(fsm_.current_state());
    return FSMRequest::OK;
}

int32_t LocomotionServer::on_set_state(const std::string & parameter, std::string & data)
{
    std::string name;
    try
    {
        name = YAML::Load(parameter)["state"].as<std::string>();
    }
    catch (const YAML::Exception &)
    {
        FSMRequest::Result bad{FSMRequest::BAD_REQUEST, state_name(fsm_.current_state()),
                               "parameter must be {\"state\": \"<name>\"}"};
        data = reply_json(bad);
        return bad.code;
    }
    const auto result = set_state(name);
    data = reply_json(result);
    return result.code;
}

int32_t LocomotionServer::on_damp(const std::string &, std::string & data)
{
    const auto result = set_state("Damping");
    data = reply_json(result);
    return result.code;
}

int32_t LocomotionServer::on_action(const std::string & parameter, std::string & data)
{
    std::string name;
    try
    {
        name = YAML::Load(parameter)["action"].as<std::string>();
    }
    catch (const YAML::Exception &)
    {
        FSMRequest::Result bad{FSMRequest::BAD_REQUEST, state_name(fsm_.current_state()),
                               "parameter must be {\"action\": \"<name>\"}"};
        data = reply_json(bad);
        return bad.code;
    }
    auto result = FSMRequest::ask_action(name, kAnswerTimeout);
    if (result.state.empty()) result.state = state_name(fsm_.current_state());
    spdlog::info("API: Action {} -> {} ({}){}", name, result.code, result.state,
                 result.reason.empty() ? "" : ", " + result.reason);
    data = reply_json(result);
    return result.code;
}

std::string LocomotionServer::state_json(int state) const
{
    const auto name = state_name(state);
    const bool cmd_vel_active = FSMState::cmd_vel && !FSMState::cmd_vel->isTimeout();
    YAML::Emitter out;
    json_emitter(out) << YAML::BeginMap
        << YAML::Key << "state" << YAML::Value << name
        << YAML::Key << "allowed" << YAML::Value << (allowed_.count(name) ? allowed_.at(name) : std::vector<std::string>{})
        << YAML::Key << "cmd_vel_active" << YAML::Value << cmd_vel_active
        << YAML::Key << "upright" << YAML::Value << upright_phase(state)
        << YAML::Key << "height_scan" << YAML::Value << height_scan_status(state)
        << YAML::EndMap;
    return out.c_str();
}

std::string LocomotionServer::reply_json(const FSMRequest::Result & result) const
{
    YAML::Emitter out;
    json_emitter(out) << YAML::BeginMap
        << YAML::Key << "state" << YAML::Value << result.state
        << YAML::Key << "reason" << YAML::Value << result.reason
        << YAML::Key << "allowed" << YAML::Value
        << (allowed_.count(result.state) ? allowed_.at(result.state) : std::vector<std::string>{})
        << YAML::EndMap;
    return out.c_str();
}

void LocomotionServer::publish_state()
{
    const int state = fsm_.current_state();
    const auto upright = upright_phase(state);
    const auto scan = height_scan_status(state);
    if (state == last_state_ && upright == last_upright_ && scan == last_scan_
        && ++checks_since_publish_ < kHeartbeatChecks) return;
    last_state_ = state;
    last_upright_ = upright;
    last_scan_ = scan;
    checks_since_publish_ = 0;

    std_msgs::msg::dds_::String_ msg;
    msg.data(state_json(state));
    state_pub_->Write(msg);
}

}  // namespace goaway_locomotion
