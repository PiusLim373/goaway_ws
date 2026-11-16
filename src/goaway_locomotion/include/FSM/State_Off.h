#pragma once

#include "FSMState.h"

// Off: sends no motor commands, for while Unitree's own controller drives the robot.
// The FSM starts here. It leaves only by API request (config.yaml: empty conditions),
// and only once no other program is sending motor commands, i.e. after Unitree's
// controller has been released.
class State_Off : public FSMState
{
public:
    State_Off(int state, std::string state_string)
    : FSMState(state, state_string, false)
    {
        other_lowcmd_ = std::make_shared<unitree::robot::go2::subscription::LowCmd>();
        for (auto & check : this->registered_checks)
        {
            auto asked = check.first;
            check.first = [this, asked]()->bool{
                if (!asked()) return false;
                if (other_lowcmd_->isTimeout()) return true;
                const std::string why = "another controller is still sending motor commands (release Unitree's first)";
                spdlog::warn("{}: {}.", this->getStateString(), why);
                FSMRequest::refuse(why);
                return false;
            };
        }
    }

    void enter()
    {
        spdlog::info("{}: not sending motor commands.", getStateString());
    }

    void run() {}
    void post_run() {}

private:
    // Hears any LowCmd on the bus; ours too, so it reads busy for a moment after we stop.
    std::shared_ptr<unitree::robot::go2::subscription::LowCmd> other_lowcmd_;
};

REGISTER_FSM(State_Off)
