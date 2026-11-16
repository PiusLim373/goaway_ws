#pragma once

#include "Types.h"
#include "param.h"
#include "FSM/BaseState.h"
#include "FSM/FSMRequest.h"
#include "isaaclab/devices/keyboard/keyboard.h"
#include "unitree_joystick_dsl.hpp"

class FSMState : public BaseState
{
public:
    // damping_on_timeout: switch to Damping when LowState stops (every state but Off).
    FSMState(int state, std::string state_string, bool damping_on_timeout = true)
    : BaseState(state, state_string)
    {
        spdlog::info("Initializing State_{} ...", state_string);

        auto transitions = param::config["FSM"][state_string]["transitions"];

        if(transitions)
        {
            auto transition_map = transitions.as<std::map<std::string, std::string>>();

            for(auto it = transition_map.begin(); it != transition_map.end(); ++it)
            {
                std::string target_fsm = it->first;
                if(!FSMStringMap.right.count(target_fsm))
                {
                    spdlog::warn("FSM State_'{}' not found in FSMStringMap!", target_fsm);
                    continue;
                }

                int fsm_id = FSMStringMap.right.at(target_fsm);

                // An empty condition means API request only (no button).
                std::string condition = it->second;
                if(condition.empty())
                {
                    registered_checks.emplace_back(
                        [fsm_id]()->bool{ return FSMRequest::requested(fsm_id); }, fsm_id);
                    continue;
                }
                unitree::common::dsl::Parser p(condition);
                auto ast = p.Parse();
                auto func = unitree::common::dsl::Compile(*ast);
                registered_checks.emplace_back(
                    std::make_pair(
                        [func, fsm_id]()->bool{
                            return func(FSMState::lowstate->joystick) || FSMRequest::requested(fsm_id);
                        },
                        fsm_id
                    )
                );
            }
        }

        // register for all states
        if(damping_on_timeout)
        {
            registered_checks.emplace_back(
                std::make_pair(
                    []()->bool{ return lowstate->isTimeout(); },
                    FSMStringMap.right.at("Damping")
                )
            );
        }
    }

    void pre_run()
    {
        lowstate->update();
        if(keyboard) keyboard->update();
    }

    void post_run()
    {
        lowcmd->unlockAndPublish();
    }

    static std::unique_ptr<LowCmd_t> lowcmd;
    static std::shared_ptr<LowState_t> lowstate;
    static std::shared_ptr<CmdVel_t> cmd_vel;
    static std::shared_ptr<Keyboard> keyboard;
};