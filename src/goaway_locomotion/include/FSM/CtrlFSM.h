// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <unitree/common/thread/recurrent_thread.hpp>
#include "BaseState.h"
#include "FSMRequest.h"
#include <atomic>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

class CtrlFSM
{
public:
    CtrlFSM(std::shared_ptr<BaseState> initstate)
    {
        // Initialize FSM states
        states.push_back(std::move(initstate));

    }

    CtrlFSM(YAML::Node cfg)
    {
        auto fsms = cfg["_"]; // enabled FSMs

        // register FSM string map; used for state transition
        for (auto it = fsms.begin(); it != fsms.end(); ++it)
        {
            std::string fsm_name = it->first.as<std::string>();
            int id = it->second["id"].as<int>();
            FSMStringMap.insert({id, fsm_name});
        }

        // Initialize FSM states
        for (auto it = fsms.begin(); it != fsms.end(); ++it)
        {
            std::string fsm_name = it->first.as<std::string>();
            int id = it->second["id"].as<int>();
            std::string fsm_type = it->second["type"] ? it->second["type"].as<std::string>() : fsm_name;
            auto fsm_class = getFsmMap().find("State_" + fsm_type);
            if (fsm_class == getFsmMap().end()) {
                throw std::runtime_error("FSM: Unknown FSM type " + fsm_type);
            }
            auto state_instance = fsm_class->second(id, fsm_name);
            add(state_instance);
        }
    }

    void start()
    {
        // Start from the first state in config.yaml (Off)
        currentState = states[0];
        currentState->enter();
        current_id_ = currentState->getState();

        fsm_thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "FSM", 0, this->dt * 1e6, &CtrlFSM::run_, this);
        spdlog::info("FSM: Start {}", currentState->getStateString());
    }

    void add(std::shared_ptr<BaseState> state)
    {
        for(auto & s : states)
        {
            if(s->isState(state->getState()))
            {
                spdlog::error("FSM: State_{} already exists", state->getStateString());
                std::exit(0);
            }
        }

        states.push_back(std::move(state));
    }
    
    ~CtrlFSM()
    {
        states.clear();
    }

    std::vector<std::shared_ptr<BaseState>> states;

    // Safe to call from any thread.
    int current_state() const { return current_id_; }

private:
    const double dt = 0.001;

    void run_()
    {
        currentState->pre_run();
        currentState->run();
        currentState->post_run();

        const int request = FSMRequest::begin_tick();

        // Check if need to change state
        int nextStateMode = 0;
        for(int i(0); i<currentState->registered_checks.size(); i++)
        {
            if(currentState->registered_checks[i].first())
            {
                nextStateMode = currentState->registered_checks[i].second;
                break;
            }
        }

        bool switched = false;
        if(nextStateMode != 0 && !currentState->isState(nextStateMode))
        {
            for(auto & state : states)
            {
                if(state->isState(nextStateMode))
                {
                    spdlog::info("FSM: Change state from {} to {}", currentState->getStateString(), state->getStateString());
                    BaseState::previous_state = currentState->getState();
                    currentState->exit();
                    currentState = state;
                    currentState->enter();
                    switched = true;
                    break;
                }
            }
        }
        current_id_ = currentState->getState();

        if(request != 0) answer(request, switched);

        const std::string action = FSMRequest::take_action();
        if(!action.empty()) answer_action(action);
    }

    // Runs an API action in the current state.
    void answer_action(const std::string & action)
    {
        FSMRequest::Result result{FSMRequest::OK, currentState->getStateString(), ""};
        const auto why = currentState->action(action);
        if(!why)
        {
            result.code = FSMRequest::NOT_ALLOWED;
            result.reason = action + " is not available in " + result.state;
        }
        else if(!why->empty())
        {
            result.code = FSMRequest::REFUSED;
            result.reason = *why;
        }
        FSMRequest::finish(std::move(result));
    }

    // Answers the API request handled in this tick.
    void answer(int request, bool switched)
    {
        FSMRequest::Result result{FSMRequest::NO_ANSWER, currentState->getStateString(), ""};
        if(currentState->isState(request))
        {
            const std::string why = switched ? currentState->refusal() : "";
            if(why.empty())
            {
                result.code = FSMRequest::OK;
                if(!switched) result.reason = "already in " + result.state;
            }
            else
            {
                // It hands back on the next tick.
                result.code = FSMRequest::REFUSED;
                result.reason = why;
                if(FSMStringMap.left.count(BaseState::previous_state))
                    result.state = FSMStringMap.left.at(BaseState::previous_state);
            }
        }
        else if(!FSMRequest::reason().empty())
        {
            result.code = FSMRequest::REFUSED;
            result.reason = FSMRequest::reason();
        }
        else if(switched)
        {
            result.code = FSMRequest::REFUSED;
            result.reason = "switched to " + result.state + " instead";
        }
        else
        {
            result.code = FSMRequest::NOT_ALLOWED;
            result.reason = "not allowed from " + result.state;
        }
        FSMRequest::finish(std::move(result));
    }

    std::shared_ptr<BaseState> currentState;
    std::atomic<int> current_id_{0};
    unitree::common::RecurrentThreadPtr fsm_thread_;
};