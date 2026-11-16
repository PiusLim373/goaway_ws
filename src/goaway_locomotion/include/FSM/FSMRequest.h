#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <utility>

// A state change asked for over the API. The FSM thread takes it on its next tick and
// treats it like a one-tick button press, so only config.yaml transitions can be
// requested and every guard still applies. An action is the same for something inside
// the current state (BaseState::action).
class FSMRequest
{
public:
    enum Code : int32_t
    {
        OK = 0,
        NOT_ALLOWED = 4101,   // no transition to it from the current state
        REFUSED = 4102,       // a guard said no (reason given)
        NO_ANSWER = 4103,     // the FSM didn't take it in time
        BAD_REQUEST = 4104,   // unknown state or bad parameter
    };

    struct Result
    {
        int32_t code;
        std::string state;    // the state after the request was handled
        std::string reason;
    };

    // API thread: one request at a time; blocks until the FSM answers or timeout.
    static Result ask(int target, std::chrono::milliseconds timeout)
    {
        std::lock_guard<std::mutex> one_at_a_time(ask_mutex_);
        std::unique_lock<std::mutex> lock(mutex_);
        pending_ = target;
        answered_ = false;
        if (!cv_.wait_for(lock, timeout, []{ return answered_; }))
        {
            pending_ = 0;
            return {NO_ANSWER, "", "the FSM did not answer"};
        }
        return result_;
    }

    // API thread: as ask, for an action of the current state.
    static Result ask_action(const std::string & name, std::chrono::milliseconds timeout)
    {
        std::lock_guard<std::mutex> one_at_a_time(ask_mutex_);
        std::unique_lock<std::mutex> lock(mutex_);
        pending_action_ = name;
        answered_ = false;
        if (!cv_.wait_for(lock, timeout, []{ return answered_; }))
        {
            pending_action_.clear();
            return {NO_ANSWER, "", "the FSM did not answer"};
        }
        return result_;
    }

    // FSM thread, after the transition checks: takes the pending action (empty if none).
    static std::string take_action()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return std::exchange(pending_action_, {});
    }

    // FSM thread, before the transition checks: takes the pending request (0 if none).
    static int begin_tick()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        active_ = pending_;
        pending_ = 0;
        reason_.clear();
        return active_;
    }

    // FSM thread: true during the tick that handles a request for this state.
    static bool requested(int target) { return active_ != 0 && active_ == target; }

    // FSM thread: a guard blocked a transition; kept as the answer's reason.
    static void refuse(const std::string & reason)
    {
        if (active_ != 0 && reason_.empty()) reason_ = reason;
    }

    static const std::string & reason() { return reason_; }

    // FSM thread, after the transition checks.
    static void finish(Result result)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            result_ = std::move(result);
            answered_ = true;
            active_ = 0;
        }
        cv_.notify_all();
    }

private:
    inline static std::mutex ask_mutex_;
    inline static std::mutex mutex_;
    inline static std::condition_variable cv_;
    inline static int pending_ = 0;
    inline static int active_ = 0;
    inline static std::string pending_action_;
    inline static bool answered_ = false;
    inline static std::string reason_;
    inline static Result result_{NO_ANSWER, "", ""};
};
