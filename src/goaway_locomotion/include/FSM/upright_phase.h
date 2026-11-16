#pragma once

#include <atomic>

// What UprightWalk is doing, for the state topic's "upright" (written on the FSM thread).
namespace upright
{
enum class Phase
{
    Upright,      // upright, both arms in
    Rising,       // not upright yet since entering; no arm
    Tipped,       // tip_exit put the arm back; rising again
    PawLeft,
    PawRight,
    WaveLeft,
    WaveRight,
    ArmBack,      // an arm on its way in
    ComingDown,   // from the come_down press (an arm out comes back first)
};

inline std::atomic<Phase> phase{Phase::Rising};

inline const char * phase_name(Phase p)
{
    switch (p)
    {
        case Phase::Rising: return "rising";
        case Phase::Tipped: return "tipped";
        case Phase::PawLeft: return "paw_left";
        case Phase::PawRight: return "paw_right";
        case Phase::WaveLeft: return "wave_left";
        case Phase::WaveRight: return "wave_right";
        case Phase::ArmBack: return "arm_back";
        case Phase::ComingDown: return "coming_down";
        default: return "";
    }
}
}  // namespace upright
