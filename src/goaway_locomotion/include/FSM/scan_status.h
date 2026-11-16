#pragma once

#include <atomic>

// Whether the walk policy is on the terrain scan or its flat-ground fallback, for the state
// topic's "height_scan" (written by the height_scan observation on the FSM thread).
namespace terrain_scan
{
inline std::atomic<bool> lost{false};
}  // namespace terrain_scan
