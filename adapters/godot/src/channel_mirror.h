// adapters/godot/src/channel_mirror.h - control-channel mirroring between an adapter's own channel map and ps::World
// (assists I7c, vault PHYS-074).
//
// The adapter re-sends EVERY channel from its own atomic map to the World before each tick (push_channels). Since
// assists I7c the physics can WRITE a channel itself (a drive-mode change writes its traction_control_default /
// launch_control_default into assist.traction_control / assist.launch_control; World::step applies the write after the
// tick). Without a mirror back, the next push would put the adapter's stale value over the physics' write and a UI toggle
// would read the old state. adopt_world_channel_writes() therefore runs right after the World step: every channel whose
// World value differs from the value pushed before that step is a physics write and is adopted into the adapter's
// atomic with compare_exchange from the pushed value - a user write (set_control, main thread) made meanwhile changed
// the atomic away from the pushed value, so the exchange fails and the user's value wins.
//
// Godot-free and allocation-free after the first tick (the `pushed` scratch is sized once), so it is unit-tested in
// tests/unit/test_assist_channel_writeback.cpp without godot-cpp and reusable by any adapter (UE5). Both functions must
// see the SAME map with no insertion in between (the adapter pre-seeds every key before the sim thread starts), which
// makes the iteration order identical; `pushed` is index-aligned with it.
#pragma once

#include <atomic>
#include <cstddef>
#include <vector>

namespace ps_godot {

// Channels: map-like with .first = name, .second = std::atomic<float>. set_world(name, float) pushes one channel.
template <class Channels, class SetWorld>
void push_channels(Channels& channels, std::vector<float>& pushed, SetWorld&& set_world) {
    if (pushed.size() != channels.size()) pushed.resize(channels.size()); // first tick only
    std::size_t i = 0;
    for (auto& kv : channels) {
        const float v = kv.second.load(std::memory_order_relaxed);
        pushed[i++] = v;
        set_world(kv.first, v);
    }
}

// get_world(name) -> the World's current value. Returns the number of adopted channels.
template <class Channels, class GetWorld>
std::size_t adopt_world_channel_writes(Channels& channels, const std::vector<float>& pushed, GetWorld&& get_world) {
    std::size_t adopted = 0, i = 0;
    for (auto& kv : channels) {
        if (i >= pushed.size()) break;
        const float before = pushed[i++];
        const float now = static_cast<float>(get_world(kv.first));
        if (now == before) continue;
        float expected = before;
        if (kv.second.compare_exchange_strong(expected, now, std::memory_order_relaxed)) ++adopted;
    }
    return adopted;
}

} // namespace ps_godot
