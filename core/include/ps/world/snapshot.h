// ps/world/snapshot.h — world state snapshot + lock-free triple buffer
// (PLAN.md D3: "the adapter runs the sim on its own thread and reads
// interpolated snapshots (lock-free triple buffer)").
//
// SnapshotTripleBuffer::publish is called by exactly one writer (World's
// snapshot stage); read() may be called from exactly one reader thread
// (e.g. a future Godot adapter's render thread). A second concurrent reader
// needs its own buffer instance - this type intentionally supports single-
// writer/single-reader only, which is what an atomic index hand-off can
// give you without a mutex.

#pragma once

#include "ps/math/pose.h"
#include "ps/world/ids.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace ps {

struct BodySnapshot {
    BodyId id;
    Pose pose;
    Motion motion;
};

struct WorldSnapshot {
    std::uint64_t tick = 0;
    real sim_time = 0.0;
    std::vector<BodySnapshot> bodies;

    // Deformation payload seam (PLAN.md section 6): empty in P1. A future
    // deformable body type can populate this per-body blob without changing
    // the snapshot protocol the adapter reads.
    std::vector<std::byte> deformation_payload;
};

class SnapshotTripleBuffer {
public:
    SnapshotTripleBuffer() {
        state_.store(pack(0, false), std::memory_order_relaxed);
        write_index_ = 1;
        read_index_ = 2;
    }

    // Writer thread only. Returns the slot to fill in place, then call
    // publish() once it is complete.
    WorldSnapshot& write_slot() { return slots_[static_cast<std::size_t>(write_index_)]; }

    void publish() {
        const std::uint8_t new_state = pack(write_index_, true);
        const std::uint8_t old_state = state_.exchange(new_state, std::memory_order_acq_rel);
        write_index_ = index_of(old_state);
    }

    // Reader thread only. Returns the most recently published snapshot.
    const WorldSnapshot& read() {
        const std::uint8_t s = state_.load(std::memory_order_acquire);
        if (is_dirty(s)) {
            const std::uint8_t new_state = pack(read_index_, false);
            const std::uint8_t old_state = state_.exchange(new_state, std::memory_order_acq_rel);
            read_index_ = index_of(old_state);
        }
        return slots_[static_cast<std::size_t>(read_index_)];
    }

private:
    static std::uint8_t pack(int idx, bool dirty) {
        return static_cast<std::uint8_t>((idx & 0x3) | (dirty ? 0x4 : 0));
    }
    static int index_of(std::uint8_t s) { return s & 0x3; }
    static bool is_dirty(std::uint8_t s) { return (s & 0x4) != 0; }

    WorldSnapshot slots_[3];
    std::atomic<std::uint8_t> state_{0};
    int write_index_ = 1;
    int read_index_ = 2;
};

} // namespace ps
