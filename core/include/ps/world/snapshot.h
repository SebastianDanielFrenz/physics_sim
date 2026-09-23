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

#include "ps/backend/i_rigid_backend.h" // ContactEvent - plain struct, no Jolt/Godot type (PLAN.md D2)
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

// PLAN.md D10/P1.5a + P2 adapter report / this task's step 3: one live
// articulation's pose/motion, additive to WorldSnapshot. `links[0]` is
// always the root (link_index == -1 in World::joint_position et al. -
// meaningful pose/motion even for a fixed, non-floating root, since
// Instance::link_world_pose(-1) returns the articulation's own held root
// pose either way); `links[1 + i]` is public link index `i`
// (ArticulationDesc::links[i]), in the same order World's own
// joint_position(id, link_index)/link_frame(id, link_index) address them -
// an adapter never needs to know the internal body numbering the solver
// uses (PLAN.md D10's own scope: reduced-coordinate internals stay
// internal).
struct ArticulationLinkSnapshot {
    int link_index = -1; // -1 == root, else the ArticulationDesc::links index
    Pose pose;
    Motion motion;
};

struct ArticulationSnapshot {
    ArticulationId id;
    std::vector<ArticulationLinkSnapshot> links; // root first, then link 0, 1, 2, ...
};

struct WorldSnapshot {
    std::uint64_t tick = 0;
    real sim_time = 0.0;
    std::vector<BodySnapshot> bodies;

    // PLAN.md D10/P1.5a + P2 adapter report, this task's step 3: every live
    // articulation's link poses, filled by World::stage_snapshot() from
    // articulation::System - additive; a World with no articulations
    // publishes an empty vector here exactly like it always has for
    // `bodies` (an articulation's own Jolt-backed collision proxies, if
    // any - PLAN.md P1.5b's ProxyBody strategy - already appear in `bodies`
    // like any other body; this is only the reduced-coordinate link poses
    // themselves, which have no IRigidBackend body at all).
    std::vector<ArticulationSnapshot> articulations;

    // PLAN.md P2 adapter report, this task's step 3: a RACE-FREE copy of
    // this tick's contact events, filled by World::stage_snapshot() from
    // World::last_contact_events() (itself filled earlier in the same
    // tick's pipeline, stage_backend_step - see world.cpp). World::
    // last_contact_events() returns a live reference that the NEXT tick's
    // stage_backend_step overwrites in place; a reader on another thread
    // (PLAN.md D3: "the adapter runs the sim on its own thread") calling
    // that accessor between ticks would be racing the sim thread's next
    // drain_contact_events_into call with no synchronisation at all. This
    // field instead travels through the same SnapshotTripleBuffer::publish
    // hand-off every other field here does, so a reader only ever sees a
    // complete, stable, already-published tick's contacts.
    std::vector<ContactEvent> contacts;

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
