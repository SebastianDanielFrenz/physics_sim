// adapters/godot/src/triple_buffer.h — generic single-writer/single-reader
// lock-free triple buffer, same algorithm as ps::SnapshotTripleBuffer
// (core/include/ps/world/snapshot.h) but templated so the adapter can
// publish its OWN per-tick snapshot data without touching core/.
//
// Why the adapter needs its own: ps::WorldSnapshot (the core's own triple
// buffer payload) only carries rigid-body poses (PLAN.md D10/P1.5a
// articulation links have no snapshot entry - see this adapter's final
// report for the requested core addition). Until core adds that,
// PsSimulation's SimThread publishes articulation link poses into one of
// these every tick, computed right after ps::World::step() returns (i.e.
// on the sim thread, at the one moment nothing else is concurrently
// touching the backend) - safe without any lock, exactly like the core's
// own snapshot mechanism, and with no core change needed.

#pragma once

#include <atomic>
#include <cstdint>

namespace ps_godot {

template <typename T>
class TripleBuffer {
public:
    TripleBuffer() {
        state_.store(pack(0, false), std::memory_order_relaxed);
        write_index_ = 1;
        read_index_ = 2;
    }

    T& write_slot() { return slots_[static_cast<std::size_t>(write_index_)]; }

    void publish() {
        const std::uint8_t new_state = pack(write_index_, true);
        const std::uint8_t old_state = state_.exchange(new_state, std::memory_order_acq_rel);
        write_index_ = index_of(old_state);
    }

    const T& read() {
        const std::uint8_t s = state_.load(std::memory_order_acquire);
        if (is_dirty(s)) {
            const std::uint8_t new_state = pack(read_index_, false);
            const std::uint8_t old_state = state_.exchange(new_state, std::memory_order_acq_rel);
            read_index_ = index_of(old_state);
        }
        return slots_[static_cast<std::size_t>(read_index_)];
    }

private:
    static std::uint8_t pack(int idx, bool dirty) { return static_cast<std::uint8_t>((idx & 0x3) | (dirty ? 0x4 : 0)); }
    static int index_of(std::uint8_t s) { return s & 0x3; }
    static bool is_dirty(std::uint8_t s) { return (s & 0x4) != 0; }

    T slots_[3]{};
    std::atomic<std::uint8_t> state_{0};
    int write_index_ = 1;
    int read_index_ = 2;
};

} // namespace ps_godot
