// tests/support/counter_force_element.h — a minimal IForceElement used by
// the determinism and bench test suites.
//
// Produces a wrench purely as a function of (element id, tick) through a
// counter-based hash (PLAN.md D8: "randomness only through counter-based
// generators keyed by (entity ID, tick, stream)") - never a stateful RNG,
// so the exact same sequence of wrenches is produced regardless of which
// worker evaluates which element or in what order, letting a determinism
// test isolate scheduling as the only possible source of divergence.

#pragma once

#include "ps/world/force_element.h"

#include <cmath>
#include <cstdint>

namespace ps_test {

class CounterForceElement final : public ps::IForceElement {
public:
    CounterForceElement(ps::Frame frame, std::uint64_t id, double magnitude = 2.0)
        : frame_(frame), id_(id), magnitude_(magnitude) {}

    [[nodiscard]] const ps::Frame& frame() const override { return frame_; }

    [[nodiscard]] ps::Wrench evaluate(const ps::ForceElementContext& ctx) const override {
        // Splitmix64-style counter hash of (id, tick) - deterministic,
        // stateless, no shared mutable RNG.
        auto hash64 = [](std::uint64_t x) {
            x += 0x9E3779B97F4A7C15ULL;
            x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
            x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
            return x ^ (x >> 31);
        };
        const std::uint64_t h1 = hash64(id_ * 0x100000001B3ULL ^ ctx.tick);
        const std::uint64_t h2 = hash64(h1);
        const std::uint64_t h3 = hash64(h2);
        auto to_unit = [](std::uint64_t h) {
            return (static_cast<double>(h >> 11) / static_cast<double>(1ULL << 53)) * 2.0 - 1.0; // [-1, 1)
        };
        ps::Wrench w;
        w.force = ps::Vec3{to_unit(h1), to_unit(h2), to_unit(h3)} * magnitude_;
        w.torque = ps::Vec3::zero();
        return w;
    }

private:
    ps::Frame frame_;
    std::uint64_t id_;
    double magnitude_;
};

} // namespace ps_test
