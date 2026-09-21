// ps/world/force_element.h — IForceElement (PLAN.md section 5).
//
// Tyres, springs, dampers, aero elements, hitch compliance, soil reactions
// are all force elements (later phases; P1 only ships the interface plus a
// couple of generic test elements under tests/). evaluate() must be const
// with respect to any shared state and must only read ctx - it is called
// concurrently from many worker threads during World's force-element stage
// (PLAN.md D8: scatter -> compute -> ordered gather).

#pragma once

#include "ps/math/pose.h"
#include "ps/world/frame.h"

#include <cstdint>

namespace ps {

// Immutable snapshot of tick-start state a force element may read. P1 keeps
// this minimal; vehicle/aero-specific context arrives in later phases
// without changing the interface below.
struct ForceElementContext {
    Pose frame_pose;
    Motion frame_motion;
    real dt = 0.0;
    std::uint64_t tick = 0;
};

class IForceElement {
public:
    virtual ~IForceElement() = default;

    // The frame this element's wrench is applied to; also determines which
    // body the World's gather stage applies it to.
    [[nodiscard]] virtual const Frame& frame() const = 0;

    // Must be safe to call concurrently with evaluate() on other elements
    // from other threads; must not touch World/backend state directly (use
    // only what ctx provides).
    [[nodiscard]] virtual Wrench evaluate(const ForceElementContext& ctx) const = 0;
};

} // namespace ps
