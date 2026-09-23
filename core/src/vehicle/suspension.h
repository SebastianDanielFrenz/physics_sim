// core/src/vehicle/suspension.h — spring/damper/bump-stop/ARB force curves
// (PLAN.md 5.1). Pure, stateless functions of the SuspensionDesc and this
// substep's measured travel state - the travel state itself (x, xdot) is
// owned by wheel_instance.h's WheelState, produced by a cylinder shape cast
// each substep (out/handover_p3.md decision 1), not integrated here.

#pragma once

#include "ps/vehicle/vehicle_desc.h"

namespace ps::vehicle {

// Force along SuspensionDesc::axis (world), i.e. the magnitude pushing the
// chassis attachment point AWAY from the wheel (toward full extension is
// negative travel direction - see vehicle_desc.h's SuspensionDesc comment
// for the axis convention). Always clamped to >= 0: a spring/damper/bump
// stop cannot pull the chassis down onto the wheel in this simplified
// (kinematic-wheel-position) model.
[[nodiscard]] inline real spring_force(const SuspensionDesc& d, real x) {
    const real f = d.spring_curve.x.empty() ? d.spring_rate * x : d.spring_curve.evaluate(x);
    return f + d.spring_preload;
}

[[nodiscard]] inline real damper_force(const SuspensionDesc& d, real xdot) {
    if (!d.damper_curve.x.empty()) return d.damper_curve.evaluate(xdot);
    return (xdot >= 0.0 ? d.damper_bump_rate : d.damper_rebound_rate) * xdot;
}

// Progressive bump stop engaging over the last 10% of travel (a documented
// simplification of a real progressive-rate bump rubber - see
// out/handover_p3.md's deferred list).
[[nodiscard]] inline real bump_stop_force(const SuspensionDesc& d, real x) {
    const real engage = real(0.9) * d.travel_max;
    if (x <= engage) return 0.0;
    return d.bump_stop_rate * (x - engage);
}

// Anti-roll bar: extra force from this wheel's compression differing from
// its ARB partner's (vehicle_desc.h's WheelDesc::arb_partner).
[[nodiscard]] inline real arb_force(const SuspensionDesc& d, real x_self, real x_partner) {
    return d.arb_rate * (x_self - x_partner);
}

[[nodiscard]] inline real total_suspension_force(const SuspensionDesc& d, real x, real xdot, real x_partner) {
    real f = spring_force(d, x) + damper_force(d, xdot) + bump_stop_force(d, x) + arb_force(d, x, x_partner);
    if (f < 0.0) f = 0.0; // PLAN.md section 6 seam note in this file's header: no pull-down in this model
    return f;
}

} // namespace ps::vehicle
