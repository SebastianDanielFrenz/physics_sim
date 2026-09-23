// Validation: step-steer response is stable, with no growing oscillation
// (PLAN.md P3 acceptance).
//
// Drives straight, then applies a step steer input and watches the yaw
// rate. Stability is checked as: the oscillation amplitude (peak-to-peak
// yaw rate) in a LATER time window must not exceed the amplitude in an
// EARLIER window by more than a small margin - a growing oscillation
// (the failure mode this test exists to catch) shows up as the later
// window's amplitude clearly exceeding the earlier one's, whereas a
// stable, settling (or steady-state circular) response does not.

#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace ps;

namespace {
real amplitude(const std::vector<real>& v) {
    if (v.empty()) return 0.0;
    real mn = v.front(), mx = v.front();
    for (real x : v) { mn = std::min(mn, x); mx = std::max(mx, x); }
    return mx - mn;
}
} // namespace

TEST_CASE("Step-steer yaw-rate response does not grow", "[validation][vehicle][long]") {
    auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0,
                                           Vec3{2.0, 0.4, 0.15}, 0.6);
    ps_test::settle(*rig.world, 0.5);
    rig.world->backend().set_motion(rig.chassis, Motion{Vec3{20.0, 0.0, 0.0}, Vec3::zero()});
    ps_test::settle(*rig.world, 0.5); // drive straight briefly before the step

    rig.world->set_control("steer", 0.25); // the step

    std::vector<real> window_early, window_late;
    const real dt = rig.world->dt();
    const std::uint64_t n = static_cast<std::uint64_t>(3.0 / dt + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) {
        rig.world->step();
        const real t = static_cast<real>(i) * dt;
        const real yaw_rate = rig.world->get_motion(rig.chassis).angular.z;
        REQUIRE(std::isfinite(yaw_rate));
        REQUIRE(std::abs(yaw_rate) < 5.0); // sanity bound - a divergent instability would blow past this quickly
        if (t >= 0.2 && t < 1.0) window_early.push_back(yaw_rate);
        if (t >= 2.2 && t < 3.0) window_late.push_back(yaw_rate);
    }

    const real amp_early = amplitude(window_early);
    const real amp_late = amplitude(window_late);
    INFO("amp_early=" << amp_early << " rad/s, amp_late=" << amp_late << " rad/s");
    CHECK(amp_late <= amp_early * 1.1 + 1e-4);
}
