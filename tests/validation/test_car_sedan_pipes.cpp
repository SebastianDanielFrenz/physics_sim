// ps_audio A11 (owner ruling 2026-09-29, "can I drive the same engine that
// drives the audio?"): data/vehicles/car_sedan_pipes.json is the first
// vehicle whose REAL physics engine is a pipe-network `"kind": "simulated"`
// engine with NO map override (data/engines/demo_i4_pipes.json's "cycle"
// block generates VE/eta_i/friction maps at load time, I7c C16a) - every
// other simulated-engine vehicle so far (car_sedan_sim.json,
// car_sedan_gen.json) either ships measured maps directly or generates them
// from a cycle block that was fitted/cited against real dyno data.
// demo_i4_pipes.json is [AS] (assumed, unmeasured) throughout - this test
// only checks the car DRIVES, not that it drives like any real car:
//   - idles without stalling (engine stays Running, within a measured,
//     tightened band around idle_rpm). idle_pipes task (2026-09-29,
//     root-caused via a per-tick trace, see data/engines/demo_i4_pipes.json's
//     own "source"/"cycle.source" fields for the full writeup): the ORIGINAL
//     hunt (~1040-1280 rpm, anchored on dfco_reenable_rpm/dfco_enable_rpm)
//     had two combined causes, both fixed here - (1) the generated map grid
//     did not cover idle_rpm (800, grid_rpm started at 1000), so the idle
//     feed-forward bisection and every low-rpm lookup used Table2D's
//     edge-clamped 1000 rpm row, physically wrong for 800 rpm; grid_rpm now
//     includes 800 (the lowest node the offline generator could actually
//     converge at - 250/500 rpm were tried and are structurally unreachable/
//     not-settled at this engine's low-rpm plenum targets, independent of
//     cause (2)); (2) even with 800 rpm covered, the closed-throttle minimum
//     leak area (throttle_body.area_fraction_vs_position at x=0) was too
//     large - even fully closed throttle produced positive net torque at
//     800 rpm (measured net(0) = +46.99 Nm), so no idle equilibrium existed
//     at all; reduced 0.01 -> 0.002, which gives net(0) = -11.93 Nm and a
//     valid idle_ff solution (idle_p_im ~= 29.5 kPa, close to the 30 kPa
//     grid node). KNOWN LIMITATION, still present and NOT chased further
//     (confirmed independent of DFCO - disabling it made the residual
//     oscillation worse - and independent of simple PI gain scaling - a 10x
//     gain cut left a similar-magnitude oscillation - so this is a
//     structural idle-loop issue, consistent with CLAUDE.md's documented
//     "idle PI's own d(theta_idle)/d(omega) is not in the implicit solve's
//     d" gap, not something to force-tune here): the idle PI still hunts in
//     a bounded, non-growing limit cycle, now roughly 730-1300 rpm over a
//     2 s settle window (steady-state, confirmed sustained not decaying out
//     to 20 s: ~759-1301 rpm). The low end now sits much closer to the true
//     800 rpm target than before. The car is still fully drivable (see the
//     launch case below);
//   - a neutral (no gear engaged) full-throttle free rev reaches the
//     engine's own fuel-cut limiter band and falls back below it on
//     throttle lift, mirroring tests/unit/test_engine_rig_audio.cpp's R1
//     (there on a wheel-less EngineRig; here through the real vehicle/
//     World path with wheels, gearbox, clutch);
//   - a full-throttle 1st-gear launch from settled rest makes real forward
//     progress without stalling.
#include "vehicle_test_helpers.h"

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/drivetrain/powertrain_state.h"
#include "ps/io/vehicle_io.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <string>

using namespace ps;
using ps::drivetrain::EngineState;
using ps::drivetrain::SimulatedEngineDesc;

namespace {

constexpr real kMass = 1500.0;
const Vec3 kHalfExtents{2.0, 0.4, 0.15};
constexpr real kOriginHeight = 0.6;

ps_test::VehicleTestRig build_rig() {
    return ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan_pipes.json", kMass,
                                       kHalfExtents, kOriginHeight, /*slope_grade=*/0.0, /*job_workers=*/4);
}

} // namespace

TEST_CASE("car_sedan_pipes.json: idles, free-revs to the limiter and back, launches without stalling",
          "[vehicle][drivetrain][engine][cycle]") {
    const std::string path = std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan_pipes.json";
    const vehicle::VehicleDesc desc = io::load_vehicle_json(path);
    real idle_rpm_desc = -1.0, limiter_rpm_desc = -1.0, hysteresis_rpm_desc = -1.0;
    for (const auto& c : desc.powertrain.components) {
        if (const auto* s = std::get_if<SimulatedEngineDesc>(&c.params)) {
            idle_rpm_desc = s->idle_rpm;
            limiter_rpm_desc = s->limiter.rpm;
            hysteresis_rpm_desc = s->limiter.hysteresis_rpm;
        }
    }
    REQUIRE(idle_rpm_desc > 0.0);   // confirms the engine component IS a simulated (pipe) one
    REQUIRE(limiter_rpm_desc > 0.0);

    ps_test::VehicleTestRig rig = build_rig();
    World& w = *rig.world;
    bool stalled = false;
    auto check = [&]() {
        if (w.powertrain_state(rig.vehicle).engine_state != EngineState::Running) stalled = true;
    };

    // Settle at idle (default initial_state = Running, no starter needed).
    // See this file's header comment: the pipe/cycle engine's idle PI does
    // not converge to idle_rpm here, it hunts in a bounded limit cycle -
    // track the min/max seen instead of asserting convergence.
    real idle_rpm_min = 1e9, idle_rpm_max = -1e9;
    for (int t = 0; t < 480; ++t) {
        w.step();
        check();
        const real r = w.powertrain_state(rig.vehicle).engine_rpm;
        idle_rpm_min = std::min(idle_rpm_min, r);
        idle_rpm_max = std::max(idle_rpm_max, r);
    } // 2 s @ 240 Hz
    const auto idle = w.powertrain_state(rig.vehicle);
    const real idle_rpm = idle.engine_rpm;

    // Neutral free rev to the limiter and back (no gear_request issued -
    // the manual_tcu default gear is Neutral).
    w.set_control("throttle", 1.0);
    real max_rpm = 0.0;
    for (int t = 0; t < 720; ++t) { // 3 s
        w.step();
        check();
        max_rpm = std::max(max_rpm, w.powertrain_state(rig.vehicle).engine_rpm);
    }
    w.set_control("throttle", 0.0);
    for (int t = 0; t < 240; ++t) { w.step(); check(); } // 1 s
    const real rpm_after_lift = w.powertrain_state(rig.vehicle).engine_rpm;

    std::printf("[car_sedan_pipes idle] rpm range over 2 s settle: [%.1f, %.1f] (desc idle_rpm %.0f - see header "
                "comment on the idle hunt), final %.1f\n",
                idle_rpm_min, idle_rpm_max, idle_rpm_desc, idle_rpm);
    std::printf("[car_sedan_pipes free rev] free-rev max %.1f rpm (limiter %.0f + hysteresis), 1 s after lift %.1f "
                "rpm\n",
                max_rpm, limiter_rpm_desc, rpm_after_lift);
    CHECK_FALSE(stalled);
    // The idle PI still hunts (see header comment) rather than converging to
    // idle_rpm, but the idle_pipes fix (grid_rpm now covers 800, the
    // closed-throttle leak area no longer forces positive torque at idle)
    // moved the band much closer to idle_rpm (800) than the pre-fix
    // ~1040-1280 rpm. Measured over this 2 s settle window: min ~731.7 rpm,
    // max ~1300.2 rpm; sustained steady-state (20 s trace) ~759-1301 rpm.
    // Margin: min must clear stall_rpm (400) by a wide margin and land
    // clearly below the old floor (1000); max must stay clear of the
    // dfco_enable_rpm/limiter and land clearly below the old loose 2000
    // bound. Sabotage (pre-fix grid_rpm/leak area): idle_rpm_min measured
    // ~1003-1040, which fails idle_rpm_min < 900.0 below.
    CHECK(idle_rpm_min > 600.0);
    CHECK(idle_rpm_min < 900.0);
    CHECK(idle_rpm_max < 1450.0);
    CHECK(max_rpm >= limiter_rpm_desc); // reached the fuel-cut band
    CHECK(rpm_after_lift < limiter_rpm_desc - hysteresis_rpm_desc); // fell back below the restore point

    // Settle back to idle in neutral before the launch (the free rev above
    // left the crank spinning down).
    for (int t = 0; t < 480; ++t) { w.step(); check(); } // 2 s

    // WOT launch in 1st from settled rest.
    w.set_control("throttle", 1.0);
    w.set_control("gear_request", 1.0);
    for (int t = 0; t < 720; ++t) { w.step(); check(); } // 3 s
    const real speed_3s = w.get_motion(rig.chassis).linear.length();
    const auto launch = w.powertrain_state(rig.vehicle);

    std::printf("[car_sedan_pipes launch] gear %d, engine %.0f rpm, speed@3s %.4f m/s\n",
                static_cast<int>(launch.gear), launch.engine_rpm, speed_3s);
    CHECK_FALSE(stalled);
    CHECK(static_cast<int>(launch.gear) == 1);
    CHECK(speed_3s > 2.0); // real forward progress; see the printed measured value for margin
}

