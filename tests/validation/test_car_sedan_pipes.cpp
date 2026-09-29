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
//   - idles without stalling (engine stays Running, never leaves a broad
//     plausible idle band). KNOWN LIMITATION (measured, not chased further
//     - out of A11's scope): unlike car_sedan_sim.json's MVEM engine, this
//     pipe/cycle-model engine's idle PI does not converge to idle_rpm
//     (800) - it settles into a sustained ~3 s limit cycle roughly
//     1040-1280 rpm (confirmed out to 10 s of settle time, not a slow
//     transient). Likely the idle PI's gains, tuned against the smooth
//     MVEM torque tables, are too aggressive for the C16a-generated
//     pipe-engine maps' coarser grid near idle; a generator/PI tuning
//     issue, not a stall or an instability that grows. The car is still
//     drivable (see the launch case below);
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
    // The idle PI hunts (see header comment) rather than converging to
    // idle_rpm - bound it to a plausible band well clear of stall_rpm (400)
    // and the limiter (6800), not a tight match to the setpoint.
    CHECK(idle_rpm_min > 500.0);
    CHECK(idle_rpm_max < 2000.0);
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
