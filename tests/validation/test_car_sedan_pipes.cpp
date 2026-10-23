// ps_audio A11 (owner ruling 2026-09-29, "can I drive the same engine that
// drives the audio?"): tests/support/fixtures/vehicles/car_sedan_pipes.json is the first
// vehicle whose REAL physics engine is a pipe-network `"kind": "simulated"`
// engine with NO map override (data/engines/demo_i4_pipes.json's "cycle"
// block generates VE/eta_i/friction maps at load time, I7c C16a) - every
// other simulated-engine vehicle so far (car_sedan_sim.json,
// car_sedan_gen.json) either ships measured maps directly or generates them
// from a cycle block that was fitted/cited against real dyno data.
// demo_i4_pipes.json is [AS] (assumed, unmeasured) throughout - this test
// only checks the car DRIVES, not that it drives like any real car:
//   - idles without stalling and SETTLES at idle_rpm. idle_pipes task
//     (2026-09-29, root-caused with a per-tick rpm/p_im/throttle_eff/
//     idle-integrator trace; see data/engines/demo_i4_pipes.json's "source"
//     for the writeup). The original hunt (~1040-1300 rpm, ~3 s period) had
//     four causes, all in the [AS] data file, none in code:
//     (1) grid_rpm started at 1000, above idle_rpm 800 - every idle lookup
//         used Table2D's edge-clamped 1000 rpm row; grid_rpm now starts at
//         800 (250/500 rpm nodes do not settle in the generator);
//     (2) the closed-throttle leak (area fraction at x=0) was 0.01: even a
//         closed throttle gave +46.99 N*m net at 800 rpm, no idle
//         equilibrium; now 0.002 (net(0) = -11.93 N*m);
//     (3) the idle point (~29.5 kPa) sat on the 30 kPa lower edge of
//         grid_p_im_kpa, where Table2D's slopes drop to 0 - a 25 kPa node
//         was added (20 kPa does not settle at 800 rpm in the generator);
//     (4) the throttle area curve rose in one 0 -> 0.5 segment (plant gain
//         ~4x a real body near closed) and the idle PI gains were
//         kp 1e-3 / ki 4e-3, ~33x/67x the A25A sim engine's (3e-5 / 6e-5)
//         on the same 3.0 L manifold and similar crank inertia; both now
//         follow toyota_a25a_fks_sim.json (its curve shape with this file's
//         own 0.002 leak, and its idle_control) - taken as a working
//         precedent, not tuned to this test.
//     Discriminating runs (20 s idle, rpm range over the last 2 s): (1)+(2)
//     only 759-1301; +(3) only 749-1299; +(4) only 772-827 (+-30, a slow
//     ~2.5 s swing still decaying at 20 s); all four 800.0-800.0.
//     car_sedan_sim.json in the same harness settles to 800.0 in ~4 s.
//   - a neutral (no gear engaged) full-throttle free rev reaches the
//     engine's own fuel-cut limiter band and falls back below it on
//     throttle lift, mirroring tests/unit/test_engine_rig_audio.cpp's R1
//     (there on a wheel-less EngineRig; here through the real vehicle/
//     World path with wheels, gearbox, clutch);
//   - a full-throttle 1st-gear launch from settled rest makes real forward
//     progress without stalling.
// Run for car_sedan_pipes.json (FWD 60/40) and car_sedan_pipes_rwd.json
// (owner 2026-09-29, "give me RWD": car_sedan_rwd.json, same engine) - idle
// and free rev are identical (neutral), launch speed@3s 12.21 / 13.28 m/s.
#include "vehicle_test_helpers.h"

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/drivetrain/powertrain_state.h"
#include "ps/io/vehicle_io.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

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

ps_test::VehicleTestRig build_rig(const std::string& path) {
    return ps_test::build_vehicle_rig(path, kMass,
                                       kHalfExtents, kOriginHeight, /*slope_grade=*/0.0, /*job_workers=*/4);
}

} // namespace

TEST_CASE("car_sedan_pipes.json: idles, free-revs to the limiter and back, launches without stalling",
          "[vehicle][drivetrain][engine][cycle][long]") {   // 2 x ~45 s in release (map generation + 3 sim phases): [long] per CLAUDE.md
    // car_sedan_pipes_rwd.json (owner 2026-09-29, "give me RWD"): car_sedan_rwd.json with the same engine.
    const std::string name = GENERATE(std::string("car_sedan_pipes"), std::string("car_sedan_pipes_rwd"));
    CAPTURE(name);
    const std::string path = std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/vehicles/" + name + ".json";
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

    ps_test::VehicleTestRig rig = build_rig(path);
    World& w = *rig.world;
    bool stalled = false;
    auto check = [&]() {
        if (w.powertrain_state(rig.vehicle).engine_state != EngineState::Running) stalled = true;
    };

    // Settle at idle (default initial_state = Running, no starter needed):
    // 10 s, then the rpm range and mean over the last 2 s.
    constexpr int kIdleTicks = 2400, kWindowStart = 1920; // 10 s / last 2 s @ 240 Hz
    real idle_rpm_min = 1e9, idle_rpm_max = -1e9, idle_rpm_sum = 0.0;
    int n_window = 0;
    real last_off_band_t = 0.0; // last time |rpm - idle_rpm| > 10 rpm
    for (int t = 0; t < kIdleTicks; ++t) {
        w.step();
        check();
        const real r = w.powertrain_state(rig.vehicle).engine_rpm;
        if (std::abs(r - idle_rpm_desc) > 10.0) last_off_band_t = (t + 1) / 240.0;
        if (t < kWindowStart) continue;
        idle_rpm_min = std::min(idle_rpm_min, r);
        idle_rpm_max = std::max(idle_rpm_max, r);
        idle_rpm_sum += r;
        ++n_window;
    }
    const real idle_rpm_mean = idle_rpm_sum / n_window;
    const auto idle = w.powertrain_state(rig.vehicle);
    const real idle_rpm = idle.engine_rpm;

    // Neutral free rev to the limiter and back (no gear_request issued -
    // the manual_tcu default gear is Neutral).
    w.set_control("throttle", 1.0);
    real max_rpm = 0.0;
    bool limiter_cut = false; // the fuel cut fired at least once (EngineSoundState::limiter_active)
    for (int t = 0; t < 720; ++t) { // 3 s
        w.step();
        check();
        const auto& ps = w.powertrain_state(rig.vehicle);
        max_rpm = std::max(max_rpm, ps.engine_rpm);
        if (!ps.engines.empty() && ps.engines[0].limiter_active) limiter_cut = true;
    }
    w.set_control("throttle", 0.0);
    real lift_min_rpm = 1e9; // lift-off to idle, neutral: must not dip below idle (owner 2026-09-29)
    for (int t = 0; t < 240; ++t) {
        w.step();
        check();
        lift_min_rpm = std::min(lift_min_rpm, w.powertrain_state(rig.vehicle).engine_rpm);
    } // 1 s
    const real rpm_after_lift = w.powertrain_state(rig.vehicle).engine_rpm;

    std::printf("[%s idle] last 2 s of 10 s: rpm [%.3f, %.3f] mean %.3f (desc idle_rpm %.0f), "
                "within +-10 rpm from t = %.3f s, final %.3f\n",
                name.c_str(), idle_rpm_min, idle_rpm_max, idle_rpm_mean, idle_rpm_desc, last_off_band_t, idle_rpm);
    std::printf("[%s free rev] free-rev max %.1f rpm (limiter %.0f + hysteresis), 1 s after lift %.1f "
                "rpm\n",
                name.c_str(), max_rpm, limiter_rpm_desc, rpm_after_lift);
    CHECK_FALSE(stalled);
    // Settled idle (see the header comment). Measured: last 2 s range
    // 0.653 rpm, mean 800.054, inside +-10 rpm from 3.054 s. Sabotages (the
    // header's discriminating runs): without the 25 kPa node the range is
    // ~550 rpm, without the A25A throttle curve/idle gains ~55 rpm.
    CHECK(idle_rpm_max - idle_rpm_min < 2.0);
    CHECK(std::abs(idle_rpm_mean - idle_rpm_desc) < 2.0);
    CHECK(last_off_band_t < 5.0);
    // Reached the fuel-cut band. 2026-10-19 (burn law F, vault PHYS-079): was max_rpm >= limiter_rpm_desc on the
    // TICK-END rpm, which samples the crank once per 4 substeps: the cut fires on the substep that crosses 6800 rpm and
    // the crank is back at 6797.96 rpm by the tick end (rise ~7400 rpm/s = ~31 rpm per tick just below the cut; trace:
    // hidden "car_sedan_pipes diag: free-rev approach to the limiter" - the engine cycles 6650-6800 rpm on the cut with
    // limiter_active set, net WOT torque at 6790 rpm 124.9 N*m on the dyno). Under law F the crossing falls early in a
    // tick, so no tick-end sample reaches 6800 (max 6798.0); the cut's own flag is the criterion, the tick-end max
    // must lie within one tick's rise (40 rpm [D]) below the cut. Sabotage (2026-10-19): free rev 0.4 s instead of 3 s
    // (tick-end max 4828 rpm) - both checks fail.
    CHECK(limiter_cut);
    CHECK(max_rpm >= limiter_rpm_desc - 40.0);
    CHECK(rpm_after_lift < limiter_rpm_desc - hysteresis_rpm_desc); // fell back below the restore point

    // Settle back to idle in neutral before the launch (the free rev above
    // left the crank spinning down); the lift-off minimum spans all of it.
    for (int t = 0; t < 1440; ++t) { // 6 s
        w.step();
        check();
        lift_min_rpm = std::min(lift_min_rpm, w.powertrain_state(rig.vehicle).engine_rpm);
    }
    const real rpm_settled = w.powertrain_state(rig.vehicle).engine_rpm;
    std::printf("[%s lift] min rpm over 7 s after the lift %.3f, final %.3f\n", name.c_str(), lift_min_rpm, rpm_settled);
    // SimulatedEngine idle rework (owner 2026-09-29: "dips below 800 rpms
    // multiple times ... That should not happen"): the DFCO dashpot, the
    // manifold-fill term and the one-sided pedal-released PI. Before it the
    // car_sedan_pipes lift from WOT (in a World scenario) dipped to 539 rpm.
    CHECK(lift_min_rpm > idle_rpm_desc - 2.0);
    CHECK(std::abs(rpm_settled - idle_rpm_desc) < 2.0);

    // WOT launch in 1st from settled rest.
    w.set_control("throttle", 1.0);
    w.set_control("gear_request", 1.0);
    for (int t = 0; t < 720; ++t) { w.step(); check(); } // 3 s
    const real speed_3s = w.get_motion(rig.chassis).linear.length();
    const auto launch = w.powertrain_state(rig.vehicle);

    std::printf("[%s launch] gear %d, engine %.0f rpm, speed@3s %.4f m/s\n",
                name.c_str(), static_cast<int>(launch.gear), launch.engine_rpm, speed_3s);
    CHECK_FALSE(stalled);
    CHECK(static_cast<int>(launch.gear) == 1);
    CHECK(speed_3s > 2.0); // real forward progress; see the printed measured value for margin
}

// Burn law phase 2b (2026-10-19): the neutral WOT free rev's approach to the 6800 rpm fuel cut, per tick above
// 6000 rpm - rpm, its rate, the post-limiter combustion fraction, plate, manifold pressure, lambda, applied spark,
// fuel flow, the engine's torque demand and the limiter flag. Hidden; prints only.
TEST_CASE("car_sedan_pipes diag: free-rev approach to the limiter", "[.][diag][vehicle][drivetrain][cycle]") {
    const std::string path = std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/vehicles/car_sedan_pipes.json";
    ps_test::VehicleTestRig rig = build_rig(path);
    World& w = *rig.world;
    for (int t = 0; t < 1200; ++t) w.step(); // 5 s idle
    w.set_control("throttle", 1.0);
    real prev = w.powertrain_state(rig.vehicle).engine_rpm;
    std::printf("    t_s      rpm  drpm/s  load  plate  p_im_kPa  lambda  spark  fuel_g_s  T_dem  lim\n");
    for (int t = 0; t < 1440; ++t) { // 6 s
        w.step();
        const auto& ps = w.powertrain_state(rig.vehicle);
        const real rpm = ps.engine_rpm;
        if (rpm > 6000.0 && !ps.engines.empty() && (rpm > 6700.0 || t % 4 == 0)) {
            const auto& e = ps.engines[0];
            std::printf("%7.3f %8.2f %7.1f %5.3f %6.3f %9.2f %7.3f %6.2f %9.3f %6.2f %d\n", (t + 1) / 240.0, rpm,
                        (rpm - prev) * 240.0, e.load, e.throttle_position, e.p_intake_pa / 1000.0, e.lambda, e.spark_advance_deg,
                        e.fuel_flow_g_s, w.powertrain_debug(rig.vehicle).engine_torque_demand, e.limiter_active ? 1 : 0);
        }
        prev = rpm;
    }
}

