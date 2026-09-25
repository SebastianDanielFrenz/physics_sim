// tests/validation/test_powertrain_auto_clutch_shifts.cpp — auto-clutch
// behaviour around upshifts on the real car (data/vehicles/car_sedan.json,
// auto-clutch/auto-blip ON, the gamepad defaults). Owner test drive
// 2026-09-25: "a lot of wheel spin in 2nd", plus two engagement bugs found
// while diagnosing it:
//   - selecting 2nd at walking pace stalled the engine and left the car
//     coasting with the throttle pressed (the Shift-mode reclose was
//     rpm-blind and dragged the engine down to a ~350 rpm input shaft);
//   - after an upshift the clutch could sit partially engaged for seconds.
// Every case drives the whole vehicle through World's public controls and
// reads World::powertrain_state/powertrain_debug/wheel_telemetry per tick.

#include "vehicle_test_helpers.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>

using namespace ps;

namespace {

constexpr real kPi = 3.14159265358979323846;
real rad_s_to_rpm(real w) { return w * real(60.0) / (real(2.0) * kPi); }

ps_test::VehicleTestRig make_sedan() {
    return ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0, Vec3{2.0, 0.4, 0.15}, 0.6,
                                      0.0, 4, Vec3::unit_y(), 240.0, 960.0, 3000.0);
}

real speed(const ps_test::VehicleTestRig& rig) { return rig.world->get_motion(rig.chassis).linear.length(); }

// Steps until speed >= v_target (m/s) or timeout; returns true if reached.
bool drive_until_speed(ps_test::VehicleTestRig& rig, real v_target, real timeout_s) {
    const std::uint64_t n = static_cast<std::uint64_t>(timeout_s / rig.world->dt() + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) {
        rig.world->step();
        if (speed(rig) >= v_target) return true;
    }
    return false;
}

// Per-tick observations shared by every case.
struct Watch {
    bool stalled = false;
    real max_driven_abs_kappa = 0.0; // FL/FR (the driven front axle)
    real t_lock = -1.0;              // first time (s since the watch started) the clutch is stuck with |slip| < 80 rpm in the target gear
    int final_gear = 0;
};

void run_watch(ps_test::VehicleTestRig& rig, real seconds, int target_gear, Watch& w, const std::function<void(real)>& per_tick = {}) {
    const real dt = rig.world->dt();
    const std::uint64_t n = static_cast<std::uint64_t>(seconds / dt + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) {
        const real t = static_cast<real>(i + 1) * dt;
        rig.world->step();
        const auto& pt = rig.world->powertrain_state(rig.vehicle);
        if (pt.engine_state == drivetrain::EngineState::Stalled) w.stalled = true;
        for (std::size_t wi = 0; wi < 2; ++wi) {
            w.max_driven_abs_kappa = std::max(w.max_driven_abs_kappa, std::abs(rig.world->wheel_telemetry(rig.vehicle, wi).slip_ratio));
        }
        const auto dbg = rig.world->powertrain_debug(rig.vehicle);
        if (w.t_lock < 0.0 && pt.gear == target_gear && dbg.clutch_stuck && std::abs(rad_s_to_rpm(dbg.clutch_slip)) < 80.0) w.t_lock = t;
        w.final_gear = pt.gear;
        if (per_tick) per_tick(t);
    }
}

} // namespace

// Secondary robustness case (owner: 1->2 at 5 km/h is unrealistically low,
// but must never leave the car silently disconnected). Before the fix the
// engine stalled ~0.25 s after the request (rpm 391 -> 0) and the car
// coasted at -0.1 m/s^2 for the rest of the run, WOT included.
TEST_CASE("Auto-clutch: selecting 2nd at walking pace never stalls and pulls away on throttle", "[validation][vehicle][powertrain]") {
    auto rig = make_sedan();
    ps_test::settle(*rig.world, 1.0);
    rig.world->set_control("gear_request", 1.0);
    rig.world->set_control("throttle", 0.3);
    REQUIRE(drive_until_speed(rig, 5.0 / 3.6, 15.0));

    rig.world->set_control("gear_request", 2.0);
    rig.world->set_control("throttle", 0.25);
    Watch w;
    const real v_request = speed(rig);
    run_watch(rig, 4.0, 2, w);
    const real v_part = speed(rig);
    INFO("v_request_kmh=" << v_request * 3.6 << " v_after_part_throttle_kmh=" << v_part * 3.6);
    CHECK(w.final_gear == 2);
    CHECK_FALSE(w.stalled);
    CHECK(v_part > v_request); // still pulling (slowly) at part throttle, not coasting

    rig.world->set_control("throttle", 1.0);
    run_watch(rig, 3.0, 2, w);
    const real v_wot = speed(rig);
    INFO("v_after_3s_wot_kmh=" << v_wot * 3.6 << " t_lock=" << w.t_lock << " max|kappa|=" << w.max_driven_abs_kappa);
    CHECK_FALSE(w.stalled);
    CHECK(v_wot - v_part > 5.0); // > 1.67 m/s^2 mean over 3 s: actually driving, not disconnected
    CHECK(w.t_lock > 0.0);       // and the clutch closed during the WOT pull
}

// Primary case (owner): a part-throttle 1->2 upshift at a realistic ~15 km/h
// (2nd ~1050 rpm): the clutch must lock within a bounded time, the engine
// must not stall and the driven wheels must not spin.
TEST_CASE("Auto-clutch: part-throttle 1->2 upshift at 15 km/h locks promptly without wheelspin", "[validation][vehicle][powertrain]") {
    auto rig = make_sedan();
    ps_test::settle(*rig.world, 1.0);
    rig.world->set_control("gear_request", 1.0);
    rig.world->set_control("throttle", 0.6);
    REQUIRE(drive_until_speed(rig, 15.0 / 3.6, 15.0));
    REQUIRE(rig.world->powertrain_state(rig.vehicle).gear == 1);

    rig.world->set_control("throttle", 0.3);
    rig.world->set_control("gear_request", 2.0);
    Watch w;
    const real v_request = speed(rig);
    run_watch(rig, 3.0, 2, w);
    INFO("v_request_kmh=" << v_request * 3.6 << " t_lock=" << w.t_lock << " max|kappa|=" << w.max_driven_abs_kappa
                          << " v_end_kmh=" << speed(rig) * 3.6);
    CHECK(w.final_gear == 2);
    CHECK_FALSE(w.stalled);
    CHECK(w.t_lock > 0.0);
    CHECK(w.t_lock < 1.0);
    CHECK(w.max_driven_abs_kappa < 0.1);
    CHECK(speed(rig) > v_request);
}

// Holds the car near v_target (m/s) in the current gear with a simple
// proportional throttle law for `seconds`.
void hold_speed(ps_test::VehicleTestRig& rig, real v_target, real seconds) {
    const std::uint64_t n = static_cast<std::uint64_t>(seconds / rig.world->dt() + 0.5);
    for (std::uint64_t i = 0; i < n; ++i) {
        rig.world->set_control("throttle", std::clamp(real(0.15) + real(0.5) * (v_target - speed(rig)), real(0.0), real(1.0)));
        rig.world->step();
    }
}

// Task D (torque-following post-shift close, AutoClutchDesc::sync_time_s).
// Before it, a WOT 1->2 left the engine flaring to the limiter during the
// shift (the upshift cut ended at Selecting) and the fixed-rate reclose put
// the full 300 Nm clutch capacity plus the flywheel's slip energy into the
// front axle: FL/FR kappa ~1.0 for ~1 s right after the shift (probe
// wot23: 1->2 at 30 km/h WOT).
TEST_CASE("Auto-clutch: WOT 1->2 upshift from a non-spinning 1st keeps the driven wheels below peak slip", "[validation][vehicle][powertrain]") {
    auto rig = make_sedan();
    ps_test::settle(*rig.world, 1.0);
    rig.world->set_control("gear_request", 1.0);
    rig.world->set_control("throttle", 0.45);
    Watch w1;
    bool reached = false;
    run_watch(rig, 20.0, 1, w1, [&](real) { reached = reached || speed(rig) >= 35.0 / 3.6; });
    // (run_watch has no early exit; the pull to 35 km/h is well inside 20 s)
    INFO("1st-gear max|kappa|=" << w1.max_driven_abs_kappa);
    REQUIRE(reached);
    REQUIRE(w1.max_driven_abs_kappa < 0.2); // precondition: 1st gear itself was not spinning

    auto rig2 = make_sedan();
    ps_test::settle(*rig2.world, 1.0);
    rig2.world->set_control("gear_request", 1.0);
    rig2.world->set_control("throttle", 0.45);
    REQUIRE(drive_until_speed(rig2, 35.0 / 3.6, 20.0));
    rig2.world->set_control("gear_request", 2.0);
    rig2.world->set_control("throttle", 1.0);
    const real v_request = speed(rig2);
    Watch w;
    run_watch(rig2, 3.0, 2, w);
    INFO("v_request_kmh=" << v_request * 3.6 << " t_lock=" << w.t_lock << " max|kappa|=" << w.max_driven_abs_kappa
                          << " v_end_kmh=" << speed(rig2) * 3.6);
    CHECK(w.final_gear == 2);
    CHECK_FALSE(w.stalled);
    CHECK(w.max_driven_abs_kappa < 0.2);
    CHECK(w.t_lock > 0.0);
    CHECK(w.t_lock < 2.0);
    CHECK(speed(rig2) - v_request > 3.0 * 2.0); // > 2 m/s^2 mean: pulling hard, not a torque hole
}

TEST_CASE("Auto-clutch: WOT in 2nd from a steady 20 km/h does not spin the driven wheels", "[validation][vehicle][powertrain]") {
    auto rig = make_sedan();
    ps_test::settle(*rig.world, 1.0);
    rig.world->set_control("gear_request", 1.0);
    rig.world->set_control("throttle", 0.5);
    REQUIRE(drive_until_speed(rig, 15.0 / 3.6, 15.0));
    rig.world->set_control("gear_request", 2.0);
    rig.world->set_control("throttle", 0.3);
    Watch wshift;
    run_watch(rig, 2.0, 2, wshift);
    hold_speed(rig, 20.0 / 3.6, 4.0);
    const auto dbg0 = rig.world->powertrain_debug(rig.vehicle);
    INFO("steady: v_kmh=" << speed(rig) * 3.6 << " clutch_stuck=" << dbg0.clutch_stuck);
    REQUIRE(rig.world->powertrain_state(rig.vehicle).gear == 2);
    REQUIRE(std::abs(speed(rig) * 3.6 - 20.0) < 2.0);
    REQUIRE(dbg0.clutch_stuck);

    rig.world->set_control("throttle", 1.0);
    const real v0 = speed(rig);
    Watch w;
    run_watch(rig, 3.0, 2, w);
    INFO("max|kappa|=" << w.max_driven_abs_kappa << " v_end_kmh=" << speed(rig) * 3.6);
    CHECK_FALSE(w.stalled);
    CHECK(w.max_driven_abs_kappa < 0.15);
    CHECK(speed(rig) - v0 > 3.0 * 1.5);
}

// Owner's primary case at full throttle: 1->2 at a realistic 15 km/h.
TEST_CASE("Auto-clutch: full-throttle 1->2 upshift at 15 km/h locks promptly without wheelspin", "[validation][vehicle][powertrain]") {
    auto rig = make_sedan();
    ps_test::settle(*rig.world, 1.0);
    rig.world->set_control("gear_request", 1.0);
    rig.world->set_control("throttle", 0.6);
    REQUIRE(drive_until_speed(rig, 15.0 / 3.6, 15.0));
    rig.world->set_control("throttle", 1.0);
    rig.world->set_control("gear_request", 2.0);
    const real v_request = speed(rig);
    Watch w;
    run_watch(rig, 3.0, 2, w);
    INFO("v_request_kmh=" << v_request * 3.6 << " t_lock=" << w.t_lock << " max|kappa|=" << w.max_driven_abs_kappa
                          << " v_end_kmh=" << speed(rig) * 3.6);
    CHECK(w.final_gear == 2);
    CHECK_FALSE(w.stalled);
    CHECK(w.t_lock > 0.0);
    CHECK(w.t_lock < 1.0);
    CHECK(w.max_driven_abs_kappa < 0.2);
    CHECK(speed(rig) - v_request > 3.0 * 1.5);
}

// Haptics agent's report (unverified, adapter-side): after a 2->3 upshift at
// part throttle the clutch sat at ~0.07 engagement for seconds. From a
// realistic shift point (2nd at ~2500 rpm) the clutch must lock within 1 s.
TEST_CASE("Auto-clutch: part-throttle 2->3 upshift from a realistic shift point locks within 1 s", "[validation][vehicle][powertrain]") {
    auto rig = make_sedan();
    ps_test::settle(*rig.world, 1.0);
    rig.world->set_control("gear_request", 1.0);
    rig.world->set_control("throttle", 0.6);
    REQUIRE(drive_until_speed(rig, 15.0 / 3.6, 15.0));
    rig.world->set_control("gear_request", 2.0);
    rig.world->set_control("throttle", 0.5);
    REQUIRE(drive_until_speed(rig, 36.0 / 3.6, 20.0)); // 2nd at ~2500 rpm
    REQUIRE(rig.world->powertrain_state(rig.vehicle).gear == 2);

    rig.world->set_control("throttle", 0.3);
    rig.world->set_control("gear_request", 3.0);
    const real v_request = speed(rig);
    Watch w;
    run_watch(rig, 3.0, 3, w);
    INFO("v_request_kmh=" << v_request * 3.6 << " t_lock=" << w.t_lock << " max|kappa|=" << w.max_driven_abs_kappa
                          << " v_end_kmh=" << speed(rig) * 3.6);
    CHECK(w.final_gear == 3);
    CHECK_FALSE(w.stalled);
    CHECK(w.t_lock > 0.0);
    CHECK(w.t_lock < 1.0);
    CHECK(w.max_driven_abs_kappa < 0.1);
}

// Haptics agent's scripted demo drive (coordinator report): N->1st requested
// (shift_up_count, the gamepad's own path) in the SAME tick as full throttle,
// from rest, auto-clutch on. Before the fix the rpm-gated Launch law closed
// the clutch as soon as the flaring engine passed bite_rpm, while the
// gearbox was still Selecting/Synchronising 1st: the loaded input shaft held
// the synchro off (it can only drag the car through its own slip), so the
// dog never went in. On the flat the synchro's drag happened to pull the
// car past launch_speed_threshold_mps, which re-classified the engagement
// as a rolling shift and opened the clutch (gear in after ~0.65 s); uphill
// the gearbox aborted after abort_after_s and left the car in Neutral with
// the engine on the limiter indefinitely.
TEST_CASE("Auto-clutch: N->1 requested at WOT from rest engages and drives away", "[validation][vehicle][powertrain]") {
    for (const real grade : {real(0.0), real(-0.15)}) { // -0.15: 15% uphill (+X is downhill, see build_vehicle_rig)
        auto rig = ps_test::build_vehicle_rig(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json", 1500.0,
                                              Vec3{2.0, 0.4, 0.15}, 0.6, grade, 4, Vec3::unit_y(), 240.0, 960.0, 3000.0);
        rig.world->set_control("handbrake", 1.0); // hold the car on the slope while settling
        ps_test::settle(*rig.world, 1.0);
        rig.world->set_control("handbrake", 0.0);
        rig.world->set_control("shift_up_count", 1.0);
        rig.world->set_control("throttle", 1.0);
        real t_engaged = -1.0;
        Watch w;
        run_watch(rig, 3.0, 1, w, [&](real t) {
            if (t_engaged < 0.0 && rig.world->powertrain_state(rig.vehicle).gear == 1) t_engaged = t;
        });
        const auto& pt = rig.world->powertrain_state(rig.vehicle);
        INFO("grade=" << grade << " t_engaged=" << t_engaged << " t_lock=" << w.t_lock << " gear=" << int(pt.gear)
                      << " rpm=" << pt.engine_rpm << " v_end_kmh=" << speed(rig) * 3.6 << " max|kappa|=" << w.max_driven_abs_kappa);
        CHECK_FALSE(w.stalled);
        CHECK(w.final_gear == 1);
        CHECK(t_engaged > 0.0);
        CHECK(t_engaged < 1.5);
        CHECK(speed(rig) > 10.0 / 3.6); // actually driving away within the 3 s
    }
}

// Coordinator report 2026-09-25: with the runtime assist.auto_shift channel
// switched on (the HUD/gamepad toggle) 1st gear at WOT sat on the 6800 rpm
// limiter for 25 s - ManualTcu gated auto-shift on AutoShiftDesc::enabled
// (false in car_sedan.json), never on the channel, while
// PowertrainSnapshot::assist_active_bits reported bit2 from the channel.
// The channel is the switch (plan section 7.1); the desc flag is only its
// default. car_sedan.json: up_rpm_at_full_throttle 6300.
TEST_CASE("Auto-shift: the assist.auto_shift channel alone enables it; WOT upshifts near up_rpm_at_full_throttle",
          "[validation][vehicle][powertrain]") {
    auto rig = make_sedan();
    ps_test::settle(*rig.world, 1.0);
    REQUIRE(rig.world->get_control("assist.auto_shift") == 0.0); // desc default (enabled: false)
    rig.world->set_control("assist.auto_shift", 1.0);
    rig.world->set_control("throttle", 1.0); // auto-shift itself selects 1st from N (throttle > 0.05 at rest)
    int prev_gear = rig.world->powertrain_state(rig.vehicle).gear;
    real prev_rpm = rig.world->powertrain_state(rig.vehicle).engine_rpm;
    real rpm_at_1_2 = -1.0, rpm_at_2_3 = -1.0, t_1 = -1.0;
    bool stalled = false;
    const real dt = rig.world->dt();
    for (int i = 1; i <= static_cast<int>(20.0 / dt); ++i) {
        const auto& pt = rig.world->powertrain_state(rig.vehicle);
        prev_gear = pt.gear;
        prev_rpm = pt.engine_rpm;
        const int target_before = pt.gear_target;
        rig.world->step();
        const auto& now = rig.world->powertrain_state(rig.vehicle);
        if (now.engine_state == drivetrain::EngineState::Stalled) stalled = true;
        if (t_1 < 0.0 && now.gear == 1) t_1 = i * dt;
        // the rpm at which the TCU commanded the upshift (target moved past the current gear)
        if (now.gear_target == 2 && target_before == 1 && rpm_at_1_2 < 0.0) rpm_at_1_2 = prev_rpm;
        if (now.gear_target == 3 && target_before == 2 && rpm_at_2_3 < 0.0) rpm_at_2_3 = prev_rpm;
    }
    (void)prev_gear;
    const auto& pt = rig.world->powertrain_state(rig.vehicle);
    INFO("t_in_1st=" << t_1 << " rpm_at_1_2=" << rpm_at_1_2 << " rpm_at_2_3=" << rpm_at_2_3 << " final_gear=" << int(pt.gear)
                     << " v_end_kmh=" << speed(rig) * 3.6 << " assist_bits=" << pt.assist_active_bits);
    CHECK_FALSE(stalled);
    CHECK((pt.assist_active_bits & 4u) != 0u);
    CHECK(t_1 > 0.0);
    CHECK(rpm_at_1_2 > 6000.0);
    CHECK(rpm_at_1_2 < 6600.0);
    CHECK(rpm_at_2_3 > 6000.0);
    CHECK(rpm_at_2_3 < 6600.0);
    CHECK(pt.gear >= 3);
}
