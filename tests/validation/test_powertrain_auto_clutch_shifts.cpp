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
#include <vector>

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
TEST_CASE("Auto-clutch: selecting 2nd at walking pace never stalls and pulls away on throttle", "[validation][vehicle][powertrain][drivetrain]") {
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
TEST_CASE("Auto-clutch: part-throttle 1->2 upshift at 15 km/h locks promptly without wheelspin", "[validation][vehicle][powertrain][drivetrain]") {
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
TEST_CASE("Auto-clutch: WOT 1->2 upshift from a non-spinning 1st keeps the driven wheels below peak slip", "[validation][vehicle][powertrain][drivetrain]") {
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

TEST_CASE("Auto-clutch: WOT in 2nd from a steady 20 km/h does not spin the driven wheels", "[validation][vehicle][powertrain][drivetrain]") {
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
TEST_CASE("Auto-clutch: full-throttle 1->2 upshift at 15 km/h locks promptly without wheelspin", "[validation][vehicle][powertrain][drivetrain]") {
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
TEST_CASE("Auto-clutch: part-throttle 2->3 upshift from a realistic shift point locks within 1 s", "[validation][vehicle][powertrain][drivetrain]") {
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
TEST_CASE("Auto-clutch: N->1 requested at WOT from rest engages and drives away", "[validation][vehicle][powertrain][drivetrain]") {
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
          "[validation][vehicle][powertrain][drivetrain]") {
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

// Owner test drive 2026-09-25: "You said you would cut ignition during the
// upshift, but I still see the speed ticking throughout the entire shift
// process." The upshift cut ended with the gearbox phase, so for the whole
// post-shift close (~0.8 s) the engine ran at full power through the
// torque-following clutch (engine torque + the sync term, ~245 Nm vs. the
// 200 Nm peak) and the car pulled at 3.5-4 m/s^2 - harder than locked 2nd.
// With the cut held through the close until the engine is within
// AutoBlipDesc::upshift_cut_release_slip_rpm of the new gear, the car coasts
// through the shift, then the torque ramps back in over
// upshift_torque_ramp_s. Per tick, from the shift request:
namespace {
struct UpshiftObs {
    real t_neutral = -1.0;  // first tick the gearbox reports neutral (clutch open, dog out)
    real t_dog_in = -1.0;   // first tick in the target gear
    real t_cut_end = -1.0;  // first tick (after dog-in) the engine no longer reads the cut, pedal ceiling back to 1
    real t_hole_end = -1.0; // first tick after t_cut_end with the clutch carrying at least the engine's torque demand
    real t_lock = -1.0;
    real v_neutral = 0.0, v_cut_end = 0.0;
    real a_close_max = -1e9;        // max 50 ms-window accel between dog-in and the cut's end (the ClutchClosing hold)
    real max_clutch_excess = -1e9;  // max |T_clutch| - max(0, T_demand + max_sync_torque_nm) after the cut's end
    real max_driven_abs_kappa = 0.0;
    bool stalled = false;
    int final_gear = 0;
};

constexpr real kMaxSyncTorqueNm = 50.0; // car_sedan.json auto_clutch.max_sync_torque_nm

UpshiftObs watch_upshift(ps_test::VehicleTestRig& rig, int target_gear, real seconds) {
    UpshiftObs o;
    const real dt = rig.world->dt();
    const int n = static_cast<int>(seconds / dt + 0.5);
    std::vector<real> v_hist; // speed per tick, index i = after tick i+1
    v_hist.reserve(static_cast<std::size_t>(n));
    int i_dog_in = -1, i_cut_end = -1;
    for (int i = 0; i < n; ++i) {
        const real t = static_cast<real>(i + 1) * dt;
        rig.world->step();
        const auto& pt = rig.world->powertrain_state(rig.vehicle);
        const auto dbg = rig.world->powertrain_debug(rig.vehicle);
        const real v = speed(rig);
        v_hist.push_back(v);
        if (pt.engine_state == drivetrain::EngineState::Stalled) o.stalled = true;
        for (std::size_t wi = 0; wi < 2; ++wi) {
            o.max_driven_abs_kappa = std::max(o.max_driven_abs_kappa, std::abs(rig.world->wheel_telemetry(rig.vehicle, wi).slip_ratio));
        }
        if (o.t_neutral < 0.0 && pt.gear == 0) {
            o.t_neutral = t;
            o.v_neutral = v;
        }
        if (o.t_dog_in < 0.0 && o.t_neutral > 0.0 && pt.gear == target_gear) {
            o.t_dog_in = t;
            i_dog_in = i;
        }
        if (o.t_dog_in > 0.0 && o.t_cut_end < 0.0 && !dbg.ignition_cut) {
            o.t_cut_end = t;
            o.v_cut_end = v;
            i_cut_end = i;
        }
        if (o.t_cut_end > 0.0) {
            const real t_clutch = std::abs(dbg.clutch_torque);
            if (o.t_hole_end < 0.0 && dbg.throttle_ceiling >= 1.0 && t_clutch >= dbg.engine_torque_demand) o.t_hole_end = t;
            // The close law is T = max(0, T_demand + min(sync, max_sync_torque_nm)), so its ceiling is
            // max(0, T_demand + max_sync_torque_nm). Without the max(0, .) a motoring (negative) demand
            // below -50 N*m made even a fully open clutch (T = 0) read as "excess" (60/40 split, auto 2->3).
            o.max_clutch_excess = std::max(o.max_clutch_excess,
                                           t_clutch - std::max(real(0.0), dbg.engine_torque_demand + kMaxSyncTorqueNm));
        }
        if (o.t_lock < 0.0 && o.t_dog_in > 0.0 && dbg.clutch_stuck && std::abs(rad_s_to_rpm(dbg.clutch_slip)) < 80.0) o.t_lock = t;
        o.final_gear = pt.gear;
    }
    const int win = static_cast<int>(0.05 / dt + 0.5);
    if (i_dog_in >= 0 && i_cut_end > i_dog_in) {
        for (int j = i_dog_in; j + win <= i_cut_end; ++j) {
            o.a_close_max = std::max(o.a_close_max, (v_hist[static_cast<std::size_t>(j + win)] - v_hist[static_cast<std::size_t>(j)]) / (win * dt));
        }
    }
    return o;
}

void check_upshift(const UpshiftObs& o, int target_gear, real max_kappa, real hole_max_s) {
    const real a_coast = (o.t_cut_end > o.t_neutral) ? (o.v_cut_end - o.v_neutral) / (o.t_cut_end - o.t_neutral) : 1e9;
    INFO("t_neutral=" << o.t_neutral << " t_dog_in=" << o.t_dog_in << " t_cut_end=" << o.t_cut_end << " t_hole_end=" << o.t_hole_end
                      << " t_lock=" << o.t_lock << " a_coast_mean=" << a_coast << " a_close_max(50ms)=" << o.a_close_max
                      << " max_clutch_excess=" << o.max_clutch_excess << " max|kappa|=" << o.max_driven_abs_kappa
                      << " final_gear=" << o.final_gear);
    CHECK(o.final_gear == target_gear);
    CHECK_FALSE(o.stalled);
    REQUIRE(o.t_dog_in > 0.0);
    REQUIRE(o.t_cut_end > o.t_dog_in); // the cut outlives the gearbox phase
    // The car coasts from the moment the clutch is open until the torque
    // comes back (the synchro's brief input-shaft kick included in the mean).
    CHECK(a_coast < 0.3);
    CHECK(o.a_close_max < 0.3);
    // Torque hole: shift request -> pedal ceiling back at 1 and the clutch
    // carrying the engine's full torque demand again. Its length is set by
    // the engine's own motoring decay (car_sedan.json's i4_2l_na: ~61 Nm at
    // 5000 rpm on 0.16 kg*m^2, ~3300-3600 rpm/s) over the gear step, plus
    // the gearbox shift (~0.17 s) and the torque ramp (0.10 s) - per-case
    // bound from the measured value with ~10% margin.
    CHECK(o.t_hole_end > 0.3);
    CHECK(o.t_hole_end < hole_max_s);
    CHECK(o.max_clutch_excess < 5.0);
    CHECK(o.max_driven_abs_kappa < max_kappa);
    CHECK(o.t_lock > 0.0);
    CHECK(o.t_lock < 1.5);
}

// Waits (stepping) until the gearbox is commanded past `from_gear`; false on timeout.
bool wait_for_upshift_request(ps_test::VehicleTestRig& rig, int from_gear, real timeout_s) {
    const int n = static_cast<int>(timeout_s / rig.world->dt() + 0.5);
    for (int i = 0; i < n; ++i) {
        rig.world->step();
        if (rig.world->powertrain_state(rig.vehicle).gear_target > from_gear) return true;
    }
    return false;
}
} // namespace

// The owner's case (coordinator trace upshift_trace.json): 1st at WOT, 1->2
// at ~44 km/h / ~6300 rpm. Before: no coasting phase at all (a 3.5-4.1 m/s^2
// pull through the whole 0.78 s ClutchClosing).
TEST_CASE("Upshift cut: WOT 1->2 near the shift point coasts through the shift, then the torque ramps back in",
          "[validation][vehicle][powertrain][drivetrain]") {
    auto rig = make_sedan();
    ps_test::settle(*rig.world, 1.0);
    rig.world->set_control("gear_request", 1.0);
    rig.world->set_control("throttle", 0.5);
    REQUIRE(drive_until_speed(rig, 30.0 / 3.6, 15.0));
    rig.world->set_control("throttle", 1.0);
    REQUIRE(drive_until_speed(rig, 44.0 / 3.6, 10.0));
    INFO("rpm_at_request=" << rig.world->powertrain_state(rig.vehicle).engine_rpm);
    rig.world->set_control("gear_request", 2.0);
    const UpshiftObs o = watch_upshift(rig, 2, 3.0);
    check_upshift(o, 2, 0.2, 0.85); // measured 0.78 s (6033 rpm, a 2560 rpm step)
}

TEST_CASE("Upshift cut: WOT 2->3 near the shift point coasts through the shift, then the torque ramps back in",
          "[validation][vehicle][powertrain][drivetrain]") {
    auto rig = make_sedan();
    ps_test::settle(*rig.world, 1.0);
    rig.world->set_control("gear_request", 1.0);
    rig.world->set_control("throttle", 0.6);
    REQUIRE(drive_until_speed(rig, 15.0 / 3.6, 15.0));
    rig.world->set_control("gear_request", 2.0);
    rig.world->set_control("throttle", 1.0);
    REQUIRE(drive_until_speed(rig, 85.0 / 3.6, 30.0)); // 2nd at ~5900 rpm
    REQUIRE(rig.world->powertrain_state(rig.vehicle).gear == 2);
    INFO("rpm_at_request=" << rig.world->powertrain_state(rig.vehicle).engine_rpm);
    rig.world->set_control("gear_request", 3.0);
    const UpshiftObs o = watch_upshift(rig, 3, 3.0);
    check_upshift(o, 3, 0.15, 0.65); // measured 0.57 s (6151 rpm, a 1950 rpm step)
}

// The same speed-matched cut on the auto-shift path (assist.auto_shift=1 at
// WOT: the TCU itself requests 1->2 and 2->3 near up_rpm_at_full_throttle).
// Pulls away in 1st at part throttle first: a WOT launch from rest spins
// the front tyres (a TC topic, I5), and auto-shift reads ENGINE rpm, so it
// would then upshift at a road speed whose 2nd-gear rpm is below its own
// down_rpm_at_full_throttle.
TEST_CASE("Upshift cut: auto-shift WOT upshifts coast through the shift too", "[validation][vehicle][powertrain][drivetrain]") {
    auto rig = make_sedan();
    ps_test::settle(*rig.world, 1.0);
    rig.world->set_control("shift_up_count", 1.0);
    rig.world->set_control("throttle", 0.5);
    REQUIRE(drive_until_speed(rig, 30.0 / 3.6, 15.0));
    REQUIRE(rig.world->powertrain_state(rig.vehicle).gear == 1);
    rig.world->set_control("assist.auto_shift", 1.0);
    rig.world->set_control("throttle", 1.0);
    for (const int from : {1, 2}) {
        INFO("auto upshift from " << from);
        REQUIRE(wait_for_upshift_request(rig, from, 30.0));
        INFO("rpm_at_request=" << rig.world->powertrain_state(rig.vehicle).engine_rpm);
        const UpshiftObs o = watch_upshift(rig, from + 1, 1.5);
        check_upshift(o, from + 1, from == 1 ? real(0.2) : real(0.15), from == 1 ? real(0.85) : real(0.65));
    }
}
