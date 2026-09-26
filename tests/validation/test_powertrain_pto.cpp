// tests/validation/test_powertrain_pto.cpp — PHYS-033 P4 I4 C11: whole-
// vehicle acceptance test for the new `pto` (power take-off) drivetrain
// component, mirroring test_powertrain_mfwd.cpp's/test_powertrain_energy_
// balance.cpp's own patterns (this file's unit-level companion,
// tests/unit/test_powertrain_pto.cpp, already covers the raw-network row
// identities, the engagement ramp, the mode-switch eligibility rule, the
// disengaged stub-brake's exact implicit-Euler recurrence, the efficiency
// loss identity, row-placement on both car_sedan and tractor, debug_alloc,
// and 1-vs-10-worker determinism - none of that is repeated here).
//
// Rig: tractor.json (data/vehicles/tractor.json) plus a Viscous load_dyno
// shafted onto "pto.out" IN CODE (LoadDyno is a test/bench-only component -
// no shipped vehicle carries one, PHYS-033 P4 I4 C11 plan section 3.5) -
// load_tractor_with_pto_dyno() below is the SAME fixture (viscous 5.0
// N*m*s/rad, inertia 0.02 kg*m^2) as tests/unit/test_powertrain_pto.cpp's
// own helper of the same name, so both files exercise one physically
// consistent load. tractor.json's own shafts[0] unions "engine.crank",
// "clutch.in" AND "pto.in" onto ONE rigid node (a "live PTO" tapped
// directly off the crank, ahead of the clutch/gearbox entirely) - so
// gear_request/park_lock only ever affect the DRIVE path to the wheels,
// never the PTO branch. Ground: build_vehicle_rig_from_desc's flat, UNTAGGED
// BoxShape ground (tests/support/vehicle_test_helpers.h) - explicitly
// exempt from PHYS-047's mesh-tessellation rule (CLAUDE.md: "flat/tilted
// BoxShape grounds ... are fine as-is") and adequate here since the vehicle
// never needs to move (gearbox in Neutral, park_lock engaged).
//
// Steady-state power identity: LoadDynoMode::Viscous's own per-substep
// dissipation is EXACTLY h*c*omega_new^2 (energy_ledger.h's own comment on
// PowertrainEnergy::external_absorbed: "viscous/quadratic: -h*tau_applied*
// omega'", and tau_applied = -c*omega_new for Viscous) - this holds
// regardless of the pto's own efficiency/ratio/gearing on the "in" side, so
// checking it needs no derivation through the friction row's own multiplier
// or the engine-side torque balance at all. At a genuine steady state (the
// dyno's own port speed no longer changing tick-to-tick), the measured
// energy-drain RATE (external_absorbed's delta over a trailing window,
// divided by the window's duration) must equal c times the mean of
// omega_out^2 sampled once per tick over that same window (omega_out from
// PowertrainSnapshot::pto_rpm, converted rad/s = rpm*2*pi/60) - both
// computed independently of one another (one from the cumulative energy
// ledger, the other from the kinematic snapshot), so this is a genuine
// discriminating cross-check, not a tautology.
//
// PHYS-033 P4 I4 C11, review round 2 (coordinator finding): the 1000-mode
// case previously ran at the SAME throttle (0.35) as the 540-mode case and
// was asserted/labelled as reaching "a new steady state" on the strength of
// a per-tick gate (`max_abs_domega_out < 0.02`) alone. That gate cannot see
// slow monotonic drift: over a 720-tick window it permits 720*0.02 = 14.4
// rad/s of cumulative change, far more than what was actually happening.
// Measured with a temporary trace: at throttle 0.35 in 1000 mode, omega_out
// fell from 50.877 to 47.660 rad/s and engine rpm fell from 947 to 887 rpm
// over the old 720-tick (3 s) window - genuinely still drifting, not
// steady. Even a 60 s settle only reached 696.07 rpm (below the 850 rpm
// idle target) and was STILL creeping (37.406 -> 37.381 rad/s over a
// further 3 s) - this operating point does not settle in any practical
// window. Root cause: data/engines/diesel_i4_4l5_turbo.json's
// idle_control.max_throttle (0.25) caps only the idle-speed PI's own
// contribution to theta_eff (engine_torque_map.cpp's theta_idle path); the
// driver-commanded theta_req (0.35 here) is NOT capped by it and reaches
// EngineTorqueMap::pre_solve directly (theta_eff = max(theta_req,
// theta_idle)). At throttle 0.35 with 1000 mode's smaller ratio (1.95 vs.
// 540's 3.61) reflecting more of the dyno's viscous load onto the crank
// (c_effective = c/ratio^2 -> c/3.80 vs. c/13.03), the operating point sits
// on a nearly-flat stretch of the diesel's own WOT torque curve well below
// the soft_cut limiter band (2200-2400 rpm) - an open-loop torque-map
// balance with a very slow (tens-of-seconds) time constant, not a defect in
// the pto row itself (the friction row's own ratio and slip were already
// exact at every sample: 1.950000000 and ~2e-12 rad/s respectively). This
// is expected I3-era behaviour (an all-speed governor is deferred to a
// later increment, PLAN.md section 5.3) - the fix here is entirely test-
// side: (1) a window-drift steadiness gate (first-vs-last sample over the
// measurement window, not a per-tick bound) that would have caught the
// original defect, and (2) commanding throttle 1.0 after the mode switch
// so 1000 mode sits on the SAME soft_cut limiter band the 540-mode case
// already relies on for its own steadiness - verified below by actual
// measurement (window_drift_omega_out/window_drift_engine_rpm), not
// assumed. MEASURED at the new (WOT) 1000-mode operating point (this
// file's own run, release build): engine settles at 2243.47 rpm (inside
// the 2200-2400 rpm soft_cut band), pto.out at 120.48 rad/s (1150.5 rpm),
// dyno power 72577.1 W - both window_drift_omega_out and
// window_drift_engine_rpm measure exactly 0.0 (bit-constant first-to-last
// sample), i.e. genuinely steady, not merely "under the per-tick bound".
// The 540-mode case (throttle 0.35 the whole time) was already genuinely
// steady (omega_out bit-constant at 64.215547 rad/s, engine 2213.70 rpm,
// ratio 3.610000000 exact, slip ~4e-12 rad/s) and is unchanged.
//
// Because the two modes now run at DIFFERENT throttle commands (0.35 for
// 540, 1.0 for 1000 after the switch), their absolute omega_out/engine_rpm
// values are not comparable to each other at all - neither test asserts an
// inequality between them, only that each individually reaches a genuine,
// measured steady state.
//
// PHYS-033 P4 I4 C12 (2026-09-26): tractor.json's engine gained a real
// all-speed governor (docs/data_formats/engine.md's "governor" section),
// so "throttle" (the "throttle" control channel, which the governor's
// lever = max(hand_throttle, pedal) formula still reads) no longer sets
// fuel directly - it sets the governor's no-load speed TARGET, n_nl =
// idle_rpm + lever*(high_idle_rpm - idle_rpm) = 850 + lever*1550, which the
// governor then holds (within its droop band) against load. This is a
// genuine, expected change in operating point, re-measured (not tuned to
// pass) rather than assumed: at throttle 0.35, n_nl = 1392.5 rpm no-load,
// and the 540-mode case now settles at engine_rpm=1355.6 (droop pulling it
// ~37 rpm below the no-load target under the dyno's load) instead of the
// pre-C12 2213.70 rpm (that old figure came from WOT fuelling hitting the
// soft_cut-as-governor band directly, a fundamentally different mechanism -
// see the history above). pto.out settles at 39.3234 rad/s (375.511 rpm,
// ratio 3.610000000 exact, slip 2.64e-12 rad/s), power 7731.66 W (predicted
// c*mean(omega_out^2) also 7731.66 W), gear_loss 309.266 W (predicted the
// same), worst relative balance_error 2.12e-15 - still bit-exactly steady
// (window_drift_omega_out/window_drift_engine_rpm both measure exactly
// 0.0). At throttle 1.0 (lever 1.0, n_nl = high_idle_rpm = 2400 rpm exactly,
// the governor's ceiling), the 1000-mode case settles at engine_rpm=2243.49
// - essentially UNCHANGED from the pre-C12 2243.47 rpm, because full lever
// is the one operating point where the new all-speed-governor model and the
// old soft_cut-as-governor idiom coincide (both regulate toward the same
// ~2400 rpm ceiling under load). pto.out settles at 120.481 rad/s (1150.51
// rpm, ratio 1.950000000 exact, slip 5.20e-12 rad/s), power 72578 W
// (predicted also 72578 W), gear_loss 2903.12 W (predicted the same), worst
// relative balance_error 4.56e-16 - also still bit-exactly steady. Both
// modes' window-drift/ratio/slip/power/balance-error assertions (none of
// which hard-code an absolute rpm) pass unchanged; only the absolute
// engine_rpm/pto_rpm figures quoted in comments above and here moved, and
// only for the 540-mode (partial-lever) case, exactly as the governor model
// predicts.

#include "vehicle_test_helpers.h"

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/vehicle_io.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <string>

using namespace ps;
using namespace ps::drivetrain;

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/validation/CMakeLists.txt)"
#endif

namespace {

// core/src/drivetrain/powertrain.h's own pto_rpm() literal, reproduced here
// (this file has no access to that internal header's constant) - PLAN.md D7:
// a fixed literal, never a runtime libm transcendental.
constexpr real kPi = 3.14159265358979323846;
constexpr real kRadPerSecToRpm = real(60.0) / (real(2.0) * kPi);
constexpr real kRpmToRadPerSec = real(1.0) / kRadPerSecToRpm;

constexpr real kChassisMass = 6000.0;
const Vec3 kChassisHalfExtents{1.2, 0.35, 0.9};
constexpr real kChassisOriginHeight = 1.28;

vehicle::VehicleDesc load_tractor() { return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/tractor.json"); }

// SAME fixture as tests/unit/test_powertrain_pto.cpp's helper of the same
// name (viscous 5.0 N*m*s/rad, inertia 0.02 kg*m^2, shafted onto pto.out -
// which has no shaft entry of its own in tractor.json, so this is a brand
// new ShaftDesc, not an append onto an existing one).
vehicle::VehicleDesc load_tractor_with_pto_dyno() {
    vehicle::VehicleDesc d = load_tractor();
    LoadDynoDesc dyno_desc;
    dyno_desc.mode = LoadDynoMode::Viscous;
    dyno_desc.viscous_nm_s_rad = 5.0;
    dyno_desc.inertia_kgm2 = 0.02;
    ComponentDesc dyno_component;
    dyno_component.id = "dyno";
    dyno_component.params = dyno_desc;
    d.powertrain.components.push_back(dyno_component);
    ShaftDesc shaft;
    shaft.ports = {PortRef{"pto", "out", false}, PortRef{"dyno", "shaft", false}};
    d.powertrain.shafts.push_back(shaft);
    return d;
}

constexpr real kDynoViscous = 5.0;   // N*m*s/rad - must match load_tractor_with_pto_dyno() above
constexpr real kPtoEfficiency = 0.96; // must match data/vehicles/tractor.json's pto.efficiency
constexpr real kPtoRatio540 = 3.61;   // must match data/vehicles/tractor.json's pto.ratios["540"]
constexpr real kPtoRatio1000 = 1.95;  // must match data/vehicles/tractor.json's pto.ratios["1000"]

struct SteadyResult {
    real measured_power_w = 0.0;  // from PowertrainEnergy::external_absorbed's delta / window
    real predicted_power_w = 0.0; // c * mean(omega_out^2) over the same window
    real mean_omega_out_rad_s = 0.0;
    real max_abs_domega_out = 0.0; // per-tick gate: largest tick-to-tick |delta omega_out| in the window
    real worst_balance_error = 0.0;
    real engine_rpm = 0.0;
    real pto_rpm = 0.0;
    real engine_torque_demand_nm = 0.0;
    real pto_clutch_slip = 0.0;
    real pto_clutch_torque_nm = 0.0;

    // Window-drift steadiness (coordinator review round 2): first-vs-last
    // sample over the WHOLE window, relative to the window's own mean - a
    // per-tick bound cannot see slow monotonic drift (720 ticks * a 0.02
    // rad/s per-tick allowance is 14.4 rad/s of permitted cumulative
    // drift), this can.
    real first_omega_out_rad_s = 0.0;
    real last_omega_out_rad_s = 0.0;
    real window_drift_omega_out = 0.0; // relative
    real first_engine_rpm = 0.0;
    real last_engine_rpm = 0.0;
    real window_drift_engine_rpm = 0.0; // relative

    // Efficiency identity (coordinator review round 2): the pto's own
    // (1-eta)*|lambda|*|omega_in| loss accounting, measured two
    // independent ways - see components.cpp's Pto::post_solve, which
    // books this exact term into PowertrainEnergy::gear_loss (NOT
    // pto_clutch_loss, which is only the friction row's own slip loss and
    // the disengaged stub-brake term - both ~0 here since the row is
    // locked). GearPair's own gearbox-mesh loss and Differential's own
    // mesh loss also book into gear_loss (components.cpp lines ~114 and
    // ~255, plus powertrain.cpp's gearbox_own_loss_tau_ term) - this rig
    // runs gear_request=0 (Neutral) with park_lock engaged, so neither
    // should be contributing anything; the tightness of the match below
    // (not a loosened tolerance) is itself the verification that they are
    // not, rather than an assumption.
    real measured_gear_loss_rate_w = 0.0;
    real predicted_gear_loss_rate_w = 0.0;
};

// Runs the rig for settle_seconds (checking balance_error every tick, as
// test_powertrain_mfwd.cpp/test_powertrain_energy_balance.cpp both do), then
// measures the steady-state power identity over a trailing window_seconds.
SteadyResult run_and_measure(World& w, VehicleId vehicle, real settle_seconds, real window_seconds) {
    SteadyResult r;
    const real dt = w.dt();

    const int settle_ticks = static_cast<int>(std::lround(settle_seconds / dt));
    for (int i = 0; i < settle_ticks; ++i) {
        w.step();
        const PowertrainEnergy& e = w.powertrain_energy(vehicle);
        const real denom = std::max(real(1.0), std::abs(e.engine_out));
        r.worst_balance_error = std::max(r.worst_balance_error, std::abs(e.balance_error) / denom);
        CHECK(std::abs(e.balance_error) / denom <= 5e-8);
    }

    const real energy_start = w.powertrain_energy(vehicle).external_absorbed;
    const real gear_loss_start = w.powertrain_energy(vehicle).gear_loss;
    const int window_ticks = static_cast<int>(std::lround(window_seconds / dt));
    real sum_omega = 0.0, sum_omega_sq = 0.0, sum_engine_rpm = 0.0, sum_abs_torque_omega_in = 0.0;
    real prev_omega = w.powertrain_state(vehicle).pto_rpm * kRpmToRadPerSec;
    for (int i = 0; i < window_ticks; ++i) {
        w.step();
        const PowertrainEnergy& e = w.powertrain_energy(vehicle);
        const real denom = std::max(real(1.0), std::abs(e.engine_out));
        r.worst_balance_error = std::max(r.worst_balance_error, std::abs(e.balance_error) / denom);
        CHECK(std::abs(e.balance_error) / denom <= 5e-8);

        const PowertrainSnapshot& snap = w.powertrain_state(vehicle);
        const PowertrainDebug dbg = w.powertrain_debug(vehicle);
        const real omega = snap.pto_rpm * kRpmToRadPerSec;
        const real omega_in = snap.engine_rpm * kRpmToRadPerSec; // pto.in IS the crank node (shafts[0])

        sum_omega += omega;
        sum_omega_sq += omega * omega;
        sum_engine_rpm += snap.engine_rpm;
        sum_abs_torque_omega_in += std::abs(dbg.pto_clutch_torque) * std::abs(omega_in);
        r.max_abs_domega_out = std::max(r.max_abs_domega_out, std::abs(omega - prev_omega));
        prev_omega = omega;

        if (i == 0) {
            r.first_omega_out_rad_s = omega;
            r.first_engine_rpm = snap.engine_rpm;
        }
        if (i == window_ticks - 1) {
            r.last_omega_out_rad_s = omega;
            r.last_engine_rpm = snap.engine_rpm;
        }
    }
    const real n = static_cast<real>(window_ticks);
    r.mean_omega_out_rad_s = sum_omega / n;
    r.predicted_power_w = kDynoViscous * (sum_omega_sq / n);

    const real energy_end = w.powertrain_energy(vehicle).external_absorbed;
    r.measured_power_w = (energy_end - energy_start) / window_seconds;

    const real gear_loss_end = w.powertrain_energy(vehicle).gear_loss;
    r.measured_gear_loss_rate_w = (gear_loss_end - gear_loss_start) / window_seconds;
    r.predicted_gear_loss_rate_w = (real(1.0) - kPtoEfficiency) * (sum_abs_torque_omega_in / n);

    r.window_drift_omega_out = std::abs(r.last_omega_out_rad_s - r.first_omega_out_rad_s) / std::abs(r.mean_omega_out_rad_s);
    const real mean_engine_rpm = sum_engine_rpm / n;
    r.window_drift_engine_rpm = std::abs(r.last_engine_rpm - r.first_engine_rpm) / std::abs(mean_engine_rpm);

    const PowertrainSnapshot& final_snap = w.powertrain_state(vehicle);
    r.engine_rpm = final_snap.engine_rpm;
    r.pto_rpm = final_snap.pto_rpm;
    const PowertrainDebug dbg = w.powertrain_debug(vehicle);
    r.engine_torque_demand_nm = dbg.engine_torque_demand;
    r.pto_clutch_slip = dbg.pto_clutch_slip;
    r.pto_clutch_torque_nm = dbg.pto_clutch_torque;
    return r;
}

} // namespace

TEST_CASE("tractor PTO (540 mode) driving a viscous load_dyno reaches a steady state whose measured "
          "dissipation rate matches c*omega_out^2, with balance_error bounded throughout",
          "[validation][vehicle][powertrain][drivetrain]") {
    const vehicle::VehicleDesc desc = load_tractor_with_pto_dyno();
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, kChassisMass, kChassisHalfExtents, kChassisOriginHeight, 0.0, 4);
    World& w = *rig.world;
    ps_test::settle(w, 1.0);

    w.set_control("gear_request", 0.0); // Neutral - the drive path is irrelevant to the PTO branch
    w.set_control("park_lock", 1.0);
    w.set_control("throttle", 0.35);
    w.set_control("pto", 1.0); // engage; "pto_speed" left at its declared default (540.0)

    // engage_time_s=1.5 (tractor.json) plus time for the network's own
    // transient to decay - 8 s is generous for a first-order viscous-damped
    // single-inertia-pair system at these J/c scales.
    const SteadyResult r = run_and_measure(w, rig.vehicle, /*settle_seconds=*/8.0, /*window_seconds=*/3.0);

    INFO("540 mode: mean_omega_out=" << r.mean_omega_out_rad_s << " rad/s (" << (r.mean_omega_out_rad_s * kRadPerSecToRpm)
                                      << " rpm) max|domega_out| in window=" << r.max_abs_domega_out);
    INFO("540 mode: window_drift_omega_out=" << r.window_drift_omega_out << " window_drift_engine_rpm=" << r.window_drift_engine_rpm
                                              << " (first/last omega_out=" << r.first_omega_out_rad_s << "/" << r.last_omega_out_rad_s
                                              << " rad/s, first/last engine_rpm=" << r.first_engine_rpm << "/" << r.last_engine_rpm
                                              << ")");
    INFO("540 mode: measured_power=" << r.measured_power_w << " W  predicted (c*mean(omega^2))=" << r.predicted_power_w << " W  ratio="
                                      << (r.measured_power_w / r.predicted_power_w));
    INFO("540 mode: engine_rpm=" << r.engine_rpm << " pto_rpm=" << r.pto_rpm << " ratio=" << (r.engine_rpm / r.pto_rpm)
                                  << " engine_torque_demand=" << r.engine_torque_demand_nm << " N*m"
                                  << " pto_clutch_torque=" << r.pto_clutch_torque_nm << " N*m pto_clutch_slip=" << r.pto_clutch_slip
                                  << " rad/s");
    INFO("540 mode: measured_gear_loss_rate=" << r.measured_gear_loss_rate_w
                                               << " W  predicted ((1-eta)*|torque|*|omega_in|)=" << r.predicted_gear_loss_rate_w
                                               << " W");
    INFO("worst relative balance_error = " << r.worst_balance_error);

    CHECK(w.powertrain_state(rig.vehicle).pto_engagement == Catch::Approx(1.0).margin(1e-6));
    CHECK(w.powertrain_state(rig.vehicle).pto_mode == Catch::Approx(540.0));
    CHECK(r.max_abs_domega_out < 0.02); // rad/s per tick - not still lurching tick-to-tick
    CHECK(r.window_drift_omega_out <= 1e-3);   // genuinely steady over the WHOLE window, not just tick-to-tick
    CHECK(r.window_drift_engine_rpm <= 1e-3);
    CHECK(std::abs(r.pto_clutch_slip) < 1e-9); // the row is locked (stuck), not slipping, at steady state
    CHECK((r.engine_rpm / r.pto_rpm) == Catch::Approx(kPtoRatio540).epsilon(1e-9));
    CHECK(r.measured_power_w > 0.0);
    CHECK(r.measured_power_w == Catch::Approx(r.predicted_power_w).epsilon(0.02));
    CHECK(r.measured_gear_loss_rate_w == Catch::Approx(r.predicted_gear_loss_rate_w).epsilon(1e-3));
}

TEST_CASE("tractor PTO switched to 1000 mode after a clean disengage reaches a genuinely steady operating "
          "point (WOT, on the same soft_cut limiter band as the 540-mode case) with the same c*omega_out^2 "
          "identity",
          "[validation][vehicle][powertrain][drivetrain]") {
    const vehicle::VehicleDesc desc = load_tractor_with_pto_dyno();
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, kChassisMass, kChassisHalfExtents, kChassisOriginHeight, 0.0, 4);
    World& w = *rig.world;
    ps_test::settle(w, 1.0);

    w.set_control("gear_request", 0.0);
    w.set_control("park_lock", 1.0);
    w.set_control("throttle", 0.35);
    w.set_control("pto", 1.0); // 540 mode first (default pto_speed)

    const SteadyResult r540 = run_and_measure(w, rig.vehicle, 8.0, 3.0);
    INFO("540 mode (pre-switch): measured_power=" << r540.measured_power_w << " W  predicted=" << r540.predicted_power_w
                                                   << " W  engine_rpm=" << r540.engine_rpm << " window_drift_omega_out="
                                                   << r540.window_drift_omega_out
                                                   << " window_drift_engine_rpm=" << r540.window_drift_engine_rpm);
    CHECK(r540.window_drift_omega_out <= 1e-3);
    CHECK(r540.window_drift_engine_rpm <= 1e-3);
    CHECK(r540.measured_power_w == Catch::Approx(r540.predicted_power_w).epsilon(0.02));

    // Disengage and wait for the mode-switch eligibility rule (e == 0 AND
    // |omega_out| < 1 rad/s, tests/unit/test_powertrain_pto.cpp's own Test 4)
    // to actually become true before requesting 1000 mode - a mode change
    // requested any earlier would (per spec) simply be refused and stay
    // pending at 540.
    w.set_control("pto", 0.0);
    const real dt = w.dt();
    const int max_wait_ticks = static_cast<int>(std::lround(15.0 / dt));
    bool eligible = false;
    for (int i = 0; i < max_wait_ticks; ++i) {
        w.step();
        const PowertrainSnapshot& snap = w.powertrain_state(rig.vehicle);
        const real omega_out = snap.pto_rpm * kRpmToRadPerSec;
        if (snap.pto_engagement == 0.0 && std::abs(omega_out) < 1.0) {
            eligible = true;
            break;
        }
    }
    REQUIRE(eligible); // mode-switch window must actually be reached within a generous wait

    // Coordinator review round 2: at the PREVIOUS throttle (0.35, same as
    // 540 mode), 1000 mode's smaller ratio (1.95 vs. 3.61) reflects a much
    // larger share of the dyno's viscous load onto the crank
    // (c_effective = c/ratio^2: c/3.80 here vs. c/13.03 for 540 mode),
    // landing the operating point on a nearly-flat stretch of the diesel's
    // WOT torque curve well below the soft_cut band (2200-2400 rpm per
    // data/engines/diesel_i4_4l5_turbo.json) - measured (temporary trace,
    // since reverted): omega_out fell 50.877 -> 47.660 rad/s and engine rpm
    // fell 947 -> 887 over the old 3 s window, and even a 60 s settle only
    // reached 696.07 rpm while still creeping. Commanding full throttle
    // instead puts 1000 mode on the SAME soft_cut limiter band the 540-mode
    // case already relies on for its own steadiness - verified below by the
    // window-drift gate actually passing, not assumed from the WOT/limiter
    // reasoning alone. Measured result at WOT (this file's own run):
    // engine 2243.47 rpm (inside the 2200-2400 rpm band), pto.out 120.48
    // rad/s, both window-drift measures exactly 0.0 over the 3 s window.
    w.set_control("pto_speed", 1000.0);
    w.set_control("throttle", 1.0);
    w.set_control("pto", 1.0);

    const SteadyResult r1000 = run_and_measure(w, rig.vehicle, 8.0, 3.0);
    INFO("1000 mode (post-switch, WOT): mean_omega_out=" << r1000.mean_omega_out_rad_s
                                                          << " rad/s max|domega_out|=" << r1000.max_abs_domega_out);
    INFO("1000 mode: window_drift_omega_out=" << r1000.window_drift_omega_out
                                               << " window_drift_engine_rpm=" << r1000.window_drift_engine_rpm
                                               << " (first/last omega_out=" << r1000.first_omega_out_rad_s << "/"
                                               << r1000.last_omega_out_rad_s << " rad/s, first/last engine_rpm="
                                               << r1000.first_engine_rpm << "/" << r1000.last_engine_rpm << ")");
    INFO("1000 mode: measured_power=" << r1000.measured_power_w << " W  predicted=" << r1000.predicted_power_w
                                       << " W  ratio=" << (r1000.measured_power_w / r1000.predicted_power_w));
    INFO("1000 mode: engine_rpm=" << r1000.engine_rpm << " pto_rpm=" << r1000.pto_rpm
                                   << " ratio=" << (r1000.engine_rpm / r1000.pto_rpm)
                                   << " engine_torque_demand=" << r1000.engine_torque_demand_nm
                                   << " N*m pto_clutch_torque=" << r1000.pto_clutch_torque_nm
                                   << " N*m pto_clutch_slip=" << r1000.pto_clutch_slip << " rad/s");
    INFO("1000 mode: measured_gear_loss_rate=" << r1000.measured_gear_loss_rate_w
                                                << " W  predicted ((1-eta)*|torque|*|omega_in|)="
                                                << r1000.predicted_gear_loss_rate_w << " W");
    INFO("worst relative balance_error = " << r1000.worst_balance_error);

    CHECK(w.powertrain_state(rig.vehicle).pto_engagement == Catch::Approx(1.0).margin(1e-6));
    CHECK(w.powertrain_state(rig.vehicle).pto_mode == Catch::Approx(1000.0));
    CHECK(r1000.max_abs_domega_out < 0.02);
    CHECK(r1000.window_drift_omega_out <= 1e-3);   // the actual fix under review: this used to fail badly
    CHECK(r1000.window_drift_engine_rpm <= 1e-3);
    CHECK(std::abs(r1000.pto_clutch_slip) < 1e-9);
    CHECK((r1000.engine_rpm / r1000.pto_rpm) == Catch::Approx(kPtoRatio1000).epsilon(1e-9));
    CHECK(r1000.measured_power_w > 0.0);
    CHECK(r1000.measured_power_w == Catch::Approx(r1000.predicted_power_w).epsilon(0.02));
    CHECK(r1000.measured_gear_loss_rate_w == Catch::Approx(r1000.predicted_gear_loss_rate_w).epsilon(1e-3));

    // NOT asserted: which mode spins pto.out faster. The two runs now use
    // DIFFERENT throttle commands (0.35 for 540, 1.0 for 1000 after the
    // switch - see the file header and the comment above), specifically
    // BECAUSE the same throttle does not give 1000 mode a genuine steady
    // state at this load/gearing combination, so the two absolute
    // omega_out/engine_rpm values are not a like-for-like comparison and no
    // inequality between them is asserted; both are reported via INFO above
    // for anyone who wants them.
}
