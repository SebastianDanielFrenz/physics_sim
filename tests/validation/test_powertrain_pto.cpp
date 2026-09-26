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
// Engine operating point (rpm/torque) is reported (INFO) at both steady
// states for transparency, but NOT asserted against any hand-derived
// identity: the "in"-side torque balance through a Coulomb-becomes-rigid
// friction row's own reaction multiplier, further split by the pto's
// lagged efficiency-loss reconstruction, is materially more involved to
// derive correctly than the dyno-side identity above and is already
// covered at the raw-network level by tests/unit/test_powertrain_pto.cpp's
// efficiency-loss identity case - repeating a hand-derived version of it
// here at whole-vehicle scale would risk introducing exactly the kind of
// unverified explanation the project's own testing discipline forbids.

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

constexpr real kDynoViscous = 5.0; // N*m*s/rad - must match load_tractor_with_pto_dyno() above

struct SteadyResult {
    real measured_power_w = 0.0;  // from PowertrainEnergy::external_absorbed's delta / window
    real predicted_power_w = 0.0; // c * mean(omega_out^2) over the same window
    real mean_omega_out_rad_s = 0.0;
    real max_abs_domega_out = 0.0; // steadiness gate: largest tick-to-tick |delta omega_out| in the window
    real worst_balance_error = 0.0;
    real engine_rpm = 0.0;
    real engine_torque_demand_nm = 0.0;
    real pto_clutch_slip = 0.0;
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
    const int window_ticks = static_cast<int>(std::lround(window_seconds / dt));
    real sum_omega_sq = 0.0, sum_omega = 0.0;
    real prev_omega = w.powertrain_state(vehicle).pto_rpm * kRpmToRadPerSec;
    for (int i = 0; i < window_ticks; ++i) {
        w.step();
        const PowertrainEnergy& e = w.powertrain_energy(vehicle);
        const real denom = std::max(real(1.0), std::abs(e.engine_out));
        r.worst_balance_error = std::max(r.worst_balance_error, std::abs(e.balance_error) / denom);
        CHECK(std::abs(e.balance_error) / denom <= 5e-8);

        const PowertrainSnapshot& snap = w.powertrain_state(vehicle);
        const real omega = snap.pto_rpm * kRpmToRadPerSec;
        sum_omega += omega;
        sum_omega_sq += omega * omega;
        r.max_abs_domega_out = std::max(r.max_abs_domega_out, std::abs(omega - prev_omega));
        prev_omega = omega;
    }
    const real n = static_cast<real>(window_ticks);
    r.mean_omega_out_rad_s = sum_omega / n;
    r.predicted_power_w = kDynoViscous * (sum_omega_sq / n);

    const real energy_end = w.powertrain_energy(vehicle).external_absorbed;
    r.measured_power_w = (energy_end - energy_start) / window_seconds;

    const PowertrainSnapshot& final_snap = w.powertrain_state(vehicle);
    r.engine_rpm = final_snap.engine_rpm;
    const PowertrainDebug dbg = w.powertrain_debug(vehicle);
    r.engine_torque_demand_nm = dbg.engine_torque_demand;
    r.pto_clutch_slip = dbg.pto_clutch_slip;
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
    INFO("540 mode: measured_power=" << r.measured_power_w << " W  predicted (c*mean(omega^2))=" << r.predicted_power_w << " W  ratio="
                                      << (r.measured_power_w / r.predicted_power_w));
    INFO("540 mode: engine_rpm=" << r.engine_rpm << " engine_torque_demand=" << r.engine_torque_demand_nm << " N*m"
                                  << " pto_clutch_slip=" << r.pto_clutch_slip << " rad/s");
    INFO("worst relative balance_error = " << r.worst_balance_error);

    CHECK(w.powertrain_state(rig.vehicle).pto_engagement == Catch::Approx(1.0).margin(1e-6));
    CHECK(w.powertrain_state(rig.vehicle).pto_mode == Catch::Approx(540.0));
    CHECK(r.max_abs_domega_out < 0.02); // rad/s per tick - genuinely steady, not still decaying
    CHECK(std::abs(r.pto_clutch_slip) < 0.02); // the row is locked (stuck), not slipping, at steady state
    CHECK(r.measured_power_w > 0.0);
    CHECK(r.measured_power_w == Catch::Approx(r.predicted_power_w).epsilon(0.02));
}

TEST_CASE("tractor PTO switched to 1000 mode after a clean disengage reaches a NEW steady state with the "
          "same c*omega_out^2 identity",
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
                                                   << " W  engine_rpm=" << r540.engine_rpm);
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

    w.set_control("pto_speed", 1000.0);
    w.set_control("pto", 1.0);

    const SteadyResult r1000 = run_and_measure(w, rig.vehicle, 8.0, 3.0);
    INFO("1000 mode (post-switch): mean_omega_out=" << r1000.mean_omega_out_rad_s
                                                      << " rad/s max|domega_out|=" << r1000.max_abs_domega_out);
    INFO("1000 mode: measured_power=" << r1000.measured_power_w << " W  predicted=" << r1000.predicted_power_w
                                       << " W  ratio=" << (r1000.measured_power_w / r1000.predicted_power_w));
    INFO("1000 mode: engine_rpm=" << r1000.engine_rpm << " engine_torque_demand=" << r1000.engine_torque_demand_nm
                                   << " N*m pto_clutch_slip=" << r1000.pto_clutch_slip << " rad/s");
    INFO("worst relative balance_error = " << r1000.worst_balance_error);

    CHECK(w.powertrain_state(rig.vehicle).pto_engagement == Catch::Approx(1.0).margin(1e-6));
    CHECK(w.powertrain_state(rig.vehicle).pto_mode == Catch::Approx(1000.0));
    CHECK(r1000.max_abs_domega_out < 0.02);
    CHECK(std::abs(r1000.pto_clutch_slip) < 0.02);
    CHECK(r1000.measured_power_w > 0.0);
    CHECK(r1000.measured_power_w == Catch::Approx(r1000.predicted_power_w).epsilon(0.02));

    // NOT asserted: which mode spins pto.out faster at this FIXED throttle.
    // First guess (WRONG, caught by this file's own first run - a real
    // discriminating measurement, not an assumption left unverified): since
    // ratio_1000 (1.95) < ratio_540 (3.61), at a given CRANK speed a lower
    // ratio makes pto.out spin faster, so "1000 mode's omega_out > 540
    // mode's" looks obvious - but crank speed is NOT fixed here, it is
    // whatever the engine's own throttle map settles at against the
    // reflected load, and an ideal geared coupling's reflected damping
    // scales with 1/ratio^2: c_eff,540 = c/3.61^2 = c/13.03 vs. c_eff,1000 =
    // c/1.95^2 = c/3.80 - 1000 mode reflects a LARGER damping onto the
    // crank, pulling engine_rpm down much further (measured 2213.7 -> 887.5
    // rpm, a 2.49x drop) than the 1.95/3.61 ratio change alone would - a
    // large enough drop that mean_omega_out actually ends up LOWER in 1000
    // mode (49.2 rad/s) than 540 mode (64.2 rad/s) at this throttle. This
    // matches the 1/ratio^2 reflected-damping account in direction and
    // rough magnitude (predicted engine_rpm ratio ~ sqrt(3.80/13.03) = 0.54
    // if the diesel's torque curve were flat there; measured 887.5/2213.7 =
    // 0.40 - same order, the gap being the torque curve's own shape/idle
    // governor over that range, not re-diagnosed here since it is off this
    // component's spec). Whether 1000 spins faster than 540 therefore
    // depends on throttle/operating point, not a fixed relationship - so no
    // inequality is asserted; both absolute numbers are reported via INFO
    // above for anyone who wants them.
}
