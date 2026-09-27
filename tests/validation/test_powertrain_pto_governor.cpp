// tests/validation/test_powertrain_pto_governor.cpp — PHYS-033 P4 I4 C12:
// whole-vehicle acceptance test for the tractor's new all-speed diesel
// governor with droop (data/engines/diesel_i4_4l5_turbo.json's "governor"
// block, docs/data_formats/engine.md), exercised as a dynamometer/lug-curve
// test through the PTO - the classic way a real tractor's governor curve is
// measured (a PTO dynamometer applies a KNOWN constant braking torque and
// the engine settles at whatever rpm its own governed torque output balances
// that load). Mirrors test_powertrain_pto.cpp's rig/settle/balance_error
// pattern (that file's own header comment already covers the raw pto-row
// mechanics, the mode-switch eligibility rule and the pto row never being
// this test's own bottleneck) - this file is entirely new coverage for the
// governor itself, not a repeat of that file's own assertions.
//
// Independent reference (derived from first principles against
// core/src/drivetrain/engine_torque_map.cpp's actual pre_solve() and
// core/src/drivetrain/components.cpp's Pto::pre_solve/post_solve +
// LoadDyno::pre_solve, NOT copied from either): at a genuine steady state
// (zero angular acceleration everywhere), the PTO's own friction row is
// locked (its capacity, 700 N*m per data/vehicles/tractor.json, is never
// approached below - the maximum load this file ever commands is well
// under that, see the 120% figure in the file-level numbers below), so
// omega_in = ratio*omega_out exactly. Torque balance at the crank node then
// gives:
//
//     m(n) + theta_gov(n)*(w(n) - m(n)) = (2 - eta) * tau_dyno / ratio
//
// where n is the engine crank rpm, w/m are the engine's own WOT/motoring
// torque curves (Curve1D, read from the loaded VehicleDesc - never hand-
// copied), eta is the pto's efficiency (0.96), ratio is the active pto
// mode's ratio (3.61 for "540", 1.95 for "1000"), tau_dyno is the load_dyno's
// commanded Brake-mode capacity, and theta_gov(n) = clamp((n_nl - n)/band,
// 0, 1) is the governor's own throttle-replacement law (n_nl = idle_rpm +
// lever*(high_idle_rpm - idle_rpm), band = droop*rated_rpm - see
// docs/data_formats/engine.md). The LHS is exactly what EngineTorqueMap's
// own pre_solve() computes as PreSolveResult::torque_demand at steady
// running state (no limiter/idle-PI/starter contribution at these rpms, all
// well above idle_rpm=850 and below the 2650 rpm fuel_cut overspeed cutoff)
// - so PowertrainDebug::engine_torque_demand gives a SECOND, independent
// cross-check of the same identity straight from the engine's own internal
// state, not just from the measured rpm mapped back through the WOT curve.
//
// Derivation of the RHS torque balance (independent of and cross-checked
// against docs/data_formats/engine.md, from components.cpp directly): the
// pto row's own multiplier lambda_pto (a=in/crank, b=out) satisfies, at the
// shared "pto.out"/"dyno.shaft" node (rigidly joined by this file's own
// ShaftDesc, see build_governed_pto_rig() below), ratio*lambda_pto =
// tau_dyno (the dyno's Brake-mode row - the ONLY other torque at that node -
// has a fixed kinetic capacity it slips at exactly, once anything is
// driving the shaft, so its own multiplier is exactly tau_dyno at any
// nonzero speed). At the crank node, Pto::post_solve books TWO terms drawn
// off the crank by the row: the row's own direct reaction (lambda_pto) and,
// because efficiency < 1, a SEPARATE lagged loss term
// -(1-eta)*|lambda_pto|*sign(omega_in) applied at the SAME "in" port
// (components.cpp's Pto::pre_solve/post_solve) - so the crank's total load
// is lambda_pto*(1 + (1-eta)) = lambda_pto*(2-eta) = tau_dyno*(2-eta)/ratio.
//
// "100% of full-rack PTO torque": at full lever (hand_throttle=1), the
// governor's droop band is exactly [rated_rpm, high_idle_rpm] = [2200,
// 2400] (band = droop*rated_rpm = 0.0909*2200 = 199.98 ~= 200 =
// high_idle_rpm - rated_rpm - this file's own bisection reference never
// hard-codes this, it is a read of the JSON's own governor block). The
// natural "100%" reference point is theta_gov == 1 exactly, i.e. n ==
// rated_rpm, where the LHS collapses to the pure WOT value w(rated_rpm) (440
// N*m per data/engines/diesel_i4_4l5_turbo.json - "~100 kW at 2200 rpm" per
// that file's own "source" text, 2*pi*2200/60*440 = 101.3 kW, consistent).
// So target_crank_torque(fraction) = fraction * w(rated_rpm) - a load-
// fraction expression that is RATIO-INDEPENDENT (the ratio only enters when
// converting that target crank torque into an actual tau_dyno command for a
// given pto mode), which is exactly why the same load fraction is expected
// to settle at the same crank rpm in either pto mode (see the cross-check
// test below) - and why 120% (528 N*m) drives the operating point below
// rated_rpm, onto the ascending-as-n-falls-toward-1500 stretch of the WOT
// curve where theta_gov is already clamped to 1 (a genuine "lugging" engine
// - not a defect, and not asserted-away).
//
// Bisection domain: the reference solves for n by bisection over
// [n_at_wot_peak, high_idle_rpm] (n_at_wot_peak found from the curve's own
// data, the rpm at which wot_torque_nm_vs_rpm peaks - 1500 rpm for this
// engine, never hand-copied) rather than a wider range: the WOT curve is
// unimodal (rises 500->1500, falls 1500->2400), so a target below the peak
// value has a SECOND, spurious root on the rising branch (a physically
// unstable equilibrium no real governed engine settles on - torque
// increasing with speed cannot restore itself after a perturbation) that a
// wider bisection would find instead. Restricting to [peak, high_idle_rpm]
// is exactly the region PLAN.md's "lug curve" describes (droop band plus
// the WOT curve's own falling shoulder) and is the region a real governed
// diesel actually operates in.
//
// MEASURED numbers (this file's own run, release build, hand_throttle=1.0
// throughout, tractor.json's stock 540/1000 ratios and 0.96 efficiency):
// see the CHECK/INFO lines below for the exact per-fraction table; the
// header comment intentionally does not duplicate those numbers to avoid a
// second place they could drift out of sync with the code.

#include "vehicle_test_helpers.h"

#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/vehicle_io.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/validation/CMakeLists.txt)"
#endif

namespace {

constexpr real kPi = 3.14159265358979323846;
constexpr real kRadPerSecToRpm = real(60.0) / (real(2.0) * kPi);
constexpr real kRpmToRadPerSec = real(1.0) / kRadPerSecToRpm;

constexpr real kChassisMass = 6000.0;
const Vec3 kChassisHalfExtents{1.2, 0.35, 0.9};
constexpr real kChassisOriginHeight = 1.28;

vehicle::VehicleDesc load_tractor() { return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/tractor.json"); }

const ComponentDesc& find_component(const vehicle::VehicleDesc& d, const std::string& id) {
    for (const auto& c : d.powertrain.components) {
        if (c.id == id) return c;
    }
    throw std::runtime_error("component '" + id + "' not found");
}

const TorqueMapEngineDesc& engine_desc_of(const vehicle::VehicleDesc& d) {
    const auto* e = std::get_if<TorqueMapEngineDesc>(&find_component(d, "engine").params);
    if (e == nullptr) throw std::runtime_error("'engine' component is not a TorqueMapEngineDesc");
    return *e;
}

const PtoDesc& pto_desc_of(const vehicle::VehicleDesc& d) {
    const auto* p = std::get_if<PtoDesc>(&find_component(d, "pto").params);
    if (p == nullptr) throw std::runtime_error("'pto' component is not a PtoDesc");
    return *p;
}

// The rpm at which a Curve1D's y is maximal - read from the curve's own
// data, never hand-copied (1500 rpm for this engine's WOT curve).
real curve_peak_x(const vehicle::Curve1D& c) {
    std::size_t best = 0;
    for (std::size_t i = 1; i < c.y.size(); ++i) {
        if (c.y[i] > c.y[best]) best = i;
    }
    return c.x[best];
}

real theta_gov_of(real n, real n_nl, real band) { return std::clamp((n_nl - n) / band, real(0.0), real(1.0)); }

// The independent bisection reference: solves for the engine crank rpm n at
// which m(n) + theta_gov(n)*(w(n)-m(n)) == target_crank_torque_nm, for a
// governor commanded at full lever (hand_throttle == 1, so n_nl ==
// high_idle_rpm exactly) - see the file header for the derivation and why
// the search is restricted to [curve_peak_x, high_idle_rpm].
real governor_reference_rpm(const TorqueMapEngineDesc& engine, real target_crank_torque_nm) {
    const real n_nl = engine.governor.high_idle_rpm; // lever == 1
    const real band = engine.governor.droop * engine.governor.rated_rpm;
    const auto f = [&](real n) {
        const real w = engine.wot_torque_nm_vs_rpm.evaluate(n);
        const real m = engine.motoring_torque_nm_vs_rpm.evaluate(n);
        const real theta = theta_gov_of(n, n_nl, band);
        return (m + theta * (w - m)) - target_crank_torque_nm;
    };
    real lo = curve_peak_x(engine.wot_torque_nm_vs_rpm);
    real hi = n_nl;
    // f(lo) >= 0 (target never exceeds the WOT curve's own peak in this
    // file's sweep) and f(hi) < 0 (theta_gov(hi) == 0, so f(hi) == m(hi), a
    // negative motoring-friction value) - f is monotonically non-increasing
    // over [lo, hi] (see the file header), so plain bisection is exact.
    for (int i = 0; i < 80; ++i) {
        const real mid = real(0.5) * (lo + hi);
        if (f(mid) >= real(0.0)) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return real(0.5) * (lo + hi);
}

// tau_dyno for a given load FRACTION (of "full-rack", see file header) and
// pto ratio, derived from the reference torque balance above:
//   target_crank_torque(fraction) = fraction * w(rated_rpm)   [ratio-independent]
//   tau_dyno = target_crank_torque * ratio / (2 - eta)
real tau_dyno_for_fraction(const TorqueMapEngineDesc& engine, const PtoDesc& pto, real fraction, real ratio) {
    const real target_crank_torque = fraction * engine.wot_torque_nm_vs_rpm.evaluate(engine.governor.rated_rpm);
    return target_crank_torque * ratio / (real(2.0) - pto.efficiency);
}

// Builds tractor.json plus a Brake-mode load_dyno shafted onto "pto.out",
// with torque_nm fixed at the 540-mode 100% reference (computed HERE from
// the loaded desc's own curves/ratio/efficiency, never a hand-copied
// constant) and a named scale_channel so a test can command any load
// fraction, in EITHER pto mode, purely via World::set_control (see
// tau_dyno_for_fraction's own ratio scaling above - the 1000-mode
// cross-check test sets scale = fraction * ratio_1000/ratio_540, which
// multiplies this fixed 540-mode reference into the correct 1000-mode
// tau_dyno for that same load fraction).
vehicle::VehicleDesc build_governed_pto_rig_desc() {
    vehicle::VehicleDesc d = load_tractor();
    const TorqueMapEngineDesc engine = engine_desc_of(d);
    const PtoDesc pto = pto_desc_of(d);
    REQUIRE(pto.has_540);
    const real tau_dyno_100pct_540 = tau_dyno_for_fraction(engine, pto, real(1.0), pto.ratio_540);

    LoadDynoDesc dyno_desc;
    dyno_desc.mode = LoadDynoMode::Brake;
    dyno_desc.torque_nm = tau_dyno_100pct_540;
    dyno_desc.inertia_kgm2 = 0.02;
    dyno_desc.scale_channel = "pto_load_scale";
    ComponentDesc dyno_component;
    dyno_component.id = "dyno";
    dyno_component.params = dyno_desc;
    d.powertrain.components.push_back(dyno_component);

    ShaftDesc shaft;
    shaft.ports = {PortRef{"pto", "out", false}, PortRef{"dyno", "shaft", false}};
    d.powertrain.shafts.push_back(shaft);
    return d;
}

struct GovernorPoint {
    real fraction = 0.0;
    real scale_commanded = 0.0;
    real target_crank_torque_nm = 0.0;
    real tau_dyno_nm = 0.0;
    real reference_rpm = 0.0;
    real measured_engine_rpm = 0.0;
    real measured_torque_demand_nm = 0.0;
    real rel_error_rpm = 0.0;
    real window_drift_engine_rpm = 0.0;
    real worst_balance_error = 0.0;
    real governor_setpoint_rpm = 0.0;
    real hand_throttle = 0.0;
};

// Runs the rig for settle_seconds (checking balance_error every tick, as
// test_powertrain_pto.cpp/test_powertrain_mfwd.cpp both do), then measures
// over a trailing window_seconds - first-vs-last engine_rpm sample (not a
// per-tick bound: see test_powertrain_pto.cpp's own header comment on why a
// per-tick gate alone cannot see slow monotonic drift).
GovernorPoint measure_governor_point(World& w, VehicleId vehicle, real settle_seconds, real window_seconds) {
    GovernorPoint r;
    const real dt = w.dt();

    const int settle_ticks = static_cast<int>(std::lround(settle_seconds / dt));
    for (int i = 0; i < settle_ticks; ++i) {
        w.step();
        const PowertrainEnergy& e = w.powertrain_energy(vehicle);
        const real denom = std::max(real(1.0), std::abs(e.engine_out));
        r.worst_balance_error = std::max(r.worst_balance_error, std::abs(e.balance_error) / denom);
        CHECK(std::abs(e.balance_error) / denom <= 5e-8);
    }

    const int window_ticks = static_cast<int>(std::lround(window_seconds / dt));
    real first_rpm = 0.0, last_rpm = 0.0, sum_rpm = 0.0;
    for (int i = 0; i < window_ticks; ++i) {
        w.step();
        const PowertrainEnergy& e = w.powertrain_energy(vehicle);
        const real denom = std::max(real(1.0), std::abs(e.engine_out));
        r.worst_balance_error = std::max(r.worst_balance_error, std::abs(e.balance_error) / denom);
        CHECK(std::abs(e.balance_error) / denom <= 5e-8);

        const PowertrainSnapshot& snap = w.powertrain_state(vehicle);
        sum_rpm += snap.engine_rpm;
        if (i == 0) first_rpm = snap.engine_rpm;
        if (i == window_ticks - 1) last_rpm = snap.engine_rpm;
    }
    const real mean_rpm = sum_rpm / static_cast<real>(window_ticks);
    r.window_drift_engine_rpm = std::abs(last_rpm - first_rpm) / std::max(real(1.0), std::abs(mean_rpm));

    const PowertrainSnapshot& snap = w.powertrain_state(vehicle);
    const PowertrainDebug dbg = w.powertrain_debug(vehicle);
    r.measured_engine_rpm = snap.engine_rpm;
    r.measured_torque_demand_nm = dbg.engine_torque_demand;
    r.governor_setpoint_rpm = snap.governor_setpoint_rpm;
    r.hand_throttle = snap.hand_throttle;
    return r;
}

} // namespace

TEST_CASE("tractor diesel governor: PTO dynamometer load sweep (0/25/50/75/100/120% of full-rack torque) "
          "matches an independent bisection reference, rpm strictly decreasing with load, 120% genuinely lugs",
          "[validation][long][vehicle][powertrain][drivetrain]") {
    const vehicle::VehicleDesc desc = build_governed_pto_rig_desc();
    const TorqueMapEngineDesc engine = engine_desc_of(desc);
    const PtoDesc pto = pto_desc_of(desc);

    auto rig = ps_test::build_vehicle_rig_from_desc(desc, kChassisMass, kChassisHalfExtents, kChassisOriginHeight, 0.0, 4);
    World& w = *rig.world;
    ps_test::settle(w, 1.0);

    w.set_control("gear_request", 0.0); // Neutral - the drive path is irrelevant to the PTO branch
    w.set_control("park_lock", 1.0);
    w.set_control("pto_load_scale", 0.0); // no load yet
    w.set_control("hand_throttle", 0.0);  // idle first
    w.set_control("pto", 1.0);            // engage at idle, zero load (540 mode default)

    // engage_time_s=1.5 (tractor.json) plus network settle, at idle/no load.
    ps_test::settle(w, 4.0);

    // Now command full lever (WOT) - the governor's no-load setpoint n_nl
    // becomes exactly high_idle_rpm (2400 rpm).
    w.set_control("hand_throttle", 1.0);

    // scale == fraction exactly for 540 mode: the dyno's fixed torque_nm
    // field IS the 540-mode 100% reference (build_governed_pto_rig_desc()),
    // so tau_dyno_for_fraction(frac, ratio_540) / torque_nm_ref == frac (the
    // ratio_540 factor cancels against the reference's own ratio_540).
    const std::vector<real> fractions = {0.0, 0.25, 0.5, 0.75, 1.0, 1.2};
    std::vector<GovernorPoint> points;
    std::ostringstream table;
    table << "frac  tau_dyno[Nm]  ref_rpm    measured_rpm  rel_err_rpm  torque_demand[Nm]  ref_torque[Nm]  window_drift\n";

    real prev_rpm = std::numeric_limits<real>::infinity();
    for (const real frac : fractions) {
        w.set_control("pto_load_scale", frac);

        // settle_seconds chosen generously: the network's own transient at
        // these J/torque scales decays in well under 1 s, but the shadow-
        // chassis/tyre relaxation state and the governor's own droop-band
        // search need more; 6 s is ample headroom, re-measured (not tuned to
        // pass) via the window-drift gate below actually reading 0 drift.
        GovernorPoint pt = measure_governor_point(w, rig.vehicle, /*settle_seconds=*/6.0, /*window_seconds=*/2.0);
        pt.fraction = frac;
        pt.scale_commanded = frac;
        pt.target_crank_torque_nm = frac * engine.wot_torque_nm_vs_rpm.evaluate(engine.governor.rated_rpm);
        pt.tau_dyno_nm = tau_dyno_for_fraction(engine, pto, frac, pto.ratio_540);
        pt.reference_rpm = governor_reference_rpm(engine, pt.target_crank_torque_nm);
        pt.rel_error_rpm = std::abs(pt.measured_engine_rpm - pt.reference_rpm) / pt.reference_rpm;

        table << frac << "  " << pt.tau_dyno_nm << "  " << pt.reference_rpm << "  " << pt.measured_engine_rpm << "  "
              << pt.rel_error_rpm << "  " << pt.measured_torque_demand_nm << "  " << pt.target_crank_torque_nm << "  "
              << pt.window_drift_engine_rpm << "\n";
        points.push_back(pt);

        INFO("governor validation table so far:\n" << table.str());
        INFO("point: fraction=" << frac << " tau_dyno=" << pt.tau_dyno_nm << " N*m target_crank_torque="
                                 << pt.target_crank_torque_nm << " N*m reference_rpm=" << pt.reference_rpm
                                 << " measured_engine_rpm=" << pt.measured_engine_rpm << " rel_error_rpm=" << pt.rel_error_rpm
                                 << " measured_torque_demand=" << pt.measured_torque_demand_nm << " N*m"
                                 << " governor_setpoint_rpm=" << pt.governor_setpoint_rpm << " hand_throttle=" << pt.hand_throttle
                                 << " window_drift_engine_rpm=" << pt.window_drift_engine_rpm
                                 << " worst_balance_error=" << pt.worst_balance_error);

        CHECK(pt.window_drift_engine_rpm <= 1e-3); // genuinely steady over the whole window, not just tick-to-tick
        CHECK(pt.hand_throttle == Catch::Approx(1.0).margin(1e-9));
        CHECK(pt.governor_setpoint_rpm == Catch::Approx(engine.governor.high_idle_rpm).epsilon(1e-9)); // lever == 1

        // 0-100% within 0.3% (spec), 120% (genuine lugging, below rated_rpm
        // on the WOT curve's own falling shoulder) within the looser 0.5%
        // the spec calls for that regime.
        const real tol = (frac <= real(1.0)) ? real(0.003) : real(0.005);
        CHECK(pt.rel_error_rpm <= tol);

        // Second, independent cross-check straight from the engine's own
        // internal state (PowertrainDebug::engine_torque_demand), not routed
        // back through the WOT/motoring curves at all.
        CHECK(pt.measured_torque_demand_nm == Catch::Approx(pt.target_crank_torque_nm).epsilon(0.01).margin(0.5));

        // rpm strictly decreasing as load increases.
        CHECK(pt.measured_engine_rpm < prev_rpm);
        prev_rpm = pt.measured_engine_rpm;
    }

    // 120% specifically must land BELOW rated_rpm (2200 rpm) - the WOT
    // curve's own falling-shoulder "lug" regime, not the droop band proper
    // ([rated_rpm, high_idle_rpm]) - confirming the spec's "lug curve"
    // language is a real, measured phenomenon here, not merely asserted.
    REQUIRE(points.size() == fractions.size());
    CHECK(points.back().measured_engine_rpm < engine.governor.rated_rpm);
    // ...and strictly above the WOT curve's own peak-torque rpm (1500) -
    // still on the curve's monotonically-falling shoulder, not past the peak
    // into the unstable rising branch.
    CHECK(points.back().measured_engine_rpm > curve_peak_x(engine.wot_torque_nm_vs_rpm));

    INFO("final governor validation table:\n" << table.str());
    CHECK(true); // keep the table's INFO attached to a passing assertion for visibility on -s runs
}

TEST_CASE("tractor diesel governor: the SAME PTO load fraction gives the same crank rpm in 540 vs. 1000 mode "
          "(target crank torque is ratio-independent when expressed as a load fraction)",
          "[validation][vehicle][powertrain][drivetrain]") {
    const vehicle::VehicleDesc desc = build_governed_pto_rig_desc();
    const TorqueMapEngineDesc engine = engine_desc_of(desc);
    const PtoDesc pto = pto_desc_of(desc);
    REQUIRE(pto.has_1000);

    auto rig = ps_test::build_vehicle_rig_from_desc(desc, kChassisMass, kChassisHalfExtents, kChassisOriginHeight, 0.0, 4);
    World& w = *rig.world;
    ps_test::settle(w, 1.0);

    w.set_control("gear_request", 0.0);
    w.set_control("park_lock", 1.0);
    w.set_control("pto_load_scale", 0.0);
    w.set_control("hand_throttle", 0.0);
    w.set_control("pto", 1.0); // 540 mode (default)
    ps_test::settle(w, 4.0);
    w.set_control("hand_throttle", 1.0);

    constexpr real kFraction = 0.5; // mid-load: comfortably inside the droop band for both modes

    // 540 mode at the test fraction: scale == fraction exactly (see the
    // sweep test above for why).
    w.set_control("pto_load_scale", kFraction);
    const GovernorPoint p540 = measure_governor_point(w, rig.vehicle, 6.0, 2.0);
    const real target_crank_torque = kFraction * engine.wot_torque_nm_vs_rpm.evaluate(engine.governor.rated_rpm);
    const real reference_rpm = governor_reference_rpm(engine, target_crank_torque);

    INFO("540 mode: measured_engine_rpm=" << p540.measured_engine_rpm << " reference_rpm=" << reference_rpm
                                            << " window_drift=" << p540.window_drift_engine_rpm);
    CHECK(p540.window_drift_engine_rpm <= 1e-3);
    CHECK(p540.measured_engine_rpm == Catch::Approx(reference_rpm).epsilon(0.003));

    // Disengage and wait for the mode-switch eligibility rule (e == 0 AND
    // |omega_out| < 1 rad/s - tests/unit/test_powertrain_pto.cpp's own Test
    // 4) before requesting 1000 mode, exactly as test_powertrain_pto.cpp
    // does for its own mode switch.
    w.set_control("pto_load_scale", 0.0);
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
    REQUIRE(eligible);

    w.set_control("pto_speed", 1000.0);
    w.set_control("pto", 1.0);
    // scale for 1000 mode at the SAME load fraction: the dyno's torque_nm
    // field is fixed at the 540-mode 100% reference, so this multiplies it
    // into the correct 1000-mode tau_dyno for this fraction (see
    // tau_dyno_for_fraction's own derivation and the file header).
    const real scale_1000 = kFraction * pto.ratio_1000 / pto.ratio_540;
    w.set_control("pto_load_scale", scale_1000);

    const GovernorPoint p1000 = measure_governor_point(w, rig.vehicle, 6.0, 2.0);
    INFO("1000 mode: measured_engine_rpm=" << p1000.measured_engine_rpm << " reference_rpm=" << reference_rpm
                                             << " window_drift=" << p1000.window_drift_engine_rpm
                                             << " (540-mode rpm was " << p540.measured_engine_rpm << ")");
    CHECK(w.powertrain_state(rig.vehicle).pto_mode == Catch::Approx(1000.0));
    CHECK(p1000.window_drift_engine_rpm <= 1e-3);
    CHECK(p1000.measured_engine_rpm == Catch::Approx(reference_rpm).epsilon(0.003));

    // The actual cross-check: same load fraction -> same crank rpm,
    // regardless of pto mode/ratio.
    CHECK(p1000.measured_engine_rpm == Catch::Approx(p540.measured_engine_rpm).epsilon(0.003));
}
