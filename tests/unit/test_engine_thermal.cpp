// tests/unit/test_engine_thermal.cpp - engine thermal plan T1 unit tests U1-U8
// (out/engine_thermal_plan.md section 12, vault PHYS-063). White-box: core/src is
// on the include path. Tags [thermal][drivetrain].
//
// All numbers use the shipped class defaults through the A25A thermal file
// (data/engine_thermal/toyota_a25a_fks.json) and the sedan cooling system
// (data/cooling_systems/sedan.json); [C]/[D]/[AS] tags are in those files.
//
// Sabotage record: each sabotage below was applied once to core/src/drivetrain/
// engine_thermal.cpp (or .h), the test run, and the measured failure recorded here.
//   U1: the Walther constant 0.7 replaced by 0.6 in the fit and the inverse: nu(20 C) off by 0.72 (bound 0.1), nu(0 C) by 6.65 (0.3), nu(-30 C) by 245 (3). (A natural-log inner log is NOT a sabotage: the fit is invariant under the log base, checked numerically.)
//   U2: density correction dropped from the dynamic viscosity: k(20 C) off by 0.0456 (bound 0.002), k(0 C) by 0.103, k(-20 C) by 0.274; the k(-30 C) and monotone checks fail likewise.
//   U3: sign of the oil-coolant coupling a12 flipped: coolant error 173.07 K / oil 889.35 K against bound 0.05 K, error ratio 1.0000002 (bound 1.8..2.2).
//   U4: radiator heat booked at the OLD coolant temperature: worst |balance_error| / sum|flows| = 8.09e-6 (bound 1e-9).
//   U5: wax lag removed (wax = engine temperature): opening starts at 89.0 C (bound 91.0 +- 0.2), closes at 87.0 C (bound 85.0 +- 0.2).
//   U6: Cmin and Cmax swapped: the conductance with a huge coolant capacity rate is 3.77 W/K against the expected 1057.34 (bound 1e-6 relative); the other 23 assertions still pass, that one fails.
//   U7: single 95.5 C switching threshold instead of the 98/93 C band: 482 failed assertions (switch-on and switch-off points and the in-band checks).
//   U8: a local std::vector allocated in step(): own_violation_count() = 1000 over the 1000 armed ticks (bound 0); the negative control inside the test proves the hook sees job-side allocations.

#include "drivetrain/engine_thermal.h"

#include "ps/io/engine_thermal_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
using Catch::Approx;

namespace {

const std::string kRoot = PS_SOURCE_DIR;

EngineThermalConfig sedan_a25a_config() {
    EngineThermalConfig c;
    c.engine = io::load_engine_thermal_json(kRoot + "/data/engine_thermal/toyota_a25a_fks.json");
    c.cooling = io::load_cooling_system_json(kRoot + "/data/cooling_systems/sedan.json");
    return c;
}

constexpr real kC = kCelsiusToKelvin;

// SplitMix64 counter hash -> uniform [0, 1).
real unit_hash(std::uint64_t counter, std::uint64_t salt) {
    std::uint64_t z = counter * 0x9E3779B97F4A7C15ULL + salt * 0xD1B54A32D192ED03ULL + 0x9E3779B97F4A7C15ULL;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    z = z ^ (z >> 31);
    return static_cast<real>(z >> 11) * (1.0 / 9007199254740992.0);
}

volatile real* g_sink = nullptr;

} // namespace

TEST_CASE("U1 Walther viscosity fit", "[thermal][drivetrain]") {
    const EngineThermal th(sedan_a25a_config());
    CHECK(th.kinematic_viscosity_mm2_s(40.0 + kC) == Approx(69.1).epsilon(1e-9));
    CHECK(th.kinematic_viscosity_mm2_s(100.0 + kC) == Approx(12.0).epsilon(1e-9));
    CHECK(std::abs(th.kinematic_viscosity_mm2_s(20.0 + kC) - 171.8) < 0.1);
    CHECK(std::abs(th.kinematic_viscosity_mm2_s(0.0 + kC) - 562.6) < 0.3);
    // Plan 6.1 table rows (the 0.7 constant of ASTM D341 is what separates these from a differently built fit).
    CHECK(std::abs(th.kinematic_viscosity_mm2_s(-30.0 + kC) - 7356.0) < 3.0);
    CHECK(std::abs(th.kinematic_viscosity_mm2_s(60.0 + kC) - 33.83) < 0.02);
    CHECK(std::abs(th.kinematic_viscosity_mm2_s(120.0 + kC) - 8.16) < 0.01);
    // Monotone decreasing with temperature over the whole grid.
    real prev = 1e30;
    for (int t = -40; t <= 150; t += 5) {
        const real nu = th.kinematic_viscosity_mm2_s(t + kC);
        CHECK(nu < prev);
        prev = nu;
    }
}

TEST_CASE("U2 friction scale k(T_oil)", "[thermal][drivetrain]") {
    const EngineThermal th(sedan_a25a_config());
    CHECK(th.friction_scale_from_curve(95.0 + kC) == 1.0);
    for (int t = 95; t <= 150; ++t) CHECK(th.friction_scale_from_curve(t + kC) == 1.0);
    CHECK(std::abs(th.friction_scale_from_curve(20.0 + kC) - 2.338) < 0.002);
    CHECK(std::abs(th.friction_scale_from_curve(0.0 + kC) - 3.847) < 0.003);
    CHECK(std::abs(th.friction_scale_from_curve(-20.0 + kC) - 7.92) < 0.01);
    CHECK(std::abs(th.friction_scale_from_curve(-30.0 + kC) - 12.72) < 0.02); // no cap (owner 2026-10-05)
    // Never below the floor, monotone non-increasing with temperature.
    real prev = 1e30;
    for (int t = -40; t <= 150; t += 5) {
        const real k = th.friction_scale_from_curve(t + kC);
        CHECK(k >= 1.0);
        CHECK(k <= prev);
        prev = k;
    }
    // The grid curve follows the exact law between knots (convex, linear interpolation errs high).
    for (const real t : {7.0, 33.0, 57.0, 78.0, 91.0}) {
        const real exact = th.friction_scale_exact(t + kC);
        CHECK(th.friction_scale_from_curve(t + kC) == Approx(exact).epsilon(0.03));
    }
}

namespace {

struct ExactLinear {
    // dT/dt = A T + b for the two lumps with a closed thermostat (constant conductances).
    real a11, a12, a21, a22, b1, b2;
    void at(real t, real t0_e, real t0_o, real& te, real& to) const {
        const real det = a11 * a22 - a12 * a21;
        const real tss_e = (-a22 * b1 + a12 * b2) / det;
        const real tss_o = (a21 * b1 - a11 * b2) / det;
        const real d_e = t0_e - tss_e, d_o = t0_o - tss_o;
        const real tr = a11 + a22;
        const real disc = std::sqrt(tr * tr - 4.0 * det);
        const real l1 = 0.5 * (tr + disc), l2 = 0.5 * (tr - disc);
        const real e1 = std::exp(l1 * t), e2 = std::exp(l2 * t);
        // exp(At) = (e1 (A - l2 I) - e2 (A - l1 I)) / (l1 - l2)
        const real m11 = (e1 * (a11 - l2) - e2 * (a11 - l1)) / (l1 - l2);
        const real m12 = (e1 * a12 - e2 * a12) / (l1 - l2);
        const real m21 = (e1 * a21 - e2 * a21) / (l1 - l2);
        const real m22 = (e1 * (a22 - l2) - e2 * (a22 - l1)) / (l1 - l2);
        te = tss_e + m11 * d_e + m12 * d_o;
        to = tss_o + m21 * d_e + m22 * d_o;
    }
};

// Runs the constant-input case and returns {coolant, oil} error vs the exact solution after `seconds`.
void run_constant_case(real dt, real seconds, real& err_e, real& err_o) {
    EngineThermalConfig cfg = sedan_a25a_config();
    cfg.engine.thermostat.open_k = 2000.0; // thermostat never opens: x = 0, G_rad = 0, constant conductances
    cfg.engine.thermostat.full_open_k = 2100.0;
    cfg.engine.water_jacket.enabled = false; // the exact solution below is the two-lump model (jacket: J4)
    EngineThermal th(cfg);
    const real t_amb = 20.0 + kC;
    th.set_env(ThermalEnv{t_amb, 101325.0, 0.0, 0.0});
    th.set_state(EngineThermalState{20.0, 20.0});
    const real p_wall = 8000.0, p_fric = 3000.0;
    const real released_per_tick = p_wall * dt / cfg.engine.wall_heat_fraction;
    const std::int64_t n = static_cast<std::int64_t>(std::llround(seconds / dt));
    for (std::int64_t i = 0; i < n; ++i) {
        th.accumulate(released_per_tick, p_fric * dt, dt);
        th.step(0.0);
    }
    const EngineThermalDesc& e = cfg.engine;
    const real g_oc = e.oil_capacity_j_k / e.oil_coolant_time_constant_s;
    ExactLinear ex;
    ex.a11 = -(e.block_air_w_k + g_oc) / e.engine_capacity_j_k;
    ex.a12 = g_oc / e.engine_capacity_j_k;
    ex.a21 = g_oc / e.oil_capacity_j_k;
    ex.a22 = -(e.oil_air_w_k + g_oc) / e.oil_capacity_j_k;
    ex.b1 = (p_wall + (1.0 - e.friction_to_oil_fraction) * p_fric + e.block_air_w_k * t_amb) / e.engine_capacity_j_k;
    ex.b2 = (e.friction_to_oil_fraction * p_fric + e.oil_air_w_k * t_amb) / e.oil_capacity_j_k;
    real te = 0.0, to = 0.0;
    ex.at(seconds, t_amb, t_amb, te, to);
    err_e = std::abs(th.engine_temperature_k() - te);
    err_o = std::abs(th.oil_temperature_k() - to);
}

} // namespace

TEST_CASE("U3 backward Euler against the exact matrix exponential", "[thermal][drivetrain]") {
    real e1 = 0.0, o1 = 0.0, e2 = 0.0, o2 = 0.0;
    run_constant_case(1.0 / 240.0, 3600.0, e1, o1);
    run_constant_case(1.0 / 480.0, 3600.0, e2, o2);
    INFO("coolant error dt=1/240: " << e1 << "  dt=1/480: " << e2 << "  oil: " << o1 << " / " << o2);
    CHECK(e1 < 0.05);
    CHECK(o1 < 0.05);
    const real ratio = e1 / e2;
    CHECK(ratio > 1.8);
    CHECK(ratio < 2.2);
}

TEST_CASE("U4 ledger closure over random inputs", "[thermal][drivetrain]") {
    EngineThermal th(sedan_a25a_config());
    th.set_state(EngineThermalState{30.0, 30.0});
    const real dt = 1.0 / 240.0;
    real worst_rel = 0.0;
    for (std::uint64_t i = 0; i < 100000; ++i) {
        const real amb = 255.0 + 55.0 * unit_hash(i, 1);
        const real ram = 40.0 * unit_hash(i / 50, 2);
        const real rpm = 7000.0 * unit_hash(i / 200, 3);
        th.set_env(ThermalEnv{amb, 101325.0, ram, rpm});
        th.accumulate(2.0e5 * unit_hash(i, 4) * dt, 2.0e4 * unit_hash(i, 5) * dt, dt);
        th.step(rpm);
        if (i % 997 == 0 && th.ledger().abs_flow_sum > 0.0) {
            worst_rel = std::max(worst_rel, std::abs(th.ledger().balance_error) / th.ledger().abs_flow_sum);
        }
    }
    const EngineThermalLedger& l = th.ledger();
    worst_rel = std::max(worst_rel, std::abs(l.balance_error) / l.abs_flow_sum);
    INFO("worst |balance_error| / sum|flows| = " << worst_rel);
    CHECK(worst_rel <= 1e-9);
    CHECK(l.wall_in > 0.0);
    CHECK(l.friction_in > 0.0);
    CHECK(l.radiator_out != 0.0);
}

TEST_CASE("U5 thermostat lag and hysteresis", "[thermal][drivetrain]") {
    const EngineThermalConfig cfg = sedan_a25a_config();
    const EngineThermalDesc::Thermostat& th = cfg.engine.thermostat;
    const real dt = 1.0 / 240.0;
    real wax = 80.0 + kC, play = 80.0 + kC, x = 0.0;
    real t_e = 80.0 + kC;
    real t_open_rising = -1.0, t_close_falling = -1.0;
    bool opened = false;
    // Rising ramp 80 -> 105 C at 0.1 K/s.
    while (t_e < 105.0 + kC) {
        EngineThermal::thermostat_update(th, dt, t_e, wax, play, x);
        if (!opened && x > 0.0) {
            opened = true;
            t_open_rising = t_e - kC;
        }
        t_e += 0.1 * dt;
    }
    REQUIRE(opened);
    // Falling ramp back to 80 C.
    bool closed = false;
    while (t_e > 80.0 + kC) {
        EngineThermal::thermostat_update(th, dt, t_e, wax, play, x);
        if (!closed && x == 0.0) {
            closed = true;
            t_close_falling = t_e - kC;
        }
        t_e -= 0.1 * dt;
    }
    REQUIRE(closed);
    INFO("opening starts at " << t_open_rising << " C rising, closes at " << t_close_falling << " C falling");
    CHECK(std::abs(t_open_rising - 91.0) < 0.2);
    CHECK(std::abs(t_close_falling - 85.0) < 0.2);
}

TEST_CASE("U6 radiator effectiveness (eps-NTU, cross-flow unmixed)", "[thermal][drivetrain]") {
    for (const real ntu : {0.1, 0.5, 1.0, 3.0, 8.0}) {
        CHECK(EngineThermal::radiator_effectiveness(ntu, 0.0) == Approx(1.0 - std::exp(-ntu)).epsilon(1e-12));
        CHECK(EngineThermal::radiator_effectiveness(ntu, 1e-10) == Approx(1.0 - std::exp(-ntu)).epsilon(1e-12));
        // Continuity of the general branch toward Cr -> 0.
        CHECK(EngineThermal::radiator_effectiveness(ntu, 1e-6) == Approx(1.0 - std::exp(-ntu)).epsilon(1e-4));
    }
    // Incropera Table 11.3 formula evaluated at NTU = 1, Cr = 1: 1 - exp(exp(-1) - 1) = 0.468536
    // (the plan's 0.46848 is a slip of 5.6e-5; the formula's value is pinned).
    CHECK(std::abs(EngineThermal::radiator_effectiveness(1.0, 1.0) - 0.468536) < 1e-5);
    // Effectiveness grows with NTU and shrinks with Cr.
    CHECK(EngineThermal::radiator_effectiveness(2.0, 0.5) > EngineThermal::radiator_effectiveness(1.0, 0.5));
    CHECK(EngineThermal::radiator_effectiveness(2.0, 0.5) > EngineThermal::radiator_effectiveness(2.0, 1.0));

    const EngineThermal th(sedan_a25a_config());
    CHECK(th.radiator_conductance(1.0, 0.0) == 0.0); // thermostat shut: no coolant flow
    CHECK(th.radiator_conductance(0.0, 2.0) == 0.0); // no air
    CHECK(th.radiator_conductance(-1.0, 2.0) == 0.0);
    // Unbalanced stream: G_rad = eps * Cmin, Cmin = the air side here.
    const real g = th.radiator_conductance(2.0, 2.0);
    const real c_air = 2.0 * 1006.0;
    CHECK(g > 0.0);
    CHECK(g < c_air); // eps < 1
    // With the reference air flow and a huge coolant capacity rate the conductance is eps_ntu * air capacity rate.
    const real ntu = 1500.0 / c_air;
    CHECK(th.radiator_conductance(2.0, 1.0e12) == Approx((1.0 - std::exp(-ntu)) * c_air).epsilon(1e-6));
}

TEST_CASE("U7 fan hysteresis", "[thermal][drivetrain]") {
    const EngineThermalConfig cfg = sedan_a25a_config();
    const CoolingSystemDesc::Fan& fan = cfg.cooling.fan;
    bool on = false;
    real switched_on_at = -1.0, switched_off_at = -1.0;
    for (real t = 90.0; t <= 105.0; t += 0.01) {
        const bool next = EngineThermal::fan_update(fan, on, t + kC);
        if (next != on) {
            REQUIRE(next); // only an "on" edge can occur while rising
            switched_on_at = t;
        }
        on = next;
        if (t < 97.9) CHECK_FALSE(on); // nothing inside the band while rising
    }
    CHECK(on);
    for (real t = 105.0; t >= 85.0; t -= 0.01) {
        const bool next = EngineThermal::fan_update(fan, on, t + kC);
        if (next != on) {
            REQUIRE_FALSE(next);
            switched_off_at = t;
        }
        on = next;
        if (t > 93.1) CHECK(on); // no switching inside the band while falling
    }
    CHECK_FALSE(on);
    INFO("on at " << switched_on_at << " C, off at " << switched_off_at << " C");
    CHECK(std::abs(switched_on_at - 98.0) < 0.02);
    CHECK(std::abs(switched_off_at - 93.0) < 0.02);
}

TEST_CASE("U8 zero allocation of step()", "[thermal][drivetrain][debug_alloc][jobs]") {
    if (!ps::jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc hook compiled out (ASan/TSan): U8 asserts nothing here");
        return;
    }
    EngineThermal th(sedan_a25a_config());
    th.set_env(ThermalEnv{293.15, 101325.0, 20.0, 3000.0});
    const real dt = 1.0 / 240.0;
    ps::jobs::JobSystem pool(2);
    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    pool.parallel_for(1, 1, [&](std::size_t, std::size_t) {
        for (int i = 0; i < 1000; ++i) {
            th.begin_tick();
            th.accumulate(1.0e3, 2.0e2, dt);
            th.step(3000.0);
        }
    });
    ps::jobs::debug_alloc::set_armed(false);
    CHECK(ps::jobs::debug_alloc::own_violation_count() == 0);

    // Negative control: the hook sees an allocation made inside a job.
    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    pool.parallel_for(1, 1, [&](std::size_t, std::size_t) {
        std::vector<real>* v = new std::vector<real>(64, 1.0);
        g_sink = v->data(); // escapes, so the allocation cannot be elided
        th.accumulate(1.0, 1.0, dt);
        th.step(3000.0);
        delete v;
    });
    ps::jobs::debug_alloc::set_armed(false);
    CHECK(ps::jobs::debug_alloc::own_violation_count() > 0);
}
