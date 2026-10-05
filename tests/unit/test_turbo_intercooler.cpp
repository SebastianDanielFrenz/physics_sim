// tests/unit/test_turbo_intercooler.cpp -- PHYS-033 P4 I7d D4: the intercooler
// pressure drop. Optional `intercooler_pressure_drop_bar_at_max_flow` of a
// physics_sim.turbo_configuration/1 (TurboPairDesc::intercooler_dp_pa_at_max_flow):
// dp = k * (compressor flow / max flow)^2, the compressor makes head above
// ambient, the throttle (and the boost controller, the usual sensor position)
// sees that minus dp: charge_pa is the pressure AFTER the cooler,
// compressor_outlet_pa = charge_pa + dp. k = 0 / absent is bit-identical (guard
// hashes in test_turbo_pair.cpp); k != 0 is hashed through the desc fold.
//
// Fixture: tests/support/turbo_fixtures.h hyper_like_pair() (head =
// 4.5e5 * s^2 * (1 - f^2), f = flow / 1.2, target 1.8 bar gauge).
//
// Measured (release, 2026-10-04):
//   frozen rotor at 120000 rpm, air 0.9 kg/s: outlet - charge == k*0.5625 exactly (0 / 11250 /
//     30937.5 Pa for k 0 / 20000 / 55000), charge 298200.0 / 286950.0 / 267262.5 Pa.
//   steady WOT (air 0.9, 12 s): charge 281323 Pa vs target 281325 with and without k 40000
//     (22500 Pa at f 0.75); rotor 114734 rpm (head-curve inversion 114742) vs 121693 rpm
//     (121702) - the compressor makes boost + dp; wastegate 0.540 vs 0.520.
//   engine at a held 5000 rpm (little flow, so a small effect): boost 1.7745 bar either
//     way, rotor 77995 vs 78333 rpm, published pr 2.7762 vs 2.7920.
//   vehicle: hash identical at 1 and 10 workers (0x95cc15f5043e91b8), own allocations 0.
// Sabotages (each makes the named check fail, then reverted): see
//   out/i7d_d1_d4_progress.md "Sabotage results".

#include "drivetrain/engine_simulated.h"
#include "drivetrain/turbo_pair.h"

#include "mvem_fixtures.h"
#include "turbo_fixtures.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {

using namespace ps::drivetrain;
using ps::real;
namespace fx = ps::test::mvem;
namespace tb = ps::test::turbo;

constexpr real kH = 1.0 / 960.0;
constexpr real kPamb = 101325.0;
constexpr real kTamb = 288.15;
constexpr real kPi = 3.14159265358979323846;

TurboPairDesc with_dp(real dp_pa) {
    TurboPairDesc d = tb::hyper_like_pair();
    d.intercooler_dp_pa_at_max_flow = dp_pa;
    return d;
}

} // namespace

TEST_CASE("D4: outlet minus charge equals k (flow/max)^2 at a frozen state", "[turbo][drivetrain][intercooler]") {
    // Rotor held at the reference speed, air 0.9 kg/s (f = 0.75): dp = k * 0.5625, to rounding.
    for (const real k : {0.0, 20000.0, 55000.0}) {
        TurboPair p(with_dp(k));
        p.reset(kPamb, kTamb);
        const real wr = p.desc.reference_rpm * 2.0 * kPi / 60.0;
        for (int i = 0; i < 960; ++i) {
            p.energy_j = 0.5 * p.desc.rotor_inertia_kgm2 * wr * wr;
            p.step(kH, kPamb, kTamb, 0.9, 0.96, 0.0, 1.0);
        }
        const real expect_dp = k * 0.5625; // (0.9 / 1.2)^2
        CHECK(p.intercooler_dp_pa == Catch::Approx(expect_dp).epsilon(1e-12).margin(0.0));
        CHECK(p.compressor_outlet_pa - p.charge_pa == Catch::Approx(expect_dp).epsilon(1e-9).margin(1e-8));
        // the charge settles on ambient + head - dp: head(s = 1, f = 0.75) = 4.5e5 * 0.4375 = 196875 Pa
        CHECK(p.charge_pa == Catch::Approx(kPamb + 196875.0 - expect_dp).epsilon(1e-6));
        // and the published compressor pressure ratio is the OUTLET over ambient
        CHECK(p.compressor_pr == Catch::Approx((p.charge_pa + expect_dp) / kPamb).epsilon(1e-12));
        std::printf("[D4 frozen] k %.0f Pa: dp %.4f Pa, outlet - charge %.4f Pa, charge %.1f Pa, pr %.4f\n", k, p.intercooler_dp_pa,
                    p.compressor_outlet_pa - p.charge_pa, p.charge_pa, p.compressor_pr);
    }
}

TEST_CASE("D4: k = 0 is bit-identical to a pair without the key", "[turbo][drivetrain][intercooler]") {
    TurboPair a(tb::hyper_like_pair()), b(with_dp(0.0));
    a.reset(kPamb, kTamb);
    b.reset(kPamb, kTamb);
    int mismatches = 0;
    for (int i = 0; i < tb::kScriptSteps; ++i) {
        const tb::Phase ph = tb::script_phase(i);
        a.step(kH, kPamb, kTamb, ph.air, ph.exhaust, ph.port_k, ph.plate);
        b.step(kH, kPamb, kTamb, ph.air, ph.exhaust, ph.port_k, ph.plate);
        if (a.energy_j != b.energy_j || a.charge_pa != b.charge_pa || a.charge_k != b.charge_k || a.wastegate != b.wastegate ||
            a.exhaust_pa != b.exhaust_pa || a.compressor_pr != b.compressor_pr)
            ++mismatches;
        REQUIRE(b.intercooler_dp_pa == 0.0);
    }
    CHECK(mismatches == 0);
}

TEST_CASE("D4: steady WOT boost stays on target, the rotor runs faster by the drop", "[turbo][drivetrain][intercooler]") {
    // Constant WOT air 0.9 kg/s (f 0.75), exhaust 0.96: the controller holds the post-cooler pressure,
    // so the compressor must make boost + dp: s = sqrt((boost + dp) / (4.5e5 * (1 - f^2))).
    struct Steady { real charge, rpm, outlet, wastegate; };
    const auto settle = [](real k) {
        TurboPair p(with_dp(k));
        p.reset(kPamb, kTamb);
        for (int i = 0; i < 12 * 960; ++i) p.step(kH, kPamb, kTamb, 0.9, 0.96, 1200.0, 1.0); // port gas 1200 K [AS]
        return Steady{p.charge_pa, p.rpm(), p.compressor_outlet_pa, p.wastegate};
    };
    const real k = 40000.0; // 0.4 bar at max flow -> 22500 Pa at f 0.75
    const Steady s0 = settle(0.0), s1 = settle(k);
    const real target = kPamb + 1.8e5;
    const real dp = k * 0.5625;
    const real rpm_ref = 120000.0;
    const real expect0 = rpm_ref * std::sqrt(1.8e5 / 196875.0);
    const real expect1 = rpm_ref * std::sqrt((1.8e5 + dp) / 196875.0);
    std::printf("[D4 steady WOT] k 0: charge %.1f Pa (target %.1f), rotor %.0f rpm (head-curve %.0f), wastegate %.3f | k %.0f: charge %.1f Pa, rotor %.0f rpm (head-curve %.0f), outlet %.1f Pa, wastegate %.3f\n",
                s0.charge, target, s0.rpm, expect0, s0.wastegate, k, s1.charge, s1.rpm, expect1, s1.outlet, s1.wastegate);
    CHECK(s0.charge == Catch::Approx(target).margin(300.0));
    CHECK(s1.charge == Catch::Approx(target).margin(300.0)); // boost on target after the cooler
    CHECK(s1.outlet - s1.charge == Catch::Approx(dp).epsilon(1e-6));
    CHECK(s1.rpm > s0.rpm);
    CHECK(s0.rpm == Catch::Approx(expect0).epsilon(2e-3)); // independent head-curve inversion
    CHECK(s1.rpm == Catch::Approx(expect1).epsilon(2e-3));
}

TEST_CASE("D4: the engine publishes the post-cooler pressure and hashes a nonzero drop", "[turbo][drivetrain][intercooler][sim_engine]") {
    struct Out { real boost_bar, rpm, charge, pr; std::uint64_t hash; };
    const auto run = [](real k) {
        auto d = fx::electronic_engine();
        d.turbo_pair = tb::hyper_like_pair();
        d.turbo_pair->intercooler_dp_pa_at_max_flow = k;
        SimulatedEngine e(d);
        const real omega = 5000.0 * 2.0 * kPi / 60.0;
        Sensors s;
        s.omega = omega;
        s.ignition = 1.0;
        PreSolveResult r;
        for (int i = 0; i < 8 * 960; ++i) {
            r = e.pre_solve(s, ThrottleCommand{1.0, 0.0, 1.0}, TcuRequest{}, kH);
            (void)e.post_solve(omega, kH);
        }
        return Out{r.boost_bar, r.turbo_rpm, r.p_charge_pa, r.compressor_pressure_ratio, e.fold_state_hash(0)};
    };
    const Out a = run(0.0), b = run(30000.0), a2 = run(0.0);
    std::printf("[D4 engine] held 5000 rpm WOT 8 s: k 0: boost %.4f bar, rotor %.0f rpm, charge %.0f Pa, pr %.4f | k 30000: boost %.4f bar, rotor %.0f rpm, charge %.0f Pa, pr %.4f\n",
                a.boost_bar, a.rpm, a.charge, a.pr, b.boost_bar, b.rpm, b.charge, b.pr);
    // desc fold alone: two freshly built engines (no step yet, identical state) that differ only in k
    const auto fresh_hash = [](real k) {
        auto d = fx::electronic_engine();
        d.turbo_pair = tb::hyper_like_pair();
        d.turbo_pair->intercooler_dp_pa_at_max_flow = k;
        return SimulatedEngine(d).fold_state_hash(0);
    };
    CHECK(fresh_hash(0.0) != fresh_hash(30000.0));
    CHECK(fresh_hash(30000.0) == fresh_hash(30000.0));
    CHECK(a.hash == a2.hash);
    CHECK(a.hash != b.hash);       // a nonzero drop is part of the engine's hashed desc
    CHECK(b.rpm > a.rpm);          // the compressor makes boost + dp
    CHECK(b.pr > a.pr);            // published compressor pressure ratio is the outlet's
    CHECK(b.boost_bar == Catch::Approx(a.boost_bar).margin(0.05)); // the manifold boost is still on target
}
