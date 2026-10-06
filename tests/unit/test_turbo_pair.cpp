// tests/unit/test_turbo_pair.cpp -- PHYS-033 P4 I7d D1-D4: white-box tests of
// ps::drivetrain::TurboPair (core/src/drivetrain/turbo_pair.h) and of its use
// by SimulatedEngine.
//
// "No new blocks" GUARD (recorded on master 976e8d7 BEFORE any I7d change, must
// stay unchanged through D1-D4): a fixed script on a code-built hyper-like pair
// without any I7d block (tests/support/turbo_fixtures.h) - spool at WOT,
// lift-off, re-apply - folds every legacy state field of every step into an
// FNV-1a hash; a second script drives a SimulatedEngine (mvem electronic
// fixture) with that pair on a held crank and folds fold_state_hash() plus the
// published PreSolveResult fields of every substep. Both hashes are literals.

#include "drivetrain/engine_simulated.h"
#include "drivetrain/turbo_pair.h"

#include "mvem_fixtures.h"
#include "turbo_fixtures.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

using namespace ps::drivetrain;
using ps::real;
namespace fx = ps::test::mvem;
namespace tb = ps::test::turbo;

constexpr real kH = 1.0 / 960.0;
constexpr real kPamb = 101325.0;
constexpr real kTamb = 288.15;
constexpr real kPi = 3.14159265358979323846;

struct Fnv {
    std::uint64_t h = 14695981039346656037ULL;
    void pod(real v) {
        unsigned char b[sizeof v];
        std::memcpy(b, &v, sizeof v);
        for (unsigned char c : b) { h ^= c; h *= 1099511628211ULL; }
    }
};

using tb::Phase;
using tb::script_phase;
using tb::kScriptSteps;

// Legacy-field hash of the pair script (turbo_pair.h as of master 976e8d7).
std::uint64_t pair_script_hash() {
    TurboPair pair(tb::hyper_like_pair());
    pair.reset(kPamb, kTamb);
    Fnv f;
    for (int i = 0; i < kScriptSteps; ++i) {
        const Phase p = script_phase(i);
        pair.step(kH, kPamb, kTamb, p.air, p.exhaust, p.port_k, p.plate);
        f.pod(pair.energy_j); f.pod(pair.charge_pa); f.pod(pair.charge_k);
        f.pod(pair.exhaust_pa); f.pod(pair.wastegate); f.pod(pair.boost_integral);
        f.pod(pair.compressor_w); f.pod(pair.turbine_w);
    }
    return f.h;
}

// Engine-level hash: SimulatedEngine + hyper-like pair on a held 5000 rpm crank.
std::uint64_t engine_script_hash() {
    auto d = fx::electronic_engine();
    d.turbo_pair = tb::hyper_like_pair();
    SimulatedEngine e(d);
    Fnv f;
    const real omega = 5000.0 * 2.0 * kPi / 60.0;
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    for (int i = 0; i < kScriptSteps; ++i) {
        const real t = i * kH;
        const real c = t < 1.5 ? 1.0 : (t < 2.5 ? 0.0 : 1.0);
        const PreSolveResult r = e.pre_solve(s, ThrottleCommand{c, 0.0, 1.0}, TcuRequest{}, kH);
        (void)e.post_solve(omega, kH);
        f.pod(r.tau0); f.pod(r.d); f.pod(r.turbo_rpm); f.pod(r.boost_bar); f.pod(r.p_exhaust_pa);
        f.pod(r.p_charge_pa); f.pod(r.t_charge_k); f.pod(r.torque_demand); f.pod(r.p_intake_pa);
        f.pod(r.exhaust_mass_flow_kg_s); f.pod(r.throttle_position);
        const std::uint64_t sh = e.fold_state_hash(0);
        f.pod(real(sh >> 32)); f.pod(real(sh & 0xffffffffULL));
    }
    return f.h;
}

} // namespace

TEST_CASE("Guard: a pair without I7d blocks is bit-identical to master 976e8d7", "[turbo][drivetrain][guard]") {
    const std::uint64_t ph = pair_script_hash();
    const std::uint64_t eh = engine_script_hash();
    std::printf("[turbo guard] pair script hash 0x%016llx, engine script hash 0x%016llx\n",
                static_cast<unsigned long long>(ph), static_cast<unsigned long long>(eh));
    // Both re-recorded for the exhaust energy balance + exhaust wall thermal mass (no 1200 K cap, no 0.5 s
    // heat hold): TurboPair::step takes the port gas temperature instead of a fuel power, steps a hashed wall
    // temperature and computes the turbine from the post-wall gas temperature; the engine hands it the port
    // temperature of its own energy balance. Was pair 0x42be4848b80f27d5 / engine 0x90b9167102a14c12 (e38b56c);
    // the engine hash before that, on b486223, 0xc5a3c62d9ce14ffa (compressor-flow fix).
    // Re-recorded again for the review fix (test_turbo_wastegate_split.cpp): the wastegate is a port in
    // parallel with the turbine (flow split by effective area, default 2.4 x the throat) instead of a
    // 100 % bypass at full opening, and the compressor power is bounded by the head of the current rotor
    // speed; the engine hash also folds the wastegate area. Was pair 0xeb6a5f7cc032271f / engine
    // 0xd8b9c8a08566580c (9fd087c).
    CHECK(ph == 0xe7d5137fd1bbb44dULL);
    // Engine hash re-recorded 2026-10-06 for the boost-aware idle feed-forward (test_hyper_idle_recovery.cpp):
    // the script's 1 s lift at 5000 rpm runs the DFCO dashpot, whose idle plate is now solved against the
    // charge pressure while it is above ambient; the pair hash is unchanged. Was engine 0x721d080fecaad8b0.
    CHECK(eh == 0x9187a72a2a306b09ULL);
}

TEST_CASE("Guard scripts exercise the turbo: spool, lift-off, re-apply", "[turbo][drivetrain][guard]") {
    // A guard hash over a script that never boosted would pin nothing.
    TurboPair pair(tb::hyper_like_pair());
    pair.reset(kPamb, kTamb);
    real rpm_spooled = 0.0, charge_spooled = 0.0, rpm_lift = 0.0, rpm_reapplied = 0.0;
    for (int i = 0; i < kScriptSteps; ++i) {
        const Phase p = script_phase(i);
        pair.step(kH, kPamb, kTamb, p.air, p.exhaust, p.port_k, p.plate);
        if (i == 1439) { rpm_spooled = pair.rpm(); charge_spooled = pair.charge_pa; }
        if (i == 2399) rpm_lift = pair.rpm();
        if (i == kScriptSteps - 1) rpm_reapplied = pair.rpm();
    }
    std::printf("[turbo guard] rotor rpm: spooled %.0f, end of lift %.0f, re-applied %.0f; charge at spool %.0f Pa\n",
                rpm_spooled, rpm_lift, rpm_reapplied, charge_spooled);
    CHECK(rpm_spooled > 60000.0);
    CHECK(charge_spooled > kPamb + 0.8e5);
    CHECK(rpm_lift < rpm_spooled);
    CHECK(rpm_reapplied > rpm_lift);
}

// ---------------------------------------------------------------------------
// D1: sound-state plumbing (wastegate_opening, compressor_flow_kg_s,
// compressor_pressure_ratio). Literal values derived by hand from the desc's
// own head formula, not from the code under test.
// ---------------------------------------------------------------------------
TEST_CASE("D1: TurboPair publishes compressor flow, pressure ratio and wastegate position", "[turbo][drivetrain]") {
    const TurboPairDesc d = tb::hyper_like_pair();
    // (a) zero rotor speed: no head, charge stays at ambient, pr == 1 exactly;
    //     plate 0 commands the wastegate open, the actuator moves h/(tau+h) =
    //     1/(0.1*960 + 1) = 1/97 of the way in one 960 Hz step.
    {
        TurboPair p(d);
        p.reset(kPamb, kTamb);
        p.step(kH, kPamb, kTamb, 0.3, 0.3, 0.0, 0.0);
        CHECK(p.compressor_flow_kg_s == 0.3);
        CHECK(p.compressor_pr == 1.0);
        CHECK(p.wastegate == Catch::Approx(1.0 / 97.0).epsilon(1e-14));
    }
    // (b) negative air flow is no flow
    {
        TurboPair p(d);
        p.reset(kPamb, kTamb);
        p.step(kH, kPamb, kTamb, -0.2, 0.0, 0.0, 0.0);
        CHECK(p.compressor_flow_kg_s == 0.0);
    }
    // (c) rotor at s = 0.5 (nodes of the head table: s 20/40, f = 0.25 = 10/40):
    //     head = 4.5e5 * 0.25 * (1 - 0.0625) = 105468.75 Pa; charge moves
    //     h/(0.025+h) = 1/25 of the way from ambient toward ambient + head.
    {
        TurboPair p(d);
        p.reset(kPamb, kTamb);
        const real wr = d.reference_rpm * 2.0 * kPi / 60.0;
        p.energy_j = 0.5 * d.rotor_inertia_kgm2 * wr * wr * 0.25;
        p.step(kH, kPamb, kTamb, 0.3, 0.3, 0.0, 1.0);
        const real charge = kPamb + 105468.75 / 25.0;
        CHECK(p.charge_pa == Catch::Approx(charge).epsilon(1e-12));
        CHECK(p.compressor_pr == Catch::Approx(charge / kPamb).epsilon(1e-12));
        CHECK(p.compressor_flow_kg_s == 0.3);
    }
}

TEST_CASE("D1: the engine publishes the turbo sound state, 0 without a turbo", "[turbo][drivetrain][sim_engine]") {
    auto d = fx::electronic_engine();
    SimulatedEngine na(d);
    d.turbo_pair = tb::hyper_like_pair();
    SimulatedEngine boosted(d);
    const real omega = 5000.0 * 2.0 * kPi / 60.0;
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    PreSolveResult r;
    real wg_wot = -1.0, wg_lift = -1.0;
    for (int i = 0; i < 3 * 960; ++i) {
        const real t = i * kH;
        const real c = t < 1.5 ? 1.0 : 0.0;
        r = boosted.pre_solve(s, ThrottleCommand{c, 0.0, 1.0}, TcuRequest{}, kH);
        (void)boosted.post_solve(omega, kH);
        // The compressor flow is the relaxed throttle-path flow of the previous substep (I7d compressor-flow
        // fix; before it, the cylinder air of the previous substep). Its identity is checked substep by
        // substep in test_turbo_compressor_flow.cpp CF1; here only that it is a flow.
        CHECK(r.compressor_flow_kg_s >= 0.0);
        // No baseline cooler drop, no valve: outlet pressure ratio = charge / ambient (>= 1).
        CHECK(r.compressor_pressure_ratio == std::max(1.0, r.p_charge_pa / kPamb));
        CHECK(r.wastegate_opening >= 0.0);
        CHECK(r.wastegate_opening <= 1.0);
        if (i == 1439) wg_wot = r.wastegate_opening;
        if (i == 2879) wg_lift = r.wastegate_opening;
    }
    std::printf("[D1] wastegate opening: WOT hold %.4f, 1.5 s after lift-off %.6f; compressor flow at end %.4f kg/s, pr %.4f\n",
                wg_wot, wg_lift, r.compressor_flow_kg_s, r.compressor_pressure_ratio);
    CHECK(wg_wot < 0.9);     // holding boost: partly closed
    CHECK(wg_lift > 0.999);  // plate < 10 %: commanded open, actuator tau 0.1 s -> 1 - exp(-15)
    // The same engine without a turbo reports 0 everywhere.
    PreSolveResult n;
    for (int i = 0; i < 960; ++i) {
        n = na.pre_solve(s, ThrottleCommand{1.0, 0.0, 1.0}, TcuRequest{}, kH);
        (void)na.post_solve(omega, kH);
    }
    CHECK(n.wastegate_opening == 0.0);
    CHECK(n.compressor_flow_kg_s == 0.0);
    CHECK(n.compressor_pressure_ratio == 0.0);
}
