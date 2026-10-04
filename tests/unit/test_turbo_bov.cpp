// tests/unit/test_turbo_bov.cpp -- PHYS-033 P4 I7d D2: the turbo blow-off valve
// (TurboPair, core/src/drivetrain/turbo_pair.h) at pair, engine and vehicle level.
//
// Model recap: the diaphragm sees charge pressure below and manifold pressure
// above, command = clamp((p_charge - p_im - crack)/(full - crack), 0, 1), the
// opening follows with a first-order lag; vent flow = b*A*p_c/sqrt(R*T_c)*
// Psi(p_amb/p_c) (the engine's own tabulated Psi) adds to the COMPRESSOR flow
// (head curve and compressor power), not to the engine's air.
//
// Fixture: tests/support/turbo_fixtures.h (hyper-like pair; valve 2 x 460 mm2,
// crack 0.9 bar, full 1.6 bar, tau 40 ms, all [AS]).
//
// Measured (release, 2026-10-04):
//   pair level, fixed air 0.04 kg/s after a WOT spool, lift at 1.5 s: compressor
//     flow 50 ms after the lift 0.0400 (no valve) vs 0.4552 kg/s (valve, opening
//     0.716, vent 0.4152); rotor after 500 ms 33152 rpm vs 0 rpm (the valve makes
//     the rotor decelerate FASTER: more flow at lower head, more compressor power).
//   shipped hyper V8 (held 6000 rpm, WOT 2 s then a lift), rotor at the lift
//     100663 rpm. AFTER the I7d compressor-flow fix (compressor fed the throttle flow,
//     exhaust heat held): no valve: rotor +50 ms 69674, +100 ms 41615, +250 ms 34989,
//     +500 ms 29602 rpm, compressor flow +50 ms 0.4463, +100 ms 0.0580 kg/s; 0.9 bar valve
//     (opening peaks at 0.315, vent peak 0.0943 kg/s): rotor +100 ms 36058, +250 ms 6144,
//     +500 ms 4527 rpm; 0.2 bar / 10 ms valve (opening 1.000, vent peak 0.4050 kg/s): the
//     rotor is at 0 rpm by +100 ms (the vent keeps 0.28 kg/s on the compressor, more
//     load than the rotor's energy). BEFORE the fix: 0 rpm already 50 ms after the lift
//     with or without a valve, compressor flow +50 ms 0.4027 kg/s in every variant,
//     +100 ms 0.0966 (none, 0.9 bar valve, opening peak 0.128, vent peak 0.0004 kg/s)
//     vs 0.2153 kg/s (0.2 bar valve, vent peak 0.1362 kg/s).
//   A real crack pressure must exceed the idle vacuum: a 0.2 bar valve also cracks once
//   at the t=0 spool-up (charge 101 kPa over a 30 kPa manifold).
//   vehicle, sedan chassis + hyper-like pair + 0.2 bar valve: events 2 (launch
//     spool-up, lift), max opening 1.000, max vent 0.3381 kg/s (0.1006 before the
//     fix), hash identical at 1 and 10 workers, own allocations 0.
//   engine fixture (held 5000 rpm, 0.2 bar valve): max vent 0.3170 kg/s (0.0972 before).
// Sabotages (each makes the named check fail, then reverted): see
//   out/i7d_d1_d4_progress.md "Sabotage results".
// The printf lines "[D2 ...]" print the numbers above.

#include "drivetrain/engine_simulated.h"
#include "drivetrain/turbo_pair.h"

#include "mvem_fixtures.h"
#include "turbo_fixtures.h"

#include "ps/io/vehicle_io.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

using namespace ps::drivetrain;
using ps::real;
namespace fx = ps::test::mvem;
namespace tb = ps::test::turbo;
using tb::Phase;
using tb::script_phase;
using tb::kScriptSteps;

constexpr real kH = 1.0 / 960.0;
constexpr real kPamb = 101325.0;
constexpr real kTamb = 288.15;
constexpr real kPi = 3.14159265358979323846;

const std::vector<real>& psi_table() {
    static const auto maps = SimulatedEngine::build_maps(fx::electronic_engine());
    return maps->psi;
}

// Independent isentropic flow function (std::pow; the engine's table is built
// with ps::math::pow and linear interpolation).
real psi_reference(real pr) {
    const real g = 1.4;
    const real crit = std::pow(2.0 / (g + 1.0), g / (g - 1.0));
    if (pr <= crit) return std::sqrt(g * std::pow(2.0 / (g + 1.0), (g + 1.0) / (g - 1.0)));
    return std::sqrt(2.0 * g / (g - 1.0) * (std::pow(pr, 2.0 / g) - std::pow(pr, (g + 1.0) / g)));
}

struct Rec {
    real flow, vent, rpm, charge, bov, comp_pr;
    std::uint32_t events;
};

// Spool 1.5 s at WOT (the valve senses a small charge-manifold difference),
// lift for lift_s, re-apply for reapply_s.
std::vector<Rec> run_lift_script(const TurboPairDesc& d, real lift_s, real reapply_s) {
    TurboPair pair(d);
    pair.reset(kPamb, kTamb);
    std::vector<Rec> out;
    const int n_spool = 1440, n_lift = int(lift_s * 960), n_reapply = int(reapply_s * 960);
    for (int i = 0; i < n_spool + n_lift + n_reapply; ++i) {
        Phase p;
        real p_im;
        if (i < n_spool) {
            const real t = i * kH;
            const real air = 0.2 + 0.7 * t / 1.5;
            p = {1.0, air, air * 1.07, air / 12.5 * 43e6};
            p_im = pair.charge_pa - 1.5e4;
        } else if (i < n_spool + n_lift) {
            p = {0.0, 0.04, 0.045, 0.0};
            p_im = 3.0e4;
        } else {
            p = {1.0, 0.9, 0.96, 0.9 / 12.5 * 43e6};
            p_im = pair.charge_pa - 1.5e4;
        }
        pair.step(kH, kPamb, kTamb, p.air, p.exhaust, p.port_k, p.plate, p_im, &psi_table());
        out.push_back({pair.compressor_flow_kg_s, pair.bov_flow_kg_s, pair.rpm(), pair.charge_pa, pair.bov_opening,
                       pair.compressor_pr, pair.bov_events});
    }
    return out;
}

} // namespace

TEST_CASE("D2: vent flow equals the isentropic orifice at frozen states", "[turbo][drivetrain][bov]") {
    // Psi table vs the independent formula: choked, unchoked and near 1.
    for (const real q : {0.2, 0.36, 0.5, 0.53, 0.63, 0.84, 0.97})
        CHECK(std::abs(psi_table_lookup(psi_table(), q) - psi_reference(q)) < 2e-6);

    const TurboPairDesc d = tb::hyper_like_pair_with_bov();
    const real area = d.blow_off_valve->effective_area_m2;
    struct Case { real charge, t_c, b; };
    // choked (q = 0.362), unchoked (q = 0.633), nearly equalised (q = 0.844)
    for (const Case c : {Case{2.8e5, 294.0, 0.6}, Case{1.6e5, 330.0, 1.0}, Case{1.2e5, 300.0, 0.3}}) {
        TurboPair p(d);
        p.reset(kPamb, kTamb);
        p.charge_pa = c.charge;
        p.charge_k = c.t_c;
        p.bov_opening = c.b;
        const real air = 0.2;
        p.step(kH, kPamb, kTamb, air, air, 0.0, 1.0, c.charge - 2e4, &psi_table());
        const real expected = c.b * area * c.charge / std::sqrt(287.05 * c.t_c) * psi_table_lookup(psi_table(), kPamb / c.charge);
        // tight against the table the engine uses, loose against the independent Psi
        CHECK(p.bov_flow_kg_s == Catch::Approx(expected).epsilon(1e-12));
        CHECK(p.bov_flow_kg_s == Catch::Approx(c.b * area * c.charge / std::sqrt(287.05 * c.t_c) * psi_reference(kPamb / c.charge)).epsilon(5e-6));
        CHECK(p.compressor_flow_kg_s == Catch::Approx(air + expected).epsilon(1e-14));
        std::printf("[D2 vent] charge %.0f Pa, T %.0f K, opening %.2f -> vent %.4f kg/s\n", c.charge, c.t_c, c.b, p.bov_flow_kg_s);
    }
    // closed valve: no flow; open valve with charge at or below ambient: no reverse flow
    {
        TurboPair p(d);
        p.reset(kPamb, kTamb);
        p.step(kH, kPamb, kTamb, 0.2, 0.2, 0.0, 1.0, 5e4, &psi_table());
        CHECK(p.bov_flow_kg_s == 0.0);
        p.bov_opening = 1.0;
        p.charge_pa = 0.9e5;
        p.step(kH, kPamb, kTamb, 0.2, 0.2, 0.0, 1.0, 5e4, &psi_table());
        CHECK(p.bov_flow_kg_s == 0.0);
    }
    // no table, no vent (a white-box caller without an engine)
    {
        TurboPair p(d);
        p.reset(kPamb, kTamb);
        p.charge_pa = 2.8e5;
        p.bov_opening = 1.0;
        p.step(kH, kPamb, kTamb, 0.2, 0.2, 0.0, 1.0, 5e4, nullptr);
        CHECK(p.bov_flow_kg_s == 0.0);
    }
}

TEST_CASE("D2: opening command, lag and events", "[turbo][drivetrain][bov]") {
    TurboPairDesc d = tb::hyper_like_pair_with_bov();
    // lag: charge 2.8 bar, manifold 0.3 bar -> command 1; the opening moves
    // h/(tau+h) = 1/(0.04*960 + 1) = 1/39.4 of the way in one step
    {
        TurboPair p(d);
        p.reset(kPamb, kTamb);
        p.charge_pa = 2.8e5;
        p.step(kH, kPamb, kTamb, 0.2, 0.2, 0.0, 0.0, 3.0e4, &psi_table());
        CHECK(p.bov_opening == Catch::Approx(1.0 / 39.4).epsilon(1e-12));
    }
    // tau 0: the opening follows the command exactly; walk the command through
    // the event thresholds. Rotor at rest and plate 0: the charge relaxes toward
    // ambient by 1/25 per step (h/(0.025+h)), so the post-step charge the valve
    // senses is known in advance.
    d.blow_off_valve->tau_s = 0.0;
    TurboPair p(d);
    p.reset(kPamb, kTamb);
    p.charge_pa = 2.8e5;
    const real crack = d.blow_off_valve->crack_dp_pa, full = d.blow_off_valve->full_open_dp_pa;
    const real targets[] = {0.5, 0.07, 0.12, 0.02, 0.3, 0.0};
    // 0.5 crosses 0.1 (event 1); 0.07 does not release (>= 0.05); 0.12 does not
    // count again; 0.02 releases; 0.3 is event 2; 0.0 releases
    const std::uint32_t events_after[] = {1, 1, 1, 1, 2, 2};
    for (int k = 0; k < 6; ++k) {
        const real charge_after = p.charge_pa + (kPamb - p.charge_pa) / 25.0;
        const real p_im = charge_after - crack - targets[k] * (full - crack);
        p.step(kH, kPamb, kTamb, 0.2, 0.2, 0.0, 0.0, p_im, &psi_table());
        CHECK(p.bov_opening == Catch::Approx(targets[k]).margin(1e-9));
        CHECK(p.bov_events == events_after[k]);
        p.charge_pa = 2.8e5; // hold the frozen state for the next target
    }
}

TEST_CASE("D2: a closed valve is invisible", "[turbo][drivetrain][bov]") {
    // At steady WOT the diaphragm sees 15 kPa (< crack 90 kPa): the pair with a
    // valve is bit-identical to the one without it, every step of the whole
    // guard script - the valve costs nothing while closed.
    TurboPair with(tb::hyper_like_pair_with_bov()), without(tb::hyper_like_pair());
    with.reset(kPamb, kTamb);
    without.reset(kPamb, kTamb);
    int mismatches = 0;
    for (int i = 0; i < kScriptSteps; ++i) {
        const Phase ph = script_phase(i);
        const real p_im = with.charge_pa - 1.5e4;
        with.step(kH, kPamb, kTamb, ph.air, ph.exhaust, ph.port_k, ph.plate, p_im, &psi_table());
        without.step(kH, kPamb, kTamb, ph.air, ph.exhaust, ph.port_k, ph.plate, p_im, &psi_table());
        if (with.energy_j != without.energy_j || with.charge_pa != without.charge_pa || with.wastegate != without.wastegate ||
            with.exhaust_pa != without.exhaust_pa || with.boost_integral != without.boost_integral)
            ++mismatches;
        REQUIRE(with.bov_opening == 0.0);
    }
    CHECK(mismatches == 0);
    CHECK(with.bov_events == 0);
}

TEST_CASE("D2: lift-off with and without the valve (pair level)", "[turbo][drivetrain][bov]") {
    const auto off = run_lift_script(tb::hyper_like_pair(), 1.0, 0.5);
    const auto on = run_lift_script(tb::hyper_like_pair_with_bov(), 1.0, 0.5);
    constexpr int kLift = 1440; // first lift-off step
    const auto at = [&](const std::vector<Rec>& v, real after_s) { return v[kLift + int(after_s * 960)]; };
    const Rec o50 = at(off, 0.05), v50 = at(on, 0.05), o500 = at(off, 0.5), v500 = at(on, 0.5);
    std::printf("[D2 lift pair] compressor flow after 50 ms: no valve %.4f, valve %.4f kg/s (vent %.4f, opening %.3f); "
                "rotor after 500 ms: no valve %.0f, valve %.0f rpm; charge after 500 ms: %.0f / %.0f Pa\n",
                o50.flow, v50.flow, v50.vent, v50.bov, o500.rpm, v500.rpm, o500.charge, v500.charge);
    CHECK(v50.flow > 3.0 * o50.flow); // the valve keeps the compressor flowing
    CHECK(v50.vent > 0.0);
    CHECK(v500.rpm < o500.rpm);       // and loads the rotor
    CHECK(on[kLift - 1].bov == 0.0);  // closed until the lift
    // one opening event for the lift; closed again after the re-apply
    CHECK(on[kLift + 960 - 1].events == 1);
    CHECK(on.back().events == 1);
    CHECK(on.back().bov < 0.05);
    std::printf("[D2 lift pair] after re-apply: rotor %.0f rpm (no valve %.0f), valve opening %.4f\n", on.back().rpm, off.back().rpm,
                on.back().bov);
}

// ---------------------------------------------------------------------------
// Engine level: the valve's sound state through SimulatedEngine.
// ---------------------------------------------------------------------------
TEST_CASE("D2: the engine publishes the valve and counts one event per lift", "[turbo][drivetrain][bov][sim_engine]") {
    auto d = fx::electronic_engine();
    d.turbo_pair = tb::hyper_like_pair();
    d.turbo_pair->blow_off_valve = tb::sensitive_bov();
    SimulatedEngine e(d);
    auto dn = fx::electronic_engine();
    dn.turbo_pair = tb::hyper_like_pair();
    SimulatedEngine en(dn);
    const real omega = 5000.0 * 2.0 * kPi / 60.0;
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    PreSolveResult r, rn, prev;
    real flow_ref = 0.0, max_open_wot = 0.0, max_open_lift = 0.0, max_vent = 0.0;
    std::uint32_t ev_before = 0, ev_after = 0;
    for (int i = 0; i < 3 * 960; ++i) {
        const real t = i * kH;
        // WOT 0-1 s, lift 1-2 s, WOT 2-3 s
        const real c = (t < 1.0 || t >= 2.0) ? 1.0 : 0.0;
        prev = r;
        r = e.pre_solve(s, ThrottleCommand{c, 0.0, 1.0}, TcuRequest{}, kH);
        rn = en.pre_solve(s, ThrottleCommand{c, 0.0, 1.0}, TcuRequest{}, kH);
        (void)e.post_solve(omega, kH);
        (void)en.post_solve(omega, kH);
        if (t >= 0.5 && t < 1.0) max_open_wot = std::max(max_open_wot, r.bov_opening); // settled WOT
        if (i == 959) ev_before = r.bov_events;
        if (i == 1919) ev_after = r.bov_events;
        if (t >= 1.0 && t < 2.0) { max_open_lift = std::max(max_open_lift, r.bov_opening); max_vent = std::max(max_vent, r.bov_flow_kg_s); }
        // compressor flow = the relaxed throttle-path flow of the previous substep (I7d compressor-flow fix, see
        // test_turbo_compressor_flow.cpp CF1; was the cylinder air before it) + the vent flow of this one.
        // The vent is NOT subtracted from the engine's air, so m_thr is the same with or without the valve.
        if (i >= 2) {
            const real published = r.compressor_flow_kg_s - r.bov_flow_kg_s;
            if (i == 2) flow_ref = published; // seed: f_1
            else CHECK(published == Catch::Approx(flow_ref).epsilon(1e-9).margin(1e-12));
            const real m_thr = e.manifold_rhs(prev.p_intake_pa, omega, r.throttle_position, 0.0).m_thr;
            flow_ref += (kH / (0.005 + kH)) * (std::max(real(0.0), m_thr) - flow_ref); // literal 5 ms = kTurboFlowLagS
        }
        CHECK(rn.bov_opening == 0.0);
        CHECK(rn.bov_events == 0);
    }
    std::printf("[D2 engine] max opening at WOT %.3f, during lift %.3f; max vent %.4f kg/s; events %u\n", max_open_wot, max_open_lift,
                max_vent, ev_after - ev_before);
    CHECK(max_open_wot < 1e-6);  // closed at settled WOT (the opening decays exponentially after the t=0 spool-up crack)
    CHECK(max_open_lift > 0.99); // fully open after the lift
    CHECK(max_vent > 0.0);
    CHECK(ev_after - ev_before == 1); // exactly one event for the lift (the spool-up at t=0 also opens this 0.2 bar valve once)
    CHECK(r.bov_vent == 1);      // fixture valve recirculates
    CHECK(rn.bov_vent == 0);
}

// ---------------------------------------------------------------------------
// The shipped hyper V8 (maps generated at load, cached) on a held 6000 rpm
// crank: WOT 2 s, then a lift for 1 s, with no valve, the fixture valve (crack
// 0.9 bar, tau 40 ms) and a sensitive valve (crack 0.2 bar, tau 10 ms).
// ---------------------------------------------------------------------------
namespace {
struct LiftMeasure {
    real boost_wot = 0, flow50 = 0, flow100 = 0, rpm0 = 0, rpm50 = 0, rpm100 = 0, rpm250 = 0, rpm500 = 0, charge250 = 0, vent_peak = 0, open_peak = 0;
    std::uint32_t events = 0;      // events counted during the lift only
    std::uint32_t events_wot = 0;  // events counted before the lift
};
LiftMeasure measure_hyper(const SimulatedEngineDesc& base, const std::optional<BlowOffValveDesc>& valve) {
    SimulatedEngineDesc d = base;
    d.turbo_pair->blow_off_valve = valve;
    SimulatedEngine e(d);
    const real omega = 6000.0 * 2.0 * kPi / 60.0;
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    LiftMeasure m;
    const int n_wot = 2 * 960, n_lift = 960;
    for (int i = 0; i < n_wot + n_lift; ++i) {
        const real c = i < n_wot ? 1.0 : 0.0;
        const PreSolveResult r = e.pre_solve(s, ThrottleCommand{c, 0.0, 1.0}, TcuRequest{}, kH);
        (void)e.post_solve(omega, kH);
        if (i == n_wot - 1) { m.boost_wot = r.boost_bar; m.events_wot = r.bov_events; m.rpm0 = r.turbo_rpm; }
        if (i >= n_wot) {
            const int k = i - n_wot + 1; // steps since the lift
            m.vent_peak = std::max(m.vent_peak, r.bov_flow_kg_s);
            m.open_peak = std::max(m.open_peak, r.bov_opening);
            if (k == 48) m.flow50 = r.compressor_flow_kg_s;
            if (k == 96) { m.flow100 = r.compressor_flow_kg_s; m.rpm100 = r.turbo_rpm; }
            if (k == 48) m.rpm50 = r.turbo_rpm;
            if (k == 240) { m.rpm250 = r.turbo_rpm; m.charge250 = r.p_charge_pa; }
            if (k == 480) m.rpm500 = r.turbo_rpm;
            m.events = r.bov_events - m.events_wot;
        }
    }
    return m;
}
SimulatedEngineDesc load_hyper() {
    ps::jobs::JobSystem pool(10);
    ps::io::EngineMapOptions options;
    options.pool = &pool;
    options.cache_dir = (std::filesystem::temp_directory_path() / "ps_i7d_engine_cache").string();
    return std::get<SimulatedEngineDesc>(
        ps::io::load_engine_json(std::string(PS_SOURCE_DIR) + "/data/engines/hyper_v8_one1_audio.json", options));
}
} // namespace

TEST_CASE("D2: hyper V8 lift-off with and without the valve (measured)", "[turbo][bov][sim_engine][hyper]") {
    const SimulatedEngineDesc base = load_hyper();
    REQUIRE(base.turbo_pair.has_value());
    const BlowOffValveDesc sensitive = tb::sensitive_bov();
    const LiftMeasure none = measure_hyper(base, std::nullopt);
    const LiftMeasure fixture = measure_hyper(base, tb::hyper_like_bov());
    const LiftMeasure sens = measure_hyper(base, sensitive);
    const auto line = [](const char* name, const LiftMeasure& m) {
        std::printf("[D2 hyper lift] %-18s boost at WOT %.3f bar | compressor flow +50 ms %.4f, +100 ms %.4f kg/s | rotor at lift %.0f, +50 ms %.0f, +100 ms %.0f, +250 ms %.0f, +500 ms %.0f rpm | "
                    "charge +250 ms %.0f Pa | vent peak %.4f kg/s, opening peak %.3f, events %u (before lift %u)\n",
                    name, m.boost_wot, m.flow50, m.flow100, m.rpm0, m.rpm50, m.rpm100, m.rpm250, m.rpm500, m.charge250, m.vent_peak, m.open_peak, m.events, m.events_wot);
    };
    line("no valve", none);
    line("fixture valve", fixture);
    line("sensitive valve", sens);
    CHECK(none.events == 0);
    CHECK(none.vent_peak == 0.0);
    CHECK(sens.events == 1);
    CHECK(fixture.events == 1); // even the 0.9 bar valve cracks once, barely (peak opening ~0.13)
    CHECK(sens.vent_peak > 0.0);
    CHECK(sens.open_peak > 0.99);
}

// ---------------------------------------------------------------------------
// Hash coverage: two engines whose valves stay shut for the whole run (crack
// 90 kPa vs 100 kPa at a steady WOT hold, the dynamics are bit-identical) must
// still hash differently - the valve parameters are part of the engine's state
// hash - and an engine without a valve hashes differently again.
// ---------------------------------------------------------------------------
TEST_CASE("D2: the valve parameters are hashed", "[turbo][drivetrain][bov][sim_engine]") {
    const auto run = [](const std::optional<BlowOffValveDesc>& valve) {
        auto d = fx::electronic_engine();
        d.turbo_pair = tb::hyper_like_pair();
        d.turbo_pair->blow_off_valve = valve;
        SimulatedEngine e(d);
        const real omega = 5000.0 * 2.0 * kPi / 60.0;
        Sensors s;
        s.omega = omega;
        s.ignition = 1.0;
        for (int i = 0; i < 960; ++i) {
            (void)e.pre_solve(s, ThrottleCommand{1.0, 0.0, 1.0}, TcuRequest{}, kH);
            (void)e.post_solve(omega, kH);
        }
        return e.fold_state_hash(0);
    };
    BlowOffValveDesc a = tb::hyper_like_bov(), b = a;
    b.crack_dp_pa = 1.0e5;
    const std::uint64_t ha = run(a), hb = run(b), hn = run(std::nullopt);
    CHECK(ha == run(a)); // deterministic
    CHECK(ha != hb);
    CHECK(ha != hn);
    CHECK(hb != hn);
    BlowOffValveDesc c = a;
    c.tau_s = 0.05;
    CHECK(run(c) != ha);
    c = a;
    c.effective_area_m2 *= 1.01;
    CHECK(run(c) != ha);
    c = a;
    c.full_open_dp_pa += 1e3;
    CHECK(run(c) != ha);
}
