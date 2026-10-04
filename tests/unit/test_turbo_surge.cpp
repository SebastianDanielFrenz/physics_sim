// tests/unit/test_turbo_surge.cpp -- PHYS-033 P4 I7d D3: the compressor surge
// FLAG (TurboPair::surge_active / surge_events, ruling N2: no dynamics). The line
// (SurgeLineDesc, optional "surge_line" block of a physics_sim.turbo/1 hardware
// file) is the surge flow as a fraction of the pair's max flow over compressor
// pressure ratio. surge_active = pr > 1.05 and flow_fraction < line(pr); it is
// cleared when flow_fraction > 1.05 * line(pr) or pr <= 1.05 (hysteresis);
// surge_events counts entries. Nothing in the model reads the flag, so a pair or
// an engine with a line behaves bit-identically to one without (every step of
// the script, and the engine state hash).
//
// Fixture: tests/support/turbo_fixtures.h hyper_like_surge_line() ([AS]: pr
// 1/1.5/2/3 -> flow fraction 0.04/0.12/0.22/0.40).
//
// Measured (release, 2026-10-04):
//   pair level, 1 s lift at fixed air 0.04 kg/s after a 1.5 s WOT spool: surge time
//     0.801 s with no valve, 0.046 s with the 0.9 bar valve, 0.011 s with the 0.2 bar
//     valve (flag edges at steps 74/96 = a spool-up blip, then 1440 = the lift, off at
//     2209 / 1484 / 1451); 2 entries each.
//   shipped hyper V8 (held 6000 rpm, WOT 2 s then lift), fixture line, AFTER the I7d
//     compressor-flow fix (compressor fed the throttle flow, the rotor coasts): surge time
//     in the 1 s after the lift 0.946 s (1 entry) with no valve, 0.150 s (1) with the 0.9 bar
//     valve, 0 s with the 0.2 bar valve; line x1.5 0.966 / 0.170 / 0. Before the fix: no surge
//     at all with the fixture line (the rotor was at 0 rpm 50 ms after the lift), line x1.5
//     0.007 / 0.007 / 0 s.
//   vehicle (sedan chassis + hyper-like pair, no valve): 2 entries (3 before the fix), state
//     hash identical with and without the line (0x77ed68600cda4d85; was 0x0f8fb8cfb73d2ebe)
//     at 1 and 10 workers.
// Sabotages (each makes the named check fail, then reverted): see
//   out/i7d_d1_d4_progress.md "Sabotage results".

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
#include <vector>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

using namespace ps::drivetrain;
using ps::real;
namespace fx = ps::test::mvem;
namespace tb = ps::test::turbo;

constexpr real kH = 1.0 / 960.0;
constexpr real kPamb = 101325.0;
constexpr real kTamb = 288.15;
constexpr real kPi = 3.14159265358979323846;

const std::vector<real>& psi_table() {
    static const auto maps = SimulatedEngine::build_maps(fx::electronic_engine());
    return maps->psi;
}

TurboPairDesc pair_with_surge() {
    TurboPairDesc d = tb::hyper_like_pair();
    d.surge_line = tb::hyper_like_surge_line();
    return d;
}

// Hold the rotor at a normalised speed and feed a constant air flow until the
// charge pressure settles; returns nothing, the pair's flags are read after.
void hold(TurboPair& p, real speed_norm, real air_kg_s, int steps = 480) {
    const real wr = p.desc.reference_rpm * 2.0 * kPi / 60.0;
    for (int i = 0; i < steps; ++i) {
        p.energy_j = 0.5 * p.desc.rotor_inertia_kgm2 * (speed_norm * wr) * (speed_norm * wr);
        p.step(kH, kPamb, kTamb, air_kg_s, air_kg_s * 1.07, 0.0, 1.0);
    }
}

} // namespace

TEST_CASE("D3: surge flag, hysteresis and event count at held operating points", "[turbo][drivetrain][surge]") {
    TurboPair p(pair_with_surge());
    p.reset(kPamb, kTamb);
    // Rotor at the reference speed: head = 4.5e5 * (1 - f^2), f = air / 1.2.
    // air 0.9 (f 0.75): pr ~2.9 (line clamps at 0.40 above pr 3), f 0.75 -> clear of the line.
    hold(p, 1.0, 0.9);
    CHECK_FALSE(p.surge_active);
    CHECK(p.surge_events == 0);
    // air 0.1 (f 0.083, pr ~5.4): left of the line -> enters, once.
    hold(p, 1.0, 0.1);
    CHECK(p.surge_active);
    CHECK(p.surge_events == 1);
    hold(p, 1.0, 0.1, 100);
    CHECK(p.surge_events == 1); // staying in surge does not count again
    // air 0.49 (f 0.408): between the line (0.40 at pr >= 3) and 1.05 x the line
    // (0.42): while active it stays active (hysteresis).
    hold(p, 1.0, 0.49);
    CHECK(p.compressor_pr > 3.0);
    CHECK(p.surge_active);
    CHECK(p.surge_events == 1);
    // air 0.9: well above 1.05 x the line -> leaves.
    hold(p, 1.0, 0.9);
    CHECK_FALSE(p.surge_active);
    // air 0.49 again, now coming from outside: f 0.408 > the line -> does NOT enter.
    hold(p, 1.0, 0.49);
    CHECK_FALSE(p.surge_active);
    CHECK(p.surge_events == 1);
    // below the line again: second entry.
    hold(p, 1.0, 0.1);
    CHECK(p.surge_active);
    CHECK(p.surge_events == 2);
    // a stopped rotor: pr = 1 <= 1.05 clears the flag whatever the flow.
    hold(p, 0.0, 0.05);
    CHECK(p.compressor_pr < 1.0001);
    CHECK_FALSE(p.surge_active);
    CHECK(p.surge_events == 2);
    // no pressure ratio, no entry, even at a tiny flow
    hold(p, 0.0, 0.001);
    CHECK_FALSE(p.surge_active);
    CHECK(p.surge_events == 2);
}

TEST_CASE("D3: without a surge line the flag is never raised", "[turbo][drivetrain][surge]") {
    TurboPair p(tb::hyper_like_pair());
    p.reset(kPamb, kTamb);
    hold(p, 1.0, 0.1);
    CHECK(p.compressor_pr > 3.0);
    CHECK_FALSE(p.surge_active);
    CHECK(p.surge_events == 0);
}

namespace {
struct Lift {
    int surge_steps = 0;       // substeps with the flag raised during the lift (1 s)
    real surge_s = 0;
    std::uint32_t events = 0;  // entries over the whole script
    bool active_at_lift_end = false;
    std::uint64_t digest = 0;  // FNV over the dynamic state of every step
    std::vector<int> edges;    // step indices at which the flag changed
};

std::uint64_t fold(std::uint64_t h, real v) {
    unsigned char b[sizeof v];
    std::memcpy(b, &v, sizeof v);
    for (unsigned char c : b) { h ^= c; h *= 1099511628211ULL; }
    return h;
}

Lift run_lift(const TurboPairDesc& d) {
    TurboPair pair(d);
    pair.reset(kPamb, kTamb);
    Lift out;
    out.digest = 14695981039346656037ULL;
    bool was_active = false;
    constexpr int n_spool = 1440, n_lift = 960, n_reapply = 480;
    for (int i = 0; i < n_spool + n_lift + n_reapply; ++i) {
        const tb::Phase ph = tb::script_phase(i);
        const real p_im = i < n_spool ? pair.charge_pa - 1.5e4 : (i < n_spool + n_lift ? 3.0e4 : pair.charge_pa - 1.5e4);
        pair.step(kH, kPamb, kTamb, ph.air, ph.exhaust, ph.fuel_w, ph.plate, p_im, &psi_table());
        if (pair.surge_active != was_active) { out.edges.push_back(i); was_active = pair.surge_active; }
        if (i >= n_spool && i < n_spool + n_lift && pair.surge_active) ++out.surge_steps;
        if (i == n_spool + n_lift - 1) out.active_at_lift_end = pair.surge_active;
        for (real v : {pair.energy_j, pair.charge_pa, pair.charge_k, pair.exhaust_pa, pair.wastegate, pair.boost_integral,
                       pair.compressor_w, pair.turbine_w, pair.compressor_flow_kg_s, pair.compressor_pr, pair.bov_opening})
            out.digest = fold(out.digest, v);
    }
    out.surge_s = out.surge_steps * kH;
    out.events = pair.surge_events;
    return out;
}
} // namespace

TEST_CASE("D3: a surge line changes nothing the model computes", "[turbo][drivetrain][surge]") {
    // every dynamic value of every step of the guard script, with and without a line
    // (and with a blow-off valve present too)
    CHECK(run_lift(tb::hyper_like_pair()).digest == run_lift(pair_with_surge()).digest);
    TurboPairDesc a = tb::hyper_like_pair_with_bov(), b = a;
    b.surge_line = tb::hyper_like_surge_line();
    CHECK(run_lift(a).digest == run_lift(b).digest);
}

TEST_CASE("D3: lift-off surge with and without the blow-off valve (pair level)", "[turbo][drivetrain][surge][bov]") {
    const Lift none = run_lift(pair_with_surge());
    TurboPairDesc fixture = pair_with_surge();
    fixture.blow_off_valve = tb::hyper_like_bov();
    TurboPairDesc sensitive = pair_with_surge();
    sensitive.blow_off_valve = tb::sensitive_bov();
    const Lift fx_valve = run_lift(fixture);
    const Lift sens = run_lift(sensitive);
    std::printf("[D3 lift pair] surge time during a 1 s lift: no valve %.3f s (events %u), 0.9 bar valve %.3f s (events %u), 0.2 bar valve %.3f s (events %u)\n",
                none.surge_s, none.events, fx_valve.surge_s, fx_valve.events, sens.surge_s, sens.events);
    for (const auto* l : {&none, &fx_valve, &sens}) {
        std::printf("[D3 lift pair] flag edges (step, lift starts at 1440):");
        for (int e : l->edges) std::printf(" %d", e);
        std::printf("\n");
    }
    // measured: 0.801 / 0.046 / 0.011 s; the 2 entries are a brief spool-up blip (steps 74-96, rotor
    // still accelerating at 0.2 kg/s, charge pr jumps past the fixture line) and the lift.
    CHECK(none.surge_s > 0.6);
    CHECK(fx_valve.surge_s < 0.1);
    CHECK(sens.surge_s < 0.03);
    CHECK(none.events == 2);
    CHECK(fx_valve.events == 2);
    CHECK(sens.events == 2);
    CHECK(none.edges.at(2) == 1440); // the lift entry is immediate: flow drops from 0.75 to 0.033 in one step
    CHECK(none.active_at_lift_end == false);
}

TEST_CASE("D3: the engine publishes the surge flag and the line is not hashed", "[turbo][drivetrain][surge][sim_engine]") {
    const auto run = [](bool line, std::uint64_t& hash, std::uint32_t& events, bool& saw_flag, real& rpm_sum) {
        auto d = fx::electronic_engine();
        d.turbo_pair = tb::hyper_like_pair();
        if (line) d.turbo_pair->surge_line = tb::hyper_like_surge_line();
        SimulatedEngine e(d);
        const real omega = 5000.0 * 2.0 * kPi / 60.0;
        Sensors s;
        s.omega = omega;
        s.ignition = 1.0;
        saw_flag = false;
        rpm_sum = 0.0;
        PreSolveResult r;
        for (int i = 0; i < 3 * 960; ++i) {
            const real t = i * kH;
            const real c = (t < 1.0 || t >= 2.0) ? 1.0 : 0.0;
            r = e.pre_solve(s, ThrottleCommand{c, 0.0, 1.0}, TcuRequest{}, kH);
            (void)e.post_solve(omega, kH);
            if (r.surge_active) saw_flag = true;
            rpm_sum += r.turbo_rpm + r.p_charge_pa;
        }
        hash = e.fold_state_hash(0);
        events = r.surge_events;
    };
    std::uint64_t h1 = 0, h0 = 0;
    std::uint32_t e1 = 0, e0 = 0;
    bool f1 = false, f0 = false;
    real s1 = 0, s0 = 0;
    run(true, h1, e1, f1, s1);
    run(false, h0, e0, f0, s0);
    std::printf("[D3 engine] surge events with the line %u (flag seen %d), without %u; state hash equal %d\n", e1, int(f1), e0, int(h1 == h0));
    CHECK(h1 == h0);   // the line is not hashed
    CHECK(s1 == s0);   // and moves nothing the engine publishes
    CHECK(f1);
    CHECK(e1 >= 1);
    CHECK_FALSE(f0);
    CHECK(e0 == 0);
}

namespace {
ps::drivetrain::SimulatedEngineDesc load_hyper() {
    ps::jobs::JobSystem pool(10);
    ps::io::EngineMapOptions options;
    options.pool = &pool;
    options.cache_dir = (std::filesystem::temp_directory_path() / "ps_i7d_engine_cache").string();
    return std::get<SimulatedEngineDesc>(
        ps::io::load_engine_json(std::string(PS_SOURCE_DIR) + "/data/engines/hyper_v8_one1_audio.json", options));
}
struct HyperSurge {
    real surge_s = 0;
    std::uint32_t events = 0;
};
HyperSurge hyper_lift(const SimulatedEngineDesc& base, const std::optional<BlowOffValveDesc>& valve, real line_scale) {
    SimulatedEngineDesc d = base;
    d.turbo_pair->blow_off_valve = valve;
    d.turbo_pair->surge_line = tb::hyper_like_surge_line();
    for (real& y : d.turbo_pair->surge_line->flow_fraction_vs_pr.y) y = std::min(0.99, y * line_scale);
    SimulatedEngine e(d);
    const real omega = 6000.0 * 2.0 * kPi / 60.0;
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    HyperSurge out;
    const int n_wot = 2 * 960, n_lift = 960;
    int steps = 0;
    std::uint32_t events_wot = 0;
    for (int i = 0; i < n_wot + n_lift; ++i) {
        const real c = i < n_wot ? 1.0 : 0.0;
        const PreSolveResult r = e.pre_solve(s, ThrottleCommand{c, 0.0, 1.0}, TcuRequest{}, kH);
        (void)e.post_solve(omega, kH);
        if (i == n_wot - 1) events_wot = r.surge_events;
        if (i >= n_wot && r.surge_active) ++steps;
        out.events = r.surge_events - events_wot;
    }
    out.surge_s = steps * kH;
    return out;
}
} // namespace

TEST_CASE("D3: hyper V8 lift-off surge with and without the valve (measured)", "[turbo][surge][bov][sim_engine][hyper]") {
    const SimulatedEngineDesc base = load_hyper();
    REQUIRE(base.turbo_pair.has_value());
    for (const real scale : {1.0, 1.5}) {
        const HyperSurge none = hyper_lift(base, std::nullopt, scale);
        const HyperSurge fixture = hyper_lift(base, tb::hyper_like_bov(), scale);
        const HyperSurge sens = hyper_lift(base, tb::sensitive_bov(), scale);
        std::printf("[D3 hyper lift] line x%.1f, surge time in the 1 s after the lift: no valve %.3f s (%u entries), 0.9 bar valve %.3f s (%u), 0.2 bar valve %.3f s (%u)\n",
                    scale, none.surge_s, none.events, fixture.surge_s, fixture.events, sens.surge_s, sens.events);
        // With the compressor fed the throttle flow (I7d compressor-flow fix) the rotor coasts after the lift
        // and the pressure ratio stays above 1.05 for ~1 s while the throttle flow is ~0.006 kg/s: without a
        // valve the flag is raised for almost the whole second; the 0.9 bar valve (opens to ~0.3) shortens it
        // to ~0.15 s; the 0.2 bar valve (fully open at once) keeps the compressor out of surge altogether.
        CHECK(none.events == 1);
        CHECK(none.surge_s > 0.5);
        CHECK(fixture.events == 1);
        CHECK(fixture.surge_s > 0.0);
        CHECK(fixture.surge_s < 0.5 * none.surge_s);
        CHECK(sens.events == 0);
        CHECK(sens.surge_s == 0.0);
    }
    // measured: line x1.0 no valve 0.946 s (1 entry), 0.9 bar valve 0.150 s (1), 0.2 bar valve 0 s; line x1.5:
    // 0.966 / 0.170 / 0. (Before the fix: x1.0 never surged - the rotor was at 0 rpm 50 ms after the lift;
    // x1.5 0.007 / 0.007 / 0.) A line x2 puts steady WOT on the line itself (0.8 vs a flow fraction of
    // ~0.75) and was dropped.
}
