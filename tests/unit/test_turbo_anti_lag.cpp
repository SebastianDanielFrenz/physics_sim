// tests/unit/test_turbo_anti_lag.cpp -- PHYS-033 P4 I7d D5: anti-lag (ALS), the
// optional "anti_lag" block of a turbo configuration (docs/data_formats/turbo.md),
// at pair, engine, vehicle and loader level.
//
// Model recap (core/src/drivetrain/engine_simulated.cpp, turbo_pair.h): active
// while the channel reads >= 0.5 (NaN = not wired -> default_enabled), the
// throttle command is below arm_below_command, rpm in [min_rpm, max_rpm], the
// engine is Running, for at most max_active_s per lift (re-armed when the
// command reaches arm_below_command; a substep that starves the engine of fuel
// while active disarms anti-lag for the rest of the lift). While
// active: DFCO inhibited, plate target >= plate_floor, spark = scheduled -
// retard_deg through eta_spark; the indicated work the retard removes,
// omega_old*(tau_comb(scheduled spark) - tau_comb(retarded)), stays in the
// exhaust gas through the engine's exhaust energy balance (exhaust_thermal,
// test_turbo_exhaust_thermal.cpp) - no separate term since that change; before
// it, this work was peak-held for 0.5 s and added to a 25 % fuel-heat fraction
// under a 1200 K cap, and the numbers below were measured on that model.
//
// Fixtures: tests/support/mvem_fixtures.h electronic engine (2 L I4) +
// tests/support/turbo_fixtures.h hyper-like pair + hyper_like_anti_lag()
// (arm < 0.1, 3000-7500 rpm, retard 30 deg, plate floor 0.5, 1.5 s; all [AS]).
//
// Measured (release, 2026-10-04), held crank at 5000 rpm, lift to command 0
// after 2 s WOT, rotor rpm at +0/0.25/0.5/1/2 s:
//   off            73811 / 51844 / 47521 / 39756 / 28252 (DFCO from the lift)
//   on (fixture)   73811 / 51503 / 51983 / 52959 / 39547 (active exactly 1440
//                  substeps = 1.5 s, then DFCO; boost +0.78/+0.73/+0.77 bar at
//                  0.25/0.5/1 s, extra_exhaust_w 87460/85050/86683 W, turbine
//                  inlet at the 1200 K cap, wastegate ~0.2)
//   on, retard 0   73811 / 48095 / 47536 / 50530 / 41310 (plate floor only)
// Floor sweep (+1 s rotor, [.][diag]): 0.2 -> 15.5-18.6k (below the DFCO
// coast: the lumped pair's rotor equilibrium follows airflow), 0.35 ->
// 38.7-42.2k, 0.5 -> 50.5-53.0k; retard 30 vs 45 identical (1200 K cap).
// Hyper twin V8 [hyper] at 6000 rpm (100663 rpm, 1.634 bar at the lift):
// off 34989/29602/21475/11613 at +0.25/0.5/1/2 s, on 55967/56055/57483/29050
// (boost +0.59..0.61 bar, extra 125-129 kW, crank torque tau0 +630..640 N*m
// while active - eta_spark at 30 deg retard is still high), floor 0.2
// 16457/15649/18894/14412. Vehicle (sedan + simulated turbo engine): state
// hash 1 == 10 workers, 0 own allocations (Jolt external = the known
// QuadTree gap), worst balance_error 2.0e-15 relative, fuel identity
// 1.9e-15, 2 activations with a channel toggle during the lift.
// Re-measured with the exhaust energy balance + wall (cold wall, 2026-10-05):
//   fixture 5000 rpm off 73904 / 30851 / 28483 / 24126 / 17412, on 73904 /
//   50112 / 50891 / 52685 / 26818 (port 1676 K, turbine 1167-1181 K, extra
//   83-86 kW, tau0 +245..256 N*m), on retard 0 73904 / 43933 / 43728 / 49269 /
//   26950. Hyper 6000 rpm off 101288 / 0 / 0 / 0 / 0 (the shipped [AS] rotor
//   is drained within ~0.1 s, see test_turbo_compressor_flow.cpp), on 101288 /
//   54673 / 55636 / 57161 / 0 (port 1538-1542 K, turbine 1324-1334 K, extra
//   123-128 kW, tau0 +626..640 N*m), retard 0 52390 / 53833 / 56037 / 3941,
//   floor 0.2 10904 / 10509 / 13318 / 10115. No cap: a deeper retard heats the
//   gas further without a ceiling.
//
// Sabotages (each made the named check fail, then reverted):
//   heat not routed to the turbine        -> lift "on > on0 + 1000 rpm"
//   extra heat vs MBT (1 - eta_spark)     -> white-box extra_w literal
//   retard not applied                    -> white-box spark/extra_w literals
//   max_active_s ignored                  -> active == 1440, used-up checks
//   DFCO not inhibited                    -> lift !dfco, als_and_dfco == 0,
//                                            fired-under-ALS REQUIRE
//   plate floor not applied (elec/mech)   -> plate-floor case (plate 0.0069 /
//                                            0.0 vs >= 0.5)
//   no re-arm                             -> re-arm checks
//   starvation does not disarm            -> starving case (als at +1, +10)
//   timer / armed / plate_floor not folded-> the matching hash checks
//   channel not declared                  -> declaration + vehicle toggle
//   loader unknown-key check removed      -> loader rejection check

#include "drivetrain/engine_simulated.h"
#include "drivetrain/powertrain.h"
#include "drivetrain/turbo_pair.h"

#include "mvem_fixtures.h"
#include "powertrain_multi_diff_desc.h" // ps_test::load_sedan_desc()
#include "turbo_fixtures.h"

#include "ps/drivetrain/controller_desc.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <variant>
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
constexpr real kPi = 3.14159265358979323846;

real omega_of(real rpm) { return rpm * 2.0 * kPi / 60.0; }

SimulatedEngineDesc engine_with(const std::optional<AntiLagDesc>& al) {
    auto d = fx::electronic_engine();
    d.turbo_pair = tb::hyper_like_pair();
    d.turbo_pair->anti_lag = al;
    return d;
}

// The fixture's eta_spark curve, typed in from tests/support/mvem_fixtures.h (literal knots, linear
// interpolation, clamped at the ends) - an independent re-evaluation, not the engine's Curve1D.
real eta_spark_literal(real dx) {
    const real xs[] = {-40.0, -20.0, -8.0, 0.0, 8.0};
    const real ys[] = {0.55, 0.82, 0.965, 1.0, 0.965};
    if (dx <= xs[0]) return ys[0];
    if (dx >= xs[4]) return ys[4];
    for (int i = 1; i < 5; ++i)
        if (dx <= xs[i]) return ys[i - 1] + (ys[i] - ys[i - 1]) * (dx - xs[i - 1]) / (xs[i] - xs[i - 1]);
    return ys[4];
}

// One lift-off on a held crank: WOT for wot_s, then command `lift_cmd` for lift_s.
struct Sample {
    real turbo_rpm = 0, boost_bar = 0, extra_w = 0, port_k = 0, exhaust_k = 0, wastegate = 0, plate = 0, tau0 = 0, fuel_g_s = 0;
    bool dfco = false, als = false;
};
struct LiftRun {
    std::vector<Sample> s; // one per substep
    int n_wot = 0;
    std::uint32_t activations = 0;
    real max_ledger_violation = 0.0; // max(h*extra_w - (fuel_energy_in - indicated_work)) per substep, J
    real min_heat_margin = 1e300;     // min over ALS substeps of (fuel_energy_in - indicated) - h*extra_w, J
    int als_and_dfco = 0;             // substeps with both flags set (must be 0)
};
LiftRun run_lift(const SimulatedEngineDesc& d, real channel, real rpm = 5000.0, real wot_s = 2.0, real lift_s = 3.0,
                 real lift_cmd = 0.0, real fuel_available = std::numeric_limits<real>::infinity()) {
    SimulatedEngine e(d);
    const real omega = omega_of(rpm);
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    s.anti_lag_channel = channel;
    LiftRun out;
    out.n_wot = int(wot_s * 960.0);
    const int n = out.n_wot + int(lift_s * 960.0);
    for (int i = 0; i < n; ++i) {
        const bool wot = i < out.n_wot;
        s.fuel_available_kg = wot ? std::numeric_limits<real>::infinity() : fuel_available;
        const PreSolveResult r = e.pre_solve(s, ThrottleCommand{wot ? 1.0 : lift_cmd, 0.0, 1.0}, TcuRequest{}, kH);
        const PostSolveResult p = e.post_solve(omega, kH);
        Sample sm;
        sm.turbo_rpm = r.turbo_rpm;
        sm.boost_bar = r.boost_bar;
        sm.extra_w = r.anti_lag_extra_exhaust_w;
        sm.port_k = r.exhaust_port_k;
        sm.fuel_g_s = r.fuel_flow_g_s;
        sm.exhaust_k = e.turbo()->exhaust_k;
        sm.wastegate = r.wastegate_opening;
        sm.plate = r.throttle_position;
        sm.tau0 = r.tau0;
        sm.dfco = r.dfco_active;
        sm.als = r.anti_lag_active != 0;
        out.s.push_back(sm);
        const real heat = p.fuel_energy_in_delta - p.indicated_work_delta;
        out.max_ledger_violation = std::max(out.max_ledger_violation, kH * r.anti_lag_extra_exhaust_w - heat);
        if (sm.als) out.min_heat_margin = std::min(out.min_heat_margin, heat - kH * r.anti_lag_extra_exhaust_w);
        if (sm.als && sm.dfco) ++out.als_and_dfco;
        out.activations = r.anti_lag_activations;
    }
    return out;
}

} // namespace

// ---------------------------------------------------------------------------
// An anti_lag block with the channel off is invisible to the dynamics (only the
// state hash differs: the block's parameters are folded).
// ---------------------------------------------------------------------------
TEST_CASE("D5: a switched-off anti-lag leaves every published result bit-identical", "[turbo][drivetrain][anti_lag][sim_engine]") {
    SimulatedEngine with(engine_with(tb::hyper_like_anti_lag()));
    SimulatedEngine without(engine_with(std::nullopt));
    const real omega = omega_of(5000.0);
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    s.anti_lag_channel = 0.0;
    Sensors s0 = s;
    s0.anti_lag_channel = std::numeric_limits<real>::quiet_NaN();
    for (int i = 0; i < tb::kScriptSteps; ++i) {
        const real t = i * kH;
        const real c = t < 1.5 ? 1.0 : (t < 2.5 ? 0.0 : 1.0);
        const PreSolveResult a = with.pre_solve(s, ThrottleCommand{c, 0.0, 1.0}, TcuRequest{}, kH);
        const PreSolveResult b = without.pre_solve(s0, ThrottleCommand{c, 0.0, 1.0}, TcuRequest{}, kH);
        (void)with.post_solve(omega, kH);
        (void)without.post_solve(omega, kH);
        REQUIRE(a.tau0 == b.tau0);
        REQUIRE(a.d == b.d);
        REQUIRE(a.turbo_rpm == b.turbo_rpm);
        REQUIRE(a.p_intake_pa == b.p_intake_pa);
        REQUIRE(a.dfco_active == b.dfco_active);
        REQUIRE(a.torque_demand == b.torque_demand);
        REQUIRE(a.anti_lag_active == 0);
        REQUIRE(a.anti_lag_extra_exhaust_w == 0.0);
    }
    CHECK(with.fold_state_hash(0) != without.fold_state_hash(0));
}

// ---------------------------------------------------------------------------
// White-box: the retard and the routed heat against literal fixture values.
// ---------------------------------------------------------------------------
TEST_CASE("D5: spark retard and extra exhaust heat equal the literal lost indicated work", "[turbo][drivetrain][anti_lag][sim_engine]") {
    SimulatedEngine e(engine_with(tb::hyper_like_anti_lag()));
    const real rpm = 5000.0, omega = omega_of(rpm);
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    s.anti_lag_channel = 1.0;
    int checked = 0;
    for (int i = 0; i < 2 * 960 + 480; ++i) {
        const bool wot = i < 2 * 960;
        const PreSolveResult r = e.pre_solve(s, ThrottleCommand{wot ? 1.0 : 0.0, 0.0, 1.0}, TcuRequest{}, kH);
        const PostSolveResult p = e.post_solve(omega, kH);
        if (wot || (i % 48) != 0) continue;
        REQUIRE(r.anti_lag_active == 1);
        // Table2D clamps at the p axis' ends (0..120 kPa); spark and MBT are linear in (rpm, p), so the
        // bilinear tables reproduce the closed forms exactly up to rounding.
        const real P = std::clamp(e.p_im(), 0.0, 120000.0) / 1e5;
        const real u = rpm / 8000.0;
        const real sched = 10.0 + 25.0 * u - 12.0 * P;
        const real mbt = 12.0 + 24.0 * u - 8.0 * P;
        CHECK(r.spark_advance_deg == Catch::Approx(sched - 30.0).margin(1e-11));
        const real tau_comb = p.indicated_work_delta / (kH * omega);
        REQUIRE(tau_comb > 0.0); // fired: DFCO inhibited
        const real expected = omega * tau_comb * (eta_spark_literal(sched - mbt) / eta_spark_literal(sched - 30.0 - mbt) - 1.0);
        CHECK(r.anti_lag_extra_exhaust_w == Catch::Approx(expected).epsilon(1e-11));
        CHECK(r.anti_lag_extra_exhaust_w > 1000.0); // a regime where the term is not small
        ++checked;
    }
    CHECK(checked == 10);
}

// ---------------------------------------------------------------------------
// Lift-off at 5000 rpm on the held crank, ALS on vs off.
// ---------------------------------------------------------------------------
TEST_CASE("D5: lift-off at 5000 rpm - anti-lag keeps the rotor spinning, DFCO returns after max_active_s", "[turbo][drivetrain][anti_lag][sim_engine]") {
    const SimulatedEngineDesc d = engine_with(tb::hyper_like_anti_lag());
    const LiftRun off = run_lift(d, 0.0);
    const LiftRun on = run_lift(d, 1.0);
    AntiLagDesc no_retard = tb::hyper_like_anti_lag();
    no_retard.retard_deg = 0.0;
    const LiftRun on0 = run_lift(engine_with(no_retard), 1.0); // plate floor only: isolates the routed heat
    const int k0 = on.n_wot - 1;
    const int marks[] = {0, 240, 480, 960, 1920};
    for (const char* name : {"off", "on", "on0"}) {
        const std::string n(name);
        const LiftRun& r = n == "on" ? on : (n == "on0" ? on0 : off);
        std::printf("[D5 lift 5000] ALS %-3s", name);
        for (int m : marks) {
            const Sample& x = r.s[std::size_t(k0 + m)];
            std::printf(" | +%.2f s rotor %6.0f rpm boost %+.3f bar dfco %d als %d extra %7.0f W T_port %4.0f K T_turb %4.0f K wg %.3f plate %.3f tau0 %+6.1f fuel %.2f g/s",
                        m / 960.0, x.turbo_rpm, x.boost_bar, int(x.dfco), int(x.als), x.extra_w, x.port_k, x.exhaust_k, x.wastegate, x.plate, x.tau0, x.fuel_g_s);
        }
        std::printf(" | activations %u\n", r.activations);
    }
    const Sample& on1 = on.s[std::size_t(k0 + 960)];
    const Sample& off1 = off.s[std::size_t(k0 + 960)];
    CHECK(on1.turbo_rpm > off1.turbo_rpm + 5000.0);
    // the routed heat reaches the turbine: the same plate floor without the retard spins it slower
    CHECK(on1.turbo_rpm > on0.s[std::size_t(k0 + 960)].turbo_rpm + 1000.0);
    CHECK(on0.s[std::size_t(k0 + 960)].extra_w == 0.0);
    CHECK(on1.als);
    CHECK_FALSE(on1.dfco);
    CHECK(off1.dfco);
    CHECK(on.als_and_dfco == 0);
    // active for exactly max_active_s (1.5 s = 1440 substeps), then DFCO
    int active = 0;
    for (std::size_t i = std::size_t(on.n_wot); i < on.s.size(); ++i) active += on.s[i].als ? 1 : 0;
    CHECK(active == 1440);
    CHECK(on.s[std::size_t(on.n_wot + 1439)].als);
    CHECK_FALSE(on.s[std::size_t(on.n_wot + 1440)].als);
    CHECK(on.s[std::size_t(on.n_wot + 1441)].dfco);
    CHECK(on.activations == 1);
    CHECK(off.activations == 0);
    // ledger: the routed heat never exceeds the heat the substep rejects
    CHECK(on.max_ledger_violation <= 1e-9);
    CHECK(on.min_heat_margin > 0.0);
}

// The plate floor reaches the plate: electronic (target floor, then the 0.05 s
// actuator) and mechanical (the plate itself, a throttle kicker).
TEST_CASE("D5: the plate floor holds the throttle plate while active (electronic and mechanical)", "[turbo][drivetrain][anti_lag][sim_engine]") {
    SimulatedEngineDesc mech = fx::mechanical_engine();
    mech.turbo_pair = tb::hyper_like_pair_with_anti_lag();
    for (const bool mechanical : {false, true}) {
        const SimulatedEngineDesc d = mechanical ? mech : engine_with(tb::hyper_like_anti_lag());
        const LiftRun on = run_lift(d, 1.0, 5000.0, 0.5, 0.6);
        const LiftRun off = run_lift(d, 0.0, 5000.0, 0.5, 0.6);
        const Sample& a = on.s[std::size_t(on.n_wot + 240)];
        const Sample& b = off.s[std::size_t(off.n_wot + 240)];
        std::printf("[D5 plate] %s +0.25 s plate on %.6f off %.6f (als %d)\n", mechanical ? "mechanical" : "electronic", a.plate, b.plate, int(a.als));
        CHECK(a.als);
        CHECK(a.plate >= 0.5 - 1e-6);
        CHECK(b.plate < 0.1);
    }
}

// Diagnostic sweep (not run by default): rotor speed 1 s after a 5000 rpm lift
// over plate floor x retard, with the crank torque the held crank would see.
TEST_CASE("D5 diag: plate floor x retard sweep at 5000 rpm", "[.][diag][anti_lag]") {
    for (real floor : {0.0, 0.1, 0.2, 0.35, 0.5, 0.75, 1.0}) {
        for (real retard : {0.0, 15.0, 30.0, 45.0}) {
            AntiLagDesc al = tb::hyper_like_anti_lag();
            al.plate_floor = floor;
            al.retard_deg = retard;
            al.max_active_s = 3.0;
            SimulatedEngine e(engine_with(al));
            const real omega = omega_of(5000.0);
            Sensors s;
            s.omega = omega;
            s.ignition = 1.0;
            s.anti_lag_channel = 1.0;
            PreSolveResult r;
            for (int i = 0; i < 2 * 960 + 960; ++i) {
                r = e.pre_solve(s, ThrottleCommand{i < 2 * 960 ? 1.0 : 0.0, 0.0, 1.0}, TcuRequest{}, kH);
                (void)e.post_solve(omega, kH);
            }
            std::printf("[D5 diag sweep] floor %.2f retard %4.1f: +1 s rotor %6.0f rpm boost %+.3f bar wg %.3f T_exh %4.0f K "
                        "extra %6.0f W tau0 %+7.1f N*m compressor %.4f kg/s turbine %.0f W compressor %.0f W\n",
                        floor, retard, r.turbo_rpm, r.boost_bar, r.wastegate_opening, e.turbo()->exhaust_k, r.anti_lag_extra_exhaust_w,
                        r.tau0, r.compressor_flow_kg_s, e.turbo()->turbine_w, e.turbo()->compressor_w);
        }
    }
}

// ---------------------------------------------------------------------------
// Each arming condition switches it.
// ---------------------------------------------------------------------------
TEST_CASE("D5: each arming condition switches anti-lag", "[turbo][drivetrain][anti_lag][sim_engine]") {
    const AntiLagDesc al = tb::hyper_like_anti_lag();
    const SimulatedEngineDesc d = engine_with(al);
    const auto active_after_lift = [](const LiftRun& r, int k) { return r.s[std::size_t(r.n_wot + k)].als; };
    // channel
    CHECK(active_after_lift(run_lift(d, 1.0, 5000.0, 0.5, 0.2), 10));
    CHECK_FALSE(active_after_lift(run_lift(d, 0.0, 5000.0, 0.5, 0.2), 10));
    // unwired channel: default_enabled decides
    CHECK_FALSE(active_after_lift(run_lift(d, std::numeric_limits<real>::quiet_NaN(), 5000.0, 0.5, 0.2), 10));
    {
        AntiLagDesc on_default = al;
        on_default.default_enabled = 1.0;
        CHECK(active_after_lift(run_lift(engine_with(on_default), std::numeric_limits<real>::quiet_NaN(), 5000.0, 0.5, 0.2), 10));
    }
    // command at / below arm_below_command
    CHECK_FALSE(active_after_lift(run_lift(d, 1.0, 5000.0, 0.5, 0.2, 0.1), 10));
    CHECK(active_after_lift(run_lift(d, 1.0, 5000.0, 0.5, 0.2, 0.0999), 10));
    // rpm window [3000, 7500] (held crank; 7500 is above the 6800 rpm fuel-cut limiter, so test the upper bound with a lower max)
    CHECK_FALSE(active_after_lift(run_lift(d, 1.0, 2900.0, 0.5, 0.2), 10));
    CHECK(active_after_lift(run_lift(d, 1.0, 3000.0, 0.5, 0.2), 10));
    {
        AntiLagDesc low_max = al;
        low_max.max_rpm = 4500.0;
        CHECK_FALSE(active_after_lift(run_lift(engine_with(low_max), 1.0, 5000.0, 0.5, 0.2), 10));
        CHECK(active_after_lift(run_lift(engine_with(low_max), 1.0, 4400.0, 0.5, 0.2), 10));
    }
    // starving: the first lift substep starves (no fuel at all) and disarms anti-lag for the rest of the lift
    {
        const LiftRun r = run_lift(d, 1.0, 5000.0, 0.5, 0.2, 0.0, 0.0);
        CHECK(r.s[std::size_t(r.n_wot)].als);
        CHECK_FALSE(r.s[std::size_t(r.n_wot + 1)].als);
        CHECK_FALSE(r.s[std::size_t(r.n_wot + 10)].als);
        CHECK(r.activations == 1); // no toggling
    }
    // not Running: ignition off -> Off
    {
        SimulatedEngine e(d);
        const real omega = omega_of(5000.0);
        Sensors s;
        s.omega = omega;
        s.ignition = 1.0;
        s.anti_lag_channel = 1.0;
        for (int i = 0; i < 480; ++i) {
            (void)e.pre_solve(s, ThrottleCommand{1.0, 0.0, 1.0}, TcuRequest{}, kH);
            (void)e.post_solve(omega, kH);
        }
        s.ignition = 0.0;
        const PreSolveResult r = e.pre_solve(s, ThrottleCommand{0.0, 0.0, 1.0}, TcuRequest{}, kH);
        CHECK(r.state == EngineState::Off);
        CHECK(r.anti_lag_active == 0);
    }
    // re-arming: a second lift after the command crossed arm_below_command activates again
    {
        SimulatedEngine e(d);
        const real omega = omega_of(5000.0);
        Sensors s;
        s.omega = omega;
        s.ignition = 1.0;
        s.anti_lag_channel = 1.0;
        const auto run = [&](real c, int n) {
            PreSolveResult r;
            for (int i = 0; i < n; ++i) {
                r = e.pre_solve(s, ThrottleCommand{c, 0.0, 1.0}, TcuRequest{}, kH);
                (void)e.post_solve(omega, kH);
            }
            return r;
        };
        run(1.0, 480);
        CHECK(run(0.0, 1920).anti_lag_active == 0); // used up after 1.5 s
        CHECK_FALSE(e.anti_lag_armed());
        CHECK(run(0.05, 10).anti_lag_active == 0);  // still below the arm threshold: stays disarmed
        run(0.1, 1);                                 // command reaches arm_below_command: re-armed
        CHECK(e.anti_lag_armed());
        CHECK(e.anti_lag_timer_s() == 0.0);
        const PreSolveResult r = run(0.0, 10);
        CHECK(r.anti_lag_active == 1);
        CHECK(r.anti_lag_activations == 2);
    }
}

// ---------------------------------------------------------------------------
// Hashing: state and parameters are folded only when the block exists.
// ---------------------------------------------------------------------------
TEST_CASE("D5: anti-lag state and parameters are hashed", "[turbo][drivetrain][anti_lag][sim_engine]") {
    const auto run = [](const AntiLagDesc& al, real channel) {
        SimulatedEngine e(engine_with(al));
        const real omega = omega_of(5000.0);
        Sensors s;
        s.omega = omega;
        s.ignition = 1.0;
        s.anti_lag_channel = channel;
        for (int i = 0; i < 960; ++i) {
            (void)e.pre_solve(s, ThrottleCommand{1.0, 0.0, 1.0}, TcuRequest{}, kH);
            (void)e.post_solve(omega, kH);
        }
        return e.fold_state_hash(0);
    };
    const AntiLagDesc a = tb::hyper_like_anti_lag();
    const std::uint64_t h = run(a, 1.0);
    CHECK(h == run(a, 1.0));
    CHECK(h == run(a, 0.0)); // at WOT the channel moves nothing
    AntiLagDesc b = a;
    b.retard_deg = 25.0;
    CHECK(run(b, 1.0) != h);
    b = a; b.plate_floor = 0.25; CHECK(run(b, 1.0) != h);
    b = a; b.max_active_s = 1.0; CHECK(run(b, 1.0) != h);
    b = a; b.arm_below_command = 0.15; CHECK(run(b, 1.0) != h);
    b = a; b.min_rpm = 2000.0; CHECK(run(b, 1.0) != h);
    b = a; b.max_rpm = 7000.0; CHECK(run(b, 1.0) != h);
    // the activity state itself: the same engine 0.2 s into a lift with ALS on vs off
    const auto lift = [](real channel) {
        SimulatedEngine e(engine_with(tb::hyper_like_anti_lag()));
        const real omega = omega_of(5000.0);
        Sensors s;
        s.omega = omega;
        s.ignition = 1.0;
        s.anti_lag_channel = channel;
        for (int i = 0; i < 960 + 192; ++i) {
            (void)e.pre_solve(s, ThrottleCommand{i < 960 ? 1.0 : 0.0, 0.0, 1.0}, TcuRequest{}, kH);
            (void)e.post_solve(omega, kH);
        }
        return e.fold_state_hash(0);
    };
    CHECK(lift(1.0) != lift(0.0));
}

// The timer and the armed flag on their own: during an upshift-style ignition cut anti-lag is active (it
// arms on the command) but nothing burns, and with plate floor 0 the plate is the same as in DFCO - so two
// engines whose anti-lag histories differ have bit-identical dynamics, and only the folded anti-lag state
// can tell them apart.
TEST_CASE("D5: the anti-lag timer and armed flag are hashed", "[turbo][drivetrain][anti_lag][sim_engine]") {
    AntiLagDesc al = tb::hyper_like_anti_lag();
    al.plate_floor = 0.0;
    al.max_active_s = 0.05; // 48 substeps
    struct Out {
        std::uint64_t hash;
        std::vector<real> p_im, rotor, tau0;
        bool armed;
        real timer;
    };
    const auto run = [&](int channel_on_substeps) {
        SimulatedEngine e(engine_with(al));
        const real omega = omega_of(5000.0);
        Sensors s;
        s.omega = omega;
        s.ignition = 1.0;
        Out o;
        for (int i = 0; i < 960 + 200; ++i) {
            const bool lift = i >= 960;
            s.anti_lag_channel = (lift && i - 960 < channel_on_substeps) ? 1.0 : 0.0;
            const PreSolveResult r = e.pre_solve(s, ThrottleCommand{lift ? 0.0 : 1.0, 0.0, 1.0}, TcuRequest{lift}, kH);
            (void)e.post_solve(omega, kH);
            o.p_im.push_back(r.p_intake_pa);
            o.rotor.push_back(r.turbo_rpm);
            o.tau0.push_back(r.tau0);
        }
        o.hash = e.fold_state_hash(0);
        o.armed = e.anti_lag_armed();
        o.timer = e.anti_lag_timer_s();
        return o;
    };
    const Out never = run(0), short_on = run(20), used_48 = run(48), used_up = run(60);
    std::printf("[D5 hash] timers %.6f / %.6f / %.6f / %.6f, armed %d %d %d %d\n", never.timer, short_on.timer, used_48.timer,
                used_up.timer, int(never.armed), int(short_on.armed), int(used_48.armed), int(used_up.armed));
    // identical dynamics (bitwise)
    CHECK(never.p_im == short_on.p_im);
    CHECK(never.rotor == short_on.rotor);
    CHECK(never.tau0 == short_on.tau0);
    CHECK(used_48.p_im == used_up.p_im);
    CHECK(used_48.tau0 == used_up.tau0);
    // timer differs -> hash differs
    CHECK(short_on.timer == Catch::Approx(20.0 * kH).epsilon(1e-12));
    CHECK(never.hash != short_on.hash);
    // same timer, only the armed flag differs -> hash differs
    CHECK(used_48.timer == used_up.timer);
    CHECK(used_48.armed);
    CHECK_FALSE(used_up.armed);
    CHECK(used_48.hash != used_up.hash);
}

// ---------------------------------------------------------------------------
// Hot swap: a configuration without anti-lag clears the state; one with it keeps reading the channel.
// ---------------------------------------------------------------------------
TEST_CASE("D5: replace_turbo_configuration without anti-lag clears its state", "[turbo][drivetrain][anti_lag][sim_engine]") {
    SimulatedEngine e(engine_with(tb::hyper_like_anti_lag()));
    const real omega = omega_of(5000.0);
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    s.anti_lag_channel = 1.0;
    for (int i = 0; i < 960 + 96; ++i) {
        (void)e.pre_solve(s, ThrottleCommand{i < 960 ? 1.0 : 0.0, 0.0, 1.0}, TcuRequest{}, kH);
        (void)e.post_solve(omega, kH);
    }
    REQUIRE(e.anti_lag_active());
    REQUIRE(e.anti_lag_extra_exhaust_w() > 0.0);
    e.replace_turbo_configuration(tb::hyper_like_pair());
    CHECK_FALSE(e.anti_lag_active());
    CHECK(e.anti_lag_extra_exhaust_w() == 0.0);
    CHECK(e.anti_lag_timer_s() == 0.0);
    const PreSolveResult r = e.pre_solve(s, ThrottleCommand{0.0, 0.0, 1.0}, TcuRequest{}, kH);
    CHECK(r.anti_lag_active == 0);
    CHECK(r.dfco_active);
}

// ---------------------------------------------------------------------------
// Vehicle level.
// ---------------------------------------------------------------------------
namespace {

ps::vehicle::VehicleDesc turbo_sedan(const TurboPairDesc& pair) {
    ps::vehicle::VehicleDesc d = ps_test::load_sedan_desc();
    SimulatedEngineDesc eng = fx::electronic_engine();
    eng.turbo_pair = pair;
    for (auto& c : d.powertrain.components) {
        if (const auto* tm = std::get_if<TorqueMapEngineDesc>(&c.params)) {
            eng.crank_axis_local = tm->crank_axis_local;
            c.params = eng;
            break;
        }
    }
    return d;
}

real sum_abs_terms(const PowertrainEnergy& e) {
    return std::abs(e.engine_out) + std::abs(e.wheel_out) + std::abs(e.tyre_loss) + std::abs(e.brake_loss) + std::abs(e.gear_loss) +
           std::abs(e.integrator_loss) + std::abs(e.clutch_loss) + std::abs(e.synchro_loss) + std::abs(e.engine_friction_loss) +
           std::abs(e.engagement_loss) + std::abs(e.starter_in) + std::abs(e.external_reset) + std::abs(e.diff_loss);
}

struct VehicleRun {
    std::uint64_t hash = 0;
    std::uint32_t activations = 0;
    int active_ticks = 0;
    real max_extra_w = 0.0, worst_balance = 0.0, worst_fuel_identity = 0.0, rotor_end_of_lift = 0.0;
};

VehicleRun run_vehicle(const TurboPairDesc& pair, int workers, bool arm_alloc, bool toggle) {
    using namespace ps;
    WorldConfig cfg;
    cfg.job_workers = workers;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    World w(cfg);
    BodyDesc ground;
    ground.shape = BoxShape{Vec3{1000.0, 1000.0, 0.5}};
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    ground.pose.position = Vec3{0.0, 0.0, -0.5};
    w.create_body(ground);
    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = Vec3{0.0, 0.0, 0.6};
    const BodyId body = w.create_body(chassis);
    w.set_surface_table(std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json"));
    const VehicleId v = w.create_vehicle(turbo_sedan(pair), body);
    w.set_control("assist.auto_clutch", 1.0);
    w.set_control("assist.auto_blip", 0.0);
    w.set_control("assist.auto_shift", 0.0);
    w.set_control("clutch", 0.0);
    w.set_control("throttle", 0.0);
    vehicle::VehicleResetOptions o;
    o.gear = 1;
    o.engine_state = EngineState::Running;
    o.clutch_locked = false;
    w.reset_vehicle(v, o);
    for (int t = 0; t < 120; ++t) w.step();
    namespace da = ps::jobs::debug_alloc;
    const bool arm = arm_alloc && da::hook_available();
    if (arm) {
        da::reset();
        da::set_armed(true);
    }
    VehicleRun out;
    const auto tick = [&] {
        w.step();
        const PowertrainEnergy& e = w.powertrain_energy(v);
        const real denom = sum_abs_terms(e);
        if (denom > 1e-9) out.worst_balance = std::max(out.worst_balance, std::abs(e.balance_error) / denom);
        out.worst_fuel_identity = std::max(out.worst_fuel_identity,
                                           std::abs(e.fuel_energy_in - e.indicated_work - e.engine_heat) / std::max(1.0, e.fuel_energy_in));
        const auto& es = w.powertrain_state(v).engines.at(0);
        out.active_ticks += es.anti_lag_active;
        out.max_extra_w = std::max(out.max_extra_w, es.anti_lag_extra_exhaust_w);
        out.activations = es.anti_lag_activations;
    };
    w.set_control("anti_lag", 1.0);
    w.set_control("throttle", 1.0);
    for (int t = 0; t < 480; ++t) tick();
    w.set_control("throttle", 0.0);
    for (int t = 0; t < 240; ++t) {
        if (toggle && t == 60) w.set_control("anti_lag", 0.0);
        if (toggle && t == 120) w.set_control("anti_lag", 1.0);
        tick();
    }
    out.rotor_end_of_lift = w.powertrain_state(v).engines.at(0).turbo_rpm;
    if (arm) {
        da::set_armed(false);
        WARN("D5 vehicle debug_alloc (" << workers << " workers): own " << da::own_violation_count() << ", external (Jolt) "
                                        << da::external_violation_count());
        CHECK(da::own_violation_count() == 0);
    }
    out.hash = w.state_hash();
    return out;
}

} // namespace

TEST_CASE("D5: the anti-lag channel is declared only with the block, default = default_enabled", "[turbo][drivetrain][anti_lag]") {
    const auto channels = [](const std::optional<AntiLagDesc>& al) {
        TurboPairDesc pair = tb::hyper_like_pair();
        pair.anti_lag = al;
        return Powertrain::declare_control_channels(turbo_sedan(pair).powertrain);
    };
    CHECK(channels(std::nullopt).empty());
    const auto with = channels(tb::hyper_like_anti_lag());
    REQUIRE(with.size() == 1);
    CHECK(with[0].name == "anti_lag");
    CHECK(with[0].default_value == 0.0);
    AntiLagDesc on = tb::hyper_like_anti_lag();
    on.channel = "als_switch";
    on.default_enabled = 1.0;
    const auto named = channels(on);
    REQUIRE(named.size() == 1);
    CHECK(named[0].name == "als_switch");
    CHECK(named[0].default_value == 1.0);
}

TEST_CASE("D5: vehicle with anti-lag - worker-count invariant, allocation-free, ledger closed", "[turbo][drivetrain][vehicle][anti_lag][determinism]") {
    if (!ps::jobs::debug_alloc::hook_available()) WARN("debug_alloc hook unavailable in this build - allocation check skipped");
    const TurboPairDesc with = tb::hyper_like_pair_with_anti_lag();
    const VehicleRun a1 = run_vehicle(with, 1, true, true);
    const VehicleRun a10 = run_vehicle(with, 10, true, true);
    const VehicleRun steady = run_vehicle(with, 1, false, false);
    const VehicleRun none = run_vehicle(tb::hyper_like_pair(), 1, false, false);
    std::printf("[D5 vehicle] hash 1w %016llx 10w %016llx, untoggled %016llx, no block %016llx; activations %u (untoggled %u), "
                "active ticks %d, max extra %.0f W, rotor at end of lift %.0f rpm (no block %.0f); worst balance %.3e rel, fuel identity %.3e\n",
                static_cast<unsigned long long>(a1.hash), static_cast<unsigned long long>(a10.hash), static_cast<unsigned long long>(steady.hash),
                static_cast<unsigned long long>(none.hash), a1.activations, steady.activations, a1.active_ticks, a1.max_extra_w,
                steady.rotor_end_of_lift, none.rotor_end_of_lift, a1.worst_balance, a1.worst_fuel_identity);
    CHECK(a1.hash == a10.hash);
    CHECK(a1.hash != steady.hash); // the toggle reached the dynamics
    CHECK(a1.hash != none.hash);
    CHECK(a1.activations == 2);    // the lift, then again after the channel came back
    CHECK(steady.activations == 1);
    CHECK(a1.max_extra_w > 0.0);
    CHECK(none.active_ticks == 0);
    CHECK(a1.worst_balance <= 5e-8);
    CHECK(steady.worst_balance <= 5e-8);
    CHECK(a1.worst_fuel_identity <= 1e-12);
}

// ---------------------------------------------------------------------------
// Loader.
// ---------------------------------------------------------------------------
namespace {
using json = nlohmann::json;
using Catch::Matchers::ContainsSubstring;

json base_config() {
    return json::parse(R"({
      "format": "physics_sim.turbo_configuration/1", "name": "als_test", "turbo": "HARDWARE",
      "count": 2, "target_boost_bar": 1.8, "intercooler_effectiveness": 0.95
    })");
}
json valid_als() {
    return json::parse(R"({
      "arm_below_command": 0.1, "min_rpm": 3000, "max_rpm": 7500, "retard_deg": 30,
      "plate_floor": 0.2, "max_active_s": 1.5, "source": "[AS] io test"
    })");
}
TurboPairDesc load(json cfg, const std::string& tag) {
    cfg["turbo"] = std::string(PS_SOURCE_DIR) + "/data/turbos/hyper_turbo_approx.json";
    const auto dir = std::filesystem::temp_directory_path() / "ps_turbo_als_io_test";
    std::filesystem::create_directories(dir);
    const auto path = dir / (tag + ".json");
    std::ofstream(path) << cfg.dump(2);
    return ps::io::load_turbo_configuration_json(path.string());
}
} // namespace

TEST_CASE("D5: anti_lag loader - values and rejections", "[turbo][io][anti_lag]") {
    {
        json c = base_config();
        c["anti_lag"] = valid_als();
        const TurboPairDesc d = load(c, "valid");
        REQUIRE(d.anti_lag.has_value());
        CHECK(d.anti_lag->channel == "anti_lag");
        CHECK(d.anti_lag->default_enabled == 0.0);
        CHECK(d.anti_lag->arm_below_command == 0.1);
        CHECK(d.anti_lag->min_rpm == 3000.0);
        CHECK(d.anti_lag->max_rpm == 7500.0);
        CHECK(d.anti_lag->retard_deg == 30.0);
        CHECK(d.anti_lag->plate_floor == 0.2);
        CHECK(d.anti_lag->max_active_s == 1.5);
        CHECK(d.anti_lag->source == "[AS] io test");
        CHECK_FALSE(load(base_config(), "none").anti_lag.has_value());
        json c2 = base_config();
        json a2 = valid_als();
        a2["channel"] = "als";
        a2["default_enabled"] = 1;
        c2["anti_lag"] = a2;
        const TurboPairDesc d2 = load(c2, "named");
        CHECK(d2.anti_lag->channel == "als");
        CHECK(d2.anti_lag->default_enabled == 1.0);
    }
    const auto reject = [](const char* key, const json& value, const char* message) {
        json c = base_config();
        json a = valid_als();
        if (value.is_null()) a.erase(key);
        else a[key] = value;
        c["anti_lag"] = a;
        CHECK_THROWS_WITH(load(c, std::string("bad_") + key), ContainsSubstring(message));
    };
    reject("arm_below_command", 0.0, "'arm_below_command' must be in (0, 1]");
    reject("arm_below_command", 1.5, "'arm_below_command' must be in (0, 1]");
    reject("arm_below_command", nullptr, "'arm_below_command' is required");
    reject("min_rpm", -1.0, "'min_rpm' must be >= 0");
    reject("max_rpm", 3000.0, "'max_rpm' must be > 'min_rpm'");
    reject("retard_deg", -5.0, "'retard_deg' must be >= 0");
    reject("plate_floor", 1.2, "'plate_floor' must be in [0, 1]");
    reject("plate_floor", -0.1, "'plate_floor' must be in [0, 1]");
    reject("max_active_s", 0.0, "'max_active_s' must be > 0");
    reject("default_enabled", 0.5, "'default_enabled' must be 0 or 1");
    reject("channel", "", "'channel' must be a non-empty string");
    reject("source", nullptr, "'source' is required");
    reject("retard", 30.0, "unknown key 'retard'");
    {
        json c = base_config();
        c["anti_lag"] = 1;
        CHECK_THROWS_WITH(load(c, "not_object"), ContainsSubstring("'anti_lag' must be an object"));
    }
}

// ---------------------------------------------------------------------------
// The shipped hyper V8 (maps generated at load, cached), held 6000 rpm, WOT 2 s
// then a 3 s lift, ALS off vs on with the fixture block (measured; the owner's
// table). Not part of the shipped data (D6).
// ---------------------------------------------------------------------------
TEST_CASE("D5: hyper V8 lift-off with and without anti-lag (measured)", "[turbo][anti_lag][sim_engine][hyper]") {
    ps::jobs::JobSystem pool(10);
    ps::io::EngineMapOptions options;
    options.pool = &pool;
    options.cache_dir = (std::filesystem::temp_directory_path() / "ps_i7d_engine_cache").string();
    SimulatedEngineDesc base = std::get<SimulatedEngineDesc>(
        ps::io::load_engine_json(std::string(PS_SOURCE_DIR) + "/data/engines/hyper_v8_one1_audio.json", options));
    REQUIRE(base.turbo_pair.has_value());
    base.turbo_pair->anti_lag = tb::hyper_like_anti_lag();
    const LiftRun off = run_lift(base, 0.0, 6000.0);
    const LiftRun on = run_lift(base, 1.0, 6000.0);
    SimulatedEngineDesc base0 = base;
    base0.turbo_pair->anti_lag->retard_deg = 0.0;
    const LiftRun on0 = run_lift(base0, 1.0, 6000.0);
    SimulatedEngineDesc base_low = base;
    base_low.turbo_pair->anti_lag->plate_floor = 0.2;
    const LiftRun on_low = run_lift(base_low, 1.0, 6000.0);
    const int k0 = on.n_wot - 1;
    const int marks[] = {0, 240, 480, 960, 1920};
    for (const char* name : {"off", "on", "on0", "on_low"}) {
        const std::string n(name);
        const LiftRun& r = n == "on" ? on : (n == "on0" ? on0 : (n == "on_low" ? on_low : off));
        std::printf("[D5 hyper lift 6000] ALS %-6s", name);
        for (int m : marks) {
            const Sample& x = r.s[std::size_t(k0 + m)];
            std::printf(" | +%.2f s rotor %6.0f rpm boost %+.3f bar dfco %d als %d extra %7.0f W T_port %4.0f K T_turb %4.0f K wg %.3f plate %.3f tau0 %+6.1f fuel %.2f g/s",
                        m / 960.0, x.turbo_rpm, x.boost_bar, int(x.dfco), int(x.als), x.extra_w, x.port_k, x.exhaust_k, x.wastegate, x.plate, x.tau0, x.fuel_g_s);
        }
        std::printf(" | activations %u, heat margin min %.3e J, ledger violation max %.3e J\n", r.activations, r.min_heat_margin,
                    r.max_ledger_violation);
    }
    CHECK(on.s[std::size_t(k0 + 960)].turbo_rpm > off.s[std::size_t(k0 + 960)].turbo_rpm);
    CHECK(on.als_and_dfco == 0);
    CHECK(on.max_ledger_violation <= 1e-9);
}
