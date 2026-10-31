// tests/unit/test_turbo_compressor_flow.cpp -- PHYS-033 P4 I7d fix: the
// compressor delivers the THROTTLE-path mass flow, not the cylinder flow.
//
// Defect (found on the I7d D1-D4 merge b486223): SimulatedEngine::pre_solve fed
// TurboPair::step the cylinder air flow (m_air_rev*|omega|/2pi). The compressor
// feeds the charge pipe, which feeds the throttle; the cylinders draw from the
// intake manifold behind it. At steady state the two flows are equal, but on a
// lift-off the throttle flow collapses at once while the cylinders keep pumping
// the manifold down for ~100 ms, so the model kept the compressor at ~0.4 kg/s
// against the full pressure ratio. The turbo now sees max(0, ManifoldRhs::m_thr)
// of the air step (m_thr = throttle plate + idle-air bypass flow, the whole path
// from the charge pipe into the manifold; negative = reverse flow out of the
// manifold, which does not pass the compressor forward), relaxed with
// kTurboFlowLagS = 5 ms (engine_simulated.cpp, PROVISIONAL) and handed over one
// substep later, exactly as the cylinder flow was (the turbo step runs before
// the air step). The relaxation is needed for stability: fed raw, the loop
// charge pressure -> throttle flow -> compressor flow -> head curve -> charge
// pressure has a one-substep delay and a gain above 1 near the end of the
// compressor map; a held 7500 rpm WOT ran away through a period-2 oscillation
// at t = 3 s (rotor to its 1.5x cap, 3.2 bar instead of 1.70 bar).
//
// Second change in the same commit (PROVISIONAL, kExhaustHeatHoldS = 0.5 s): the
// turbine-side exhaust heat (burnt fuel power) rises with the fuel flow at once
// and decays with 0.5 s once the fuel flow falls. Measured on the shipped hyper
// V8 (held 6000 rpm, WOT 2 s, then throttle 0): with the compressor flow fixed
// alone the turbine power still fell 97 kW -> 9.6 kW within 10 ms of the lift
// (fuel cut: exhaust gas at ambient temperature at once) and the rotor, 2.2 kJ =
// 23 ms of WOT compressor power, was at 0 rpm 75 ms after the lift. Only with
// the exhaust heat held does the rotor coast (see CF2).
//
// SUPERSEDED (exhaust energy balance + wall thermal mass): the hold is gone. The
// port gas temperature now comes from the engine's energy balance and passes a
// lumped exhaust wall; after a fuel-cut lift the air gets only the wall's heat
// (turbine 10.7 kW at +10 ms against 90.7 kW compressor from a 1000 K wall), so
// the shipped [AS] rotor (2e-5 kg*m^2, ~2.4 kJ at the lift) is at 0 rpm ~0.1 s
// after the lift again - the plate (tau 40 ms) needs ~50 ms to cut the
// compressor flow, and that drain alone exceeds the rotor's energy. A 5x rotor
// inertia coasts (76k rpm at +0.25 s, 72.6k at +0.5 s; x10: 90k / 88k).
// CF2 therefore ran on a TEST RIG (rotor inertia x5). Review fix 2026-10-05: the
// shipped rotor is now 2.3e-4 kg*m^2 per turbo (derived for a ~500 kW unit, see
// data/turbos/hyper_turbo_approx.json) and CF2 runs on shipped data again.
//
// Tests
//   CF1  White-box, every substep of WOT -> lift -> WOT on the fixture engine:
//        the published compressor flow equals a reference relaxation (re-
//        implemented here with the literal 5 ms) of max(0, m_thr), m_thr
//        recomputed independently through the public SimulatedEngine::
//        manifold_rhs, to 1e-9 relative, while the cylinder flow differs from
//        the compressor flow by more than 0.04 kg/s during the lift. Feeding the
//        cylinder flow back fails it (sabotage in the commit message).
//   CF2  Lift-off spin-down on the shipped hyper V8 (held 6000 rpm, WOT 2 s, then
//        throttle 0). Energy estimate (old 2e-5 rotor): E0 = J*w0^2/2 = 2.2 kJ at
//        the lift; 2.3e-4 rotor: 28 kJ.
//        The plate (actuator tau 40 ms, area fraction convex in lift) closes in
//        ~100 ms; until then the compressor delivers the throttle flow and draws
//        up to 96 kW, the turbine (wall-heated air) returns a little of it. After
//        0.15 s the throttle flow is < 0.03 kg/s: the rotor can then lose energy
//        only through the shaft loss (loss_ref*s^2, falling with speed) and the
//        compressor power at that small flow, and the turbine only adds. The
//        test integrates both from the published flow/pressure ratio and asserts
//        E(0.5 s) >= E(0.15 s) - int(P_comp + P_loss) (a coast must not be
//        drained by anything else), plus the absolute floors rotor(0.15 s) >=
//        65000 rpm and rotor(0.5 s) >= 60000 rpm (measured-with-margin pins on
//        the x5 rig: measured 73.3k / 69.1k; 75.0k / 70.7k after the review fix
//        (parallel wastegate, compressor bound); shipped 2.3e-4 rotor 92.9k /
//        90.1k (floors kept); cylinder flow fed to the
//        compressor instead: 63.8k / 59.6k, fails both). With the shipped
//        rotor and the former heat
//        hold the floors were 25k / 20k (measured 38.5k / 29.6k).
//   CF3  Steady WOT hold at 3000..8200 rpm: steady-state m_thr == m_cyl, so the
//        pre-fix values hold (literal pre-fix numbers below, recorded on
//        b486223); plus the stability check the raw feed failed.
//
// Measured numbers (pre-fix vs post-fix) are in the task report; the printf
// lines "[CF ...]" print them.

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
#include <functional>
#include <optional>
#include <string>
#include <vector>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

using namespace ps::drivetrain;
using ps::real;
namespace fx = ps::test::mvem;
namespace tb = ps::test::turbo;

namespace {

constexpr real kH = 1.0 / 960.0;
constexpr real kPi = 3.14159265358979323846;
constexpr real kFlowLagS = 0.005; // literal copy of engine_simulated.cpp's kTurboFlowLagS

SimulatedEngineDesc load_hyper() {
    ps::jobs::JobSystem pool(10);
    ps::io::EngineMapOptions options;
    options.pool = &pool;
    options.cache_dir = (std::filesystem::temp_directory_path() / "ps_i7d_engine_cache").string();
    return std::get<SimulatedEngineDesc>(
        ps::io::load_engine_json(std::string(PS_SOURCE_DIR) + "/data/engines/hyper_v8_one1_audio.json", options));
}

// One substep's record. m_thr is the throttle flow of the air step done INSIDE
// pre_solve i, recomputed from the engine's public manifold RHS right after it
// (p0 = the manifold pressure the previous substep left, the plate of this
// substep, the engine's turbo charge state is then the one the air step used).
// m_cyl is the cylinder air flow of the same substep (exhaust - fuel).
struct Rec {
    real t = 0, pim = 0, rpm = 0, boost_bar = 0, charge_pa = 0, comp_flow = 0, m_thr = 0, m_cyl = 0, pr = 0, plate = 0;
};

// Runs n steps on a held crank; command(i) is the throttle command.
std::vector<Rec> run_held(SimulatedEngine& e, real rpm_held, int n, const std::function<real(int)>& command) {
    const real omega = rpm_held * 2.0 * kPi / 60.0;
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    std::vector<Rec> out;
    out.reserve(n);
    real p_prev = 0.0;
    bool have_prev = false;
    for (int i = 0; i < n; ++i) {
        const PreSolveResult r = e.pre_solve(s, ThrottleCommand{command(i), 0.0, 1.0}, TcuRequest{}, kH);
        (void)e.post_solve(omega, kH);
        Rec c;
        c.t = i * kH;
        c.rpm = r.turbo_rpm;
        c.boost_bar = r.boost_bar;
        c.charge_pa = r.p_charge_pa;
        c.comp_flow = r.compressor_flow_kg_s - r.bov_flow_kg_s;
        c.pr = r.compressor_pressure_ratio;
        c.plate = r.throttle_position;
        c.pim = r.p_intake_pa;
        c.m_cyl = r.exhaust_mass_flow_kg_s - r.fuel_flow_g_s * 1e-3;
        // Electronic throttle (every engine used here): x_iac = 0, the plate alone sets the area.
        c.m_thr = have_prev ? e.manifold_rhs(p_prev, omega, r.throttle_position, 0.0).m_thr : 0.0;
        p_prev = r.p_intake_pa;
        have_prev = true;
        out.push_back(c);
    }
    return out;
}

} // namespace

TEST_CASE("CF1: the compressor flow is the throttle flow of the previous substep, not the cylinder flow",
          "[turbo][drivetrain][sim_engine][compressor_flow]") {
    auto d = fx::electronic_engine();
    d.turbo_pair = tb::hyper_like_pair();
    SimulatedEngine e(d);
    // WOT 0-1 s, lift 1-2 s, WOT 2-3 s at a held 5000 rpm.
    const int kLift = 960, kReapply = 1920;
    const auto rec = run_held(e, 5000.0, 3 * 960, [&](int i) { return (i < kLift || i >= kReapply) ? 1.0 : 0.0; });
    const real a = kH / (kFlowLagS + kH);
    // Reference relaxation of max(0, m_thr): f_i is what the air step of substep i hands to the turbo step of
    // substep i+1, which publishes it as compressor_flow_kg_s. Seeded from the published value of substep 2
    // (= f_1), since m_thr of substep 0 (no previous manifold pressure) is not recomputed.
    real f = rec[2].comp_flow;
    int checked = 0;
    real max_diff_lift = 0.0, max_cyl_lift = 0.0, max_comp_lift = 0.0;
    for (std::size_t i = 2; i + 1 < rec.size(); ++i) {
        f += a * (std::max(real(0.0), rec[i].m_thr) - f);
        CHECK(rec[i + 1].comp_flow == Catch::Approx(f).epsilon(1e-9).margin(1e-12));
        ++checked;
        if (int(i) >= kLift && int(i) < kReapply) {
            max_diff_lift = std::max(max_diff_lift, std::abs(rec[i + 1].comp_flow - rec[i].m_cyl));
            max_cyl_lift = std::max(max_cyl_lift, rec[i].m_cyl);
            max_comp_lift = std::max(max_comp_lift, rec[i + 1].comp_flow);
        }
    }
    std::printf("[CF1] %d substeps checked; during the lift |compressor flow - cylinder flow| peaks at %.4f kg/s "
                "(cylinder flow peak %.4f, compressor flow peak %.4f)\n",
                checked, max_diff_lift, max_cyl_lift, max_comp_lift);
    CHECK(max_diff_lift > 0.04); // the two flows genuinely differ on the lift, so the equality above discriminates
}

TEST_CASE("CF2: hyper V8 lift-off spin-down (measured, energy bound)", "[turbo][sim_engine][hyper][compressor_flow]") {
    SimulatedEngineDesc base = load_hyper();
    REQUIRE(base.turbo_pair.has_value());
    // 2026-10-13: the shipped hyper twin configuration now carries a blow-off valve (hyper BOV). This case measures the
    // lift-off energy bound (compressor power from the engine's own flow, no vent), which the valve's vent flow (extra compressor load) changes by design; it keeps its subject by
    // clearing the valve. The valve itself is covered by test_turbo_bov.cpp and test_hyper_bov.cpp.
    base.turbo_pair->blow_off_valve.reset();
    // Shipped data (rotor 2.3e-4 kg*m^2 per turbo since the review fix; the x5 test rig on the old 2e-5 [AS]
    // rotor is gone, see the header).
    const TurboPairDesc& t = *base.turbo_pair;
    SimulatedEngine e(base);
    const int n_wot = 2 * 960, n_total = n_wot + 2 * 960 + 1;
    const auto rec = run_held(e, 6000.0, n_total, [&](int i) { return i < n_wot ? 1.0 : 0.0; });
    const Rec& at_lift = rec[n_wot - 1];
    std::printf("[CF2 lift] at the lift: boost %.3f bar, rotor %.0f rpm, charge %.0f Pa, compressor flow %.4f, throttle flow %.4f, cylinder flow %.4f kg/s\n",
                at_lift.boost_bar, at_lift.rpm, at_lift.charge_pa, at_lift.comp_flow, at_lift.m_thr, at_lift.m_cyl);
    for (int ms : {0, 10, 20, 30, 40, 50, 75, 100, 150, 250, 500, 1000, 2000}) {
        const int k = n_wot + int(std::lround(ms * 0.96)) - (ms == 0 ? 1 : 0);
        const Rec& r = rec[std::min<int>(k, int(rec.size()) - 1)];
        std::printf("[CF2 lift] +%4d ms: rotor %7.0f rpm | charge %8.0f Pa (boost %.3f bar) | pr %.3f | compressor flow %.4f | throttle flow %.4f | cylinder flow %.4f kg/s | plate %.4f\n",
                    ms, r.rpm, r.charge_pa, r.boost_bar, r.pr, r.comp_flow, r.m_thr, r.m_cyl, r.plate);
    }

    // Coast energy ledger from t1 = 0.15 s to t2 = 0.5 s after the lift (turbine power >= 0 is dropped).
    const int k1 = n_wot + 144, k2 = n_wot + 480;
    const real wr = t.reference_rpm * 2.0 * kPi / 60.0;
    const auto energy = [&](real rpm) {
        const real w = rpm * 2.0 * kPi / 60.0;
        return 0.5 * t.rotor_inertia_kgm2 * w * w;
    };
    real drain = 0.0;
    for (int k = k1 + 1; k <= k2; ++k) {
        const real w = rec[k].rpm * 2.0 * kPi / 60.0;
        const real p_loss = t.shaft_loss_w_at_reference * (w * w) / (wr * wr);
        const real p_comp = rec[k].comp_flow * 1005.0 * 288.15 * (std::pow(rec[k].pr, 2.0 / 7.0) - 1.0) / t.compressor_efficiency;
        drain += kH * (p_loss + p_comp);
    }
    const real e1 = energy(rec[k1].rpm), e2 = energy(rec[k2].rpm);
    std::printf("[CF2 bound] E(0.15 s) %.1f J, E(0.5 s) %.1f J, integrated shaft loss + compressor power over the coast %.1f J "
                "(E(0.15) - drain = %.1f J); rotor 0.15 s %.0f rpm, 0.5 s %.0f rpm\n",
                e1, e2, drain, e1 - drain, rec[k1].rpm, rec[k2].rpm);
    CHECK(e2 >= e1 - drain - 2.0); // 2 J: the turbine adds ~0 here; slack for the independent re-derivation of the compressor power
    CHECK(rec[k1].rpm >= 65000.0);
    CHECK(rec[k2].rpm >= 60000.0);
}

TEST_CASE("CF3: steady WOT boost and torque by speed", "[turbo][sim_engine][hyper][compressor_flow]") {
    const SimulatedEngineDesc base = load_hyper();
    REQUIRE(base.turbo_pair.has_value());
    struct Pre { real rpm, tau0, boost, rotor; };
    // 6 s held WOT from a cold exhaust wall, last 1 s mean for tau0/boost. Re-recorded for the exhaust energy
    // balance (turbine inlet 1250-1310 K from the engine's own energy balance instead of the 1200 K cap):
    // tau0 +0.2..+0.8 %, boost +0.0..+0.3 %, rotor +0.0..+0.07 %. Recorded on b486223 (cylinder flow fed to the
    // compressor, 25 % fuel heat capped at 1200 K): 1256.2183/1.7807/79052.6, 1364.4133/1.7677/85994.5,
    // 1396.9949/1.7298/106769.1, 1299.3794/1.7007/138456.6, 1210.2170/1.6968/145154.3 - the compressor-flow fix
    // itself moved none of them beyond these tolerances.
    // Re-recorded for the review fix (parallel wastegate port, compressor bound): an unsaturated equilibrium is
    // the same in both wastegate forms (p_exh - p_amb = m_turbine^2/(2 rho A_t^2)), the gate now sits at
    // 0.17 / 0.40 / 0.63 / 0.70 / 0.71; tau0 -0.04..+0.06 %, boost -0.07..+0.09 %, rotor -0.02..+0.17 %. Was
    // 1258.2169/1.7832/79106.9, 1368.1835/1.7687/86025.6, 1405.4251/1.7306/106815.2, 1308.6220/1.7011/138521.5,
    // 1219.7386/1.6972/145229.3 (9fd087c).
    // Re-recorded for the derived 2.3e-4 rotor (was 2e-5 [AS]): a steady equilibrium does not depend on the
    // inertia, but the 6 s hold from a cold wall with the boost PID is not fully settled and the larger rotor
    // answers the gate more slowly; the last-second means moved tau0 -0.01..+0.26 %, boost -0.11..+0.10 %,
    // rotor -0.87..+0.04 % (the largest at 7500/8000 rpm). Was 1257.6810/1.7820/79088.8, 1368.4982/1.7693/86046.5,
    // 1406.0689/1.7321/106902.6, 1309.3552/1.7027/138720.8, 1220.4495/1.6988/145471.0 (258f5c2).
    // Re-recorded 2026-10-19 for the hyper on E85 with the knock limit (margin 2) under knock model v3 (v4: identical), which does not
    // bind at WOT (knock retard 0 at every point of the engine WOT table): tau0 -0.4..-1.2 %, boost -0.01..+0.01 %,
    // rotor -0.6..+0.01 % - the fuel change, not the knock limit. Was 1257.5909/1.7819/79091.6, 1369.2901/1.7710/86075.2,
    // 1406.5556/1.7333/106941.9, 1309.6103/1.7009/137741.7, 1223.6028/1.7006/144205.3 (E5, no knock limit).
    // Re-recorded 2026-10-20 (burn v3 / knock v6, owner 2026-10-20: hyper is a guess, not tuned): the burn law's
    // pressure exponent burns the boosted charge faster; tau0 +1.7..+4.1 % (the most at 6000 rpm), boost -0.02..+0.08 %, rotor -0.01..+0.19 %.
    // Was 1252.1514/1.7821/79095.1, 1361.0286/1.7709/86066.6, 1394.3545/1.7334/106812.2, 1294.9342/1.7011/137083.9,
    // 1208.5143/1.7003/143267.8.
    const Pre pre[] = {{3000.0, 1273.3113, 1.7818, 79092.5}, {4500.0, 1387.0448, 1.7708, 86065.2},
                       {6000.0, 1451.9154, 1.7335, 106804.5}, {7500.0, 1321.2979, 1.7024, 137270.3},
                       {8000.0, 1233.1793, 1.7015, 143538.5}};
    for (real rpm : {3000.0, 4500.0, 6000.0, 7000.0, 7500.0, 7750.0, 8000.0, 8200.0}) {
        SimulatedEngine e(base);
        const real omega = rpm * 2.0 * kPi / 60.0;
        Sensors s;
        s.omega = omega;
        s.ignition = 1.0;
        PreSolveResult r;
        real tau_avg = 0.0, boost_avg = 0.0;
        const int n = 6 * 960;
        for (int i = 0; i < n; ++i) {
            r = e.pre_solve(s, ThrottleCommand{1.0, 0.0, 1.0}, TcuRequest{}, kH);
            (void)e.post_solve(omega, kH);
            if (i >= n - 960) { tau_avg += r.tau0 / 960.0; boost_avg += r.boost_bar / 960.0; }
        }
        std::printf("[CF3 steady] %5.0f rpm WOT 6 s: tau0 %.4f N*m, boost %.4f bar, rotor %.1f rpm, charge %.1f Pa, p_im %.1f Pa, wastegate %.4f, comp flow %.5f kg/s\n",
                    rpm, tau_avg, boost_avg, r.turbo_rpm, r.p_charge_pa, r.p_intake_pa, r.wastegate_opening, r.compressor_flow_kg_s);
        // Stability: no runaway through the compressor-flow loop (the raw feed went to 3.2 bar / the rotor cap at 7500).
        CHECK(boost_avg > 1.60);
        CHECK(boost_avg < 1.80);
        CHECK(r.turbo_rpm < 1.25 * base.turbo_pair->reference_rpm); // the rotor cap is 1.5x
        for (const Pre& p : pre) {
            if (p.rpm != rpm) continue;
            CHECK(tau_avg == Catch::Approx(p.tau0).epsilon(1e-3));
            CHECK(boost_avg == Catch::Approx(p.boost).margin(2e-3));
            CHECK(r.turbo_rpm == Catch::Approx(p.rotor).epsilon(1e-3));
        }
    }
}
