// tests/unit/test_n2o_turbo.cpp -- N2O plan N3 (out/n2o_plan.md section 5, 8): the
// wet kit on a TURBO simulated engine - the exhaust energy balance (turbine heat),
// the work the kit's spark retard removes (retard heat), the exhaust mass flow, and
// the spool/boost consequence on the hyper V8.
//
// What N2 had already done (out/n2o_n2_progress.md), not redone here:
//   - T_in of the cooling balance is the turbo's charge_k (E1 turbo section: 293.05 K);
//   - the exhaust mass flow includes mdot_n and the kit fuel (booked fuel);
//   - torque_demand = max(table, uncut torque with the charge input) (D-N2-5).
// What N3 changes (engine_simulated.cpp, the exhaust balance of pre_solve):
//   - with the wet charge model the balance follows the engine's own release
//     bookkeeping (D-N2-9) instead of the ECU lambda over the booked fuel: only the
//     fuel of the FIRED charges burns (kit fuel through a cut cylinder leaves as
//     chemical enthalpy), it burns at the COMBINED lambda (eta_c = min(1, lambda_c)),
//     and the decomposition heat of the burnt N2O (n2o_energy_in) is released in full:
//         fuel_w + n2o_w = indicated_w + wall_w + sensible_w + chemical_w - clamp_w;
//   - the kit retard's lost indicated work (D5 P2 rule, relative to the knock-limited
//     schedule) is reported as ExhaustSplit::n2o_retard_w. It needs no extra term: the
//     balance books the RETARDED indicated work, so the lost part is already inside
//     sensible_w, and from there the lumped exhaust wall (the held heat).
// Differences from the brief's wording (the code on master moved on): there is no
// peak-held turbo_fuel_w_ (kExhaustHeatHoldS is gone since 4deb7cc: the exhaust wall
// is the hold) and no turbo_als_w_/extra_exhaust_w (D5's extra heat is the work the
// retard removes, which the balance gets by construction); the brief's two deliverables
// map to the exhaust balance above. See out/n2o_n3_progress.md.
//
//   T1 exhaust balance at a frozen turbo state against the test's own bisection solve
//      (decomposition power, burnt-fuel release at lambda_c, chemical share, port
//      temperature, exhaust flow, compressor flow stays the throttle-path air); a cut
//      substep (kit fuel passes unburnt, nothing released)
//   T2 retard heat equals the lost indicated work (white-box, against the test's own
//      eta_spark reads, knock-limited schedule) and lands in the exhaust wall
//   T3 spool comparison: hyper V8 (fixture car_hyper + kit) 3rd-gear WOT from 3000 rpm,
//      and a dyno-held steady state, with and without N2O; plus a short fixture-engine
//      variant that is not [long]
//   T4 D5 guard: an active block with zero N2O flow is bit-identical on a turbo+anti-lag
//      engine (every substep, the state hash, the exhaust split)
//   T5 per-substep closure on the turbo engine with N2O (ledger terms, exhaust balance)
//   T6 whole vehicle: per-tick ledger closure, bottle closure, 1 vs 4 workers, zero alloc
//
// Measured numbers (release, Windows clang-cl, 2026-10-06). Labels: [M] measured here, [AS] assumed
// (kit, 4 deg retard, 0.20 exhaust wall fraction), [C] cited (dH_dec 82.05 kJ/mol).
//
//  T1 (fixture turbo engine, 5000 rpm WOT, 35.6 g/s N2O + 9.4 g/s kit fuel, 2 deg retard) [M]:
//     p_im 245.3 kPa, lambda_c 0.741 (ECU 0.908); fuel 1232.9 kW (kit 413.6), N2O decomposition 66.37 kW,
//     released 980.0, indicated 458.1, wall 196.0, sensible 325.9, chemical 319.3 kW; exhaust 0.3088 kg/s,
//     port gas 1253.0 K, turbine inlet 1004.5 K. The PRE-N3 rule (ECU lambda over the booked fuel, no decomposition)
//     would have said 1581.5 K: the kit fuel makes the combined mixture richer than the ECU's lambda, so less of the
//     fuel burns - the old rule OVERSTATED the exhaust heat of a shot by 328 K here. Decomposition alone is worth
//     +156 K at the port. Cut substep: fuel 413.6 kW (kit fuel only) all chemical, sensible 0, port == charge 293.39 K.
//  T2 [M]: retard 4 deg, no knock map: lost work 16.292 kW == the test's own D5 P2 value (rel < 1e-9), port gas
//     +48.0 K; with a knock map (limit 9.95 deg under the 11.25 deg schedule) 13.644 kW / +40.2 K; retard 1 deg below
//     the knock limit costs exactly 0. After 2 s the retarded engine's turbine inlet is +36 K, its exhaust wall +0.93 K
//     (a 9200 J/K wall: slow).
//  T3 [M], whole car (fixture car_hyper + wet kit, 3rd gear WOT from 3000 rpm, shot 35.6 g/s N2O / up to 16.2 g/s fuel):
//                      without N2O        with N2O
//     rotor +0.25 s     8538 rpm          17701 rpm
//     rotor +0.5 s     21225 rpm          44978 rpm   (2.1x; boost 0.115 -> 0.531 bar)
//     rotor +1.0 s     51047 rpm          95258 rpm   (boost 0.695 -> 1.523 bar, wastegate 0.027 -> 0.391)
//     time to 0.5 / 1.0 / 1.5 bar:  0.892 / 1.154 / 1.462 s   vs   0.487 / 0.717 / 0.967 s
//     (the kit's rpm window ends at 6000 rpm, so the shot stops after ~1 s and the with-N2O peak, 1.578 bar, is
//      below the 1.664 bar the plain car reaches later; the spool advantage is the point, not the peak)
//     Dyno-held 5000 rpm, shot flows from the car run: rotor +0.5 s 70631 -> 76869 rpm, steady boost 1.7597 vs
//     1.7596 bar (BOTH 0.04 bar under the 1.8 bar target: the existing PID/intercooler offset, not N3's), steady
//     wastegate 0.492 -> 0.596 (further open), rotor 91203 vs 91292 rpm, port gas 1333 -> 1397 K, torque 1409 -> 1558
//     N*m, exhaust flow 0.7219 -> 0.7751 kg/s.
//     Short variant (fixture engine, 3000 rpm held, 1.5 s pre-spooled without the shot): rotor +0.5 s 14662 vs 48970.
//     Runtime: 0.4 s with a warm engine-map cache, 9.2 s with a cold cache (the 4-worker map generation of the hyper
//     V8, run once per process): T3 and T6 are [long]; T3s is the short variant.
//  T4 [M]: 3840 substeps identical, anti-lag active in 1000 of them. T5 [M]: 3600 substeps (2058 fired, 1542 cut,
//     1200 of them limiter cuts) close to 1.9e-16. T6 [M]: ledger closure 2.4e-15, bottle 2.7e-13 kg, hash
//     0x4d80a266de635162 at 1 and 4 workers, own allocations 0 (Jolt's known external gap 304).
// Sabotages (each built, run, reverted; out/n2o_n3_progress.md): n2o_w left out of released -> T1, T3s, T5;
// pre-N3 rule -> T1, T2, T5; retard report dropped -> T2; kit fuel through a cut counted as released -> T1, T5;
// gate on `active` alone -> T4; retard double counted -> T1, T2, T5; ECU lambda for the burn -> T1, T5.
// PS_N3_ENGINE_CACHE=<empty dir> forces a cold map generation (measurement hook).
#include "drivetrain/engine_simulated.h"
#include "drivetrain/nitrous_flow.h"
#include "drivetrain/turbo_pair.h"

#include "mvem_fixtures.h"
#include "turbo_fixtures.h"
#include "vehicle_test_helpers.h"

#include "ps/drivetrain/oxidizer_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/nitrous_io.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

using namespace ps;
using namespace ps::drivetrain;
namespace fx = ps::test::mvem;
namespace tb = ps::test::turbo;

namespace {

using ExhaustSplit = SimulatedEngine::ExhaustSplit;

constexpr real kPi = 3.14159265358979323846;
constexpr real kH = 1.0 / 960.0;
constexpr real kR = 287.05;             // [C] ISO 2533 / isa_air.h
constexpr real kCpAir = kR * 1.4 / 0.4; // [D]
constexpr real kMAir = 0.02896442;      // [C] ISO 2533:1975, kg/mol
constexpr real kO2Air = 0.2314;         // [D] ISO 2533 dry-air composition
constexpr real kHinSat950 = 256.52e3;   // [C] NIST saturated liquid h at 305.41 K / 66.247 bar abs (~950 psig), plan 1.1
constexpr real kCpExhaust = 1100.0;     // TurboPair::kExhaustCp, typed in
constexpr real kWallFraction = 0.20;    // TurboPairDesc::in_cylinder_wall_heat_fraction default [AS], typed in
constexpr real kLhvE5 = 44.0e6;         // mvem_fixtures gasoline(), typed in
constexpr real kDhDecLiteral = 82050.0 / 0.0440128; // J/kg: 82.05 kJ/mol [C Chase 1998] / 44.0128 g/mol [C]

real w_of(real rpm) { return rpm * 2.0 * kPi / 60.0; }
real rel(real a, real b) { return std::abs(a - b) / std::max({std::abs(a), std::abs(b), 1e-300}); }

std::string data_path(const std::string& r) { return std::string(PS_SOURCE_DIR) + "/data/" + r; }
std::string fixture_path(const std::string& n) { return std::string(PS_SOURCE_DIR) + "/tests/support/fixtures/" + n; }

const OxidizerDesc& oxidizer() {
    static const OxidizerDesc d = io::load_oxidizer_json(data_path("oxidizers/nitrous_oxide.json"));
    return d;
}
const OxidizerTables& tables() {
    static const OxidizerTables t(oxidizer());
    return t;
}
real ox_molar() { return oxidizer().molar_mass_g_mol * 1e-3; }
real ox_dh() { return oxidizer().formation_enthalpy_kj_mol * 1e3 / ox_molar(); }
real ox_ko2() { return oxidizer().oxygen_mass_fraction / kO2Air; }

// h_g(T) on the 1 bar isobar: linear between rows, extrapolated along the end rows.
real ref_h_g(real t) {
    const auto& ts = oxidizer().gas_1bar.t_k;
    const auto& hs = oxidizer().gas_1bar.h_kj_kg;
    std::size_t i = 0;
    while (i + 2 < ts.size() && t > ts[i + 1]) ++i;
    return 1e3 * (hs[i] + (hs[i + 1] - hs[i]) * (t - ts[i]) / (ts[i + 1] - ts[i]));
}

struct RefCharge {
    real a = 0, t_mix = 0, n = 0, kf = 0, a_eq = 0, f_ecu = 0, f = 0, lambda_c = 0, lambda_t = 0;
};

// Section 1.2 steps 1-4 by bisection on T (the test's own solve, as test_n2o_engine.cpp).
RefCharge ref_charge(const SimulatedEngineDesc& d, real rpm, real p, real t_in, real mdot_n, real mdot_kf, real h_in) {
    RefCharge c;
    const real omega = w_of(rpm);
    c.n = mdot_n * 2.0 * kPi / omega;
    c.kf = mdot_kf * 2.0 * kPi / omega;
    const real vd = real(d.piston.cylinders) * kPi / 4.0 * d.piston.bore_m * d.piston.bore_m * d.piston.stroke_m;
    const real x = d.maps.ve.value(rpm, p) * p * vd * 0.5 / kR;
    const real r = kMAir / ox_molar();
    auto resid = [&](real t) {
        const real a = x / t - r * c.n;
        return a * kCpAir * (t_in - t) - c.n * (ref_h_g(t) - h_in);
    };
    real lo = 100.0, hi = t_in + 1.0;
    for (int i = 0; i < 300; ++i) {
        const real mid = 0.5 * (lo + hi);
        if (mid <= lo || mid >= hi) break;
        (resid(mid) > 0.0 ? lo : hi) = mid;
    }
    c.t_mix = 0.5 * (lo + hi);
    c.a = x / c.t_mix - r * c.n;
    c.a_eq = c.a + ox_ko2() * c.n;
    c.lambda_t = d.ecu.lambda_target.value(rpm, p);
    const real afr = d.fuels[0].afr_stoich;
    c.f_ecu = c.a / (c.lambda_t * afr);
    c.f = c.f_ecu + c.kf;
    c.lambda_c = c.a_eq / (afr * c.f);
    return c;
}

// eta_i(a_eq)*eta_spark at spark = min(schedule - retard, theta_kl).
real ref_gs(const SimulatedEngineDesc& d, real rpm, real p, const RefCharge& c, real retard) {
    real spark = d.ecu.spark_advance_deg.value(rpm, p) - retard;
    if (d.maps.has_theta_kl) spark = std::min(spark, d.maps.theta_kl_deg.value(rpm, p));
    return d.maps.eta_i.value(rpm, c.a_eq) * d.maps.eta_spark.evaluate(spark - d.maps.theta_mbt_deg.value(rpm, p));
}

// Heat released per rev in the fired charges (fire 1, s 1): f_lambda(lambda_c)*LHV*f + dH*n.
real ref_heat(const SimulatedEngineDesc& d, const RefCharge& c) {
    return d.maps.f_lambda.evaluate(c.lambda_c) * d.fuels[0].lhv_j_kg * c.f + ox_dh() * c.n;
}

NitrousInput nitrous_sensor(real mdot_n, real mdot_kf, real h_in, real retard) {
    NitrousInput n;
    n.mdot_n_kg_s = mdot_n;
    n.mdot_kf_kg_s = mdot_kf;
    n.h_in_j_kg = h_in;
    n.retard_deg = retard;
    n.active = true;
    n.oxidizer = &tables();
    return n;
}

Sensors running(real omega) {
    Sensors s;
    s.omega = omega;
    s.ignition = 1.0;
    return s;
}
ThrottleCommand wot() { return ThrottleCommand{1.0, 0.0, 1.0}; }

SimulatedEngineDesc turbo_engine(bool knock_map) {
    SimulatedEngineDesc d = knock_map ? fx::with_reserved_maps(fx::electronic_engine()) : fx::electronic_engine();
    d.turbo_pair = tb::hyper_like_pair();
    return d;
}

// Kit flows of the shot the tests use (a 100 hp-class shot [AS]: the N1 fixture kit's 35.6 g/s N2O, N2O:fuel ~3.8).
constexpr real kMdotN = 0.0356;
constexpr real kMdotKf = 0.0094;

struct Frozen {
    std::unique_ptr<SimulatedEngine> e;
    PreSolveResult pre;
    PostSolveResult post;
    real p = 0, t_charge = 0;
};

// 1440 substeps at a held speed with the shot, then ONE more recorded substep.
Frozen frozen_state(const SimulatedEngineDesc& d, real rpm, real mdot_n, real mdot_kf, real retard, bool cut_last = false) {
    Frozen f;
    f.e = std::make_unique<SimulatedEngine>(d);
    const real w = w_of(rpm);
    Sensors s = running(w);
    s.nitrous = nitrous_sensor(mdot_n, mdot_kf, kHinSat950, retard);
    for (int i = 0; i < 1440; ++i) {
        (void)f.e->pre_solve(s, wot(), TcuRequest{}, kH);
        (void)f.e->post_solve(w, kH);
    }
    TcuRequest tcu;
    tcu.ignition_cut = cut_last;
    f.pre = f.e->pre_solve(s, wot(), tcu, kH);
    f.post = f.e->post_solve(w, kH);
    f.p = f.e->p_im();
    f.t_charge = f.e->turbo()->charge_k;
    return f;
}

} // namespace

// ---------------------------------------------------------------------------
// T1: the exhaust balance at a frozen turbo state (5000 rpm, WOT, a 35.6 g/s shot, 2 deg
// retard) against the test's own evaluation of section 1.2 (bisection charge solve, fixture
// tables read through the desc) and the hand-written split. Literal anchors: the
// decomposition power is mdot_n * 82.05 kJ/mol / 44.0128 g/mol; the exhaust flow is
// air + ECU fuel + kit fuel + N2O; the compressor flow stays the throttle-path air (at
// steady state a*rev, NOT plus the N2O and the kit fuel, which enter after the intercooler).
// Sabotages (progress file): n2o_w left out of released; eta_c from the ECU lambda over the
// booked fuel; kit fuel through a cut cylinder counted as released.
// ---------------------------------------------------------------------------
TEST_CASE("N3 T1: turbo exhaust balance with N2O at a frozen state", "[unit][n2o][turbo][drivetrain]") {
    const SimulatedEngineDesc td = turbo_engine(false);
    const real rpm = 5000.0, retard = 2.0;
    const real w = w_of(rpm);
    const real rev = w / (2.0 * kPi);

    SECTION("fired charge") {
        const Frozen f = frozen_state(td, rpm, kMdotN, kMdotKf, retard);
        const ExhaustSplit& x = f.e->exhaust_split();
        const RefCharge c = ref_charge(td, rpm, f.p, f.t_charge, kMdotN, kMdotKf, kHinSat950);
        REQUIRE(f.pre.state == EngineState::Running);
        REQUIRE_FALSE(f.pre.limiter_active);
        REQUIRE_FALSE(f.pre.dfco_active);

        const real fuel_w = (c.f_ecu + c.kf) * rev * kLhvE5;
        const real n2o_w = kMdotN * kDhDecLiteral;
        const real eta_c = std::min(1.0, c.lambda_c);
        REQUIRE(c.lambda_c < 1.0); // rich: the chemical term is exercised
        const real released = eta_c * fuel_w + n2o_w;
        const real ind = w * ref_gs(td, rpm, f.p, c, retard) * ref_heat(td, c) / (2.0 * kPi);
        const real wall = kWallFraction * released;
        const real sens = released - ind - wall;
        const real mdot_exh = c.a * rev + (c.f_ecu + c.kf) * rev + kMdotN;
        const real port = f.t_charge + sens / (kCpExhaust * mdot_exh);

        // the pre-N3 rule (ECU lambda over the booked fuel, no decomposition) at the same state, for the comparison
        const real eta_old = std::min(1.0, c.lambda_t);
        const real sens_old = eta_old * fuel_w - ind - kWallFraction * eta_old * fuel_w;
        const real port_old = f.t_charge + sens_old / (kCpExhaust * mdot_exh);

        std::printf("[N3 T1] p_im %.0f Pa T_charge %.2f K: fuel %.1f kW (kit %.1f), N2O decomposition %.2f kW, lambda_c %.4f (ECU %.4f), "
                    "released %.1f kW, indicated %.1f kW, wall %.1f kW, sensible %.1f kW, chemical %.1f kW; exhaust %.4f kg/s, port %.1f K "
                    "(pre-N3 rule %.1f K, +%.1f K), turbine inlet %.1f K\n",
                    f.p, f.t_charge, x.fuel_w * 1e-3, c.kf * rev * kLhvE5 * 1e-3, x.n2o_w * 1e-3, c.lambda_c, c.lambda_t, released * 1e-3,
                    x.indicated_w * 1e-3, x.wall_w * 1e-3, x.sensible_w * 1e-3, x.chemical_w * 1e-3, f.pre.exhaust_mass_flow_kg_s,
                    f.pre.exhaust_port_k, port_old, f.pre.exhaust_port_k - port_old, f.pre.turbine_inlet_k);

        CHECK(rel(x.fuel_w, fuel_w) < 1e-9);
        CHECK(rel(x.n2o_w, n2o_w) < 1e-9);
        CHECK(rel(x.n2o_w, kMdotN * ox_dh()) < 1e-12);
        CHECK(rel(x.chemical_w, (1.0 - eta_c) * fuel_w) < 1e-9);
        CHECK(rel(x.indicated_w, ind) < 1e-9);
        CHECK(rel(x.wall_w, wall) < 1e-9);
        CHECK(rel(x.sensible_w, sens) < 1e-9);
        CHECK(x.clamp_w == 0.0);
        CHECK(x.manifold_w == 0.0);
        // closure, and the engine's own ledger terms
        CHECK(std::abs((x.fuel_w + x.n2o_w) - (x.indicated_w + x.wall_w + x.sensible_w + x.chemical_w - x.clamp_w)) < 1e-9 * (x.fuel_w + x.n2o_w));
        CHECK(rel(x.fuel_w * kH, f.post.fuel_energy_in_delta) < 1e-12);
        CHECK(rel(x.n2o_w * kH, f.post.n2o_energy_in_delta) < 1e-12);
        CHECK(rel(x.indicated_w * kH, f.post.indicated_work_delta) < 1e-12);
        // exhaust flow (item 3): air + ECU fuel + kit fuel + N2O
        CHECK(rel(f.pre.exhaust_mass_flow_kg_s, mdot_exh) < 1e-9);
        CHECK(rel(f.pre.exhaust_port_k, port) < 1e-9);
        // the N2O share of the heat is real: the pre-N3 rule would have missed it
        // (the pre-N3 rule is HOTTER here: the kit fuel enriches the combined mixture to lambda_c 0.74, so the fuel burns at
        // 0.74 not at the ECU's 0.91; the decomposition heat does not make up for it - reported, not asserted)
        // the decomposition heat itself, at the same state: without n2o_w in the balance the port gas is colder by
        // (1 - wall fraction) * n2o_w / (cp * mdot_exh) - what a build that drops n2o_w from released produces
        CHECK(std::abs((f.pre.exhaust_port_k - (port - (1.0 - kWallFraction) * n2o_w / (kCpExhaust * mdot_exh)))
                       - (1.0 - kWallFraction) * n2o_w / (kCpExhaust * mdot_exh)) < 1e-6);
        CHECK((1.0 - kWallFraction) * n2o_w / (kCpExhaust * mdot_exh) > 100.0);
        // compressor flow = throttle-path air, at steady state the cylinders' air a*rev (item 3, last bullet)
        CHECK(rel(f.pre.compressor_flow_kg_s, c.a * rev) < 5e-2);
        CHECK(f.pre.compressor_flow_kg_s < c.a * rev + 0.5 * (kMdotN + kMdotKf));
        // T_in of the cooling model is the turbo's charge temperature (N2, E1)
        CHECK(f.e->last_charge().t_in_k == f.t_charge);
        CHECK(f.t_charge != 288.15);
    }

    SECTION("ignition cut: the kit fuel passes unburnt, no heat") {
        const Frozen f = frozen_state(td, rpm, kMdotN, kMdotKf, retard, true);
        const ExhaustSplit& x = f.e->exhaust_split();
        const real kit_fuel_w = kMdotKf * kLhvE5; // kf per rev * rev rate = the kit's mass flow
        std::printf("[N3 T1 cut] fuel %.3f kW (booked kit fuel), n2o_w %.3f, chemical %.3f, sensible %.3f, port %.2f K (charge %.2f K), "
                    "exhaust %.4f kg/s\n",
                    x.fuel_w * 1e-3, x.n2o_w * 1e-3, x.chemical_w * 1e-3, x.sensible_w * 1e-3, f.pre.exhaust_port_k, f.t_charge,
                    f.pre.exhaust_mass_flow_kg_s);
        CHECK(f.post.fuel_mass_delta == f.post.kit_fuel_mass_delta);
        CHECK(rel(x.fuel_w, kit_fuel_w) < 1e-9);
        CHECK(x.n2o_w == 0.0);
        CHECK(x.indicated_w == 0.0);
        CHECK(x.wall_w == 0.0);
        CHECK(x.sensible_w == 0.0);
        CHECK(x.chemical_w == x.fuel_w);
        CHECK(f.pre.exhaust_port_k == f.t_charge);
        // the N2O and the kit fuel still leave through the exhaust
        CHECK(f.pre.exhaust_mass_flow_kg_s > kMdotN + kMdotKf);
    }
}

// ---------------------------------------------------------------------------
// T2: the kit retard's lost indicated work equals the D5 P2 rule at the same charge
// (eta_spark(schedule) - eta_spark(schedule - retard)) * eta_i * heat * omega / 2pi, the
// schedule being the KNOCK-LIMITED one (a kit retard below the knock limit costs nothing).
// Two engines with the same history differ only in the last substep's retard: their
// sensible exhaust power differs by exactly that work, the port temperature follows, and
// over seconds the held exhaust wall ends up hotter. Sabotages: the unretarded reference
// at the raw schedule (ignoring the knock limit); no n2o_retard_w; indicated booked at the
// unretarded work (the heat vanishes).
// ---------------------------------------------------------------------------
TEST_CASE("N3 T2: the kit retard's lost work lands in the exhaust heat and the held wall", "[unit][n2o][turbo][drivetrain]") {
    struct Case {
        const char* name;
        bool knock_map;
        real rpm, retard;
        bool expect_loss;
    };
    const Case cases[] = {
        {"no knock map, retard 4 at 5000 rpm", false, 5000.0, 4.0, true},
        {"knock map: limit 9.95 deg under schedule 11.25, retard 4 -> 7.25 deg", true, 5000.0, 4.0, true},
        {"knock map: limit binds either way, retard 1 -> costs nothing", true, 5000.0, 1.0, false},
    };
    for (const Case& k : cases) {
        INFO(k.name);
        const SimulatedEngineDesc td = turbo_engine(k.knock_map);
        const real w = w_of(k.rpm);
        SimulatedEngine a(td), b(td);
        Sensors s0 = running(w), s1 = running(w);
        s0.nitrous = nitrous_sensor(kMdotN, kMdotKf, kHinSat950, 0.0);
        s1.nitrous = nitrous_sensor(kMdotN, kMdotKf, kHinSat950, 0.0);
        for (int i = 0; i < 1440; ++i) {
            (void)a.pre_solve(s0, wot(), TcuRequest{}, kH);
            (void)a.post_solve(w, kH);
            (void)b.pre_solve(s1, wot(), TcuRequest{}, kH);
            (void)b.post_solve(w, kH);
        }
        REQUIRE(a.fold_state_hash(0) == b.fold_state_hash(0));
        s1.nitrous = nitrous_sensor(kMdotN, kMdotKf, kHinSat950, k.retard);
        const PreSolveResult ra = a.pre_solve(s0, wot(), TcuRequest{}, kH);
        const PreSolveResult rb = b.pre_solve(s1, wot(), TcuRequest{}, kH);
        const ExhaustSplit xa = a.exhaust_split();
        const ExhaustSplit xb = b.exhaust_split();
        REQUIRE(a.p_im() == b.p_im()); // the retard does not touch the manifold
        const real p = a.p_im(), t_charge = a.turbo()->charge_k;
        const RefCharge c = ref_charge(td, k.rpm, p, t_charge, kMdotN, kMdotKf, kHinSat950);
        // the test's own D5 P2 rule
        const real lost_ref = w * (ref_gs(td, k.rpm, p, c, 0.0) - ref_gs(td, k.rpm, p, c, k.retard)) * ref_heat(td, c) / (2.0 * kPi);
        const real lost_sens = xb.sensible_w - xa.sensible_w;
        const real lost_ind = xa.indicated_w - xb.indicated_w;
        const real dport = rb.exhaust_port_k - ra.exhaust_port_k;
        std::printf("[N3 T2] %s: lost work %.3f kW (reference %.3f), sensible +%.3f kW, port +%.3f K (expected %.3f K), n2o_retard_w %.3f kW\n",
                    k.name, lost_ind * 1e-3, lost_ref * 1e-3, lost_sens * 1e-3, dport,
                    lost_ref / (kCpExhaust * ra.exhaust_mass_flow_kg_s), xb.n2o_retard_w * 1e-3);
        CHECK(xa.n2o_retard_w == 0.0);
        if (k.expect_loss) {
            CHECK(lost_ref > 1000.0);
            CHECK(rel(xb.n2o_retard_w, lost_ref) < 1e-9);
            CHECK(rel(lost_ind, lost_ref) < 1e-9);
            CHECK(rel(lost_sens, lost_ref) < 1e-9);
            CHECK(rel(dport, lost_ref / (kCpExhaust * ra.exhaust_mass_flow_kg_s)) < 1e-6);
        } else {
            CHECK(lost_ref == 0.0);
            CHECK(xb.n2o_retard_w == 0.0);
            CHECK(lost_sens == 0.0);
            CHECK(dport == 0.0);
        }
        if (k.expect_loss && !k.knock_map) {
            // held state: two seconds on, the retarded engine's exhaust wall (and the turbine inlet) are hotter
            for (int i = 0; i < 2 * 960; ++i) {
                (void)a.pre_solve(s0, wot(), TcuRequest{}, kH);
                (void)a.post_solve(w, kH);
                (void)b.pre_solve(s1, wot(), TcuRequest{}, kH);
                (void)b.post_solve(w, kH);
            }
            std::printf("[N3 T2 held] after 2 s: wall %.2f K vs %.2f K (+%.2f K), turbine inlet %.2f K vs %.2f K\n", b.turbo()->wall_k,
                        a.turbo()->wall_k, b.turbo()->wall_k - a.turbo()->wall_k, b.turbo()->exhaust_k, a.turbo()->exhaust_k);
            CHECK(b.turbo()->wall_k > a.turbo()->wall_k + 0.5);
            CHECK(b.turbo()->exhaust_k > a.turbo()->exhaust_k + 10.0);
        }
    }
}

// ---------------------------------------------------------------------------
// T3: spool comparison.
// ---------------------------------------------------------------------------
namespace {

struct Snap {
    real rotor_rpm = 0, boost_bar = 0, wastegate = 0, engine_rpm = 0, p_exh = 0, port_k = 0, turbine_k = 0;
};

struct SpoolRun {
    Snap at_025, at_05, at_10, at_20;
    real t05 = -1.0, t10 = -1.0, t15 = -1.0, peak_boost = 0.0;
    int spray_ticks = 0;
    real n2o_g_s = 0.0, kit_g_s = 0.0;
    real worst_closure = 0.0, worst_bottle = 0.0;
    std::uint64_t hash = 0;
    std::uint64_t alloc_own = 0;
};

Snap snap_of(const PowertrainSnapshot& st) {
    Snap s;
    const EngineSoundState& e = st.engines.at(0);
    s.rotor_rpm = e.turbo_rpm;
    s.boost_bar = e.boost_bar;
    s.wastegate = e.wastegate_opening;
    s.engine_rpm = st.engine_rpm;
    s.p_exh = e.p_exhaust_pa;
    s.port_k = e.exhaust_port_k;
    s.turbine_k = e.turbine_inlet_k;
    return s;
}

const vehicle::VehicleDesc& hyper_fixture() {
    static const vehicle::VehicleDesc d = [] {
        jobs::JobSystem pool(4);
        io::EngineMapOptions opt;
        opt.pool = &pool;
        opt.cache_dir = (std::filesystem::temp_directory_path() / "ps_i7d_engine_cache").string();
        // measurement hook: PS_N3_ENGINE_CACHE = an empty dir gives a cold generation
#ifdef _WIN32
        char* alt = nullptr;
        std::size_t alt_len = 0;
        if (_dupenv_s(&alt, &alt_len, "PS_N3_ENGINE_CACHE") == 0 && alt != nullptr) {
            opt.cache_dir = alt;
            std::free(alt);
        }
#else
        if (const char* alt = std::getenv("PS_N3_ENGINE_CACHE")) opt.cache_dir = alt;
#endif
        return io::load_vehicle_json(fixture_path("n2o/vehicle_hyper_n2o.json"), opt);
    }();
    return d;
}

SimulatedEngineDesc hyper_engine_desc() {
    for (const auto& c : hyper_fixture().powertrain.components)
        if (const auto* e = std::get_if<SimulatedEngineDesc>(&c.params)) return *e;
    FAIL("no simulated engine in the hyper fixture");
    return {};
}

// car_hyper (+ kit) in 3rd gear, rolling at the speed of 3000 rpm, light throttle 1 s, then WOT for up to `seconds`.
SpoolRun run_hyper(bool arm, int workers, real seconds, bool check_alloc = false) {
    const vehicle::VehicleDesc& desc = hyper_fixture();
    auto rig = ps_test::build_vehicle_rig_from_desc(desc, 1500, {2, .4, .15}, .5, 0.0, workers, Vec3::unit_y(), 240.0, 960.0, 3000.0);
    World& w = *rig.world;
    ps_test::settle(w, .5);
    const real v3000 = w_of(3000.0) / (1.65 * 3.35) * 0.364; // WG diag's own speed for 3000 rpm in 3rd
    auto motion = w.get_motion(rig.chassis);
    motion.linear = {v3000, 0, 0};
    w.backend().set_motion(rig.chassis, motion);
    vehicle::VehicleResetOptions reset;
    reset.gear = 3;
    reset.clutch_locked = true;
    w.reset_vehicle(rig.vehicle, reset);
    w.set_control("gear_request", 3);
    w.set_control("throttle", 0.1);
    ps_test::settle(w, 1.0);
    const real bottle0 = w.n2o_bottle_mass(rig.vehicle);

    SpoolRun r;
    namespace da = ps::jobs::debug_alloc;
    const bool arm_alloc = check_alloc && da::hook_available();
    if (arm_alloc) {
        da::reset();
        da::set_armed(true);
    }
    w.set_control("nitrous_arm", arm ? 1.0 : 0.0);
    w.set_control("throttle", 1.0);
    const int ticks = static_cast<int>(seconds * 240.0);
    for (int tick = 1; tick <= ticks; ++tick) {
        w.step();
        const PowertrainSnapshot& st = w.powertrain_state(rig.vehicle);
        if (st.engines.empty()) break;
        const EngineSoundState& e = st.engines[0];
        r.peak_boost = std::max(r.peak_boost, e.boost_bar);
        if (r.t05 < 0 && e.boost_bar >= 0.5) r.t05 = tick / 240.0;
        if (r.t10 < 0 && e.boost_bar >= 1.0) r.t10 = tick / 240.0;
        if (r.t15 < 0 && e.boost_bar >= 1.5) r.t15 = tick / 240.0;
        if (e.n2o_active) {
            ++r.spray_ticks;
            r.n2o_g_s = e.n2o_flow_g_s;
            r.kit_g_s = std::max(r.kit_g_s, e.n2o_kit_fuel_g_s);
        }
        if (tick == 60) r.at_025 = snap_of(st);
        if (tick == 120) r.at_05 = snap_of(st);
        if (tick == 240) r.at_10 = snap_of(st);
        if (tick == 480) r.at_20 = snap_of(st);
        const PowertrainEnergy& en = w.powertrain_energy(rig.vehicle);
        const real in = en.fuel_energy_in + en.n2o_energy_in;
        if (in > 0.0) r.worst_closure = std::max(r.worst_closure, std::abs(in - en.indicated_work - en.engine_heat) / in);
        r.worst_bottle = std::max(r.worst_bottle, std::abs((bottle0 - w.n2o_bottle_mass(rig.vehicle)) - (en.n2o_mass + en.n2o_purged - en.n2o_external_added)));
        if (st.engine_rpm > 8100.0) break;
    }
    if (arm_alloc) {
        da::set_armed(false);
        r.alloc_own = da::own_violation_count();
        WARN("N3 T6 debug_alloc (" << workers << " workers): own " << r.alloc_own << ", external (Jolt) " << da::external_violation_count());
    }
    r.hash = w.state_hash();
    return r;
}

struct Dyno {
    Snap at_05, at_10, at_30, steady; // steady = mean over the last second
    real tau0 = 0.0, exhaust_kg_s = 0.0;
};

// An engine held at `rpm`, WOT from its idle state, for `seconds`; the shot (if any) from t = 0.
Dyno run_dyno(const SimulatedEngineDesc& d, real rpm, bool spray, real mdot_n, real mdot_kf, real retard, real seconds, real pre_s = 0.0) {
    SimulatedEngine e(d);
    const real w = w_of(rpm);
    Sensors s = running(w);
    for (int i = 0; i < static_cast<int>(pre_s * 960.0); ++i) { // optional spool-up without the shot
        (void)e.pre_solve(s, wot(), TcuRequest{}, kH);
        (void)e.post_solve(w, kH);
    }
    if (spray) s.nitrous = nitrous_sensor(mdot_n, mdot_kf, kHinSat950, retard);
    Dyno r;
    const int n = static_cast<int>(seconds * 960.0);
    Snap acc;
    int cnt = 0;
    for (int i = 1; i <= n; ++i) {
        const PreSolveResult p = e.pre_solve(s, wot(), TcuRequest{}, kH);
        (void)e.post_solve(w, kH);
        Snap sn;
        sn.rotor_rpm = p.turbo_rpm;
        sn.boost_bar = p.boost_bar;
        sn.wastegate = p.wastegate_opening;
        sn.engine_rpm = p.rpm;
        sn.p_exh = p.p_exhaust_pa;
        sn.port_k = p.exhaust_port_k;
        sn.turbine_k = p.turbine_inlet_k;
        if (i == 480) r.at_05 = sn;
        if (i == 960) r.at_10 = sn;
        if (i == 2880) r.at_30 = sn;
        if (i > n - 960) {
            acc.rotor_rpm += sn.rotor_rpm; acc.boost_bar += sn.boost_bar; acc.wastegate += sn.wastegate;
            acc.p_exh += sn.p_exh; acc.port_k += sn.port_k; acc.turbine_k += sn.turbine_k; acc.engine_rpm += sn.engine_rpm;
            r.tau0 += p.tau0; r.exhaust_kg_s += p.exhaust_mass_flow_kg_s;
            ++cnt;
        }
    }
    r.steady = Snap{acc.rotor_rpm / cnt, acc.boost_bar / cnt, acc.wastegate / cnt, acc.engine_rpm / cnt, acc.p_exh / cnt, acc.port_k / cnt, acc.turbine_k / cnt};
    r.tau0 /= cnt;
    r.exhaust_kg_s /= cnt;
    return r;
}

void print_snap(const char* tag, const Snap& s) {
    std::printf("%s rotor %.0f rpm, boost %+.4f bar, wastegate %.4f, p_exh %.0f Pa, port %.1f K, turbine inlet %.1f K\n", tag, s.rotor_rpm,
                s.boost_bar, s.wastegate, s.p_exh, s.port_k, s.turbine_k);
}

} // namespace

// Short, non-long variant on the fixture turbo engine: 3000 rpm held, rotor spooled 1.5 s without the shot,
// then the shot: rotor and exhaust heat vs. the same engine without it.
TEST_CASE("N3 T3s: spool with and without N2O on the fixture turbo engine (short)", "[unit][n2o][turbo][drivetrain]") {
    const SimulatedEngineDesc td = turbo_engine(false);
    const Dyno off = run_dyno(td, 3000.0, false, 0, 0, 0, 3.0, 1.5);
    const Dyno on = run_dyno(td, 3000.0, true, kMdotN, kMdotKf, 4.0, 3.0, 1.5);
    print_snap("[N3 T3s] no N2O +0.5 s:", off.at_05);
    print_snap("[N3 T3s] N2O    +0.5 s:", on.at_05);
    print_snap("[N3 T3s] no N2O steady:", off.steady);
    print_snap("[N3 T3s] N2O    steady:", on.steady);
    CHECK(on.at_05.rotor_rpm > off.at_05.rotor_rpm);
    CHECK(on.at_05.port_k > off.at_05.port_k);
    CHECK(on.at_10.rotor_rpm > off.at_10.rotor_rpm);
}

TEST_CASE("N3 T3: hyper V8 spool with and without N2O (3rd-gear WOT from 3000 rpm, dyno-held steady state)",
          "[unit][n2o][turbo][drivetrain][cycle][long]") {
    const SpoolRun off = run_hyper(false, 1, 3.0);
    const SpoolRun on = run_hyper(true, 1, 3.0);
    print_snap("[N3 T3 vehicle] no N2O +0.25 s:", off.at_025);
    print_snap("[N3 T3 vehicle] N2O    +0.25 s:", on.at_025);
    print_snap("[N3 T3 vehicle] no N2O +0.5 s:", off.at_05);
    print_snap("[N3 T3 vehicle] N2O    +0.5 s:", on.at_05);
    print_snap("[N3 T3 vehicle] no N2O +1.0 s:", off.at_10);
    print_snap("[N3 T3 vehicle] N2O    +1.0 s:", on.at_10);
    std::printf("[N3 T3 vehicle] time to 0.5/1.0/1.5 bar (-1 = not reached in 3 s): no N2O %.3f/%.3f/%.3f s, N2O %.3f/%.3f/%.3f s; peak %.3f vs %.3f bar; "
                "spray ticks %d (%.2f g/s N2O, up to %.2f g/s kit fuel), engine rpm at +0.5 s %.0f vs %.0f\n",
                off.t05, off.t10, off.t15, on.t05, on.t10, on.t15, off.peak_boost, on.peak_boost, on.spray_ticks, on.n2o_g_s, on.kit_g_s,
                off.at_05.engine_rpm, on.at_05.engine_rpm);
    REQUIRE(on.spray_ticks > 100);
    CHECK(off.spray_ticks == 0);
    CHECK(on.at_05.rotor_rpm > off.at_05.rotor_rpm);
    CHECK(on.at_05.rotor_rpm > 1.5 * off.at_05.rotor_rpm);
    CHECK(on.t10 > 0);
    CHECK((off.t10 < 0 || on.t10 < off.t10));

    // Dyno-held steady state at 5000 rpm (inside the controller's 1500-6000 rpm window), the shot's flows as the vehicle run
    // delivered them.
    const SimulatedEngineDesc hd = hyper_engine_desc();
    const real mdot_n = on.n2o_g_s * 1e-3, mdot_kf = on.kit_g_s * 1e-3;
    const Dyno d_off = run_dyno(hd, 5000.0, false, 0, 0, 0, 10.0);
    const Dyno d_on = run_dyno(hd, 5000.0, true, mdot_n, mdot_kf, 4.0, 10.0);
    print_snap("[N3 T3 dyno 5000 rpm] no N2O +0.5 s:", d_off.at_05);
    print_snap("[N3 T3 dyno 5000 rpm] N2O    +0.5 s:", d_on.at_05);
    print_snap("[N3 T3 dyno 5000 rpm] no N2O steady:", d_off.steady);
    print_snap("[N3 T3 dyno 5000 rpm] N2O    steady:", d_on.steady);
    std::printf("[N3 T3 dyno] tau0 %.1f vs %.1f N*m, exhaust %.4f vs %.4f kg/s\n", d_off.tau0, d_on.tau0, d_off.exhaust_kg_s, d_on.exhaust_kg_s);
    CHECK(d_on.at_05.rotor_rpm > d_off.at_05.rotor_rpm);
    // boost on target with the wastegate further open (both settle at 1.760 bar in 10 s, 0.04 below the 1.8 bar target - the
    // existing PID / intercooler offset, not N3's; the N2O must not move it, only the wastegate and the rotor's way there)
    CHECK(std::abs(d_on.steady.boost_bar - 1.8) < 0.06);
    CHECK(std::abs(d_off.steady.boost_bar - 1.8) < 0.06);
    CHECK(std::abs(d_on.steady.boost_bar - d_off.steady.boost_bar) < 0.005);
    CHECK(rel(d_on.steady.rotor_rpm, d_off.steady.rotor_rpm) < 0.01);
    CHECK(d_on.steady.wastegate > d_off.steady.wastegate + 0.05);
}

// ---------------------------------------------------------------------------
// T4: D5 guard. An active kit block with ZERO N2O flow (kit fuel, retard and tables all set)
// leaves a turbo + anti-lag engine bit-identical on a script that spools, lifts into anti-lag
// and re-applies: every substep's PreSolveResult/PostSolveResult, the exhaust split and the
// state hash. The existing turbo/anti-lag pins (test_turbo_*.cpp [turbo]) run unchanged in
// the same binary. Sabotage: the N3 branch gated on `active` alone (not mdot_n > 0).
// ---------------------------------------------------------------------------
TEST_CASE("N3 T4: a zero-flow kit block leaves the turbo + anti-lag engine bit-identical", "[unit][n2o][turbo][drivetrain]") {
    SimulatedEngineDesc d = turbo_engine(true);
    d.turbo_pair->anti_lag = tb::hyper_like_anti_lag();
    SimulatedEngine a(d), b(d);
    const real w = w_of(5000.0);
    int als = 0, dfco = 0;
    for (int i = 0; i < 4 * 960; ++i) {
        Sensors sa = running(w);
        sa.anti_lag_channel = 1.0;
        Sensors sb = sa;
        sb.nitrous = nitrous_sensor(0.0, 0.004, kHinSat950, 4.0);
        const real cmd = (i < 1500 || i >= 2500) ? 1.0 : 0.0;
        const PreSolveResult ra = a.pre_solve(sa, ThrottleCommand{cmd, 0.0, 1.0}, TcuRequest{}, kH);
        const PreSolveResult rb = b.pre_solve(sb, ThrottleCommand{cmd, 0.0, 1.0}, TcuRequest{}, kH);
        const PostSolveResult pa = a.post_solve(w, kH);
        const PostSolveResult pb = b.post_solve(w, kH);
        REQUIRE(ra.tau0 == rb.tau0);
        REQUIRE(ra.d == rb.d);
        REQUIRE(ra.exhaust_mass_flow_kg_s == rb.exhaust_mass_flow_kg_s);
        REQUIRE(ra.exhaust_port_k == rb.exhaust_port_k);
        REQUIRE(ra.turbine_inlet_k == rb.turbine_inlet_k);
        REQUIRE(ra.exhaust_wall_k == rb.exhaust_wall_k);
        REQUIRE(ra.turbo_rpm == rb.turbo_rpm);
        REQUIRE(ra.anti_lag_extra_exhaust_w == rb.anti_lag_extra_exhaust_w);
        REQUIRE(ra.torque_demand == rb.torque_demand);
        REQUIRE(pa.released_heat_delta == pb.released_heat_delta);
        REQUIRE(pa.indicated_work_delta == pb.indicated_work_delta);
        const ExhaustSplit& xa = a.exhaust_split();
        const ExhaustSplit& xb = b.exhaust_split();
        REQUIRE(xa.sensible_w == xb.sensible_w);
        REQUIRE(xa.chemical_w == xb.chemical_w);
        REQUIRE(xa.manifold_w == xb.manifold_w);
        REQUIRE(xb.n2o_w == 0.0);
        REQUIRE(xb.n2o_retard_w == 0.0);
        REQUIRE(a.fold_state_hash(0) == b.fold_state_hash(0));
        if (ra.anti_lag_active) ++als;
        if (ra.dfco_active) ++dfco;
    }
    std::printf("[N3 T4] 3840 substeps identical (anti-lag active %d, DFCO %d)\n", als, dfco);
    CHECK(als > 500);
}

// ---------------------------------------------------------------------------
// T5: per-substep closure on the turbo engine with N2O, including ignition-cut substeps
// (kit fuel unburnt, N2O unburnt) and the fuel-cut limiter: the engine's ledger terms
// against the exhaust balance, indicated work against eta_i*eta_spark*heat, and the
// balance closing to round-off. Sabotage: released taken on the booked fuel (the kit fuel
// of a cut substep counted as released) fails the cut substeps' closure.
// ---------------------------------------------------------------------------
TEST_CASE("N3 T5: exhaust balance and ledger terms close every substep on the turbo engine with N2O", "[unit][n2o][turbo][drivetrain]") {
    const SimulatedEngineDesc td = turbo_engine(true);
    SimulatedEngine e(td);
    real worst_close = 0.0, worst_fuel = 0.0, worst_n2o = 0.0, worst_ind = 0.0, worst_chem_cut = 0.0;
    int fired = 0, cut = 0, limiter = 0;
    for (real rpm : {3500.0, 5500.0, 6900.0}) {
        const real w = w_of(rpm);
        Sensors s = running(w);
        s.nitrous = nitrous_sensor(kMdotN, kMdotKf, kHinSat950, 3.0);
        for (int i = 0; i < 1200; ++i) {
            TcuRequest tcu;
            tcu.ignition_cut = (i % 7) == 3; // an upshift-cut-like pattern: ~1 substep in 7
            const PreSolveResult pre = e.pre_solve(s, wot(), tcu, kH);
            const PostSolveResult po = e.post_solve(w, kH);
            const ExhaustSplit& x = e.exhaust_split();
            const real in = x.fuel_w + x.n2o_w;
            worst_close = std::max(worst_close, std::abs(in - (x.indicated_w + x.wall_w + x.sensible_w + x.chemical_w - x.clamp_w)) / std::max(1.0, in));
            worst_fuel = std::max(worst_fuel, std::abs(x.fuel_w * kH - po.fuel_energy_in_delta) / std::max(1e-9, po.fuel_energy_in_delta));
            worst_n2o = std::max(worst_n2o, std::abs(x.n2o_w * kH - po.n2o_energy_in_delta) / std::max(1e-9, std::abs(po.n2o_energy_in_delta)));
            worst_ind = std::max(worst_ind, std::abs(x.indicated_w * kH - po.indicated_work_delta) / std::max(1e-9, std::abs(po.indicated_work_delta)));
            CHECK(x.clamp_w == 0.0);
            if (pre.limiter_active || tcu.ignition_cut) {
                // a cut substep: nothing burns, nothing is released; the kit fuel that was booked passes as chemical enthalpy
                ++cut;
                if (pre.limiter_active) ++limiter;
                worst_chem_cut = std::max(worst_chem_cut, std::abs(x.chemical_w - x.fuel_w) / std::max(1.0, x.fuel_w));
                CHECK(x.n2o_w == 0.0);
                CHECK(x.sensible_w == 0.0);
                CHECK(po.n2o_unburnt_delta == po.n2o_mass_delta);
            } else {
                ++fired;
                CHECK(x.n2o_w > 0.0);
                CHECK(po.n2o_unburnt_delta == 0.0);
            }
            // heat released in the engine's thermal booking (post_solve) equals the exhaust balance's released share
            const real released = x.wall_w + x.sensible_w + x.indicated_w;
            if (!tcu.ignition_cut && !pre.limiter_active)
                CHECK(std::abs(released * kH - po.released_heat_delta) <= 1e-9 * std::max(1e-9, po.released_heat_delta));
        }
    }
    std::printf("[N3 T5] fired %d, cut %d (limiter %d): closure worst %.2e, fuel_energy_in %.2e, n2o_energy_in %.2e, indicated %.2e, chemical==fuel in cuts %.2e\n",
                fired, cut, limiter, worst_close, worst_fuel, worst_n2o, worst_ind, worst_chem_cut);
    CHECK(worst_close < 1e-12);
    CHECK(worst_fuel < 1e-12);
    CHECK(worst_n2o < 1e-12);
    CHECK(worst_ind < 1e-12);
    CHECK(worst_chem_cut < 1e-12);
    CHECK(fired > 1000);
    CHECK(cut > 500);
}

// ---------------------------------------------------------------------------
// T6: the whole hyper vehicle with an active kit: the per-tick ledger closure
// fuel_energy_in + n2o_energy_in - indicated_work - engine_heat, the bottle closure, 1 vs 4
// workers (the owner is driving on this PC: 4, not 24, hence not the 10 of E8), zero own
// allocation. Fixture only: car_hyper_n2o.json is N7.
// ---------------------------------------------------------------------------
TEST_CASE("N3 T6: hyper vehicle with an active kit - ledger and bottle closure, 1 vs 4 workers, allocation-free",
          "[unit][n2o][turbo][drivetrain][cycle][jobs][determinism][long]") {
    const SpoolRun r1 = run_hyper(true, 1, 1.5, true);
    const SpoolRun r4 = run_hyper(true, 4, 1.5, true);
    std::printf("[N3 T6] closure worst rel %.2e, bottle %.2e kg, hash 1 worker 0x%016llx, 4 workers 0x%016llx\n", r1.worst_closure, r1.worst_bottle,
                static_cast<unsigned long long>(r1.hash), static_cast<unsigned long long>(r4.hash));
    CHECK(r1.spray_ticks > 100);
    CHECK(r1.worst_closure < 1e-12);
    CHECK(r1.worst_bottle < 1e-12 * 4.536);
    CHECK(r1.hash == r4.hash);
    CHECK(r1.alloc_own == 0);
    CHECK(r4.alloc_own == 0);
}
