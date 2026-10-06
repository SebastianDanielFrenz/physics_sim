// tests/unit/test_n2o_engine.cpp -- N2O plan N2 (out/n2o_plan.md section 1.2, 1.3, 4,
// 9.1): the simulated engine's wet charge model (core/src/drivetrain/
// engine_simulated.cpp - solve_n2o_air, the N2O branch of evaluate_torque_impl and
// manifold_rhs_impl, pre_solve/post_solve bookings) and the Powertrain's N2O
// sub-ledger booking.
//
// Reference values are the test's OWN evaluation of section 1.2 from the loaded
// public descs: the charge temperature by bisection on T of the two port balances
// (moles a + r*n = X/T, energy a*cp_a*(T_in - T) = n*(h_g(T) - h_in), h_g linear
// between the oxidizer file's 1 bar rows), never the engine's closed-form quadratic;
// the table reads go through the fixture desc's own Table2D/Curve1D. Typed-in
// constants: R 287.05 J/(kg K) [C ISO 2533, isa_air.h], gamma 1.4, M_air 28.96442
// g/mol [C ISO 2533:1975], O2 in air 0.2314 [D ISO 2533 composition]; the N2O side
// (molar mass, O2 fraction, formation enthalpy) is read from the loaded oxidizer desc.
//
//   E1 white-box charge and combustion torque at three frozen states (NA WOT 4000 and
//      6000 rpm on the fixture engine with the knock-limit map, a turbo engine at 5000
//      rpm with its own charge temperature)
//   E2 dtau/domega against a central difference (every N2O term nonzero)
//   E3 manifold dfdp against a central difference; m_cyl equals the torque path's air
//   E4 per-tick ledger closure fuel_energy_in + n2o_energy_in - indicated_work -
//      engine_heat through World, and indicated == eta_i*eta_spark*(f_lambda*LHV*f +
//      dH*n) per substep (T5 pattern)
//   E5 no N2O flow (a disarmed kit, an active block with zero flow) is bit-identical
//   E6 a cut substep books the N2O as unburnt and releases no energy
//   E7 NACA MR E5F26 Table II (V1) on the generated A25A maps
//   E8 1 vs 10 workers with an active kit, allocation-free
#include "drivetrain/engine_simulated.h"
#include "drivetrain/nitrous_flow.h"

#include "mvem_fixtures.h"
#include "turbo_fixtures.h"
#include "vehicle_test_helpers.h"

#include "ps/drivetrain/nitrous_desc.h"
#include "ps/drivetrain/oxidizer_desc.h"
#include "ps/drivetrain/powertrain_desc.h"
#include "ps/io/fuel_io.h"
#include "ps/io/nitrous_io.h"
#include "ps/io/surface_table.h"
#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <variant>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;
namespace fx = ps::test::mvem;
namespace tb = ps::test::turbo;

namespace {

constexpr real kPi = 3.14159265358979323846;
constexpr real kH = 1.0 / 960.0;
constexpr real kR = 287.05;                       // [C] ISO 2533 / isa_air.h
constexpr real kCpAir = kR * 1.4 / 0.4;           // [D]
constexpr real kMAir = 0.02896442;                // [C] ISO 2533:1975, kg/mol
constexpr real kO2Air = 0.2314;                   // [D] ISO 2533 dry-air composition
constexpr real kHinSat950 = 256.52e3;             // [C] NIST saturated liquid h at 305.41 K / 66.247 bar abs (~950 psig), plan 1.1

real w_of(real rpm) { return rpm * 2.0 * kPi / 60.0; }
real rpm_of(real w) { return std::abs(w) * 60.0 / (2.0 * kPi); }
real rel(real a, real b) { return std::abs(a - b) / std::max(std::abs(b), 1e-300); }

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
    real a = 0, t_mix = 0, n = 0, kf = 0, a_eq = 0, f_ecu = 0, f = 0, lambda_c = 0;
};

// Section 1.2 steps 1-4 by bisection on T (the test's own solve).
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
    real lo = 100.0, hi = t_in + 1.0; // the mixture is never warmer than the inlet air for h_in <= h_g(T_in)
    for (int i = 0; i < 300; ++i) {
        const real mid = 0.5 * (lo + hi);
        if (mid <= lo || mid >= hi) break;
        (resid(mid) > 0.0 ? lo : hi) = mid;
    }
    c.t_mix = 0.5 * (lo + hi);
    c.a = x / c.t_mix - r * c.n;
    c.a_eq = c.a + ox_ko2() * c.n;
    const real lam_t = d.ecu.lambda_target.value(rpm, p);
    const real afr = d.fuels[0].afr_stoich;
    c.f_ecu = c.a / (lam_t * afr);
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

// Combustion torque, fire 1, s 1: eta_i*eta_spark*(f_lambda(lambda_c)*LHV*f + dH*n)/(2 pi).
real ref_tau_comb(const SimulatedEngineDesc& d, real rpm, real p, const RefCharge& c, real retard) {
    return ref_gs(d, rpm, p, c, retard) * (d.maps.f_lambda.evaluate(c.lambda_c) * d.fuels[0].lhv_j_kg * c.f + ox_dh() * c.n) / (2.0 * kPi);
}

NitrousCharge charge_in(real mdot_n, real mdot_kf, real h_in, real retard) {
    return NitrousCharge{mdot_n, mdot_kf, h_in, retard, &tables()};
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

const std::shared_ptr<const SimulatedEngineMaps>& reserved_maps() {
    static const auto m = SimulatedEngine::build_maps(fx::with_reserved_maps(fx::electronic_engine()));
    return m;
}

// ---- whole-vehicle rig: the N1 fixture sedans with their graph engine swapped for the MVEM fixture ----

template <typename T>
T* find_component(vehicle::VehicleDesc& d) {
    for (auto& c : d.powertrain.components)
        if (auto* p = std::get_if<T>(&c.params)) return p;
    return nullptr;
}

vehicle::VehicleDesc sim_sedan(bool with_kit) {
    vehicle::VehicleDesc d = io::load_vehicle_json(fixture_path(with_kit ? "vehicle_n2o_sedan.json" : "vehicle_tank_sedan.json"));
    SimulatedEngineDesc e = fx::electronic_engine();
    e.fuels = {io::load_fuel_json(data_path("fuels/e5.json"))}; // the fixture tank's fuel
    for (auto& c : d.powertrain.components) {
        if (std::holds_alternative<TorqueMapEngineDesc>(c.params)) c.params = e;
    }
    return d;
}

struct Rig {
    std::unique_ptr<World> world;
    VehicleId vehicle;
    BodyId chassis;
};

Rig make_rig(const vehicle::VehicleDesc& desc, int workers = 1, real tick_hz = 240.0) {
    Rig rig;
    WorldConfig cfg;
    cfg.job_workers = workers;
    cfg.tick_rate_hz = tick_hz;
    cfg.substep_rate_hz = 960.0;
    rig.world = std::make_unique<World>(cfg);
    BodyDesc ground;
    ground.shape = BoxShape{Vec3{1000.0, 1000.0, 0.5}};
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    ground.pose.position = Vec3{0.0, 0.0, -0.5};
    rig.world->create_body(ground);
    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = Vec3{0.0, 0.0, 0.6};
    rig.chassis = rig.world->create_body(chassis);
    rig.world->set_surface_table(std::make_shared<io::SurfaceTable>(data_path("surfaces/surfaces.json")));
    rig.vehicle = rig.world->create_vehicle(desc, rig.chassis);
    rig.world->set_control("assist.auto_clutch", 1.0);
    rig.world->set_control("assist.auto_blip", 0.0);
    rig.world->set_control("assist.auto_shift", 0.0);
    rig.world->set_control("clutch", 0.0);
    rig.world->set_control("throttle", 0.0);
    vehicle::VehicleResetOptions o;
    o.gear = 1;
    o.engine_state = EngineState::Running;
    o.clutch_locked = false;
    rig.world->reset_vehicle(rig.vehicle, o);
    return rig;
}

real sum_abs_terms(const PowertrainEnergy& e) {
    return std::abs(e.engine_out) + std::abs(e.wheel_out) + std::abs(e.tyre_loss) + std::abs(e.brake_loss) + std::abs(e.gear_loss) +
           std::abs(e.integrator_loss) + std::abs(e.clutch_loss) + std::abs(e.synchro_loss) + std::abs(e.engine_friction_loss) +
           std::abs(e.engagement_loss) + std::abs(e.starter_in) + std::abs(e.external_reset) + std::abs(e.diff_loss);
}

} // namespace

// ---------------------------------------------------------------------------
// E1: the charge (a, T_mix, lambda_c) and the combustion torque at three frozen
// states against the test's own bisection + table reads. Sabotages (each recorded in
// out/n2o_n2_progress.md): no cooling, no displacement (r = 0), O2 factor 1.0, no
// decomposition, eta_i at a instead of a_eq, f_lambda on the decomposition share.
// ---------------------------------------------------------------------------
TEST_CASE("N2 E1: wet charge and combustion torque at frozen states match the test's own solve", "[unit][n2o][drivetrain]") {
    struct Case {
        const char* name;
        real rpm, p, mdot_n, mdot_kf, retard;
    };
    const SimulatedEngineDesc rdesc = fx::with_reserved_maps(fx::electronic_engine());
    SimulatedEngine na(rdesc, reserved_maps());
    // NA: 4000 rpm the knock limit wins over schedule - 1 deg, 6000 rpm schedule - 4 deg wins (literal fixture values:
    // fx_kl(4000, 1e5) 9.0 < fx_spark 10.5 - 1; fx_kl(6000, 1e5) 16.5 > fx_spark 16.75 - 4).
    REQUIRE(fx::fx_kl(4000.0, 1e5) < fx::fx_spark(4000.0, 1e5) - 1.0);
    REQUIRE(fx::fx_kl(6000.0, 1e5) > fx::fx_spark(6000.0, 1e5) - 4.0);
    const Case cases[] = {
        {"NA WOT 4000 rpm, 15.8 g/s (50 hp shot), retard 1 (knock limit wins)", 4000.0, 100000.0, 0.0158, 0.0040, 1.0},
        {"NA WOT 6000 rpm, 35.6 g/s (100 hp shot), retard 4", 6000.0, 100000.0, 0.0356, 0.0090, 4.0},
    };
    for (const Case& k : cases) {
        const NitrousCharge nc = charge_in(k.mdot_n, k.mdot_kf, kHinSat950, k.retard);
        const auto ev = na.evaluate_torque(w_of(k.rpm), k.p, true, 1.0, false, nullptr, &nc);
        const auto ev0 = na.evaluate_torque(w_of(k.rpm), k.p, true, 1.0, false);
        const RefCharge c = ref_charge(rdesc, k.rpm, k.p, 288.15, k.mdot_n, k.mdot_kf, kHinSat950);
        const real tau_ref = ref_tau_comb(rdesc, k.rpm, k.p, c, k.retard);
        std::printf("[N2 E1] %-62s a %.6e (no-N2O %.6e, %+.2f %%) T_mix %.3f K (dT %.2f K) n/a %.4f lambda_c %.4f tau_comb %.4f N*m "
                    "(no-N2O %.4f, %+.2f %%); rel a %.1e T %.1e lambda %.1e tau %.1e\n",
                    k.name, ev.charge.air_kg_rev, ev0.m_air_rev, 100.0 * (ev.charge.air_kg_rev / ev0.m_air_rev - 1.0), ev.charge.t_mix_k,
                    ev.charge.t_mix_k - 288.15, c.n / c.a, ev.charge.lambda_combined, ev.tau_comb, ev0.tau_comb,
                    100.0 * (ev.tau_comb / ev0.tau_comb - 1.0), rel(ev.charge.air_kg_rev, c.a), rel(ev.charge.t_mix_k, c.t_mix),
                    rel(ev.charge.lambda_combined, c.lambda_c), rel(ev.tau_comb, tau_ref));
        INFO(k.name);
        CHECK(rel(ev.charge.air_kg_rev, c.a) < 1e-9);
        CHECK(ev.m_air_rev == ev.charge.air_kg_rev);
        CHECK(rel(ev.charge.t_mix_k, c.t_mix) < 1e-9);
        CHECK(rel(ev.charge.o2_equiv_air_kg_rev, c.a_eq) < 1e-9);
        CHECK(rel(ev.charge.lambda_combined, c.lambda_c) < 1e-9);
        CHECK(rel(ev.tau_comb, tau_ref) < 1e-9);
        CHECK(rel(ev.m_fuel_rev, c.f) < 1e-9);
        CHECK(rel(ev.n2o_burnt_rev, c.n) < 1e-12);
        CHECK_FALSE(ev.charge.overflow);
        CHECK(ev.charge.t_mix_k < 288.15 - 5.0); // the evaporating N2O cools the charge
    }

    SECTION("turbo engine at 5000 rpm: the N2O meets the charge air at the intercooler outlet temperature") {
        SimulatedEngineDesc td = fx::electronic_engine();
        td.turbo_pair = tb::hyper_like_pair();
        SimulatedEngine te(td);
        const real w = w_of(5000.0);
        for (int i = 0; i < 1440; ++i) {
            (void)te.pre_solve(running(w), wot(), TcuRequest{}, kH);
            (void)te.post_solve(w, kH);
        }
        REQUIRE(te.turbo() != nullptr);
        const real t_charge = te.turbo()->charge_k;
        const real p = te.p_im();
        const real mdot_n = 0.0356, mdot_kf = 0.0090, retard = 2.0;
        const NitrousCharge nc = charge_in(mdot_n, mdot_kf, kHinSat950, retard);
        const auto ev = te.evaluate_torque(w, p, true, 1.0, false, nullptr, &nc);
        const RefCharge c = ref_charge(td, 5000.0, p, t_charge, mdot_n, mdot_kf, kHinSat950);
        const real tau_ref = ref_tau_comb(td, 5000.0, p, c, retard);
        std::printf("[N2 E1] turbo 5000 rpm p_im %.0f Pa T_charge %.2f K: a %.6e T_mix %.3f K lambda_c %.4f tau_comb %.4f; rel a %.1e T %.1e "
                    "tau %.1e\n",
                    p, t_charge, ev.charge.air_kg_rev, ev.charge.t_mix_k, ev.charge.lambda_combined, ev.tau_comb, rel(ev.charge.air_kg_rev, c.a),
                    rel(ev.charge.t_mix_k, c.t_mix), rel(ev.tau_comb, tau_ref));
        CHECK(t_charge != 288.15);
        CHECK(ev.charge.t_in_k == t_charge);
        CHECK(rel(ev.charge.air_kg_rev, c.a) < 1e-9);
        CHECK(rel(ev.charge.t_mix_k, c.t_mix) < 1e-9);
        CHECK(rel(ev.charge.lambda_combined, c.lambda_c) < 1e-9);
        CHECK(rel(ev.tau_comb, tau_ref) < 1e-9);
    }
}

// ---------------------------------------------------------------------------
// E2: dtau/domega against a central difference, every N2O term nonzero (n and kf fall
// as 1/omega, a and T_mix move with them, eta_i at a_eq, f_lambda at lambda_c). Sabotage:
// dropping dn/drpm = -n/rpm fails.
// ---------------------------------------------------------------------------
TEST_CASE("N2 E2: dtau/domega with N2O against a central difference, 1e-6 relative", "[unit][n2o][drivetrain]") {
    auto soft = fx::with_reserved_maps(fx::electronic_engine());
    soft.limiter.type = LimiterType::SoftCut;
    auto soft_maps = SimulatedEngine::build_maps(soft);
    SimulatedEngine e(fx::with_reserved_maps(fx::electronic_engine()), reserved_maps());
    SimulatedEngine es(soft, soft_maps);
    struct Case {
        const char* name;
        SimulatedEngine* eng;
        real rpm, p, s;
        real mdot_n, mdot_kf, retard;
    };
    const Case cases[] = {
        {"WOT 4137 rpm, knock limit active", &e, 4137.0, 93170.0, 1.0, 0.0158, 0.0040, 1.0},
        {"part load 3011 rpm", &e, 3011.0, 61234.0, 1.0, 0.0158, 0.0040, 2.0},
        {"WOT 5873 rpm, retard wins", &e, 5873.0, 97310.0, 1.0, 0.0356, 0.0090, 4.0},
        {"soft-cut band 6700 rpm", &es, 6700.0, 91000.0, 1.0, 0.0158, 0.0040, 2.0},
        {"starving s = 0.5", &e, 4137.0, 93170.0, 0.5, 0.0158, 0.0040, 1.0},
    };
    for (const Case& k : cases) {
        const NitrousCharge nc = charge_in(k.mdot_n, k.mdot_kf, kHinSat950, k.retard);
        const real w = w_of(k.rpm);
        const real dw = 1e-3;
        const auto ev = k.eng->evaluate_torque(w, k.p, true, k.s, false, nullptr, &nc);
        const real tp = k.eng->evaluate_torque(w + dw, k.p, true, k.s, false, nullptr, &nc).tau;
        const real tm = k.eng->evaluate_torque(w - dw, k.p, true, k.s, false, nullptr, &nc).tau;
        const real fd = (tp - tm) / (2.0 * dw);
        const real d0 = k.eng->evaluate_torque(w, k.p, true, k.s, false).dtau_domega;
        std::printf("[N2 E2] %-34s dtau/domega %.10f vs central %.10f (rel %.2e); without N2O %.10f\n", k.name, ev.dtau_domega, fd,
                    rel(ev.dtau_domega, fd), d0);
        INFO(k.name);
        CHECK(rel(ev.dtau_domega, fd) < 1e-6);
        CHECK(ev.dtau_domega != d0);
    }
}

// ---------------------------------------------------------------------------
// E3: manifold_rhs's dfdp carries da/dp of the charge model; m_cyl is the torque path's
// air. Sabotage: the pre-N2 dm_cyl/dp in the N2O branch fails.
// ---------------------------------------------------------------------------
TEST_CASE("N2 E3: manifold dfdp with N2O against a central difference; plenum mass consistent", "[unit][n2o][drivetrain]") {
    SimulatedEngine e(fx::with_reserved_maps(fx::electronic_engine()), reserved_maps());
    struct Case {
        real rpm, p, x;
    };
    const Case cases[] = {{4137.0, 63170.0, 1.0}, {2789.0, 41234.0, 0.3}, {5873.0, 91234.0, 1.0}};
    const NitrousCharge nc = charge_in(0.0158, 0.0040, kHinSat950, 0.0);
    for (const Case& k : cases) {
        const real w = w_of(k.rpm);
        const real dp = 1.0;
        const auto r = e.manifold_rhs(k.p, w, k.x, 0.0, &nc);
        const real fp = e.manifold_rhs(k.p + dp, w, k.x, 0.0, &nc).f;
        const real fm = e.manifold_rhs(k.p - dp, w, k.x, 0.0, &nc).f;
        const real fd = (fp - fm) / (2.0 * dp);
        const auto ev = e.evaluate_torque(w, k.p, true, 1.0, false, nullptr, &nc);
        const real m_cyl_torque = ev.m_air_rev * w / (2.0 * kPi);
        const auto r0 = e.manifold_rhs(k.p, w, k.x, 0.0);
        std::printf("[N2 E3] %.0f rpm p %.0f x %.2f: dfdp %.6f vs central %.6f (rel %.2e); m_cyl %.6e (no N2O %.6e) vs torque path %.6e\n",
                    k.rpm, k.p, k.x, r.dfdp, fd, rel(r.dfdp, fd), r.m_cyl, r0.m_cyl, m_cyl_torque);
        CHECK(rel(r.dfdp, fd) < 1e-6);
        CHECK(rel(r.m_cyl, m_cyl_torque) < 1e-12);
        // Displacement and cooling compete: at 2789 rpm / 41 kPa the N2O displaces more air than its cooling adds,
        // at the two WOT-ish points the cooling wins (+0.37 %, +0.38 % air) - only "the charge model is active".
        CHECK(r.m_cyl != r0.m_cyl);
        CHECK(r.dfdp != r0.dfdp);
    }
}

// ---------------------------------------------------------------------------
// E4: energy closure. (a) White-box, per substep on a held crank: indicated_work ==
// eta_i*eta_spark*(f_lambda(lambda_c)*LHV*fuel_mass + n2o_energy_in) (T5 pattern), the
// whole delivered N2O burnt, kit fuel = h*kf*omega/(2 pi). (b) Through World: every tick
// fuel_energy_in + n2o_energy_in - indicated_work - engine_heat == 0 (relative 1e-12 of
// the energy in), the bottle and the tank close, balance_error stays at its 5e-8 bound.
// Sabotage: the Powertrain's engine_heat without the N2O term fails (b).
// ---------------------------------------------------------------------------
TEST_CASE("N2 E4: indicated work and engine_heat close with the decomposition energy", "[unit][n2o][drivetrain][fuel]") {
    SECTION("white-box, per substep") {
        const SimulatedEngineDesc d = fx::with_reserved_maps(fx::electronic_engine());
        SimulatedEngine e(d, reserved_maps());
        const real mdot_n = 0.0158, mdot_kf = 0.0040, retard = 2.0;
        real worst = 0.0, worst_kit = 0.0, n2o_e = 0.0, ind = 0.0;
        int substeps = 0;
        for (real rpm : {3137.0, 4421.0, 5791.0}) {
            const real w = w_of(rpm);
            Sensors s = running(w);
            s.nitrous = nitrous_sensor(mdot_n, mdot_kf, kHinSat950, retard);
            for (int i = 0; i < 480; ++i) {
                const PreSolveResult pre = e.pre_solve(s, wot(), TcuRequest{}, kH);
                const PostSolveResult po = e.post_solve(w, kH);
                REQUIRE_FALSE(pre.limiter_active);
                const RefCharge c = ref_charge(d, rpm_of(w), e.p_im(), 288.15, mdot_n, mdot_kf, kHinSat950);
                const real gs = ref_gs(d, rpm_of(w), e.p_im(), c, retard);
                const real expect = gs * (d.maps.f_lambda.evaluate(c.lambda_c) * d.fuels[0].lhv_j_kg * po.fuel_mass_delta + po.n2o_energy_in_delta);
                worst = std::max(worst, rel(po.indicated_work_delta, expect));
                CHECK(po.n2o_mass_delta == kH * mdot_n);
                CHECK(po.n2o_unburnt_delta == 0.0);
                CHECK(rel(po.n2o_energy_in_delta, po.n2o_mass_delta * ox_dh()) < 1e-14);
                worst_kit = std::max(worst_kit, rel(po.kit_fuel_mass_delta, kH * c.kf * w / (2.0 * kPi)));
                CHECK(pre.n2o_kit_fuel_g_s > 0.0);
                CHECK(pre.charge_temperature_k < 288.15);
                CHECK(po.released_heat_delta > 0.0);
                n2o_e += po.n2o_energy_in_delta;
                ind += po.indicated_work_delta;
                ++substeps;
            }
        }
        std::printf("[N2 E4a] %d substeps: indicated vs eta_i*eta_spark*(f_lambda*LHV*m_f + dH*m_n) worst rel %.2e; kit fuel worst rel %.2e; "
                    "decomposition %.1f J of %.1f J indicated\n",
                    substeps, worst, worst_kit, n2o_e, ind);
        CHECK(worst < 1e-10);
        CHECK(worst_kit < 1e-12);
    }

    SECTION("through World, every tick") {
        const vehicle::VehicleDesc desc = sim_sedan(true);
        Rig rig = make_rig(desc);
        World& w = *rig.world;
        const real bottle0 = w.n2o_bottle_mass(rig.vehicle);
        const real tank0 = w.fuel_tank_mass(rig.vehicle);
        for (int t = 0; t < 240; ++t) w.step();
        w.set_control("throttle", 1.0);
        w.set_control("nitrous_arm", 1.0);
        real worst_closure = 0.0, worst_balance = 0.0, worst_bottle = 0.0, worst_tank = 0.0;
        int flowing_ticks = 0;
        // Upshift near the controller's 6000 rpm window edge so the kit sprays in 1st, 2nd and 3rd (each gear
        // crosses the window edge once: the cut at max_rpm and the re-arm after the shift are both exercised).
        for (int t = 0; t < 960; ++t) {
            const PowertrainSnapshot& ps = w.powertrain_state(rig.vehicle);
            if (ps.gear >= 1 && ps.gear < 3 && ps.shift_phase == ShiftPhase::Idle && ps.engine_rpm > 5900.0)
                w.set_control("gear_request", static_cast<real>(ps.gear + 1));
            w.step();
            const PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
            const real in = e.fuel_energy_in + e.n2o_energy_in;
            const real resid = in - e.indicated_work - e.engine_heat;
            if (in > 0.0) worst_closure = std::max(worst_closure, std::abs(resid) / in);
            const real denom = sum_abs_terms(e);
            if (denom > 1e-9) worst_balance = std::max(worst_balance, std::abs(e.balance_error) / denom);
            worst_bottle = std::max(worst_bottle, std::abs((bottle0 - w.n2o_bottle_mass(rig.vehicle)) - (e.n2o_mass + e.n2o_purged - e.n2o_external_added)));
            worst_tank = std::max(worst_tank, std::abs((tank0 - w.fuel_tank_mass(rig.vehicle)) - (e.fuel_mass - e.fuel_external_added)));
            if (w.powertrain_state(rig.vehicle).engines.at(0).n2o_active) ++flowing_ticks;
        }
        const PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
        std::printf("[N2 E4b] %d of 960 ticks spraying: n2o %.4f g (unburnt %.6f g), decomposition %.1f J, kit fuel %.4f g of %.4f g fuel, "
                    "indicated %.1f J, engine_heat %.1f J; closure worst rel %.2e, balance worst %.2e, bottle %.2e kg, tank %.2e kg, "
                    "efficiency violations %llu\n",
                    flowing_ticks, e.n2o_mass * 1e3, e.n2o_unburnt * 1e3, e.n2o_energy_in, e.kit_fuel_mass * 1e3, e.fuel_mass * 1e3,
                    e.indicated_work, e.engine_heat, worst_closure, worst_balance, worst_bottle, worst_tank,
                    static_cast<unsigned long long>(e.fuel_efficiency_violations));
        CHECK(flowing_ticks > 100);
        CHECK(e.n2o_energy_in > 0.0);
        CHECK(e.kit_fuel_mass > 0.0);
        CHECK(e.kit_fuel_mass < e.fuel_mass);
        CHECK(e.n2o_unburnt < e.n2o_mass);
        CHECK(e.n2o_energy_in <= (e.n2o_mass - e.n2o_unburnt) * ox_dh() * (1.0 + 1e-12));
        CHECK(worst_closure < 1e-12);
        CHECK(worst_balance < 5e-8);
        CHECK(worst_bottle < 1e-12 * bottle0);
        CHECK(worst_tank < 1e-12 * tank0); // test_fuel_tank T2's bound, relative to the initial content
        CHECK(e.fuel_efficiency_violations == 0);
    }
}

// ---------------------------------------------------------------------------
// E5: no N2O flow is bit-identical. White-box: an active block with zero N2O flow (kit
// fuel, retard, tables all set) and a null charge input give the same bytes as no block;
// whole vehicle: a disarmed kit on the MVEM sedan is bit-identical, tick for tick, to the
// kit-less sedan in every engine/ledger/chassis quantity. Sabotage: gating the charge
// model on `active` alone applies the retard (and the overflow branch) and fails.
// ---------------------------------------------------------------------------
TEST_CASE("N2 E5: zero N2O flow is bit-identical to no kit", "[unit][n2o][drivetrain]") {
    SECTION("white-box engine") {
        const SimulatedEngineDesc d = fx::with_reserved_maps(fx::electronic_engine());
        SimulatedEngine a(d, reserved_maps());
        SimulatedEngine b(d, reserved_maps());
        const NitrousCharge zero = charge_in(0.0, 0.004, kHinSat950, 4.0);
        for (real rpm : {1234.0, 4137.0, 6100.0}) {
            const auto x = a.evaluate_torque(w_of(rpm), 87000.0, true, 1.0, false);
            const auto y = a.evaluate_torque(w_of(rpm), 87000.0, true, 1.0, false, nullptr, &zero);
            CHECK(x.tau == y.tau);
            CHECK(x.dtau_domega == y.dtau_domega);
            CHECK(x.m_fuel_rev == y.m_fuel_rev);
            CHECK(x.spark_deg == y.spark_deg);
            const auto mx = a.manifold_rhs(87000.0, w_of(rpm), 0.7, 0.0);
            const auto my = a.manifold_rhs(87000.0, w_of(rpm), 0.7, 0.0, &zero);
            CHECK(mx.f == my.f);
            CHECK(mx.dfdp == my.dfdp);
        }
        real w = w_of(2500.0);
        for (int i = 0; i < 960; ++i) {
            Sensors sa = running(w);
            Sensors sb = sa;
            sb.nitrous = nitrous_sensor(0.0, 0.004, kHinSat950, 4.0); // active, no N2O
            const real c = i < 480 ? 1.0 : 0.2;
            const PreSolveResult ra = a.pre_solve(sa, ThrottleCommand{c, 0.0, 1.0}, TcuRequest{}, kH);
            const PreSolveResult rb = b.pre_solve(sb, ThrottleCommand{c, 0.0, 1.0}, TcuRequest{}, kH);
            const real wn = w + kH * (ra.tau0 - ra.friction_cap_kinetic) / (d.inertia_kgm2 + kH * ra.d);
            const PostSolveResult pa = a.post_solve(wn, kH);
            const PostSolveResult pb = b.post_solve(wn, kH);
            REQUIRE(ra.tau0 == rb.tau0);
            REQUIRE(ra.d == rb.d);
            REQUIRE(ra.spark_advance_deg == rb.spark_advance_deg);
            REQUIRE(ra.exhaust_mass_flow_kg_s == rb.exhaust_mass_flow_kg_s);
            REQUIRE(ra.torque_demand == rb.torque_demand);
            REQUIRE(pa.fuel_mass_delta == pb.fuel_mass_delta);
            REQUIRE(pa.indicated_work_delta == pb.indicated_work_delta);
            REQUIRE(pa.released_heat_delta == pb.released_heat_delta);
            REQUIRE(pb.n2o_mass_delta == 0.0);
            REQUIRE(pb.kit_fuel_mass_delta == 0.0);
            REQUIRE(rb.charge_temperature_k == 0.0);
            REQUIRE(a.fold_state_hash(0) == b.fold_state_hash(0));
            w = wn;
        }
    }
    SECTION("whole vehicle: a disarmed kit vs no kit") {
        Rig a = make_rig(sim_sedan(false));
        Rig b = make_rig(sim_sedan(true));
        for (int t = 0; t < 960; ++t) {
            const real thr = t < 120 ? 0.0 : 1.0;
            for (Rig* r : {&a, &b}) {
                r->world->set_control("throttle", thr);
                r->world->step();
            }
            const PowertrainEnergy& ea = a.world->powertrain_energy(a.vehicle);
            const PowertrainEnergy& eb = b.world->powertrain_energy(b.vehicle);
            const EngineSoundState& sa = a.world->powertrain_state(a.vehicle).engines.at(0);
            const EngineSoundState& sb = b.world->powertrain_state(b.vehicle).engines.at(0);
            REQUIRE(ea.fuel_mass == eb.fuel_mass);
            REQUIRE(ea.fuel_energy_in == eb.fuel_energy_in);
            REQUIRE(ea.indicated_work == eb.indicated_work);
            REQUIRE(ea.engine_heat == eb.engine_heat);
            REQUIRE(ea.engine_out == eb.engine_out);
            REQUIRE(ea.balance_error == eb.balance_error);
            REQUIRE(eb.n2o_mass == 0.0);
            REQUIRE(eb.kit_fuel_mass == 0.0);
            REQUIRE(sa.omega == sb.omega);
            REQUIRE(sa.spark_advance_deg == sb.spark_advance_deg);
            REQUIRE(sa.p_intake_pa == sb.p_intake_pa);
            const Pose pa = a.world->get_pose(a.chassis);
            const Pose pb = b.world->get_pose(b.chassis);
            REQUIRE(pa.position.x == pb.position.x);
            REQUIRE(pa.position.z == pb.position.z);
        }
        CHECK(a.world->powertrain_state(a.vehicle).engine_rpm > 2000.0);
    }
}

// ---------------------------------------------------------------------------
// E6: a cut substep (ignition cut, fuel-cut limiter) books the delivered N2O as
// unburnt, releases no decomposition energy and does no indicated work; the kit fuel
// still flows (and is booked) while the ECU fuel stops. Sabotage: fire_n = 1 (the N2O
// decomposing in a cut cylinder) fails.
// ---------------------------------------------------------------------------
TEST_CASE("N2 E6: a cut substep books unburnt N2O and zero energy", "[unit][n2o][drivetrain]") {
    const SimulatedEngineDesc d = fx::with_reserved_maps(fx::electronic_engine());
    SimulatedEngine e(d, reserved_maps());
    const real mdot_n = 0.0158, mdot_kf = 0.0040;
    const real w = w_of(4137.0);
    Sensors s = running(w);
    s.nitrous = nitrous_sensor(mdot_n, mdot_kf, kHinSat950, 2.0);
    for (int i = 0; i < 240; ++i) {
        (void)e.pre_solve(s, wot(), TcuRequest{}, kH);
        (void)e.post_solve(w, kH);
    }
    SECTION("ignition cut") {
        TcuRequest cut;
        cut.ignition_cut = true;
        const PreSolveResult pre = e.pre_solve(s, wot(), cut, kH);
        const PostSolveResult po = e.post_solve(w, kH);
        const real kf_rev = mdot_kf * 2.0 * kPi / w;
        std::printf("[N2 E6] ignition cut: n2o %.3e kg unburnt %.3e kg energy %.3e J indicated %.3e J fuel %.3e kg (kit %.3e kg)\n",
                    po.n2o_mass_delta, po.n2o_unburnt_delta, po.n2o_energy_in_delta, po.indicated_work_delta, po.fuel_mass_delta,
                    po.kit_fuel_mass_delta);
        CHECK(po.n2o_mass_delta == kH * mdot_n);
        CHECK(po.n2o_unburnt_delta == po.n2o_mass_delta);
        CHECK(po.n2o_energy_in_delta == 0.0);
        CHECK(po.indicated_work_delta == 0.0);
        CHECK(po.fuel_mass_delta == po.kit_fuel_mass_delta);
        CHECK(rel(po.kit_fuel_mass_delta, kH * kf_rev * w / (2.0 * kPi)) < 1e-15);
        CHECK(pre.exhaust_mass_flow_kg_s > mdot_n);
        CHECK(po.released_heat_delta == 0.0);
    }
    SECTION("fuel-cut limiter") {
        const real wl = w_of(6850.0);
        Sensors sl = running(wl);
        sl.nitrous = s.nitrous;
        const PreSolveResult pre = e.pre_solve(sl, wot(), TcuRequest{}, kH);
        const PostSolveResult po = e.post_solve(wl, kH);
        REQUIRE(pre.limiter_active);
        CHECK(po.n2o_unburnt_delta == po.n2o_mass_delta);
        CHECK(po.n2o_mass_delta > 0.0);
        CHECK(po.n2o_energy_in_delta == 0.0);
        CHECK(po.indicated_work_delta == 0.0);
    }
}

// ---------------------------------------------------------------------------
// E7 / V1: NACA MR E5F26 (Tauschek, Corrington, Huppert 1945, NTRS 19930093145), Table
// II, as transcribed in out/n2o_plan.md section 9.1 (the scan was not re-read in N2: no
// download permission). Gas injection (no cooling: h_in = h_g(T_in)), constant manifold
// pressure, fuel/oxygen mass 0.410 held by the lambda target and the kit fuel. The
// generated A25A maps (car_sedan_gen's engine) at 3000 rpm, p_im frozen at its WOT steady
// value. (tau_comb + tau_pump) ratio vs the measured IMEP ratio, +-3 percentage points
// [AS, owner Q3]; the 0.15 row is report only (its scan row was garbled in the OCR).
// ---------------------------------------------------------------------------
TEST_CASE("N2 E7: NACA MR E5F26 IMEP increase vs N2O/air ratio on the generated A25A", "[unit][n2o][drivetrain][cycle]") {
    vehicle::VehicleDesc car;
    {
        jobs::JobSystem pool(10);
        io::EngineMapOptions opt;
        opt.pool = &pool;
        car = io::load_vehicle_json(data_path("vehicles/car_sedan_gen.json"), opt);
    }
    const SimulatedEngineDesc* gen = find_component<SimulatedEngineDesc>(car);
    REQUIRE(gen != nullptr);
    const real afr = gen->fuels[0].afr_stoich;
    const real rpm = 3000.0;
    const real w = w_of(rpm);

    // p_im at WOT, steady (the engine's own lambda schedule; the override below only changes fuelling).
    SimulatedEngine base(*gen);
    base.reset(EngineState::Running);
    for (int i = 0; i < 1920; ++i) {
        (void)base.pre_solve(running(w), wot(), TcuRequest{}, kH);
        (void)base.post_solve(w, kH);
    }
    const real p = base.p_im();

    // lambda = 1 / (F/O * O2_air * AFR_st): the same fuel/oxygen ratio 0.410 for the air.
    const real lambda_n = 1.0 / (0.410 * kO2Air * afr);
    SimulatedEngineDesc dn = *gen;
    std::fill(dn.ecu.lambda_target.values.begin(), dn.ecu.lambda_target.values.end(), lambda_n);
    SimulatedEngine e(dn);
    const auto ev0 = e.evaluate_torque(w, p, true, 1.0, false);
    const real a0 = ev0.m_air_rev;
    const real t_in = ev0.charge.t_in_k;
    const real h_in = ref_h_g(t_in); // gas at the inlet temperature: no evaporative cooling
    const real r = kMAir / ox_molar();
    const real imep0 = ev0.tau_comb + ev0.tau_pump;

    struct Row {
        real ratio, measured; // measured IMEP increase, fraction (NaN = report only)
    };
    const Row rows[] = {{0.05, 255.0 / 237.0 - 1.0}, {0.10, 271.0 / 237.0 - 1.0}, {0.15, std::nan("")}, {0.20, 297.0 / 237.0 - 1.0}};
    std::printf("[N2 E7] A25A generated maps, %.0f rpm, p_im %.0f Pa (WOT steady), T_in %.2f K, lambda %.4f (F/O 0.410, AFR %.3f), "
                "IMEP0 %.3f N*m\n",
                rpm, p, t_in, lambda_n, afr, imep0);
    for (const Row& row : rows) {
        const real n = row.ratio * a0 / (1.0 + r * row.ratio); // n/a = ratio with a = a0 - r*n (moles at T_in)
        const real kf = 0.410 * oxidizer().oxygen_mass_fraction * n;
        const NitrousCharge nc = charge_in(n * w / (2.0 * kPi), kf * w / (2.0 * kPi), h_in, 0.0);
        const auto ev = e.evaluate_torque(w, p, true, 1.0, false, nullptr, &nc);
        const real inc = (ev.tau_comb + ev.tau_pump) / imep0 - 1.0;
        std::printf("[N2 E7] N2O/air %.2f: n/a %.6f T_mix %.3f K lambda_c %.4f eta_i %.4f (no N2O %.4f) IMEP %+.2f %% (NACA %s%.2f %%)\n",
                    row.ratio, ev.charge.n2o_kg_rev / ev.charge.air_kg_rev, ev.charge.t_mix_k, ev.charge.lambda_combined, ev.eta_i, ev0.eta_i,
                    100.0 * inc, std::isnan(row.measured) ? "report only, plan's transcription ~" : "",
                    std::isnan(row.measured) ? 20.0 : 100.0 * row.measured);
        CHECK(rel(ev.charge.n2o_kg_rev / ev.charge.air_kg_rev, row.ratio) < 1e-9);
        CHECK(std::abs(ev.charge.t_mix_k - t_in) < 1e-9);
        CHECK(rel(ev.charge.lambda_combined, lambda_n) < 1e-9);
        if (!std::isnan(row.measured)) CHECK(std::abs(inc - row.measured) <= 0.03);
    }
}

// ---------------------------------------------------------------------------
// E8: an active kit on the MVEM sedan hashes identically on 1 and 10 workers and steps
// without allocating (own count 0; external = Jolt's known QuadTree gap, reported).
// ---------------------------------------------------------------------------
namespace {
std::uint64_t run_e8(int workers, bool arm_alloc) {
    vehicle::VehicleDesc desc = sim_sedan(true);
    Rig rig = make_rig(desc, workers);
    World& w = *rig.world;
    for (int t = 0; t < 240; ++t) w.step();
    namespace da = ps::jobs::debug_alloc;
    const bool arm = arm_alloc && da::hook_available();
    if (arm) {
        da::reset();
        da::set_armed(true);
    }
    w.set_control("throttle", 1.0);
    w.set_control("nitrous_arm", 1.0);
    for (int t = 0; t < 480; ++t) {
        if (t == 200) w.set_control("nitrous_arm", 0.0);
        if (t == 260) w.set_control("nitrous_arm", 1.0);
        w.step();
    }
    if (arm) {
        da::set_armed(false);
        const auto own = da::own_violation_count();
        WARN("N2 E8 debug_alloc (" << workers << " workers): own " << own << ", external (Jolt) " << da::external_violation_count());
        CHECK(own == 0);
    }
    CHECK(w.powertrain_energy(rig.vehicle).n2o_energy_in > 0.0);
    return w.state_hash();
}
} // namespace

TEST_CASE("N2 E8: an active kit on the simulated engine hashes identically on 1 and 10 workers, allocation-free",
          "[unit][n2o][drivetrain][jobs][determinism]") {
    const std::uint64_t h1 = run_e8(1, true);
    const std::uint64_t h10 = run_e8(10, true);
    std::printf("[N2 E8] state hash 1 worker 0x%016llx, 10 workers 0x%016llx\n", static_cast<unsigned long long>(h1),
                static_cast<unsigned long long>(h10));
    CHECK(h1 == h10);
}
