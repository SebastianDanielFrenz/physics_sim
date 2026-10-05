// tests/unit/test_engine_thermal_coupling.cpp - engine thermal plan T1: how the oil-viscosity friction scale k and
// the heat bookkeeping act inside the two engine models (out/engine_thermal_plan.md section 6.1 / 12, vault PHYS-063).
// Tags [thermal][drivetrain].
//
// C4 content: I1-graph (EngineTorqueMap, white-box pre_solve/post_solve). C5 adds I1-sim (SimulatedEngine) and I2.
//
//
// Engine thermal T1b replaced the T1 law (k multiplied the whole smooth friction, k_g = 1 + phi_f (k - 1) for the graph
// engine, and the Coulomb caps): k now sets only the hydrodynamic increment dF(n) = (k - 1) F(n_ref) sqrt(n / n_ref)
// (core/src/drivetrain/hydrodynamic_friction.h), the Coulomb/breakaway caps are never scaled, and the graph engine's
// idle feed-forward/authority rise by dF_ecu(idle) / (w - m)(idle). The T1 sabotages C4a-c and C5a, C5b, C5f were
// rewritten against the new law; C4d, C5c-e, C5h are unchanged in substance.
//
// Sabotage record (each applied once to core/src, measured 2026-10-05, the named check then fails):
//   B1 caps scaled by k again (graph and sim) -> I1-graph caps 14.028 vs 6.0 / 28.056 vs 12.0 N*m at k 2.338
//      (152.64 vs 12.0 static at 12.72), I1-sim caps fail likewise; I9 -30 C: the sim car never turns (max cranking
//      rpm 5.8e-11), I2 long -10 C does not start, the viscosity ratio reads 1.635 (cited 1.45).
//   B2 speed shape linear (sqrt(n/n_ref) -> n/n_ref) -> I1-graph tau0/torque_demand off by 10.68 N*m, d off by
//      0.0885, idle shift 0.0770 vs 0.1218; I1-sim tau0 off by 2.39 N*m at 1234 rpm; I9 ratio 1.182 (cited 1.45),
//      r100/r2000 0.786 (bound < 0.6); I3 fails too.
//   B3 graph increment on the whole motoring curve (phi_f dropped from f_hyd_ref) -> I1-graph tau0 off by 25.93 N*m,
//      idle shift 0.2030 vs 0.1218, heat 29.73 vs 20.54 J (the I9 graph start still passes: the idle shift grows with it).
//   B4 graph idle feed-forward/authority shift removed -> I1-graph throttle shift 0.0 vs 0.1218, cap 0.30 vs 0.4218;
//      I9 -30 C: car_sedan_thermal never reaches Running (t_running -1).
//   B5 graph friction-heat increment dropped -> I1-graph friction_heat_delta 6.754 vs 20.540 J.
//   B6 d without the increment's slope (graph and sim) -> I1-graph d 0.0344 vs finite difference 0.0915 (k 2.338) and
//      0.0344 vs 0.5348 (k 12.72); I1-sim dtau/domega vs finite difference off by 0.0431 at 1234 rpm.
//   C4d I1-graph: released_heat_delta = 0 -> 0 vs 257.7 J (fuel energy of the substep).
//   C5c I1-sim: the ECU's live steady torque built with k = 1 -> the k_ecu demand CHECK fails.
//   C5d I1-sim: live idle control ignoring k_ecu -> idle plate 0.02111, the CHECK needs > 1.02 x.
//   C5e I1-sim: released heat = the whole fuel energy (no eta_c) -> 24.818 J vs lambda * fuel energy.
//   C5f I1-sim: the PLANT evaluating with the ECU's k -> tau0 does not fall by dF.
//   C5h I1-sim: no live idle recompute when only k_ecu changes -> idle plate 0.00015 (uninitialised live feed-forward).
// U9a graph (test_engine_thermal_powertrain.cpp): the plant scale handed to the engine perturbed by +1e-14 -> the
//   rig trajectory differs from the no-thermal one at tick 0 (a perturbation inside the engine alone would hit both
//   runs equally and prove nothing).

#include "drivetrain/engine_simulated.h"
#include "drivetrain/engine_torque_map.h"
#include "drivetrain/powertrain.h"

#include "mvem_fixtures.h"

#include "ps/drivetrain/engine_maps.h"
#include "ps/io/vehicle_io.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string>
#include <variant>

using namespace ps;
using namespace ps::drivetrain;
using Catch::Approx;

namespace {

constexpr real kPi = 3.14159265358979323846;
constexpr real kRpmToRad = 2.0 * kPi / 60.0;
constexpr real kH = 1.0 / 960.0;

std::string data_path(const std::string& rel) { return std::string(PS_SOURCE_DIR) + "/data/" + rel; }

TorqueMapEngineDesc graph_params() {
    ComponentParams p = io::load_engine_json(data_path("engines/i4_2l_na.json"));
    REQUIRE(std::holds_alternative<TorqueMapEngineDesc>(p));
    return std::get<TorqueMapEngineDesc>(p);
}

Sensors sensors_at(real rpm, real k) {
    Sensors s;
    s.omega = rpm * kRpmToRad;
    s.ignition = 1.0;
    s.starter = 0.0;
    s.clutch_engaged_fraction = 1.0;
    s.gearbox_in_neutral = true;
    s.friction_scale = k;
    s.ecu_friction_scale = k;
    return s;
}

struct Eval {
    PreSolveResult pre;
    PostSolveResult post;
};

// A fresh Running engine every call (no state carried between evaluations), one pre_solve + post_solve.
Eval eval_graph(const EngineTorqueMap::Desc& d, real rpm, real theta, real k, real omega_new_rpm = -1.0) {
    EngineTorqueMap e(d);
    ThrottleCommand tc;
    tc.command = theta;
    Eval r;
    r.pre = e.pre_solve(sensors_at(rpm, k), tc, TcuRequest{}, kH);
    r.post = e.post_solve((omega_new_rpm < 0.0 ? rpm : omega_new_rpm) * kRpmToRad, kH);
    return r;
}

constexpr real kK = 2.338; // k at 20 C for the A25A oil class (plan section 6.1)

} // namespace

// The T1b hydrodynamic increment, closed form (n_ref = Sensors' default 2000 rpm, linear below 50 rpm).
static real hyd_shape(real rpm) { return rpm >= 50.0 ? std::sqrt(rpm / 2000.0) : rpm / 50.0 * std::sqrt(50.0 / 2000.0); }

TEST_CASE("I1-graph: friction scale k in EngineTorqueMap (hydrodynamic increment, unscaled caps, slope, idle, heat)",
          "[thermal][drivetrain]") {
    const TorqueMapEngineDesc tm = graph_params();
    const real phi = 0.6;
    EngineTorqueMap::Desc d = to_engine_desc(tm);
    d.thermal_friction_fraction = phi;
    const real c_k = -d.motoring_torque_nm_vs_rpm.evaluate(0.0);
    REQUIRE(c_k > 0.0);
    const real f_ref = -d.motoring_torque_nm_vs_rpm.evaluate(2000.0) * phi; // friction share of the curve at n_ref
    auto dF = [&](real k, real rpm) { return (k - 1.0) * f_ref * hyd_shape(rpm); };

    const real rpm = 3250.0; // mid-segment of both curves (knots every 500 rpm): finite differences stay on one segment
    const real m = d.motoring_torque_nm_vs_rpm.evaluate(rpm);
    const real m_smooth = m + c_k;
    REQUIRE(m_smooth < 0.0);

    SECTION("net torque: only the hydrodynamic increment changes; pumping, w - m and the boundary caps do not") {
        for (const real theta : {1.0, 0.5}) {
            INFO("theta " << theta);
            const Eval a = eval_graph(d, rpm, theta, 1.0);
            const Eval b = eval_graph(d, rpm, theta, kK);
            const real expected = -dF(kK, rpm);
            REQUIRE(expected < -1.0);
            CHECK(std::abs((b.pre.tau0 - a.pre.tau0) - expected) <= 1e-12 * std::abs(expected));
            // The ECU-visible torque demand carries the ECU's estimate of the same increment.
            CHECK(std::abs((b.pre.torque_demand - a.pre.torque_demand) - expected) <= 1e-12 * std::abs(expected));
        }
        const Eval w = eval_graph(d, rpm, 1.0, kK);
        const real w_rpm = d.wot_torque_nm_vs_rpm.evaluate(rpm);
        CHECK(std::abs(w.pre.tau0 - (m_smooth - dF(kK, rpm) + (w_rpm - m))) <= 1e-12 * std::abs(w_rpm));
        // Plant and ECU split: the plant k moves tau0 only, the ECU k the demand only.
        EngineTorqueMap ep(d), ee(d);
        Sensors sp = sensors_at(rpm, kK);
        sp.ecu_friction_scale = 1.0;
        Sensors se = sensors_at(rpm, 1.0);
        se.ecu_friction_scale = kK;
        ThrottleCommand tc;
        tc.command = 0.5;
        const PreSolveResult rp = ep.pre_solve(sp, tc, TcuRequest{}, kH);
        const PreSolveResult re = ee.pre_solve(se, tc, TcuRequest{}, kH);
        const Eval a = eval_graph(d, rpm, 0.5, 1.0);
        CHECK(rp.torque_demand == a.pre.torque_demand);
        CHECK(re.tau0 == a.pre.tau0);
    }

    SECTION("boundary caps: breakaway and Coulomb are never viscosity-scaled, whatever k") {
        for (const real k : {1.0, kK, 12.72}) {
            INFO("k " << k);
            for (const real r : {0.0, 120.0, rpm}) {
                const Eval e = eval_graph(d, r, 0.0, k);
                CHECK(e.pre.friction_cap_kinetic == c_k);
                CHECK(e.pre.friction_cap_static == d.breakaway_torque_nm);
            }
        }
    }

    SECTION("d matches a central finite difference of tau0 (the increment's sqrt slope included)") {
        for (const real k : {1.0, kK, 12.72}) {
            for (const real theta : {0.2, 0.5}) {
                INFO("k " << k << " theta " << theta);
                const real dr = 5.0; // rpm; the 500 rpm segments keep both points on one linear piece
                const Eval hi = eval_graph(d, rpm + dr, theta, k);
                const Eval lo = eval_graph(d, rpm - dr, theta, k);
                const real fd = -(hi.pre.tau0 - lo.pre.tau0) / (2.0 * dr * kRpmToRad); // d = -dtau/domega
                const Eval mid = eval_graph(d, rpm, theta, k);
                CHECK(mid.pre.d == Approx(std::max(fd, 0.0)).epsilon(1e-6));
            }
        }
        const Eval a = eval_graph(d, rpm, 0.2, 1.0);
        const Eval b = eval_graph(d, rpm, 0.2, kK);
        const real expected_dd = (kK - 1.0) * f_ref * 0.5 * hyd_shape(rpm) / rpm * (60.0 / (2.0 * kPi));
        CHECK(std::abs((b.pre.d - a.pre.d) - expected_dd) <= 1e-12 * expected_dd);
    }

    SECTION("idle: the feed-forward and the authority rise by dF_ecu(idle) / (w - m)(idle)") {
        const real span = d.wot_torque_nm_vs_rpm.evaluate(d.idle_rpm) - d.motoring_torque_nm_vs_rpm.evaluate(d.idle_rpm);
        const real shift = dF(kK, d.idle_rpm) / span;
        REQUIRE(shift > 0.03);
        // At idle speed (e = 0, fresh integrator) theta_eff = the feed-forward.
        const Eval a = eval_graph(d, d.idle_rpm, 0.0, 1.0);
        const Eval b = eval_graph(d, d.idle_rpm, 0.0, kK);
        CHECK(b.pre.throttle_eff - a.pre.throttle_eff == Approx(shift).epsilon(1e-12));
        // Far below idle the PI saturates at the authority cap: max_throttle at k = 1, max_throttle + shift cold.
        const Eval ca = eval_graph(d, 400.0, 0.0, 1.0);
        const Eval cb = eval_graph(d, 400.0, 0.0, kK);
        CHECK(ca.pre.throttle_eff == d.idle.max_throttle);
        CHECK(cb.pre.throttle_eff == Approx(d.idle.max_throttle + shift).epsilon(1e-12));
        WARN("I1-graph idle: theta_ff " << a.pre.throttle_eff << " -> " << b.pre.throttle_eff << ", cap " << ca.pre.throttle_eff
                                        << " -> " << cb.pre.throttle_eff);
    }

    SECTION("heat: released = fuel energy (eta_c = 1), friction heat = phi_f share of the warm curve + the whole increment") {
        REQUIRE(d.has_fuel_model);
        const Eval e = eval_graph(d, rpm, 1.0, kK);
        const real omega = rpm * kRpmToRad;
        const real expected = -(kH * omega * m_smooth * phi) + kH * omega * dF(kK, rpm);
        CHECK(e.post.friction_heat_delta == Approx(expected).epsilon(1e-12));
        CHECK(e.post.friction_heat_delta > 0.0);
        CHECK(e.post.released_heat_delta == e.post.fuel_energy_in_delta);
        CHECK(e.post.released_heat_delta > 0.0);
        const Eval c = eval_graph(d, rpm, 0.0, kK);
        CHECK(c.post.released_heat_delta == 0.0);
        CHECK(c.post.friction_heat_delta > 0.0);
    }

    SECTION("k = 1.0 leaves every output bit-identical to the unscaled expressions") {
        const Eval a = eval_graph(d, rpm, 0.7, 1.0);
        EngineTorqueMap::Desc d1 = d;
        d1.thermal_friction_fraction = 1.0;
        const Eval b = eval_graph(d1, rpm, 0.7, 1.0);
        CHECK(a.pre.tau0 == b.pre.tau0);
        CHECK(a.pre.d == b.pre.d);
        CHECK(a.pre.torque_demand == b.pre.torque_demand);
        const real w_rpm = d.wot_torque_nm_vs_rpm.evaluate(rpm);
        CHECK(a.pre.tau0 == (m + c_k) + 0.7 * (w_rpm - m));
    }
}

// ---------------------------------------------------------------------------------------------------------------
// I1-sim: SimulatedEngine, plant/ECU split (white-box pre_solve/post_solve on the analytic mvem fixture)
// ---------------------------------------------------------------------------------------------------------------
namespace {

namespace fx = ps::test::mvem;

Sensors sim_sensors(real rpm, real k_plant, real k_ecu, real p_amb = 101325.0) {
    Sensors s = sensors_at(rpm, k_plant);
    s.ecu_friction_scale = k_ecu;
    s.ambient.pressure_pa = p_amb;
    s.ambient.temperature_k = 288.15;
    return s;
}

ThrottleCommand cmd(real c) { return ThrottleCommand{c, 0.0, 1.0}; }

} // namespace

TEST_CASE("I1-sim: friction scale k in SimulatedEngine (plant scales the physics, the ECU scales its demand model)",
          "[thermal][drivetrain]") {
    const SimulatedEngineDesc desc = fx::electronic_engine();
    const auto maps = SimulatedEngine::build_maps(desc);
    const real c_k = desc.maps.friction_torque_nm.evaluate(0.0);
    REQUIRE(c_k == 6.0);
    auto friction = [&](real rpm) { return desc.maps.friction_torque_nm.evaluate(rpm); };
    const real f_ref = friction(2000.0);
    auto dF = [&](real k, real rpm) { return (k - 1.0) * f_ref * hyd_shape(rpm); };

    SECTION("plant: tau0 falls by dF, caps stay c_k and breakaway, d vs central finite difference") {
        for (const real rpm : {1234.0, 3137.0, 6021.0}) {
            INFO("rpm " << rpm);
            const real w = rpm * kRpmToRad;
            SimulatedEngine a(desc, maps), b(desc, maps);
            const PreSolveResult ra = a.pre_solve(sim_sensors(rpm, 1.0, 1.0), cmd(1.0), TcuRequest{}, kH);
            const PreSolveResult rb = b.pre_solve(sim_sensors(rpm, kK, 1.0), cmd(1.0), TcuRequest{}, kH); // ECU view unchanged
            const real expected = -dF(kK, rpm);
            REQUIRE(std::abs(expected) > 1.0);
            CHECK(std::abs((rb.tau0 - ra.tau0) - expected) <= 1e-12 * std::abs(expected) + 1e-13);
            for (const PreSolveResult* r : {&ra, &rb}) {
                CHECK(r->friction_cap_kinetic == c_k);
                CHECK(r->friction_cap_static == desc.breakaway_torque_nm);
            }
            CHECK(rb.torque_demand == ra.torque_demand); // the ECU's demand does not see the plant's extra friction

            const real p = rb.p_intake_pa;
            const real dw = 1e-3;
            const auto ev = b.evaluate_torque(w, p, true, 1.0, false);
            const real fd = (b.evaluate_torque(w + dw, p, true, 1.0, false).tau - b.evaluate_torque(w - dw, p, true, 1.0, false).tau) / (2.0 * dw);
            CHECK(std::abs(ev.dtau_domega - fd) <= 1e-6 * std::abs(fd));
            const auto eva = a.evaluate_torque(w, p, true, 1.0, false);
            const real expected_dd = -(kK - 1.0) * f_ref * 0.5 * hyd_shape(rpm) / rpm * (60.0 / (2.0 * kPi));
            REQUIRE(std::abs(expected_dd) > 1e-3);
            CHECK(std::abs((ev.dtau_domega - eva.dtau_domega) - expected_dd) <= 1e-9 * std::abs(expected_dd));
        }
        // The caps stay unscaled at the extreme -30 C scale too.
        SimulatedEngine x(desc, maps);
        const PreSolveResult rx = x.pre_solve(sim_sensors(120.0, 12.72, 12.72), cmd(0.0), TcuRequest{}, kH);
        CHECK(rx.friction_cap_kinetic == c_k);
        CHECK(rx.friction_cap_static == desc.breakaway_torque_nm);
    }

    SECTION("heat: released = min(1, lambda) x fuel energy; friction heat = h omega (F - c_k + dF)") {
        const real rpm = 3137.0;
        const real w = rpm * kRpmToRad;
        SimulatedEngine b(desc, maps);
        const PreSolveResult rb = b.pre_solve(sim_sensors(rpm, kK, kK), cmd(1.0), TcuRequest{}, kH);
        const PostSolveResult pb = b.post_solve(w, kH);
        REQUIRE(rb.lambda > 0.0);
        REQUIRE(rb.lambda < 1.0); // fx_lambda < 1 here: the oxygen-limited share is a real discriminator
        REQUIRE(pb.fuel_energy_in_delta > 0.0);
        CHECK(pb.released_heat_delta == Approx(rb.lambda * pb.fuel_energy_in_delta).epsilon(1e-14));
        CHECK(pb.released_heat_delta < pb.fuel_energy_in_delta);
        CHECK(pb.friction_heat_delta == Approx(kH * w * (friction(rpm) - c_k + dF(kK, rpm))).epsilon(1e-12));
        CHECK(pb.friction_heat_delta == -pb.friction_work_delta);
        SimulatedEngine c(desc, maps);
        const PreSolveResult rc = c.pre_solve(sim_sensors(rpm, kK, kK), cmd(0.0), TcuRequest{}, kH); // DFCO
        const PostSolveResult pc = c.post_solve(w, kH);
        CHECK(rc.dfco_active);
        CHECK(pc.released_heat_delta == 0.0);
        CHECK(pc.friction_heat_delta > 0.0);
    }

    SECTION("ECU: the demand model and the idle feed-forward carry k_ecu; no steady grid is ever rebuilt") {
        const real rpm = 3137.0;
        // Both engines on the live path (non-reference ambient), so only k_ecu differs.
        SimulatedEngine a(desc, maps), c(desc, maps);
        const PreSolveResult ra = a.pre_solve(sim_sensors(rpm, 1.0, 1.0, 100000.0), cmd(0.0), TcuRequest{}, kH);
        const PreSolveResult rc = c.pre_solve(sim_sensors(rpm, kK, kK, 100000.0), cmd(0.0), TcuRequest{}, kH);
        // Motoring demand: pumping at the steady manifold pressure minus the increment (Coulomb row unscaled).
        const real expected = -dF(kK, rpm);
        CHECK(std::abs((rc.torque_demand - ra.torque_demand) - expected) <= 1e-9 * std::abs(expected));
        CHECK(a.steady_rebuild_count() == c.steady_rebuild_count());

        // Idle feed-forward grows with k_ecu: the engine held at idle speed for 0.25 s, the actuator settles at the
        // ECU's target (feed-forward plus the manifold-fill compensation).
        SimulatedEngine i1(desc, maps), i2(desc, maps);
        PreSolveResult ri1, ri2;
        for (int n = 0; n < 240; ++n) {
            ri1 = i1.pre_solve(sim_sensors(800.0, 1.0, 1.0), cmd(0.0), TcuRequest{}, kH);
            (void)i1.post_solve(800.0 * kRpmToRad, kH);
            ri2 = i2.pre_solve(sim_sensors(800.0, 1.0, kK), cmd(0.0), TcuRequest{}, kH);
            (void)i2.post_solve(800.0 * kRpmToRad, kH);
        }
        WARN("I1-sim idle plate position: k_ecu 1 -> " << ri1.throttle_eff << ", k_ecu " << kK << " -> " << ri2.throttle_eff);
        CHECK(ri2.throttle_eff > ri1.throttle_eff * 1.02);

        SimulatedEngine g(desc, maps);
        const std::uint64_t rebuilds0 = g.steady_rebuild_count();
        const PreSolveResult g1 = g.pre_solve(sim_sensors(rpm, 1.0, 1.0), cmd(0.0), TcuRequest{}, kH);
        const PreSolveResult g2 = g.pre_solve(sim_sensors(rpm, 1.0, 2.0), cmd(0.0), TcuRequest{}, kH);
        const PreSolveResult g3 = g.pre_solve(sim_sensors(rpm, 1.0, 1.5), cmd(0.0), TcuRequest{}, kH);
        const PreSolveResult g4 = g.pre_solve(sim_sensors(rpm, 1.0, 1.0), cmd(0.0), TcuRequest{}, kH);
        CHECK(g2.torque_demand < g1.torque_demand);
        CHECK(g3.torque_demand < g1.torque_demand);
        CHECK(g3.torque_demand > g2.torque_demand);
        CHECK(g4.torque_demand == g1.torque_demand); // tables again, bit for bit
        CHECK(g.steady_rebuild_count() == rebuilds0);
    }
}
