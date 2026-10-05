// tests/unit/test_engine_thermal_coupling.cpp - engine thermal plan T1: how the oil-viscosity friction scale k and
// the heat bookkeeping act inside the two engine models (out/engine_thermal_plan.md section 6.1 / 12, vault PHYS-063).
// Tags [thermal][drivetrain].
//
// C4 content: I1-graph (EngineTorqueMap, white-box pre_solve/post_solve). C5 adds I1-sim (SimulatedEngine) and I2.
//
// Sabotage record (each applied once, the named check then fails):
//   C4a I1-graph: the friction slope in d not scaled by k_g -> the finite-difference CHECK fails (d 0.0344 vs FD 0.0393,
//       and d(k) == d(1) at theta 0.2).
//   C4b I1-graph: the kinetic Coulomb cap left at c_k -> cap 15.0 vs 35.07 and the net-torque CHECK off by 20.07 N*m.
//   C4c I1-graph: friction heat without the phi_f share -> 20.29 vs the expected -(h*omega*tau_smooth*phi_f).
//   C4d I1-graph: released_heat_delta = 0 -> 0 vs 257.7 J (fuel energy of the substep).
//   C5a I1-sim: friction slope in d not scaled by k -> d vs finite difference off by 0.0345 (tolerance 4e-8).
//   C5b I1-sim: static Coulomb cap unscaled -> 12.0 vs 28.056.
//   C5c I1-sim: the ECU's live steady torque built with k = 1 -> demand error 19.65 N*m, the k_ecu ordering check fails.
//   C5d I1-sim: live idle control ignoring k_ecu -> idle plate 0.02111 instead of 0.03451 (the CHECK needs > 0.02154).
//   C5e I1-sim: released heat = the whole fuel energy (no eta_c) -> 24.818 J vs lambda * fuel energy.
//   C5f I1-sim: the PLANT evaluating with the ECU's k -> tau0 does not fall by (k-1)(F - c_k) (3.39 N*m off).
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

TEST_CASE("I1-graph: friction scale k in EngineTorqueMap (smooth part by k_g, caps by k, slope, heat)", "[thermal][drivetrain]") {
    const TorqueMapEngineDesc tm = graph_params();
    const real phi = 0.6;
    EngineTorqueMap::Desc d = to_engine_desc(tm);
    d.thermal_friction_fraction = phi;
    const real c_k = -d.motoring_torque_nm_vs_rpm.evaluate(0.0);
    REQUIRE(c_k > 0.0);
    const real k_g = 1.0 + phi * (kK - 1.0);

    const real rpm = 3250.0; // mid-segment of both curves (knots every 500 rpm): finite differences stay on one segment
    const real m = d.motoring_torque_nm_vs_rpm.evaluate(rpm);
    const real m_smooth = m + c_k; // the friction part of the motoring curve (<= 0)
    REQUIRE(m_smooth < 0.0);

    SECTION("net torque: only the smooth friction and the Coulomb caps change, the indicated torque w - m does not") {
        for (const real theta : {1.0, 0.5}) {
            INFO("theta " << theta);
            const Eval a = eval_graph(d, rpm, theta, 1.0);
            const Eval b = eval_graph(d, rpm, theta, kK);
            // Smooth friction acts on the node torque: tau0 differs by exactly (k_g - 1)*m_smooth.
            const real expected_dtau0 = (k_g - 1.0) * m_smooth;
            CHECK(std::abs((b.pre.tau0 - a.pre.tau0) - expected_dtau0) <= 1e-12 * std::abs(expected_dtau0));
            // Coulomb caps scale with k (the full friction at rest is pure Coulomb).
            CHECK(a.pre.friction_cap_kinetic == c_k);
            CHECK(a.pre.friction_cap_static == d.breakaway_torque_nm);
            CHECK(b.pre.friction_cap_kinetic == Approx(c_k * kK).epsilon(1e-14));
            CHECK(b.pre.friction_cap_static == Approx(d.breakaway_torque_nm * kK).epsilon(1e-14));
            // Net engine torque including the Coulomb row's kinetic value: falls by (k_g-1)*m_smooth - (k-1)*c_k.
            const real net_a = a.pre.tau0 - a.pre.friction_cap_kinetic;
            const real net_b = b.pre.tau0 - b.pre.friction_cap_kinetic;
            const real expected_dnet = (k_g - 1.0) * m_smooth - (kK - 1.0) * c_k;
            CHECK(std::abs((net_b - net_a) - expected_dnet) <= 1e-12 * std::abs(expected_dnet));
            // The ECU-visible torque demand of the graph engine uses the plant k (it has no separate ECU model).
            const real expected_dtd = (k_g - 1.0) * m_smooth - (kK - 1.0) * c_k;
            CHECK(std::abs((b.pre.torque_demand - a.pre.torque_demand) - expected_dtd) <= 1e-12 * std::abs(expected_dtd));
        }
        // WOT in particular: the indicated torque w - m is untouched (compare against the curves directly).
        const Eval w = eval_graph(d, rpm, 1.0, kK);
        const real w_rpm = d.wot_torque_nm_vs_rpm.evaluate(rpm);
        CHECK(std::abs(w.pre.tau0 - (k_g * m_smooth + (w_rpm - m))) <= 1e-12 * std::abs(w_rpm));
    }

    SECTION("d matches a central finite difference of tau0 (the friction slope carries k_g, the combustion slope does not)") {
        for (const real k : {1.0, kK}) {
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
        // The scale is not a no-op for d: the friction slope of this motoring curve is nonzero here.
        const Eval a = eval_graph(d, rpm, 0.2, 1.0);
        const Eval b = eval_graph(d, rpm, 0.2, kK);
        CHECK(a.pre.d > 0.0);
        CHECK(std::abs(b.pre.d - a.pre.d) > 1e-6);
    }

    SECTION("heat: released = fuel energy (eta_c = 1), friction heat = -h*omega*tau_smooth*phi_f with tau_smooth carrying k_g") {
        REQUIRE(d.has_fuel_model);
        const Eval e = eval_graph(d, rpm, 1.0, kK);
        const real omega = rpm * kRpmToRad;
        const real tau_smooth = (m_smooth)*k_g; // s = +1
        CHECK(e.post.friction_heat_delta == Approx(-(kH * omega * tau_smooth * phi)).epsilon(1e-12));
        CHECK(e.post.friction_heat_delta > 0.0);
        CHECK(e.post.released_heat_delta == e.post.fuel_energy_in_delta);
        CHECK(e.post.released_heat_delta > 0.0);
        // Coasting (no fuel): friction heat still flows, nothing is released.
        const Eval c = eval_graph(d, rpm, 0.0, kK);
        CHECK(c.post.released_heat_delta == 0.0);
        CHECK(c.post.friction_heat_delta > 0.0);
    }

    SECTION("k = 1.0 leaves every output bit-identical to the unscaled expressions") {
        const Eval a = eval_graph(d, rpm, 0.7, 1.0);
        EngineTorqueMap::Desc d1 = d;
        d1.thermal_friction_fraction = 1.0; // k_g = 1 whatever phi_f is
        const Eval b = eval_graph(d1, rpm, 0.7, 1.0);
        CHECK(a.pre.tau0 == b.pre.tau0);
        CHECK(a.pre.d == b.pre.d);
        CHECK(a.pre.torque_demand == b.pre.torque_demand);
        // and against the closed forms of the pre-thermal code
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

    SECTION("plant: tau0 falls by (k-1)(F - c_k), caps = k*c_k and k*breakaway, d vs central finite difference") {
        for (const real rpm : {1234.0, 3137.0, 6021.0}) {
            INFO("rpm " << rpm);
            const real w = rpm * kRpmToRad;
            SimulatedEngine a(desc, maps), b(desc, maps);
            const PreSolveResult ra = a.pre_solve(sim_sensors(rpm, 1.0, 1.0), cmd(1.0), TcuRequest{}, kH);
            const PreSolveResult rb = b.pre_solve(sim_sensors(rpm, kK, 1.0), cmd(1.0), TcuRequest{}, kH); // ECU view unchanged
            const real expected = -(kK - 1.0) * (friction(rpm) - c_k);
            REQUIRE(std::abs(expected) > 1.0);
            CHECK(std::abs((rb.tau0 - ra.tau0) - expected) <= 1e-12 * std::abs(expected) + 1e-13);
            CHECK(ra.friction_cap_kinetic == c_k);
            CHECK(ra.friction_cap_static == desc.breakaway_torque_nm);
            CHECK(rb.friction_cap_kinetic == Approx(c_k * kK).epsilon(1e-14));
            CHECK(rb.friction_cap_static == Approx(desc.breakaway_torque_nm * kK).epsilon(1e-14));
            // The ECU's demand does not see the plant's extra friction.
            CHECK(rb.torque_demand == ra.torque_demand);

            // d: analytic dtau/domega at frozen p_im against a central difference, k = kK; and its k-increment.
            const real p = rb.p_intake_pa;
            const real dw = 1e-3;
            const auto ev = b.evaluate_torque(w, p, true, 1.0, false);
            const real fd = (b.evaluate_torque(w + dw, p, true, 1.0, false).tau - b.evaluate_torque(w - dw, p, true, 1.0, false).tau) / (2.0 * dw);
            CHECK(std::abs(ev.dtau_domega - fd) <= 1e-6 * std::abs(fd));
            const auto eva = a.evaluate_torque(w, p, true, 1.0, false);
            const real slope_f = (friction(rpm + 0.5) - friction(rpm - 0.5)); // per rpm, same segment
            const real expected_dd = -(kK - 1.0) * (60.0 / (2.0 * kPi)) * slope_f;
            REQUIRE(std::abs(expected_dd) > 1e-3);
            CHECK(std::abs((ev.dtau_domega - eva.dtau_domega) - expected_dd) <= 1e-9 * std::abs(expected_dd));
        }
    }

    SECTION("heat: released = min(1, lambda) x fuel energy; friction heat = h omega k (F - c_k)") {
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
        CHECK(pb.friction_heat_delta == Approx(kH * w * kK * (friction(rpm) - c_k)).epsilon(1e-12));
        CHECK(pb.friction_heat_delta == -pb.friction_work_delta);
        // A rich point (lambda > 1 would be capped): eta_c = 1 is the cap. Coasting releases nothing.
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
        // Motoring demand: pumping at the steady manifold pressure minus k*F (the Coulomb row included).
        const real expected = -(kK - 1.0) * friction(rpm);
        CHECK(std::abs((rc.torque_demand - ra.torque_demand) - expected) <= 1e-9 * std::abs(expected));
        CHECK(a.steady_rebuild_count() == c.steady_rebuild_count());

        // Idle feed-forward grows with k_ecu: the engine held at idle speed for 0.25 s, the actuator settles at the
        // ECU's target (feed-forward plus the manifold-fill compensation). A k_ecu-blind idle control would sit at
        // the k = 1 position (the old live/table difference is ~1e-7, far below this margin).
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

        // Changing k_ecu back and forth never rebuilds the grid, and returning to exactly 1.0 returns to the tables.
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
