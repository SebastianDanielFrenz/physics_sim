// tests/unit/test_powertrain_compliance.cpp - PHYS-033 P4 I4 C13: compliant
// drivetrain shafts (PowertrainDesc::compliance == Compliant, ShaftDesc::
// stiffness/damping). White-box (core/src on the include path): drives a
// Powertrain substep by substep like test_powertrain_diff.cpp's Driver, plus
// whole-vehicle World cases (zero-alloc, launch smoke, worker-count hash).
//
// Scheme under test (powertrain.h's CompliantShaft comment): per shaft,
// twist theta = integral(omega_a - omega_b), torque tau_s = -tau_s on a,
// +tau_s on b. Trapezoidal by default (c_impl = c/2 + k*h/4, tau_expl =
// k*theta + c_impl*v_old, theta' = theta + h*(v_old + v_new)/2 - for c = 0
// this IS the implicit midpoint rule, so KE + PE is conserved exactly and
// the discrete frequency is (2/h)*atan(omega_n*h/2)); backward Euler on the
// first substep after a mode change (topology switch or friction-row stick
// change in the previous substep, or a reset()).
//
// The analytic oscillator is two free wheel hubs (no components, no brakes)
// joined by one compliant shaft: omega_n^2 = k*mu, 2*zeta*omega_n = c*mu,
// mu = 1/J1 + 1/J2. Built from car_sedan.json's wheel list with the
// powertrain replaced in code (no data file changes).

#include "drivetrain/components.h"
#include "drivetrain/network.h"
#include "drivetrain/powertrain.h"

#include "powertrain_multi_diff_desc.h"
#include "vehicle_test_helpers.h"

#include "ps/io/surface_table.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

constexpr real kPi = 3.14159265358979323846;
constexpr real kH = 1.0 / 960.0;

Powertrain build(const vehicle::VehicleDesc& d) { return Powertrain(d.powertrain, d.brake_system, d.wheels, d.controllers, d.name); }

// car_sedan.json's wheels only: no components, no brakes, no controllers.
// Optionally overrides FL/FR's spin inertia (hub node inertia).
vehicle::VehicleDesc bare_hubs_desc(real j_fl = 0.0, real j_fr = 0.0) {
    vehicle::VehicleDesc d = ps_test::load_sedan_desc();
    d.powertrain = PowertrainDesc{};
    d.brake_system = BrakeSystemDesc{};
    d.controllers.clear();
    if (j_fl > 0.0) d.wheels[0].wheel_inertia = j_fl;
    if (j_fr > 0.0) d.wheels[1].wheel_inertia = j_fr;
    d.name = "compliance_two_hub_test";
    return d;
}

// Two free hubs FL/FR joined by one compliant shaft (ports[0] = FL = a).
vehicle::VehicleDesc two_hub_desc(real k, real c, real j_fl = 0.0, real j_fr = 0.0) {
    vehicle::VehicleDesc d = bare_hubs_desc(j_fl, j_fr);
    d.powertrain.compliance = Compliance::Compliant;
    ShaftDesc s = ps_test::shaft(ps_test::wheel_port("FL"), ps_test::wheel_port("FR"));
    s.stiffness = k;
    s.damping = c;
    d.powertrain.shafts.push_back(s);
    return d;
}

// car_sedan.json with shafts[idx] compliant (k, c). shafts[1] is
// clutch.out - gearbox.in.
vehicle::VehicleDesc compliant_sedan(std::size_t idx = 1, real k = 600.0, real c = 2.0) {
    vehicle::VehicleDesc d = ps_test::load_sedan_desc();
    d.powertrain.compliance = Compliance::Compliant;
    d.powertrain.shafts.at(idx).stiffness = k;
    d.powertrain.shafts.at(idx).damping = c;
    return d;
}

struct Driver {
    Powertrain& pt;
    ComponentSubstepCtx ctx;
    long substep = 0;
    int substeps_per_tick = 4;

    explicit Driver(Powertrain& p) : pt(p) {
        ctx.h = kH;
        ctx.ignition = 0.0;
        ctx.assist_auto_clutch = false;
        ctx.assist_auto_blip = false;
        ctx.assist_auto_shift = false;
    }
    void step(const std::vector<real>& hub_tau = {}, const std::vector<real>& hub_d = {}) {
        ctx.first_substep_of_tick = (substep % substeps_per_tick) == 0;
        pt.begin_substep();
        pt.pre_solve_components(ctx);
        for (std::size_t i = 0; i < hub_tau.size(); ++i) pt.add_hub_terms(i, hub_tau[i], i < hub_d.size() ? hub_d[i] : real(0.0));
        pt.solve(ctx.h);
        pt.post_solve_components(ctx);
        ++substep;
    }
};

void reset_neutral_engine_off(Powertrain& pt, const std::vector<real>& hubs) {
    vehicle::VehicleResetOptions opts;
    opts.gear = 0;
    opts.engine_state = EngineState::Off;
    opts.clutch_locked = false;
    pt.reset(opts, hubs);
}

real twist_rate(const Powertrain& pt) {
    const auto& cs = pt.compliant_shafts().at(0);
    return pt.network().omega(cs.a) - pt.network().omega(cs.b);
}

// KE of every node + PE of every compliant shaft, computed from state (not
// from the ledger).
real mechanical_energy(const Powertrain& pt) {
    real e = 0.0;
    for (int i = 0; i < pt.network().node_count(); ++i) {
        const NodeId nid{static_cast<std::uint16_t>(i)};
        const real w = pt.network().omega(nid);
        e += real(0.5) * pt.network().node_inertia(nid) * w * w;
    }
    for (const auto& cs : pt.compliant_shafts()) e += real(0.5) * cs.k * cs.theta * cs.theta;
    return e;
}

std::vector<std::uint8_t> stuck_flags(const Powertrain& pt) {
    std::vector<std::uint8_t> out(static_cast<std::size_t>(pt.network().friction_row_count()));
    for (int r = 0; r < pt.network().friction_row_count(); ++r) {
        out[static_cast<std::size_t>(r)] = pt.network().row_stuck(FrictionRowId{static_cast<std::uint16_t>(r)}) ? 1 : 0;
    }
    return out;
}

// Downward zero crossings of a sampled signal, linearly interpolated.
struct CrossingTracker {
    std::vector<real> t;
    real prev = 0.0;
    bool has_prev = false;
    void feed(real time, real v) {
        if (has_prev && prev > 0.0 && v <= 0.0) t.push_back(time - kH + kH * prev / (prev - v));
        prev = v;
        has_prev = true;
    }
    [[nodiscard]] real mean_period() const { return (t.back() - t.front()) / static_cast<real>(t.size() - 1); }
};

real sum_abs_terms(const PowertrainEnergy& e) {
    return std::abs(e.engine_out) + std::abs(e.wheel_out) + std::abs(e.tyre_loss) + std::abs(e.brake_loss) + std::abs(e.gear_loss) +
           std::abs(e.integrator_loss) + std::abs(e.clutch_loss) + std::abs(e.synchro_loss) + std::abs(e.engine_friction_loss) +
           std::abs(e.engagement_loss) + std::abs(e.starter_in) + std::abs(e.external_reset) + std::abs(e.diff_loss) +
           std::abs(e.shaft_pe) + std::abs(e.shaft_damping_loss) + std::abs(e.rotor_ke);
}

} // namespace

TEST_CASE("compliant shaft, undamped two-inertia: period matches 2*pi/omega_n and the trapezoidal discrete frequency",
          "[unit][drivetrain][compliance]") {
    struct Case {
        const char* label;
        real j1, j2; // 0 = car_sedan's own wheel inertia
        real k;      // 0 = choose k for omega_n = 50 rad/s
    };
    // (a) omega_n = 50 rad/s on the sedan's own hub inertias (omega_n*h = 0.052):
    //     the spec's 0.5% bound against the continuous period.
    // (b) the sedan clutch.out/gearbox.in pair (J 0.004/0.010, k 600:
    //     omega_n = 458 rad/s, omega_n*h = 0.477) - the trapezoidal rule's
    //     own frequency warping, (2/h)*atan(omega_n*h/2), is ~1.9% here, so
    //     this case is checked against the DISCRETE frequency (and the
    //     continuous error is printed, not bounded).
    for (const Case& cse : {Case{"omega_n 50 on sedan hubs", 0.0, 0.0, 0.0}, Case{"sedan clutch pair J 0.004/0.010 k 600", 0.004, 0.010, 600.0}}) {
        INFO(cse.label);
        Powertrain probe = build(bare_hubs_desc(cse.j1, cse.j2));
        const real j1 = probe.hub_inertia(0), j2 = probe.hub_inertia(1);
        const real mu = 1.0 / j1 + 1.0 / j2;
        const real k = cse.k > 0.0 ? cse.k : 50.0 * 50.0 / mu;
        const real wn = std::sqrt(k * mu);
        const real w_eff = (2.0 / kH) * std::atan(wn * kH / 2.0);

        Powertrain pt = build(two_hub_desc(k, 0.0, cse.j1, cse.j2));
        REQUIRE(pt.has_compliant_shafts());
        reset_neutral_engine_off(pt, {1.0, 0.0, 0.0, 0.0});
        Driver drv(pt);
        CrossingTracker ct;
        ct.feed(0.0, twist_rate(pt));
        const long n = static_cast<long>(std::ceil(12.0 * (2.0 * kPi / wn) / kH));
        for (long s = 1; s <= n; ++s) {
            drv.step();
            ct.feed(static_cast<real>(s) * kH, twist_rate(pt));
        }
        REQUIRE(ct.t.size() >= 10);
        const real T = ct.mean_period();
        const real err_cont = T / (2.0 * kPi / wn) - 1.0;
        const real err_disc = T / (2.0 * kPi / w_eff) - 1.0;
        std::printf("[compliance period] %-40s omega_n=%.4f omega_n*h=%.4f T=%.9f err_vs_2pi/omega_n=%.4e err_vs_discrete=%.4e\n",
                    cse.label, static_cast<double>(wn), static_cast<double>(wn * kH), static_cast<double>(T),
                    static_cast<double>(err_cont), static_cast<double>(err_disc));
        INFO("err_cont=" << err_cont << " err_disc=" << err_disc);
        CHECK(std::abs(err_disc) <= 1e-4);
        if (cse.k == 0.0) CHECK(std::abs(err_cont) <= 0.005);
    }
}

TEST_CASE("compliant shaft, undamped: KE+PE drift <= 1e-9 relative over 10^4 trapezoidal substeps", "[unit][drivetrain][compliance]") {
    for (const real jj : {0.0, 0.004}) {
        INFO("j override " << jj);
        Powertrain probe = build(bare_hubs_desc(jj, jj > 0.0 ? 0.010 : 0.0));
        const real mu = 1.0 / probe.hub_inertia(0) + 1.0 / probe.hub_inertia(1);
        const real k = jj > 0.0 ? 600.0 : 2500.0 / mu;
        Powertrain pt = build(two_hub_desc(k, 0.0, jj, jj > 0.0 ? 0.010 : 0.0));
        reset_neutral_engine_off(pt, {3.0, -1.0, 0.0, 0.0});
        Driver drv(pt);
        drv.step(); // substep 1 is backward Euler (after the reset) - the reference starts after it
        REQUIRE(pt.compliant_be_last());
        const real e0 = mechanical_energy(pt);
        const real integ0 = pt.ledger().integrator_loss;
        real max_drift = 0.0;
        for (int s = 0; s < 10000; ++s) {
            drv.step();
            REQUIRE_FALSE(pt.compliant_be_last());
            max_drift = std::max(max_drift, std::abs(mechanical_energy(pt) - e0) / e0);
            // The ledger's stored terms are the same quantity.
            REQUIRE((pt.ledger().rotor_ke + pt.ledger().shaft_pe) == Catch::Approx(mechanical_energy(pt)).epsilon(1e-12));
        }
        const real integ = pt.ledger().integrator_loss - integ0;
        std::printf("[compliance drift] k=%.3f max |dE|/E=%.3e  integrator_loss over 1e4 substeps=%.3e J (E=%.6f J)\n",
                    static_cast<double>(k), static_cast<double>(max_drift), static_cast<double>(integ), static_cast<double>(e0));
        CHECK(max_drift <= 1e-9);
        // Implicit midpoint: the network's own 1/2*J*dw^2 and the spring
        // residual (h/2)*tau_s*(v_new - v_old) cancel exactly per substep.
        CHECK(std::abs(integ) <= 1e-9 * e0);
        CHECK(pt.ledger().shaft_damping_loss == 0.0);
    }
}

TEST_CASE("compliant shaft, damped: logarithmic decrement matches the analytic zeta within 2%", "[unit][drivetrain][compliance]") {
    for (const real zeta : {0.02, 0.1}) {
        INFO("zeta " << zeta);
        Powertrain probe = build(bare_hubs_desc());
        const real mu = 1.0 / probe.hub_inertia(0) + 1.0 / probe.hub_inertia(1);
        const real wn = 50.0;
        const real k = wn * wn / mu;
        const real c = 2.0 * zeta * wn / mu;
        Powertrain pt = build(two_hub_desc(k, c));
        reset_neutral_engine_off(pt, {1.0, 0.0, 0.0, 0.0});
        Driver drv(pt);

        // Peaks of the twist rate, each refined by a parabola through the
        // three samples around it.
        std::vector<real> peaks;
        real vm2 = twist_rate(pt);
        drv.step();
        real vm1 = twist_rate(pt);
        const long n = static_cast<long>(std::ceil(8.0 * (2.0 * kPi / wn) / kH));
        for (long s = 2; s <= n; ++s) {
            drv.step();
            const real v = twist_rate(pt);
            if (vm1 > vm2 && vm1 >= v && vm1 > 0.0) {
                const real denom = vm2 - 2.0 * vm1 + v;
                const real off = denom != 0.0 ? 0.5 * (vm2 - v) / denom : 0.0;
                peaks.push_back(vm1 - 0.25 * (vm2 - v) * off);
            }
            vm2 = vm1;
            vm1 = v;
        }
        REQUIRE(peaks.size() >= 6);
        const std::size_t m = peaks.size() - 1;
        const real delta = std::log(peaks.front() / peaks.back()) / static_cast<real>(m);
        const real zeta_meas = delta / std::sqrt(4.0 * kPi * kPi + delta * delta);
        const real err = zeta_meas / zeta - 1.0;
        std::printf("[compliance damping] zeta=%.3f measured=%.6f err=%.4e over %zu cycles, damping_loss=%.6e J\n", static_cast<double>(zeta),
                    static_cast<double>(zeta_meas), static_cast<double>(err), m, static_cast<double>(pt.ledger().shaft_damping_loss));
        INFO("zeta_meas=" << zeta_meas);
        CHECK(std::abs(err) <= 0.02);
        CHECK(pt.ledger().shaft_damping_loss > 0.0);
    }
}

TEST_CASE("compliant shaft: backward Euler after a reset, then trapezoidal; reset relaxes the twist",
          "[unit][drivetrain][compliance]") {
    Powertrain probe = build(bare_hubs_desc());
    const real mu = 1.0 / probe.hub_inertia(0) + 1.0 / probe.hub_inertia(1);
    const real k = 2500.0 / mu;
    Powertrain pt = build(two_hub_desc(k, 0.0));
    CHECK(pt.compliant_be_next()); // a fresh Powertrain starts in backward Euler too
    reset_neutral_engine_off(pt, {1.0, 0.0, 0.0, 0.0});
    REQUIRE(pt.compliant_be_next());
    Driver drv(pt);

    // Discriminating measurement: an undamped backward-Euler substep loses
    // KE+PE (the network's 1/2*dw^T*J*dw plus the spring's 1/2*k*h^2*v_new^2,
    // both > 0), the trapezoidal/midpoint step conserves it exactly.
    const real e_start = mechanical_energy(pt);
    drv.step();
    CHECK(pt.compliant_be_last());
    CHECK_FALSE(pt.compliant_be_next());
    const real e_be = mechanical_energy(pt);
    CHECK(e_be < e_start * (1.0 - 1e-6));
    drv.step();
    CHECK_FALSE(pt.compliant_be_last());
    CHECK(std::abs(mechanical_energy(pt) - e_be) <= 1e-12 * e_be);

    for (int s = 0; s < 30; ++s) drv.step();
    REQUIRE(std::abs(pt.compliant_shafts()[0].theta) > 1e-4);
    reset_neutral_engine_off(pt, {2.0, 2.0, 0.0, 0.0});
    CHECK(pt.compliant_shafts()[0].theta == 0.0);
    CHECK(pt.compliant_be_next());
    drv.step();
    CHECK(pt.compliant_be_last());
    drv.step();
    CHECK_FALSE(pt.compliant_be_last());
}

TEST_CASE("compliant shaft: backward Euler exactly on the substep after a friction-row stick change",
          "[unit][drivetrain][compliance]") {
    // Two hubs + a service brake on FR: the brake row slips while FR spins
    // down, sticks, and the spring may unstick it again - every stick change
    // must make the NEXT substep backward Euler, and only those (no topology
    // switches exist here).
    Powertrain probe = build(bare_hubs_desc());
    const real mu = 1.0 / probe.hub_inertia(0) + 1.0 / probe.hub_inertia(1);
    const real k = 2500.0 / mu;
    vehicle::VehicleDesc d = two_hub_desc(k, 0.5);
    d.brake_system.corners.push_back(BrakeCornerDesc{"FR", 50.0});
    Powertrain pt = build(d);
    reset_neutral_engine_off(pt, {2.0, 2.0, 0.0, 0.0});
    Driver drv(pt);
    drv.ctx.brake = 1.0;

    std::vector<std::uint8_t> prev = stuck_flags(pt);
    bool expect_be = true; // after the reset
    int changes = 0, be_count = 0, trap_after_change = 0;
    bool seen_change = false;
    for (int s = 0; s < 2000; ++s) {
        drv.step();
        INFO("substep " << s);
        REQUIRE(pt.compliant_be_last() == expect_be);
        if (pt.compliant_be_last()) ++be_count;
        else if (seen_change) ++trap_after_change;
        const std::vector<std::uint8_t> now = stuck_flags(pt);
        const bool changed = now != prev;
        if (changed) {
            ++changes;
            seen_change = true;
        }
        REQUIRE(pt.compliant_be_next() == changed);
        prev = now;
        expect_be = changed;
    }
    std::printf("[compliance stick] stick changes=%d backward-Euler substeps=%d trapezoidal after first change=%d\n", changes, be_count,
                trap_after_change);
    CHECK(changes >= 1);
    CHECK(trap_after_change > 0);
}

TEST_CASE("compliant shaft: backward Euler on the substep after a topology switch (sedan N -> 1st)", "[unit][drivetrain][compliance]") {
    Powertrain pt = build(compliant_sedan(1));
    REQUIRE(pt.has_compliant_shafts());
    reset_neutral_engine_off(pt, {0.0, 0.0, 0.0, 0.0});
    Driver drv(pt);
    drv.ctx.clutch_pedal = 1.0; // clutch fully open
    drv.ctx.gear_request = 1.0;

    int topo_prev = pt.network().topology_state();
    std::vector<std::uint8_t> stuck_prev = stuck_flags(pt);
    bool expect_be = true;
    int switches = 0;
    for (int s = 0; s < 4000; ++s) {
        drv.step();
        INFO("substep " << s);
        REQUIRE(pt.compliant_be_last() == expect_be);
        const int topo = pt.network().topology_state();
        const std::vector<std::uint8_t> stuck = stuck_flags(pt);
        const bool switched = topo != topo_prev;
        if (switched) ++switches;
        // A topology switch clears every stick flag before the solve; a flag
        // comparison across it is meaningless, and the switch alone forces
        // backward Euler anyway.
        expect_be = switched || stuck != stuck_prev;
        REQUIRE(pt.compliant_be_next() == expect_be);
        if (switched) CHECK(pt.compliant_be_next());
        topo_prev = topo;
        stuck_prev = stuck;
    }
    std::printf("[compliance topology] switches=%d final gear state=%d\n", switches, pt.network().topology_state());
    CHECK(switches >= 1);
}

TEST_CASE("compliant sedan: reset leaves every compliant shaft relaxed (omega_a == omega_b, theta 0)", "[unit][drivetrain][compliance]") {
    using Catch::Matchers::ContainsSubstring;
    for (const std::size_t idx : {std::size_t{0}, std::size_t{1}, std::size_t{2}}) {
        INFO("shafts[" << idx << "]");
        vehicle::VehicleDesc d = compliant_sedan(idx);
        if (idx == 0) {
            // engine.crank - clutch.in: clutch.in has no inertia in car_sedan.json.
            for (auto& c : d.powertrain.components) {
                if (auto* cl = std::get_if<ClutchDesc>(&c.params)) cl->in_inertia_kgm2 = 0.002;
            }
        }
        Powertrain pt = build(d);
        Driver drv(pt);
        drv.ctx.throttle = 0.5;
        drv.ctx.ignition = 1.0;
        for (const bool in_gear : {true, false}) {
            vehicle::VehicleResetOptions opts;
            opts.gear = in_gear ? 1 : 0;
            opts.engine_state = EngineState::Running;
            opts.clutch_locked = true;
            pt.reset(opts, {40.0, 40.0, 40.0, 40.0});
            const auto& cs = pt.compliant_shafts().at(0);
            CHECK(cs.theta == 0.0);
            CHECK(pt.network().omega(cs.a) == pt.network().omega(cs.b));
            CHECK(pt.compliant_be_next());
            for (int s = 0; s < 200; ++s) drv.step();
            CHECK(std::abs(pt.compliant_shafts().at(0).theta) > 0.0);
        }
    }
    SECTION("a compliant shaft with a zero-inertia endpoint is rejected at build") {
        vehicle::VehicleDesc d = compliant_sedan(0); // clutch.in inertia 0
        REQUIRE_THROWS_WITH(build(d), ContainsSubstring("has zero inertia"));
        vehicle::VehicleDesc d3 = compliant_sedan(3); // diff.left inertia 0
        REQUIRE_THROWS_WITH(build(d3), ContainsSubstring("has zero inertia"));
    }
}

TEST_CASE("compliant shaft: twist is hashed (with the backward-Euler flag); a Rigid vehicle's hash ignores stiffness",
          "[unit][drivetrain][compliance]") {
    Powertrain probe = build(bare_hubs_desc());
    const real mu = 1.0 / probe.hub_inertia(0) + 1.0 / probe.hub_inertia(1);
    const real k = 2500.0 / mu;
    const std::uint64_t seed = 0xcbf29ce484222325ull;

    // A: at rest, one substep (theta stays 0, be_next false).
    Powertrain a = build(two_hub_desc(k, 0.0));
    reset_neutral_engine_off(a, {0.0, 0.0, 0.0, 0.0});
    const std::uint64_t h_a_reset = a.fold_state_hash(seed);
    Driver da(a);
    da.step();
    REQUIRE(a.compliant_shafts()[0].theta == 0.0);
    const std::uint64_t h_a = a.fold_state_hash(seed);
    CHECK(h_a != h_a_reset); // only be_next differs (true after reset, false now)

    // A': identical replica -> identical hash.
    Powertrain a2 = build(two_hub_desc(k, 0.0));
    reset_neutral_engine_off(a2, {0.0, 0.0, 0.0, 0.0});
    Driver da2(a2);
    da2.step();
    CHECK(a2.fold_state_hash(seed) == h_a);

    // B: twisted, then every omega brought back to 0 - differs from A only in theta.
    Powertrain b = build(two_hub_desc(k, 0.0));
    reset_neutral_engine_off(b, {1.0, 0.0, 0.0, 0.0});
    Driver db(b);
    for (int s = 0; s < 7; ++s) db.step();
    REQUIRE(b.compliant_shafts()[0].theta != 0.0);
    REQUIRE_FALSE(b.compliant_be_next());
    for (std::size_t i = 0; i < 4; ++i) b.seed_hub_omega(i, 0.0);
    CHECK(b.fold_state_hash(seed) != h_a);

    // Rigid: stiffness/damping on a shaft are ignored entirely - same hash,
    // no compliant shaft, no shaft ledger terms, as the unmodified sedan.
    vehicle::VehicleDesc rigid_stiff = ps_test::load_sedan_desc();
    REQUIRE(rigid_stiff.powertrain.compliance == Compliance::Rigid);
    rigid_stiff.powertrain.shafts.at(1).stiffness = 600.0;
    rigid_stiff.powertrain.shafts.at(1).damping = 2.0;
    Powertrain r0 = build(ps_test::load_sedan_desc());
    Powertrain r1 = build(rigid_stiff);
    CHECK_FALSE(r1.has_compliant_shafts());
    CHECK(r1.network().node_count() == r0.network().node_count());
    CHECK(r1.network().coupling_count() == r0.network().coupling_count());
    for (Powertrain* p : {&r0, &r1}) {
        vehicle::VehicleResetOptions opts;
        opts.gear = 1;
        opts.engine_state = EngineState::Running;
        opts.clutch_locked = true;
        p->reset(opts, {30.0, 30.0, 30.0, 30.0});
    }
    Driver d0(r0), d1(r1);
    for (Driver* dd : {&d0, &d1}) {
        dd->ctx.ignition = 1.0;
        dd->ctx.throttle = 0.6;
    }
    for (int s = 0; s < 400; ++s) {
        d0.step({-40.0, -40.0, 0.0, 0.0}, {5.0, 5.0, 0.0, 0.0});
        d1.step({-40.0, -40.0, 0.0, 0.0}, {5.0, 5.0, 0.0, 0.0});
    }
    CHECK(r1.fold_state_hash(seed) == r0.fold_state_hash(seed));
    CHECK(r1.ledger().shaft_pe == 0.0);
    CHECK(r1.ledger().shaft_damping_loss == 0.0);
}

TEST_CASE("compliant sedan in gear: the whole-Powertrain ledger closes per tick", "[unit][drivetrain][compliance]") {
    for (const real c : {0.0, 2.0}) {
        INFO("damping " << c);
        Powertrain pt = build(compliant_sedan(1, 600.0, c));
        vehicle::VehicleResetOptions opts;
        opts.gear = 1;
        opts.engine_state = EngineState::Running;
        opts.clutch_locked = true;
        pt.reset(opts, {20.0, 20.0, 20.0, 20.0});
        Driver drv(pt);
        drv.ctx.ignition = 1.0;
        const real h = drv.ctx.h;
        real max_ratio = 0.0;
        real max_twist = 0.0;
        for (int tick = 0; tick < 480; ++tick) {
            drv.ctx.throttle = tick < 240 ? 0.8 : 0.0; // drive, then overrun (torque reversal through the shaft)
            pt.begin_tick();
            for (int s = 0; s < 4; ++s) {
                // A linear road load on the driven hubs.
                const std::vector<real> tau = {-60.0, -60.0, 0.0, 0.0};
                const std::vector<real> dd = {4.0, 4.0, 0.0, 0.0};
                const std::vector<real> w_old = {pt.hub_omega(0), pt.hub_omega(1)};
                drv.step(tau, dd);
                real wheel_out = 0.0;
                for (std::size_t i = 0; i < 2; ++i) {
                    const real w_new = pt.hub_omega(i);
                    wheel_out += -h * w_new * (tau[i] - dd[i] * (w_new - w_old[i]));
                }
                pt.add_wheel_energy(wheel_out, 0.0);
            }
            pt.end_tick();
            const PowertrainEnergy& e = pt.ledger();
            if (tick > 0) max_ratio = std::max(max_ratio, std::abs(e.balance_error) / sum_abs_terms(e));
            max_twist = std::max(max_twist, std::abs(pt.compliant_shafts()[0].theta));
        }
        std::printf("[compliance ledger] c=%.1f max |balance_error|/scale=%.3e max|theta|=%.4f rad damping_loss=%.4e J\n",
                    static_cast<double>(c), static_cast<double>(max_ratio), static_cast<double>(max_twist),
                    static_cast<double>(pt.ledger().shaft_damping_loss));
        INFO("max_ratio=" << max_ratio);
        CHECK(max_ratio <= 1e-12);
        CHECK(max_twist > 1e-3); // the shaft actually carries torque
        if (c > 0.0) CHECK(pt.ledger().shaft_damping_loss > 0.0);
        else CHECK(pt.ledger().shaft_damping_loss == 0.0);
    }
}

TEST_CASE("a vehicle with a compliant shaft allocates nothing while stepping",
          "[unit][vehicle][world][drivetrain][compliance][debug_alloc][jobs]") {
    if (!ps::jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc's own operator new/delete overrides are compiled out under ASan/TSan - nothing to check here on this build.");
        return;
    }
    WorldConfig cfg;
    cfg.job_workers = 1;
    cfg.tick_rate_hz = 240.0;
    cfg.substep_rate_hz = 960.0;
    World world(cfg);
    BodyDesc ground;
    ground.shape = BoxShape{Vec3{1000.0, 1000.0, 0.5}};
    ground.motion = BodyMotionType::Static;
    ground.friction = 1.0;
    ground.pose.position = Vec3{0.0, 0.0, -0.5};
    world.create_body(ground);
    BodyDesc chassis;
    chassis.shape = BoxShape{Vec3{2.0, 0.4, 0.15}};
    chassis.motion = BodyMotionType::Dynamic;
    chassis.mass = 1500.0;
    chassis.friction = 0.5;
    chassis.allow_sleep = false;
    chassis.pose.position = Vec3{0.0, 0.0, 0.6};
    const BodyId chassis_id = world.create_body(chassis);
    world.set_surface_table(std::make_shared<io::SurfaceTable>(std::string(PS_SOURCE_DIR) + "/data/surfaces/surfaces.json"));

    const VehicleId v = world.create_vehicle(compliant_sedan(1, 600.0, 2.0), chassis_id);
    world.set_control("gear_request", 1.0);
    world.set_control("throttle", 0.4);
    world.set_control("steer", 0.05);
    for (int i = 0; i < 300; ++i) world.step();

    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    for (int i = 0; i < 120; ++i) world.step();
    ps::jobs::debug_alloc::set_armed(false);

    const std::uint64_t own = ps::jobs::debug_alloc::own_violation_count();
    const std::uint64_t external = ps::jobs::debug_alloc::external_violation_count();
    ps::jobs::debug_alloc::reset();

    WARN("external (Jolt-internal, ungated, PLAN.md P9) violation count=" << external);
    REQUIRE(own == 0);
    CHECK(world.powertrain_debug(v).shaft_twist_rad[0] != 0.0); // the shaft is actually loaded
    CHECK(world.powertrain_energy(v).shaft_damping_loss > 0.0);
}

TEST_CASE("compliant sedan launches from settled rest in gear 1 (part and full throttle): no stall, ledger, bounded shaft torque",
          "[unit][vehicle][world][drivetrain][compliance]") {
    for (const real throttle : {0.4, 1.0}) {
        INFO("throttle " << throttle);
        real speed[2] = {0.0, 0.0};
        for (const bool compliant : {false, true}) {
            auto rig = ps_test::build_vehicle_rig_from_desc(compliant ? compliant_sedan(1, 600.0, 2.0) : ps_test::load_sedan_desc(), 1500.0,
                                                            Vec3{2.0, 0.4, 0.15}, 0.6, 0.0, 4);
            World& w = *rig.world;
            ps_test::settle(w, 0.5);
            w.set_control("gear_request", 1.0);
            w.set_control("throttle", throttle);
            bool stalled = false;
            real max_ratio = 0.0, max_shaft = 0.0, max_cap = 0.0, max_clutch = 0.0;
            // Last second: |shaft torque - torque the clutch delivers to
            // clutch.out|. The clutch row's lambda acts +lambda on clutch.in
            // and -lambda on clutch.out (dissipation -h*v*lambda >= 0), the
            // spring -tau_s on clutch.out (ports[0]) - so in quasi-steady
            // drive tau_s ~ -lambda, and the residual is clutch.out's own
            // J*domega/dt (J = 0.004 kg*m^2).
            real late_max_dev = 0.0;
            real late_max_shaft = 0.0;
            const int ticks = static_cast<int>(3.0 / w.dt() + 0.5);
            for (int i = 0; i < ticks; ++i) {
                w.step();
                if (w.powertrain_state(rig.vehicle).engine_state == EngineState::Stalled) stalled = true;
                const PowertrainEnergy& e = w.powertrain_energy(rig.vehicle);
                const real denom = sum_abs_terms(e);
                if (denom > 1e-9) max_ratio = std::max(max_ratio, std::abs(e.balance_error) / denom);
                if (compliant) {
                    const PowertrainDebug dbg = w.powertrain_debug(rig.vehicle);
                    max_shaft = std::max(max_shaft, std::abs(dbg.shaft_torque[0]));
                    max_cap = std::max(max_cap, dbg.clutch_cap_static);
                    max_clutch = std::max(max_clutch, std::abs(dbg.clutch_torque));
                    if (i >= ticks - static_cast<int>(1.0 / w.dt())) {
                        late_max_dev = std::max(late_max_dev, std::abs(dbg.shaft_torque[0] + dbg.clutch_torque));
                        late_max_shaft = std::max(late_max_shaft, std::abs(dbg.shaft_torque[0]));
                    }
                }
            }
            speed[compliant ? 1 : 0] = w.get_motion(rig.chassis).linear.length();
            std::printf("[compliance launch] throttle=%.1f %s speed@3s=%.4f stalled=%d max_balance_ratio=%.3e max|shaft|=%.2f "
                        "max|clutch|=%.2f max_cap_static=%.2f late max|shaft|=%.3f late max|shaft+clutch|=%.3f\n",
                        static_cast<double>(throttle), compliant ? "compliant" : "rigid    ", static_cast<double>(speed[compliant ? 1 : 0]),
                        stalled ? 1 : 0, static_cast<double>(max_ratio), static_cast<double>(max_shaft), static_cast<double>(max_clutch),
                        static_cast<double>(max_cap), static_cast<double>(late_max_shaft), static_cast<double>(late_max_dev));
            CHECK_FALSE(stalled);
            CHECK(max_ratio <= 5e-8);
            if (compliant) {
                CHECK(max_shaft > 0.0);
                // A spring-damper driven through a friction clutch cannot
                // carry more than twice the clutch's static capacity (the
                // undamped step-response overshoot bound); a numerically
                // unstable coupling would.
                CHECK(max_shaft <= 2.0 * max_cap);
                // Quasi-static transmission once launched: measured
                // 1.873/74.26 (2.5%) at throttle 0.4, 0.883/169.8 (0.5%) at 1.0.
                CHECK(late_max_dev <= 0.05 * late_max_shaft);
            }
        }
        INFO("speed rigid=" << speed[0] << " compliant=" << speed[1]);
        CHECK(speed[1] > 0.5 * speed[0]);
    }
}

TEST_CASE("compliant sedan state hash is worker-count invariant (1 vs 10 workers)",
          "[unit][vehicle][drivetrain][compliance][determinism]") {
    auto run = [](int workers) {
        auto rig = ps_test::build_vehicle_rig_from_desc(compliant_sedan(1, 600.0, 2.0), 1500.0, Vec3{2.0, 0.4, 0.15}, 0.6, 0.0, workers);
        World& w = *rig.world;
        ps_test::settle(w, 0.5);
        w.set_control("gear_request", 1.0);
        w.set_control("throttle", 1.0);
        for (int i = 0; i < 240; ++i) w.step();
        w.set_control("steer", 0.3);
        w.set_control("throttle", 0.0);
        for (int i = 0; i < 120; ++i) w.step();
        return w.state_hash();
    };
    const std::uint64_t h1 = run(1);
    const std::uint64_t h10 = run(10);
    CHECK(h1 == h10);
}

TEST_CASE("compliant sedan WOT launch: speed against shaft stiffness (diagnostic)", "[.][diag][unit][vehicle][drivetrain][compliance]") {
    // clutch.out (0.004) / gearbox.in (0.010): mu = 350; zeta held at
    // c*mu/(2*omega_n) = 0.764 (the shipped test's k 600 / c 2 pair).
    // Measured 2026-09-26 (release): rigid 12.4724 m/s; k 150/600/2400/9600/
    // 38400 -> 12.4095/12.5186/12.5300/12.5255/12.0857, max |kappa_FL|
    // 1.89-2.06 throughout (sustained FWD wheelspin, no traction control).
    // Non-monotonic in k, and k 600 with c = 2.0005 here vs. c = 2.0 exactly
    // in the launch case above differs by 3% (12.52 vs 12.16) - the WOT
    // speed spread is this wheelspin regime's own parameter sensitivity, not
    // a compliance trend (at throttle 0.4, no wheelspin, compliant and rigid
    // agree to 6e-4 m/s).
    auto run = [](bool compliant, real k) {
        vehicle::VehicleDesc d = ps_test::load_sedan_desc();
        if (compliant) {
            const real mu = 1.0 / 0.004 + 1.0 / 0.010;
            const real wn = std::sqrt(k * mu);
            d = compliant_sedan(1, k, 2.0 * 0.764 * wn / mu);
        }
        auto rig = ps_test::build_vehicle_rig_from_desc(d, 1500.0, Vec3{2.0, 0.4, 0.15}, 0.6, 0.0, 4);
        World& w = *rig.world;
        ps_test::settle(w, 0.5);
        w.set_control("gear_request", 1.0);
        w.set_control("throttle", 1.0);
        real max_kappa = 0.0;
        for (int i = 0; i < 720; ++i) {
            w.step();
            max_kappa = std::max(max_kappa, std::abs(w.wheel_telemetry(rig.vehicle, 0).slip_ratio));
        }
        std::printf("[compliance diag] %s k=%9.1f speed@3s=%.4f max|kappa_FL|=%.3f\n", compliant ? "compliant" : "rigid    ",
                    static_cast<double>(k), static_cast<double>(w.get_motion(rig.chassis).linear.length()), static_cast<double>(max_kappa));
    };
    run(false, 0.0);
    for (const real k : {150.0, 600.0, 2400.0, 9600.0, 38400.0}) run(true, k);
}
