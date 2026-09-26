// tests/unit/test_powertrain_transfer_case.cpp — PHYS-033 P4 I4 C9 (plan
// section 3.3): the MFWD transfer_case component (tractor.json's front
// axle). White-box (core/src on this target's include path): the friction
// row's ratio-generalised momentum/energy identities in three states
// (slipping, stuck, disengaged - mirrors test_powertrain_network.cpp's own
// "two inertias lock"/"a friction row's ratio generalises its coupling"
// pattern exactly, at the same raw NetworkBuilder/Assembly level those
// tests already use, since the transfer case's front row is built exactly
// like any other friction row - see components.h's TransferCase comment),
// the rigid `rear` row / reset-derivation propagation, and the speedo
// path's restriction to `rear` only (tractor.json's speedo must stay
// bit-identical to a hand-built pre-C9 topology with no transfer case /
// front axle at all).

#include "drivetrain/components.h"
#include "drivetrain/network.h"
#include "drivetrain/powertrain.h"

#include "ps/io/vehicle_io.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;

namespace {
Powertrain build(const vehicle::VehicleDesc& d) { return Powertrain(d.powertrain, d.brake_system, d.wheels, d.controllers, d.name); }

vehicle::VehicleDesc load_tractor() { return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/tractor.json"); }
} // namespace

// --- Truth test: the transfer case's front friction row, three states -----
// (plan's own requirement: "verified against a momentum-balance truth, with
// the MFWD row slipping, stuck, and off"). Built at the raw network level,
// not through the whole vehicle, so the identity being checked is exactly
// the one TransferCase::pre_solve/post_solve rely on (Assembly::
// set_friction_row_ratio(row, front_ratio), the same row shape a gearbox
// synchro row uses) - independent of any wheel/tyre model.
TEST_CASE("transfer_case friction row: slipping, stuck and disengaged states satisfy the "
          "ratio-generalised momentum/energy identities",
          "[unit][drivetrain][network]") {
    NetworkBuilder nb;
    const NodeId n_in = nb.add_node(0.5);    // stands in for transfer.in's unioned shaft inertia
    const NodeId n_front = nb.add_node(0.25); // stands in for transfer.front's unioned shaft inertia
    const FrictionRowId row = nb.add_friction_row(n_in, n_front);
    ShaftNetwork net = nb.build("test-transfer-case");

    const real J_in = 0.5, J_front = 0.25, ratio = 1.0; // tractor.json's own front_ratio = 1.0
    const real w_in0 = 120.0, w_front0 = 0.0;           // rear driveline spinning, front axle at rest
    net.set_omega(n_in, w_in0);
    net.set_omega(n_front, w_front0);
    const real h = 1.0 / 960.0;
    const real M0 = ratio * J_in * w_in0 + J_front * w_front0;

    SECTION("slipping: momentum conserved, row velocity is the ratio'd slip, KE identity holds") {
        const real cap = 40.0; // small enough that slip persists for many substeps
        Assembly assembly;
        bool seen_slip = false;
        for (int s = 0; s < 50; ++s) {
            assembly.begin_substep(net.node_count());
            assembly.begin_substep_rows(net.friction_row_count());
            assembly.set_friction_row_ratio(row, ratio);
            assembly.set_friction_capacity(row, cap, cap);

            const real w_in_before = net.omega(n_in), w_front_before = net.omega(n_front);
            const real ke_before = 0.5 * (J_in * w_in_before * w_in_before + J_front * w_front_before * w_front_before);

            net.solve(h, assembly);
            REQUIRE_FALSE(net.row_stuck(row)); // capacity is far below what full lock-up at this slip would need

            const real w_in_after = net.omega(n_in), w_front_after = net.omega(n_front);
            const real ke_after = 0.5 * (J_in * w_in_after * w_in_after + J_front * w_front_after * w_front_after);
            const real m_after = ratio * J_in * w_in_after + J_front * w_front_after;
            CHECK(std::abs(m_after - M0) <= 1e-9 * std::max(real(1.0), std::abs(M0)));

            const real v_expected = w_in_before - ratio * w_front_before; // pre_solve's own slip definition
            CHECK(net.row_velocity(row) == Catch::Approx(w_in_after - ratio * w_front_after).margin(1e-9));
            (void)v_expected;

            const real lambda = net.row_lambda(row);
            CHECK(std::abs(lambda) <= cap * (1.0 + 1e-9)); // kinetic capacity bound, never exceeded while slipping
            CHECK(lambda < 0.0); // opposes the in->front slip direction (drags front up, in down)

            const real integrator_loss = 0.5 * (J_in * (w_in_after - w_in_before) * (w_in_after - w_in_before) +
                                                 J_front * (w_front_after - w_front_before) * (w_front_after - w_front_before));
            const real predicted = h * net.row_velocity(row) * lambda - integrator_loss;
            CHECK(std::abs((ke_after - ke_before) - predicted) <= 1e-9 * std::max(real(1.0), std::abs(ke_before)));
            seen_slip = true;
        }
        REQUIRE(seen_slip);
        CHECK(net.omega(n_in) < w_in0);       // the row has dragged the driveline down
        CHECK(net.omega(n_front) > w_front0); // and spun the front axle up
    }

    SECTION("stuck: locks at the exact closed-form common speed and stays ratio'd") {
        const real cap = 500.0; // large enough to lock quickly
        Assembly assembly;
        bool locked = false;
        for (int s = 0; s < 2000 && !locked; ++s) {
            assembly.begin_substep(net.node_count());
            assembly.begin_substep_rows(net.friction_row_count());
            assembly.set_friction_row_ratio(row, ratio);
            assembly.set_friction_capacity(row, cap, cap);
            net.solve(h, assembly);
            if (net.row_stuck(row)) locked = true;
        }
        REQUIRE(locked);
        const real w_front_expected = M0 / (ratio * ratio * J_in + J_front);
        const real w_in_expected = ratio * w_front_expected;
        CHECK(net.omega(n_front) == Catch::Approx(w_front_expected).epsilon(1e-9));
        CHECK(net.omega(n_in) == Catch::Approx(w_in_expected).epsilon(1e-9));

        // Stays stuck, momentum and the ratio'd lock both hold, for more substeps.
        for (int s = 0; s < 50; ++s) {
            assembly.begin_substep(net.node_count());
            assembly.begin_substep_rows(net.friction_row_count());
            assembly.set_friction_row_ratio(row, ratio);
            assembly.set_friction_capacity(row, cap, cap);
            net.solve(h, assembly);
            REQUIRE(net.row_stuck(row));
            CHECK(net.omega(n_in) == Catch::Approx(ratio * net.omega(n_front)).epsilon(1e-9));
            const real m_after = ratio * J_in * net.omega(n_in) + J_front * net.omega(n_front);
            CHECK(std::abs(m_after - M0) <= 1e-9 * std::max(real(1.0), std::abs(M0)));
        }
    }

    SECTION("disengaged (capacity 0): the row carries zero torque and each node free-wheels independently") {
        Assembly assembly;
        for (int s = 0; s < 50; ++s) {
            assembly.begin_substep(net.node_count());
            assembly.begin_substep_rows(net.friction_row_count());
            assembly.set_friction_row_ratio(row, ratio);
            assembly.set_friction_capacity(row, real(0.0), real(0.0)); // engagement e == 0
            net.solve(h, assembly);
            CHECK(net.row_lambda(row) == real(0.0));
            // No other torque acts on either node - a disengaged row must not
            // couple them at all, so both stay at their initial speed (to
            // within the linear solve's own floating-point roundoff, ~1e-13
            // absolute here - the reduced system is still assembled and
            // solved even at zero capacity, it just yields exactly zero
            // coupling torque, not literally two decoupled scalar ODEs).
            CHECK(net.omega(n_in) == Catch::Approx(w_in0).margin(1e-9));
            CHECK(net.omega(n_front) == Catch::Approx(w_front0).margin(1e-9));
            // The slip itself is nonzero and persists (the front axle is
            // simply free-wheeling relative to the driveline, per
            // TransferCaseDesc's own doc comment) - the ledger contribution
            // -h*v*lambda is exactly zero only because lambda is zero, not
            // because the slip vanished.
            CHECK(net.row_velocity(row) == Catch::Approx(w_in0 - ratio * w_front0).margin(1e-12));
        }
    }
}

// --- Graph integration: rigid `rear` row, reset derivation, driven wheels --
TEST_CASE("tractor.json's transfer_case and front differential integrate into the shaft-network graph",
          "[unit][drivetrain][vehicle]") {
    const vehicle::VehicleDesc d = load_tractor();
    Powertrain pt = build(d);

    // Two differentials now: rear (component "diff", built first) then the
    // new front axle ("front_diff") - component order in the JSON file.
    REQUIRE(pt.diff_count() == 2);
    CHECK(pt.diff(0).ratio() == Catch::Approx(20.0));
    CHECK(pt.diff(1).ratio() == Catch::Approx(15.30));

    // Wheel order in tractor.json is FL(0), FR(1), RL(2), RR(3). Before this
    // commit only RL/RR were driven; the front axle is now driven too
    // (through the front_diff, whatever the transfer case's own engagement -
    // wheel_is_driven means "this hub's shaft port was connected to
    // anything", not "currently receiving torque").
    CHECK(pt.wheel_is_driven(0)); // FL
    CHECK(pt.wheel_is_driven(1)); // FR
    CHECK(pt.wheel_is_driven(2)); // RL
    CHECK(pt.wheel_is_driven(3)); // RR

    // Reset derivations, leaves up (PHYS-033 P4 I4 C4's own mechanism,
    // extended by C9's transfer.in = rear_ratio*omega(rear) rule): the rear
    // diff and the front diff are both leaves (fed only by wheel hubs), so
    // both come before the transfer case, which depends on the rear diff's
    // own carrier.
    const auto& der = pt.reset_derivations();
    REQUIRE(der.size() == 3);
    CHECK(der[0].is_diff);
    CHECK(der[0].target == pt.diff(0).carrier_node()); // rear diff, ready first (component order among leaves)
    CHECK(der[1].is_diff);
    CHECK(der[1].target == pt.diff(1).carrier_node()); // front diff, also a leaf
    CHECK_FALSE(der[2].is_diff);                       // the transfer case's own in = rear_ratio*rear rule
    CHECK(der[2].a == pt.diff(0).carrier_node());       // reads the REAR diff's carrier only, never the front's
    CHECK(der[2].ratio == Catch::Approx(1.0));          // tractor.json's rear_ratio

    vehicle::VehicleResetOptions opts;
    opts.gear = 1;
    const std::vector<real> hubs{9.0, 9.2, 4.0, 4.1}; // FL, FR, RL, RR (rad/s)
    pt.reset(opts, hubs);
    const ShaftNetwork& net = pt.network();

    const real w_rear_carrier = real(20.0) * ((real(4.0) + real(4.1)) / real(2.0));
    const real w_front_carrier = real(15.30) * ((real(9.0) + real(9.2)) / real(2.0));
    CHECK(net.omega(pt.diff(0).carrier_node()) == Catch::Approx(w_rear_carrier));
    CHECK(net.omega(pt.diff(1).carrier_node()) == Catch::Approx(w_front_carrier));
    // The transfer case's `in` node: rear_ratio (1.0) * the rear carrier -
    // NEVER a function of the front carrier (der[2].a above already pins
    // this structurally; this pins the actual numeric outcome too).
    CHECK(net.omega(der[2].target) == Catch::Approx(real(1.0) * w_rear_carrier));
}

// --- Speedo path: unaffected by adding the MFWD front axle -----------------
TEST_CASE("tractor.json's speedo path is bit-identical before and after the MFWD front axle "
          "(I4 C9 plan section 2.5)",
          "[unit][drivetrain][vehicle]") {
    const vehicle::VehicleDesc mfwd = load_tractor();
    const Powertrain pt_mfwd = build(mfwd);

    // Hand-build the pre-C9 topology: drop "transfer"/"front_diff" and
    // rewire "gearbox.out" directly onto "diff.carrier" - literally what
    // tractor.json looked like before this commit (see git history).
    vehicle::VehicleDesc baseline = mfwd;
    std::vector<drivetrain::ComponentDesc> kept_components;
    for (const auto& c : baseline.powertrain.components) {
        if (c.id == "transfer" || c.id == "front_diff") continue;
        kept_components.push_back(c);
    }
    baseline.powertrain.components = kept_components;

    std::vector<drivetrain::ShaftDesc> kept_shafts;
    for (const auto& s : baseline.powertrain.shafts) {
        bool touches_removed = false;
        for (const auto& p : s.ports) {
            if (p.component == "transfer" || p.component == "front_diff") touches_removed = true;
        }
        if (!touches_removed) kept_shafts.push_back(s);
    }
    drivetrain::ShaftDesc direct;
    direct.ports = {{"gearbox", "out"}, {"diff", "carrier"}};
    kept_shafts.push_back(direct);
    baseline.powertrain.shafts = kept_shafts;

    const Powertrain pt_baseline = build(baseline);

    // The whole point of the C9 plan's speedo restriction: adding the front
    // axle must not move either speedo quantity by a single bit.
    CHECK(pt_mfwd.gearbox_output_ratio_magnitude() == pt_baseline.gearbox_output_ratio_magnitude());
    CHECK(pt_mfwd.speedo_radius() == pt_baseline.speedo_radius());
    CHECK(pt_mfwd.gearbox_output_ratio_magnitude() == Catch::Approx(20.0));
    CHECK(pt_mfwd.speedo_radius() == Catch::Approx(1.025));

    // Baseline (no transfer case) also has only the 2 driven rear wheels -
    // confirms the baseline really does reproduce the pre-C9 vehicle, not
    // just the same speedo numbers by coincidence.
    CHECK_FALSE(pt_baseline.wheel_is_driven(0));
    CHECK_FALSE(pt_baseline.wheel_is_driven(1));
    CHECK(pt_baseline.wheel_is_driven(2));
    CHECK(pt_baseline.wheel_is_driven(3));
}
