// tests/unit/test_powertrain_multi_diff.cpp — PHYS-033 P4 I4 C4 (plan
// sections 2.5/2.7): multi-differential powertrains. White-box (core/src on
// this target's include path): Powertrain's per-state rigid-row order, each
// Differential's reconstruction mode, the speedo path, reset propagation
// leaves-up, and reconstruct_mesh_torque_pair / the nested (non-leaf)
// reconstruction against an independent carrier-side momentum balance.
// Whole-vehicle AWD behaviour lives in tests/validation/
// test_powertrain_multi_diff_awd.cpp.

#include "drivetrain/components.h"
#include "drivetrain/network.h"
#include "drivetrain/powertrain.h"

#include "powertrain_multi_diff_desc.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;

namespace {

Powertrain build(const vehicle::VehicleDesc& d) { return Powertrain(d.powertrain, d.brake_system, d.wheels, d.controllers, d.name); }

// One diff row's terms exactly as Powertrain emits them (carrier +1, L/R
// -ratio/2).
void check_diff_row(const RigidRow& row, const Differential& d) {
    REQUIRE(row.terms.size() == 3);
    CHECK(row.terms[0].first == static_cast<int>(d.carrier_node().index));
    CHECK(row.terms[0].second == real(1.0));
    CHECK(row.terms[1].first == static_cast<int>(d.left_node().index));
    CHECK(row.terms[1].second == -d.ratio() / real(2.0));
    CHECK(row.terms[2].first == static_cast<int>(d.right_node().index));
    CHECK(row.terms[2].second == -d.ratio() / real(2.0));
}

// Walks build_rigid_rows() state by state: every diff in component order,
// then the gearbox row (in +1, out -ratio) in every state but Neutral.
void check_row_order(const Powertrain& pt) {
    const auto& rows = pt.build_rigid_rows();
    const int n_rev = static_cast<int>(pt.gearbox().desc().reverse_ratios.size());
    const int n_states = n_rev + 1 + static_cast<int>(pt.gearbox().desc().forward_ratios.size());
    std::size_t k = 0;
    for (int state = 0; state < n_states; ++state) {
        for (std::size_t di = 0; di < pt.diff_count(); ++di) {
            REQUIRE(k < rows.size());
            CHECK(rows[k].first == state);
            check_diff_row(rows[k].second, pt.diff(di));
            ++k;
        }
        const int position = state - n_rev;
        if (position != 0) {
            REQUIRE(k < rows.size());
            CHECK(rows[k].first == state);
            REQUIRE(rows[k].second.terms.size() == 2);
            CHECK(rows[k].second.terms[0].second == real(1.0));
            CHECK(rows[k].second.terms[1].second == -pt.gearbox().ratio_for_position(position));
            ++k;
        }
    }
    CHECK(k == rows.size()); // no gear_pair in any of these graphs
}

} // namespace

TEST_CASE("multi-diff: car_sedan's rigid-row order is I3's [diff, gearbox] per state", "[unit][drivetrain][vehicle]") {
    const vehicle::VehicleDesc d = ps_test::load_sedan_desc();
    const Powertrain pt = build(d);
    REQUIRE(pt.diff_count() == 1);
    CHECK(pt.diff(0).recon_mode() == Differential::ReconMode::Leaf);
    check_row_order(pt);
    // Speedo path: I3's own |diff.ratio| and driven mean radius, bit-exact.
    CHECK(pt.gearbox_output_ratio_magnitude() == std::abs(pt.diff_ratio()));
    CHECK(pt.speedo_radius() == pt.driven_wheel_mean_radius());
    CHECK(pt.gearbox_output_ratio_magnitude() == real(4.06));
    // Reset derivation: the single diff carrier from its two hubs.
    REQUIRE(pt.reset_derivations().size() == 1);
    CHECK(pt.reset_derivations()[0].target == pt.diff(0).carrier_node());
}

TEST_CASE("multi-diff: an AWD graph (open centre -> two axle diffs) builds with the right rows, modes and speedo",
          "[unit][drivetrain][vehicle]") {
    for (const real centre_eff : {real(1.0), real(0.99)}) {
        INFO("centre efficiency " << centre_eff);
        const vehicle::VehicleDesc d = ps_test::make_awd_desc(centre_eff);
        const Powertrain pt = build(d);
        REQUIRE(pt.diff_count() == 3);
        CHECK(pt.diff(0).recon_mode() == Differential::ReconMode::Nested); // centre
        CHECK(pt.diff(1).recon_mode() == Differential::ReconMode::Leaf);   // front
        CHECK(pt.diff(2).recon_mode() == Differential::ReconMode::Leaf);   // rear
        CHECK(pt.diff(0).left_node() == pt.diff(1).carrier_node());
        CHECK(pt.diff(0).right_node() == pt.diff(2).carrier_node());
        check_row_order(pt);
        for (std::size_t w = 0; w < 4; ++w) CHECK(pt.wheel_is_driven(w));
        // Every path is 1.0 * 4.06 and every radius 0.317: the literal form.
        CHECK(pt.gearbox_output_ratio_magnitude() == real(1.0) * real(4.06));
        CHECK(pt.speedo_radius() == Catch::Approx(0.317).epsilon(1e-15));
    }
}

TEST_CASE("multi-diff: two axle diffs sharing one rigid carrier shaft build as two leaf diffs", "[unit][drivetrain][vehicle]") {
    const vehicle::VehicleDesc d = ps_test::make_parallel_4wd_desc();
    const Powertrain pt = build(d);
    REQUIRE(pt.diff_count() == 2);
    CHECK(pt.diff(0).recon_mode() == Differential::ReconMode::Leaf);
    CHECK(pt.diff(1).recon_mode() == Differential::ReconMode::Leaf);
    CHECK(pt.diff(0).carrier_node() == pt.diff(1).carrier_node());
    check_row_order(pt);
    // Two complete estimates of the same carrier speed are averaged, never
    // summed: still 4.06, not 8.12.
    CHECK(pt.gearbox_output_ratio_magnitude() == real(4.06));
    CHECK(pt.speedo_radius() == Catch::Approx(0.317).epsilon(1e-15));
}

TEST_CASE("multi-diff: a lossy diff whose output carries a non-diff-carrier rigid row is rejected", "[unit][drivetrain][vehicle]") {
    // a.left is joined to b.LEFT (not b's carrier): a's left node carries
    // b's output term, so neither the leaf nor the nested reconstruction
    // applies to a.
    vehicle::VehicleDesc d = ps_test::load_sedan_desc();
    const DifferentialDesc axle = ps_test::strip_sedan_diff(d);
    d.powertrain.components.push_back(ComponentDesc{"a", axle});
    d.powertrain.components.push_back(ComponentDesc{"b", axle});
    auto& sh = d.powertrain.shafts;
    sh.push_back(ps_test::shaft(ps_test::comp_port("gearbox", "out"), ps_test::comp_port("a", "carrier")));
    ShaftDesc left;
    left.ports = {ps_test::comp_port("a", "left"), ps_test::comp_port("b", "left"), ps_test::wheel_port("FL")};
    sh.push_back(left);
    sh.push_back(ps_test::shaft(ps_test::comp_port("a", "right"), ps_test::wheel_port("FR")));
    sh.push_back(ps_test::shaft(ps_test::comp_port("b", "carrier"), ps_test::wheel_port("RL")));
    sh.push_back(ps_test::shaft(ps_test::comp_port("b", "right"), ps_test::wheel_port("RR")));
    CHECK_THROWS_WITH(build(d), Catch::Matchers::ContainsSubstring("differential 'a' has efficiency < 1"));
}

TEST_CASE("reconstruct_mesh_torque_pair recovers a diff row's multiplier under a lock row, an L-R friction row or an L-R coupling",
          "[unit][drivetrain][network]") {
    // Carrier C carries ONLY the diff row, so its own momentum balance is an
    // independent truth for lambda (reconstruct_mesh_torque at C, coef 1).
    const real ratio = 3.7;
    const real h = 1.0 / 960.0;
    for (int variant = 0; variant < 3; ++variant) {
        INFO("variant " << variant << " (0 lock row, 1 L-R friction row, 2 L-R coupling)");
        NetworkBuilder nb;
        const NodeId c = nb.add_node(0.05);
        const NodeId l = nb.add_node(1.0);
        const NodeId r = nb.add_node(1.3);
        RigidRow diff_row;
        diff_row.terms = {{static_cast<int>(c.index), real(1.0)}, {static_cast<int>(l.index), -ratio / real(2.0)},
                           {static_cast<int>(r.index), -ratio / real(2.0)}};
        nb.add_rigid_row(0, diff_row);
        FrictionRowId lsd{};
        CouplingId coupling{};
        if (variant == 0) {
            RigidRow lock;
            lock.terms = {{static_cast<int>(l.index), real(1.0)}, {static_cast<int>(r.index), real(-1.0)}};
            nb.add_rigid_row(0, lock);
        } else if (variant == 1) {
            lsd = nb.add_friction_row(l, r);
        } else {
            coupling = nb.add_coupling(l, r);
        }
        const FrictionRowId brake_r = nb.add_friction_row(r); // a ground row at R: must be subtracted, not cancel
        ShaftNetwork net = nb.build("pair-test");
        net.set_omega(l, 5.0);
        net.set_omega(r, 9.0);
        net.set_omega(c, ratio * ((5.0 + 9.0) / 2.0));
        if (variant == 0) {
            net.set_omega(r, 5.0);
            net.set_omega(c, ratio * 5.0);
        }

        Assembly a;
        for (int s = 0; s < 40; ++s) {
            a.begin_substep(net.node_count());
            a.begin_substep_rows(net.friction_row_count());
            a.begin_substep_couplings(net.coupling_count());
            a.torque(c, 250.0, 0.0);
            a.torque(l, -120.0, 6.0);
            a.torque(r, -60.0, 14.0);
            a.set_friction_capacity(brake_r, 25.0, 25.0);
            if (variant == 1) a.set_friction_capacity(lsd, 40.0, 40.0);
            if (variant == 2) a.set_coupling(coupling, 1.0, 1.0, 8.0);
            const real wc = net.omega(c), wl = net.omega(l), wr = net.omega(r);
            net.solve(h, a);
            const real truth = reconstruct_mesh_torque(c, real(1.0), real(0.0), net, a, wc, h);
            const real pair = reconstruct_mesh_torque_pair(l, r, -ratio, 0.0, 0.0, 0.0, 0.0, net, a, wl, wr, h);
            CHECK(pair == Catch::Approx(truth).epsilon(1e-9).margin(1e-9));
        }
    }
}

TEST_CASE("nested diff reconstruction: centre and axle multipliers match carrier-side truth, in any post_solve order",
          "[unit][drivetrain][network]") {
    // G = centre carrier (== gearbox.out), A/B = axle carriers (the centre's
    // outputs), four hubs. Every diff lossy, so each applies a real loss
    // torque at its own carrier from the second substep on - the axle
    // diffs' losses land ON the centre's reconstruction nodes A/B.
    const real h = 1.0 / 960.0;
    auto make = [](NetworkBuilder& nb, std::vector<NodeId>& n) {
        n.push_back(nb.add_node(0.03)); // G
        n.push_back(nb.add_node(0.02)); // A
        n.push_back(nb.add_node(0.02)); // B
        for (int i = 0; i < 4; ++i) n.push_back(nb.add_node(0.9 + 0.05 * i));
        auto row = [](NodeId carrier, NodeId left, NodeId right, real ratio) {
            RigidRow rr;
            rr.terms = {{static_cast<int>(carrier.index), real(1.0)}, {static_cast<int>(left.index), -ratio / real(2.0)},
                        {static_cast<int>(right.index), -ratio / real(2.0)}};
            return rr;
        };
        nb.add_rigid_row(0, row(n[0], n[1], n[2], 1.0));
        nb.add_rigid_row(0, row(n[1], n[3], n[4], 4.06));
        nb.add_rigid_row(0, row(n[2], n[5], n[6], 4.06));
    };
    DifferentialDesc centre_d;
    centre_d.ratio = 1.0;
    centre_d.efficiency = 0.99;
    DifferentialDesc axle_d;
    axle_d.ratio = 4.06;
    axle_d.efficiency = 0.97;

    struct Rig {
        std::vector<NodeId> n;
        ShaftNetwork net;
        FrictionRowId brake;
        std::unique_ptr<Differential> centre, front, rear;
    };
    auto make_rig = [&]() {
        Rig rig;
        NetworkBuilder nb;
        make(nb, rig.n);
        rig.brake = nb.add_friction_row(rig.n[5]); // one braked rear hub
        rig.net = nb.build("nested-test");
        rig.centre = std::make_unique<Differential>(rig.n[0], rig.n[1], rig.n[2], centre_d);
        rig.front = std::make_unique<Differential>(rig.n[1], rig.n[3], rig.n[4], axle_d);
        rig.rear = std::make_unique<Differential>(rig.n[2], rig.n[5], rig.n[6], axle_d);
        rig.front->configure_reconstruction(Differential::ReconMode::Leaf, {}, {});
        rig.rear->configure_reconstruction(Differential::ReconMode::Leaf, {}, {});
        rig.centre->configure_reconstruction(Differential::ReconMode::Nested, {rig.front.get()}, {rig.rear.get()});
        return rig;
    };
    Rig r1 = make_rig();
    Rig r2 = make_rig();

    ComponentSubstepCtx ctx;
    ctx.h = h;
    Assembly a1, a2;
    PowertrainEnergy e1{}, e2{};
    for (int s = 0; s < 60; ++s) {
        auto pre = [&](Rig& rig, Assembly& a) -> real {
            a.begin_substep(rig.net.node_count());
            a.begin_substep_rows(rig.net.friction_row_count());
            a.begin_substep_couplings(rig.net.coupling_count());
            a.torque(rig.n[0], 400.0, 0.0);
            for (int i = 0; i < 4; ++i) a.torque(rig.n[static_cast<std::size_t>(3 + i)], -20.0 - 15.0 * i, 3.0 + i);
            a.set_friction_capacity(rig.brake, 30.0, 30.0);
            rig.centre->pre_solve(ctx, rig.net, a);
            rig.front->pre_solve(ctx, rig.net, a);
            rig.rear->pre_solve(ctx, rig.net, a);
            return rig.net.omega(rig.n[0]);
        };
        const real wg1 = pre(r1, a1);
        const real wa1 = r1.net.omega(r1.n[1]);
        (void)pre(r2, a2);
        r1.net.solve(h, a1);
        r2.net.solve(h, a2);
        // Order 1: centre first (children reconstructed on demand).
        r1.centre->post_solve(ctx, r1.net, a1, e1);
        r1.front->post_solve(ctx, r1.net, a1, e1);
        r1.rear->post_solve(ctx, r1.net, a1, e1);
        // Order 2: children first.
        r2.rear->post_solve(ctx, r2.net, a2, e2);
        r2.front->post_solve(ctx, r2.net, a2, e2);
        r2.centre->post_solve(ctx, r2.net, a2, e2);

        // Truth for the centre: G carries only the centre row (its loss is a
        // real applied torque in tau0 at G, NOT subtracted - lambda is the
        // pure row multiplier).
        const real lc_truth = reconstruct_mesh_torque(r1.n[0], real(1.0), real(0.0), r1.net, a1, wg1, h);
        CHECK(r1.centre->lambda_mesh() == Catch::Approx(lc_truth).epsilon(1e-9).margin(1e-9));
        // Truth for the front diff: A's balance = lambda_front - 0.5*lambda_centre.
        const real bal_a = reconstruct_mesh_torque(r1.n[1], real(1.0), real(0.0), r1.net, a1, wa1, h);
        CHECK(r1.front->lambda_mesh() == Catch::Approx(bal_a + real(0.5) * lc_truth).epsilon(1e-9).margin(1e-9));
        // The hashed state is order-independent, bit for bit.
        const real p1 = r1.centre->lambda_mesh_prev(), p2 = r2.centre->lambda_mesh_prev();
        CHECK(std::memcmp(&p1, &p2, sizeof(real)) == 0);
        CHECK(r1.centre->fold_state_hash(1469598103934665603ULL) == r2.centre->fold_state_hash(1469598103934665603ULL));
    }
    CHECK(e1.gear_loss == Catch::Approx(e2.gear_loss).epsilon(1e-12)); // same terms, different summation order
    CHECK(e1.gear_loss > 0.0);
    CHECK(r1.centre->lambda_mesh() < 0.0); // driving: the row's reaction on the carrier (+1*lambda) opposes the drive torque
}

TEST_CASE("multi-diff reset: derived nodes propagate leaves up with the literal carrier expression", "[unit][drivetrain][vehicle]") {
    const vehicle::VehicleDesc d = ps_test::make_awd_desc(0.99);
    Powertrain pt = build(d);
    const auto& der = pt.reset_derivations();
    REQUIRE(der.size() == 3);
    // Component order is centre, front, rear; the centre reads the axle
    // carriers, so it must come last.
    CHECK(der[0].target == pt.diff(1).carrier_node());
    CHECK(der[1].target == pt.diff(2).carrier_node());
    CHECK(der[2].target == pt.diff(0).carrier_node());

    vehicle::VehicleResetOptions opts;
    opts.gear = 1;
    opts.clutch_locked = true;
    const std::vector<real> hubs{10.0, 11.0, 12.5, 13.0};
    pt.reset(opts, hubs);
    const ShaftNetwork& net = pt.network();
    const real wa = real(4.06) * ((real(10.0) + real(11.0)) / real(2.0));
    const real wb = real(4.06) * ((real(12.5) + real(13.0)) / real(2.0));
    const real wg = real(1.0) * ((wa + wb) / real(2.0));
    CHECK(net.omega(pt.diff(1).carrier_node()) == wa);
    CHECK(net.omega(pt.diff(2).carrier_node()) == wb);
    CHECK(net.omega(pt.diff(0).carrier_node()) == wg);
    CHECK(pt.gearbox_out_omega() == wg);
    CHECK(pt.gearbox_in_omega() == pt.gearbox().ratio_for_position(1) * wg);
    // Every diff row is satisfied by the reset state.
    for (std::size_t i = 0; i < pt.diff_count(); ++i) {
        const Differential& df = pt.diff(i);
        const real res = net.omega(df.carrier_node()) - df.ratio() / 2.0 * (net.omega(df.left_node()) + net.omega(df.right_node()));
        CHECK(std::abs(res) < 1e-12);
    }
}

TEST_CASE("single-diff reset keeps I3's carrier value", "[unit][drivetrain][vehicle]") {
    const vehicle::VehicleDesc d = ps_test::load_sedan_desc();
    Powertrain pt = build(d);
    vehicle::VehicleResetOptions opts;
    opts.gear = 2;
    const std::vector<real> hubs{7.25, 7.5, 3.0, 3.0};
    pt.reset(opts, hubs);
    // I3: diff_ratio_ * ((0 + 7.25 + 7.5) / 2) over the driven hubs FL, FR.
    const real i3 = real(4.06) * ((real(0.0) + real(7.25) + real(7.5)) / static_cast<real>(2));
    CHECK(pt.gearbox_out_omega() == i3);
}
