// tests/unit/test_powertrain_diff.cpp — PHYS-033 P4 I4 C6 (plan section
// 3.1): DifferentialKind::Locked (spool: an UNCONDITIONAL, state-independent
// omega_L == omega_R row alongside the diff's own open row, spending no
// composed-state bit) and DifferentialKind::Viscous (the open row plus an
// implicit L-R coupling, DifferentialDesc::viscous_nm_s_rad). White-box
// (core/src on this target's include path), mirroring
// test_powertrain_topology.cpp's own patterns: raw NetworkBuilder+
// Differential construction for the pure-math reconstruction checks, the
// Driver/Powertrain::build() pattern for whole-graph mode-selection/
// rejection checks, and a full World for the D8 zero-alloc check.

#include "drivetrain/components.h"
#include "drivetrain/network.h"
#include "drivetrain/powertrain.h"

#include "powertrain_multi_diff_desc.h"

#include "ps/io/surface_table.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/vehicle/vehicle_reset.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

using namespace ps;
using namespace ps::drivetrain;

#ifndef PS_SOURCE_DIR
#error "PS_SOURCE_DIR must be defined by CMake (tests/unit/CMakeLists.txt)"
#endif

namespace {

Powertrain build(const vehicle::VehicleDesc& d) { return Powertrain(d.powertrain, d.brake_system, d.wheels, d.controllers, d.name); }

DifferentialDesc& diff_desc(vehicle::VehicleDesc& d, const std::string& id) {
    for (auto& c : d.powertrain.components) {
        if (c.id == id) return std::get<DifferentialDesc>(c.params);
    }
    throw std::runtime_error("no differential '" + id + "'");
}

// car_sedan.json with its (only) diff set to kind Locked or Viscous.
// `lossless` sets the diff's efficiency to 1 (Locked/Viscous itself still
// dissipates via the coupling for Viscous - efficiency only gates the row's
// own mesh loss, see Differential::post_solve).
vehicle::VehicleDesc locked_sedan(bool lossless = false) {
    vehicle::VehicleDesc d = ps_test::load_sedan_desc();
    DifferentialDesc& diff = diff_desc(d, "diff");
    diff.kind = DifferentialKind::Locked;
    if (lossless) diff.efficiency = 1.0;
    return d;
}

vehicle::VehicleDesc viscous_sedan(real c, bool lossless = false) {
    vehicle::VehicleDesc d = ps_test::load_sedan_desc();
    DifferentialDesc& diff = diff_desc(d, "diff");
    diff.kind = DifferentialKind::Viscous;
    diff.viscous_nm_s_rad = c;
    if (lossless) diff.efficiency = 1.0;
    return d;
}

// tests/support/powertrain_multi_diff_desc.h's make_awd_desc(), generalised
// (LOCAL to this file per the task's own instruction not to add new
// functions to that shared header) so the centre diff can be any kind -
// open front/rear axle diffs (car_sedan's own diff template, ratio 4.06,
// efficiency 0.97) fed by a centre diff of the requested kind/efficiency,
// ratio 1 - same shape as make_awd_desc, only the centre component differs.
vehicle::VehicleDesc make_centre_kind_awd_desc(DifferentialKind kind, real efficiency, real viscous_c = 0.0) {
    vehicle::VehicleDesc desc = ps_test::load_sedan_desc();
    const DifferentialDesc axle = ps_test::strip_sedan_diff(desc);
    DifferentialDesc centre = axle;
    centre.kind = kind;
    centre.ratio = 1.0;
    centre.efficiency = efficiency;
    centre.viscous_nm_s_rad = viscous_c;
    centre.carrier_inertia_kgm2 = 0.01;
    auto& comps = desc.powertrain.components;
    comps.push_back(ComponentDesc{"centre", centre});
    comps.push_back(ComponentDesc{"front", axle});
    comps.push_back(ComponentDesc{"rear", axle});
    auto& shafts = desc.powertrain.shafts;
    shafts.push_back(ps_test::shaft(ps_test::comp_port("gearbox", "out"), ps_test::comp_port("centre", "carrier")));
    shafts.push_back(ps_test::shaft(ps_test::comp_port("centre", "left"), ps_test::comp_port("front", "carrier")));
    shafts.push_back(ps_test::shaft(ps_test::comp_port("centre", "right"), ps_test::comp_port("rear", "carrier")));
    shafts.push_back(ps_test::shaft(ps_test::comp_port("front", "left"), ps_test::wheel_port("FL")));
    shafts.push_back(ps_test::shaft(ps_test::comp_port("front", "right"), ps_test::wheel_port("FR")));
    shafts.push_back(ps_test::shaft(ps_test::comp_port("rear", "left"), ps_test::wheel_port("RL")));
    shafts.push_back(ps_test::shaft(ps_test::comp_port("rear", "right"), ps_test::wheel_port("RR")));
    desc.name = "car_sedan_centre_kind_awd_test";
    return desc;
}

// Drives one Powertrain substep by substep - test_powertrain_topology.cpp's
// Driver, no declared channels needed here (kind Locked/Viscous consult
// neither locks_ nor ComponentSubstepCtx::channels).
struct Driver {
    Powertrain& pt;
    ComponentSubstepCtx ctx;

    explicit Driver(Powertrain& p) : pt(p) {
        ctx.h = 1.0 / 960.0;
        ctx.ignition = 0.0;
        ctx.assist_auto_clutch = false;
        ctx.assist_auto_blip = false;
        ctx.assist_auto_shift = false;
    }
    void step(const std::vector<real>& hub_tau = {}, const std::vector<real>& hub_d = {}) {
        pt.begin_substep();
        pt.pre_solve_components(ctx);
        for (std::size_t i = 0; i < hub_tau.size(); ++i) pt.add_hub_terms(i, hub_tau[i], i < hub_d.size() ? hub_d[i] : real(0.0));
        pt.solve(ctx.h);
        pt.post_solve_components(ctx);
    }
};

void reset_neutral_engine_off(Powertrain& pt, const std::vector<real>& hubs) {
    vehicle::VehicleResetOptions opts;
    opts.gear = 0;
    opts.engine_state = EngineState::Off;
    opts.clutch_locked = false;
    pt.reset(opts, hubs);
}

real omega_l(const Powertrain& pt) { return pt.network().omega(pt.diff(0).left_node()); }
real omega_r(const Powertrain& pt) { return pt.network().omega(pt.diff(0).right_node()); }

} // namespace

TEST_CASE("kind Locked (spool): |omega_L - omega_R| stays at rounding level over 2000 substeps, lossy and lossless",
          "[unit][drivetrain][diff]") {
    for (const bool lossless : {true, false}) {
        INFO("lossless " << lossless);
        const vehicle::VehicleDesc d = locked_sedan(lossless);
        Powertrain pt = build(d);
        CHECK(pt.diff(0).recon_mode() == Differential::ReconMode::LeafPair);
        reset_neutral_engine_off(pt, {12.0, 12.5, 0.0, 0.0});
        // Unlike a lockable (C5) diff, kind Locked needs no engage step - it
        // is locked from the very first substep, unconditionally.
        REQUIRE(std::abs(omega_l(pt) - omega_r(pt)) <= 1e-9);
        Driver drv(pt);
        for (int k = 0; k < 2000; ++k) {
            drv.step({80.0, -45.0, 0.0, 0.0}, {3.0, 11.0, 0.0, 0.0});
            const real wl = omega_l(pt), wr = omega_r(pt);
            REQUIRE(std::abs(wl - wr) <= 1e-12 * std::max(real(1.0), std::abs(wl)));
        }
        CHECK(pt.diff_lock_count() == 0); // not a locks_/channel entry - see always_locked_diffs_
    }
}

TEST_CASE("kind Viscous: hub torque difference equals 2*c*|omega_L-omega_R|, diff_loss matches sum h*c*v^2, ledger balances",
          "[unit][drivetrain][diff]") {
    const real ratio = 3.7;
    const real h = 1.0 / 960.0;
    const real c_visc = 8.0;
    NetworkBuilder nb;
    const NodeId c = nb.add_node(0.05);
    const NodeId l = nb.add_node(1.0);
    const NodeId r = nb.add_node(1.3);
    RigidRow diff_row;
    diff_row.terms = {{static_cast<int>(c.index), real(1.0)}, {static_cast<int>(l.index), -ratio / real(2.0)},
                      {static_cast<int>(r.index), -ratio / real(2.0)}};
    nb.add_rigid_row(0, diff_row);
    const CouplingId coupling = nb.add_coupling(l, r);
    const FrictionRowId brake_r = nb.add_friction_row(r);
    ShaftNetwork net = nb.build("viscous-diff-test");
    net.set_topology_state(0);
    net.set_omega(l, 5.0);
    net.set_omega(r, 3.0); // asymmetric: a real relative velocity for the coupling to act on
    net.set_omega(c, ratio * 4.0);

    DifferentialDesc lossy;
    lossy.kind = DifferentialKind::Viscous;
    lossy.ratio = ratio;
    lossy.efficiency = 0.97;
    lossy.viscous_nm_s_rad = c_visc;
    Differential visc(c, l, r, lossy);
    visc.configure_reconstruction(Differential::ReconMode::LeafPair, {}, {});
    visc.set_viscous_coupling(coupling);

    ComponentSubstepCtx ctx;
    ctx.h = h;
    Assembly a;
    PowertrainEnergy e{};
    real expected_diff_loss = 0.0;

    for (int s = 0; s < 200; ++s) {
        a.begin_substep(net.node_count());
        a.begin_substep_rows(net.friction_row_count());
        a.begin_substep_couplings(net.coupling_count());
        a.torque(c, 250.0, 0.0);
        a.torque(l, -120.0, 6.0);
        a.torque(r, -60.0, 14.0);
        a.set_friction_capacity(brake_r, 25.0, 25.0);
        visc.pre_solve(ctx, net, a);
        const real wc = net.omega(c);
        const real wl_old = net.omega(l), wr_old = net.omega(r);
        net.solve(h, a);
        visc.post_solve(ctx, net, a, e);

        // LeafPair reconstruction (at the carrier, unaffected by the L-R
        // coupling, which does not touch node c at all) still matches the
        // carrier's own independent momentum-balance truth.
        const real truth = reconstruct_mesh_torque(c, real(1.0), real(0.0), net, a, wc, h);
        CHECK(visc.lambda_mesh() == Catch::Approx(truth).epsilon(1e-9).margin(1e-9));

        const real v_new = net.coupling_velocity(coupling);
        expected_diff_loss += h * c_visc * v_new * v_new;

        // Hub torque difference: reconstruct_mesh_torque at L and R
        // individually (coef 1, no loss already applied) returns each
        // node's FULL unmodeled reaction - the diff row's own reaction is
        // SYMMETRIC (same sign/magnitude, -ratio/2*lambda, at both L and R)
        // and cancels in the DIFFERENCE, leaving only the coupling's
        // ANTISYMMETRIC reaction (-c*v at L, +c*v at R), so bal_l - bal_r ==
        // -2*c*v exactly.
        const real bal_l = reconstruct_mesh_torque(l, real(1.0), real(0.0), net, a, wl_old, h);
        const real bal_r = reconstruct_mesh_torque(r, real(1.0), real(0.0), net, a, wr_old, h);
        const real expected_diff = -real(2.0) * c_visc * v_new;
        CHECK((bal_l - bal_r) == Catch::Approx(expected_diff).epsilon(1e-6).margin(1e-6));
        CHECK(std::abs(bal_l - bal_r) == Catch::Approx(real(2.0) * c_visc * std::abs(v_new)).epsilon(1e-6).margin(1e-6));
    }
    CHECK(e.diff_loss == Catch::Approx(expected_diff_loss).epsilon(1e-9).margin(1e-9));
    CHECK(e.diff_loss > 0.0);
    CHECK(e.gear_loss > 0.0); // the lossy row's own mesh loss, a separate channel
}

TEST_CASE("LeafPair reconstruction equals the carrier-side truth for a locked (spool) leaf diff", "[unit][drivetrain][diff]") {
    const real ratio = 3.7;
    const real h = 1.0 / 960.0;
    NetworkBuilder nb;
    const NodeId c = nb.add_node(0.05);
    const NodeId l = nb.add_node(1.0);
    const NodeId r = nb.add_node(1.3);
    RigidRow diff_row;
    diff_row.terms = {{static_cast<int>(c.index), real(1.0)}, {static_cast<int>(l.index), -ratio / real(2.0)},
                      {static_cast<int>(r.index), -ratio / real(2.0)}};
    nb.add_rigid_row(0, diff_row);
    RigidRow lock;
    lock.terms = {{static_cast<int>(l.index), real(1.0)}, {static_cast<int>(r.index), real(-1.0)}};
    nb.add_rigid_row(0, lock); // I4 C6: kind Locked's own row is unconditional - present in every (here, the only) state
    const FrictionRowId brake_r = nb.add_friction_row(r);
    ShaftNetwork net = nb.build("locked-leafpair-test");
    net.set_topology_state(0);
    net.set_omega(l, 5.0);
    net.set_omega(r, 5.0);
    net.set_omega(c, ratio * 5.0);

    DifferentialDesc lossy;
    lossy.kind = DifferentialKind::Locked;
    lossy.ratio = ratio;
    lossy.efficiency = 0.97;
    Differential locked(c, l, r, lossy);
    locked.configure_reconstruction(Differential::ReconMode::LeafPair, {}, {});

    ComponentSubstepCtx ctx;
    ctx.h = h;
    Assembly a;
    PowertrainEnergy e{};
    for (int s = 0; s < 40; ++s) {
        a.begin_substep(net.node_count());
        a.begin_substep_rows(net.friction_row_count());
        a.begin_substep_couplings(net.coupling_count());
        a.torque(c, 250.0, 0.0);
        a.torque(l, -120.0, 6.0);
        a.torque(r, -60.0, 14.0);
        a.set_friction_capacity(brake_r, 25.0, 25.0);
        locked.pre_solve(ctx, net, a);
        const real wc = net.omega(c);
        net.solve(h, a);
        locked.post_solve(ctx, net, a, e);
        const real truth = reconstruct_mesh_torque(c, real(1.0), real(0.0), net, a, wc, h);
        CHECK(locked.lambda_mesh() == Catch::Approx(truth).epsilon(1e-9).margin(1e-9));
        CHECK(locked.compute_row_lambda(net, a, h) == Catch::Approx(truth).epsilon(1e-9).margin(1e-9));
        REQUIRE(std::abs(net.omega(l) - net.omega(r)) <= 1e-12 * std::max(real(1.0), std::abs(net.omega(l))));
    }
    CHECK(e.gear_loss > 0.0);
}

TEST_CASE("Nested reconstruction is exact for a lossless locked and a lossless viscous centre diff",
          "[unit][drivetrain][diff]") {
    // Mirrors test_powertrain_multi_diff.cpp's nested-diff pattern: a centre
    // diff (open row) feeding two child diffs' carriers, generalised so the
    // CENTRE's own L/R additionally carry either an unconditional lock row
    // (kind Locked) or an L-R coupling (kind Viscous) - the parent's own
    // lambda_mesh() must still match an independent truth at the carrier,
    // since neither mechanism touches the carrier node at all.
    const real centre_ratio = 1.0;
    const real child_ratio = 4.06;
    const real h = 1.0 / 960.0;
    const real c_visc = 5.0;

    for (const DifferentialKind kind : {DifferentialKind::Locked, DifferentialKind::Viscous}) {
        INFO("kind " << (kind == DifferentialKind::Locked ? "locked" : "viscous"));
        NetworkBuilder nb;
        const NodeId gearbox_out = nb.add_node(0.02);
        const NodeId centre_l = nb.add_node(0.03); // == front carrier
        const NodeId centre_r = nb.add_node(0.03); // == rear carrier
        const NodeId fl = nb.add_node(1.0);
        const NodeId fr = nb.add_node(1.0);
        const NodeId rl = nb.add_node(1.0);
        const NodeId rr = nb.add_node(1.0);

        RigidRow centre_row;
        centre_row.terms = {{static_cast<int>(gearbox_out.index), real(1.0)}, {static_cast<int>(centre_l.index), -centre_ratio / real(2.0)},
                             {static_cast<int>(centre_r.index), -centre_ratio / real(2.0)}};
        nb.add_rigid_row(0, centre_row);
        RigidRow front_row;
        front_row.terms = {{static_cast<int>(centre_l.index), real(1.0)}, {static_cast<int>(fl.index), -child_ratio / real(2.0)},
                            {static_cast<int>(fr.index), -child_ratio / real(2.0)}};
        nb.add_rigid_row(0, front_row);
        RigidRow rear_row;
        rear_row.terms = {{static_cast<int>(centre_r.index), real(1.0)}, {static_cast<int>(rl.index), -child_ratio / real(2.0)},
                           {static_cast<int>(rr.index), -child_ratio / real(2.0)}};
        nb.add_rigid_row(0, rear_row);

        CouplingId centre_coupling{};
        if (kind == DifferentialKind::Locked) {
            RigidRow lock;
            lock.terms = {{static_cast<int>(centre_l.index), real(1.0)}, {static_cast<int>(centre_r.index), real(-1.0)}};
            nb.add_rigid_row(0, lock);
        } else {
            centre_coupling = nb.add_coupling(centre_l, centre_r);
        }
        const FrictionRowId brake_rr = nb.add_friction_row(rr);
        ShaftNetwork net = nb.build("nested-kind-centre-test");
        net.set_topology_state(0);
        net.set_omega(fl, 10.0);
        net.set_omega(fr, 10.0);
        net.set_omega(rl, 10.0);
        net.set_omega(rr, 10.0);
        net.set_omega(centre_l, child_ratio * 10.0);
        net.set_omega(centre_r, child_ratio * 10.0);
        net.set_omega(gearbox_out, centre_ratio * child_ratio * 10.0);

        DifferentialDesc front_desc;
        front_desc.ratio = child_ratio;
        front_desc.efficiency = 1.0; // LOSSLESS: only a lossless kind-!=Open diff may sit as a Nested candidate
        Differential front(centre_l, fl, fr, front_desc);
        front.configure_reconstruction(Differential::ReconMode::Leaf, {}, {});
        DifferentialDesc rear_desc = front_desc;
        Differential rear(centre_r, rl, rr, rear_desc);
        rear.configure_reconstruction(Differential::ReconMode::Leaf, {}, {});

        DifferentialDesc centre_desc;
        centre_desc.kind = kind;
        centre_desc.ratio = centre_ratio;
        centre_desc.efficiency = 1.0; // lossless centre - required for a kind != Open diff to be a Nested candidate
        centre_desc.viscous_nm_s_rad = c_visc;
        Differential centre(gearbox_out, centre_l, centre_r, centre_desc);
        centre.configure_reconstruction(Differential::ReconMode::Nested, {&front}, {&rear});
        if (kind == DifferentialKind::Viscous) centre.set_viscous_coupling(centre_coupling);

        ComponentSubstepCtx ctx;
        ctx.h = h;
        Assembly a;
        PowertrainEnergy e{};
        for (int s = 0; s < 40; ++s) {
            a.begin_substep(net.node_count());
            a.begin_substep_rows(net.friction_row_count());
            a.begin_substep_couplings(net.coupling_count());
            a.torque(gearbox_out, 400.0, 0.0);
            a.torque(fl, -60.0, 4.0);
            a.torque(fr, -70.0, 5.0);
            a.torque(rl, -50.0, 3.0);
            a.torque(rr, -40.0, 6.0);
            a.set_friction_capacity(brake_rr, 15.0, 15.0);
            centre.pre_solve(ctx, net, a);
            front.pre_solve(ctx, net, a);
            rear.pre_solve(ctx, net, a);
            const real w_gearbox_out = net.omega(gearbox_out);
            net.solve(h, a);
            // Order matters: children's lambda must be computed for the
            // parent's post_solve, so children go first (mirrors
            // Powertrain::post_solve_components' own component-array order,
            // where a centre diff appears before its axle children per
            // test_powertrain_multi_diff.cpp's own build order check).
            front.post_solve(ctx, net, a, e);
            rear.post_solve(ctx, net, a, e);
            centre.post_solve(ctx, net, a, e);

            const real truth = reconstruct_mesh_torque(gearbox_out, real(1.0), real(0.0), net, a, w_gearbox_out, h);
            CHECK(centre.lambda_mesh() == Catch::Approx(truth).epsilon(1e-9).margin(1e-9));
        }
        if (kind == DifferentialKind::Viscous) CHECK(e.diff_loss > 0.0);
    }
}

TEST_CASE("build-level mode selection: lossless centre diff of kind locked/viscous is Nested; lossy is rejected; plain open is unchanged",
          "[unit][drivetrain][diff]") {
    using Catch::Matchers::ContainsSubstring;

    SECTION("lossless locked centre builds as Nested") {
        vehicle::VehicleDesc d = make_centre_kind_awd_desc(DifferentialKind::Locked, /*efficiency=*/1.0);
        const Powertrain pt = build(d);
        REQUIRE(pt.diff_count() == 3);
        CHECK(pt.diff(0).recon_mode() == Differential::ReconMode::Nested); // centre
        CHECK(pt.diff(1).recon_mode() == Differential::ReconMode::Leaf);   // front
        CHECK(pt.diff(2).recon_mode() == Differential::ReconMode::Leaf);   // rear
    }
    SECTION("lossless viscous centre builds as Nested") {
        vehicle::VehicleDesc d = make_centre_kind_awd_desc(DifferentialKind::Viscous, /*efficiency=*/1.0, /*viscous_c=*/5.0);
        const Powertrain pt = build(d);
        REQUIRE(pt.diff_count() == 3);
        CHECK(pt.diff(0).recon_mode() == Differential::ReconMode::Nested);
        CHECK(pt.diff(1).recon_mode() == Differential::ReconMode::Leaf);
        CHECK(pt.diff(2).recon_mode() == Differential::ReconMode::Leaf);
    }
    SECTION("lossy locked centre is rejected") {
        vehicle::VehicleDesc d = make_centre_kind_awd_desc(DifferentialKind::Locked, /*efficiency=*/0.97);
        CHECK_THROWS_WITH(build(d),
                           ContainsSubstring("differential 'centre' of kind locked must drive nodes with no other rigid coupling"));
    }
    SECTION("lossy viscous centre is rejected") {
        vehicle::VehicleDesc d = make_centre_kind_awd_desc(DifferentialKind::Viscous, /*efficiency=*/0.97, /*viscous_c=*/5.0);
        CHECK_THROWS_WITH(build(d),
                           ContainsSubstring("differential 'centre' of kind viscous must drive nodes with no other rigid coupling"));
    }
    SECTION("plain open diff (car_sedan) still builds Leaf with the same row list - regression guard") {
        const vehicle::VehicleDesc d = ps_test::load_sedan_desc();
        const Powertrain pt = build(d);
        REQUIRE(pt.diff_count() == 1);
        CHECK(pt.diff(0).recon_mode() == Differential::ReconMode::Leaf);
        CHECK(pt.diff_lock_count() == 0);
        // Exactly one rigid row (the diff's own open Willis row) per state -
        // no unconditional lock row and no coupling added for an Open diff.
        int rows_touching_diff = 0;
        for (const auto& [state, row] : pt.build_rigid_rows()) {
            for (const auto& [idx, coef] : row.terms) {
                if (idx == static_cast<int>(pt.diff(0).left_node().index)) {
                    ++rows_touching_diff;
                    break;
                }
            }
        }
        CHECK(rows_touching_diff == pt.gear_state_count()); // one row per gearbox state, nothing extra
    }
}

TEST_CASE("a vehicle with a viscous diff allocates nothing while stepping", "[unit][vehicle][world][drivetrain][debug_alloc][jobs]") {
    if (!ps::jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc's own operator new/delete overrides are compiled out under ASan/TSan - nothing to check here on this build.");
        return;
    }

    // Same minimal rig as test_powertrain_topology.cpp's own debug_alloc case.
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

    const vehicle::VehicleDesc desc = viscous_sedan(/*c=*/6.0);
    const VehicleId v = world.create_vehicle(desc, chassis_id);
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
    CHECK(world.powertrain_energy(v).diff_loss > 0.0); // the viscous coupling is actually dissipating, not a no-op
}
