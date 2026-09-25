// tests/unit/test_powertrain_topology.cpp — PHYS-033 P4 I4 C5 (plan section
// 2.6/2.7): composed topology states s = g + n_g * lock_bits and lockable
// open differentials. White-box (core/src on this target's include path):
// the Powertrain is driven substep by substep directly (begin_substep ->
// pre_solve_components -> hub terms -> solve -> post_solve_components, the
// order VehicleInstance::step_tick uses) with car_sedan.json's own graph and
// its "diff" made lockable IN THE TEST (no data file), gearbox in Neutral and
// the engine Off so the diff/hub cluster sees only the hub torques the test
// applies. The last case runs a whole World for the D8 zero-alloc check.

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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <variant>
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

// car_sedan.json with its (only) diff lockable. `lossless` sets the diff's
// efficiency to 1 so the lagged mesh-loss torque never enters a check that
// wants an exact projection/zero-torque answer.
vehicle::VehicleDesc lockable_sedan(bool lossless = false, real max_mismatch = 2.0) {
    vehicle::VehicleDesc d = ps_test::load_sedan_desc();
    DifferentialDesc& diff = diff_desc(d, "diff");
    diff.lockable = true;
    diff.lock.max_engage_mismatch_rad_s = max_mismatch;
    if (lossless) diff.efficiency = 1.0;
    return d;
}

// Drives one Powertrain substep by substep; the lock command is the single
// declared channel (slot 0).
struct Driver {
    Powertrain& pt;
    real lock_cmd = 0.0;
    ComponentSubstepCtx ctx;

    explicit Driver(Powertrain& p) : pt(p) {
        ctx.h = 1.0 / 960.0;
        ctx.ignition = 0.0;
        ctx.assist_auto_clutch = false;
        ctx.assist_auto_blip = false;
        ctx.assist_auto_shift = false;
        ctx.channels = &lock_cmd;
        ctx.channel_count = 1;
    }
    // hub_tau/hub_d: per-wheel (FL, FR, RL, RR) linear hub torque terms.
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

// Number of topology states the constructor built (every state carries at
// least one diff row, so the highest recorded state + 1).
int built_state_count(const Powertrain& pt) {
    int n = 0;
    for (const auto& [state, row] : pt.build_rigid_rows()) n = std::max(n, state + 1);
    return n;
}

real omega_l(const Powertrain& pt) { return pt.network().omega(pt.diff(0).left_node()); }
real omega_r(const Powertrain& pt) { return pt.network().omega(pt.diff(0).right_node()); }

bool is_lock_row(const RigidRow& row, const Differential& d) {
    return row.terms.size() == 2 && row.terms[0].first == static_cast<int>(d.left_node().index) && row.terms[0].second == real(1.0) &&
           row.terms[1].first == static_cast<int>(d.right_node().index) && row.terms[1].second == real(-1.0);
}

} // namespace

TEST_CASE("composed topology state: mixed-radix round trip and per-state row lists", "[unit][drivetrain][network]") {
    const vehicle::VehicleDesc d = lockable_sedan();
    const Powertrain pt = build(d);
    const int n_g = static_cast<int>(pt.gearbox().desc().reverse_ratios.size() + 1 + pt.gearbox().desc().forward_ratios.size());
    REQUIRE(pt.gear_state_count() == n_g);
    REQUIRE(pt.diff_lock_count() == 1);
    CHECK(pt.diff_lock_diff_index(0) == 0);
    CHECK(pt.diff_lock_channel_slot(0) == 0);
    CHECK(pt.diff(0).recon_mode() == Differential::ReconMode::LeafPair);
    CHECK(built_state_count(pt) == 2 * n_g);

    for (int s = 0; s < 2 * n_g; ++s) {
        CHECK(pt.composed_state(pt.state_gear_index(s), pt.state_lock_bits(s)) == s);
    }
    for (int g = 0; g < n_g; ++g) {
        for (int bits = 0; bits < 2; ++bits) {
            const int s = pt.composed_state(g, bits);
            CHECK(pt.state_gear_index(s) == g);
            CHECK(pt.state_lock_bits(s) == bits);
        }
    }

    // Per state: [diff, gearbox (not in N)], then the lock row LAST and only
    // when the state's lock bit is set; the unlocked half is exactly the
    // gear-only list, so bit 0 states match the pre-C5 layout.
    const auto& rows = pt.build_rigid_rows();
    std::vector<std::vector<const RigidRow*>> by_state(static_cast<std::size_t>(2 * n_g));
    for (const auto& [state, row] : rows) {
        REQUIRE(state < 2 * n_g);
        by_state[static_cast<std::size_t>(state)].push_back(&row);
    }
    for (int s = 0; s < 2 * n_g; ++s) {
        INFO("state " << s);
        const auto& list = by_state[static_cast<std::size_t>(s)];
        const bool locked = pt.state_lock_bits(s) != 0;
        const auto& gear_only = by_state[static_cast<std::size_t>(pt.state_gear_index(s))];
        REQUIRE(list.size() == gear_only.size() + (locked ? 1u : 0u));
        for (std::size_t k = 0; k < gear_only.size(); ++k) CHECK(list[k]->terms == gear_only[k]->terms);
        int lock_rows = 0;
        for (const RigidRow* r : list) lock_rows += is_lock_row(*r, pt.diff(0)) ? 1 : 0;
        CHECK(lock_rows == (locked ? 1 : 0));
        if (locked) CHECK(is_lock_row(*list.back(), pt.diff(0)));
    }
}

TEST_CASE("no lockable diff: s == g and no lock state is built", "[unit][drivetrain][network]") {
    const vehicle::VehicleDesc d = ps_test::load_sedan_desc();
    Powertrain pt = build(d);
    const int n_g = pt.gear_state_count();
    CHECK(pt.diff_lock_count() == 0);
    CHECK(built_state_count(pt) == n_g);
    CHECK(pt.diff(0).recon_mode() == Differential::ReconMode::Leaf);
    for (const int gear : {0, 1, 3, -1}) {
        vehicle::VehicleResetOptions opts;
        opts.gear = gear;
        pt.reset(opts, {5.0, 5.0, 5.0, 5.0});
        Driver drv(pt);
        drv.lock_cmd = 1.0; // no lockable diff: a stray channel value must change nothing
        drv.step();
        CHECK(pt.network().topology_state() == pt.gearbox().state_index_for_position(gear));
        CHECK(pt.network().topology_state() == pt.last_gearbox_result().state_index);
    }
}

TEST_CASE("diff lock: engage refused above the mismatch threshold stays pending, engages once below it",
          "[unit][drivetrain][network]") {
    const vehicle::VehicleDesc d = lockable_sedan(/*lossless=*/true);
    Powertrain pt = build(d);
    reset_neutral_engine_off(pt, {10.0, 5.0, 0.0, 0.0});
    Driver drv(pt);

    drv.step(); // channel 0: nothing requested
    CHECK_FALSE(pt.diff_lock_engaged(0));
    CHECK_FALSE(pt.diff_lock_pending(0));

    drv.lock_cmd = 1.0;
    drv.step();
    CHECK_FALSE(pt.diff_lock_engaged(0));
    CHECK(pt.diff_lock_pending(0));
    CHECK(pt.state_lock_bits(pt.network().topology_state()) == 0);
    CHECK(pt.ledger().engagement_loss == 0.0);

    // Pull L and R together with opposing hub torques; the lock engages on
    // the first substep whose START mismatch is within the threshold.
    bool engaged = false;
    for (int k = 0; k < 20000 && !engaged; ++k) {
        const real mismatch_old = std::abs(omega_l(pt) - omega_r(pt));
        drv.step({-40.0, 40.0, 0.0, 0.0});
        engaged = pt.diff_lock_engaged(0);
        INFO("substep " << k << " mismatch_old " << mismatch_old);
        CHECK(engaged == (mismatch_old <= 2.0));
        CHECK(pt.diff_lock_pending(0) == !engaged);
    }
    REQUIRE(engaged);
    CHECK(pt.state_lock_bits(pt.network().topology_state()) == 1);
    CHECK(pt.state_gear_index(pt.network().topology_state()) == pt.gearbox().state_index_for_position(0));
    CHECK(pt.ledger().engagement_loss > 0.0);
}

TEST_CASE("diff lock: engagement loss is the analytic projection loss, release is lossless", "[unit][drivetrain][network]") {
    const vehicle::VehicleDesc d = lockable_sedan(/*lossless=*/true);
    Powertrain pt = build(d);
    const real wl0 = 10.0, wr0 = 8.5;
    reset_neutral_engine_off(pt, {wl0, wr0, 3.0, 3.0});
    const ShaftNetwork& net = pt.network();
    const Differential& diff = pt.diff(0);
    const real ratio = diff.ratio();
    const real jc = net.node_inertia(diff.carrier_node());
    const real jl = net.node_inertia(diff.left_node());
    const real jr = net.node_inertia(diff.right_node());
    const real wc0 = net.omega(diff.carrier_node());
    REQUIRE(wc0 == ratio * ((wl0 + wr0) / 2.0));

    // Independent momentum-conserving projection onto {L = R = nu, C = ratio*nu}.
    const real nu = (ratio * jc * wc0 + jl * wl0 + jr * wr0) / (ratio * ratio * jc + jl + jr);
    const real expected = 0.5 * (jc * (wc0 - ratio * nu) * (wc0 - ratio * nu) + jl * (wl0 - nu) * (wl0 - nu) +
                                 jr * (wr0 - nu) * (wr0 - nu));

    Driver drv(pt);
    drv.lock_cmd = 1.0;
    const real loss_before = pt.ledger().engagement_loss;
    drv.step(); // zero hub torque: after the projection the solve changes nothing in this cluster
    REQUIRE(pt.diff_lock_engaged(0));
    const real loss = pt.ledger().engagement_loss - loss_before;
    CHECK(loss == Catch::Approx(expected).epsilon(1e-12));
    CHECK(loss > 0.0);
    CHECK(omega_l(pt) == Catch::Approx(nu).epsilon(1e-13));
    CHECK(omega_r(pt) == Catch::Approx(nu).epsilon(1e-13));
    CHECK(net.omega(diff.carrier_node()) == Catch::Approx(ratio * nu).epsilon(1e-13));

    // Release: dropping a row projects the (already consistent) state onto
    // itself - rounding only.
    const real ke = 0.5 * (jc * ratio * ratio + jl + jr) * nu * nu;
    drv.lock_cmd = 0.0;
    const real before_release = pt.ledger().engagement_loss;
    drv.step();
    CHECK_FALSE(pt.diff_lock_engaged(0));
    CHECK_FALSE(pt.diff_lock_pending(0));
    CHECK(pt.ledger().engagement_loss - before_release <= 1e-12 * ke);

    // ...and the outputs are free again.
    for (int k = 0; k < 50; ++k) drv.step({30.0, -30.0, 0.0, 0.0});
    CHECK(omega_l(pt) - omega_r(pt) > 1.0);
}

TEST_CASE("diff lock: while locked |omega_L - omega_R| stays at rounding level under asymmetric hub torques",
          "[unit][drivetrain][network]") {
    for (const bool lossless : {true, false}) {
        INFO("lossless " << lossless);
        const vehicle::VehicleDesc d = lockable_sedan(lossless);
        Powertrain pt = build(d);
        reset_neutral_engine_off(pt, {12.0, 12.5, 0.0, 0.0});
        Driver drv(pt);
        drv.lock_cmd = 1.0;
        drv.step();
        REQUIRE(pt.diff_lock_engaged(0));
        for (int k = 0; k < 2000; ++k) {
            drv.step({80.0, -45.0, 0.0, 0.0}, {3.0, 11.0, 0.0, 0.0});
            const real wl = omega_l(pt), wr = omega_r(pt);
            REQUIRE(std::abs(wl - wr) <= 1e-12 * std::max(real(1.0), std::abs(wl)));
        }
        CHECK(pt.diff_lock_engaged(0));
    }
}

TEST_CASE("LeafPair reconstruction equals the carrier-side truth with the lock row engaged", "[unit][drivetrain][network]") {
    // Carrier C carries ONLY the diff row, so its own momentum balance is an
    // independent truth for lambda; L/R also carry the lock row, whose
    // reaction the left-only Leaf path would wrongly fold into lambda.
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
    nb.add_rigid_row(1, diff_row);
    RigidRow lock;
    lock.terms = {{static_cast<int>(l.index), real(1.0)}, {static_cast<int>(r.index), real(-1.0)}};
    nb.add_rigid_row(1, lock);
    const FrictionRowId brake_r = nb.add_friction_row(r);
    ShaftNetwork net = nb.build("leafpair-test");
    net.set_topology_state(1);
    net.set_omega(l, 5.0);
    net.set_omega(r, 5.0);
    net.set_omega(c, ratio * 5.0);

    DifferentialDesc lossy;
    lossy.ratio = ratio;
    lossy.efficiency = 0.97;
    Differential pair(c, l, r, lossy);
    pair.configure_reconstruction(Differential::ReconMode::LeafPair, {}, {});
    DifferentialDesc plain = lossy;
    plain.efficiency = 1.0; // pre_solve adds nothing to the assembly: a read-only contrast
    Differential leaf(c, l, r, plain);
    leaf.configure_reconstruction(Differential::ReconMode::Leaf, {}, {});

    ComponentSubstepCtx ctx;
    ctx.h = h;
    Assembly a;
    PowertrainEnergy e{};
    real max_leaf_error = 0.0;
    for (int s = 0; s < 40; ++s) {
        a.begin_substep(net.node_count());
        a.begin_substep_rows(net.friction_row_count());
        a.begin_substep_couplings(net.coupling_count());
        a.torque(c, 250.0, 0.0);
        a.torque(l, -120.0, 6.0);
        a.torque(r, -60.0, 14.0);
        a.set_friction_capacity(brake_r, 25.0, 25.0);
        pair.pre_solve(ctx, net, a);
        leaf.pre_solve(ctx, net, a);
        const real wc = net.omega(c);
        net.solve(h, a);
        pair.post_solve(ctx, net, a, e);
        const real truth = reconstruct_mesh_torque(c, real(1.0), real(0.0), net, a, wc, h);
        CHECK(pair.lambda_mesh() == Catch::Approx(truth).epsilon(1e-9).margin(1e-9));
        CHECK(pair.compute_row_lambda(net, a, h) == Catch::Approx(truth).epsilon(1e-9).margin(1e-9));
        max_leaf_error = std::max(max_leaf_error, std::abs(leaf.compute_row_lambda(net, a, h) - truth));
        REQUIRE(std::abs(net.omega(l) - net.omega(r)) <= 1e-12 * std::max(real(1.0), std::abs(net.omega(l))));
    }
    CHECK(e.gear_loss > 0.0);
    CHECK(max_leaf_error > 1.0); // why a lockable leaf needs the pair path
}

TEST_CASE("reset keeps the composed state and satisfies an engaged lock", "[unit][drivetrain][network]") {
    const vehicle::VehicleDesc d = lockable_sedan();
    Powertrain pt = build(d);
    reset_neutral_engine_off(pt, {4.0, 4.0, 0.0, 0.0});
    Driver drv(pt);
    drv.lock_cmd = 1.0;
    drv.step();
    REQUIRE(pt.diff_lock_engaged(0));

    vehicle::VehicleResetOptions opts;
    opts.gear = 2;
    pt.reset(opts, {7.0, 8.0, 3.0, 3.0});
    CHECK(pt.diff_lock_engaged(0));
    CHECK_FALSE(pt.diff_lock_pending(0));
    CHECK(pt.network().topology_state() == pt.composed_state(pt.gearbox().state_index_for_position(2), 1));
    CHECK(omega_l(pt) == 7.5);
    CHECK(omega_r(pt) == 7.5);
    CHECK(pt.network().omega(pt.diff(0).carrier_node()) == pt.diff(0).ratio() * 7.5);
    CHECK(pt.gearbox_in_omega() == pt.gearbox().ratio_for_position(2) * pt.gearbox_out_omega());

    // Unlocked stays unlocked (a reset is not a lock request).
    drv.lock_cmd = 0.0;
    drv.step();
    REQUIRE_FALSE(pt.diff_lock_engaged(0));
    pt.reset(opts, {7.0, 8.0, 3.0, 3.0});
    CHECK_FALSE(pt.diff_lock_engaged(0));
    CHECK(pt.network().topology_state() == pt.gearbox().state_index_for_position(2));
    CHECK(omega_l(pt) == 7.0);
    CHECK(omega_r(pt) == 8.0);
}

TEST_CASE("diff lock build errors: too many composed states, a lockable non-leaf diff", "[unit][drivetrain][network]") {
    using Catch::Matchers::ContainsSubstring;

    SECTION("n_g * 2^L above kMaxStates") {
        // car_sedan's 8 gearbox states x 2^L: L = 4 is exactly 128, L = 5 overflows.
        for (const int lockable : {4, 5}) {
            INFO("lockable diffs " << lockable);
            vehicle::VehicleDesc d = ps_test::load_sedan_desc();
            DifferentialDesc axle = ps_test::strip_sedan_diff(d);
            axle.lockable = true;
            const vehicle::WheelDesc wheel_template = d.wheels.front();
            ShaftDesc carrier;
            carrier.ports = {ps_test::comp_port("gearbox", "out")};
            std::vector<std::string> wheel_names;
            for (const auto& w : d.wheels) wheel_names.push_back(w.name);
            for (int i = 0; static_cast<int>(wheel_names.size()) < 2 * lockable; ++i) {
                vehicle::WheelDesc w = wheel_template;
                w.name = "X" + std::to_string(i);
                d.wheels.push_back(w);
                wheel_names.push_back(w.name);
            }
            for (int i = 0; i < lockable; ++i) {
                const std::string id = "d" + std::to_string(i);
                d.powertrain.components.push_back(ComponentDesc{id, axle});
                carrier.ports.push_back(ps_test::comp_port(id, "carrier"));
                d.powertrain.shafts.push_back(ps_test::shaft(ps_test::comp_port(id, "left"),
                                                             ps_test::wheel_port(wheel_names[static_cast<std::size_t>(2 * i)])));
                d.powertrain.shafts.push_back(ps_test::shaft(ps_test::comp_port(id, "right"),
                                                             ps_test::wheel_port(wheel_names[static_cast<std::size_t>(2 * i + 1)])));
            }
            d.powertrain.shafts.push_back(carrier);
            if (lockable == 4) {
                const Powertrain pt = build(d);
                CHECK(built_state_count(pt) == kMaxStates);
                CHECK(pt.diff_lock_count() == 4);
            } else {
                CHECK_THROWS_WITH(build(d), ContainsSubstring("kMaxStates"));
            }
        }
    }
    SECTION("a lockable centre diff (outputs carry the axle diffs' rows) is rejected") {
        vehicle::VehicleDesc d = ps_test::make_awd_desc();
        diff_desc(d, "centre").lockable = true;
        CHECK_THROWS_WITH(build(d), ContainsSubstring("differential 'centre' of kind lockable open must drive nodes with no other rigid coupling"));
    }
    SECTION("a lockable axle diff under an open centre builds as a LeafPair leaf") {
        vehicle::VehicleDesc d = ps_test::make_awd_desc();
        diff_desc(d, "rear").lockable = true;
        const Powertrain pt = build(d);
        REQUIRE(pt.diff_count() == 3);
        CHECK(pt.diff(0).recon_mode() == Differential::ReconMode::Nested);
        CHECK(pt.diff(1).recon_mode() == Differential::ReconMode::Leaf);
        CHECK(pt.diff(2).recon_mode() == Differential::ReconMode::LeafPair);
        REQUIRE(pt.diff_lock_count() == 1);
        CHECK(pt.diff_lock_diff_index(0) == 2);
    }
}

TEST_CASE("a lockable diff declares its lock channel with default 0", "[unit][drivetrain]") {
    vehicle::VehicleDesc d = lockable_sedan();
    const std::vector<ChannelDecl> decl = Powertrain::declare_control_channels(d.powertrain);
    REQUIRE(decl.size() == 1);
    CHECK(decl[0].name == "diff_lock");
    CHECK(decl[0].default_value == 0.0);
    diff_desc(d, "diff").lock.channel = "rear_lock";
    const std::vector<ChannelDecl> renamed = Powertrain::declare_control_channels(d.powertrain);
    REQUIRE(renamed.size() == 1);
    CHECK(renamed[0].name == "rear_lock");
}

TEST_CASE("a vehicle with a lockable diff allocates nothing while the lock engages and releases",
          "[unit][vehicle][world][drivetrain][debug_alloc][jobs]") {
    if (!ps::jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc's own operator new/delete overrides are compiled out under ASan/TSan - nothing to check here on this build.");
        return;
    }

    // Same rig as test_powertrain_control_channels.cpp's MinimalRig.
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

    const vehicle::VehicleDesc desc = lockable_sedan();
    const VehicleId v = world.create_vehicle(desc, chassis_id);
    world.set_control("gear_request", 1.0);
    world.set_control("throttle", 0.4);
    world.set_control("steer", 0.05);
    for (int i = 0; i < 300; ++i) world.step();

    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    real max_locked_mismatch = 0.0;
    for (int i = 0; i < 120; ++i) {
        world.set_control("diff_lock", (i / 30) % 2 == 0 ? 1.0 : 0.0); // engage, release, engage, release
        world.step();
        // The channel reaches the vehicle one tick late; sample only ticks
        // fully inside an engaged window.
        if ((i / 30) % 2 == 0 && i % 30 >= 2) {
            const real wl = world.wheel_omega(v, 0), wr = world.wheel_omega(v, 1);
            max_locked_mismatch = std::max(max_locked_mismatch, std::abs(wl - wr) / std::max(real(1.0), std::abs(wl)));
        }
    }
    ps::jobs::debug_alloc::set_armed(false);

    const std::uint64_t own = ps::jobs::debug_alloc::own_violation_count();
    const std::uint64_t external = ps::jobs::debug_alloc::external_violation_count();
    ps::jobs::debug_alloc::reset();

    WARN("external (Jolt-internal, ungated, PLAN.md P9) violation count=" << external);
    REQUIRE(own == 0);
    CHECK(max_locked_mismatch <= 1e-12);
    CHECK(world.wheel_omega(v, 0) > 1.0); // actually driving, not a trivially-parked lock
}
