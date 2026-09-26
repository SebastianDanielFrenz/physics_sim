// tests/unit/test_powertrain_park_lock.cpp — PHYS-033 P4 I4 C10 (plan
// section 3.4, owner ruling): the park_lock component (tractor.json's
// gearbox.out pawl). White-box (core/src on this target's include path),
// mirroring test_powertrain_transfer_case.cpp's own pattern: raw
// ParkLock/NetworkBuilder/Assembly-level tests for the component's own
// pending/engage/release/saturation state machine (ParkLock owns that logic
// itself, unlike TransferCase's engagement ramp which is a pure function of
// the channel alone - so these tests drive the ParkLock component directly
// rather than a bare friction row), a whole-vehicle row-placement/opt-in
// check (tractor.json vs. the same desc with "park" stripped, and
// car_sedan.json which never declares one at all), and the usual D8/D7
// whole-World checks (debug_alloc-armed zero-alloc stepping, 1-vs-10-worker
// state_hash equality) with the channel toggled mid-run.

#include "drivetrain/components.h"
#include "drivetrain/network.h"
#include "drivetrain/powertrain.h"

#include "vehicle_test_helpers.h"

#include "ps/io/vehicle_io.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/world/world.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

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

vehicle::VehicleDesc load_tractor() { return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/tractor.json"); }
vehicle::VehicleDesc load_sedan() { return io::load_vehicle_json(std::string(PS_SOURCE_DIR) + "/data/vehicles/car_sedan.json"); }
} // namespace

// --- Component-level: pending / engage / hold, driven directly against a
// raw one-node-to-ground friction row (the exact shape Powertrain's own
// Phase G builds for a park_lock component - see powertrain.cpp) ----------
TEST_CASE("ParkLock: a request above max_engage_speed_rad_s stays pending while spinning down, "
          "engages the first substep at or below the threshold, and holds under an applied "
          "torque below capacity",
          "[unit][drivetrain][network]") {
    NetworkBuilder nb;
    const NodeId shaft = nb.add_node(0.6); // stands in for gearbox.out/transfer.in's unioned shaft inertia
    const FrictionRowId row = nb.add_friction_row(shaft); // b defaults to kGround, exactly like Powertrain's own wiring
    ShaftNetwork net = nb.build("test-park-lock-engage");

    ParkLockDesc desc;
    desc.capacity_nm = 500.0;
    desc.static_factor = 1.2;
    desc.max_engage_speed_rad_s = 5.0;
    desc.channel = "park_lock";
    ParkLock lock(shaft, row, desc, /*channel_slot=*/0);

    real cmd = 1.0; // requested engaged throughout this test
    ComponentSubstepCtx ctx;
    ctx.h = 1.0 / 960.0;
    ctx.channels = &cmd;
    ctx.channel_count = 1;

    Assembly assembly;
    PowertrainEnergy ledger{};

    // A monotonically decreasing speed schedule straddling the 5.0 rad/s
    // threshold - net.set_omega stands in for whatever external dynamics
    // (drag, a load) are decelerating the shaft; only ParkLock::pre_solve's
    // own engagement DECISION is under test here, not the network's solve.
    const std::vector<real> speeds = {40.0, 20.0, 10.0, 6.0, 5.5, 5.0, 4.9, 3.0, 0.0};
    for (real s : speeds) {
        net.set_omega(shaft, s);
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        lock.pre_solve(ctx, net, assembly);
        if (s > desc.max_engage_speed_rad_s) {
            CHECK_FALSE(lock.engaged());
            CHECK(assembly.cap_kinetic(row) == 0.0);
            CHECK(assembly.cap_static(row) == 0.0);
        } else {
            CHECK(lock.engaged());
            CHECK(assembly.cap_kinetic(row) == Catch::Approx(desc.capacity_nm));
            CHECK(assembly.cap_static(row) == Catch::Approx(desc.capacity_nm * desc.static_factor));
        }
        net.solve(ctx.h, assembly);
        lock.post_solve(ctx, net, assembly, ledger);
    }
    REQUIRE(lock.engaged()); // engaged by the time speed reached the 5.0 rad/s threshold, held since

    // Holding: once engaged, an applied torque well below capacity_nm is
    // fully arrested - the shaft stays pinned at 0 and the row reports stuck.
    net.set_omega(shaft, 0.0);
    for (int s = 0; s < 50; ++s) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        lock.pre_solve(ctx, net, assembly);
        REQUIRE(lock.engaged()); // "holds regardless of the speed the solve computes" (components.h's own comment)
        assembly.torque(shaft, 300.0, 0.0); // below capacity_nm == 500
        net.solve(ctx.h, assembly);
        lock.post_solve(ctx, net, assembly, ledger);
        REQUIRE(net.row_stuck(row));
        CHECK(net.omega(shaft) == Catch::Approx(0.0).margin(1e-9));
    }
}

TEST_CASE("ParkLock: release on channel < 0.5 is immediate, no ramp", "[unit][drivetrain][network]") {
    NetworkBuilder nb;
    const NodeId shaft = nb.add_node(0.6);
    const FrictionRowId row = nb.add_friction_row(shaft);
    ShaftNetwork net = nb.build("test-park-lock-release");

    ParkLockDesc desc;
    desc.capacity_nm = 500.0;
    desc.static_factor = 1.2;
    desc.max_engage_speed_rad_s = 5.0;
    ParkLock lock(shaft, row, desc, /*channel_slot=*/0);

    real cmd = 1.0;
    ComponentSubstepCtx ctx;
    ctx.h = 1.0 / 960.0;
    ctx.channels = &cmd;
    ctx.channel_count = 1;
    Assembly assembly;
    PowertrainEnergy ledger{};

    // Engage at rest first.
    net.set_omega(shaft, 0.0);
    assembly.begin_substep(net.node_count());
    assembly.begin_substep_rows(net.friction_row_count());
    lock.pre_solve(ctx, net, assembly);
    net.solve(ctx.h, assembly);
    lock.post_solve(ctx, net, assembly, ledger);
    REQUIRE(lock.engaged());
    REQUIRE(assembly.cap_kinetic(row) == Catch::Approx(desc.capacity_nm));

    // A single substep with the channel below 0.5 releases immediately - no
    // partial/ramped capacity in between, unlike TransferCase's engagement.
    cmd = 0.3;
    assembly.begin_substep(net.node_count());
    assembly.begin_substep_rows(net.friction_row_count());
    lock.pre_solve(ctx, net, assembly);
    CHECK_FALSE(lock.engaged());
    CHECK(assembly.cap_kinetic(row) == 0.0);
    CHECK(assembly.cap_static(row) == 0.0);
}

TEST_CASE("ParkLock: an applied torque above capacity slips at exactly the kinetic capacity, "
          "dissipation books into brake_loss and the ledger accounting closes",
          "[unit][drivetrain][network]") {
    NetworkBuilder nb;
    const NodeId shaft = nb.add_node(0.4);
    const FrictionRowId row = nb.add_friction_row(shaft);
    ShaftNetwork net = nb.build("test-park-lock-saturation");

    ParkLockDesc desc;
    desc.capacity_nm = 500.0;
    desc.static_factor = 1.0;
    desc.max_engage_speed_rad_s = 5.0;
    ParkLock lock(shaft, row, desc, /*channel_slot=*/0);

    real cmd = 1.0;
    ComponentSubstepCtx ctx;
    ctx.h = 1.0 / 960.0;
    ctx.channels = &cmd;
    ctx.channel_count = 1;
    Assembly assembly;
    PowertrainEnergy ledger{};

    // Engage at rest.
    net.set_omega(shaft, 0.0);
    assembly.begin_substep(net.node_count());
    assembly.begin_substep_rows(net.friction_row_count());
    lock.pre_solve(ctx, net, assembly);
    net.solve(ctx.h, assembly);
    lock.post_solve(ctx, net, assembly, ledger);
    REQUIRE(lock.engaged());

    // An 800 N*m "hill load" well above the 500 N*m capacity - the pawl
    // cannot hold it: it slips at exactly the kinetic capacity, opposing
    // the resulting motion, and the shaft accelerates away from 0 under
    // the 300 N*m net imbalance.
    const real tau_ext = 800.0;
    real expected_brake_loss = 0.0;
    for (int s = 0; s < 200; ++s) {
        assembly.begin_substep(net.node_count());
        assembly.begin_substep_rows(net.friction_row_count());
        lock.pre_solve(ctx, net, assembly);
        REQUIRE(lock.engaged()); // "holds regardless of the speed" - stays requested and engaged throughout
        assembly.torque(shaft, tau_ext, 0.0);
        net.solve(ctx.h, assembly);
        REQUIRE_FALSE(net.row_stuck(row)); // capacity is exceeded - this is a slip, not a lock

        const real lambda = net.row_lambda(row);
        const real v = net.row_velocity(row);
        CHECK(std::abs(lambda) <= desc.capacity_nm * (1.0 + 1e-9)); // never exceeds kinetic capacity
        CHECK(std::abs(std::abs(lambda) - desc.capacity_nm) <= 1e-6); // saturated exactly at capacity
        CHECK(lambda * v <= 0.0); // dissipative: opposes the slip direction

        expected_brake_loss += -ctx.h * v * lambda; // the spec's own formula, ParkLock::post_solve's literal computation
        lock.post_solve(ctx, net, assembly, ledger);
        CHECK(ledger.brake_loss == Catch::Approx(expected_brake_loss).epsilon(1e-9));
    }
    CHECK(net.omega(shaft) > 0.0); // the pawl did not hold - net omega has run away under the unbalanced 300 N*m
    CHECK(ledger.brake_loss > 0.0);
}

// --- Row placement / opt-in: park_lock's row is inserted right after the
// transfer_case row and before the brake/diff-LSD rows (Phase G before
// Phase H, powertrain.cpp) - NOT appended after everything - and touches
// nothing for a vehicle that never declares one ----------------------------
TEST_CASE("tractor.json's park_lock friction row lands between the transfer_case row and the brake "
          "rows; a desc without one is unaffected",
          "[unit][drivetrain][vehicle]") {
    const vehicle::VehicleDesc mfwd = load_tractor();
    const Powertrain pt_mfwd = build(mfwd);
    REQUIRE(pt_mfwd.has_park_lock());

    // Hand-build the pre-C10 topology: drop the "park" component and its
    // shaft port only - the shared shaft keeping gearbox.out/transfer.in
    // together (PHYS-033 P4 I4 C9's own union) loses its third port.
    vehicle::VehicleDesc baseline = mfwd;
    std::vector<drivetrain::ComponentDesc> kept_components;
    for (const auto& c : baseline.powertrain.components) {
        if (c.id == "park") continue;
        kept_components.push_back(c);
    }
    baseline.powertrain.components = kept_components;

    std::vector<drivetrain::ShaftDesc> kept_shafts;
    for (auto s : baseline.powertrain.shafts) {
        std::vector<drivetrain::PortRef> kept_ports;
        for (const auto& p : s.ports) {
            if (p.component != "park") kept_ports.push_back(p);
        }
        s.ports = kept_ports;
        kept_shafts.push_back(s);
    }
    baseline.powertrain.shafts = kept_shafts;

    const Powertrain pt_baseline = build(baseline);
    REQUIRE_FALSE(pt_baseline.has_park_lock());
    CHECK(pt_baseline.park_lock_engaged() == 0.0);
    CHECK(pt_baseline.park_lock_torque() == 0.0);

    // Row placement: Powertrain builds friction rows in a fixed phase order
    // (powertrain.cpp) - clutch, engine, synchro (gearbox), transfer_case,
    // THEN park_lock, all in "Phase G" - followed by one row per braked
    // corner ("Phase H") and finally any clutch_lsd/torsen diff rows
    // ("Phase H2"). tractor.json has a clutch, engine, gearbox and
    // transfer_case, so with park_lock present its row lands at index 4,
    // immediately BEFORE the 4 brake rows that follow (which shift from
    // indices [4..7] in the park-lock-free baseline to [5..8] once park_lock
    // is added) - NOT appended after every row. The rows before index 4
    // (clutch/engine/synchro/transfer_case) are identical in both networks;
    // the rows from index 4 on in the baseline reappear one slot later in
    // pt_mfwd.
    constexpr int kParkLockRowIndex = 4;
    const int n_baseline = pt_baseline.network().friction_row_count();
    REQUIRE(n_baseline >= kParkLockRowIndex);
    CHECK(pt_mfwd.network().friction_row_count() == n_baseline + 1);
    for (int i = 0; i < kParkLockRowIndex; ++i) {
        const FrictionRowId id{static_cast<std::uint16_t>(i)};
        CHECK(pt_mfwd.network().friction_row_a(id) == pt_baseline.network().friction_row_a(id));
        CHECK(pt_mfwd.network().friction_row_b(id) == pt_baseline.network().friction_row_b(id));
    }
    for (int i = kParkLockRowIndex; i < n_baseline; ++i) {
        const FrictionRowId mfwd_id{static_cast<std::uint16_t>(i + 1)};
        const FrictionRowId base_id{static_cast<std::uint16_t>(i)};
        CHECK(pt_mfwd.network().friction_row_a(mfwd_id) == pt_baseline.network().friction_row_a(base_id));
        CHECK(pt_mfwd.network().friction_row_b(mfwd_id) == pt_baseline.network().friction_row_b(base_id));
    }

    // car_sedan.json never declares a park_lock component at all - the
    // feature is fully opt-in, zero effect on a vehicle without one.
    const Powertrain sedan = build(load_sedan());
    CHECK_FALSE(sedan.has_park_lock());
    CHECK(sedan.park_lock_engaged() == 0.0);
    CHECK(sedan.park_lock_torque() == 0.0);
}

// --- Whole-World D8/D7 checks (tractor.json - the only shipped vehicle with
// a park_lock component) ---------------------------------------------------
TEST_CASE("a tractor with park_lock toggled mid-run allocates nothing while stepping",
          "[unit][vehicle][world][drivetrain][debug_alloc][jobs]") {
    if (!ps::jobs::debug_alloc::hook_available()) {
        WARN("debug_alloc's own operator new/delete overrides are compiled out under ASan/TSan - nothing to check here on this build.");
        return;
    }
    auto rig = ps_test::build_vehicle_rig_from_desc(load_tractor(), 6000.0, Vec3{1.2, 0.35, 0.9}, 1.28, 0.0, 1);
    World& world = *rig.world;
    world.set_control("park_lock", 1.0);
    for (int i = 0; i < 300; ++i) world.step();

    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    for (int i = 0; i < 60; ++i) world.step();
    world.set_control("park_lock", 0.0);
    for (int i = 0; i < 60; ++i) world.step();
    world.set_control("park_lock", 1.0);
    for (int i = 0; i < 60; ++i) world.step();
    ps::jobs::debug_alloc::set_armed(false);

    const std::uint64_t own = ps::jobs::debug_alloc::own_violation_count();
    const std::uint64_t external = ps::jobs::debug_alloc::external_violation_count();
    ps::jobs::debug_alloc::reset();

    WARN("external (Jolt-internal, ungated, PLAN.md P9) violation count=" << external);
    REQUIRE(own == 0);
    CHECK(std::abs(world.powertrain_energy(rig.vehicle).balance_error) < 1e-6);
}

TEST_CASE("tractor state hash with park_lock toggled mid-run is worker-count invariant (1 vs 10 workers)",
          "[unit][vehicle][drivetrain][determinism]") {
    auto run = [](int workers) {
        auto rig = ps_test::build_vehicle_rig_from_desc(load_tractor(), 6000.0, Vec3{1.2, 0.35, 0.9}, 1.28, 0.0, workers);
        World& w = *rig.world;
        w.set_control("park_lock", 1.0);
        ps_test::settle(w, 1.0);
        w.set_control("park_lock", 0.0);
        for (int i = 0; i < 120; ++i) w.step();
        w.set_control("park_lock", 1.0);
        for (int i = 0; i < 120; ++i) w.step();
        return w.state_hash();
    };
    const std::uint64_t h1 = run(1);
    const std::uint64_t h10 = run(10);
    CHECK(h1 == h10);
}
